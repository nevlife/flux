use std::sync::Arc;
use std::sync::atomic::Ordering;

use super::{Channel, ChannelShared, Published};
use crate::layout::{DType, MAX_DIMS};
use crate::qos::{MemoryPolicy, QoS};
use crate::segment::Segment;

fn pair() -> (Arc<ChannelShared>, Channel, Channel) {
    pair_with_aux(0)
}

fn pair_with_aux(aux_size: u32) -> (Arc<ChannelShared>, Channel, Channel) {
    let sh = Arc::new(ChannelShared::new(
        Arc::new(Segment::create_heap(64, 2, aux_size, 0).unwrap()),
        crate::trace::NO_STAMP,
    ));
    let p = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
    let s = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
    (sh, p, s)
}

fn scribble(sh: &ChannelShared, slot: u32) {
    // SAFETY: single-threaded test; stands in for another process writing the header.
    let m = unsafe { &mut *sh.seg.slot(slot).meta.get() };
    m.ndim = 8;
    m.shape = [1 << 20; MAX_DIMS];
    m.nbytes = u64::MAX;
}

/// A device-backed segment maps no payload, so a peer that flips its storage kind afterwards
/// would have a view read past the mapping.
#[test]
fn a_storage_kind_rewritten_after_attach_is_not_believed() {
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};

    use crate::layout::{FrameMeta, SegmentLayout, StorageKind};
    let layout = SegmentLayout {
        slot_size: 64,
        slot_count: 2,
        max_subscribers: 1,
        aux_size: 0,
    };
    // SAFETY: a valid C string and no other arguments.
    let raw = unsafe { libc::memfd_create(c"flux-kind".as_ptr(), libc::MFD_CLOEXEC) };
    // SAFETY: a fresh descriptor this process owns.
    let file = std::fs::File::from(unsafe { OwnedFd::from_raw_fd(raw) });
    file.set_len(layout.payload_offset() as u64).unwrap();
    let mine = crate::sys::map_shared(file.as_raw_fd(), layout.payload_offset()).unwrap();
    // SAFETY: a fresh mapping of payload_offset bytes, as a device-backed header needs.
    unsafe { crate::segment::init_header(mine, layout, 7, Some((0, [1; 16]))) };
    let (base, len, header) = crate::discovery::map_validate(file.as_raw_fd(), "kind", 7).unwrap();
    let id = crate::segment::SegmentId::default();
    let seg = Segment::adopt_shm(base, len, header, String::new(), id);
    let mut ch = Channel::from_segment(seg, MemoryPolicy::default(), crate::trace::NO_STAMP);
    let seg = &ch.sh.seg;
    let meta = FrameMeta {
        ndim: 1,
        itemsize: 1,
        nbytes: 64,
        shape: [64, 0, 0, 0, 0, 0, 0, 0],
        ..FrameMeta::default()
    };
    // SAFETY: nothing else uses the slot; stands in for the publisher's commit.
    unsafe { seg.slot(0).meta.get().write(meta) };
    seg.slot(0).commit_ticket.store(1, Ordering::Release);
    seg.ctrl().latest.store(1, Ordering::Release);
    let kind = std::mem::offset_of!(crate::layout::ControlHeader, storage_kind);
    let at = mine.as_ptr().wrapping_add(kind);
    // SAFETY: inside the peer's mapping of the header; stands in for its write.
    unsafe { at.write(StorageKind::HostInline as u8) };
    let v = ch.take().unwrap();
    assert!(v.data().is_empty());
    // SAFETY: mapped above with this length; nothing refers to it.
    unsafe { crate::sys::munmap(mine, layout.payload_offset()) };
}

#[test]
fn a_view_keeps_the_header_take_checked() {
    let (sh, mut p, mut s) = pair();
    assert_eq!(
        p.publish_typed(&[0; 24], DType::F32, &[2, 3]),
        Published::Ok
    );
    let v = s.take().unwrap();
    scribble(&sh, v.slot);
    assert_eq!(v.meta().shape(), &[2, 3]);
    assert_eq!(v.data().len(), 24);
}

#[test]
fn a_header_that_disagrees_with_itself_is_refused() {
    let (sh, mut p, mut s) = pair();
    assert_eq!(p.publish(&[0; 8]), Published::Ok);
    for slot in 0..2 {
        // SAFETY: as scribble.
        let m = unsafe { &mut *sh.seg.slot(slot).meta.get() };
        m.shape[0] = 9;
    }
    assert!(s.take().is_none());
    assert_eq!(s.refused().bad_frame, 1);
}

#[test]
fn a_bad_frame_is_passed_over_once() {
    let (sh, mut p, mut s) = pair();
    s.set_qos(QoS::new(2).unwrap()).unwrap();
    assert_eq!(p.publish(&[0; 8]), Published::Ok);
    assert_eq!(p.publish(&[1; 8]), Published::Ok);
    let first = (0..2)
        .find(|&slot| sh.seg.slot(slot).commit_ticket.load(Ordering::Relaxed) == 1)
        .unwrap();
    // SAFETY: as scribble.
    unsafe { &mut *sh.seg.slot(first).meta.get() }.dtype = 0xEE;
    assert_eq!(s.take().unwrap().data(), &[1; 8]);
    assert!(s.take().is_none());
    assert_eq!(s.refused().bad_frame, 1);
    assert_eq!(s.lost(), 0);
}

#[test]
fn a_qos_set_mid_stream_neither_repeats_nor_hides_frames() {
    let (_sh, mut p, mut s) = pair();
    s.set_qos(QoS::new(2).unwrap()).unwrap();
    assert_eq!(p.publish(&[0; 8]), Published::Ok);
    assert_eq!(s.take().unwrap().data(), &[0; 8]);
    s.set_qos(QoS::new(1).unwrap().transient_local(1)).unwrap();
    assert_eq!(p.publish(&[1; 8]), Published::Ok);
    assert_eq!(p.publish(&[2; 8]), Published::Ok);
    assert_eq!(s.take().unwrap().data(), &[2; 8]);
    assert_eq!(s.lost(), 1);
    assert!(s.take().is_none());
}

/// A skipped frame spends a retry as a lost race does, so a run of bad frames as long as
/// the budget ends the take as contended.
#[test]
fn a_take_out_of_retries_counts_contended() {
    let sh = Arc::new(ChannelShared::new(
        Arc::new(Segment::create_heap(8, 64, 0, 0).unwrap()),
        crate::trace::NO_STAMP,
    ));
    let mut p = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
    let mut s = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
    s.set_qos(QoS::new(64).unwrap()).unwrap();
    for i in 0..64 {
        assert_eq!(p.publish(&[i; 8]), Published::Ok);
    }
    for slot in 0..64 {
        // SAFETY: as scribble.
        unsafe { &mut *sh.seg.slot(slot).meta.get() }.dtype = 0xEE;
    }
    assert!(s.take().is_none());
    assert_eq!((s.refused().bad_frame, s.refused().contended), (64, 1));
}

#[test]
fn a_frame_of_unknown_dtype_is_refused() {
    let (sh, mut p, mut s) = pair();
    assert_eq!(p.publish(&[0; 8]), Published::Ok);
    for slot in 0..2 {
        // SAFETY: as scribble.
        unsafe { &mut *sh.seg.slot(slot).meta.get() }.dtype = 0xEE;
    }
    assert!(s.take().is_none());
    assert_eq!(s.refused().bad_frame, 1);
}

#[test]
fn a_loans_aux_bytes_arrive_with_its_frame() {
    let (_sh, mut p, mut s) = pair_with_aux(100);
    assert_eq!(p.aux_size(), 100);
    let mut w = p.loan().unwrap();
    assert_eq!(w.aux_mut().len(), 100);
    w.aux_mut()[..5].copy_from_slice(b"hdr01");
    w.set_aux_len(5);
    w.data_mut()[..3].copy_from_slice(&[9; 3]);
    assert_eq!(w.commit_len(3), Published::Ok);
    let v = s.take().unwrap();
    assert_eq!(
        (v.data(), v.aux(), v.meta().aux_len),
        (&[9u8; 3][..], &b"hdr01"[..], 5)
    );
    drop(v);
    // A copying publish and a fresh loan carry none.
    assert_eq!(p.publish(&[1; 4]), Published::Ok);
    assert!(s.take().unwrap().aux().is_empty());
    let w = p.loan().unwrap();
    assert_eq!(w.commit_len(1), Published::Ok);
    assert!(s.take().unwrap().aux().is_empty());
}

#[test]
fn an_aux_length_past_the_region_is_refused_at_commit() {
    let (_sh, mut p, mut s) = pair_with_aux(16);
    for n in [17, usize::from(u16::MAX) + 1, usize::MAX] {
        let mut w = p.loan().unwrap();
        w.set_aux_len(n);
        assert_eq!(w.commit_len(1), Published::TooLarge, "{n}");
    }
    let mut w = p.loan().unwrap();
    w.set_aux_len(16);
    assert_eq!(w.commit_len(1), Published::Ok);
    assert_eq!(s.take().unwrap().aux().len(), 16);
    let (_sh, mut p, _s) = pair_with_aux(0);
    let mut w = p.loan().unwrap();
    assert!(w.aux_mut().is_empty());
    w.set_aux_len(1);
    assert_eq!(w.commit(), Published::TooLarge);
}

#[test]
fn an_aux_length_rewritten_past_the_region_is_refused() {
    let (sh, mut p, mut s) = pair_with_aux(16);
    assert_eq!(p.publish(&[0; 8]), Published::Ok);
    for slot in 0..2 {
        // SAFETY: as scribble.
        unsafe { &mut *sh.seg.slot(slot).meta.get() }.aux_len = 17;
    }
    assert!(s.take().is_none());
    assert_eq!(s.refused().bad_frame, 1);
}

#[test]
fn a_loan_commits_on_another_thread() {
    let (_sh, mut p, mut s) = pair();
    let mut w = p.loan().unwrap();
    w.data_mut()[..8].copy_from_slice(&[3; 8]);
    let published = std::thread::spawn(move || w.commit_len(8)).join().unwrap();
    assert_eq!(published, Published::Ok);
    assert_eq!(s.take().unwrap().data(), &[3; 8]);
}

#[test]
fn commit_and_take_carry_the_frames_stamp() {
    let sh = Arc::new(ChannelShared::new(
        Arc::new(Segment::create_heap(64, 2, 0, 0).unwrap()),
        4,
    ));
    let mut p = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
    let mut s = Channel::with_shared(sh, MemoryPolicy::default());
    let stamp = 0x1122_3344_5566_7788u64;
    let mut frame = [0u8; 12];
    frame[4..].copy_from_slice(&stamp.to_le_bytes());
    crate::trace::drain();
    assert_eq!(p.publish(&frame), Published::Ok);
    drop(s.take().unwrap());
    assert_eq!(p.publish(&frame[..11]), Published::Ok);
    drop(s.take().unwrap());
    let stamps: Vec<_> = crate::trace::drain()
        .into_iter()
        .filter(|(n, _)| ["commit", "take"].contains(n))
        .map(|(n, a)| (n, *a.last().unwrap()))
        .collect();
    assert_eq!(
        stamps,
        [
            ("commit", stamp),
            ("take", stamp),
            ("commit", 0),
            ("take", 0)
        ]
    );
}

#[test]
fn a_committed_loan_can_keep_its_memory_past_the_channel() {
    let (sh, mut p, s) = pair();
    let mut w = p.loan().unwrap();
    w.data_mut()[0] = 7;
    let at = w.data_mut().as_ptr();
    let keep = w.mapped();
    assert_eq!(w.commit_len(1), Published::Ok);
    drop((sh, p, s));
    // SAFETY: keep holds the memory; Miri reports a use after free otherwise.
    assert_eq!(unsafe { *at }, 7);
    drop(keep);
}
