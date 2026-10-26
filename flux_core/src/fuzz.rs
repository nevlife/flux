//! Fuzz targets, run by cargo-fuzz (`fuzz/`) and over a fixed corpus by `cargo test`. Another
//! process writes every byte read here, so the oracle is: no panic, no access outside the mapping.
#![allow(clippy::missing_panics_doc)]

use std::fs::File;
use std::io::Write;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::sync::atomic::Ordering;

use crate::channel::{Channel, FrameView};
use crate::discovery::{SignpostView, map_validate, segment_name};
use crate::layout::{
    ControlHeader, INIT_READY, LAYOUT_VERSION, MAGIC, NO_SCHEMA, SIGNPOST_BYTES, SegmentLayout,
    SignpostPage, layout_config_ok,
};
use crate::manifest::{parse_signpost, read_entry, read_key};
use crate::qos::{MemoryPolicy, QoS};
use crate::segment::{Segment, SegmentId};
use crate::sys;

const SLOT_SIZE: u32 = 256;
const SLOT_COUNT: u32 = 4;

fn touch(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, b| acc ^ b)
}

/// Whatever a corrupt publisher leaves in the headers, a take stays inside its slot. Layout
/// fields are not fuzzed: attach checks them once and every index derives from the cached layout.
pub fn frame(data: &[u8]) {
    let Some((&plan, _)) = data.split_first() else {
        return;
    };
    let Ok(seg) = Segment::create_heap(SLOT_SIZE, SLOT_COUNT, NO_SCHEMA) else {
        return;
    };
    let (base, len) = seg.raw();
    // SAFETY: the heap segment is len bytes and nothing references it yet.
    unsafe {
        base.as_ptr()
            .copy_from_nonoverlapping(data.as_ptr(), data.len().min(len));
    };
    // SAFETY: the mapping starts with a ControlHeader and nothing else references it.
    let ctrl = unsafe { base.cast::<ControlHeader>().as_mut() };
    ctrl.magic = MAGIC;
    ctrl.version = LAYOUT_VERSION;
    ctrl.slot_size = SLOT_SIZE;
    ctrl.slot_count = SLOT_COUNT;
    ctrl.init_state.store(INIT_READY, Ordering::Release);

    let mut ch = Channel::from_segment(seg, MemoryPolicy::default(), crate::trace::NO_STAMP);
    if let Ok(q) = QoS::new(1 + u32::from(plan % 8)) {
        let _ = ch.set_qos(q);
    }
    let mut held = Vec::new();
    for step in 0..8 {
        let _ = ch.ready();
        let Some(v) = ch.take() else { continue };
        assert!(v.data().len() <= SLOT_SIZE as usize, "a view past its slot");
        touch(v.data());
        let _ = (v.meta().shape(), v.meta().dtype(), v.ticket());
        if plan >> (step % 8) & 1 != 0 {
            held.push(v);
        }
    }
}

/// Whatever bytes the segment file holds, attach refuses it or every derived offset is mapped.
/// The first byte picks the file: odd sizes it to what the header after it declares, so a short
/// input gets past the size check (a segment is at least 8 KiB); even takes the rest as it is.
/// True if attach accepted it.
pub fn attach(data: &[u8]) -> bool {
    let Some((&mode, bytes)) = data.split_first() else {
        return false;
    };
    // SAFETY: a valid C string and no other arguments.
    let raw = unsafe { libc::memfd_create(c"flux-fuzz".as_ptr(), libc::MFD_CLOEXEC) };
    if raw < 0 {
        return false;
    }
    // SAFETY: memfd_create returned a fresh descriptor this process owns.
    let mut file = File::from(unsafe { OwnedFd::from_raw_fd(raw) });
    if file.write_all(bytes).is_err() {
        return false;
    }
    if mode & 1 != 0
        && let Some(n) = declared_bytes(bytes)
        && file.set_len(n as u64).is_err()
    {
        return false;
    }
    // Taken from the bytes, so a header that is otherwise sound gets past the fingerprint check.
    let fingerprint = bytes
        .get(32..40)
        .and_then(|b| b.try_into().ok())
        .map_or(0, u64::from_ne_bytes);
    let Ok((base, len, header)) = map_validate(file.as_raw_fd(), "fuzz", fingerprint) else {
        return false;
    };
    let layout = header.layout();
    assert_eq!(layout.total_bytes(), len);
    let seg = Segment::adopt_shm(base, len, header, String::new(), SegmentId::default());
    for w in seg.waiters() {
        let _ = w.load(Ordering::Relaxed);
    }
    for t in 0..layout.index_len() as u64 {
        let _ = seg.index_entry(t).load(Ordering::Relaxed);
    }
    for s in 0..layout.slot_count {
        let _ = seg.slot(s).commit_ticket.load(Ordering::Relaxed);
        for r in seg.readers(s) {
            let _ = r.load(Ordering::Relaxed);
        }
        // SAFETY: payload(s) starts slot_size mapped bytes, which is what attach just checked.
        let payload =
            unsafe { std::slice::from_raw_parts(seg.payload(s), layout.slot_size as usize) };
        touch(payload);
    }
    true
}

/// The size a header's layout fields give, if they are in range and it is at most 16 MiB.
fn declared_bytes(header: &[u8]) -> Option<usize> {
    let field = |at: usize| -> Option<u32> {
        Some(u32::from_ne_bytes(header.get(at..at + 4)?.try_into().ok()?))
    };
    let layout = SegmentLayout {
        slot_size: field(12)?,
        slot_count: field(16)?,
        max_subscribers: field(20)?,
    };
    let ok = layout_config_ok(layout.slot_size, layout.slot_count, layout.max_subscribers);
    (ok && layout.total_bytes() <= 1 << 24).then(|| layout.total_bytes())
}

/// Whatever a signpost page holds, enumeration reads it without panicking.
pub fn roster(data: &[u8]) {
    // SAFETY: every SignpostPage field is an atomic or padding, valid for any bytes.
    let mut page: SignpostPage = unsafe { std::mem::zeroed() };
    let n = data.len().min(SIGNPOST_BYTES);
    // SAFETY: the page is SIGNPOST_BYTES long, n is at most that, and no reference to it exists.
    unsafe {
        (&raw mut page)
            .cast::<u8>()
            .copy_from_nonoverlapping(data.as_ptr(), n);
    };
    let _ = read_key(&page.key);
    for e in &page.roster {
        let _ = read_entry(e);
    }
    if let Ok(s) = std::str::from_utf8(data) {
        let _ = parse_signpost(s);
    }
}

/// Each subscriber number's borrows on the segment `signpost` advertises, summed over every slot,
/// for the numbers in use. None while no ready segment is advertised.
pub fn borrows(signpost: &str, fingerprint: u64) -> Option<Vec<u64>> {
    let epoch = SignpostView::new(signpost).epoch().filter(|&e| e != 0)?;
    let name = segment_name(signpost, epoch);
    let fd = sys::shm_open(&name, libc::O_RDWR).ok()?;
    let (base, len, layout) = map_validate(fd.as_raw_fd(), &name, fingerprint).ok()?;
    let seg = Segment::adopt_shm(base, len, layout, name, SegmentId::default());
    let numbers = seg.readers_in_use(Ordering::SeqCst);
    Some(
        (0..numbers)
            .map(|i| {
                (0..layout.slot_count)
                    .map(|s| u64::from(seg.readers(s)[i].load(Ordering::SeqCst)))
                    .sum()
            })
            .collect(),
    )
}

/// How many frames the view's segment committed after it.
pub fn lag(view: &FrameView) -> u64 {
    view.lag()
}

#[cfg(test)]
mod corpus {
    use super::{attach, frame, roster};
    use crate::layout::{ControlHeader, SegmentLayout, StorageKind};

    // Deterministic, so a failure names one seed.
    fn next(s: &mut u64) -> u64 {
        *s ^= *s << 13;
        *s ^= *s >> 7;
        *s ^= *s << 17;
        *s
    }

    fn below(s: &mut u64, n: usize) -> usize {
        usize::try_from(next(s) % n as u64).unwrap()
    }

    fn flips(seed: &mut u64, buf: &mut [u8], span: usize, target: fn(&[u8])) {
        for _ in 0..8 {
            let at = below(seed, span.min(buf.len()));
            buf[at] ^= 1 << below(seed, 8);
            target(buf);
        }
    }

    /// Truncation, all-ones lengths, random blobs, and bit flips on each blob.
    fn run(target: fn(&[u8]), max_len: usize, rounds: u32) {
        for n in (0..=max_len).step_by(max_len / 64) {
            target(&vec![0; n]);
            target(&vec![0xFF; n]);
        }
        let mut seed = 0x9E37_79B9_7F4A_7C15;
        for _ in 0..rounds {
            let mut buf: Vec<u8> = (0..=below(&mut seed, max_len))
                .map(|_| next(&mut seed).to_le_bytes()[0])
                .collect();
            target(&buf);
            let span = buf.len();
            flips(&mut seed, &mut buf, span, target);
        }
    }

    #[test]
    fn frame_corpus() {
        run(frame, 1100, 4096);
    }

    #[test]
    fn roster_corpus() {
        run(roster, crate::layout::SIGNPOST_BYTES + 64, 512);
    }

    fn attach_any(data: &[u8]) {
        attach(data);
    }

    /// A mode byte, then a sound header for `layout`; `full` writes the whole segment, filled.
    fn segment(seed: &mut u64, layout: SegmentLayout, full: Option<u8>) -> Vec<u8> {
        let header = ControlHeader::new(layout, next(seed), StorageKind::HostInline);
        let len = full.map_or(size_of::<ControlHeader>(), |_| layout.total_bytes());
        let mut buf = vec![full.unwrap_or(0); 1 + len];
        buf[0] = u8::from(full.is_none());
        // SAFETY: buf holds a ControlHeader past the mode byte; any position may take it unaligned.
        unsafe {
            buf[1..]
                .as_mut_ptr()
                .cast::<ControlHeader>()
                .write_unaligned(header);
        };
        buf[1..5].copy_from_slice(&crate::layout::INIT_READY.to_ne_bytes());
        buf
    }

    /// Random blobs never pass the size check, so start from sound headers and flip their bits.
    #[test]
    fn attach_corpus() {
        run(attach_any, 600, 256);
        let mut seed = 0x2545_F491_4F6C_DD1D;
        for round in 0..512 {
            let layout = SegmentLayout {
                slot_size: u32::try_from(1 + below(&mut seed, 5000)).unwrap(),
                slot_count: u32::try_from(1 + below(&mut seed, 4)).unwrap(),
                max_subscribers: u32::try_from(1 + below(&mut seed, 32)).unwrap(),
            };
            let short = segment(&mut seed, layout, None);
            assert!(attach(&short), "a sound header, sized by the mode byte");
            // Every counter all ones, as a corrupt peer leaves them.
            let mut buf = segment(
                &mut seed,
                layout,
                Some(if round % 2 == 0 { 0xFF } else { 0 }),
            );
            assert!(attach(&buf), "a sound whole segment");
            flips(
                &mut seed,
                &mut buf,
                1 + size_of::<ControlHeader>(),
                attach_any,
            );
        }
    }
}
