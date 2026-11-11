//! A view's storage copied and both copies released, as a C caller can do with `flux_view`.

use std::sync::Arc;
use std::sync::atomic::Ordering;

use super::{Channel, ChannelShared, FrameView, Published};
use crate::qos::{MemoryPolicy, QoS};
use crate::reader::ReaderLease;
use crate::segment::Segment;

fn pair(slots: u32) -> (Channel, Channel) {
    let seg = Segment::create_heap(8, slots, 0, 0).unwrap();
    let sub = ChannelShared::new(Arc::new(seg.alias()), crate::trace::NO_STAMP);
    let _ = sub.lease.set(Arc::new(ReaderLease::unlocked(1)));
    sub.seg.raise_readers(1);
    let p = ChannelShared::new(Arc::new(seg), crate::trace::NO_STAMP);
    let p = Channel::with_shared(Arc::new(p), MemoryPolicy::default());
    let mut s = Channel::with_shared(Arc::new(sub), MemoryPolicy::default());
    s.set_qos(QoS::new(slots).unwrap()).unwrap();
    (p, s)
}

fn copy(v: &FrameView) -> FrameView {
    FrameView {
        sh: Arc::clone(&v.sh),
        slot: v.slot,
        ticket: v.ticket,
        meta: v.meta,
    }
}

#[test]
fn a_count_found_at_zero_is_refused_and_its_slot_held() {
    let (mut p, mut s) = pair(2);
    assert_eq!(p.publish(b"a"), Published::Ok);
    let v = s.take().unwrap();
    let (slot, twin) = (v.slot, copy(&v));
    drop(v);
    drop(twin);
    for _ in 0..4 {
        assert_eq!(p.publish(b"b"), Published::Ok);
        assert_eq!(s.take().map(|v| v.data().to_vec()), Some(b"b".to_vec()));
    }
    assert_eq!(s.sh.reader(slot).load(Ordering::Relaxed), 1);
    assert_eq!(s.sh.seg.slot(slot).commit_ticket.load(Ordering::Relaxed), 1);
    assert_eq!(s.misreleased(), 1);
}

#[test]
fn a_release_of_a_rewritten_slot_leaves_the_new_borrow_standing() {
    let (mut p, mut s) = pair(1);
    assert_eq!(p.publish(b"a"), Published::Ok);
    let v = s.take().unwrap();
    let twin = copy(&v);
    drop(v);
    assert_eq!(p.publish(b"b"), Published::Ok);
    let held = s.take().unwrap();
    drop(twin);
    assert_eq!(p.publish(b"c"), Published::Backpressure);
    assert_eq!(held.data(), b"b");
    assert_eq!(s.misreleased(), 1);
}
