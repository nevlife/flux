//! The real orderings under Miri (`cargo miri test`). Subscribers stand for processes, each with
//! its own `ChannelShared` and subscriber number. A broken claim, commit or release ordering is a
//! data race on the payload or the meta; a broken wake Dekker leaves a waiter asleep forever, which
//! Miri reports as a deadlock.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use super::{Channel, ChannelShared, Published};
use crate::executor::Control;
use crate::qos::{MemoryPolicy, QoS};
use crate::reader::ReaderLease;
use crate::segment::Segment;

const SLOT: usize = 16;
const SLOTS: u32 = 3;
const FRAMES: u8 = if cfg!(miri) { 12 } else { 200 };

fn channel(sh: &Arc<ChannelShared>) -> Channel {
    let mut ch = Channel::with_shared(Arc::clone(sh), MemoryPolicy::default());
    let q = QoS::new(SLOTS).and_then(|q| q.max_borrow(2)).unwrap();
    ch.set_qos(q).unwrap();
    ch
}

fn subscriber(seg: &Segment, number: u32) -> Arc<ChannelShared> {
    let sh = ChannelShared::new(Arc::new(seg.alias()), crate::trace::NO_STAMP);
    let _ = sh.lease.set(Arc::new(ReaderLease::unlocked(number)));
    Arc::new(sh)
}

#[test]
fn a_held_view_is_never_rewritten() {
    let publisher_side = Arc::new(ChannelShared::new(
        Arc::new(Segment::create_heap(u32::try_from(SLOT).unwrap(), SLOTS, 0, 0).unwrap()),
        crate::trace::NO_STAMP,
    ));
    let subs = [1, 2].map(|n| subscriber(&publisher_side.seg, n));
    let done = AtomicBool::new(false);
    std::thread::scope(|s| {
        let mut publisher = channel(&publisher_side);
        let done = &done;
        s.spawn(move || {
            for id in 1..=FRAMES {
                let r = publisher.publish(&[id; SLOT]);
                assert!(matches!(r, Published::Ok | Published::Backpressure));
            }
            done.store(true, Ordering::Release);
        });
        for (sh, number) in subs.iter().zip([1, 2]) {
            let mut sub = channel(sh);
            s.spawn(move || {
                // While the publisher claims, as a subscriber joining a running channel does.
                sh.seg.raise_readers(number);
                let mut held = VecDeque::new();
                while !done.load(Ordering::Acquire) || sub.ready() {
                    if let Some(v) = sub.take() {
                        let first = v.data()[0];
                        assert!(v.data().iter().all(|&b| b == first), "a torn frame");
                        held.push_back((v, first));
                    }
                    if (held.len() == 2 || done.load(Ordering::Acquire))
                        && let Some((v, first)) = held.pop_front()
                    {
                        assert!(v.data().iter().all(|&b| b == first), "rewritten while held");
                    }
                    std::thread::yield_now();
                }
            });
        }
    });
}

/// One slot, so every publish claims the slot every take borrows: the Dekker pair meets on
/// each frame, and a claim that misses a borrow writes the payload under it. The depth is
/// past the ticket index, so a take that falls behind scans the slot and can find a ticket
/// newer than the `latest` it read: only the ticket's own Release orders that frame.
#[test]
fn a_claim_and_a_borrow_of_one_slot_exclude_each_other() {
    let publisher_side = Arc::new(ChannelShared::new(
        Arc::new(Segment::create_heap(u32::try_from(SLOT).unwrap(), 1, 0, 0).unwrap()),
        crate::trace::NO_STAMP,
    ));
    let sh = subscriber(&publisher_side.seg, 1);
    let done = AtomicBool::new(false);
    let one = |sh: &Arc<ChannelShared>| {
        let mut ch = Channel::with_shared(Arc::clone(sh), MemoryPolicy::default());
        ch.set_qos(QoS::new(64).unwrap()).unwrap();
        ch
    };
    let mut sub = one(&sh);
    sh.seg.raise_readers(1);
    std::thread::scope(|s| {
        let mut publisher = one(&publisher_side);
        let done = &done;
        s.spawn(move || {
            for id in 1..=FRAMES {
                let r = publisher.publish(&[id; SLOT]);
                assert!(matches!(r, Published::Ok | Published::Backpressure));
            }
            done.store(true, Ordering::Release);
        });
        while !done.load(Ordering::Acquire) {
            if let Some(v) = sub.take() {
                let first = v.data()[0];
                assert!(v.data().iter().all(|&b| b == first), "a torn frame");
            }
        }
    });
}

fn heap() -> Arc<ChannelShared> {
    Arc::new(ChannelShared::new(
        Arc::new(Segment::create_heap(u32::try_from(SLOT).unwrap(), SLOTS, 0, 0).unwrap()),
        crate::trace::NO_STAMP,
    ))
}

/// The wake gate: the publisher bumps the word and then reads the waiter counts, the waiter
/// counts itself and then reads the word. One waiter counts under its subscriber number, the
/// other, in the publisher's process, under the header. The waits have no timeout.
#[test]
fn a_waiting_subscriber_sees_the_last_frame() {
    fn wait_for_last(sh: &Arc<ChannelShared>, mut sub: Channel) {
        let mut last = 0;
        while last < FRAMES {
            let seq = sh.wake_seq();
            match sub.take() {
                Some(v) => last = v.data()[0],
                None => {
                    sh.wait(seq, -1);
                }
            }
        }
    }
    let publisher_side = heap();
    let sh = subscriber(&publisher_side.seg, 1);
    sh.seg.raise_readers(1);
    let (numbered, numberless) = (channel(&sh), channel(&publisher_side));
    std::thread::scope(|s| {
        let mut publisher = channel(&publisher_side);
        s.spawn(move || {
            for id in 1..=FRAMES {
                // A stale counter can make every slot look held for a moment.
                while publisher.publish(&[id; SLOT]) == Published::Backpressure {
                    std::thread::yield_now();
                }
            }
        });
        s.spawn(|| wait_for_last(&publisher_side, numberless));
        wait_for_last(&sh, numbered);
    });
}

/// A view released on another thread while the executor arms the wake at `max_borrow`.
#[test]
fn a_release_wakes_an_executor_at_max_borrow() {
    let sh = heap();
    let mut publisher = channel(&sh);
    let mut sub = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
    sub.set_qos(QoS::new(SLOTS).and_then(|q| q.max_borrow(1)).unwrap())
        .unwrap();
    let ctl = Control::new();
    for id in 1..=16 {
        assert_eq!(publisher.publish(&[id; SLOT]), Published::Ok);
        let v = sub.take().unwrap();
        std::thread::scope(|s| {
            s.spawn(move || drop(v));
            while sub.wake_on_release(&ctl) {
                ctl.wait(-1);
            }
        });
    }
}

/// The commit writes the ticket index before it moves `latest`, so a take that read `latest`
/// finds every frame through the index. Nothing is overwritten, so nothing may be lost.
#[test]
fn a_take_finds_every_frame_through_the_index() {
    let publisher_side = heap();
    let sh = subscriber(&publisher_side.seg, 1);
    let done = AtomicBool::new(false);
    // Reading starts where the subscription joined, so it joins first.
    let mut sub = channel(&sh);
    sh.seg.raise_readers(1);
    std::thread::scope(|s| {
        let mut publisher = channel(&publisher_side);
        let done = &done;
        s.spawn(move || {
            for id in 1..=u8::try_from(SLOTS).unwrap() {
                assert_eq!(publisher.publish(&[id; SLOT]), Published::Ok);
            }
            done.store(true, Ordering::Release);
        });
        let mut next = 1;
        while !done.load(Ordering::Acquire) || sub.ready() {
            if let Some(v) = sub.take() {
                assert_eq!(v.data()[0], next, "a frame was skipped");
                next += 1;
            }
            std::thread::yield_now();
        }
        assert_eq!(sub.lost(), 0);
    });
}
