//! A publisher whose slot size was left to flux moves to a segment of larger slots when a frame
//! does not fit (`flux_docs/design/variable_slot.md`). That frame is refused; `flux-filler` makes
//! the segment, and the publisher's next publish or loan advertises it and moves over.
//! Subscribers re-attach as on a restart and count it in `resizes`.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use crate::discovery::{SegmentPayload, advertise_resized, resized_segment};
use crate::error::Result;
use crate::gpu::{FenceStats, Stream};
use crate::qos::MemoryPolicy;
use crate::sizing::next_slot_size;
use crate::sys::{self, PiMutex};

use super::shared::ChannelShared;
use super::{Channel, ClaimOrder, Growth, filler, next_attach_generation};

/// What a growable publisher keeps across its resizes, and what `flux-filler` makes its next
/// channel with.
pub(super) struct Grow {
    growth: Growth,
    /// The slot size of the segment the publisher is on: an older one, kept by a loan still out,
    /// makes none.
    slot: AtomicU32,
    /// The largest slot; 0 once a resize failed, which stops further ones.
    max: AtomicU32,
    payload: SegmentPayload,
    mem: MemoryPolicy,
    stream: Stream,
    fence: Arc<FenceStats>,
    /// Bytes of the largest frame that did not fit since the last resize; 0 for none.
    want: AtomicU64,
    /// The channel `flux-filler` made for `want` and its segment's epoch, not yet advertised.
    made: PiMutex<Option<(Arc<ChannelShared>, u32)>>,
}

impl Grow {
    pub(super) fn new(
        growth: Growth,
        slot: u32,
        max: u32,
        payload: SegmentPayload,
        mem: MemoryPolicy,
        stream: &Stream,
        fence: &Arc<FenceStats>,
    ) -> Self {
        Grow {
            growth,
            slot: AtomicU32::new(slot),
            max: AtomicU32::new(max),
            payload,
            mem,
            stream: stream.clone(),
            fence: Arc::clone(fence),
            want: AtomicU64::new(0),
            made: PiMutex::new(None),
        }
    }

    /// Stops resizing, for a resize that failed: told once, and not tried again.
    fn give_up(&self, name: &str, e: &crate::error::Error) {
        self.max.store(0, Ordering::Relaxed);
        self.want.store(0, Ordering::Relaxed);
        eprintln!("flux: '{name}' keeps its slots, larger ones failed: {e}");
    }
}

impl ChannelShared {
    /// A frame of `bytes` did not fit: asks `flux-filler` for larger slots, if it may have them.
    pub(super) fn want_larger(&self, bytes: u64) {
        let Some(g) = self.grow.get() else {
            return;
        };
        let slot = u64::from(self.seg.layout().slot_size);
        if bytes <= slot || bytes > u64::from(g.max.load(Ordering::Relaxed)) {
            return;
        }
        if g.want.fetch_max(bytes, Ordering::Relaxed) < bytes {
            filler::wake();
        }
    }

    /// For `flux-filler`: makes the channel the largest refused frame asked for, once. The
    /// publisher does not wait on this: `made` is held only to look and to store.
    pub(super) fn make_resized(&self) {
        let Some(g) = self.grow.get() else {
            return;
        };
        let want = g.want.load(Ordering::Relaxed);
        let current = self.seg.layout().slot_size;
        if want == 0 || current != g.slot.load(Ordering::Relaxed) || g.made.lock().is_some() {
            return;
        }
        let Some(size) = next_slot_size(g.growth, current, want, g.max.load(Ordering::Relaxed))
        else {
            g.want.store(0, Ordering::Relaxed);
            return;
        };
        match self.resized(g, size) {
            Ok(m) => *g.made.lock() = Some(m),
            Err(e) => g.give_up(self.seg.name(), &e),
        }
    }

    /// This publisher's channel again, with slots of `size`. Slots given memory keep it; a
    /// publisher still waiting for its first subscriber gives the start slots theirs later.
    fn resized(&self, g: &Arc<Grow>, size: u32) -> Result<(Arc<ChannelShared>, u32)> {
        let start = self.fill_start.load(Ordering::Relaxed);
        let ready = if start == 0 {
            self.seg.slots_ready()
        } else {
            0
        };
        let (seg, epoch) = resized_segment(&self.seg, size, g.payload, ready)?;
        seg.apply(g.mem)?;
        let mut sh = ChannelShared::new(Arc::new(seg), self.stamp_offset);
        sh.attach_device(&g.stream, &g.fence, true)?;
        sh.filled = true;
        sh.fill_start.store(start, Ordering::Relaxed);
        sh.slot_only.store(self.slot_only(), Ordering::Relaxed);
        let sh = Arc::new(sh).traced();
        if !filler::watch(&sh, g.mem) {
            return Err(crate::error::Error::Unsupported(
                "flux-filler stopped".into(),
            ));
        }
        Ok((sh, epoch))
    }
}

impl Channel {
    /// Moves a publisher to the channel `flux-filler` made, if there is one: its segment is
    /// advertised and the old one's parked subscribers woken, so they re-attach. Loans and views
    /// still out keep the old one until they end.
    pub(super) fn take_resized(&mut self) {
        let Some(g) = self.sh.grow.get() else {
            return;
        };
        if g.want.load(Ordering::Relaxed) == 0 {
            return;
        }
        let Some((sh, epoch)) = g.made.lock().take() else {
            return;
        };
        let _ = sh.grow.set(Arc::clone(g));
        g.slot.store(sh.seg.layout().slot_size, Ordering::Relaxed);
        g.want.store(0, Ordering::Relaxed);
        // The new segment holds the signpost too; the old one lets go, or a loan still out on it
        // would keep the publisher lock after this publisher is gone.
        if let Some(map) = self.sh.seg.signpost.lock().take() {
            advertise_resized(&map, epoch);
        }
        self.sh.wake_all();
        self.sh.orphan_late();
        sys::retire(std::mem::replace(&mut self.sh, sh));
        self.order = ClaimOrder::default();
        self.attach_gen = next_attach_generation();
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::disallowed_types, reason = "a test")]

    use super::super::{Alloc, Channel, Growth, Published, PublisherConfig, soon};
    use crate::discovery::signpost_name;
    use crate::gpu::Stream;
    use crate::layout::NO_SCHEMA;
    use crate::qos::{MemoryPolicy, QoS};
    use crate::sizing::{SLOT_ALLOC_MAX_SLOT_SIZE, SLOT_ALLOC_SLOT_SIZE};

    const MIB: usize = 1 << 20;

    fn name(what: &str) -> String {
        signpost_name(&format!("/resize{}_{what}", std::process::id()), NO_SCHEMA).unwrap()
    }

    fn publisher(name: &str, slot_size: u32, growth: Growth) -> Channel {
        let config = PublisherConfig {
            alloc: Alloc::Slot,
            slot_size,
            slot_count: 4,
            growth,
            ..PublisherConfig::default()
        };
        let none = Stream::default();
        Channel::create_config(
            name,
            &config,
            NO_SCHEMA,
            MemoryPolicy::default(),
            none,
            -1,
            None,
        )
        .unwrap()
    }

    fn subscriber(name: &str) -> Channel {
        let mut s = Channel::open(name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
        s.set_qos(QoS::new(4).unwrap()).unwrap();
        s
    }

    /// Publishes `frame` until the publisher has moved to slots it fits in.
    fn publish_soon(p: &mut Channel, frame: &[u8]) -> bool {
        soon(|| p.publish(frame) == Published::Ok)
    }

    /// The frame past the slot is refused; the publisher then moves to the next power of two and
    /// its subscriber follows, counting a resize, not a restart.
    #[test]
    fn a_frame_past_a_slot_left_to_flux_moves_the_publisher_to_larger_slots() {
        let n = name("grow");
        let mut p = publisher(&n, 0, Growth::PowerOfTwo);
        let mut s = subscriber(&n);
        assert_eq!(p.slot_size() as usize, SLOT_ALLOC_SLOT_SIZE as usize);
        assert_eq!(p.publish(&[1]), Published::Ok);
        assert_eq!(s.take().map(|v| v.data()[0]), Some(1));
        let gen0 = s.attach_generation();
        // Not taken before the move: lost.
        assert_eq!(p.publish(&[2]), Published::Ok);
        assert_eq!(p.publish(&[3]), Published::Ok);

        let big = vec![7u8; 5 * MIB];
        assert_eq!(p.publish(&big), Published::TooLarge);
        assert!(publish_soon(&mut p, &big));
        assert_eq!(p.slot_size() as usize, 8 * MIB);

        let got = {
            let mut got = None;
            assert!(soon(|| {
                got = s.take().map(|v| (v.data().len(), v.data()[5 * MIB - 1]));
                got.is_some()
            }));
            got
        };
        assert_eq!(got, Some((5 * MIB, 7)));
        assert_ne!(s.attach_generation(), gen0);
        assert_eq!(s.resizes(), 1);
        assert_eq!(s.lost(), 2);
        assert_eq!(s.slot_size() as usize, 8 * MIB);
    }

    #[test]
    fn best_fit_moves_to_the_frame_in_pages() {
        let n = name("fit");
        let mut p = publisher(&n, 0, Growth::BestFit);
        let frame = vec![1u8; 5 * MIB + 1];
        assert_eq!(p.publish(&frame), Published::TooLarge);
        assert!(publish_soon(&mut p, &frame));
        assert_eq!(p.slot_size() as usize, 5 * MIB + 4096);
    }

    /// A given size, `Static`, and a frame past `max_slot_size` keep the slot.
    #[test]
    fn a_given_size_static_and_a_frame_past_the_most_keep_the_slot() {
        let frame = vec![1u8; 5 * MIB];
        let mut given = publisher(&name("given"), 4 << 20, Growth::PowerOfTwo);
        let mut fixed = publisher(&name("static"), 0, Growth::Static);
        let mut most = publisher(&name("most"), 0, Growth::PowerOfTwo);
        let past = vec![1u8; SLOT_ALLOC_MAX_SLOT_SIZE as usize + 1];
        assert!(given.sh.grow.get().is_none() && fixed.sh.grow.get().is_none());
        assert_eq!(given.publish(&frame), Published::TooLarge);
        assert_eq!(fixed.publish(&frame), Published::TooLarge);
        assert_eq!(most.publish(&past), Published::TooLarge);
        std::thread::sleep(std::time::Duration::from_millis(50));
        assert_eq!(most.publish(&[1]), Published::Ok);
        assert_eq!(most.slot_size() as usize, 4 * MIB);
    }

    /// A loan's writer that ran past the slot commits the length it wanted, which asks for the
    /// larger slot as a refused publish does. A loan still out on the old slots commits there.
    #[test]
    fn a_commit_past_the_slot_asks_for_larger_slots_and_a_loan_out_commits_on_the_old() {
        let n = name("loan");
        let mut p = publisher(&n, 0, Growth::PowerOfTwo);
        let mut sub = subscriber(&n);
        let out = p.loan().unwrap();
        let over = p.loan().unwrap();
        let cap = over.capacity();
        assert_eq!(over.commit_len(cap + 1), Published::TooLarge);
        let mut larger = None;
        assert!(soon(|| {
            larger = p.loan().filter(|l| l.capacity() > cap);
            larger.is_some()
        }));
        assert_eq!(larger.unwrap().capacity(), 8 * MIB);
        assert_eq!(out.commit_len(1), Published::Ok);
        // The subscriber may take that frame before it moves over, or never.
        let _ = sub.take();
    }

    /// A publisher that restarts is no resize. A loan still out on the slots it left does not
    /// keep its publisher lock.
    #[test]
    fn a_restarted_publisher_is_not_counted_as_a_resize() {
        let n = name("restart");
        let mut p = publisher(&n, 0, Growth::PowerOfTwo);
        let mut s = subscriber(&n);
        let out = p.loan().unwrap();
        let big = vec![1u8; 5 * MIB];
        assert_eq!(p.publish(&big), Published::TooLarge);
        assert!(publish_soon(&mut p, &big));
        assert!(soon(|| s.take().is_some()));
        drop(p);
        let mut p = publisher(&n, 0, Growth::PowerOfTwo);
        drop(out);
        assert_eq!(p.publish(&[2]), Published::Ok);
        assert!(soon(|| s.take().is_some_and(|v| v.data() == [2])));
        assert_eq!(s.resizes(), 1);
    }

    /// Segments left under the signpost `name`: its own object and its `.e<epoch>` ones.
    fn segments(name: &str) -> usize {
        let prefix = format!("{}.e", &name[1..]);
        std::fs::read_dir("/dev/shm")
            .unwrap()
            .filter(|e| {
                e.as_ref()
                    .is_ok_and(|e| e.file_name().to_string_lossy().starts_with(&prefix))
            })
            .count()
    }

    /// A publisher gone before it took the segment made for it leaves neither that segment nor
    /// its lock; the segments it left are gone once nothing holds them.
    #[test]
    fn a_publisher_gone_before_its_resize_leaves_no_segment_or_lock() {
        let n = name("gone");
        let mut p = publisher(&n, 0, Growth::PowerOfTwo);
        assert_eq!(p.publish(&vec![1u8; 5 * MIB]), Published::TooLarge);
        assert!(soon(|| p.sh.grow.get().unwrap().made.lock().is_some()));
        assert_eq!(segments(&n), 2);
        drop(p);
        assert_eq!(segments(&n), 0);
        let mut p = publisher(&n, 0, Growth::PowerOfTwo);
        let big = vec![1u8; 5 * MIB];
        assert_eq!(p.publish(&big), Published::TooLarge);
        assert!(publish_soon(&mut p, &big));
        assert!(
            soon(|| segments(&n) == 1),
            "the segment it left is still there"
        );
        drop(p);
        assert_eq!(segments(&n), 0);
    }

    /// A subscriber parked on the old segment is woken there and follows. A park ends within
    /// 100 ms anyway, so the wake is checked on the old segment's word.
    #[test]
    fn a_parked_subscriber_is_woken_to_follow_the_publisher_to_larger_slots() {
        let n = name("parked");
        let mut p = publisher(&n, 0, Growth::PowerOfTwo);
        let mut s = subscriber(&n);
        let waiter = std::thread::spawn(move || {
            let v = s.take_blocking(2_000_000_000);
            v.map(|v| v.data().len())
        });
        std::thread::sleep(std::time::Duration::from_millis(20));
        let old = std::sync::Arc::clone(&p.sh);
        let seq = old.wake_seq();
        let big = vec![3u8; 5 * MIB];
        assert_eq!(p.publish(&big), Published::TooLarge);
        assert!(publish_soon(&mut p, &big));
        assert_ne!(
            old.wake_seq(),
            seq,
            "the old segment's subscribers were not woken"
        );
        assert_eq!(waiter.join().unwrap(), Some(5 * MIB));
    }
}
