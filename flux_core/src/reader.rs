//! Subscriber numbers and subscriber mappings. Number `i` is an OFD write lock on byte `1+i` of
//! the segment, held while the process lives, and indexes the reader counter in every slot. One
//! number and one mapping per process and segment, shared by every Channel of that process on it.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use std::sync::{Arc, Mutex, PoisonError, Weak};
use std::time::Duration;

use crate::discovery::{
    SignpostMap, check_publisher, map_subscriber, open_subscriber_fd, segment_id,
};
use crate::error::{Error, Result};
use crate::layout::reader_lock_byte;
use crate::qos::MemoryPolicy;
use crate::segment::{Segment, SegmentId};
use crate::sys::{self, LiveFd};

// A publisher reclaiming a dead number holds its lock for the length of one clear_reader, so a
// full scan may be refused for that long although the number is about to be free.
const FULL_RESCANS: u32 = 100;
const RESCAN_NAP: Duration = Duration::from_micros(100);

pub(crate) struct ReaderLease {
    pub(crate) index: u32,
    fd: Option<LiveFd>,
}

#[cfg(test)]
impl ReaderLease {
    /// A number with no lock behind it, for tests that stand in for several processes.
    pub(crate) fn unlocked(index: u32) -> ReaderLease {
        ReaderLease { index, fd: None }
    }
}

#[derive(Default)]
struct Registry {
    leases: Vec<(SegmentId, Weak<ReaderLease>)>,
    /// Subscriber mappings, with the fork generation they were made in: a forked child maps
    /// again, as it takes a number of its own.
    maps: Vec<(SegmentId, u32, Weak<Segment>)>,
}

static REGISTRY: Mutex<Registry> = Mutex::new(Registry {
    leases: Vec::new(),
    maps: Vec::new(),
});

/// Under the fork lock: a child forked while another thread held the registry would find it held
/// for good.
fn with_registry<R>(f: impl FnOnce(&mut Registry) -> R) -> R {
    sys::fork_safe(|| f(&mut REGISTRY.lock().unwrap_or_else(PoisonError::into_inner)))
}

/// The current segment of `signpost`, mapped once per process, and this process's number on it.
pub(crate) fn subscribe(
    signpost: &SignpostMap,
    fingerprint: u64,
    mem: MemoryPolicy,
) -> Result<(Arc<Segment>, Arc<ReaderLease>, u32)> {
    let (fd, name, epoch) = open_subscriber_fd(signpost)?;
    let id = segment_id(fd.raw());
    let (seg, lease) = with_registry(|reg| {
        reg.maps.retain(|(_, _, w)| w.strong_count() > 0);
        let generation = sys::fork_gen();
        let shared = reg
            .maps
            .iter()
            .filter(|&&(i, g, _)| i == id && g == generation)
            .find_map(|(_, _, w)| w.upgrade());
        let seg = if let Some(seg) = shared {
            // The copy map_validate checked, not the header, which any peer can write.
            if seg.header().fingerprint != fingerprint {
                return Err(Error::Mismatch(format!("fingerprint mismatch '{name}'")));
            }
            check_publisher(fd.raw(), &name)?;
            seg
        } else {
            let seg = Arc::new(map_subscriber(fd.raw(), &name, fingerprint)?);
            reg.maps.push((id, generation, Arc::downgrade(&seg)));
            seg
        };
        let lease = lease_in(reg, &seg, fd)?;
        Ok((seg, lease))
    })?;
    // Outside the registry, which fork() waits on: populating or locking a large segment takes long.
    seg.apply(mem)?;
    Ok((seg, lease, epoch))
}

/// `live` is a description of `seg` that holds no other lock of this process.
pub(crate) fn lease_with_fd(seg: &Segment, live: LiveFd) -> Result<Arc<ReaderLease>> {
    with_registry(|reg| lease_in(reg, seg, live))
}

fn lease_in(reg: &mut Registry, seg: &Segment, live: LiveFd) -> Result<Arc<ReaderLease>> {
    reg.leases.retain(|(_, w)| w.strong_count() > 0);
    // A forked child inherits its parent's leases but not their locks: it takes a number of its
    // own.
    if let Some(l) = reg
        .leases
        .iter()
        .filter(|(id, _)| *id == seg.id())
        .filter_map(|(_, w)| w.upgrade())
        .find(|l| l.fd.as_ref().is_some_and(LiveFd::owned_here))
    {
        return Ok(l);
    }
    let index = free_number(seg, live.raw())?;
    // A dead holder of this number may have left counts behind.
    seg.clear_reader(index);
    seg.raise_readers(index);
    let lease = Arc::new(ReaderLease {
        index,
        fd: Some(live),
    });
    reg.leases.push((seg.id(), Arc::downgrade(&lease)));
    Ok(lease)
}

fn free_number(seg: &Segment, raw: std::os::fd::RawFd) -> Result<u32> {
    for _ in 0..FULL_RESCANS {
        if let Some(i) =
            (0..seg.layout().max_subscribers).find(|&i| sys::lock_write(raw, reader_lock_byte(i)))
        {
            return Ok(i);
        }
        std::thread::sleep(RESCAN_NAP);
    }
    Err(Error::SubscribersFull(seg.name().to_owned()))
}

/// For a publisher reading its own segment, or a heap channel, which no other process sees. A
/// subscriber's segment comes with its lease from `subscribe`.
pub(crate) fn lease_for(seg: &Segment) -> Result<Arc<ReaderLease>> {
    let Some(fd) = seg.publisher_fd() else {
        seg.raise_readers(0);
        return Ok(Arc::new(ReaderLease { index: 0, fd: None }));
    };
    let own = LiveFd::open(|| sys::reopen(fd)).map_err(|e| Error::io("reopen", seg.name(), e))?;
    lease_with_fd(seg, own)
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{lease_with_fd, with_registry};
    use crate::channel::Channel;
    use crate::discovery::{open_publisher_segment, signpost_name};
    use crate::error::Error;
    use crate::layout::{DEFAULT_MAX_SUBSCRIBERS, MAX_SUBSCRIBERS, NO_SCHEMA, reader_lock_byte};
    use crate::qos::MemoryPolicy;
    use crate::sys::{self, LiveFd};

    fn topic(test: &str) -> String {
        signpost_name(&format!("/r{}_{test}", std::process::id()), NO_SCHEMA).unwrap()
    }

    /// The publisher's reclaim holds a dead number's lock while it clears the counts. A newcomer
    /// scanning meanwhile must wait it out, not report the segment full.
    #[test]
    fn a_number_under_reclaim_is_waited_for() {
        let seg = open_publisher_segment(
            &topic("reclaim"),
            64,
            1,
            DEFAULT_MAX_SUBSCRIBERS,
            NO_SCHEMA,
            None,
        )
        .unwrap();
        let pfd = seg.publisher_fd().unwrap();
        let reopen = || LiveFd::open(|| sys::reopen(pfd)).unwrap();
        let live: Vec<LiveFd> = (1..DEFAULT_MAX_SUBSCRIBERS)
            .map(|i| {
                let l = reopen();
                assert!(sys::lock_write(l.raw(), reader_lock_byte(i)));
                l
            })
            .collect();
        assert!(sys::lock_write(pfd, reader_lock_byte(0)));
        std::thread::scope(|s| {
            s.spawn(|| {
                std::thread::sleep(Duration::from_millis(5));
                sys::unlock(pfd, reader_lock_byte(0));
            });
            assert_eq!(lease_with_fd(&seg, reopen()).unwrap().index, 0);
        });
        drop(live);
    }

    /// The publisher picks how many subscribing processes fit: the 17th gets a number only when
    /// the segment was made for more than 16.
    #[test]
    fn a_segment_takes_as_many_subscribers_as_it_was_made_for() {
        for (max, past_default) in [(DEFAULT_MAX_SUBSCRIBERS, false), (64, true)] {
            let seg =
                open_publisher_segment(&topic(&format!("max{max}")), 64, 1, max, NO_SCHEMA, None)
                    .unwrap();
            let pfd = seg.publisher_fd().unwrap();
            let reopen = || LiveFd::open(|| sys::reopen(pfd)).unwrap();
            let live: Vec<LiveFd> = (0..DEFAULT_MAX_SUBSCRIBERS)
                .map(|i| {
                    let l = reopen();
                    assert!(sys::lock_write(l.raw(), reader_lock_byte(i)));
                    l
                })
                .collect();
            match lease_with_fd(&seg, reopen()) {
                Ok(lease) => assert!(past_default && lease.index == DEFAULT_MAX_SUBSCRIBERS),
                Err(e) => assert!(
                    !past_default && matches!(e, Error::SubscribersFull(_)),
                    "{e}"
                ),
            }
            drop(live);
        }
        for bad in [0, MAX_SUBSCRIBERS + 1] {
            let r = open_publisher_segment(&topic("maxbad"), 64, 1, bad, NO_SCHEMA, None);
            assert!(matches!(r, Err(Error::InvalidArgument(_))), "{bad}");
        }
    }

    /// A fork while another thread holds the registry must not hand the child a held lock.
    #[test]
    fn a_child_forked_while_leases_are_held_can_subscribe() {
        let name = topic("fork");
        let _p = Channel::create(
            &name,
            64,
            2,
            DEFAULT_MAX_SUBSCRIBERS,
            NO_SCHEMA,
            MemoryPolicy::default(),
        )
        .unwrap();
        let (held, holding) = std::sync::mpsc::channel();
        let holder = std::thread::spawn(move || {
            with_registry(|_| {
                held.send(()).unwrap();
                std::thread::sleep(Duration::from_millis(200));
            });
        });
        holding.recv().unwrap();
        // SAFETY: the child calls _exit and only opens a channel before it.
        let pid = unsafe { libc::fork() };
        if pid == 0 {
            // SAFETY: plain syscall; a hung child dies of SIGALRM.
            unsafe { libc::alarm(5) };
            let ok = Channel::open(&name, NO_SCHEMA, MemoryPolicy::default()).is_ok();
            // SAFETY: ends the child without running the parent's destructors.
            unsafe { libc::_exit(i32::from(!ok)) };
        }
        let mut status = 0;
        // SAFETY: reaps our own child.
        unsafe { libc::waitpid(pid, &raw mut status, 0) };
        holder.join().unwrap();
        assert!(
            libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0,
            "{status:#x}"
        );
    }
}
