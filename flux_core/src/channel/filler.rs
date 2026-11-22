//! The thread that gives a fixed-slot publisher's slots their memory, so its loans never do: the
//! start slots once a subscriber has attached, then one more each time a claim asks for it. It
//! also makes the segment a growable publisher moves to (`resize`).

use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, OnceLock, Weak};
use std::time::Duration;

use super::shared::ChannelShared;
use crate::qos::MemoryPolicy;
use crate::sys;

/// How often a channel without its start slots looks for a subscriber: attaching sends the
/// publisher nothing.
const POLL: Duration = Duration::from_millis(10);

enum Ask {
    Watch(Weak<ChannelShared>, MemoryPolicy),
    Slot,
}

/// The filler's queue and the fork generation it runs in.
static FILLER: OnceLock<Option<(Sender<Ask>, u32)>> = OnceLock::new();

/// A housekeeping thread, at `SCHED_OTHER` whatever its creator ran at, as `flux-reaper`.
fn sender() -> Option<&'static Sender<Ask>> {
    let started = FILLER.get_or_init(|| {
        let (tx, rx) = mpsc::channel();
        let builder = std::thread::Builder::new().name("flux-filler".into());
        let fill = move || {
            sys::normal_priority();
            run(&rx);
        };
        sys::spawn_helper(builder, fill).ok()?;
        Some((tx, sys::fork_gen()))
    });
    match started {
        Some((tx, born)) if *born == sys::fork_gen() => Some(tx),
        _ => None,
    }
}

/// Whether this process has a filler: none in a forked child or where the thread was refused,
/// and the publisher then gives its slots their memory itself.
pub(super) fn available() -> bool {
    sender().is_some()
}

/// Hands `sh` to the filler; false when there is none.
pub(super) fn watch(sh: &Arc<ChannelShared>, mem: MemoryPolicy) -> bool {
    sender().is_some_and(|tx| tx.send(Ask::Watch(Arc::downgrade(sh), mem)).is_ok())
}

/// Asks for one more slot of `sh`; again only once the filler took the last ask.
pub(super) fn ask(sh: &ChannelShared) {
    if !sh.want_slot.swap(true, Ordering::Relaxed)
        && let Some(tx) = sender()
    {
        let _ = tx.send(Ask::Slot);
    }
}

/// Has the filler look over its channels, for a resize `want_larger` asked for.
pub(super) fn wake() {
    if let Some(tx) = sender() {
        let _ = tx.send(Ask::Slot);
    }
}

fn run(rx: &Receiver<Ask>) {
    let mut watched: Vec<(Weak<ChannelShared>, MemoryPolicy)> = Vec::new();
    let mut waiting = false;
    loop {
        let got = if waiting {
            rx.recv_timeout(POLL)
        } else {
            rx.recv().map_err(|_| RecvTimeoutError::Disconnected)
        };
        match got {
            Ok(Ask::Watch(w, mem)) => watched.push((w, mem)),
            Ok(Ask::Slot) | Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return,
        }
        waiting = false;
        watched.retain(|(w, mem)| {
            let Some(sh) = w.upgrade() else {
                return false;
            };
            waiting |= fill(&sh, *mem);
            true
        });
    }
}

/// One pass over `sh`; true while it still waits for its first subscriber.
fn fill(sh: &ChannelShared, mem: MemoryPolicy) -> bool {
    sh.make_resized();
    let seg = &sh.seg;
    let start = sh.fill_start.load(Ordering::Relaxed);
    if start != 0 {
        // order: readers_high
        if seg.readers_in_use(Ordering::Acquire) == 0 {
            return true;
        }
        while seg.slots_ready() < start && add(sh, mem) {}
        // Done, or out of room as a publisher's own claim would be: either way not retried.
        sh.fill_start.store(0, Ordering::Relaxed);
        // A claim before these slots asked for one of them.
        sh.want_slot.store(false, Ordering::Relaxed);
        return false;
    }
    if sh.want_slot.swap(false, Ordering::Relaxed) {
        add(sh, mem);
    }
    false
}

fn add(sh: &ChannelShared, mem: MemoryPolicy) -> bool {
    sh.seg.add_slot(mem).is_some()
}

/// Polls `done` for up to 2 s: the filler works off the caller's thread.
#[cfg(test)]
pub(crate) fn soon(mut done: impl FnMut() -> bool) -> bool {
    let until = std::time::Instant::now() + Duration::from_secs(2);
    while std::time::Instant::now() < until {
        if done() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    done()
}

/// A loan of `p`, once the filler gave it a slot.
#[cfg(test)]
pub(crate) fn loan_soon(p: &mut super::Channel) -> super::Loan {
    let mut w = None;
    assert!(soon(|| {
        w = p.loan();
        w.is_some()
    }));
    w.unwrap()
}
