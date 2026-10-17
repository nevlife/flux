//! A debug build's ledger of what the GPU path holds, after the kernel's `DMA_API_DEBUG`: host
//! registrations, exports, imports and the exported descriptor with their sizes, and each GPU
//! channel's borrows with the fences that let them go. Misuse panics where it happens: a release
//! of what is not held, a size that differs from the one made, a borrow returned without a fence
//! that finished. What a channel still holds when it goes, or the process at exit, is reported as
//! a leak. In a release build every function here is empty.
//!
//! A kernel writing through a read-only import is not visible here: the device faults instead,
//! which the sticky error handling (`gpu::wedged`) and the kernel log judge (`scripts/kernel_log.sh`)
//! catch.

use std::collections::BTreeMap;
use std::ffi::c_void;
use std::sync::{Mutex, MutexGuard, Once, PoisonError};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum Held {
    Registration(usize),
    Export(u64),
    Import(u64),
    Fd(i32),
}

#[derive(Default)]
struct Borrow {
    count: u32,
    fenced: u32,
}

struct Ledger {
    held: BTreeMap<Held, usize>,
    borrows: BTreeMap<(usize, u32), Borrow>,
    /// Borrows a failed fence keeps for the process's life, on purpose.
    kept: u64,
    leaks: u64,
}

static LEDGER: Mutex<Ledger> = Mutex::new(Ledger {
    held: BTreeMap::new(),
    borrows: BTreeMap::new(),
    kept: 0,
    leaks: 0,
});

fn ledger() -> Option<MutexGuard<'static, Ledger>> {
    static AT_EXIT: Once = Once::new();
    if !cfg!(debug_assertions) {
        return None;
    }
    AT_EXIT.call_once(|| {
        // SAFETY: registers a function with C linkage that takes nothing.
        unsafe { libc::atexit(at_exit) };
    });
    Some(LEDGER.lock().unwrap_or_else(PoisonError::into_inner))
}

pub(crate) fn made(what: Held, bytes: usize) {
    let Some(mut l) = ledger() else { return };
    if let Some(old) = l.held.insert(what, bytes) {
        panic!("GPU ledger: {what:?} made again while held ({old} bytes)");
    }
}

pub(crate) fn undone(what: Held, bytes: usize) {
    let Some(mut l) = ledger() else { return };
    match l.held.remove(&what) {
        None => panic!("GPU ledger: {what:?} released, but it is not held: a double release"),
        Some(made) if made != bytes => {
            panic!("GPU ledger: {what:?} made with {made} bytes, released with {bytes}")
        }
        Some(_) => {}
    }
}

pub(crate) fn borrowed(channel: *const c_void, slot: u32) {
    let Some(mut l) = ledger() else { return };
    l.borrows.entry((channel.addr(), slot)).or_default().count += 1;
}

/// The fence that lets this borrow go has finished.
pub(crate) fn fenced(channel: *const c_void, slot: u32) {
    let Some(mut l) = ledger() else { return };
    if let Some(b) = l.borrows.get_mut(&(channel.addr(), slot)) {
        b.fenced += 1;
    }
}

pub(crate) fn returned(channel: *const c_void, slot: u32) {
    let Some(mut l) = ledger() else { return };
    let key = (channel.addr(), slot);
    let Some(b) = l.borrows.get_mut(&key) else {
        panic!("GPU ledger: slot {slot} of channel {channel:?} returned, but not borrowed");
    };
    let fenced = b.fenced > 0;
    b.count -= 1;
    b.fenced = b.fenced.saturating_sub(1);
    if b.count == 0 {
        l.borrows.remove(&key);
    }
    drop(l);
    assert!(
        fenced,
        "GPU ledger: slot {slot} of channel {channel:?} returned without a finished fence"
    );
}

/// A failed fence keeps the borrow for good: no longer a leak to report.
pub(crate) fn kept(channel: *const c_void, slot: u32) {
    let Some(mut l) = ledger() else { return };
    let key = (channel.addr(), slot);
    if let Some(b) = l.borrows.get_mut(&key) {
        b.count -= 1;
        if b.count == 0 {
            l.borrows.remove(&key);
        }
    }
    l.kept += 1;
}

pub(crate) fn channel_gone(channel: *const c_void) {
    let Some(mut l) = ledger() else { return };
    let left: Vec<(u32, u32)> = l
        .borrows
        .iter()
        .filter(|((c, _), _)| *c == channel.addr())
        .map(|((_, s), b)| (*s, b.count))
        .collect();
    for (slot, count) in &left {
        eprintln!("GPU ledger: channel {channel:?} went with {count} borrow(s) of slot {slot}");
        l.borrows.remove(&(channel.addr(), *slot));
    }
    l.leaks += left.len() as u64;
}

#[cfg(test)]
pub(crate) fn leaks() -> u64 {
    ledger().map_or(0, |l| l.leaks)
}

extern "C" fn at_exit() {
    let l = LEDGER.lock().unwrap_or_else(PoisonError::into_inner);
    if l.held.is_empty() && l.borrows.is_empty() {
        return;
    }
    eprintln!(
        "GPU ledger at exit: {} held, {} borrowed, {} kept by failed fences",
        l.held.len(),
        l.borrows.len(),
        l.kept
    );
    for (what, bytes) in &l.held {
        eprintln!("  {what:?}: {bytes} bytes");
    }
}

#[cfg(all(test, debug_assertions))]
mod tests {
    use super::{Held, borrowed, fenced, made, returned, undone};

    fn channel(id: usize) -> *const std::ffi::c_void {
        std::ptr::without_provenance(id)
    }

    #[test]
    #[should_panic(expected = "a double release")]
    fn a_double_release_panics() {
        made(Held::Import(1), 8);
        undone(Held::Import(1), 8);
        undone(Held::Import(1), 8);
    }

    #[test]
    #[should_panic(expected = "made with 8 bytes, released with 16")]
    fn a_release_of_another_size_panics() {
        made(Held::Export(2), 8);
        undone(Held::Export(2), 16);
    }

    #[test]
    #[should_panic(expected = "without a finished fence")]
    fn a_borrow_returned_without_a_fence_panics() {
        borrowed(channel(3), 0);
        returned(channel(3), 0);
    }

    #[test]
    #[should_panic(expected = "not borrowed")]
    fn a_borrow_returned_twice_panics() {
        borrowed(channel(4), 0);
        fenced(channel(4), 0);
        returned(channel(4), 0);
        returned(channel(4), 0);
    }
}
