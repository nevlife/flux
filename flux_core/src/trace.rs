//! Trace events: exported, never-inlined functions a collector (by uprobe, preload or
//! `LTTng`) attaches to by name. They do nothing themselves. The names, arguments and constants are a
//! contract, versioned by `flux_trace_contract_*` and declared for C in `flux/trace.h`. A channel
//! is named by its `ChannelShared` (stable until a re-attach), an executor by its control block.

use std::ffi::{c_char, c_void};

pub const CLAIM_OK: i32 = 0;
/// Got in the second pass, after reclaiming dead subscribers.
pub const CLAIM_RECLAIMED: i32 = 1;
pub const CLAIM_BACKPRESSURE: i32 = 2;
pub const CLAIM_NOT_PUBLISHER: i32 = 3;
pub const CLAIM_TOO_LARGE: i32 = 5;
pub const CLAIM_FENCE_FAILED: i32 = 6;
/// The process's GPU is wedged (`gpu::wedged`).
pub const CLAIM_GPU_WEDGED: i32 = 7;

pub const LOAN_COMMITTED: i32 = 0;
pub const LOAN_ABORTED: i32 = 1;
pub const LOAN_TOO_LARGE: i32 = 2;
pub const LOAN_FENCE_FAILED: i32 = 3;
pub const LOAN_NOT_PUBLISHER: i32 = 4;
pub const LOAN_GPU_WEDGED: i32 = 5;

pub const WAKE_TIMEOUT: i32 = 0;
pub const WAKE_SOURCE: i32 = 1;
pub const WAKE_CONTROL: i32 = 2;
/// A word had moved before the wait (`EAGAIN`).
pub const WAKE_ALREADY: i32 = 3;
/// No longer emitted: a wait goes on after a signal. Kept, as the contract numbers it.
pub const WAKE_SIGNAL: i32 = 4;

pub const REFUSED_MAX_BORROW: i32 = 0;
pub const REFUSED_NO_READER: i32 = 1;
pub const REFUSED_CONTENDED: i32 = 2;
pub const REFUSED_BAD_FRAME: i32 = 3;
pub const REFUSED_FENCE: i32 = 4;
pub const REFUSED_GPU_WEDGED: i32 = 5;

pub const RECORD_CACHE_FULL: i32 = 0;
pub const RECORD_WRITE_FAILED: i32 = 1;
pub const RECORD_OVERRUN: i32 = 2;

pub const ROLE_PUBLISHER: i32 = 0;
pub const ROLE_SUBSCRIPTION: i32 = 1;
pub const LANG_CPP: i32 = 0;
pub const LANG_PYTHON: i32 = 1;

/// `slot` of a failed claim, `entry` of a wake no source caused.
pub const NONE: u32 = u32::MAX;
/// `stamp_offset` of a type without a top-level `std_msgs/Header`.
pub const NO_STAMP: i32 = -1;

/// An argument as the register word the asm reads.
pub trait Word {
    fn word(self) -> u64;
}

impl Word for u64 {
    fn word(self) -> u64 {
        self
    }
}

impl Word for u32 {
    fn word(self) -> u64 {
        u64::from(self)
    }
}

impl Word for i32 {
    fn word(self) -> u64 {
        u64::from_ne_bytes(i64::from(self).to_ne_bytes())
    }
}

impl Word for i64 {
    fn word(self) -> u64 {
        u64::from_ne_bytes(self.to_ne_bytes())
    }
}

impl<T> Word for *const T {
    fn word(self) -> u64 {
        self.addr() as u64
    }
}

/// A call site. `--cfg flux_trace_off` removes it with its argument loads; that build exists only
/// to measure what the calls cost (`benches/trace_cost.rs`).
macro_rules! event {
    ($f:ident($($a:expr),* $(,)?)) => {
        #[cfg(not(flux_trace_off))]
        $crate::trace::$f($($a),*);
        #[cfg(flux_trace_off)]
        let _ = || ($($a,)*);
    };
    ($f:ident($($a:expr),* $(,)?) where $bind:pat = $e:expr) => {
        #[cfg(not(flux_trace_off))]
        {
            let $bind = $e;
            $crate::trace::$f($($a),*);
        }
        #[cfg(flux_trace_off)]
        let _ = || {
            let $bind = $e;
            ($($a,)*)
        };
    };
}
pub(crate) use event;

#[cfg(test)]
thread_local! {
    pub(crate) static CALLS: std::cell::RefCell<Vec<(&'static str, Vec<u64>)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

macro_rules! operand_slots {
    ($($arg:ident),*) => { concat!($(" {", stringify!($arg), "}"),*) };
}

/// Every argument is an asm input: a function that reads none lets a fat-LTO or one-codegen-unit
/// build leave the call site's argument registers unset. The text differs per event, or functions
/// of one signature are merged into one address and a uprobe on one fires for all of them.
macro_rules! emit {
    ($event:literal $(, $arg:ident)*) => {
        #[cfg(test)]
        CALLS.with(|c| c.borrow_mut().push(($event, vec![$(Word::word($arg)),*])));
        // Miri runs no asm, and no collector attaches under it.
        #[cfg(not(miri))]
        // SAFETY: an asm comment touches no memory or registers.
        unsafe {
            std::arch::asm!(
                concat!("/* flux_trace_", $event, operand_slots!($($arg),*), " */"),
                $($arg = in(reg) Word::word($arg),)*
                options(nostack, preserves_flags)
            )
        };
    };
}

/// Made or re-attached. `stamp_offset` is where `header.stamp` sits in the payload, `NO_STAMP`
/// without one.
#[unsafe(no_mangle)]
#[inline(never)]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn flux_trace_channel_init(
    channel: *const c_void,
    dev: u64,
    ino: u64,
    segment: *const c_char,
    publisher: i32,
    slot_count: u32,
    slot_size: u32,
    stamp_offset: i32,
) {
    emit!(
        "channel_init",
        channel,
        dev,
        ino,
        segment,
        publisher,
        slot_count,
        slot_size,
        stamp_offset
    );
}

/// `result` is a `CLAIM_*`; `slot` is `NONE` unless it got one. `loan` is 1 on the loan path.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_claim(channel: *const c_void, slot: u32, result: i32, loan: i32) {
    emit!("claim", channel, slot, result, loan);
}

#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_loan(channel: *const c_void, slot: u32) {
    emit!("loan", channel, slot);
}

/// `result` is a `LOAN_*`. A committed loan's `commit` comes first.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_loan_end(channel: *const c_void, slot: u32, result: i32) {
    emit!("loan_end", channel, slot, result);
}

/// `stamp` is the payload's `header.stamp` bytes as a little-endian u64 (sec low, nanosec high),
/// 0 without one.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_commit(
    channel: *const c_void,
    ticket: u64,
    slot: u32,
    nbytes: u64,
    woke: i32,
    stamp: u64,
) {
    emit!("commit", channel, ticket, slot, nbytes, woke, stamp);
}

#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_take(
    channel: *const c_void,
    ticket: u64,
    slot: u32,
    lost: u64,
    stamp: u64,
) {
    emit!("take", channel, ticket, slot, lost, stamp);
}

/// `held` 1: the release fence failed and the borrow stays.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_release(channel: *const c_void, ticket: u64, held: i32) {
    emit!("release", channel, ticket, held);
}

/// `reason` is a `REFUSED_*`.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_refused(channel: *const c_void, reason: i32) {
    emit!("refused", channel, reason);
}

/// Once per process, when a sticky CUDA error (`result`, a `CUresult`) wedges its GPU.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_gpu_wedged(result: i32) {
    emit!("gpu_wedged", result);
}

/// `sources` counts the words waited on besides the control word.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_wait(executor: *const c_void, sources: u32, timeout_ns: i64) {
    emit!("wait", executor, sources, timeout_ns);
}

/// `result` is a `WAKE_*`; `entry` the executor entry whose word woke it, else `NONE`.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_wake(executor: *const c_void, result: i32, entry: u32) {
    emit!("wake", executor, result, entry);
}

/// `endpoint` is the source's endpoint handle, as `endpoint_init` named it, or NULL.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_callback_start(
    executor: *const c_void,
    entry: u32,
    endpoint: *const c_void,
) {
    emit!("callback_start", executor, entry, endpoint);
}

/// `segment` names the channel delivered from ("" for a heap channel), so a tool that attached
/// after `channel_init` can still name the callback.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_callback_end(
    executor: *const c_void,
    entry: u32,
    delivered: i32,
    channel: *const c_void,
    segment: *const c_char,
) {
    emit!("callback_end", executor, entry, delivered, channel, segment);
}

/// From the bindings: an endpoint made on node `node` (its fully qualified name). `role` 0
/// publisher, 1 subscription; `lang` 0 C++, 1 Python.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_endpoint_init(
    endpoint: *const c_void,
    node: *const c_char,
    key: *const c_char,
    signpost: *const c_char,
    fingerprint: u64,
    role: i32,
    lang: i32,
) {
    emit!(
        "endpoint_init",
        endpoint,
        node,
        key,
        signpost,
        fingerprint,
        role,
        lang
    );
}

/// From the Python binding: 0 before taking the GIL back, 1 once held.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_py_gil(phase: i32) {
    emit!("py_gil", phase);
}

/// From the Python binding: the frame `take` gave is a Python object, the callback is next.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_py_ready(channel: *const c_void, ticket: u64) {
    emit!("py_ready", channel, ticket);
}

/// From a recorder: frames not written to the bag. `ticket` 0 is `count` frames not known one by
/// one. `reason` is a `RECORD_*`.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_record_drop(
    channel: *const c_void,
    ticket: u64,
    count: u32,
    reason: i32,
) {
    emit!("record_drop", channel, ticket, count, reason);
}

/// Emits nothing. A tool checks for this name before attaching: the number changes whenever an
/// event's name or arguments change.
#[unsafe(no_mangle)]
#[inline(never)]
pub extern "C" fn flux_trace_contract_2() {
    emit!("contract_2");
}

#[cfg(test)]
pub(crate) fn drain() -> Vec<(&'static str, Vec<u64>)> {
    CALLS.with(|c| std::mem::take(&mut *c.borrow_mut()))
}

#[cfg(test)]
pub(crate) fn drain_names() -> Vec<&'static str> {
    drain().into_iter().map(|(n, _)| n).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::channel::{Channel, Published};
    use crate::error::Result;
    use crate::executor::{Executor, Source};

    struct Heap(Channel);

    impl Source for Heap {
        fn attach(&mut self) -> Result<bool> {
            Ok(true)
        }
        fn deliver_one(&mut self) -> Result<bool> {
            Ok(self.0.take().is_some())
        }
        fn channel(&self) -> Option<&Channel> {
            Some(&self.0)
        }
        fn endpoint(&self) -> *const c_void {
            std::ptr::from_ref(self).cast()
        }
    }

    #[test]
    fn a_dispatched_frame_emits_its_events_in_order() {
        let mut ex = Executor::new().unwrap();
        let mut ch = Channel::new(16, 2).unwrap();
        let init = drain();
        assert_eq!(init.len(), 1);
        let (name, args) = &init[0];
        assert_eq!(*name, "channel_init");
        let id = args[0];
        assert_eq!(args[4..], [0, 2, 16, NO_STAMP.word()]);

        assert_eq!(ch.publish(b"x"), Published::Ok);
        let ev = drain();
        assert_eq!(ev[0], ("claim", vec![id, ev[0].1[1], 0, 0]));
        assert_eq!(ev[1], ("commit", vec![id, 1, ev[0].1[1], 1, 0, 0]));

        ex.add(Heap(ch), 0).unwrap();
        assert_eq!(ex.dispatch().unwrap(), 1);
        let ev = drain();
        let names: Vec<_> = ev.iter().map(|(n, _)| *n).collect();
        assert_eq!(names, ["callback_start", "take", "release", "callback_end"]);
        let executor = ev[0].1[0];
        assert_eq!(ev[0].1[1], 0);
        assert_ne!(ev[0].1[2], 0, "the source's endpoint");
        assert_eq!(ev[1].1[..2], [id, 1]);
        assert_eq!(ev[2].1, [id, 1, 0]);
        assert_eq!(ev[3].1[..4], [executor, 0, 1, id]);

        ex.wait_for_work(0).unwrap();
        let ev = drain();
        assert_eq!(ev[0].0, "wait");
        assert_eq!(ev[0].1, [executor, 1, 0]);
        assert_eq!(
            ev[1],
            ("wake", vec![executor, WAKE_TIMEOUT.word(), NONE.word()])
        );
    }

    #[test]
    fn a_loan_ends_once_whichever_way_it_goes() {
        let mut ch = Channel::new(16, 2).unwrap();
        drain();
        let mut w = ch.loan().unwrap();
        w.data_mut()[0] = 1;
        assert_eq!(w.commit_len(1), Published::Ok);
        assert_eq!(
            drain_names(),
            ["claim", "loan", "commit", "loan_end"],
            "committed"
        );

        drop(ch.loan().unwrap());
        let ev = drain();
        assert_eq!(ev[0].1[2..], [CLAIM_OK.word(), 1]);
        assert_eq!(ev[2].0, "loan_end");
        assert_eq!(ev[2].1[2], LOAN_ABORTED.word());

        assert_eq!(ch.loan().unwrap().commit_len(17), Published::TooLarge);
        let ev = drain();
        assert_eq!(ev.len(), 3);
        assert_eq!(ev[2].1[2], LOAN_TOO_LARGE.word());
    }

    #[test]
    fn a_refused_publish_is_a_claim_without_a_slot() {
        let mut ch = Channel::new(4, 2).unwrap();
        drain();
        assert_eq!(ch.publish(&[0; 5]), Published::TooLarge);
        let ev = drain();
        assert_eq!(ev[0].1[1..], [NONE.word(), CLAIM_TOO_LARGE.word(), 0]);

        let held = (ch.loan().unwrap(), ch.loan().unwrap());
        drain();
        assert_eq!(ch.publish(&[0; 4]), Published::Backpressure);
        let ev = drain();
        assert_eq!(ev[0].1[1..], [NONE.word(), CLAIM_BACKPRESSURE.word(), 0]);
        drop(held);
    }
}
