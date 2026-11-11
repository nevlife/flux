#![allow(clippy::disallowed_types, reason = "the caller's run flag")]

use std::ffi::c_void;
use std::sync::atomic::AtomicBool;

use flux_core::{
    Channel, Error, Executor, ExecutorHandle, PartitionedExecutor, PublisherWatch, Result, Source,
};

use crate::channel::{flux_channel, flux_watch};
use crate::status::{flux_status, status};

/// 0 is success; anything else ends the spin with `FLUX_CALLBACK`.
pub type flux_hook = Option<unsafe extern "C" fn(ctx: *mut c_void) -> i32>;

/// `attach`, `deliver_one`, `channel` required; `drop` runs once, on release or on a failed add.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct flux_source {
    pub ctx: *mut c_void,
    /// 1 attached, 0 nothing to attach to yet, negative failed.
    pub attach: Option<unsafe extern "C" fn(ctx: *mut c_void) -> i32>,
    /// 1 delivered a frame, 0 found none, negative failed.
    pub deliver_one: Option<unsafe extern "C" fn(ctx: *mut c_void) -> i32>,
    /// NULL until attached and after an orphaned channel is dropped. Valid until the next call.
    pub channel: Option<unsafe extern "C" fn(ctx: *mut c_void) -> *const flux_channel>,
    pub drop: Option<unsafe extern "C" fn(ctx: *mut c_void)>,
    /// Optional: while `channel` is NULL, the watch the last attach armed; else retried per tick.
    pub watch: Option<unsafe extern "C" fn(ctx: *mut c_void) -> *const flux_watch>,
    /// The handle `flux_trace_endpoint_init` named this source's endpoint by, or NULL.
    pub endpoint: *const c_void,
}

pub struct flux_executor {
    ex: Executor<'static>,
}

pub struct flux_partitioned {
    px: PartitionedExecutor<'static>,
}

/// Stop, interrupt and wake from any thread. Outlives its executor: the calls then do nothing.
pub struct flux_handle {
    h: ExecutorHandle,
}

fn callback_failed(what: &str, code: i32) -> Error {
    Error::Callback(format!("{what} returned {code}").into())
}

fn outcome(what: &str, r: i32) -> Result<bool> {
    match r {
        0 => Ok(false),
        r if r > 0 => Ok(true),
        r => Err(callback_failed(what, r)),
    }
}

struct CSource {
    s: flux_source,
    attach: unsafe extern "C" fn(*mut c_void) -> i32,
    deliver_one: unsafe extern "C" fn(*mut c_void) -> i32,
    channel: unsafe extern "C" fn(*mut c_void) -> *const flux_channel,
}

// SAFETY: the C side promises its ctx may be used from the thread the executor spins on.
unsafe impl Send for CSource {}

impl Source for CSource {
    fn attach(&mut self) -> Result<bool> {
        // SAFETY: a C function given for this ctx.
        outcome("attach", unsafe { (self.attach)(self.s.ctx) })
    }

    fn deliver_one(&mut self) -> Result<bool> {
        // SAFETY: as above.
        outcome("deliver_one", unsafe { (self.deliver_one)(self.s.ctx) })
    }

    fn channel(&self) -> Option<&Channel> {
        // SAFETY: as above.
        let p = unsafe { (self.channel)(self.s.ctx) };
        // SAFETY: NULL or a live handle that the source keeps until its next call.
        (!p.is_null()).then(|| unsafe { crate::channel::chan_ref(p) })
    }

    fn watch(&self) -> Option<&PublisherWatch> {
        let f = self.s.watch?;
        // SAFETY: as above.
        let p = unsafe { f(self.s.ctx) };
        // SAFETY: as for channel.
        (!p.is_null()).then(|| unsafe { crate::channel::watch_ref(p) })
    }

    fn endpoint(&self) -> *const c_void {
        self.s.endpoint
    }
}

impl Drop for CSource {
    fn drop(&mut self) {
        if let Some(d) = self.s.drop {
            // SAFETY: as above; called once.
            unsafe { d(self.s.ctx) };
        }
    }
}

struct Hook {
    f: unsafe extern "C" fn(*mut c_void) -> i32,
    ctx: *mut c_void,
}

// SAFETY: as for CSource.
unsafe impl Send for Hook {}
// SAFETY: a stop hook is called from the spinning thread while run runs on its own; the C side
// promises the pair is safe to call that way.
unsafe impl Sync for Hook {}

impl Hook {
    fn new(f: flux_hook, ctx: *mut c_void) -> Result<Hook> {
        f.map(|f| Hook { f, ctx })
            .ok_or_else(|| Error::InvalidArgument("hook function is NULL".into()))
    }

    fn call(&self, what: &str) -> Result<()> {
        // SAFETY: a C function given for this ctx.
        match unsafe { (self.f)(self.ctx) } {
            0 => Ok(()),
            r => Err(callback_failed(what, r)),
        }
    }
}

unsafe fn exec<'a>(p: *mut flux_executor) -> &'a mut Executor<'static> {
    // SAFETY: the caller passes a live handle, used by one thread at a time.
    unsafe { &mut (*p).ex }
}

unsafe fn part<'a>(p: *mut flux_partitioned) -> &'a mut PartitionedExecutor<'static> {
    // SAFETY: as above.
    unsafe { &mut (*p).px }
}

unsafe fn write<T>(out: *mut T, v: T) {
    // SAFETY: the caller passes a pointer valid for a write.
    unsafe { out.write(v) };
}

/// Waits with a thread per source below Linux 5.16. `FLUX_UNSUPPORTED` where seccomp forbids it.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_new(out: *mut *mut flux_executor) -> flux_status {
    status(Executor::new().map(|ex| {
        // SAFETY: out is valid for a write.
        unsafe { write(out, Box::into_raw(Box::new(flux_executor { ex }))) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_free(ex: *mut flux_executor) {
    if !ex.is_null() {
        // SAFETY: a handle from flux_executor_new, freed once.
        drop(unsafe { Box::from_raw(ex) });
    }
}

/// Equal priorities start in registration order, then take turns. Not while spinning. At most 127.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_add(
    ex: *mut flux_executor,
    src: flux_source,
    priority: i32,
) -> flux_status {
    let (Some(attach), Some(deliver_one), Some(channel)) =
        (src.attach, src.deliver_one, src.channel)
    else {
        if let Some(d) = src.drop {
            // SAFETY: a C function given for this ctx, called once.
            unsafe { d(src.ctx) };
        }
        return status(Err(Error::InvalidArgument(
            "attach, deliver_one and channel are required".into(),
        )));
    };
    // SAFETY: valid handle.
    let ex = unsafe { exec(ex) };
    status(ex.add(
        CSource {
            s: src,
            attach,
            deliver_one,
            channel,
        },
        priority,
    ))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_size(ex: *mut flux_executor) -> usize {
    // SAFETY: valid handle.
    unsafe { exec(ex) }.size()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_set_pass_budget(
    ex: *mut flux_executor,
    n: u32,
) -> flux_status {
    // SAFETY: valid handle.
    status(unsafe { exec(ex) }.set_pass_budget(n))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_pass_budget(ex: *mut flux_executor) -> u32 {
    // SAFETY: valid handle.
    unsafe { exec(ex) }.pass_budget()
}

/// Work the next pass would find, read without a syscall.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_ready(ex: *mut flux_executor) -> bool {
    // SAFETY: valid handle.
    unsafe { exec(ex) }.ready()
}

/// The last pass ran out of budget, not of frames: skip the next wait.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_has_more(ex: *mut flux_executor) -> bool {
    // SAFETY: valid handle.
    unsafe { exec(ex) }.has_more()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_on_pass(
    ex: *mut flux_executor,
    f: flux_hook,
    ctx: *mut c_void,
) -> flux_status {
    // SAFETY: valid handle.
    let ex = unsafe { exec(ex) };
    status(Hook::new(f, ctx).map(|h| ex.on_pass(move || h.call("on_pass"))))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_dispatch(
    ex: *mut flux_executor,
    delivered: *mut usize,
) -> flux_status {
    // SAFETY: valid handle and a pointer valid for a write.
    status(
        unsafe { exec(ex) }
            .dispatch()
            .map(|n| unsafe { write(delivered, n) }),
    )
}

/// `yield_now` is asked between callbacks, never before the first.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_dispatch_yielding(
    ex: *mut flux_executor,
    yield_now: Option<unsafe extern "C" fn(ctx: *mut c_void) -> bool>,
    ctx: *mut c_void,
    delivered: *mut usize,
) -> flux_status {
    // SAFETY: valid handle.
    let ex = unsafe { exec(ex) };
    let r = match yield_now {
        // SAFETY: a C function given for this ctx.
        Some(f) => ex.dispatch_yielding(|| unsafe { f(ctx) }),
        None => ex.dispatch(),
    };
    // SAFETY: delivered is valid for a write.
    status(r.map(|n| unsafe { write(delivered, n) }))
}

/// Must never overlap a dispatch.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_wait_for_work(
    ex: *mut flux_executor,
    timeout_ns: i64,
) -> flux_status {
    // SAFETY: valid handle.
    status(unsafe { exec(ex) }.wait_for_work(timeout_ns))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_spin(ex: *mut flux_executor, tick_ns: i64) -> flux_status {
    // SAFETY: valid handle.
    status(unsafe { exec(ex) }.spin(tick_ns))
}

/// Also ends once `*run` is atomically set false. `run` must outlive the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_spin_while(
    ex: *mut flux_executor,
    run: *mut bool,
    tick_ns: i64,
) -> flux_status {
    // SAFETY: run is aligned, outlives the call and is only accessed atomically meanwhile.
    let run = unsafe { AtomicBool::from_ptr(run) };
    // SAFETY: valid handle.
    status(unsafe { exec(ex) }.spin_while(run, tick_ns))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_clear_interrupt(ex: *mut flux_executor) {
    // SAFETY: valid handle.
    unsafe { exec(ex) }.clear_interrupt();
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_executor_handle(ex: *mut flux_executor) -> *mut flux_handle {
    // SAFETY: valid handle.
    let h = unsafe { exec(ex) }.handle();
    Box::into_raw(Box::new(flux_handle { h }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_handle_free(h: *mut flux_handle) {
    if !h.is_null() {
        // SAFETY: a handle from flux_*_handle, freed once.
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Held if no spin is running yet, so a stop racing the start of a spin is not lost.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_handle_stop(h: *const flux_handle) {
    // SAFETY: valid handle.
    unsafe { &(*h).h }.stop();
}

/// Ends the current wait, not the loop: `spin` runs `on_pass` and goes on.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_handle_interrupt(h: *const flux_handle) {
    // SAFETY: valid handle.
    unsafe { &(*h).h }.interrupt();
}

/// Breaks the wait so a pass runs; with nothing ready, `spin` waits on.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_handle_wake(h: *const flux_handle) {
    // SAFETY: valid handle.
    unsafe { &(*h).h }.wake();
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_handle_is_spinning(h: *const flux_handle) -> bool {
    // SAFETY: valid handle.
    unsafe { &(*h).h }.is_spinning()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_new(out: *mut *mut flux_partitioned) -> flux_status {
    status(PartitionedExecutor::new().map(|px| {
        // SAFETY: out is valid for a write.
        unsafe { write(out, Box::into_raw(Box::new(flux_partitioned { px }))) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_free(px: *mut flux_partitioned) {
    if !px.is_null() {
        // SAFETY: a handle from flux_partitioned_new, freed once.
        drop(unsafe { Box::from_raw(px) });
    }
}

/// Takes `ex`: its handle is freed here and must not be used again. Take its `flux_handle` first.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_add(
    px: *mut flux_partitioned,
    ex: *mut flux_executor,
    index: *mut usize,
) {
    // SAFETY: a handle from flux_executor_new, consumed here.
    let ex = unsafe { Box::from_raw(ex) }.ex;
    // SAFETY: valid handle.
    let i = unsafe { part(px) }.add(ex);
    // SAFETY: index is valid for a write.
    unsafe { write(index, i) };
}

/// A thread running `run` until `stop`, which may come first: `run` then returns at once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_add_external(
    px: *mut flux_partitioned,
    run: flux_hook,
    stop: flux_hook,
    ctx: *mut c_void,
    index: *mut usize,
) -> flux_status {
    let hooks = Hook::new(run, ctx).and_then(|r| Ok((r, Hook::new(stop, ctx)?)));
    status(hooks.map(|(r, s)| {
        // SAFETY: valid handle.
        let i = unsafe { part(px) }.add_external(
            move || r.call("run"),
            move || {
                let _ = s.call("stop");
            },
        );
        // SAFETY: index is valid for a write.
        unsafe { write(index, i) };
    }))
}

/// Runs on the partition's own thread before its loop, on every spin.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_on_thread_start(
    px: *mut flux_partitioned,
    index: usize,
    init: flux_hook,
    ctx: *mut c_void,
) -> flux_status {
    // SAFETY: valid handle.
    let px = unsafe { part(px) };
    status(Hook::new(init, ctx).and_then(|h| px.on_thread_start(index, move || h.call("init"))))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_size(px: *mut flux_partitioned) -> usize {
    // SAFETY: valid handle.
    unsafe { part(px) }.size()
}

/// The first failure on any partition stops every partition and is returned here.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_spin(
    px: *mut flux_partitioned,
    tick_ns: i64,
) -> flux_status {
    // SAFETY: valid handle.
    status(unsafe { part(px) }.spin(tick_ns))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_partitioned_handle(px: *mut flux_partitioned) -> *mut flux_handle {
    // SAFETY: valid handle.
    let h = unsafe { part(px) }.handle();
    Box::into_raw(Box::new(flux_handle { h }))
}
