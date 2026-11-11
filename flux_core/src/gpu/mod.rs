//! GPU payloads, NVIDIA only. The driver is opened at run time, so a host without CUDA runs the
//! same build and reports `Route::None`. A channel that declares a stream keeps the host slot
//! protocol unchanged; what changes is where `device_ptr` counts from, and that commit and release
//! wait for the stream first.

#[doc(hidden)]
pub mod arena;
mod driver;
#[cfg(test)]
mod fake;
mod fd;
pub(crate) mod ledger;
#[cfg(test)]
mod tests;
mod vmm;

use std::collections::VecDeque;
use std::ffi::c_void;
use std::fmt;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex, OnceLock, PoisonError, Weak};

use driver::{
    ATTR_HOST_REGISTER_SUPPORTED, ATTR_INTEGRATED, ATTR_PAGEABLE_MEMORY_ACCESS,
    ATTR_POSIX_FD_HANDLE_SUPPORTED, ATTR_READ_ONLY_HOST_REGISTER_SUPPORTED, ATTR_VMM_SUPPORTED,
    CuResult, DevicePtr, Driver, EVENT_BLOCKING_SYNC, EVENT_DISABLE_TIMING,
    MEM_HOST_REGISTER_DEVICEMAP, MEM_HOST_REGISTER_PORTABLE, MEM_HOST_REGISTER_READ_ONLY,
    NOT_READY, SUCCESS,
};
pub(crate) use fd::remove_dead_endpoint;
pub(crate) use vmm::{
    DeviceAlloc, DeviceBuffer, DeviceImport, device_for_uuid, device_uuid, endpoint,
};

use crate::error::{Error, Result};
use crate::executor::Control;
use crate::sys;
use crate::trace::event;

/// How a host's GPU reaches a channel's payload. Judged per device, first match wins.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Route {
    /// No route; `Platform::reason` says why.
    #[default]
    None,
    /// Integrated with pageable access: the slot mapping is already device-addressable.
    ShmDirect,
    /// Integrated: the slot mapping becomes device-addressable once registered.
    ShmRegistered,
    /// A separate device allocation, exported as a POSIX fd.
    DeviceHandle,
}

impl fmt::Display for Route {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

/// The verdict and the attributes it was read from, so a caller who disagrees sees why.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Debug, Default)]
pub struct Platform {
    pub route: Route,
    pub device: i32,
    pub integrated: bool,
    pub pageable_access: bool,
    pub host_register: bool,
    pub vmm: bool,
    pub posix_fd_handle: bool,
    /// Non-empty exactly when `route` is `None`.
    pub reason: String,
}

/// `integrated` comes first: host registration also succeeds on a discrete GPU, where the device
/// pointer it yields crosses `PCIe`. Registration wins over pageable access: on Thor an unregistered
/// slot sends `cudaMemcpyAsync` down the synchronous pageable path. A route whose handle cannot be
/// made is no route.
fn select(p: &Platform) -> Route {
    if p.integrated && p.host_register {
        Route::ShmRegistered
    } else if p.integrated && p.pageable_access {
        Route::ShmDirect
    } else if p.vmm && p.posix_fd_handle {
        Route::DeviceHandle
    } else {
        Route::None
    }
}

/// Calls `cuInit` on first use (hundreds of milliseconds). Never fails: a host without a GPU is
/// `Route::None` with a reason.
pub fn probe(device: i32) -> Platform {
    let none = |why: &str| Platform {
        device,
        reason: why.to_owned(),
        ..Platform::default()
    };
    let drv = Driver::get();
    let core = match drv.usable_core() {
        Ok(c) => c,
        Err(why) => return none(why),
    };
    let mut count = 0;
    // SAFETY: count is a valid out-pointer.
    if unsafe { (core.device_count)(&raw mut count) } != SUCCESS {
        return none("cuDeviceGetCount failed");
    }
    if count <= 0 {
        return none("no CUDA device");
    }
    if device < 0 || device >= count {
        return none("device index out of range");
    }
    let mut handle = 0;
    // SAFETY: handle is a valid out-pointer.
    if unsafe { (core.device_get)(&raw mut handle, device) } != SUCCESS {
        return none("cuDeviceGet failed");
    }
    let mut p = Platform {
        device,
        integrated: core.attribute(handle, ATTR_INTEGRATED),
        pageable_access: core.attribute(handle, ATTR_PAGEABLE_MEMORY_ACCESS),
        host_register: core.attribute(handle, ATTR_HOST_REGISTER_SUPPORTED),
        vmm: core.attribute(handle, ATTR_VMM_SUPPORTED),
        posix_fd_handle: core.attribute(handle, ATTR_POSIX_FD_HANDLE_SUPPORTED),
        ..Platform::default()
    };
    p.route = select(&p);
    #[cfg(test)]
    if let Some(r) = tests::ROUTE.get() {
        p.route = r;
    }
    if p.route == Route::None {
        "device shares no memory route: not integrated and no fd export".clone_into(&mut p.reason);
    }
    p
}

/// The current context's device, or 0 on a thread with none.
pub fn current_device() -> i32 {
    Driver::get()
        .core
        .as_ref()
        .ok()
        .and_then(driver::Core::current_device)
        .unwrap_or(0)
}

/// The route of `device`, or why there is none, spelled for a caller who declared a stream.
pub(crate) fn require_route(device: i32) -> Result<Platform> {
    refuse_wedged()?;
    let p = probe(device);
    if p.route != Route::None {
        return Ok(p);
    }
    let yn = |b| if b { "yes" } else { "no" };
    Err(Error::Unsupported(format!(
        "no GPU route on this host: {}. Device {}: integrated={}, pageable_access={}, \
         host_register={}, vmm={}, posix_fd={}. flux takes an integrated GPU (Jetson) or a \
         discrete one with VMM and POSIX fd export; without one, declare no stream",
        p.reason,
        p.device,
        yn(p.integrated),
        yn(p.pageable_access),
        yn(p.host_register),
        yn(p.vmm),
        yn(p.posix_fd_handle),
    )))
}

/// The CUDA error that left this process's GPU unusable, 0 while none has. Set once: CUDA answers
/// every later call with the same error, and only a new process clears it.
static WEDGED: AtomicI32 = AtomicI32::new(0);

/// The errors cuda.h says leave the process unusable until it is relaunched.
fn sticky(rc: CuResult) -> bool {
    matches!(rc, 226 | 700 | 702 | 710 | 714..=719 | 721 | 810 | 911)
}

/// `rc` is success. A sticky error wedges the process's GPU state on the way.
fn ok(rc: CuResult) -> bool {
    if rc == SUCCESS {
        return true;
    }
    failed(rc);
    false
}

#[cold]
fn failed(rc: CuResult) {
    if sticky(rc)
        && WEDGED
            .compare_exchange(0, rc, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
    {
        event!(flux_trace_gpu_wedged(rc));
    }
}

/// The sticky CUDA error this process hit, after which every GPU channel refuses at once: a take
/// as `refused.fence`, a loan as none, a commit or publish as `GpuWedged`, and a new GPU channel
/// or stream as `Unsupported`. Restart the process.
pub fn wedged() -> Option<i32> {
    let rc = WEDGED.load(Ordering::Relaxed);
    (rc != 0).then_some(rc)
}

fn refuse_wedged() -> Result<()> {
    match wedged() {
        None => Ok(()),
        Some(rc) => Err(Error::Unsupported(format!(
            "this process's GPU context hit CUDA error {rc}, which CUDA returns from every call \
             until the process is restarted"
        ))),
    }
}

struct StreamInner {
    native: usize,
    event: usize,
    ctx: usize,
    device: i32,
    owned: bool,
    born: u32,
    waiter: OnceLock<Option<Arc<Waiter>>>,
}

impl Drop for StreamInner {
    fn drop(&mut self) {
        // A forked child has no usable CUDA: its copies are left to the process's end.
        if sys::fork_gen() != self.born {
            return;
        }
        if let Some(Some(w)) = self.waiter.get() {
            w.stop();
        }
        let Ok(core) = &Driver::get().core else {
            return;
        };
        core.with_context(self.ctx as *mut c_void, || {
            // SAFETY: an event this value created, destroyed once.
            unsafe { (core.event_destroy)(self.event as *mut c_void) };
            if self.owned {
                // SAFETY: a stream this value created, destroyed once.
                unsafe { (core.stream_destroy)(self.native as *mut c_void) };
            }
        });
    }
}

/// The CUDA stream a channel's frames are produced or consumed on, declared once per channel.
/// Commit and release wait for it, which is per stream, not per frame: whatever else was queued on
/// it is waited for too. Clones share one stream and its fence event.
#[derive(Clone, Default)]
pub struct Stream(Option<Arc<StreamInner>>);

impl fmt::Debug for Stream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            None => f.write_str("Stream(none)"),
            Some(s) => write!(f, "Stream({:#x}, device {})", s.native, s.device),
        }
    }
}

impl Stream {
    /// A stream flux owns, on the current context or the current device's primary one. Holds
    /// only the work queued through this channel's views, so a fence waits for that alone.
    pub fn create() -> Result<Stream> {
        let core = core()?;
        let device = current_device();
        let ctx = core.ensure_context(device).ok_or_else(|| {
            Error::Unsupported("no CUDA context and the primary context was refused".into())
        })?;
        let mut native = std::ptr::null_mut();
        // SAFETY: native is a valid out-pointer; flags 0 is CU_STREAM_DEFAULT, which keeps the
        // implicit ordering with the legacy stream a caller may also use.
        if !ok(unsafe { (core.stream_create)(&raw mut native, 0) }) {
            return Err(Error::Unsupported("cuStreamCreate failed".into()));
        }
        Stream::with_event(core, native, ctx, device, true)
    }

    /// A stream flux owns on `ordinal`'s primary context, for copies the caller did not ask a
    /// stream for: a host subscriber reading a device payload.
    pub(crate) fn on_device(ordinal: i32) -> Result<Stream> {
        let core = core()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let mut native = std::ptr::null_mut();
        // SAFETY: native is a valid out-pointer; flags 0 is CU_STREAM_DEFAULT.
        let rc = core.with_context(ctx, || unsafe { (core.stream_create)(&raw mut native, 0) });
        if !ok(rc) {
            return Err(Error::Unsupported("cuStreamCreate failed".into()));
        }
        Stream::with_event(core, native, ctx, ordinal, true)
    }

    /// The caller's stream. Null is CUDA's legacy default stream, which names a different stream
    /// on each thread's current context; a channel cannot know which thread releases a view, so
    /// it is refused here.
    ///
    /// # Safety
    /// `native` must be a `CUstream` that outlives every channel and view holding this value.
    pub unsafe fn from_native(native: *mut c_void) -> Result<Stream> {
        if native.is_null() {
            return Err(Error::InvalidArgument(
                "CUDA's legacy default stream resolves against the calling thread's context, so a \
                 fence on a thread without one fails and costs a slot; pass Stream::create() or \
                 your own stream handle"
                    .into(),
            ));
        }
        let core = core()?;
        let mut ctx = std::ptr::null_mut();
        // SAFETY: the caller vouches for native; ctx is a valid out-pointer.
        if unsafe { (core.stream_get_ctx)(native, &raw mut ctx) } != SUCCESS {
            return Err(Error::InvalidArgument(
                "cuStreamGetCtx refused this stream".into(),
            ));
        }
        let device = core.with_context(ctx, || core.current_device().unwrap_or(0));
        Stream::with_event(core, native, ctx, device, false)
    }

    /// The fence event lives in the stream's context. Blocking sync, so a waiting thread sleeps
    /// instead of spinning a core through a kernel's run.
    fn with_event(
        core: &driver::Core,
        native: *mut c_void,
        ctx: *mut c_void,
        device: i32,
        owned: bool,
    ) -> Result<Stream> {
        sys::watch_fork();
        let mut event = std::ptr::null_mut();
        let rc = core.with_context(ctx, || {
            // SAFETY: event is a valid out-pointer.
            unsafe {
                (core.event_create)(&raw mut event, EVENT_BLOCKING_SYNC | EVENT_DISABLE_TIMING)
            }
        });
        if !ok(rc) {
            if owned {
                // SAFETY: created above, never handed out.
                unsafe { (core.stream_destroy)(native) };
            }
            return Err(Error::Unsupported("cuEventCreate failed".into()));
        }
        Ok(Stream(Some(Arc::new(StreamInner {
            native: native as usize,
            event: event as usize,
            ctx: ctx as usize,
            device,
            owned,
            born: sys::fork_gen(),
            waiter: OnceLock::new(),
        }))))
    }

    pub fn declared(&self) -> bool {
        self.0.is_some()
    }

    /// The `CUstream`, null when none is declared.
    pub fn native(&self) -> *mut c_void {
        self.0
            .as_ref()
            .map_or(std::ptr::null_mut(), |s| s.native as *mut c_void)
    }

    /// The device ordinal the stream's context is on, -1 when none is declared.
    pub fn device(&self) -> i32 {
        self.0.as_ref().map_or(-1, |s| s.device)
    }

    /// Until everything queued on the stream so far has run. True at once when none is declared.
    /// False means the wait did not happen: read it as "still running", never as done.
    pub fn wait(&self) -> bool {
        let Some(s) = &self.0 else {
            return true;
        };
        if sys::fork_gen() != s.born {
            return false;
        }
        #[cfg(test)]
        if tests::FAIL_FENCES.get() {
            return false;
        }
        let Ok(core) = &Driver::get().core else {
            return false;
        };
        // SAFETY: both handles live as long as s.
        let recorded =
            unsafe { (core.event_record)(s.event as *mut c_void, s.native as *mut c_void) };
        // SAFETY: as above.
        ok(recorded) && ok(unsafe { (core.event_synchronize)(s.event as *mut c_void) })
    }
}

/// How a bounded wait ended. `Late` is a failure for now that may still finish.
pub(crate) enum Fenced {
    Done,
    Failed,
    Late(Late),
}

const PENDING: u32 = 0;
const DONE: u32 = 1;
const FAILED: u32 = 2;

/// Work marked by an event of its own, which the stream's waiter thread sleeps on: the stream's
/// own event is recorded again by the next wait.
struct Mark {
    event: usize,
    ctx: usize,
    born: u32,
    state: AtomicU32,
    /// Poked once the work is done, so the channel's own thread gives the slot back.
    waker: Weak<Control>,
    /// What a channel dropped before the work ended left here: let go once the work is done, or
    /// kept for good if it failed, by the waiter.
    orphan: Mutex<Option<Box<dyn Send>>>,
}

impl Mark {
    fn query(&self) -> bool {
        if sys::fork_gen() != self.born {
            return false;
        }
        let Ok(core) = &Driver::get().core else {
            return false;
        };
        // SAFETY: an event this mark holds.
        let rc = unsafe { (core.event_query)(self.event as *mut c_void) };
        rc == SUCCESS || (rc != NOT_READY && ok(rc))
    }
}

impl Drop for Mark {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let Ok(core) = &Driver::get().core else {
            return;
        };
        core.with_context(self.ctx as *mut c_void, || {
            // SAFETY: created by Stream::mark, destroyed once, after the waiter is done with it.
            unsafe { (core.event_destroy)(self.event as *mut c_void) };
        });
    }
}

/// Work nobody waited for yet: a bounded wait that ran out, or a deferred release.
#[derive(Clone)]
pub(crate) struct Late(Arc<Mark>);

impl Late {
    /// Asked by a thread that is awake anyway, so the device may have finished just before the
    /// waiter woke: one query, never a loop.
    pub(crate) fn done(&self) -> bool {
        match self.0.state.load(Ordering::Acquire) {
            DONE => true,
            FAILED => false,
            _ => self.0.query(),
        }
    }

    pub(crate) fn failed(&self) -> bool {
        self.0.state.load(Ordering::Acquire) == FAILED
    }

    /// For work whose channel is gone: `x` is let go on the reaper once the work is done, or kept
    /// for good if it failed, as a failed fence keeps its slot. Never touches shared memory.
    pub(crate) fn hand_off(&self, x: Box<dyn Send>) {
        let mut slot = self.0.orphan.lock().unwrap_or_else(PoisonError::into_inner);
        let state = self.0.state.load(Ordering::Acquire);
        if state == PENDING {
            *slot = Some(x);
            return;
        }
        drop(slot);
        settle_orphan(state, x);
    }
}

fn settle_orphan(state: u32, x: Box<dyn Send>) {
    if state == DONE {
        sys::retire(x);
    } else {
        std::mem::forget(x);
    }
}

/// One thread per stream that defers or bounds a wait. A stream's events complete in order, so
/// taking them in order never waits behind another stream's work. It sleeps in the driver until
/// each is done and never touches a channel: giving the slot back stays on the channel's thread.
struct Waiter {
    queue: Mutex<VecDeque<Arc<Mark>>>,
    ready: Condvar,
    stop: AtomicBool,
}

impl Waiter {
    /// Detached: a hung kernel would keep a join from returning. It ends once its queue is empty
    /// and the stream is gone.
    fn start() -> Option<Arc<Waiter>> {
        let w = Arc::new(Waiter {
            queue: Mutex::new(VecDeque::new()),
            ready: Condvar::new(),
            stop: AtomicBool::new(false),
        });
        let mine = Arc::clone(&w);
        let builder = std::thread::Builder::new().name("flux-gpu-wait".into());
        sys::spawn_helper(builder, move || mine.run()).ok()?;
        Some(w)
    }

    fn run(&self) {
        let Ok(core) = &Driver::get().core else {
            return;
        };
        while let Some(mark) = self.next() {
            // SAFETY: an event the mark holds while this runs.
            let done = ok(unsafe { (core.event_synchronize)(mark.event as *mut c_void) });
            let state = if done { DONE } else { FAILED };
            mark.state.store(state, Ordering::Release);
            sys::futex_wake_private(&mark.state);
            let orphan = mark
                .orphan
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .take();
            if let Some(x) = orphan {
                settle_orphan(state, x);
            }
            if let Some(ctl) = mark.waker.upgrade() {
                ctl.poke();
            }
        }
    }

    fn next(&self) -> Option<Arc<Mark>> {
        let mut q = self.queue.lock().unwrap_or_else(PoisonError::into_inner);
        loop {
            if let Some(m) = q.pop_front() {
                return Some(m);
            }
            if self.stop.load(Ordering::Acquire) {
                return None;
            }
            q = self.ready.wait(q).unwrap_or_else(PoisonError::into_inner);
        }
    }

    fn push(&self, mark: Arc<Mark>) {
        self.queue
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push_back(mark);
        self.ready.notify_one();
    }

    fn stop(&self) {
        let _q = self.queue.lock().unwrap_or_else(PoisonError::into_inner);
        self.stop.store(true, Ordering::Release);
        self.ready.notify_one();
    }
}

impl StreamInner {
    fn waiter(&self) -> Option<&Arc<Waiter>> {
        self.waiter.get_or_init(Waiter::start).as_ref()
    }
}

impl Stream {
    /// Starts the stream's waiter thread on the caller's thread, so the first deferred or bounded
    /// wait does not start it on an executor thread.
    pub(crate) fn prepare_waiter(&self) {
        if let Some(s) = &self.0 {
            s.waiter();
        }
    }

    /// Marks everything queued so far with an event of its own and hands it to the waiter, which
    /// pokes `waker` once it is done.
    pub(crate) fn mark(&self, waker: Weak<Control>) -> Option<Late> {
        let s = self.0.as_ref()?;
        if sys::fork_gen() != s.born {
            return None;
        }
        #[cfg(test)]
        if tests::FAIL_FENCES.get() {
            return None;
        }
        let Ok(core) = &Driver::get().core else {
            return None;
        };
        let mut event = std::ptr::null_mut();
        let created = core.with_context(s.ctx as *mut c_void, || {
            // SAFETY: event is a valid out-pointer.
            unsafe {
                (core.event_create)(&raw mut event, EVENT_BLOCKING_SYNC | EVENT_DISABLE_TIMING)
            }
        });
        if !ok(created) {
            return None;
        }
        let mark = Arc::new(Mark {
            event: event as usize,
            ctx: s.ctx,
            born: s.born,
            state: AtomicU32::new(PENDING),
            waker,
            orphan: Mutex::new(None),
        });
        // SAFETY: the event just made; the stream lives as long as s.
        if !ok(unsafe { (core.event_record)(event, s.native as *mut c_void) }) {
            return None;
        }
        s.waiter()?.push(Arc::clone(&mark));
        Some(Late(mark))
    }

    /// `wait` with an upper bound, 0 for none. A bound hands the work to the waiter thread and
    /// sleeps on its word, so the device's signal still ends the wait.
    pub(crate) fn wait_within(&self, timeout_ns: u64, waker: Weak<Control>) -> Fenced {
        if self.0.is_none() || timeout_ns == 0 {
            return if self.wait() {
                Fenced::Done
            } else {
                Fenced::Failed
            };
        }
        let Some(late) = self.mark(waker) else {
            return Fenced::Failed;
        };
        let deadline = sys::monotonic_ns().saturating_add(timeout_ns);
        loop {
            match late.0.state.load(Ordering::Acquire) {
                DONE => return Fenced::Done,
                FAILED => return Fenced::Failed,
                _ => {}
            }
            let now = sys::monotonic_ns();
            if now >= deadline {
                return Fenced::Late(late);
            }
            #[allow(clippy::cast_possible_wrap)]
            sys::futex_wait_private(&late.0.state, PENDING, (deadline - now) as i64);
        }
    }

    /// Queues `n` bytes from host `src` to device `dst`. A registered `src` keeps the copy off
    /// the calling thread; a pageable one is staged before this returns.
    pub(crate) fn copy_to_device(&self, dst: DevicePtr, src: *const u8, n: usize) -> bool {
        self.copy(|c, stream| {
            // SAFETY: the caller holds src..src+n and dst..dst+n until the stream is waited for.
            unsafe { (c.htod)(dst, src.cast(), n, stream) }
        })
    }

    /// Queues `n` bytes from device `src` to host `dst`.
    pub(crate) fn copy_to_host(&self, dst: *mut u8, src: DevicePtr, n: usize) -> bool {
        self.copy(|c, stream| {
            // SAFETY: as copy_to_device.
            unsafe { (c.dtoh)(dst.cast(), src, n, stream) }
        })
    }

    fn copy(&self, f: impl FnOnce(&driver::Copies, *mut c_void) -> CuResult) -> bool {
        let Some(s) = &self.0 else {
            return false;
        };
        if sys::fork_gen() != s.born {
            return false;
        }
        let drv = Driver::get();
        let (Ok(core), Ok(copies)) = (&drv.core, &drv.copies) else {
            return false;
        };
        ok(core.with_context(s.ctx as *mut c_void, || f(copies, s.native as *mut c_void)))
    }

    /// Orders the next work queued on this stream after everything queued on `other` so far, on
    /// the device: no thread waits.
    pub fn after(&self, other: &Stream) -> bool {
        let (Some(s), Some(o)) = (&self.0, &other.0) else {
            return true;
        };
        if sys::fork_gen() != s.born {
            return false;
        }
        let Ok(core) = &Driver::get().core else {
            return false;
        };
        let event = o.event as *mut c_void;
        // SAFETY: both streams and other's event live as long as s and o.
        let recorded = unsafe { (core.event_record)(event, o.native as *mut c_void) };
        // SAFETY: as above; the wait takes the event's state now, so a later record is harmless.
        ok(recorded) && ok(unsafe { (core.stream_wait_event)(s.native as *mut c_void, event, 0) })
    }
}

fn core() -> Result<&'static driver::Core> {
    refuse_wedged()?;
    Driver::get()
        .usable_core()
        .map_err(|why| Error::Unsupported(why.to_owned()))
}

/// A host range registered with the device, unregistered on drop. Made in the device's primary
/// context, like a VMM allocation, so a context the caller destroys does not take it along; portable,
/// so a stream in any context of the device reaches it.
pub(crate) struct HostRegistration {
    base: usize,
    bytes: usize,
    device_base: DevicePtr,
    ctx: usize,
    born: u32,
}

impl HostRegistration {
    /// `read_only` for a `PROT_READ` mapping (a subscriber's payload), which a device that supports
    /// read-only registration refuses otherwise.
    pub(crate) fn new(base: *mut u8, bytes: usize, read_only: bool, device: i32) -> Result<Self> {
        let drv = Driver::get();
        let core = core()?;
        let reg = drv
            .host_reg
            .as_ref()
            .map_err(|why| Error::Unsupported((*why).to_owned()))?;
        let ctx = core
            .primary_context(device)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        sys::watch_fork();
        let mut handle = 0;
        // SAFETY: handle is a valid out-pointer.
        if unsafe { (core.device_get)(&raw mut handle, device) } != SUCCESS {
            return Err(Error::Unsupported("cuDeviceGet failed".into()));
        }
        let mut flags = MEM_HOST_REGISTER_PORTABLE | MEM_HOST_REGISTER_DEVICEMAP;
        if read_only && core.attribute(handle, ATTR_READ_ONLY_HOST_REGISTER_SUPPORTED) {
            flags |= MEM_HOST_REGISTER_READ_ONLY;
        }
        core.with_context(ctx, || {
            // SAFETY: base..base+bytes is a mapping this process holds for the registration's life.
            if unsafe { (reg.register)(base.cast(), bytes, flags) } != SUCCESS {
                return Err(Error::Unsupported(
                    "cuMemHostRegister refused the payload mapping".into(),
                ));
            }
            let mut device_base = 0;
            // SAFETY: base was registered just above; device_base is a valid out-pointer.
            if unsafe { (reg.device_pointer)(&raw mut device_base, base.cast(), 0) } != SUCCESS {
                // SAFETY: registered above.
                unsafe { (reg.unregister)(base.cast()) };
                return Err(Error::Unsupported(
                    "the registered payload has no device address".into(),
                ));
            }
            ledger::made(ledger::Held::Registration(base as usize), bytes);
            Ok(HostRegistration {
                base: base as usize,
                bytes,
                device_base,
                ctx: ctx as usize,
                born: sys::fork_gen(),
            })
        })
    }

    pub(crate) fn device_base(&self) -> DevicePtr {
        self.device_base
    }
}

impl Drop for HostRegistration {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let drv = Driver::get();
        let (Ok(core), Ok(reg)) = (&drv.core, &drv.host_reg) else {
            return;
        };
        // The thread dropping it may have no context; the registration lives in this one.
        #[cfg(test)]
        tests::note_unregister(self.base);
        ledger::undone(ledger::Held::Registration(self.base), self.bytes);
        core.with_context(self.ctx as *mut c_void, || {
            // SAFETY: registered in new(), unregistered once, before the mapping goes.
            unsafe { (reg.unregister)(self.base as *mut c_void) };
        });
    }
}

/// Whether `device` pins a `PROT_READ` mapping. Without it a subscriber's mapping cannot be
/// registered, and a copy from it is staged by the driver on the calling thread.
pub(crate) fn registers_read_only(device: i32) -> bool {
    let Ok(core) = Driver::get().usable_core() else {
        return false;
    };
    let mut handle = 0;
    // SAFETY: handle is a valid out-pointer.
    let found = unsafe { (core.device_get)(&raw mut handle, device) } == SUCCESS;
    found && core.attribute(handle, ATTR_READ_ONLY_HOST_REGISTER_SUPPORTED)
}

/// Pinned host memory of this process, for a device payload copied to the host.
pub(crate) struct HostBuffer {
    reg: Option<HostRegistration>,
    base: usize,
    bytes: usize,
}

impl HostBuffer {
    pub(crate) fn new(bytes: usize, device: i32) -> Result<HostBuffer> {
        // SAFETY: a fresh private mapping; no address is asked for.
        let p = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                bytes,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS,
                -1,
                0,
            )
        };
        if p == libc::MAP_FAILED {
            return Err(Error::io(
                "mmap",
                "host copy",
                std::io::Error::last_os_error(),
            ));
        }
        let mut b = HostBuffer {
            reg: None,
            base: p as usize,
            bytes,
        };
        b.reg = Some(HostRegistration::new(p.cast(), bytes, false, device)?);
        Ok(b)
    }

    pub(crate) fn base(&self) -> *mut u8 {
        self.base as *mut u8
    }
}

impl Drop for HostBuffer {
    fn drop(&mut self) {
        drop(self.reg.take());
        // SAFETY: mapped in new() at this size, unregistered just above.
        unsafe { libc::munmap(self.base as *mut c_void, self.bytes) };
    }
}

/// What makes a segment's payload device-addressable, held once per mapping: registering or
/// importing the same memory twice is refused by the driver.
pub(crate) enum DevicePayload {
    Registered(HostRegistration),
    Exported(DeviceAlloc),
    Imported(DeviceImport),
}

impl DevicePayload {
    /// The device address of slot 0's payload.
    pub(crate) fn base(&self) -> DevicePtr {
        match self {
            DevicePayload::Registered(r) => r.device_base(),
            DevicePayload::Exported(a) => a.base(),
            DevicePayload::Imported(i) => i.base(),
        }
    }
}

/// How long the two seams waited, and how often a wait failed. Kept per channel across attaches.
#[derive(Default)]
pub(crate) struct FenceStats {
    commit_ns: AtomicU64,
    commit_waits: AtomicU64,
    commit_max_ns: AtomicU64,
    release_ns: AtomicU64,
    release_waits: AtomicU64,
    release_max_ns: AtomicU64,
    failed: AtomicU64,
    /// The channel's bound on each wait, 0 for none. Here so it outlives a re-attach.
    pub(crate) timeout_ns: AtomicU64,
    /// Releases record an event and return; the slot goes back once the waiter saw it done.
    pub(crate) deferred: AtomicBool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Seam {
    Commit,
    Release,
}

/// Totals, counts and the longest single wait of each seam: what decides whether a deferred
/// commit or release would buy anything (`gpu_platform.md` 4.4).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FenceWait {
    pub commit_ns: u64,
    pub commit_waits: u64,
    pub commit_max_ns: u64,
    pub release_ns: u64,
    pub release_waits: u64,
    pub release_max_ns: u64,
}

impl FenceStats {
    /// A failed or timed-out wait blocked too, so it is timed like the rest, and counts as failed.
    pub(crate) fn fence(&self, stream: &Stream, seam: Seam, waker: Weak<Control>) -> Fenced {
        let t0 = sys::monotonic_ns();
        let fenced = stream.wait_within(self.timeout_ns.load(Ordering::Relaxed), waker);
        let ok = matches!(fenced, Fenced::Done);
        let dt = sys::monotonic_ns().saturating_sub(t0);
        let (sum, count, worst) = match seam {
            Seam::Commit => (&self.commit_ns, &self.commit_waits, &self.commit_max_ns),
            Seam::Release => (&self.release_ns, &self.release_waits, &self.release_max_ns),
        };
        sum.fetch_add(dt, Ordering::Relaxed);
        count.fetch_add(1, Ordering::Relaxed);
        worst.fetch_max(dt, Ordering::Relaxed);
        if !ok {
            self.failed.fetch_add(1, Ordering::Relaxed);
        }
        fenced
    }

    pub(crate) fn failed(&self) -> u64 {
        self.failed.load(Ordering::Relaxed)
    }

    /// A deferred release whose work failed: counted as the fence it replaced would have been.
    pub(crate) fn count_failed(&self) {
        self.failed.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn snapshot(&self) -> FenceWait {
        let l = |a: &AtomicU64| a.load(Ordering::Relaxed);
        FenceWait {
            commit_ns: l(&self.commit_ns),
            commit_waits: l(&self.commit_waits),
            commit_max_ns: l(&self.commit_max_ns),
            release_ns: l(&self.release_ns),
            release_waits: l(&self.release_waits),
            release_max_ns: l(&self.release_max_ns),
        }
    }
}
