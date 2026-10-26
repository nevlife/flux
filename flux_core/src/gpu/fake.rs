//! A stand-in for libcuda, in this test binary's child processes started with `ENV` set: one
//! integrated device, streams whose work the test submits and finishes or fails, a sticky error
//! the test raises, and contexts it can destroy. It needs no GPU. Using a destroyed context or
//! destroying an object twice is recorded in `misuse` rather than crashing, as the real driver may.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::ffi::{c_int, c_uint, c_void};
use std::sync::{Condvar, Mutex, MutexGuard, PoisonError};

use super::Stream;
use super::driver::{
    ATTR_HOST_REGISTER_SUPPORTED, ATTR_INTEGRATED, ATTR_PAGEABLE_MEMORY_ACCESS, Core, CuResult,
    DevicePtr, Driver, Handle, HostReg, MEM_HOST_REGISTER_PORTABLE, NOT_READY, SUCCESS,
};

pub(crate) const ENV: &str = "FLUX_FAKE_CUDA";

const INVALID_VALUE: CuResult = 1;
const INVALID_DEVICE: CuResult = 101;
const INVALID_CONTEXT: CuResult = 201;
const INVALID_HANDLE: CuResult = 400;
const ALREADY_REGISTERED: CuResult = 712;
const NOT_REGISTERED: CuResult = 713;

const PRIMARY: usize = 0x100;

struct FakeStream {
    ctx: usize,
    submitted: u64,
    completed: u64,
    error: CuResult,
}

struct Event {
    ctx: usize,
    stream: usize,
    target: u64,
}

struct Registration {
    ctx: usize,
    portable: bool,
}

struct State {
    next: usize,
    streams: BTreeMap<usize, FakeStream>,
    events: BTreeMap<usize, Event>,
    registrations: BTreeMap<usize, Registration>,
    destroyed: Vec<usize>,
    sticky: CuResult,
    calls: u64,
    misuse: Vec<String>,
}

static STATE: Mutex<State> = Mutex::new(State {
    next: 0x1000,
    streams: BTreeMap::new(),
    events: BTreeMap::new(),
    registrations: BTreeMap::new(),
    destroyed: Vec::new(),
    sticky: SUCCESS,
    calls: 0,
    misuse: Vec::new(),
});
static CHANGED: Condvar = Condvar::new();

thread_local! {
    static CURRENT: RefCell<Vec<usize>> = const { RefCell::new(Vec::new()) };
}

fn state() -> MutexGuard<'static, State> {
    STATE.lock().unwrap_or_else(PoisonError::into_inner)
}

fn current() -> usize {
    CURRENT.with(|c| c.borrow().last().copied().unwrap_or(0))
}

/// Runs a driver call: counted, refused with the sticky error once there is one.
fn call(f: impl FnOnce(&mut State) -> CuResult) -> CuResult {
    let mut s = state();
    s.calls += 1;
    if s.sticky != SUCCESS {
        return s.sticky;
    }
    f(&mut s)
}

/// The current context, which a call that makes something needs.
fn context(s: &mut State) -> Result<usize, CuResult> {
    let ctx = current();
    if ctx == 0 {
        return Err(INVALID_CONTEXT);
    }
    if s.destroyed.contains(&ctx) {
        s.misuse.push(format!("used destroyed context {ctx:#x}"));
        return Err(INVALID_CONTEXT);
    }
    Ok(ctx)
}

fn id(s: &mut State) -> usize {
    s.next += 1;
    s.next
}

unsafe extern "C" fn device_count(n: *mut c_int) -> CuResult {
    // SAFETY: the caller passes a valid out-pointer.
    call(|_| unsafe { put(n, 1) })
}

unsafe extern "C" fn device_get(dev: *mut c_int, ordinal: c_int) -> CuResult {
    call(|_| {
        if ordinal != 0 {
            return INVALID_DEVICE;
        }
        // SAFETY: the caller passes a valid out-pointer.
        unsafe { put(dev, 0) }
    })
}

unsafe extern "C" fn device_attribute(v: *mut c_int, attr: c_int, _dev: c_int) -> CuResult {
    call(|_| {
        let on = matches!(
            attr,
            ATTR_INTEGRATED | ATTR_PAGEABLE_MEMORY_ACCESS | ATTR_HOST_REGISTER_SUPPORTED
        );
        // SAFETY: the caller passes a valid out-pointer.
        unsafe { put(v, c_int::from(on)) }
    })
}

unsafe extern "C" fn device_uuid(uuid: *mut [u8; 16], _dev: c_int) -> CuResult {
    // SAFETY: the caller passes 16 writable bytes.
    call(|_| unsafe { put(uuid, [0x5a; 16]) })
}

unsafe extern "C" fn ctx_get_current(ctx: *mut Handle) -> CuResult {
    // SAFETY: the caller passes a valid out-pointer.
    call(|_| unsafe { put(ctx, current() as Handle) })
}

unsafe extern "C" fn ctx_set_current(ctx: Handle) -> CuResult {
    call(|_| {
        CURRENT.with(|c| {
            let mut c = c.borrow_mut();
            c.pop();
            c.push(ctx as usize);
        });
        SUCCESS
    })
}

unsafe extern "C" fn ctx_get_device(dev: *mut c_int) -> CuResult {
    call(|s| match context(s) {
        // SAFETY: the caller passes a valid out-pointer.
        Ok(_) => unsafe { put(dev, 0) },
        Err(rc) => rc,
    })
}

unsafe extern "C" fn ctx_push(ctx: Handle) -> CuResult {
    call(|s| {
        if s.destroyed.contains(&(ctx as usize)) {
            s.misuse
                .push(format!("pushed destroyed context {:#x}", ctx as usize));
            return INVALID_CONTEXT;
        }
        CURRENT.with(|c| c.borrow_mut().push(ctx as usize));
        SUCCESS
    })
}

unsafe extern "C" fn ctx_pop(ctx: *mut Handle) -> CuResult {
    call(|_| {
        let Some(top) = CURRENT.with(|c| c.borrow_mut().pop()) else {
            return INVALID_CONTEXT;
        };
        // SAFETY: the caller passes a valid out-pointer.
        unsafe { put(ctx, top as Handle) }
    })
}

unsafe extern "C" fn primary_ctx_retain(ctx: *mut Handle, dev: c_int) -> CuResult {
    call(|_| {
        if dev != 0 {
            return INVALID_DEVICE;
        }
        // SAFETY: the caller passes a valid out-pointer.
        unsafe { put(ctx, PRIMARY as Handle) }
    })
}

unsafe extern "C" fn stream_create(out: *mut Handle, _flags: c_uint) -> CuResult {
    call(|s| {
        let ctx = match context(s) {
            Ok(c) => c,
            Err(rc) => return rc,
        };
        let h = id(s);
        s.streams.insert(
            h,
            FakeStream {
                ctx,
                submitted: 0,
                completed: 0,
                error: SUCCESS,
            },
        );
        // SAFETY: the caller passes a valid out-pointer.
        unsafe { put(out, h as Handle) }
    })
}

unsafe extern "C" fn stream_destroy(h: Handle) -> CuResult {
    call(|s| {
        if s.streams.remove(&(h as usize)).is_none() {
            s.misuse
                .push(format!("destroyed stream {:#x} twice", h as usize));
            return INVALID_HANDLE;
        }
        SUCCESS
    })
}

unsafe extern "C" fn stream_get_ctx(h: Handle, ctx: *mut Handle) -> CuResult {
    call(|s| match s.streams.get(&(h as usize)) {
        // SAFETY: the caller passes a valid out-pointer.
        Some(st) => unsafe { put(ctx, st.ctx as Handle) },
        None => INVALID_HANDLE,
    })
}

unsafe extern "C" fn event_create(out: *mut Handle, _flags: c_uint) -> CuResult {
    call(|s| {
        let ctx = match context(s) {
            Ok(c) => c,
            Err(rc) => return rc,
        };
        let h = id(s);
        s.events.insert(
            h,
            Event {
                ctx,
                stream: 0,
                target: 0,
            },
        );
        // SAFETY: the caller passes a valid out-pointer.
        unsafe { put(out, h as Handle) }
    })
}

unsafe extern "C" fn event_destroy(h: Handle) -> CuResult {
    call(|s| {
        if s.events.remove(&(h as usize)).is_none() {
            s.misuse
                .push(format!("destroyed event {:#x} twice", h as usize));
            return INVALID_HANDLE;
        }
        SUCCESS
    })
}

unsafe extern "C" fn event_record(ev: Handle, stream: Handle) -> CuResult {
    call(|s| {
        let Some(st) = s.streams.get(&(stream as usize)) else {
            return INVALID_HANDLE;
        };
        let (ctx, target) = (st.ctx, st.submitted);
        let Some(e) = s.events.get_mut(&(ev as usize)) else {
            return INVALID_HANDLE;
        };
        if e.ctx != ctx {
            return INVALID_HANDLE;
        }
        e.stream = stream as usize;
        e.target = target;
        SUCCESS
    })
}

/// `NOT_READY` while the work the event marks runs.
fn event_state(s: &State, ev: usize) -> CuResult {
    let Some(e) = s.events.get(&ev) else {
        return INVALID_HANDLE;
    };
    match s.streams.get(&e.stream) {
        None => SUCCESS,
        Some(st) if st.completed >= e.target => SUCCESS,
        Some(st) if st.error != SUCCESS => st.error,
        Some(_) => NOT_READY,
    }
}

unsafe extern "C" fn event_synchronize(ev: Handle) -> CuResult {
    let mut s = state();
    s.calls += 1;
    loop {
        if s.sticky != SUCCESS {
            return s.sticky;
        }
        let rc = event_state(&s, ev as usize);
        if rc != NOT_READY {
            return rc;
        }
        s = CHANGED.wait(s).unwrap_or_else(PoisonError::into_inner);
    }
}

unsafe extern "C" fn event_query(ev: Handle) -> CuResult {
    call(|s| event_state(s, ev as usize))
}

unsafe extern "C" fn stream_wait_event(stream: Handle, ev: Handle, _flags: c_uint) -> CuResult {
    call(|s| {
        if s.streams.contains_key(&(stream as usize)) && s.events.contains_key(&(ev as usize)) {
            SUCCESS
        } else {
            INVALID_HANDLE
        }
    })
}

unsafe extern "C" fn host_register(p: *mut c_void, _bytes: usize, flags: c_uint) -> CuResult {
    call(|s| {
        let ctx = match context(s) {
            Ok(c) => c,
            Err(rc) => return rc,
        };
        if s.registrations.contains_key(&(p as usize)) {
            return ALREADY_REGISTERED;
        }
        let portable = flags & MEM_HOST_REGISTER_PORTABLE != 0;
        s.registrations
            .insert(p as usize, Registration { ctx, portable });
        SUCCESS
    })
}

unsafe extern "C" fn host_unregister(p: *mut c_void) -> CuResult {
    call(|s| {
        let ctx = match context(s) {
            Ok(c) => c,
            Err(rc) => return rc,
        };
        match s.registrations.get(&(p as usize)) {
            Some(r) if r.ctx == ctx => {
                s.registrations.remove(&(p as usize));
                SUCCESS
            }
            _ => NOT_REGISTERED,
        }
    })
}

unsafe extern "C" fn host_device_pointer(
    out: *mut DevicePtr,
    p: *mut c_void,
    flags: c_uint,
) -> CuResult {
    call(|s| {
        if flags != 0 {
            return INVALID_VALUE;
        }
        let ctx = match context(s) {
            Ok(c) => c,
            Err(rc) => return rc,
        };
        match s.registrations.get(&(p as usize)) {
            Some(r) if r.portable || r.ctx == ctx => {
                // SAFETY: the caller passes a valid out-pointer.
                unsafe { put(out, p as DevicePtr) }
            }
            _ => NOT_REGISTERED,
        }
    })
}

/// # Safety
/// `p` is valid for a write.
unsafe fn put<T>(p: *mut T, v: T) -> CuResult {
    // SAFETY: as the caller vouches.
    unsafe { p.write(v) };
    SUCCESS
}

pub(super) fn driver() -> Driver {
    let core = Core {
        device_count,
        device_get,
        device_attribute,
        device_uuid,
        ctx_get_current,
        ctx_set_current,
        ctx_get_device,
        ctx_push,
        ctx_pop,
        primary_ctx_retain,
        stream_create,
        stream_destroy,
        stream_get_ctx,
        event_create,
        event_destroy,
        event_record,
        event_synchronize,
        event_query,
        stream_wait_event,
    };
    let host_reg = HostReg {
        register: host_register,
        unregister: host_unregister,
        device_pointer: host_device_pointer,
    };
    Driver::stand_in(core, host_reg)
}

fn changed(f: impl FnOnce(&mut State)) {
    f(&mut state());
    CHANGED.notify_all();
}

fn native(stream: &Stream) -> usize {
    stream.native() as usize
}

/// Queues one piece of work on `stream`, as a kernel launch would.
pub(crate) fn submit(stream: &Stream) {
    changed(|s| {
        if let Some(st) = s.streams.get_mut(&native(stream)) {
            st.submitted += 1;
        }
    });
}

/// Everything queued on `stream` so far has run.
pub(crate) fn finish(stream: &Stream) {
    changed(|s| {
        if let Some(st) = s.streams.get_mut(&native(stream)) {
            st.completed = st.submitted;
        }
    });
}

/// The work queued on `stream` and not yet finished fails with `rc`, as one launch's error does.
pub(crate) fn fail(stream: &Stream, rc: CuResult) {
    changed(|s| {
        if let Some(st) = s.streams.get_mut(&native(stream)) {
            st.error = rc;
        }
    });
}

/// Every call from now on returns `rc`, waits already in the driver too.
pub(crate) fn wedge(rc: CuResult) {
    changed(|s| s.sticky = rc);
}

/// A context of the caller's own, made current on this thread.
pub(crate) fn ctx_create() -> usize {
    let mut s = state();
    let h = id(&mut s);
    CURRENT.with(|c| c.borrow_mut().push(h));
    h
}

/// Its non-portable registrations go with it, and it is popped if current.
pub(crate) fn ctx_destroy(ctx: usize) {
    let mut s = state();
    s.destroyed.push(ctx);
    s.registrations.retain(|_, r| r.ctx != ctx || r.portable);
    CURRENT.with(|c| {
        let mut c = c.borrow_mut();
        if c.last() == Some(&ctx) {
            c.pop();
        }
    });
}

pub(crate) fn calls() -> u64 {
    state().calls
}

pub(crate) fn misuse() -> Vec<String> {
    state().misuse.clone()
}

/// Streams, events and registrations alive.
pub(crate) fn live() -> (usize, usize, usize) {
    let s = state();
    (s.streams.len(), s.events.len(), s.registrations.len())
}
