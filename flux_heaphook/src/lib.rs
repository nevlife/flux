//! `LD_PRELOAD=libflux_heaphook.so` routes a thread's allocations to the flux
//! arena while it has a scope open (`flux_arena_scope_enter`). A buffer a library fills in that
//! scope is arena memory, so a frame can name it by offset instead of copying it.
//!
//! flux hands over its allocator with `flux_heaphook_attach` when it makes the arena. Until then,
//! and outside a scope, everything goes to glibc. Arena memory is freed into the arena on any
//! thread. Nothing here allocates.

#![allow(
    clippy::disallowed_types,
    reason = "the hook's addresses and counters, Relaxed only"
)]
#![allow(clippy::missing_safety_doc)]

use std::cell::Cell;
use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

unsafe extern "C" {
    fn __libc_malloc(n: usize) -> *mut c_void;
    fn __libc_free(p: *mut c_void);
    fn __libc_calloc(n: usize, m: usize) -> *mut c_void;
    fn __libc_realloc(p: *mut c_void, n: usize) -> *mut c_void;
    fn __libc_memalign(align: usize, n: usize) -> *mut c_void;
}

type AllocFn = unsafe extern "C" fn(usize, usize) -> *mut c_void;
type FreeFn = unsafe extern "C" fn(*mut c_void);
type UsableFn = unsafe extern "C" fn(*mut c_void) -> usize;

static ALLOC: AtomicUsize = AtomicUsize::new(0);
static FREE: AtomicUsize = AtomicUsize::new(0);
static USABLE: AtomicUsize = AtomicUsize::new(0);
static BASE: AtomicUsize = AtomicUsize::new(0);
static SIZE: AtomicUsize = AtomicUsize::new(0);
/// A forked child has its parent's arena mapping but not its allocator state: it leaves both alone.
static FORKED: AtomicBool = AtomicBool::new(false);

thread_local! {
    static DEPTH: Cell<u32> = const { Cell::new(0) };
}

const MIN_ALIGN: usize = 16;

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_heaphook_attach(
    alloc: AllocFn,
    free: FreeFn,
    usable: UsableFn,
    base: *mut c_void,
    size: usize,
) {
    extern "C" fn child() {
        FORKED.store(true, Ordering::Relaxed);
    }
    BASE.store(base as usize, Ordering::Relaxed);
    SIZE.store(size, Ordering::Relaxed);
    FREE.store(free as usize, Ordering::Relaxed);
    USABLE.store(usable as usize, Ordering::Relaxed);
    ALLOC.store(alloc as usize, Ordering::Release); // order: hook_alloc
    // SAFETY: registers a plain function.
    unsafe { libc::pthread_atfork(None, None, Some(child)) };
}

#[unsafe(no_mangle)]
pub extern "C" fn flux_heaphook_enter() {
    DEPTH.with(|d| d.set(d.get() + 1));
}

#[unsafe(no_mangle)]
pub extern "C" fn flux_heaphook_leave() {
    DEPTH.with(|d| d.set(d.get().saturating_sub(1)));
}

fn in_arena(p: *const c_void) -> bool {
    let off = (p as usize).wrapping_sub(BASE.load(Ordering::Relaxed));
    off < SIZE.load(Ordering::Relaxed)
}

/// The arena allocator, when this thread is in a scope and flux attached one.
fn scoped() -> Option<AllocFn> {
    let f = ALLOC.load(Ordering::Acquire); // order: hook_alloc
    if f == 0 || FORKED.load(Ordering::Relaxed) || DEPTH.with(Cell::get) == 0 {
        return None;
    }
    // SAFETY: stored from an AllocFn.
    Some(unsafe { std::mem::transmute::<usize, AllocFn>(f) })
}

/// Arena memory, or null to fall back to glibc: a full arena must not fail unrelated code.
fn arena_alloc(n: usize, align: usize) -> *mut c_void {
    scoped().map_or(std::ptr::null_mut(), |f| {
        // SAFETY: flux's allocator takes any size and a power-of-two alignment.
        unsafe { f(n, align.max(MIN_ALIGN)) }
    })
}

fn arena_free(p: *mut c_void) {
    if FORKED.load(Ordering::Relaxed) {
        return;
    }
    // SAFETY: attached before any arena pointer existed; stored from a FreeFn.
    let f = unsafe { std::mem::transmute::<usize, FreeFn>(FREE.load(Ordering::Relaxed)) };
    // SAFETY: p is arena memory.
    unsafe { f(p) };
}

fn arena_usable(p: *mut c_void) -> usize {
    // SAFETY: as arena_free.
    let f = unsafe { std::mem::transmute::<usize, UsableFn>(USABLE.load(Ordering::Relaxed)) };
    // SAFETY: p is arena memory.
    unsafe { f(p) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn malloc(n: usize) -> *mut c_void {
    let p = arena_alloc(n, MIN_ALIGN);
    if p.is_null() {
        // SAFETY: glibc's own entry point.
        unsafe { __libc_malloc(n) }
    } else {
        p
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn free(p: *mut c_void) {
    if p.is_null() {
        return;
    }
    if in_arena(p) {
        arena_free(p);
    } else {
        // SAFETY: glibc's pointer.
        unsafe { __libc_free(p) };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn calloc(n: usize, m: usize) -> *mut c_void {
    let Some(bytes) = n.checked_mul(m) else {
        return std::ptr::null_mut();
    };
    let p = arena_alloc(bytes, MIN_ALIGN);
    if p.is_null() {
        // SAFETY: glibc's own entry point.
        return unsafe { __libc_calloc(n, m) };
    }
    // SAFETY: p has bytes writable bytes; arena memory may hold an old frame.
    unsafe { std::ptr::write_bytes(p.cast::<u8>(), 0, bytes) };
    p
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn realloc(p: *mut c_void, n: usize) -> *mut c_void {
    if p.is_null() {
        // SAFETY: as malloc.
        return unsafe { malloc(n) };
    }
    if n == 0 {
        // SAFETY: as free.
        unsafe { free(p) };
        return std::ptr::null_mut();
    }
    let from_arena = in_arena(p);
    if !from_arena && scoped().is_none() {
        // SAFETY: glibc's pointer.
        return unsafe { __libc_realloc(p, n) };
    }
    // Growing inside a scope moves the buffer into the arena; an arena buffer stays there.
    let mut q = arena_alloc(n, MIN_ALIGN);
    if q.is_null() {
        // SAFETY: glibc's own entry point.
        q = unsafe { __libc_malloc(n) };
        if q.is_null() {
            return q;
        }
    }
    let old = if from_arena {
        arena_usable(p)
    } else {
        // SAFETY: glibc's pointer.
        unsafe { malloc_usable_size(p) }
    };
    // SAFETY: both blocks hold at least the bytes copied.
    unsafe { std::ptr::copy_nonoverlapping(p.cast::<u8>(), q.cast::<u8>(), old.min(n)) };
    // SAFETY: p is no longer used.
    unsafe { free(p) };
    q
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn memalign(align: usize, n: usize) -> *mut c_void {
    let p = arena_alloc(n, align);
    if p.is_null() {
        // SAFETY: glibc's own entry point.
        unsafe { __libc_memalign(align, n) }
    } else {
        p
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn aligned_alloc(align: usize, n: usize) -> *mut c_void {
    if !align.is_power_of_two() {
        return std::ptr::null_mut();
    }
    // SAFETY: as memalign.
    unsafe { memalign(align, n) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn posix_memalign(out: *mut *mut c_void, align: usize, n: usize) -> i32 {
    if !align.is_power_of_two() || !align.is_multiple_of(size_of::<*mut c_void>()) {
        return libc::EINVAL;
    }
    // SAFETY: as memalign.
    let p = unsafe { memalign(align, n) };
    if p.is_null() {
        return libc::ENOMEM;
    }
    // SAFETY: out is valid for a write.
    unsafe { out.write(p) };
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn malloc_usable_size(p: *mut c_void) -> usize {
    static ORIGINAL: AtomicUsize = AtomicUsize::new(0);
    if p.is_null() {
        return 0;
    }
    if in_arena(p) {
        return arena_usable(p);
    }
    let mut f = ORIGINAL.load(Ordering::Relaxed);
    if f == 0 {
        // SAFETY: dlsym with a valid C string.
        f = unsafe { libc::dlsym(libc::RTLD_NEXT, c"malloc_usable_size".as_ptr()) } as usize;
        ORIGINAL.store(f, Ordering::Relaxed);
    }
    // SAFETY: glibc's malloc_usable_size.
    let original = unsafe { std::mem::transmute::<usize, UsableFn>(f) };
    // SAFETY: glibc's pointer.
    unsafe { original(p) }
}
