//! The process arena through the C ABI: a loan's handle on it, a view's mapping of it, and the
//! malloc hook's scope.

use std::ffi::c_void;
use std::sync::Arc;

use flux_core::{FrameView, Heap, WriteSlot};

use crate::channel::{LoanCell, bytes, flux_loan, flux_view, view_ref};

/// A live loan's arena (`flux_loan_heap`): `channel` null for none, `base` null until made.
/// `flux_heap_alloc` gives the frame arena memory, `flux_heap_adopt` the hook's; a claim frees it.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct flux_heap {
    pub base: *mut u8,
    pub size: usize,
    pub channel: *const c_void,
    pub slot: u32,
}

impl flux_heap {
    fn core(&self) -> Option<Heap> {
        (!self.channel.is_null())
            .then(|| Heap::from_raw(self.channel, self.slot, self.base, self.size))
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_heap(l: *const flux_loan) -> flux_heap {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above).
    let h = unsafe { &*l.cast::<LoanCell>() }
        .slot
        .get()
        .and_then(WriteSlot::heap);
    h.map_or(
        flux_heap {
            base: std::ptr::null_mut(),
            size: 0,
            channel: std::ptr::null(),
            slot: 0,
        },
        |h| {
            let (channel, slot) = h.raw();
            flux_heap {
                base: h.base,
                size: h.size,
                channel,
                slot,
            }
        },
    )
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_heap_alloc(
    h: *const flux_heap,
    bytes: usize,
    align: usize,
    off: *mut u64,
) -> *mut u8 {
    // SAFETY: h is valid; its loan is live (the caller's contract).
    let Some((p, o)) = unsafe { &*h }
        .core()
        .and_then(|h| unsafe { h.alloc(bytes, align) })
    else {
        return std::ptr::null_mut();
    };
    // SAFETY: off is valid for a write.
    unsafe { off.write(o) };
    p
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_heap_adopt(
    h: *const flux_heap,
    p: *const u8,
    bytes: usize,
    release: unsafe extern "C" fn(*mut c_void),
    ctx: *mut c_void,
    off: *mut u64,
) -> bool {
    // SAFETY: h is valid; its loan is live and release is as heap_adopt asks (the caller's
    // contract).
    let Some(o) = unsafe { &*h }
        .core()
        .and_then(|h| unsafe { h.adopt(p, bytes, release, ctx) })
    else {
        return false;
    };
    // SAFETY: off is valid for a write.
    unsafe { off.write(o) };
    true
}

/// The arena the view's frame names, read-only while the view lives; null for none.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_view_heap(v: *const flux_view, size: *mut usize) -> *const u8 {
    // SAFETY: valid view storage.
    let Some(h) = unsafe { view_ref(v) }.get().and_then(FrameView::heap) else {
        return std::ptr::null();
    };
    // SAFETY: size is valid for a write.
    unsafe { size.write(h.len()) };
    h.as_ptr()
}

/// Maps the arena frame bytes name, for a reader without a view, until `flux_arena_release`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_arena_acquire(
    frame: *const u8,
    len: usize,
    base: *mut *const u8,
    size: *mut usize,
) -> *const c_void {
    // SAFETY: the caller passes len readable bytes.
    let Some(m) = flux_core::arena::map_for(unsafe { bytes(frame, len) }) else {
        return std::ptr::null();
    };
    // SAFETY: base is valid for a write.
    unsafe { base.write(m.bytes().as_ptr()) };
    // SAFETY: size is valid for a write.
    unsafe { size.write(m.bytes().len()) };
    Arc::into_raw(m).cast()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_arena_release(handle: *const c_void) {
    if !handle.is_null() {
        // SAFETY: a handle flux_arena_acquire returned, released once.
        drop(unsafe { Arc::from_raw(handle.cast::<flux_core::arena::ArenaMap>()) });
    }
}

/// Routes this thread's allocations to the arena under the hook until `flux_arena_scope_leave`.
#[unsafe(no_mangle)]
pub extern "C" fn flux_arena_scope_enter() -> bool {
    flux_core::arena::scope_enter()
}

#[unsafe(no_mangle)]
pub extern "C" fn flux_arena_scope_leave() {
    flux_core::arena::scope_leave();
}
