//! The process arena through the C ABI: a loan's handle on it, a view's mapping of it, the GPU
//! arena of both, and the malloc hook's scope.

use std::ffi::c_void;
use std::sync::Arc;

use flux_core::{Frame, GpuBlock, GpuHeap, Heap, Loan};

use crate::channel::{LoanCell, bytes, flux_frame, flux_loan, view_ref};

/// A loan's arena: `channel` null for none, `base` null until made. A claim frees what it gave.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct flux_arena {
    pub base: *mut u8,
    pub size: usize,
    pub channel: *const c_void,
    pub slot: u32,
    pub gpu: bool,
}

impl flux_arena {
    fn core(&self) -> Option<Heap> {
        (!self.channel.is_null())
            .then(|| Heap::from_raw(self.channel, self.slot, self.base, self.size, self.gpu))
    }
}

/// A frame's fields across `PCIe`: the GPU arena's offset 0 (0 for none) and reach, the frame's
/// device address, and this subscriber's copies of `count` blocks (`FLUX_GPU_ARENA_POS`).
#[repr(C)]
pub struct flux_gpu_arena {
    pub device: u64,
    pub host: *const u8,
    pub size: usize,
    pub frame: u64,
    pub blocks: *const flux_gpu_block,
    pub count: usize,
}

#[repr(C)]
pub struct flux_gpu_block {
    pub off: u64,
    pub len: u64,
    pub host: *const u8,
    pub device: u64,
}

const _: () = {
    assert!(size_of::<flux_gpu_block>() == size_of::<GpuBlock>());
    assert!(std::mem::offset_of!(flux_gpu_block, host) == std::mem::offset_of!(GpuBlock, host));
    assert!(std::mem::offset_of!(flux_gpu_block, device) == std::mem::offset_of!(GpuBlock, device));
};

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_arena(l: *const flux_loan) -> flux_arena {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above).
    let h = unsafe { &*l.cast::<LoanCell>() }
        .slot
        .get()
        .and_then(Loan::heap);
    h.map_or(
        flux_arena {
            base: std::ptr::null_mut(),
            size: 0,
            channel: std::ptr::null(),
            slot: 0,
            gpu: false,
        },
        |h| {
            let (channel, slot) = h.raw();
            flux_arena {
                base: h.base,
                size: h.size,
                channel,
                slot,
                gpu: h.gpu,
            }
        },
    )
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_arena_alloc(
    h: *const flux_arena,
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
pub unsafe extern "C" fn flux_arena_alloc_device(
    h: *const flux_arena,
    bytes: usize,
    align: usize,
    off: *mut u64,
) -> u64 {
    // SAFETY: h is valid; its loan is live (the caller's contract).
    let Some((p, o)) = unsafe { &*h }
        .core()
        .and_then(|h| unsafe { h.alloc_device(bytes, align) })
    else {
        return 0;
    };
    // SAFETY: off is valid for a write.
    unsafe { off.write(o) };
    p
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_arena_adopt(
    h: *const flux_arena,
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

/// The arena the frame names, read-only while the frame lives; null for none.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_arena(v: *const flux_frame, size: *mut usize) -> *const u8 {
    // SAFETY: valid view storage.
    let Some(h) = unsafe { view_ref(v) }.get().and_then(Frame::heap) else {
        return std::ptr::null();
    };
    // SAFETY: size is valid for a write.
    unsafe { size.write(h.len()) };
    h.as_ptr()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_gpu_arena(
    v: *const flux_frame,
    out: *mut flux_gpu_arena,
) -> bool {
    // SAFETY: valid view storage.
    let Some(GpuHeap {
        device,
        host,
        size,
        frame,
        blocks,
        count,
    }) = unsafe { view_ref(v) }.get().and_then(Frame::gpu_heap)
    else {
        return false;
    };
    let blocks = blocks.cast::<flux_gpu_block>();
    // SAFETY: out is valid for a write.
    unsafe {
        out.write(flux_gpu_arena {
            device,
            host,
            size,
            frame,
            blocks,
            count,
        });
    };
    true
}

/// Maps the arena frame bytes name, for a reader without a frame, until `flux_arena_release`.
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
