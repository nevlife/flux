use std::ffi::c_void;

use flux_core::gpu::{self, Route, Stream};

use crate::status::{flux_status, status, status_of};

/// A CUDA stream declared on channels; NULL where a function takes one means a host channel.
pub struct flux_stream {
    pub(crate) s: Stream,
}

#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum flux_route {
    FLUX_ROUTE_NONE = 0,
    FLUX_ROUTE_SHM_DIRECT = 1,
    FLUX_ROUTE_SHM_REGISTERED = 2,
    FLUX_ROUTE_DEVICE_HANDLE = 3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct flux_gpu_platform {
    pub route: flux_route,
    pub device: i32,
    pub integrated: bool,
    pub pageable_access: bool,
    pub host_register: bool,
    pub vmm: bool,
    pub posix_fd_handle: bool,
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_stream_create(out: *mut *mut flux_stream) -> flux_status {
    status(Stream::create().map(|s| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(Box::into_raw(Box::new(flux_stream { s }))) };
    }))
}

/// The caller's `CUstream`, outliving its channels and views; NULL is `FLUX_INVALID_ARGUMENT`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_stream_from_native(
    native: *mut c_void,
    out: *mut *mut flux_stream,
) -> flux_status {
    // SAFETY: the caller vouches for native.
    status(unsafe { Stream::from_native(native) }.map(|s| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(Box::into_raw(Box::new(flux_stream { s }))) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_stream_free(s: *mut flux_stream) {
    if !s.is_null() {
        // SAFETY: a handle from a constructor above, freed once.
        drop(unsafe { Box::from_raw(s) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_stream_native(s: *const flux_stream) -> *mut c_void {
    // SAFETY: valid handle.
    unsafe { &(*s).s }.native()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_stream_device(s: *const flux_stream) -> i32 {
    // SAFETY: valid handle.
    unsafe { &(*s).s }.device()
}

/// Until all work queued on the stream so far has run; false if the wait failed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_stream_wait(s: *const flux_stream) -> bool {
    // SAFETY: valid handle.
    unsafe { &(*s).s }.wait()
}

/// Fills `out` always; `FLUX_UNSUPPORTED` (reason in `flux_last_error()`) for no route.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_gpu_probe(device: i32, out: *mut flux_gpu_platform) -> flux_status {
    let p = gpu::probe(device);
    let route = match p.route {
        Route::None => flux_route::FLUX_ROUTE_NONE,
        Route::ShmDirect => flux_route::FLUX_ROUTE_SHM_DIRECT,
        Route::ShmRegistered => flux_route::FLUX_ROUTE_SHM_REGISTERED,
        Route::DeviceHandle => flux_route::FLUX_ROUTE_DEVICE_HANDLE,
    };
    // SAFETY: out is valid for a write.
    unsafe {
        out.write(flux_gpu_platform {
            route,
            device: p.device,
            integrated: p.integrated,
            pageable_access: p.pageable_access,
            host_register: p.host_register,
            vmm: p.vmm,
            posix_fd_handle: p.posix_fd_handle,
        });
    }
    if p.route == Route::None {
        return status_of(&flux_core::Error::Unsupported(p.reason));
    }
    flux_status::FLUX_OK
}

/// The sticky `CUresult` after which every GPU channel refuses until a restart; 0 while none.
#[unsafe(no_mangle)]
pub extern "C" fn flux_gpu_wedged() -> i32 {
    gpu::wedged().unwrap_or(0)
}

#[unsafe(no_mangle)]
pub extern "C" fn flux_gpu_current_device() -> i32 {
    gpu::current_device()
}

pub(crate) unsafe fn stream_of(s: *const flux_stream) -> Stream {
    if s.is_null() {
        return Stream::default();
    }
    // SAFETY: a live handle.
    unsafe { &(*s).s }.clone()
}
