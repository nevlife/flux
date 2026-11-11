//! The CUDA driver API, opened at run time: no CUDA header, no link. The values and struct shapes
//! below are cuda.h's ABI, which no CUDA version moves. Every name is the one cuda.h's macros
//! expand to: libcuda still exports the superseded ones, and a `_v1` entry point resolves fine and
//! then truncates pointers.

use std::ffi::{CStr, c_int, c_uint, c_void};
use std::sync::OnceLock;

use crate::sys;

pub(crate) type CuResult = c_int;
pub(crate) const SUCCESS: CuResult = 0;
pub(crate) const NOT_READY: CuResult = 600;

pub(crate) const ATTR_INTEGRATED: c_int = 18;
pub(crate) const ATTR_PAGEABLE_MEMORY_ACCESS: c_int = 88;
pub(crate) const ATTR_HOST_REGISTER_SUPPORTED: c_int = 99;
pub(crate) const ATTR_VMM_SUPPORTED: c_int = 102;
pub(crate) const ATTR_POSIX_FD_HANDLE_SUPPORTED: c_int = 103;
pub(crate) const ATTR_READ_ONLY_HOST_REGISTER_SUPPORTED: c_int = 113;
pub(crate) const ATTR_HOST_NUMA_ID: c_int = 134;
pub(crate) const ATTR_HOST_NUMA_VMM_SUPPORTED: c_int = 141;

pub(crate) const EVENT_BLOCKING_SYNC: c_uint = 0x1;
pub(crate) const EVENT_DISABLE_TIMING: c_uint = 0x2;

pub(crate) const MEM_HOST_REGISTER_PORTABLE: c_uint = 0x01;
pub(crate) const MEM_HOST_REGISTER_DEVICEMAP: c_uint = 0x02;
/// Required for a `PROT_READ` mapping where the device supports it (Orin), which the driver refuses
/// with `INVALID_VALUE` otherwise. Thor does not support it and registers the mapping without it.
pub(crate) const MEM_HOST_REGISTER_READ_ONLY: c_uint = 0x08;

pub(crate) const MEM_ALLOCATION_TYPE_PINNED: c_int = 0x1;
pub(crate) const MEM_HANDLE_TYPE_POSIX_FD: c_int = 0x1;
pub(crate) const MEM_LOCATION_TYPE_DEVICE: c_int = 0x1;
pub(crate) const MEM_LOCATION_TYPE_HOST_NUMA: c_int = 0x3;
pub(crate) const MEM_ACCESS_READ: c_int = 0x1;
pub(crate) const MEM_ACCESS_READ_WRITE: c_int = 0x3;
pub(crate) const MEM_GRANULARITY_MINIMUM: c_int = 0x0;

pub(crate) type DevicePtr = u64;
pub(crate) type AllocHandle = u64;
pub(crate) type Handle = *mut c_void;

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(crate) struct MemLocation {
    pub kind: c_int,
    pub id: c_int,
}

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(crate) struct AllocFlags {
    pub compression_type: u8,
    pub gpu_direct_rdma_capable: u8,
    pub usage: u16,
    pub reserved: [u8; 4],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct MemAllocationProp {
    pub kind: c_int,
    pub requested_handle_types: c_int,
    pub location: MemLocation,
    pub win32_handle_metadata: *mut c_void,
    pub alloc_flags: AllocFlags,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct MemAccessDesc {
    pub location: MemLocation,
    pub flags: c_int,
}

const _: () = {
    assert!(size_of::<MemLocation>() == 8);
    assert!(size_of::<MemAllocationProp>() == 32);
    assert!(align_of::<MemAllocationProp>() == 8);
    assert!(std::mem::offset_of!(MemAllocationProp, location) == 8);
    assert!(std::mem::offset_of!(MemAllocationProp, win32_handle_metadata) == 16);
    assert!(std::mem::offset_of!(MemAllocationProp, alloc_flags) == 24);
    assert!(size_of::<MemAccessDesc>() == 12);
};

pub(crate) struct Core {
    pub device_count: unsafe extern "C" fn(*mut c_int) -> CuResult,
    pub device_get: unsafe extern "C" fn(*mut c_int, c_int) -> CuResult,
    pub device_attribute: unsafe extern "C" fn(*mut c_int, c_int, c_int) -> CuResult,
    pub device_uuid: unsafe extern "C" fn(*mut [u8; 16], c_int) -> CuResult,
    pub ctx_get_current: unsafe extern "C" fn(*mut Handle) -> CuResult,
    pub ctx_set_current: unsafe extern "C" fn(Handle) -> CuResult,
    pub ctx_get_device: unsafe extern "C" fn(*mut c_int) -> CuResult,
    pub ctx_push: unsafe extern "C" fn(Handle) -> CuResult,
    pub ctx_pop: unsafe extern "C" fn(*mut Handle) -> CuResult,
    pub primary_ctx_retain: unsafe extern "C" fn(*mut Handle, c_int) -> CuResult,
    pub stream_create: unsafe extern "C" fn(*mut Handle, c_uint) -> CuResult,
    pub stream_destroy: unsafe extern "C" fn(Handle) -> CuResult,
    pub stream_get_ctx: unsafe extern "C" fn(Handle, *mut Handle) -> CuResult,
    pub event_create: unsafe extern "C" fn(*mut Handle, c_uint) -> CuResult,
    pub event_destroy: unsafe extern "C" fn(Handle) -> CuResult,
    pub event_record: unsafe extern "C" fn(Handle, Handle) -> CuResult,
    pub event_synchronize: unsafe extern "C" fn(Handle) -> CuResult,
    pub event_query: unsafe extern "C" fn(Handle) -> CuResult,
    pub stream_wait_event: unsafe extern "C" fn(Handle, Handle, c_uint) -> CuResult,
}

pub(crate) struct HostReg {
    pub register: unsafe extern "C" fn(*mut c_void, usize, c_uint) -> CuResult,
    pub unregister: unsafe extern "C" fn(*mut c_void) -> CuResult,
    pub device_pointer: unsafe extern "C" fn(*mut DevicePtr, *mut c_void, c_uint) -> CuResult,
}

pub(crate) struct Vmm {
    pub granularity: unsafe extern "C" fn(*mut usize, *const MemAllocationProp, c_int) -> CuResult,
    pub create:
        unsafe extern "C" fn(*mut AllocHandle, usize, *const MemAllocationProp, u64) -> CuResult,
    pub address_reserve:
        unsafe extern "C" fn(*mut DevicePtr, usize, usize, DevicePtr, u64) -> CuResult,
    pub map: unsafe extern "C" fn(DevicePtr, usize, usize, AllocHandle, u64) -> CuResult,
    pub set_access: unsafe extern "C" fn(DevicePtr, usize, *const MemAccessDesc, usize) -> CuResult,
    pub export: unsafe extern "C" fn(*mut c_void, AllocHandle, c_int, u64) -> CuResult,
    pub import: unsafe extern "C" fn(*mut AllocHandle, *mut c_void, c_int) -> CuResult,
    pub unmap: unsafe extern "C" fn(DevicePtr, usize) -> CuResult,
    pub address_free: unsafe extern "C" fn(DevicePtr, usize) -> CuResult,
    pub release: unsafe extern "C" fn(AllocHandle) -> CuResult,
}

/// Copies between host and device memory, queued on a stream.
pub(crate) struct Copies {
    pub htod: unsafe extern "C" fn(DevicePtr, *const c_void, usize, Handle) -> CuResult,
    pub dtoh: unsafe extern "C" fn(*mut c_void, DevicePtr, usize, Handle) -> CuResult,
}

/// Groups load apart: a driver without host registration or VMM still serves the other routes.
pub(crate) struct Driver {
    #[cfg_attr(not(test), allow(dead_code))]
    lib: usize,
    pub core: Result<Core, &'static str>,
    pub host_reg: Result<HostReg, &'static str>,
    pub vmm: Result<Vmm, &'static str>,
    pub copies: Result<Copies, &'static str>,
    born: u32,
}

/// # Safety
/// `T` must be the function pointer type of `name`'s C signature.
unsafe fn sym<T: Copy>(lib: usize, name: &CStr) -> Option<T> {
    const { assert!(size_of::<T>() == size_of::<*mut c_void>()) };
    // SAFETY: lib is a live dlopen handle; name is NUL-terminated.
    let p = unsafe { libc::dlsym(lib as *mut c_void, name.as_ptr()) };
    if p.is_null() {
        return None;
    }
    // SAFETY: the caller names the pointer's type; sizes match by the assert above.
    Some(unsafe { std::mem::transmute_copy::<*mut c_void, T>(&p) })
}

macro_rules! load {
    ($lib:expr, $ty:ident { $($field:ident = $name:literal),+ $(,)? }) => {
        (|| {
            Some($ty {
                $(
                    // SAFETY: each field's type is the C signature of the named entry point.
                    $field: unsafe { sym($lib, $name)? },
                )+
            })
        })()
    };
}

impl Driver {
    /// `cuInit` runs once per process and costs hundreds of milliseconds. The library is never
    /// closed: unloading a driver this process initialized would pull it from under any CUDA the
    /// application loads later.
    pub(crate) fn get() -> &'static Driver {
        static DRIVER: OnceLock<Driver> = OnceLock::new();
        DRIVER.get_or_init(Driver::load)
    }

    fn load() -> Driver {
        sys::watch_fork();
        #[cfg(test)]
        if std::env::var_os(super::fake::ENV).is_some() {
            return super::fake::driver();
        }
        let unloaded = |why| Driver {
            lib: 0,
            core: Err(why),
            host_reg: Err(why),
            vmm: Err(why),
            copies: Err(why),
            born: sys::fork_gen(),
        };
        // SAFETY: a constant NUL-terminated name.
        let lib =
            unsafe { libc::dlopen(c"libcuda.so.1".as_ptr(), libc::RTLD_LAZY | libc::RTLD_LOCAL) };
        if lib.is_null() {
            return unloaded("libcuda.so.1 is not loadable");
        }
        let lib = lib as usize;
        // SAFETY: cuInit's signature.
        let Some(init) =
            (unsafe { sym::<unsafe extern "C" fn(c_uint) -> CuResult>(lib, c"cuInit") })
        else {
            return unloaded("libcuda.so.1 has no cuInit");
        };
        let Some(core) = Driver::core(lib) else {
            return unloaded("libcuda.so.1 is missing an entry point flux needs");
        };
        // SAFETY: cuInit takes flags, which must be 0.
        if unsafe { init(0) } != SUCCESS {
            return unloaded("cuInit failed");
        }
        Driver {
            lib,
            core: Ok(core),
            host_reg: Driver::host_reg(lib).ok_or("libcuda.so.1 has no host registration"),
            vmm: Driver::vmm(lib).ok_or("libcuda.so.1 has no virtual memory management"),
            copies: Driver::copies(lib).ok_or("libcuda.so.1 has no asynchronous copies"),
            born: sys::fork_gen(),
        }
    }

    /// The driver API, unless this process is a child forked after `cuInit`, where CUDA answers
    /// only with errors that name some other cause.
    pub(crate) fn usable_core(&self) -> Result<&Core, &'static str> {
        if sys::fork_gen() != self.born {
            return Err(
                "CUDA was initialized before this process was forked, and a forked child \
                        cannot use it; create GPU channels after the fork or in an exec'd process",
            );
        }
        self.core.as_ref().map_err(|why| *why)
    }

    fn core(lib: usize) -> Option<Core> {
        load!(lib, Core {
            device_count = c"cuDeviceGetCount",
            device_get = c"cuDeviceGet",
            device_attribute = c"cuDeviceGetAttribute",
            device_uuid = c"cuDeviceGetUuid_v2",
            ctx_get_current = c"cuCtxGetCurrent",
            ctx_set_current = c"cuCtxSetCurrent",
            ctx_get_device = c"cuCtxGetDevice",
            ctx_push = c"cuCtxPushCurrent_v2",
            ctx_pop = c"cuCtxPopCurrent_v2",
            primary_ctx_retain = c"cuDevicePrimaryCtxRetain",
            stream_create = c"cuStreamCreate",
            stream_destroy = c"cuStreamDestroy_v2",
            stream_get_ctx = c"cuStreamGetCtx",
            event_create = c"cuEventCreate",
            event_destroy = c"cuEventDestroy_v2",
            event_record = c"cuEventRecord",
            event_synchronize = c"cuEventSynchronize",
            event_query = c"cuEventQuery",
            stream_wait_event = c"cuStreamWaitEvent",
        })
    }

    fn host_reg(lib: usize) -> Option<HostReg> {
        load!(lib, HostReg {
            register = c"cuMemHostRegister_v2",
            unregister = c"cuMemHostUnregister",
            device_pointer = c"cuMemHostGetDevicePointer_v2",
        })
    }

    fn vmm(lib: usize) -> Option<Vmm> {
        load!(lib, Vmm {
            granularity = c"cuMemGetAllocationGranularity",
            create = c"cuMemCreate",
            address_reserve = c"cuMemAddressReserve",
            map = c"cuMemMap",
            set_access = c"cuMemSetAccess",
            export = c"cuMemExportToShareableHandle",
            import = c"cuMemImportFromShareableHandle",
            unmap = c"cuMemUnmap",
            address_free = c"cuMemAddressFree",
            release = c"cuMemRelease",
        })
    }

    fn copies(lib: usize) -> Option<Copies> {
        load!(lib, Copies {
            htod = c"cuMemcpyHtoDAsync_v2",
            dtoh = c"cuMemcpyDtoHAsync_v2",
        })
    }

    /// The test stand-in's driver, which has no VMM.
    #[cfg(test)]
    pub(crate) fn stand_in(core: Core, host_reg: HostReg) -> Driver {
        Driver {
            lib: 0,
            core: Ok(core),
            host_reg: Ok(host_reg),
            vmm: Err("the stand-in driver has no virtual memory management"),
            copies: Err("the stand-in driver has no copies"),
            born: sys::fork_gen(),
        }
    }

    /// An entry point outside the groups above, for tests that drive the GPU themselves.
    ///
    /// # Safety
    /// As `sym`.
    #[cfg(test)]
    pub(crate) unsafe fn extra<T: Copy>(&self, name: &CStr) -> Option<T> {
        if self.lib == 0 {
            return None;
        }
        // SAFETY: forwarded.
        unsafe { sym(self.lib, name) }
    }
}

impl Core {
    /// A refused query reads as unsupported: a driver dropping one attribute should not strand a
    /// host that works.
    pub(crate) fn attribute(&self, device: c_int, attr: c_int) -> bool {
        let mut v = 0;
        // SAFETY: v is a valid out-pointer.
        unsafe { (self.device_attribute)(&raw mut v, attr, device) == SUCCESS && v != 0 }
    }

    /// The attribute's value; None for a refused query.
    pub(crate) fn attribute_value(&self, device: c_int, attr: c_int) -> Option<c_int> {
        let mut v = 0;
        // SAFETY: v is a valid out-pointer.
        (unsafe { (self.device_attribute)(&raw mut v, attr, device) } == SUCCESS).then_some(v)
    }

    pub(crate) fn current_context(&self) -> Option<Handle> {
        let mut ctx = std::ptr::null_mut();
        // SAFETY: ctx is a valid out-pointer.
        let ok = unsafe { (self.ctx_get_current)(&raw mut ctx) } == SUCCESS;
        (ok && !ctx.is_null()).then_some(ctx)
    }

    /// The ordinal of the current context's device, or `None` without a current context.
    pub(crate) fn current_device(&self) -> Option<c_int> {
        self.current_context()?;
        let mut dev = 0;
        // SAFETY: dev is a valid out-pointer.
        (unsafe { (self.ctx_get_device)(&raw mut dev) } == SUCCESS).then_some(dev)
    }

    /// The current context, or the primary context of `ordinal` made current: what the runtime
    /// does on its first use, so a caller that uses the runtime later lands on the same context.
    pub(crate) fn ensure_context(&self, ordinal: c_int) -> Option<Handle> {
        if let Some(ctx) = self.current_context() {
            return Some(ctx);
        }
        let ctx = self.primary_context(ordinal)?;
        // SAFETY: a context retained by primary_context.
        (unsafe { (self.ctx_set_current)(ctx) } == SUCCESS).then_some(ctx)
    }

    /// Retained and never released, like the runtime's, so it outlives anything made in it
    /// whatever the caller does with its own contexts.
    pub(crate) fn primary_context(&self, ordinal: c_int) -> Option<Handle> {
        let mut dev = 0;
        // SAFETY: dev is a valid out-pointer.
        if unsafe { (self.device_get)(&raw mut dev, ordinal) } != SUCCESS {
            return None;
        }
        let mut ctx = std::ptr::null_mut();
        // SAFETY: ctx is a valid out-pointer.
        (unsafe { (self.primary_ctx_retain)(&raw mut ctx, dev) } == SUCCESS).then_some(ctx)
    }

    /// Runs `f` with `ctx` current on this thread and restores the thread's own after.
    pub(crate) fn with_context<R>(&self, ctx: Handle, f: impl FnOnce() -> R) -> R {
        // SAFETY: ctx is a context handle this process holds.
        let pushed = unsafe { (self.ctx_push)(ctx) } == SUCCESS;
        let r = f();
        if pushed {
            let mut popped = std::ptr::null_mut();
            // SAFETY: pops what was pushed above.
            unsafe { (self.ctx_pop)(&raw mut popped) };
        }
        r
    }
}
