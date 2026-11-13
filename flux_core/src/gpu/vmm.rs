//! The device-handle route: one VMM allocation for the whole payload area, exported once as a
//! POSIX fd and handed to each subscriber over an `AF_UNIX` socket file beside the segment, one
//! accept thread per channel. Both ends also check that the peer is this user before the
//! descriptor moves.

use std::ffi::{c_int, c_void};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Mutex, PoisonError};

use super::driver::{
    AllocFlags, AllocHandle, Core, DevicePtr, Driver, MEM_ACCESS_READ, MEM_ACCESS_READ_WRITE,
    MEM_ALLOCATION_TYPE_PINNED, MEM_GRANULARITY_MINIMUM, MEM_HANDLE_TYPE_POSIX_FD,
    MEM_LOCATION_TYPE_DEVICE, MemAccessDesc, MemAllocationProp, MemLocation, SUCCESS, Vmm,
};
use super::ledger::{self, Held};
use crate::error::{Error, Result};
use crate::segment::SegmentId;
use crate::sys::{self};

use super::fd::{ENDPOINT_PREFIX, FdServer, Share, receive};

/// In /dev/shm, so whoever sees the segment sees its endpoint: an abstract name would follow the
/// network namespace instead. Derived from the segment object, so a restarted publisher's new
/// segment has a new endpoint.
pub(crate) fn endpoint(id: SegmentId) -> String {
    format!("/dev/shm/{ENDPOINT_PREFIX}{}.{}", id.dev, id.ino)
}

pub(crate) fn vmm() -> Result<(&'static Core, &'static Vmm)> {
    let drv = Driver::get();
    let core = drv
        .usable_core()
        .map_err(|why| Error::Unsupported(why.to_owned()))?;
    let vmm = drv
        .vmm
        .as_ref()
        .map_err(|why| Error::Unsupported((*why).to_owned()))?;
    Ok((core, vmm))
}

pub(crate) fn device_handle(core: &Core, ordinal: i32) -> Option<c_int> {
    let mut h = 0;
    // SAFETY: h is a valid out-pointer.
    (unsafe { (core.device_get)(&raw mut h, ordinal) } == SUCCESS).then_some(h)
}

pub(crate) fn device_uuid(ordinal: i32) -> Option<[u8; 16]> {
    let core = Driver::get().core.as_ref().ok()?;
    let h = device_handle(core, ordinal)?;
    let mut uuid = [0u8; 16];
    // SAFETY: uuid is 16 writable bytes, a CUuuid.
    (unsafe { (core.device_uuid)(&raw mut uuid, h) } == SUCCESS).then_some(uuid)
}

/// This process's ordinal for the device the publisher allocated on: ordinals differ between
/// processes that see different `CUDA_VISIBLE_DEVICES`, and importing on the wrong device fails or
/// reads another GPU's memory.
pub(crate) fn device_for_uuid(uuid: &[u8; 16]) -> Option<i32> {
    #[cfg(test)]
    if let Some(d) = super::tests::UUID_DEVICE.get() {
        return Some(d);
    }
    let core = Driver::get().core.as_ref().ok()?;
    let mut count = 0;
    // SAFETY: count is a valid out-pointer.
    if unsafe { (core.device_count)(&raw mut count) } != SUCCESS {
        return None;
    }
    (0..count).find(|&d| device_uuid(d).as_ref() == Some(uuid))
}

fn pinned_on(device: c_int) -> MemAllocationProp {
    MemAllocationProp {
        kind: MEM_ALLOCATION_TYPE_PINNED,
        requested_handle_types: MEM_HANDLE_TYPE_POSIX_FD,
        location: MemLocation {
            kind: MEM_LOCATION_TYPE_DEVICE,
            id: device,
        },
        win32_handle_metadata: std::ptr::null_mut(),
        alloc_flags: AllocFlags::default(),
    }
}

/// The least a subscriber's device copy backs at once: rounded up to the device's granularity.
const CHUNK_BYTES: usize = 2 << 20;

/// The minimum granularity binds the allocation's size only, not slot offsets: `PAYLOAD_ALIGN`
/// and the layout arithmetic stay as they are.
fn rounded(vmm: &Vmm, device: c_int, bytes: usize) -> Result<usize> {
    let prop = pinned_on(device);
    let mut g = 0;
    // SAFETY: valid pointers.
    if unsafe { (vmm.granularity)(&raw mut g, &raw const prop, MEM_GRANULARITY_MINIMUM) } != SUCCESS
        || g == 0
    {
        return Err(Error::Unsupported(
            "cuMemGetAllocationGranularity failed".into(),
        ));
    }
    Ok(bytes.div_ceil(g) * g)
}

/// One allocation, mapped. Comes apart in the order it was built: unmap, free the range, release.
struct Region {
    addr: DevicePtr,
    alloc: AllocHandle,
    size: usize,
    ctx: usize,
    born: u32,
}

impl Region {
    fn new(ctx: *mut c_void, size: usize) -> Region {
        Region {
            addr: 0,
            alloc: 0,
            size,
            ctx: ctx as usize,
            born: sys::fork_gen(),
        }
    }

    /// Reserves a range for `alloc` and lets `device` reach it with `access`.
    fn map(&mut self, vmm: &Vmm, device: c_int, access: c_int) -> Result<()> {
        // SAFETY: valid out-pointer; no fixed address, default alignment.
        if unsafe { (vmm.address_reserve)(&raw mut self.addr, self.size, 0, 0, 0) } != SUCCESS {
            self.addr = 0;
            return Err(Error::Unsupported("cuMemAddressReserve failed".into()));
        }
        // SAFETY: the range was reserved above at the allocation's size.
        if unsafe { (vmm.map)(self.addr, self.size, 0, self.alloc, 0) } != SUCCESS {
            // SAFETY: reserved above, nothing mapped.
            unsafe { (vmm.address_free)(self.addr, self.size) };
            self.addr = 0;
            return Err(Error::Unsupported("cuMemMap failed".into()));
        }
        let desc = MemAccessDesc {
            location: MemLocation {
                kind: MEM_LOCATION_TYPE_DEVICE,
                id: device,
            },
            flags: access,
        };
        // SAFETY: a mapped range and one valid descriptor.
        if unsafe { (vmm.set_access)(self.addr, self.size, &raw const desc, 1) } != SUCCESS {
            return Err(Error::Unsupported("cuMemSetAccess failed".into()));
        }
        Ok(())
    }
}

impl Drop for Region {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let Ok((core, vmm)) = vmm() else {
            return;
        };
        core.with_context(self.ctx as *mut c_void, || {
            // Each object this region made, released once, in reverse order.
            if self.addr != 0 {
                // SAFETY: mapped by map() at this size.
                unsafe { (vmm.unmap)(self.addr, self.size) };
                // SAFETY: reserved by map() at this size, now unmapped.
                unsafe { (vmm.address_free)(self.addr, self.size) };
            }
            if self.alloc != 0 {
                // SAFETY: created or imported by this region.
                unsafe { (vmm.release)(self.alloc) };
            }
        });
    }
}

/// The publisher's allocation. The server is dropped first, so the descriptor stops being handed
/// out before it is closed.
pub(crate) struct DeviceAlloc {
    _server: FdServer,
    fd: OwnedFd,
    region: Region,
}

impl DeviceAlloc {
    /// Serves `endpoint` before returning, so a segment advertised after this has an answer.
    pub(crate) fn create(endpoint: &str, bytes: usize, ordinal: i32) -> Result<DeviceAlloc> {
        let (core, vmm) = vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        sys::watch_fork();
        let mut region = Region::new(ctx, rounded(vmm, device, bytes)?);
        let prop = pinned_on(device);
        // SAFETY: valid pointers; size is a multiple of the granularity.
        if unsafe { (vmm.create)(&raw mut region.alloc, region.size, &raw const prop, 0) }
            != SUCCESS
        {
            region.alloc = 0;
            return Err(Error::Unsupported("cuMemCreate failed".into()));
        }
        region.map(vmm, device, MEM_ACCESS_READ_WRITE)?;
        let mut raw: c_int = -1;
        // SAFETY: a POSIX fd handle is written as an int.
        if unsafe {
            (vmm.export)(
                (&raw mut raw).cast(),
                region.alloc,
                MEM_HANDLE_TYPE_POSIX_FD,
                0,
            )
        } != SUCCESS
        {
            return Err(Error::Unsupported(
                "cuMemExportToShareableHandle failed".into(),
            ));
        }
        // SAFETY: the export handed this process a new descriptor.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        // The driver sets it today without saying so; an exec'd child holding it keeps the
        // allocation alive and writable.
        // SAFETY: fcntl on a descriptor this process owns.
        unsafe { libc::fcntl(fd.as_raw_fd(), libc::F_SETFD, libc::FD_CLOEXEC) };
        let server = FdServer::start(endpoint, Share::One(fd.as_raw_fd()))?;
        ledger::made(Held::Export(region.addr), region.size);
        ledger::made(Held::Fd(fd.as_raw_fd()), 0);
        Ok(DeviceAlloc {
            _server: server,
            fd,
            region,
        })
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.region.addr
    }
}

impl Drop for DeviceAlloc {
    fn drop(&mut self) {
        ledger::undone(Held::Fd(self.fd.as_raw_fd()), 0);
        ledger::undone(Held::Export(self.region.addr), self.region.size);
    }
}

/// A subscriber's own device copy of a host payload, never exported. Its range is reserved whole
/// and backed a chunk at a time as slots are reached (`back`): a publisher adds slots, and memory
/// for the ones it never adds would sit idle.
pub(crate) struct DeviceBuffer {
    base: DevicePtr,
    size: usize,
    chunk: usize,
    device: c_int,
    ctx: usize,
    born: u32,
    /// Chunk `i` maps at `base + i * chunk`.
    chunks: Mutex<Vec<AllocHandle>>,
    backed: AtomicUsize,
}

impl DeviceBuffer {
    /// `bytes` of range, the first `backed` of them with memory now.
    pub(crate) fn new(bytes: usize, backed: usize, ordinal: i32) -> Result<DeviceBuffer> {
        let (core, vmm) = vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        sys::watch_fork();
        let chunk = rounded(vmm, device, CHUNK_BYTES)?;
        let size = bytes.div_ceil(chunk).max(1) * chunk;
        let mut base = 0;
        // SAFETY: valid out-pointer; no fixed address, default alignment.
        let rc = core.with_context(ctx, || unsafe {
            (vmm.address_reserve)(&raw mut base, size, 0, 0, 0)
        });
        if rc != SUCCESS {
            return Err(Error::Unsupported("cuMemAddressReserve failed".into()));
        }
        ledger::made(Held::Copy(base), size);
        let buf = DeviceBuffer {
            base,
            size,
            chunk,
            device,
            ctx: ctx as usize,
            born: sys::fork_gen(),
            chunks: Mutex::new(Vec::new()),
            backed: AtomicUsize::new(0),
        };
        buf.back(backed)?;
        Ok(buf)
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.base
    }

    #[cfg(test)]
    pub(crate) fn backed(&self) -> usize {
        self.backed.load(Ordering::Relaxed)
    }

    /// Gives the first `bytes` of the range memory, if they have none yet.
    pub(crate) fn back(&self, bytes: usize) -> Result<()> {
        // order: backed
        if bytes <= self.backed.load(Ordering::Acquire) {
            return Ok(());
        }
        let (core, vmm) = vmm()?;
        let mut chunks = self.chunks.lock().unwrap_or_else(PoisonError::into_inner);
        let want = bytes.min(self.size).div_ceil(self.chunk);
        core.with_context(self.ctx as *mut c_void, || {
            while chunks.len() < want {
                let at = self.base + (chunks.len() * self.chunk) as u64;
                let prop = pinned_on(self.device);
                let mut h: AllocHandle = 0;
                // SAFETY: valid pointers; chunk is a multiple of the granularity.
                if unsafe { (vmm.create)(&raw mut h, self.chunk, &raw const prop, 0) } != SUCCESS {
                    return Err(Error::Unsupported("cuMemCreate failed".into()));
                }
                let desc = MemAccessDesc {
                    location: MemLocation {
                        kind: MEM_LOCATION_TYPE_DEVICE,
                        id: self.device,
                    },
                    flags: MEM_ACCESS_READ_WRITE,
                };
                // SAFETY: inside the reserved range, unmapped there so far.
                if unsafe { (vmm.map)(at, self.chunk, 0, h, 0) } != SUCCESS {
                    // SAFETY: created above, never mapped.
                    unsafe { (vmm.release)(h) };
                    return Err(Error::Unsupported("cuMemMap failed".into()));
                }
                // SAFETY: a mapped range and one valid descriptor.
                if unsafe { (vmm.set_access)(at, self.chunk, &raw const desc, 1) } != SUCCESS {
                    // SAFETY: mapped above at this size.
                    unsafe { (vmm.unmap)(at, self.chunk) };
                    // SAFETY: created above, now unmapped.
                    unsafe { (vmm.release)(h) };
                    return Err(Error::Unsupported("cuMemSetAccess failed".into()));
                }
                chunks.push(h);
            }
            Ok(())
        })?;
        self.backed
            .store(chunks.len() * self.chunk, Ordering::Release); // order: backed
        Ok(())
    }
}

impl Drop for DeviceBuffer {
    fn drop(&mut self) {
        ledger::undone(Held::Copy(self.base), self.size);
        if sys::fork_gen() != self.born {
            return;
        }
        let Ok((core, vmm)) = vmm() else {
            return;
        };
        let chunks = std::mem::take(
            self.chunks
                .get_mut()
                .unwrap_or_else(PoisonError::into_inner),
        );
        core.with_context(self.ctx as *mut c_void, || {
            for (i, h) in chunks.into_iter().enumerate() {
                let at = self.base + (i * self.chunk) as u64;
                // SAFETY: mapped by back() at this size.
                unsafe { (vmm.unmap)(at, self.chunk) };
                // SAFETY: created by back(), now unmapped, released once.
                unsafe { (vmm.release)(h) };
            }
            // SAFETY: reserved by new() at this size, now unmapped.
            unsafe { (vmm.address_free)(self.base, self.size) };
        });
    }
}

/// A subscriber's mapping of the publisher's allocation, readable only: a consuming kernel cannot
/// write the slot.
pub(crate) struct DeviceImport {
    region: Region,
}

impl DeviceImport {
    pub(crate) fn open(endpoint: &str, bytes: usize, ordinal: i32) -> Result<DeviceImport> {
        let (core, vmm) = vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        let fd = receive(endpoint)?;
        sys::watch_fork();
        let mut region = Region::new(ctx, rounded(vmm, device, bytes)?);
        // SAFETY: the handle is the descriptor, passed as a pointer-sized integer. The import
        // takes its own reference, so the descriptor is closed by this process either way.
        let rc = unsafe {
            (vmm.import)(
                &raw mut region.alloc,
                fd.as_raw_fd() as isize as *mut c_void,
                MEM_HANDLE_TYPE_POSIX_FD,
            )
        };
        drop(fd);
        if rc != SUCCESS {
            region.alloc = 0;
            return Err(Error::Unsupported(
                "cuMemImportFromShareableHandle failed".into(),
            ));
        }
        region.map(vmm, device, MEM_ACCESS_READ)?;
        ledger::made(Held::Import(region.addr), region.size);
        Ok(DeviceImport { region })
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.region.addr
    }
}

impl Drop for DeviceImport {
    fn drop(&mut self) {
        ledger::undone(Held::Import(self.region.addr), self.region.size);
    }
}
