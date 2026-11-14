//! The device-handle route: the payload area is a reserved range backed by VMM chunks as slots
//! are added, each exported as a POSIX fd and handed out in order over an `AF_UNIX` socket file
//! beside the segment, one accept thread per channel. Both ends also check that the peer is this
//! user before a descriptor moves.

use std::ffi::{c_int, c_void};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

use super::driver::{
    AllocFlags, AllocHandle, Core, DevicePtr, Driver, MEM_ACCESS_READ, MEM_ACCESS_READ_WRITE,
    MEM_ALLOCATION_TYPE_PINNED, MEM_GRANULARITY_MINIMUM, MEM_HANDLE_TYPE_POSIX_FD,
    MEM_LOCATION_TYPE_DEVICE, MemAccessDesc, MemAllocationProp, MemLocation, SUCCESS, Vmm,
};
use super::ledger::{self, Held};
use crate::error::{Error, Result};
use crate::segment::SegmentId;
use crate::sys::{self, PiMutex};

use super::fd::{ENDPOINT_PREFIX, FdServer, Mailbox, Share};

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

/// The chunk a slot payload of `stride` bytes is backed by: at least `CHUNK_BYTES`, at most the
/// GPU arena's, in the device's granularity. Both ends of a channel compute it from the layout.
pub(crate) fn slot_chunk(vmm: &Vmm, device: c_int, stride: usize) -> Result<usize> {
    rounded(vmm, device, stride.clamp(CHUNK_BYTES, super::arena::CHUNK))
}

/// A device range reserved whole and backed a chunk at a time, chunk `i` at `base + i * chunk`:
/// a publisher adds slots, and memory for the ones it never adds would sit idle. Comes apart in the
/// order it was built: unmap and release each chunk, then free the range.
struct Range {
    base: DevicePtr,
    size: usize,
    chunk: usize,
    device: c_int,
    ctx: usize,
    born: u32,
    handles: Mutex<Vec<AllocHandle>>,
    backed: AtomicUsize,
}

impl Range {
    fn reserve(
        ordinal: i32,
        bytes: usize,
        chunk_for: impl FnOnce(&Vmm, c_int) -> Result<usize>,
    ) -> Result<Range> {
        let (core, vmm) = vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        sys::watch_fork();
        let chunk = chunk_for(vmm, device)?;
        let size = bytes.div_ceil(chunk).max(1) * chunk;
        let mut base = 0;
        // SAFETY: valid out-pointer; no fixed address, default alignment.
        let rc = core.with_context(ctx, || unsafe {
            (vmm.address_reserve)(&raw mut base, size, 0, 0, 0)
        });
        if rc != SUCCESS {
            return Err(Error::Unsupported("cuMemAddressReserve failed".into()));
        }
        Ok(Range {
            base,
            size,
            chunk,
            device,
            ctx: ctx as usize,
            born: sys::fork_gen(),
            handles: Mutex::new(Vec::new()),
            backed: AtomicUsize::new(0),
        })
    }

    fn covers(&self, bytes: usize) -> bool {
        bytes <= self.backed.load(Ordering::Acquire) // order: backed
    }

    /// Backs the first `bytes`: maps `make(i)` at each missing chunk `i`, with `access` for the
    /// device. A chunk that fails is released and ends the call; the ones before it stay.
    fn extend(
        &self,
        bytes: usize,
        access: c_int,
        mut make: impl FnMut(&Vmm, usize) -> Result<AllocHandle>,
    ) -> Result<()> {
        if self.covers(bytes) {
            return Ok(());
        }
        if bytes > self.size {
            return Err(Error::InvalidArgument(format!(
                "{bytes} bytes are past a device range of {}",
                self.size
            )));
        }
        let (core, vmm) = vmm()?;
        let mut handles = self.handles.lock().unwrap_or_else(PoisonError::into_inner);
        let want = bytes.div_ceil(self.chunk);
        let r = core.with_context(self.ctx as *mut c_void, || {
            while handles.len() < want {
                let i = handles.len();
                let at = self.base + (i * self.chunk) as u64;
                let h = make(vmm, i)?;
                // SAFETY: inside the reserved range, unmapped there so far.
                if unsafe { (vmm.map)(at, self.chunk, 0, h, 0) } != SUCCESS {
                    // SAFETY: made above, never mapped.
                    unsafe { (vmm.release)(h) };
                    return Err(Error::Unsupported("cuMemMap failed".into()));
                }
                let desc = MemAccessDesc {
                    location: MemLocation {
                        kind: MEM_LOCATION_TYPE_DEVICE,
                        id: self.device,
                    },
                    flags: access,
                };
                // SAFETY: a mapped range and one valid descriptor.
                if unsafe { (vmm.set_access)(at, self.chunk, &raw const desc, 1) } != SUCCESS {
                    // SAFETY: mapped above at this size.
                    unsafe { (vmm.unmap)(at, self.chunk) };
                    // SAFETY: made above, now unmapped.
                    unsafe { (vmm.release)(h) };
                    return Err(Error::Unsupported("cuMemSetAccess failed".into()));
                }
                handles.push(h);
            }
            Ok(())
        });
        self.backed
            .store(handles.len() * self.chunk, Ordering::Release); // order: backed
        r
    }
}

impl Drop for Range {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let Ok((core, vmm)) = vmm() else {
            return;
        };
        let handles = std::mem::take(
            self.handles
                .get_mut()
                .unwrap_or_else(PoisonError::into_inner),
        );
        core.with_context(self.ctx as *mut c_void, || {
            for (i, h) in handles.into_iter().enumerate() {
                let at = self.base + (i * self.chunk) as u64;
                // SAFETY: mapped by extend() at this size.
                unsafe { (vmm.unmap)(at, self.chunk) };
                // SAFETY: made or imported by extend(), now unmapped, released once.
                unsafe { (vmm.release)(h) };
            }
            // SAFETY: reserved at this size, now unmapped.
            unsafe { (vmm.address_free)(self.base, self.size) };
        });
    }
}

fn create_on(vmm: &Vmm, device: c_int, bytes: usize) -> Result<AllocHandle> {
    let prop = pinned_on(device);
    let mut h: AllocHandle = 0;
    // SAFETY: valid pointers; bytes is a multiple of the granularity.
    if unsafe { (vmm.create)(&raw mut h, bytes, &raw const prop, 0) } != SUCCESS {
        return Err(Error::Unsupported("cuMemCreate failed".into()));
    }
    Ok(h)
}

/// The publisher's payload, a chunk per few slots, each exported as a POSIX fd that the endpoint
/// hands out in order. The server is dropped first, so no descriptor is handed out once they close.
pub(crate) struct DeviceAlloc {
    _server: FdServer,
    share: Arc<Share>,
    range: Range,
}

impl DeviceAlloc {
    /// `bytes` of payload of slots `stride` apart, the first `backed` with memory. Serves
    /// `endpoint` before returning, so a segment advertised after this has an answer.
    pub(crate) fn create(
        endpoint: &str,
        bytes: usize,
        stride: usize,
        backed: usize,
        ordinal: i32,
    ) -> Result<DeviceAlloc> {
        let range = Range::reserve(ordinal, bytes, |vmm, device| {
            slot_chunk(vmm, device, stride)
        })?;
        let share = Share::new(range.chunk);
        let server = FdServer::start(endpoint, Arc::clone(&share))?;
        ledger::made(Held::Export(range.base), range.size);
        let alloc = DeviceAlloc {
            _server: server,
            share,
            range,
        };
        alloc.back(backed)?;
        Ok(alloc)
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.range.base
    }

    /// Gives the first `bytes` of the payload memory, exporting each chunk it adds.
    pub(crate) fn back(&self, bytes: usize) -> Result<()> {
        let device = self.range.device;
        let chunk = self.range.chunk;
        self.range.extend(bytes, MEM_ACCESS_READ_WRITE, |vmm, _| {
            let h = create_on(vmm, device, chunk)?;
            let mut raw: c_int = -1;
            // SAFETY: a POSIX fd handle is written as an int.
            if unsafe { (vmm.export)((&raw mut raw).cast(), h, MEM_HANDLE_TYPE_POSIX_FD, 0) }
                != SUCCESS
            {
                // SAFETY: created above, never mapped.
                unsafe { (vmm.release)(h) };
                return Err(Error::Unsupported(
                    "cuMemExportToShareableHandle failed".into(),
                ));
            }
            // SAFETY: the export handed this process a new descriptor.
            let fd = unsafe { OwnedFd::from_raw_fd(raw) };
            // The driver sets it today without saying so; an exec'd child holding it keeps the
            // chunk alive and writable.
            // SAFETY: fcntl on a descriptor this process owns.
            unsafe { libc::fcntl(fd.as_raw_fd(), libc::F_SETFD, libc::FD_CLOEXEC) };
            self.share.push(fd);
            Ok(h)
        })
    }
}

impl Drop for DeviceAlloc {
    fn drop(&mut self) {
        ledger::undone(Held::Export(self.range.base), self.range.size);
    }
}

/// A subscriber's own device copy of a host payload, never exported, backed as slots are reached.
pub(crate) struct DeviceBuffer {
    range: Range,
}

impl DeviceBuffer {
    /// `bytes` of range, the first `backed` of them with memory now.
    pub(crate) fn new(bytes: usize, backed: usize, ordinal: i32) -> Result<DeviceBuffer> {
        let range = Range::reserve(ordinal, bytes, |vmm, device| {
            rounded(vmm, device, CHUNK_BYTES)
        })?;
        ledger::made(Held::Copy(range.base), range.size);
        let buf = DeviceBuffer { range };
        buf.back(backed)?;
        Ok(buf)
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.range.base
    }

    #[cfg(test)]
    pub(crate) fn backed(&self) -> usize {
        self.range.backed.load(Ordering::Relaxed)
    }

    /// Gives the first `bytes` of the range memory, if they have none yet.
    pub(crate) fn back(&self, bytes: usize) -> Result<()> {
        let (device, chunk) = (self.range.device, self.range.chunk);
        self.range.extend(bytes, MEM_ACCESS_READ_WRITE, |vmm, _| {
            create_on(vmm, device, chunk)
        })
    }
}

impl Drop for DeviceBuffer {
    fn drop(&mut self) {
        ledger::undone(Held::Copy(self.range.base), self.range.size);
    }
}

/// A subscriber's mapping of the publisher's payload, readable only: a consuming kernel cannot
/// write the slot. Chunks come on the endpoint's connection as the publisher adds them (`reach`).
pub(crate) struct DeviceImport {
    mail: PiMutex<Mailbox>,
    range: Range,
}

impl DeviceImport {
    /// Maps the first `backed` bytes now, so a publisher that is gone is the caller's error.
    pub(crate) fn open(
        endpoint: &str,
        bytes: usize,
        stride: usize,
        backed: usize,
        ordinal: i32,
    ) -> Result<DeviceImport> {
        let range = Range::reserve(ordinal, bytes, |vmm, device| {
            slot_chunk(vmm, device, stride)
        })?;
        ledger::made(Held::Import(range.base), range.size);
        let import = DeviceImport {
            mail: PiMutex::new(Mailbox::new(endpoint, range.chunk)),
            range,
        };
        import.reach(backed)?;
        Ok(import)
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.range.base
    }

    /// Maps the publisher's chunks until the first `bytes` are mapped: the ones it sent since. Only
    /// the first call (`open`'s) waits, for the chunks made before it asked.
    pub(crate) fn reach(&self, bytes: usize) -> Result<()> {
        if self.range.covers(bytes) {
            return Ok(());
        }
        let mut mail = self.mail.lock();
        let chunk = self.range.chunk;
        let have = self.range.backed.load(Ordering::Acquire) / chunk; // order: backed
        #[allow(clippy::cast_possible_truncation)]
        let fds = mail.collect(have as u32, bytes.div_ceil(chunk) as u32)?;
        // All that came, past `bytes` too: the publisher does not send a chunk twice.
        let reached = ((have + fds.len()) * chunk).min(self.range.size);
        self.range.extend(reached, MEM_ACCESS_READ, |vmm, i| {
            let fd = fds
                .get(i - have)
                .ok_or_else(|| Error::NotReady("a chunk was imported meanwhile".into()))?;
            let mut h: AllocHandle = 0;
            // SAFETY: the handle is the descriptor, passed as a pointer-sized integer. The import
            // takes its own reference, so the descriptor is closed by this process either way.
            let rc = unsafe {
                (vmm.import)(
                    &raw mut h,
                    fd.as_raw_fd() as isize as *mut c_void,
                    MEM_HANDLE_TYPE_POSIX_FD,
                )
            };
            if rc != SUCCESS {
                return Err(Error::Unsupported(
                    "cuMemImportFromShareableHandle failed".into(),
                ));
            }
            Ok(h)
        })?;
        if !self.range.covers(bytes) {
            return Err(Error::NotReady(
                "the publisher has not sent the device chunks a frame reaches".into(),
            ));
        }
        Ok(())
    }
}

impl Drop for DeviceImport {
    fn drop(&mut self) {
        ledger::undone(Held::Import(self.range.base), self.range.size);
    }
}
