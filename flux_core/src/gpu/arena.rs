//! The GPU arena: a process's memory for the large fields of frames produced on a device, one per
//! device, grown in `CHUNK`s. On an integrated GPU a chunk is host memory both processors reach
//! (`HOST_NUMA`), elsewhere device memory. Each chunk is exported as a POSIX fd and served at an
//! endpoint beside the segments; a subscriber imports the chunks a frame reaches at the same
//! offsets, so an offset means the same bytes on both sides.
//!
//! The allocator's tags live in a host shadow of the range, since the CPU cannot write device
//! memory. The publisher's arena lives for the process, as the host arena does, so it is not in
//! the debug ledger; a subscriber's imports are.

#![allow(
    clippy::disallowed_types,
    reason = "the imported chunk count; orderings tagged `order:`"
)]

use std::ffi::{c_int, c_void};
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::{Arc, Weak};

use super::driver::{
    ATTR_HOST_NUMA_ID, ATTR_HOST_NUMA_VMM_SUPPORTED, ATTR_INTEGRATED, AllocFlags, AllocHandle,
    Core, DevicePtr, MEM_ACCESS_READ, MEM_ACCESS_READ_WRITE, MEM_ALLOCATION_TYPE_PINNED,
    MEM_GRANULARITY_MINIMUM, MEM_HANDLE_TYPE_POSIX_FD, MEM_LOCATION_TYPE_DEVICE,
    MEM_LOCATION_TYPE_HOST_NUMA, MemAccessDesc, MemAllocationProp, MemLocation, SUCCESS, Vmm,
};
use super::fd::{self, ENDPOINT_PREFIX, FdServer, Share};
use super::ledger::{self, Held};
use super::vmm;
use crate::arena::{self as host_arena, Heap};
use crate::error::{Error, Result};
use crate::sys::{self, PiMutex};

/// One allocation each, so a 1 GiB arena is 32 descriptors at most. A multiple of the 2 MiB
/// granularity the GPUs flux runs on report.
pub const CHUNK: usize = 32 << 20;
const NOT_SUPPORTED: i32 = 801;

/// Where the chunks are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Place {
    Device,
    /// Host memory of this NUMA node, which the CPU reads without a copy.
    HostNuma(c_int),
}

/// `HOST_NUMA` where host memory is the GPU's own (an integrated GPU reads it at full speed),
/// device memory elsewhere.
fn place_for(core: &Core, device: c_int) -> Place {
    if core.attribute(device, ATTR_INTEGRATED)
        && core.attribute(device, ATTR_HOST_NUMA_VMM_SUPPORTED)
    {
        let id = core.attribute_value(device, ATTR_HOST_NUMA_ID);
        Place::HostNuma(id.filter(|&id| id >= 0).unwrap_or(0))
    } else {
        Place::Device
    }
}

fn location(place: Place, device: c_int) -> MemLocation {
    match place {
        Place::Device => MemLocation {
            kind: MEM_LOCATION_TYPE_DEVICE,
            id: device,
        },
        Place::HostNuma(node) => MemLocation {
            kind: MEM_LOCATION_TYPE_HOST_NUMA,
            id: node,
        },
    }
}

fn prop(place: Place, device: c_int) -> MemAllocationProp {
    MemAllocationProp {
        kind: MEM_ALLOCATION_TYPE_PINNED,
        requested_handle_types: MEM_HANDLE_TYPE_POSIX_FD,
        location: location(place, device),
        win32_handle_metadata: std::ptr::null_mut(),
        alloc_flags: AllocFlags::default(),
    }
}

fn access(vmm: &Vmm, at: DevicePtr, loc: MemLocation, flags: c_int) -> i32 {
    let desc = MemAccessDesc {
        location: loc,
        flags,
    };
    // SAFETY: a mapped range and one valid descriptor.
    unsafe { (vmm.set_access)(at, CHUNK, &raw const desc, 1) }
}

/// What the publisher's chunks need, the same for each: a device, its context, where they go.
#[derive(Clone, Copy)]
struct Target {
    device: c_int,
    ctx: usize,
    place: Place,
    base: DevicePtr,
}

/// Where a publisher with this `id` serves its chunks.
fn endpoint_of(id: u64) -> String {
    format!("/dev/shm/{ENDPOINT_PREFIX}arena-{id:016x}")
}

/// What a frame's GPU trailer says about the arena its blocks are in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpuFrame {
    pub id: u64,
    pub uuid: [u8; 16],
    pub place: Place,
    pub size: usize,
    /// Chunks made when the frame was committed: its blocks are inside them.
    pub chunks: u32,
}

const DEVICE_TAG: u8 = 1;
const HOST_NUMA_TAG: u8 = 2;

impl GpuFrame {
    pub fn encode(&self) -> [u8; host_arena::TRAILER_BYTES] {
        let mut t = [0u8; host_arena::TRAILER_BYTES];
        t[..8].copy_from_slice(&self.id.to_le_bytes());
        t[8..24].copy_from_slice(&self.uuid);
        let (tag, node) = match self.place {
            Place::Device => (DEVICE_TAG, 0),
            Place::HostNuma(n) => (HOST_NUMA_TAG, n),
        };
        t[24] = tag;
        t[28..32].copy_from_slice(&node.to_le_bytes());
        t[32..40].copy_from_slice(&(self.size as u64).to_le_bytes());
        t[40..44].copy_from_slice(&self.chunks.to_le_bytes());
        t
    }

    /// None for a trailer no publisher of this build writes: another process wrote it.
    pub fn decode(t: &[u8]) -> Option<GpuFrame> {
        let t: &[u8; host_arena::TRAILER_BYTES] = t.try_into().ok()?;
        let u64_at = |i: usize| u64::from_le_bytes(t[i..i + 8].try_into().unwrap_or_default());
        let u32_at = |i: usize| u32::from_le_bytes(t[i..i + 4].try_into().unwrap_or_default());
        let node = i32::from_le_bytes(t[28..32].try_into().ok()?);
        let place = match t[24] {
            DEVICE_TAG => Place::Device,
            HOST_NUMA_TAG => Place::HostNuma(node),
            _ => return None,
        };
        let size = usize::try_from(u64_at(32)).ok()?;
        let chunks = u32_at(40);
        let fits = size > 0
            && size <= host_arena::MAX_GPU_ARENA_BYTES
            && size.is_multiple_of(CHUNK)
            && chunks as usize <= size / CHUNK;
        fits.then(|| GpuFrame {
            id: u64_at(0),
            uuid: t[8..24].try_into().unwrap_or_default(),
            place,
            size,
            chunks,
        })
    }
}

/// The publisher side: this process's arena on one device.
pub struct GpuArena {
    ordinal: i32,
    id: u64,
    uuid: [u8; 16],
    endpoint: String,
    size: usize,
    target: Target,
    shadow: NonNull<u8>,
    inner: PiMutex<Inner>,
    chunks: Arc<PiMutex<Vec<OwnedFd>>>,
    _server: FdServer,
    born: u32,
}

struct Inner {
    heap: Heap,
    handles: Vec<AllocHandle>,
}

// SAFETY: the reserved range is fixed for the process's life and the rest is behind locks.
unsafe impl Sync for GpuArena {}
// SAFETY: as above.
unsafe impl Send for GpuArena {}

/// The arenas made in this process, one per device ordinal. Never freed: frames left in slots may
/// still name their bytes after their publisher is gone, as with the host arena.
static ARENAS: PiMutex<Vec<&'static GpuArena>> = PiMutex::new(Vec::new());

/// This process's arena on `ordinal`, made on first use. Its size is the host arena's
/// (`FLUX_ARENA_BYTES`), up to `MAX_GPU_ARENA_BYTES`.
pub fn gpu_arena(ordinal: i32) -> Result<&'static GpuArena> {
    let size = host_arena::configured()
        .ok_or_else(|| Error::Unsupported("the arena is off (FLUX_ARENA_BYTES=0)".into()))?
        .min(host_arena::MAX_GPU_ARENA_BYTES);
    let mut all = ARENAS.lock();
    let born = sys::fork_gen();
    if let Some(a) = all.iter().find(|a| a.ordinal == ordinal && a.born == born) {
        return Ok(a);
    }
    let a: &'static GpuArena = Box::leak(Box::new(GpuArena::create(ordinal, size, None)?));
    all.push(a);
    Ok(a)
}

/// Grows the arena over `[from, to)`, a run of whole chunks: makes, maps and exports each.
fn grow(
    vmm: &Vmm,
    t: Target,
    handles: &mut Vec<AllocHandle>,
    chunks: &PiMutex<Vec<OwnedFd>>,
    from: u64,
    to: u64,
) -> io::Result<()> {
    let fail = |what: &str| io::Error::other(format!("{what} failed for a GPU arena chunk"));
    #[allow(clippy::cast_possible_truncation)]
    for off in (from as usize..to as usize).step_by(CHUNK) {
        let at = t.base + off as u64;
        let mut h: AllocHandle = 0;
        let p = prop(t.place, t.device);
        // SAFETY: valid pointers; CHUNK is a multiple of the granularity.
        if unsafe { (vmm.create)(&raw mut h, CHUNK, &raw const p, 0) } != SUCCESS {
            return Err(fail("cuMemCreate"));
        }
        let undo = |mapped: bool| {
            if mapped {
                // SAFETY: mapped below at this size.
                unsafe { (vmm.unmap)(at, CHUNK) };
            }
            // SAFETY: created above.
            unsafe { (vmm.release)(h) };
        };
        // SAFETY: inside the reserved range, unmapped there so far.
        if unsafe { (vmm.map)(at, CHUNK, 0, h, 0) } != SUCCESS {
            undo(false);
            return Err(fail("cuMemMap"));
        }
        let device = location(Place::Device, t.device);
        let host_too = matches!(t.place, Place::HostNuma(_)).then(|| location(t.place, t.device));
        if access(vmm, at, device, MEM_ACCESS_READ_WRITE) != SUCCESS
            || host_too.is_some_and(|h| access(vmm, at, h, MEM_ACCESS_READ_WRITE) != SUCCESS)
        {
            undo(true);
            return Err(fail("cuMemSetAccess"));
        }
        let mut raw: c_int = -1;
        // SAFETY: a POSIX fd handle is written as an int.
        if unsafe { (vmm.export)((&raw mut raw).cast(), h, MEM_HANDLE_TYPE_POSIX_FD, 0) } != SUCCESS
        {
            undo(true);
            return Err(fail("cuMemExportToShareableHandle"));
        }
        // SAFETY: the export handed this process a new descriptor.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        // As the device-handle route: an exec'd child must not keep the chunk alive and writable.
        // SAFETY: fcntl on a descriptor this process owns.
        unsafe { libc::fcntl(fd.as_raw_fd(), libc::F_SETFD, libc::FD_CLOEXEC) };
        handles.push(h);
        chunks.lock().push(fd);
    }
    Ok(())
}

impl GpuArena {
    /// One that is not the process's, which goes when dropped; `place` overrides the device's
    /// choice. For tests: frames use `gpu_arena`.
    pub fn create(ordinal: i32, size: usize, place: Option<Place>) -> Result<GpuArena> {
        let (core, vmm) = vmm::vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = vmm::device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        let place = place.unwrap_or_else(|| place_for(core, device));
        let mut g = 0;
        let p = prop(place, device);
        // SAFETY: valid pointers.
        let rc = unsafe { (vmm.granularity)(&raw mut g, &raw const p, MEM_GRANULARITY_MINIMUM) };
        if rc != SUCCESS || g == 0 || !CHUNK.is_multiple_of(g) {
            return Err(Error::Unsupported(format!(
                "GPU arena chunks of {CHUNK} bytes do not suit this device (granularity {g}, \
                 result {rc})"
            )));
        }
        let size = size / CHUNK * CHUNK;
        sys::watch_fork();
        let mut base: DevicePtr = 0;
        // SAFETY: valid out-pointer; no fixed address, default alignment.
        let rc = core.with_context(ctx, || unsafe {
            (vmm.address_reserve)(&raw mut base, size, 0, 0, 0)
        });
        if rc != SUCCESS {
            return Err(Error::Unsupported("cuMemAddressReserve failed".into()));
        }
        // Tags only: untouched pages cost nothing, as the host arena's do.
        // SAFETY: a fresh private mapping; no existing memory is affected.
        let shadow = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_PRIVATE | libc::MAP_ANONYMOUS | libc::MAP_NORESERVE,
                -1,
                0,
            )
        };
        let Some(shadow) = NonNull::new(shadow.cast::<u8>()).filter(|_| shadow != libc::MAP_FAILED)
        else {
            return Err(Error::io(
                "mmap",
                "the GPU arena's tags",
                io::Error::last_os_error(),
            ));
        };
        let uuid = vmm::device_uuid(ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGetUuid failed".into()))?;
        let id = host_arena::random();
        let endpoint = endpoint_of(id);
        let chunks = Arc::new(PiMutex::new(Vec::new()));
        let server = FdServer::start(&endpoint, Share::Chunks(Arc::clone(&chunks)))?;
        let target = Target {
            device,
            ctx: ctx as usize,
            place,
            base,
        };
        let mut handles = Vec::new();
        let heap = core.with_context(ctx, || {
            Heap::new(
                shadow,
                size as u64,
                CHUNK as u64,
                Some(&mut |from, to| grow(vmm, target, &mut handles, &chunks, from, to)),
            )
        });
        let heap = heap.map_err(|e| Error::io("the first GPU arena chunk", &endpoint, e))?;
        Ok(GpuArena {
            ordinal,
            id,
            uuid,
            endpoint,
            size,
            target,
            shadow,
            inner: PiMutex::new(Inner { heap, handles }),
            chunks,
            _server: server,
            born: sys::fork_gen(),
        })
    }

    /// The offset of `bytes`, on an `align` boundary; None when the arena is full or a chunk could
    /// not be made.
    pub fn alloc(&self, bytes: usize, align: usize) -> Option<u64> {
        if !align.is_power_of_two() || self.born != sys::fork_gen() {
            return None;
        }
        let (core, vmm) = vmm::vmm().ok()?;
        let t = self.target;
        let mut inner = self.inner.lock();
        let Inner { heap, handles } = &mut *inner;
        let chunks = &self.chunks;
        core.with_context(t.ctx as *mut c_void, || {
            heap.alloc(bytes as u64, align as u64, &mut |from, to| {
                grow(vmm, t, handles, chunks, from, to)
            })
        })
    }

    pub fn free(&self, off: u64) {
        self.inner.lock().heap.free(off);
    }

    pub fn device_ptr(&self, off: u64) -> u64 {
        self.target.base + off
    }

    /// Where the CPU reaches `off`: only a `HOST_NUMA` arena's bytes have such an address.
    pub fn host_ptr(&self, off: u64) -> Option<*mut u8> {
        #[allow(clippy::cast_possible_truncation)]
        matches!(self.target.place, Place::HostNuma(_))
            .then(|| (self.target.base + off) as usize as *mut u8)
    }

    /// Chunks made so far; a frame names how many its blocks reach.
    pub fn chunks(&self) -> u32 {
        #[allow(clippy::cast_possible_truncation)]
        let n = self.chunks.lock().len() as u32;
        n
    }

    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn place(&self) -> Place {
        self.target.place
    }

    pub fn ordinal(&self) -> i32 {
        self.ordinal
    }

    /// What a frame with blocks of this arena, committed now, says of it.
    pub fn frame(&self) -> GpuFrame {
        GpuFrame {
            id: self.id,
            uuid: self.uuid,
            place: self.target.place,
            size: self.size,
            chunks: self.chunks(),
        }
    }
}

/// Only an arena that is not the process's (a test's) goes. The process's endpoint file outlives
/// it: the sweep removes an endpoint nobody listens on.
impl Drop for GpuArena {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let handles = std::mem::take(&mut self.inner.lock().handles);
        if let Ok((core, vmm)) = vmm::vmm() {
            core.with_context(self.target.ctx as *mut c_void, || {
                for (i, h) in handles.into_iter().enumerate() {
                    let at = self.target.base + (i * CHUNK) as u64;
                    // SAFETY: mapped by grow at this size.
                    unsafe { (vmm.unmap)(at, CHUNK) };
                    // SAFETY: created by grow.
                    unsafe { (vmm.release)(h) };
                }
                // SAFETY: reserved by create at this size, now unmapped.
                unsafe { (vmm.address_free)(self.target.base, self.size) };
            });
        }
        // SAFETY: mapped by create with size; the heap that wrote it goes with self.
        unsafe { libc::munmap(self.shadow.as_ptr().cast(), self.size) };
    }
}

/// How a subscriber's CPU reaches a `HOST_NUMA` arena: through the imported range itself (x86
/// drivers grant host access to an import), or through its own mapping of each chunk's
/// descriptor (an Orin driver refuses the grant with 801 but maps the descriptor).
enum Cpu {
    /// A device arena, or no chunk imported yet.
    None,
    Import,
    Mapped(NonNull<u8>),
}

/// A subscriber's read-only import of a publisher's arena.
pub struct GpuArenaImport {
    id: u64,
    endpoint: String,
    size: usize,
    place: Place,
    ordinal: i32,
    device: c_int,
    ctx: usize,
    base: DevicePtr,
    inner: PiMutex<ImportInner>,
    /// Chunks imported so far, read without the lock by each frame.
    mapped: AtomicU32,
    /// The CPU's base for every chunk once the first one picked the way; 0 for none.
    cpu_base: AtomicUsize,
    born: u32,
}

/// The imports of this process, shared by every channel whose frames name the same arena.
static IMPORTS: PiMutex<Vec<Weak<GpuArenaImport>>> = PiMutex::new(Vec::new());

/// This process's import of the arena `f` names, opened on the device with its UUID. Chunks come
/// with `reach`.
pub fn import_for(f: &GpuFrame) -> Result<Arc<GpuArenaImport>> {
    let born = sys::fork_gen();
    let found = {
        let mut all = IMPORTS.lock();
        all.retain(|w| w.strong_count() > 0);
        all.iter()
            .filter_map(Weak::upgrade)
            .find(|m| m.id == f.id && m.born == born)
    };
    let m = if let Some(m) = found {
        m
    } else {
        let ordinal = vmm::device_for_uuid(&f.uuid).ok_or_else(|| {
            Error::Mismatch("no device in this process has the GPU arena's UUID".into())
        })?;
        let m = Arc::new(GpuArenaImport::open_id(f.id, f.size, f.place, ordinal)?);
        let mut all = IMPORTS.lock();
        // Another thread may have opened it meanwhile; keep the first.
        if let Some(first) = all
            .iter()
            .filter_map(Weak::upgrade)
            .find(|o| o.id == f.id && o.born == born)
        {
            first
        } else {
            all.push(Arc::downgrade(&m));
            m
        }
    };
    Ok(m)
}

struct ImportInner {
    handles: Vec<AllocHandle>,
    cpu: Cpu,
}

// SAFETY: the reserved ranges are fixed while it lives and the rest is behind a lock.
unsafe impl Sync for GpuArenaImport {}
// SAFETY: as above.
unsafe impl Send for GpuArenaImport {}

impl GpuArenaImport {
    /// Reserves the range; chunks come with `reach`. `size` and `place` are the publisher's.
    pub fn open(endpoint: &str, size: usize, place: Place, ordinal: i32) -> Result<GpuArenaImport> {
        GpuArenaImport::open_at(0, endpoint.to_owned(), size, place, ordinal)
    }

    fn open_id(id: u64, size: usize, place: Place, ordinal: i32) -> Result<GpuArenaImport> {
        GpuArenaImport::open_at(id, endpoint_of(id), size, place, ordinal)
    }

    fn open_at(
        id: u64,
        endpoint: String,
        size: usize,
        place: Place,
        ordinal: i32,
    ) -> Result<GpuArenaImport> {
        let (core, vmm) = vmm::vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = vmm::device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        let size = size / CHUNK * CHUNK;
        sys::watch_fork();
        let mut base: DevicePtr = 0;
        // SAFETY: valid out-pointer; no fixed address, default alignment.
        let rc = core.with_context(ctx, || unsafe {
            (vmm.address_reserve)(&raw mut base, size, 0, 0, 0)
        });
        if rc != SUCCESS {
            return Err(Error::Unsupported("cuMemAddressReserve failed".into()));
        }
        Ok(GpuArenaImport {
            id,
            endpoint,
            size,
            place,
            ordinal,
            device,
            ctx: ctx as usize,
            base,
            inner: PiMutex::new(ImportInner {
                handles: Vec::new(),
                cpu: Cpu::None,
            }),
            mapped: AtomicU32::new(0),
            cpu_base: AtomicUsize::new(0),
            born: sys::fork_gen(),
        })
    }

    /// Imports chunks until `chunks` of them are mapped. A round trip to the publisher, so a
    /// frame asks for this only when it reaches past what is mapped.
    pub fn reach(&self, chunks: u32) -> Result<()> {
        // order: arena_mapped
        if self.mapped.load(Ordering::Acquire) >= chunks {
            return Ok(());
        }
        let mut inner = self.inner.lock();
        #[allow(clippy::cast_possible_truncation)]
        let have = inner.handles.len() as u32;
        if have >= chunks {
            return Ok(());
        }
        if chunks as usize * CHUNK > self.size {
            return Err(Error::Mismatch(format!(
                "a frame reaches {chunks} chunks of the GPU arena '{}', past its {} bytes",
                self.endpoint, self.size
            )));
        }
        let fds = fd::fetch_chunks(&self.endpoint, have)?;
        if fds.len() < (chunks - have) as usize {
            return Err(Error::NotReady(format!(
                "the GPU arena '{}' has fewer chunks than a frame reaches",
                self.endpoint
            )));
        }
        let (core, vmm) = vmm::vmm()?;
        let r = core.with_context(self.ctx as *mut c_void, || {
            for fd in &fds {
                self.import(vmm, &mut inner, fd)?;
            }
            Ok(())
        });
        let cpu = match inner.cpu {
            Cpu::None => 0,
            Cpu::Import => usize::try_from(self.base).unwrap_or(0),
            Cpu::Mapped(p) => p.as_ptr() as usize,
        };
        self.cpu_base.store(cpu, Ordering::Relaxed);
        #[allow(clippy::cast_possible_truncation)]
        self.mapped
            .store(inner.handles.len() as u32, Ordering::Release); // order: arena_mapped
        r
    }

    fn import(&self, vmm: &Vmm, inner: &mut ImportInner, fd: &OwnedFd) -> Result<()> {
        let off = inner.handles.len() * CHUNK;
        let at = self.base + off as u64;
        let mut h: AllocHandle = 0;
        // SAFETY: the handle is the descriptor, passed as a pointer-sized integer; the import takes
        // its own reference.
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
        // SAFETY: inside the reserved range, unmapped there so far.
        if unsafe { (vmm.map)(at, CHUNK, 0, h, 0) } != SUCCESS {
            // SAFETY: imported above.
            unsafe { (vmm.release)(h) };
            return Err(Error::Unsupported("cuMemMap failed".into()));
        }
        inner.handles.push(h);
        ledger::made(Held::Import(at), CHUNK);
        if access(
            vmm,
            at,
            location(Place::Device, self.device),
            MEM_ACCESS_READ,
        ) != SUCCESS
        {
            return Err(Error::Unsupported("cuMemSetAccess failed".into()));
        }
        if let Place::HostNuma(_) = self.place {
            self.reach_from_cpu(vmm, inner, fd, off)?;
        }
        Ok(())
    }

    /// The first chunk picks the way; the rest follow it, so every chunk is at one CPU base.
    fn reach_from_cpu(
        &self,
        vmm: &Vmm,
        inner: &mut ImportInner,
        fd: &OwnedFd,
        off: usize,
    ) -> Result<()> {
        let at = self.base + off as u64;
        if !matches!(inner.cpu, Cpu::Mapped(_)) {
            let rc = access(vmm, at, location(self.place, self.device), MEM_ACCESS_READ);
            if rc == SUCCESS {
                inner.cpu = Cpu::Import;
                return Ok(());
            }
            if rc != NOT_SUPPORTED || matches!(inner.cpu, Cpu::Import) {
                return Err(Error::Unsupported(format!(
                    "cuMemSetAccess for the host failed with {rc}"
                )));
            }
            // SAFETY: a fresh reservation; no existing memory is affected.
            let p = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    self.size,
                    libc::PROT_NONE,
                    libc::MAP_PRIVATE | libc::MAP_ANONYMOUS | libc::MAP_NORESERVE,
                    -1,
                    0,
                )
            };
            let Some(p) = NonNull::new(p.cast::<u8>()).filter(|_| p != libc::MAP_FAILED) else {
                return Err(Error::io(
                    "mmap",
                    &self.endpoint,
                    io::Error::last_os_error(),
                ));
            };
            inner.cpu = Cpu::Mapped(p);
        }
        let Cpu::Mapped(cpu) = inner.cpu else {
            return Ok(());
        };
        // SAFETY: off + CHUNK is inside the reservation made above.
        let at_cpu = unsafe { cpu.as_ptr().add(off) };
        // SAFETY: over this chunk's part of that reservation.
        let p = unsafe {
            libc::mmap(
                at_cpu.cast(),
                CHUNK,
                libc::PROT_READ,
                libc::MAP_SHARED | libc::MAP_FIXED,
                fd.as_raw_fd(),
                0,
            )
        };
        if p == libc::MAP_FAILED {
            return Err(Error::io(
                "mmap",
                &self.endpoint,
                io::Error::last_os_error(),
            ));
        }
        Ok(())
    }

    pub fn device_ptr(&self, off: u64) -> u64 {
        self.base + off
    }

    pub fn ordinal(&self) -> i32 {
        self.ordinal
    }

    pub fn id(&self) -> u64 {
        self.id
    }

    /// The bytes imported so far, from offset 0.
    pub fn mapped_bytes(&self) -> usize {
        self.mapped.load(Ordering::Acquire) as usize * CHUNK // order: arena_mapped
    }

    /// The CPU's address of offset 0, for a `HOST_NUMA` arena once a chunk is imported. Each
    /// later chunk lands at its offset from it.
    pub fn host_base(&self) -> Option<*const u8> {
        let any = self.mapped.load(Ordering::Acquire) > 0; // order: arena_mapped
        let p = self.cpu_base.load(Ordering::Relaxed);
        (any && p != 0).then_some(p as *const u8)
    }

    /// The CPU's view of `len` bytes at `off`, if this arena has one and they are mapped.
    pub fn host(&self, off: u64, len: usize) -> Option<*const u8> {
        let inner = self.inner.lock();
        let mapped = (inner.handles.len() * CHUNK) as u64;
        if off.checked_add(len as u64)? > mapped {
            return None;
        }
        #[allow(clippy::cast_possible_truncation)]
        match inner.cpu {
            Cpu::None => None,
            Cpu::Import => Some((self.base + off) as usize as *const u8),
            // SAFETY: off + len is inside the mapped chunks, inside the reservation.
            Cpu::Mapped(p) => Some(unsafe { p.as_ptr().add(off as usize) }),
        }
    }
}

impl Drop for GpuArenaImport {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let inner = self.inner.lock();
        if let Cpu::Mapped(p) = inner.cpu {
            // SAFETY: reserved in reach_from_cpu with size; no view outlives self.
            unsafe { libc::munmap(p.as_ptr().cast(), self.size) };
        }
        let Ok((core, vmm)) = vmm::vmm() else {
            return;
        };
        core.with_context(self.ctx as *mut c_void, || {
            for (i, &h) in inner.handles.iter().enumerate() {
                let at = self.base + (i * CHUNK) as u64;
                ledger::undone(Held::Import(at), CHUNK);
                // SAFETY: mapped by import at this size.
                unsafe { (vmm.unmap)(at, CHUNK) };
                // SAFETY: imported by import.
                unsafe { (vmm.release)(h) };
            }
            // SAFETY: reserved by open at this size, now unmapped.
            unsafe { (vmm.address_free)(self.base, self.size) };
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(place: Place) -> GpuFrame {
        GpuFrame {
            id: 0x0123_4567_89ab_cdef,
            uuid: [7; 16],
            place,
            size: 8 * CHUNK,
            chunks: 3,
        }
    }

    #[test]
    fn a_gpu_trailer_reads_back_as_written() {
        for place in [Place::Device, Place::HostNuma(1)] {
            let f = frame(place);
            assert_eq!(GpuFrame::decode(&f.encode()), Some(f));
        }
    }

    #[test]
    fn a_gpu_trailer_no_publisher_writes_is_refused() {
        let mut t = frame(Place::Device).encode();
        t[24] = 9;
        assert_eq!(GpuFrame::decode(&t), None, "an unknown place");
        let bad = [
            GpuFrame {
                chunks: 9,
                ..frame(Place::Device)
            },
            GpuFrame {
                size: CHUNK + 1,
                ..frame(Place::Device)
            },
            GpuFrame {
                size: host_arena::MAX_GPU_ARENA_BYTES + CHUNK,
                ..frame(Place::Device)
            },
        ];
        for f in bad {
            assert_eq!(GpuFrame::decode(&f.encode()), None, "{f:?}");
        }
    }

    #[test]
    fn only_a_frame_whose_trailer_says_so_has_a_gpu_trailer() {
        let mut frame = vec![0u8; 256];
        let end = frame.len();
        frame[end - 64..end].copy_from_slice(&host_arena::trailer(None, false, 0, 0));
        assert!(host_arena::gpu_trailer(&frame).is_none());
        frame[end - 64..end].copy_from_slice(&host_arena::trailer(None, true, 0, 0));
        let t = host_arena::gpu_trailer(&frame).expect("a GPU trailer");
        assert_eq!(t.as_ptr(), frame[end - 128..].as_ptr());
        assert!(
            host_arena::gpu_trailer(&frame[end - 100..]).is_none(),
            "too short for both"
        );
    }

    /// A list in the frame ends before its trailers; one in the arena is read from its mapping.
    #[test]
    fn a_block_list_is_where_the_trailer_says() {
        let mut frame = vec![0u8; 256];
        let end = frame.len();
        let heap = vec![0u8; 4096];
        let put = |frame: &mut Vec<u8>, gpu, at: u64, n| {
            let t = host_arena::trailer(None, gpu, u32::try_from(at).unwrap(), n);
            frame[end - 64..end].copy_from_slice(&t);
        };
        for (gpu, behind) in [(true, 128), (false, 64)] {
            let fits = u32::try_from((end - behind) / host_arena::BLOCK_BYTES).unwrap();
            put(&mut frame, gpu, 0, fits);
            let l = host_arena::blocks(&frame, None).expect("a list up to the trailers");
            assert_eq!((l.as_ptr(), l.len()), (frame.as_ptr(), end - behind));
            put(&mut frame, gpu, 8, fits);
            assert!(
                host_arena::blocks(&frame, None).is_none(),
                "into the trailer"
            );
            put(&mut frame, gpu, 0, u32::MAX);
            assert!(host_arena::blocks(&frame, None).is_none());
        }
        // Descriptor offsets count UNITs: 4 is arena byte 64.
        let at = host_arena::HEAP_BASE + 4;
        put(&mut frame, false, at, 2);
        assert!(
            host_arena::blocks(&frame, None).is_none(),
            "an arena not mapped"
        );
        let l = host_arena::blocks(&frame, Some(&heap)).expect("in the arena");
        assert_eq!((l.as_ptr(), l.len()), (heap[64..].as_ptr(), 16));
        put(&mut frame, false, at + (4096 - 64) / host_arena::UNIT, 2);
        assert!(host_arena::blocks(&frame, Some(&heap)).is_none(), "past it");
        put(&mut frame, false, host_arena::GPU_BASE, 1);
        assert!(host_arena::blocks(&frame, Some(&heap)).is_none());
        frame[end - 64] = b'x';
        assert!(
            host_arena::blocks(&frame, Some(&heap)).is_none(),
            "no trailer"
        );
    }

    #[test]
    fn a_frame_past_the_arena_is_refused() {
        let Ok((core, _)) = vmm::vmm() else { return };
        if vmm::device_handle(core, 0).is_none() {
            return;
        }
        let m = GpuArenaImport::open("/dev/shm/flux.gpu.arena-none", 64 << 20, Place::Device, 0)
            .unwrap();
        assert!(matches!(m.reach(3), Err(Error::Mismatch(_))));
        assert!(matches!(m.reach(1), Err(Error::NotReady(_))));
    }
}
