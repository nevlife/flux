//! Backing memory of one channel, heap or POSIX shm. Mechanics only; naming, creation and
//! unlinking policy is in `discovery`.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use std::alloc::{self, Layout};
use std::os::fd::RawFd;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::sync::{Arc, Mutex, OnceLock, PoisonError};

use crate::discovery::SignpostMap;
use crate::error::{Error, Result};
use crate::gpu::DevicePayload;
use crate::layout::{
    ControlHeader, DEFAULT_MAX_SUBSCRIBERS, HeaderConfig, INIT_READY, MAX_AUX_SIZE, MAX_SLOT_COUNT,
    MAX_SUBSCRIBERS, PAYLOAD_ALIGN, SegmentLayout, SlotHeader, StorageKind, layout_config_ok,
};
use crate::qos::MemoryPolicy;
use crate::sys::{self, LiveFd};

/// (dev, ino) of a shm object. Unlink and recreate binds a name to another object.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct SegmentId {
    pub dev: u64,
    pub ino: u64,
}

enum Backing {
    Heap(Layout),
    Shm,
}

/// The memory alone, apart from the publisher's claim on the name: a loaned array can outlive
/// its publisher without keeping the publisher alive to subscribers.
struct Mapping {
    base: NonNull<u8>,
    bytes: usize,
    backing: Backing,
}

// SAFETY: as Segment.
unsafe impl Send for Mapping {}
// SAFETY: as Segment.
unsafe impl Sync for Mapping {}

impl Drop for Mapping {
    fn drop(&mut self) {
        match self.backing {
            // SAFETY: allocated in create_heap with this layout; no reference outlives self.
            Backing::Heap(l) => unsafe { alloc::dealloc(self.base.as_ptr(), l) },
            // SAFETY: mapped by discovery with self.bytes; no reference outlives self.
            Backing::Shm => unsafe { sys::munmap(self.base, self.bytes) },
        }
    }
}

/// Keeps a segment's memory mapped, nothing more.
pub struct Mapped(#[allow(dead_code)] Arc<Mapping>);

pub struct Segment {
    base: NonNull<u8>,
    bytes: usize,
    layout: SegmentLayout,
    /// As checked at attach; the mapping's copy may have changed since.
    header: HeaderConfig,
    map: Arc<Mapping>,
    /// Publisher only: the segment fd holding a read lock on `PUBLISHER_LOCK_BYTE`.
    pub(crate) publisher: Option<LiveFd>,
    /// Publisher only: the signpost, holding `SIGNPOST_PUBLISHER_BYTE`. Dropped after the
    /// segment is unlinked.
    pub(crate) signpost: Option<SignpostMap>,
    name: String,
    id: SegmentId,
    payload_readonly: bool,
    /// The union of what the subscriptions sharing the mapping asked for.
    pages_committed: AtomicBool,
    pages_locked: AtomicBool,
    /// What makes the payload device-addressable, made once for every channel on this mapping.
    device: OnceLock<DevicePayload>,
    device_init: Mutex<()>,
}

// SAFETY: the mapping is shared memory accessed through atomics and the ticket/borrow protocol;
// nothing in Segment is tied to the creating thread.
unsafe impl Send for Segment {}
// SAFETY: as above; &Segment only hands out atomics and raw payload pointers.
unsafe impl Sync for Segment {}

impl Segment {
    pub fn create_heap(
        slot_size: u32,
        slot_count: u32,
        aux_size: u32,
        fingerprint: u64,
    ) -> Result<Segment> {
        let layout = checked_layout(slot_size, slot_count, DEFAULT_MAX_SUBSCRIBERS, aux_size)?;
        let bytes = layout.total_bytes();
        let alloc_layout = Layout::from_size_align(bytes, PAYLOAD_ALIGN)
            .map_err(|e| Error::InvalidArgument(e.to_string()))?;
        // SAFETY: bytes > 0, since every layout has a control header.
        let p = unsafe { alloc::alloc_zeroed(alloc_layout) };
        let Some(base) = NonNull::new(p) else {
            return Err(Error::io(
                "alloc",
                "heap channel",
                std::io::Error::from(std::io::ErrorKind::OutOfMemory),
            ));
        };
        // SAFETY: base is a fresh zeroed allocation of total_bytes, aligned to a page.
        let header = unsafe { init_header(base, layout, fingerprint, None) };
        Ok(Segment::adopt(
            base,
            bytes,
            header,
            Backing::Heap(alloc_layout),
            String::new(),
            SegmentId::default(),
        ))
    }

    /// `header` passed `layout_config_ok` and fits `bytes`.
    pub(crate) fn adopt_shm(
        base: NonNull<u8>,
        bytes: usize,
        header: HeaderConfig,
        name: String,
        id: SegmentId,
    ) -> Segment {
        let seg = Segment::adopt(base, bytes, header, Backing::Shm, name, id);
        seg.pin_control();
        seg
    }

    /// Whatever the `MemoryPolicy`, the control plane stays resident: a swapped-out page of it
    /// (zram on Jetson) is a major fault in take, and in `futex_wake`, which looks up the page of
    /// a shared word. Best effort: a refusal (`RLIMIT_MEMLOCK`) costs only that. The payload is
    /// left out of core dumps, where it is the bulk and rarely the evidence.
    fn pin_control(&self) {
        let control = self.layout.payload_offset().min(self.bytes);
        // SAFETY: the start of this mapping.
        unsafe { libc::mlock(self.base.as_ptr().cast(), control) };
        if control < self.bytes {
            // SAFETY: control < bytes keeps the range inside the mapping.
            let payload = unsafe { self.base.add(control) };
            // SAFETY: as above; only the dump flag of the range changes.
            unsafe {
                libc::madvise(
                    payload.as_ptr().cast(),
                    self.bytes - control,
                    libc::MADV_DONTDUMP,
                )
            };
        }
        #[cfg(debug_assertions)]
        sys::dump_shared_mappings();
    }

    fn adopt(
        base: NonNull<u8>,
        bytes: usize,
        header: HeaderConfig,
        backing: Backing,
        name: String,
        id: SegmentId,
    ) -> Segment {
        Segment {
            base,
            bytes,
            layout: header.layout(),
            header,
            map: Arc::new(Mapping {
                base,
                bytes,
                backing,
            }),
            publisher: None,
            signpost: None,
            name,
            id,
            payload_readonly: false,
            pages_committed: AtomicBool::new(false),
            pages_locked: AtomicBool::new(false),
            device: OnceLock::new(),
            device_init: Mutex::new(()),
        }
    }

    pub fn ctrl(&self) -> &ControlHeader {
        // SAFETY: the mapping starts with an initialized ControlHeader and outlives &self.
        unsafe { self.base.cast::<ControlHeader>().as_ref() }
    }

    fn slot_ptr(&self, i: u32) -> NonNull<u8> {
        debug_assert!(i < self.layout.slot_count);
        let off = self.layout.slots_offset() + i as usize * self.layout.slot_stride();
        // SAFETY: i < slot_count keeps the offset inside the mapping.
        unsafe { self.base.add(off) }
    }

    pub fn slot(&self, i: u32) -> &SlotHeader {
        // SAFETY: slot_ptr points at an initialized SlotHeader inside the mapping.
        unsafe { self.slot_ptr(i).cast::<SlotHeader>().as_ref() }
    }

    /// Number `index`'s reader counter in slot `i`. Both are this process's own values: a slot
    /// below `slot_count` and a number below `max_subscribers`, never one read from the mapping.
    pub fn reader(&self, i: u32, index: u32) -> &AtomicU32 {
        debug_assert!(index < self.layout.max_subscribers);
        let off = size_of::<SlotHeader>() + index as usize * size_of::<AtomicU32>();
        // SAFETY: the counters follow the slot header inside the slot stride, max_subscribers of
        // them.
        let p = unsafe { self.slot_ptr(i).add(off) };
        // SAFETY: a zero-initialized u32, valid as AtomicU32 for as long as self.
        unsafe { p.cast::<AtomicU32>().as_ref() }
    }

    /// Reader counters of slot `i`, one per subscriber number.
    pub fn readers(&self, i: u32) -> &[AtomicU32] {
        // SAFETY: the counters follow the slot header inside the slot stride.
        let p = unsafe { self.slot_ptr(i).add(size_of::<SlotHeader>()) };
        // SAFETY: max_subscribers zero-initialized u32s, valid as AtomicU32, live as long as self.
        unsafe {
            std::slice::from_raw_parts(
                p.cast::<AtomicU32>().as_ptr(),
                self.layout.max_subscribers as usize,
            )
        }
    }

    /// How many subscriber numbers have ever been taken, at most `max_subscribers`: the counters a
    /// claim and the wake gate read. `SeqCst` where it pairs with a subscriber's `raise_readers`.
    pub fn readers_in_use(&self, order: Ordering) -> usize {
        (self.ctrl().readers_high.load(order) as usize).min(self.layout.max_subscribers as usize)
    }

    /// Before number `index` borrows or waits for the first time.
    pub fn raise_readers(&self, index: u32) {
        self.ctrl()
            .readers_high
            .fetch_max(index + 1, Ordering::SeqCst);
    }

    /// Number `index`'s count of waits, `index` below `max_subscribers` as in `reader`.
    pub fn waiter(&self, index: u32) -> &AtomicI32 {
        debug_assert!(index < self.layout.max_subscribers);
        let off = SegmentLayout::WAITERS_OFFSET + index as usize * size_of::<AtomicI32>();
        // SAFETY: the counts follow the control header, max_subscribers of them.
        let p = unsafe { self.base.add(off) };
        // SAFETY: a zero-initialized i32, valid as AtomicI32 for as long as self.
        unsafe { p.cast::<AtomicI32>().as_ref() }
    }

    /// Waits of each subscriber number. With `ControlHeader::waiters`, the publisher's wake gate.
    pub fn waiters(&self) -> &[AtomicI32] {
        // SAFETY: the counters follow the control header inside the mapping.
        let p = unsafe { self.base.add(SegmentLayout::WAITERS_OFFSET) };
        // SAFETY: max_subscribers zero-initialized i32s, valid as AtomicI32, live as long as self.
        unsafe {
            std::slice::from_raw_parts(
                p.cast::<AtomicI32>().as_ptr(),
                self.layout.max_subscribers as usize,
            )
        }
    }

    /// Takes back what a dead holder of subscriber number `index` left: its borrows and its
    /// share of the parked count. The caller holds that number's lock.
    pub fn clear_reader(&self, index: u32) {
        for s in 0..self.layout.slot_count {
            self.reader(s, index).store(0, Ordering::Release);
        }
        self.waiter(index).store(0, Ordering::Release);
    }

    /// The slot the commit of `ticket` wrote, as the publisher recorded it: a hint any process can
    /// write, so a take checks the slot's own ticket before trusting it.
    pub fn index_entry(&self, ticket: u64) -> &AtomicU32 {
        #[allow(clippy::cast_possible_truncation)]
        let i = (ticket % self.layout.index_len() as u64) as usize;
        // SAFETY: the index follows the waiter counts, index_len u32s, zero-initialized.
        let p = unsafe {
            self.base
                .add(self.layout.index_offset() + i * size_of::<AtomicU32>())
        };
        // SAFETY: an aligned u32 inside the mapping, valid as AtomicU32 for as long as self.
        unsafe { p.cast::<AtomicU32>().as_ref() }
    }

    /// False for a device-backed segment, whose payload is not in this mapping.
    pub fn host_payload(&self) -> bool {
        self.header.host_payload()
    }

    pub(crate) fn header(&self) -> &HeaderConfig {
        &self.header
    }

    /// Made by `init` on first use, then shared by every channel on this mapping.
    pub(crate) fn device_payload(
        &self,
        init: impl FnOnce() -> Result<DevicePayload>,
    ) -> Result<&DevicePayload> {
        if let Some(d) = self.device.get() {
            return Ok(d);
        }
        let _g = self
            .device_init
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if let Some(d) = self.device.get() {
            return Ok(d);
        }
        let d = init()?;
        Ok(self.device.get_or_init(|| d))
    }

    /// Start of slot `i`'s payload, `payload_stride` apart. Outside the mapping when the payload
    /// is on the device: only a host segment's may be dereferenced.
    pub fn payload(&self, i: u32) -> *mut u8 {
        debug_assert!(i < self.layout.slot_count);
        let off = self.layout.payload_offset() + i as usize * self.layout.payload_stride();
        self.base.as_ptr().wrapping_add(off)
    }

    /// Start of slot `i`'s aux region, `aux_size` bytes in this mapping whatever the payload is.
    pub fn aux(&self, i: u32) -> *mut u8 {
        debug_assert!(i < self.layout.slot_count);
        let off = self.layout.aux_offset() + i as usize * self.layout.aux_stride();
        self.base.as_ptr().wrapping_add(off)
    }

    /// The whole mapping, for a fuzz target to overwrite.
    #[cfg(any(test, feature = "fuzz"))]
    pub(crate) fn raw(&self) -> (NonNull<u8>, usize) {
        (self.base, self.bytes)
    }

    /// The same memory as another process would see it: a second view with no publisher role.
    #[cfg(test)]
    pub(crate) fn alias(&self) -> Segment {
        Segment {
            base: self.base,
            bytes: self.bytes,
            layout: self.layout,
            header: self.header,
            map: Arc::clone(&self.map),
            publisher: None,
            signpost: None,
            name: self.name.clone(),
            id: self.id,
            payload_readonly: self.payload_readonly,
            pages_committed: AtomicBool::new(false),
            pages_locked: AtomicBool::new(false),
            device: OnceLock::new(),
            device_init: Mutex::new(()),
        }
    }

    pub fn layout(&self) -> &SegmentLayout {
        &self.layout
    }
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn id(&self) -> SegmentId {
        self.id
    }
    pub fn is_heap(&self) -> bool {
        matches!(self.map.backing, Backing::Heap(_))
    }
    pub fn mapped(&self) -> Mapped {
        Mapped(Arc::clone(&self.map))
    }
    pub fn is_publisher(&self) -> bool {
        self.publisher.is_some()
    }
    pub(crate) fn publisher_fd(&self) -> Option<RawFd> {
        self.publisher.as_ref().map(LiveFd::raw)
    }

    /// Skipped when the page size does not divide `PAYLOAD_ALIGN` or mprotect refuses.
    pub(crate) fn protect_payload(&mut self) {
        // SAFETY: no preconditions.
        let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
        let Ok(page) = usize::try_from(page) else {
            return;
        };
        if page == 0 || !PAYLOAD_ALIGN.is_multiple_of(page) {
            return;
        }
        let off = self.layout.payload_offset();
        // SAFETY: off < bytes keeps the pointer inside the mapping.
        let payload = unsafe { self.base.add(off) };
        // SAFETY: page-aligned range inside the mapping; no &mut into the payload exists here.
        let r =
            unsafe { libc::mprotect(payload.as_ptr().cast(), self.bytes - off, libc::PROT_READ) };
        self.payload_readonly = r == 0;
    }

    pub fn payload_readonly(&self) -> bool {
        self.payload_readonly
    }
    pub fn pages_committed(&self) -> bool {
        self.pages_committed.load(Ordering::Relaxed)
    }
    pub fn pages_locked(&self) -> bool {
        self.pages_locked.load(Ordering::Relaxed)
    }

    /// Adds to what earlier calls did; never undoes it. Callers sharing a mapping serialize.
    pub(crate) fn apply(&self, mem: MemoryPolicy) -> Result<()> {
        if mem.lock && !self.pages_locked() {
            self.lock_pages()
        } else if mem.precommit && !self.pages_committed() {
            self.commit_pages()
        } else {
            Ok(())
        }
    }

    // MADV_POPULATE_* rather than storing into every page: a store into a shared mapping is
    // visible to peers, and on a full tmpfs it is a SIGBUS instead of an error.
    fn commit_pages(&self) -> Result<()> {
        const MADV_POPULATE_READ: libc::c_int = 22;
        const MADV_POPULATE_WRITE: libc::c_int = 23;
        // A read-only payload refuses POPULATE_WRITE, and a read fault is enough to make it resident.
        let split = if self.payload_readonly {
            self.layout.payload_offset()
        } else {
            self.bytes
        };
        // SAFETY: the range is inside the mapping.
        if unsafe { libc::madvise(self.base.as_ptr().cast(), split, MADV_POPULATE_WRITE) } != 0 {
            return Err(Error::io(
                "madvise(MADV_POPULATE_WRITE), needs Linux 5.14",
                &self.name,
                std::io::Error::last_os_error(),
            ));
        }
        if split < self.bytes {
            // SAFETY: split < bytes keeps the range inside the mapping.
            let rest = unsafe { self.base.add(split) };
            // SAFETY: the range is inside the mapping.
            if unsafe {
                libc::madvise(rest.as_ptr().cast(), self.bytes - split, MADV_POPULATE_READ)
            } != 0
            {
                return Err(Error::io(
                    "madvise(MADV_POPULATE_READ)",
                    &self.name,
                    std::io::Error::last_os_error(),
                ));
            }
        }
        self.pages_committed.store(true, Ordering::Relaxed);
        Ok(())
    }

    fn lock_pages(&self) -> Result<()> {
        // SAFETY: the range is the whole mapping.
        if unsafe { libc::mlock(self.base.as_ptr().cast(), self.bytes) } != 0 {
            return Err(Error::io(
                "mlock (RLIMIT_MEMLOCK must cover the segment)",
                &self.name,
                std::io::Error::last_os_error(),
            ));
        }
        self.pages_locked.store(true, Ordering::Relaxed);
        self.pages_committed.store(true, Ordering::Relaxed);
        Ok(())
    }
}

impl Drop for Segment {
    fn drop(&mut self) {
        // Single publisher: its segment ends with it. A forked child's copy is not the publisher.
        if let Some(fd) = &self.publisher
            && fd.owned_here()
            && !self.name.is_empty()
        {
            sys::shm_unlink(&self.name);
        }
        // Fields drop in order, `map` (the munmap) before `device`: the fd server's thread is
        // stopped and the registration undone here, while the memory is still mapped.
        drop(self.device.take());
    }
}

pub(crate) fn checked_layout(
    slot_size: u32,
    slot_count: u32,
    max_subscribers: u32,
    aux_size: u32,
) -> Result<SegmentLayout> {
    if !layout_config_ok(slot_size, slot_count, max_subscribers, aux_size) {
        return Err(Error::InvalidArgument(format!(
            "slot_size must be >= 1, slot_count in 1..={MAX_SLOT_COUNT}, max_subscribers in \
             1..={MAX_SUBSCRIBERS} and aux_size at most {MAX_AUX_SIZE}"
        )));
    }
    Ok(SegmentLayout {
        slot_size,
        slot_count,
        max_subscribers,
        aux_size,
    })
}

/// Writes the control header and publishes `INIT_READY` last, and returns what it wrote. Zeroes only
/// the headers: shm from fallocate is already zero, and touching the payload would fault in every
/// page.
///
/// # Safety
/// `base` must point at `layout.payload_offset()` writable bytes, and `total_bytes()` for a host
/// payload, that no other process reads yet.
pub(crate) unsafe fn init_header(
    base: NonNull<u8>,
    layout: SegmentLayout,
    fingerprint: u64,
    device: Option<(u32, [u8; 16])>,
) -> HeaderConfig {
    // SAFETY: the header region is inside the allocation and nobody else reads it yet.
    unsafe { base.write_bytes(0, layout.payload_offset()) };
    let ctrl = base.cast::<ControlHeader>();
    // SAFETY: as above.
    unsafe {
        let kind = if device.is_some() {
            StorageKind::CudaIpc
        } else {
            StorageKind::HostInline
        };
        let mut h = ControlHeader::new(layout, fingerprint, kind);
        if let Some((ordinal, uuid)) = device {
            h.device_id = ordinal;
            h.device_uuid = uuid;
        }
        ctrl.write(h);
    }
    // SAFETY: just written, and no other process reads it yet.
    let header = unsafe { HeaderConfig::read(ctrl.as_ptr()) };
    // SAFETY: just written.
    let ctrl = unsafe { ctrl.as_ref() };
    ctrl.init_state
        .store(INIT_READY, std::sync::atomic::Ordering::Release);
    header
}

#[cfg(test)]
mod tests {
    use super::Segment;

    /// The index checks are `debug_assert!`s: a test build that left them out would let the next
    /// wrong index pass silently.
    #[test]
    #[cfg_attr(
        not(debug_assertions),
        ignore = "release builds leave the debug checks out"
    )]
    fn the_debug_checks_are_built_into_the_test_profile() {
        let seg = Segment::create_heap(64, 2, 0, 0).unwrap();
        let seg = std::panic::AssertUnwindSafe(&seg);
        let past = |f: fn(&Segment)| std::panic::catch_unwind(|| f(*seg)).is_err();
        assert!(past(|s| {
            s.slot(2);
        }));
        assert!(past(|s| {
            s.payload(2);
        }));
        assert!(past(|s| {
            s.aux(2);
        }));
        assert!(!past(|s| {
            s.slot(1);
        }));
    }
}
