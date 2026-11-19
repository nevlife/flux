//! Image slots on an integrated GPU: each slot's payload is a page-locked allocation of its own,
//! exported once as a dmabuf and mapped over the slot's place in the segment mapping, by the
//! publisher and by every subscriber. `Segment::payload` and the registrations above it stay as
//! they are; an engine that takes a dmabuf reads the slot itself.
//!
//! The Jetson kernel oopses when two dmabufs of one nvmap buffer are released, and the driver packs
//! small `cuMemHostAlloc`s into one shared buffer, so a slot is used only when its allocation is a
//! mapping of its own, and it is exported once.

#![allow(
    clippy::disallowed_types,
    reason = "the count of a subscriber's mapped image slots; orderings tagged `order:`"
)]

use std::ffi::{c_int, c_void};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

use super::driver::{Driver, MEM_HOST_ALLOC_DEVICEMAP, MEM_RANGE_HANDLE_TYPE_DMA_BUF_FD, SUCCESS};
use super::fd::{FdServer, Mailbox, Share};
use crate::error::{Error, Result};
use crate::sys::{self, PiMutex};

/// The least an image slot's allocation may be: smaller ones share an nvmap buffer.
pub(crate) const IMAGE_SLOT_ALIGN: usize = 2 << 20;

/// Whether `[p, p + n)` is exactly one mapping in `/proc/self/maps`.
pub(crate) fn own_mapping(p: *const c_void, n: usize) -> bool {
    let Ok(maps) = std::fs::read_to_string("/proc/self/maps") else {
        return false;
    };
    let at = p as usize;
    maps.lines().any(|l| {
        let Some((lo, hi)) = l.split_whitespace().next().and_then(|r| r.split_once('-')) else {
            return false;
        };
        let (Ok(lo), Ok(hi)) = (usize::from_str_radix(lo, 16), usize::from_str_radix(hi, 16))
        else {
            return false;
        };
        lo == at && hi - lo == n.next_multiple_of(4096)
    })
}

/// Maps `fd` over `[at, at + n)` of a mapping this process holds.
fn overlay(fd: c_int, at: usize, n: usize, prot: c_int) -> Result<()> {
    // SAFETY: the range is inside the segment mapping, which MAP_FIXED replaces page for page.
    let p = unsafe {
        libc::mmap(
            at as *mut c_void,
            n,
            prot,
            libc::MAP_SHARED | libc::MAP_FIXED,
            fd,
            0,
        )
    };
    if p == libc::MAP_FAILED || p as usize != at {
        return Err(Error::io(
            "mmap",
            "an image slot's dmabuf",
            std::io::Error::last_os_error(),
        ));
    }
    Ok(())
}

/// The publisher's image slots: slot `s` is an allocation of `stride` bytes, mapped at
/// `base + s * stride`, its dmabuf handed out in order by the endpoint.
pub(crate) struct ImageExport {
    _server: FdServer,
    share: Arc<Share>,
    base: usize,
    stride: usize,
    ctx: usize,
    allocs: Mutex<Vec<usize>>,
    born: u32,
}

impl ImageExport {
    /// The first `ready` slots get their memory now; `base` is slot 0's payload.
    pub(crate) fn create(
        endpoint: &str,
        base: *mut u8,
        stride: usize,
        ready: u32,
        ordinal: i32,
    ) -> Result<ImageExport> {
        if stride < IMAGE_SLOT_ALIGN || !stride.is_multiple_of(IMAGE_SLOT_ALIGN) {
            return Err(Error::InvalidArgument(format!(
                "an image slot takes a multiple of {IMAGE_SLOT_ALIGN} bytes, not {stride}"
            )));
        }
        let drv = Driver::get();
        let core = drv
            .usable_core()
            .map_err(|why| Error::Unsupported(why.to_owned()))?;
        drv.host_export
            .as_ref()
            .map_err(|why| Error::Unsupported((*why).to_owned()))?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        sys::watch_fork();
        let share = Share::new(stride);
        let server = FdServer::start(endpoint, Arc::clone(&share))?;
        let e = ImageExport {
            _server: server,
            share,
            base: base as usize,
            stride,
            ctx: ctx as usize,
            allocs: Mutex::new(Vec::new()),
            born: sys::fork_gen(),
        };
        for s in 0..ready {
            e.back(s)?;
        }
        Ok(e)
    }

    /// Gives slot `slot` its memory; slots get it in order, each once.
    pub(crate) fn back(&self, slot: u32) -> Result<()> {
        let mut allocs = self.allocs.lock().unwrap_or_else(PoisonError::into_inner);
        if slot as usize != allocs.len() {
            return Err(Error::InvalidArgument(format!(
                "image slot {slot} given out of order, {} have memory",
                allocs.len()
            )));
        }
        let drv = Driver::get();
        let (Ok(core), Ok(he), Ok(reg)) = (drv.usable_core(), &drv.host_export, &drv.host_reg)
        else {
            return Err(Error::Unsupported(
                "no dmabuf export in this process".into(),
            ));
        };
        let n = self.stride;
        let made = core.with_context(self.ctx as *mut c_void, || -> Result<(usize, OwnedFd)> {
            let mut p: *mut c_void = std::ptr::null_mut();
            // SAFETY: valid out-pointer.
            if unsafe { (he.alloc)(&raw mut p, n, MEM_HOST_ALLOC_DEVICEMAP) } != SUCCESS {
                return Err(Error::Unsupported("cuMemHostAlloc failed".into()));
            }
            let free = |why: &str| {
                // SAFETY: allocated above, not exported.
                unsafe { (he.free)(p) };
                Err(Error::Unsupported(why.to_owned()))
            };
            if !own_mapping(p, n) {
                return free("the image slot's allocation shares an nvmap buffer");
            }
            let mut dev = 0;
            // SAFETY: p is a page-locked allocation of this context.
            if unsafe { (reg.device_pointer)(&raw mut dev, p, 0) } != SUCCESS {
                return free("cuMemHostGetDevicePointer failed");
            }
            let mut raw: c_int = -1;
            // SAFETY: a dmabuf fd is written as an int; the range is the whole allocation.
            let rc = unsafe {
                (he.range_handle)(
                    (&raw mut raw).cast(),
                    dev,
                    n,
                    MEM_RANGE_HANDLE_TYPE_DMA_BUF_FD,
                    0,
                )
            };
            if rc != SUCCESS || raw < 0 {
                return free("cuMemGetHandleForAddressRange failed");
            }
            // SAFETY: the export handed this process a new descriptor.
            let fd = unsafe { OwnedFd::from_raw_fd(raw) };
            // SAFETY: fcntl on a descriptor this process owns.
            unsafe { libc::fcntl(fd.as_raw_fd(), libc::F_SETFD, libc::FD_CLOEXEC) };
            Ok((p as usize, fd))
        })?;
        let (p, fd) = made;
        allocs.push(p);
        overlay(
            fd.as_raw_fd(),
            self.base + slot as usize * n,
            n,
            libc::PROT_READ | libc::PROT_WRITE,
        )?;
        self.share.push(fd);
        Ok(())
    }
}

impl ImageExport {
    /// Slot `slot`'s dmabuf, held for as long as the export.
    pub(crate) fn fd(&self, slot: u32) -> Option<c_int> {
        self.share.fd(slot)
    }
}

impl Drop for ImageExport {
    /// The overlays stay until the segment is unmapped; each holds its dmabuf, and so its pages.
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let drv = Driver::get();
        let (Ok(core), Ok(he)) = (drv.usable_core(), &drv.host_export) else {
            return;
        };
        let allocs = std::mem::take(
            self.allocs
                .get_mut()
                .unwrap_or_else(PoisonError::into_inner),
        );
        core.with_context(self.ctx as *mut c_void, || {
            for p in allocs {
                // SAFETY: allocated by back(), freed once.
                unsafe { (he.free)(p as *mut c_void) };
            }
        });
    }
}

/// A subscriber's image slots: each dmabuf, as it comes on the endpoint, mapped read-only over the
/// slot's place in this mapping.
pub(crate) struct ImageImport {
    mail: PiMutex<Mailbox>,
    base: usize,
    stride: usize,
    mapped: AtomicU32,
    fds: PiMutex<Vec<OwnedFd>>,
}

impl ImageImport {
    pub(crate) fn new(endpoint: &str, base: *mut u8, stride: usize) -> ImageImport {
        ImageImport {
            mail: PiMutex::new(Mailbox::new(endpoint, stride)),
            base: base as usize,
            stride,
            mapped: AtomicU32::new(0),
            fds: PiMutex::new(Vec::new()),
        }
    }

    /// Slot `slot`'s dmabuf once it is mapped, held for as long as the import.
    pub(crate) fn fd(&self, slot: u32) -> Option<c_int> {
        let fds = self.fds.lock();
        fds.get(slot as usize).map(AsRawFd::as_raw_fd)
    }

    /// Maps the slots up to `slot` that have come. `NotReady` when `slot` has not.
    pub(crate) fn reach(&self, slot: u32) -> Result<()> {
        // order: image_mapped
        if slot < self.mapped.load(Ordering::Acquire) {
            return Ok(());
        }
        let mut mail = self.mail.lock();
        let have = self.mapped.load(Ordering::Relaxed);
        if slot < have {
            return Ok(());
        }
        let got = mail.collect(have, slot + 1)?;
        let mut fds = self.fds.lock();
        let mut n = have;
        for fd in got {
            overlay(
                fd.as_raw_fd(),
                self.base + n as usize * self.stride,
                self.stride,
                libc::PROT_READ,
            )?;
            fds.push(fd);
            n += 1;
            self.mapped.store(n, Ordering::Release); // order: image_mapped
        }
        if slot >= n {
            return Err(Error::NotReady(
                "the publisher has not sent the image slot a frame is in".into(),
            ));
        }
        Ok(())
    }
}
