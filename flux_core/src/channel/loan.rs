//! A claimed slot and its way into the arena.

use std::ffi::c_void;
use std::sync::Arc;

use crate::arena::{self, BLOCK_BYTES, GPU_TRAILER_BYTES, TRAILER_BYTES};
use crate::gpu::{self, Fenced, Seam, Stream};
use crate::layout::{DType, FrameMeta, MAX_DIMS, SegmentLayout};
use crate::segment::Mapped;
use crate::trace::{self, event};

use super::{ChannelShared, Owned, Published};

/// A claimed slot. Dropped without commit, it is aborted, and the frame the slot held before is
/// gone either way, since `data_mut` aliased it. It may be committed or dropped on another thread:
/// both touch only the shared state, as the bindings' loans do.
pub struct WriteSlot {
    pub(super) sh: Arc<ChannelShared>,
    pub(super) slot: u32,
    pub(super) meta: FrameMeta,
    pub(super) done: bool,
}

/// A loan's handle on the process arena: plain data, so a binding copies it with its writer.
/// Arena memory it hands out belongs to the loan's frame and is freed when its slot is claimed
/// again. Use it only while the loan lives. `base` is null until the arena is made, by the first
/// `alloc`. `gpu` when the channel declared a stream, so `alloc_device` may succeed.
#[derive(Clone, Copy)]
pub struct Heap {
    pub(super) sh: *const ChannelShared,
    pub(super) slot: u32,
    pub base: *mut u8,
    pub size: usize,
    pub gpu: bool,
}

impl Heap {
    /// What a binding keeps: the channel and the slot.
    pub fn raw(&self) -> (*const c_void, u32) {
        (self.sh.cast(), self.slot)
    }

    /// The handle `raw` and the arena fields describe.
    pub fn from_raw(
        channel: *const c_void,
        slot: u32,
        base: *mut u8,
        size: usize,
        gpu: bool,
    ) -> Heap {
        Heap {
            sh: channel.cast(),
            slot,
            base,
            size,
            gpu,
        }
    }

    /// `bytes` of the GPU arena on the channel stream's device: their device address and arena
    /// offset. Write them on that stream; the commit waits for it.
    ///
    /// # Safety
    /// The loan this came from is live.
    pub unsafe fn alloc_device(&self, bytes: usize, align: usize) -> Option<(u64, u64)> {
        // SAFETY: the live loan holds the channel.
        let sh = unsafe { &*self.sh };
        let len = u32::try_from(bytes).ok()?;
        let a = sh.gpu_arena()?;
        let off = a.alloc(bytes, align)?;
        sh.own(self.slot, Owned::Gpu(a, off, len));
        Some((a.device_ptr(off), off))
    }

    /// `bytes` of arena memory and their arena offset.
    ///
    /// # Safety
    /// The loan this came from is live.
    pub unsafe fn alloc(&self, bytes: usize, align: usize) -> Option<(*mut u8, u64)> {
        let len = u32::try_from(bytes).ok()?;
        let a = arena::arena()?;
        let off = a.alloc(bytes, align)?;
        // SAFETY: the live loan holds the channel.
        unsafe { &*self.sh }.own(self.slot, Owned::Block(off, len));
        Some((a.ptr(off), off))
    }

    /// Hands the frame `bytes` at `p`, arena memory allocated elsewhere (under the malloc hook),
    /// with what frees them. None, and `release` not kept, unless they are all in the arena.
    ///
    /// # Safety
    /// The loan this came from is live. `release(ctx)` must be callable once, on any thread, and
    /// nothing else may free `p` first.
    pub unsafe fn adopt(
        &self,
        p: *const u8,
        bytes: usize,
        release: unsafe extern "C" fn(*mut c_void),
        ctx: *mut c_void,
    ) -> Option<u64> {
        let len = u32::try_from(bytes).ok()?;
        let off = arena::existing()?.offset_of(p, bytes)?;
        // SAFETY: the live loan holds the channel.
        unsafe { &*self.sh }.own(self.slot, Owned::Adopted(release, ctx, off, len));
        Some(off)
    }
}

impl WriteSlot {
    /// Empty when the payload is device memory; write it through `device_ptr`.
    pub fn data_mut(&mut self) -> &mut [u8] {
        if !self.sh.seg.host_payload() {
            return &mut [];
        }
        // SAFETY: the claim (ticket claimed, every counter 0) makes this the only access to the slot.
        unsafe { std::slice::from_raw_parts_mut(self.sh.seg.payload(self.slot), self.capacity()) }
    }

    pub fn capacity(&self) -> usize {
        self.sh.seg.layout().slot_size as usize
    }

    /// The slot's whole aux region, `aux_size` bytes; `set_aux_len` says how many the frame uses.
    pub fn aux_mut(&mut self) -> &mut [u8] {
        // SAFETY: the claim makes this the only access to the slot's aux region.
        unsafe { &mut *self.aux_region() }
    }

    /// `aux_mut` as a raw slice, for a binding that hands it out more than once: no reference
    /// is made, so a later call leaves an earlier pointer valid.
    pub fn aux_region(&self) -> *mut [u8] {
        let len = self.sh.seg.layout().aux_size as usize;
        std::ptr::slice_from_raw_parts_mut(self.sh.seg.aux(self.slot), len)
    }

    /// A length past `aux_size` makes the commit return `TooLarge`.
    pub fn set_aux_len(&mut self, n: usize) {
        self.meta.aux_len = u16::try_from(n).unwrap_or(u16::MAX);
    }

    /// For memory that outlives the commit, such as an array a binding handed out.
    pub fn mapped(&self) -> Mapped {
        self.sh.seg.mapped()
    }

    /// The slot's device address, 0 when the channel declared no stream.
    pub fn device_ptr(&self) -> u64 {
        self.sh.device_ptr(self.slot)
    }

    /// The channel's stream, which the commit waits for: queue the producing work on it.
    pub fn stream(&self) -> Stream {
        self.sh.stream()
    }

    pub fn host_addressable(&self) -> bool {
        self.sh.seg.host_payload()
    }

    /// The loan's way into the arena; None with the arena off, on a device payload, and in a
    /// forked child.
    pub fn heap(&self) -> Option<Heap> {
        if !self.sh.seg.host_payload() || self.sh.inherited() {
            return None;
        }
        Some(Heap {
            sh: Arc::as_ptr(&self.sh),
            slot: self.slot,
            base: arena::existing().map_or(std::ptr::null_mut(), arena::Arena::base),
            size: arena::configured()?,
            gpu: self.sh.gpu.as_ref().is_some_and(|g| g.stream.declared()),
        })
    }

    pub fn commit(mut self) -> Published {
        let (published, end) = self.settle();
        self.end(end);
        published
    }

    pub(super) fn settle(&self) -> (Published, i32) {
        if self.sh.inherited() {
            return (Published::NotPublisher, trace::LOAN_NOT_PUBLISHER);
        }
        if !fits(&self.meta, self.sh.seg.layout()) {
            self.sh.abort(self.slot);
            return (Published::TooLarge, trace::LOAN_TOO_LARGE);
        }
        match self.sh.fence(Seam::Commit) {
            Fenced::Done => {}
            // Not an abort: that would hand the slot to the next claim while the producing kernel
            // may still write it. The claim and the mapping stay for the process's life.
            Fenced::Failed => {
                std::mem::forget(Arc::clone(&self.sh));
                return fence_failed(trace::LOAN_FENCE_FAILED, trace::LOAN_GPU_WEDGED);
            }
            // Aborted once the kernel is done, by a later claim.
            Fenced::Late(late) => {
                self.sh.defer(late, self.slot, 0, Seam::Commit, false);
                return fence_failed(trace::LOAN_FENCE_FAILED, trace::LOAN_GPU_WEDGED);
            }
        }
        self.sh.finish_commit(self.slot, &self.meta);
        (Published::Ok, trace::LOAN_COMMITTED)
    }

    pub(super) fn end(&mut self, result: i32) {
        self.done = true;
        event!(flux_trace_loan_end(
            self.sh.trace_handle(),
            self.slot,
            result
        ));
    }

    /// Shrinks a one-dimensional loan to `nbytes`. Any other rank would need the shape restated.
    pub fn commit_len(mut self, nbytes: usize) -> Published {
        let mut nbytes = nbytes;
        // Claiming the slot freed what its last frame owned, so anything owned now is this one's.
        if self.sh.may_own() {
            let Some(n) = self.write_trailer(nbytes) else {
                self.sh.abort(self.slot);
                self.end(trace::LOAN_TOO_LARGE);
                return Published::TooLarge;
            };
            nbytes = n;
        }
        let m = &mut self.meta;
        let n = nbytes as u64;
        if m.ndim != 1
            || m.itemsize == 0
            || n > m.nbytes
            || !n.is_multiple_of(u64::from(m.itemsize))
        {
            self.sh.abort(self.slot);
            self.end(trace::LOAN_TOO_LARGE);
            return Published::TooLarge;
        }
        m.nbytes = n;
        m.shape[0] = n / u64::from(m.itemsize);
        self.commit()
    }
}

impl WriteSlot {
    /// Writes past the frame's `nbytes` the list of its blocks, the GPU arena's trailer when it
    /// has GPU blocks, then the host one. A list the slot has no room for goes to the arena, as a
    /// block of the frame. The frame's new length, `nbytes` when it owns no block; None when they
    /// do not fit.
    fn write_trailer(&self, nbytes: usize) -> Option<usize> {
        let cap = self.capacity();
        let at = nbytes.next_multiple_of(8);
        let base = self.sh.seg.payload(self.slot);
        // The one hold of `owned` a commit takes: what the frame owns, and its list written.
        let (host, gpu, blocks, list_at, end) = {
            let mut owned = self.sh.owned.lock();
            let Some(v) = owned.get_mut(self.slot as usize).filter(|v| !v.is_empty()) else {
                return Some(nbytes);
            };
            let gpu = v.iter().find_map(|o| match o {
                Owned::Gpu(a, ..) => Some(*a),
                _ => None,
            });
            let host = v.iter().any(|o| !matches!(o, Owned::Gpu(..)));
            let blocks = u32::try_from(v.len()).ok()?;
            let tail = if gpu.is_some() {
                GPU_TRAILER_BYTES
            } else {
                TRAILER_BYTES
            };
            let list = BLOCK_BYTES * v.len();
            if at + tail > cap {
                return None;
            }
            if at + list + tail <= cap {
                // SAFETY: at + list <= capacity, inside the slot.
                let dst = unsafe { base.add(at) };
                // SAFETY: list bytes at dst, which the claim makes ours.
                unsafe { write_list(v, dst) };
                (host, gpu, blocks, at as u64, at + list)
            } else {
                let len = u32::try_from(list).ok()?;
                let a = arena::arena()?;
                let off = a.alloc(list, 8)?;
                // SAFETY: list bytes of the arena, this frame's from here.
                unsafe { write_list(v, a.ptr(off)) };
                v.push(Owned::Block(off, len));
                (true, gpu, blocks, arena::HEAP_BASE + off, at)
            }
        };
        let host = if host { arena::existing() } else { None };
        if host.is_none() && gpu.is_none() {
            return None;
        }
        let tail = if gpu.is_some() {
            GPU_TRAILER_BYTES
        } else {
            TRAILER_BYTES
        };
        // SAFETY: end + tail <= capacity, checked under the lock.
        let dst = unsafe { base.add(end) };
        if let Some(g) = gpu {
            let t = g.frame().encode();
            // SAFETY: the first 64 of the tail bytes at dst.
            unsafe { std::ptr::copy_nonoverlapping(t.as_ptr(), dst, TRAILER_BYTES) };
        }
        #[allow(clippy::cast_possible_truncation)]
        let host_t = arena::trailer(host, gpu.is_some(), list_at as u32, blocks);
        // SAFETY: the last 64 of the tail bytes at dst.
        let last = unsafe { dst.add(tail - TRAILER_BYTES) };
        // SAFETY: as above.
        unsafe { std::ptr::copy_nonoverlapping(host_t.as_ptr(), last, TRAILER_BYTES) };
        Some(end + tail)
    }
}

/// Writes `blocks` at `dst` as a trailer's list.
///
/// # Safety
/// `dst` holds `BLOCK_BYTES` bytes for each block, and nothing else accesses them meanwhile.
unsafe fn write_list(blocks: &[Owned], dst: *mut u8) {
    for (i, o) in blocks.iter().enumerate() {
        let e = o.entry();
        // SAFETY: entry i is inside what the caller holds.
        let to = unsafe { dst.add(i * BLOCK_BYTES) };
        // SAFETY: as above.
        unsafe { std::ptr::copy_nonoverlapping(e.as_ptr(), to, e.len()) };
    }
}

impl Drop for WriteSlot {
    fn drop(&mut self) {
        if !self.done {
            self.sh.abort(self.slot);
            event!(flux_trace_loan_end(
                self.sh.trace_handle(),
                self.slot,
                trace::LOAN_ABORTED
            ));
        }
    }
}

/// A failed fence's outcome, and its trace reason: `failed`, or `wedged` once a sticky error
/// wedged the process's GPU.
pub(super) fn fence_failed(failed: i32, wedged: i32) -> (Published, i32) {
    if gpu::wedged().is_some() {
        (Published::GpuWedged, wedged)
    } else {
        (Published::FenceFailed, failed)
    }
}

/// Also run on every take, against a header any process could have written.
pub(super) fn fits(meta: &FrameMeta, layout: &SegmentLayout) -> bool {
    let Some(dt) = meta.dtype() else {
        return false;
    };
    if usize::from(meta.ndim) > MAX_DIMS || meta.itemsize != dt.size() {
        return false;
    }
    let n = meta
        .shape()
        .iter()
        .try_fold(u64::from(meta.itemsize), |n, &d| n.checked_mul(d));
    n == Some(meta.nbytes)
        && meta.nbytes <= u64::from(layout.slot_size)
        && u32::from(meta.aux_len) <= layout.aux_size
}

/// itemsize follows from dtype and ndim from the shape, so they cannot contradict each other. An
/// overflowing byte count becomes `u64::MAX`, which the commit refuses.
pub(super) fn meta_from(dt: DType, shape: &[u64]) -> FrameMeta {
    let mut m = FrameMeta {
        dtype: dt as u8,
        itemsize: dt.size(),
        ndim: u8::try_from(shape.len()).unwrap_or(u8::MAX),
        ..FrameMeta::default()
    };
    let mut n = u64::from(m.itemsize);
    for (to, &d) in m.shape.iter_mut().zip(shape) {
        *to = d;
    }
    for &d in shape {
        n = n.saturating_mul(d);
    }
    m.nbytes = n;
    m
}
