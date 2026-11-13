//! A borrowed frame.

#![allow(
    clippy::disallowed_types,
    reason = "the slot, reader and wake words; orderings tagged `order:`"
)]

use std::ffi::c_void;
use std::sync::Arc;
use std::sync::atomic::Ordering;

use crate::arena::{self};
use crate::gpu::arena::{GpuFrame, Place, import_for};
use crate::gpu::{Fenced, Seam, Stream, ledger};
use crate::layout::FrameMeta;
use crate::trace::event;

use super::ChannelShared;

/// How this subscriber reaches the fields of a frame on the side of `PCIe` its slot is not. The
/// GPU arena: `device` and `host` are addresses of its offset 0 and cover `size` bytes. `frame`
/// is the device address of the frame's own bytes.
#[derive(Clone, Copy, Debug)]
pub struct GpuHeap {
    /// 0 unless this subscriber's stream is on the arena's device.
    pub device: u64,
    /// Null for an arena in device memory.
    pub host: *const u8,
    pub size: usize,
    /// 0 unless this subscriber declared a stream.
    pub frame: u64,
    /// This subscriber's copy of each of the frame's blocks on the other side, `count` of them,
    /// valid while the view lives: a CPU subscriber's of device memory, a discrete GPU
    /// subscriber's of the host arena. Null otherwise.
    pub blocks: *const GpuBlock,
    pub count: usize,
}

impl Default for GpuHeap {
    fn default() -> GpuHeap {
        GpuHeap {
            device: 0,
            host: std::ptr::null(),
            size: 0,
            frame: 0,
            blocks: std::ptr::null(),
            count: 0,
        }
    }
}

/// A block of a frame and this subscriber's copy of it: its descriptor offset, its length, and
/// the copy's address, in host memory or (for a GPU subscriber) device memory.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GpuBlock {
    pub off: u64,
    pub len: u64,
    pub host: *const u8,
    pub device: u64,
}

/// A borrowed frame. While it lives, the publisher does not rewrite its slot. The header is the
/// copy `take` checked: another process can still write the one in shared memory.
pub struct FrameView {
    pub(super) sh: Arc<ChannelShared>,
    pub(super) slot: u32,
    pub(super) ticket: u64,
    pub(super) meta: FrameMeta,
}

impl FrameView {
    /// The arena the frame's trailer names, read-only. None for a frame without one.
    pub fn heap(&self) -> Option<&[u8]> {
        let data = self.data();
        let m = if let Some(m) = self.sh.arena_map.get() {
            m
        } else {
            let m = arena::map_for(data)?;
            self.sh.arena_map.get_or_init(|| m)
        };
        m.names(data).then(|| m.bytes())
    }

    /// How this subscriber reaches the frame's fields across `PCIe`. None for a CPU subscriber
    /// of a frame without GPU blocks, and while they cannot be imported or copied. Blocks on the
    /// other side are copied by the first call, and the view's later calls read the copy.
    pub fn gpu_heap(&self) -> Option<GpuHeap> {
        let data = self.data();
        let stream = self
            .sh
            .gpu
            .as_ref()
            .map(|g| &g.stream)
            .filter(|s| s.declared());
        let mut h = GpuHeap::default();
        if let Some(t) = arena::gpu_trailer(data) {
            let f = GpuFrame::decode(t)?;
            let m = self
                .sh
                .gpu_import
                .get_or_init(|| {
                    import_for(&f)
                        .inspect_err(|e| {
                            eprintln!("flux: a frame's GPU arena was not imported: {e}");
                        })
                        .ok()
                })
                .as_ref()?;
            // shortcut: a frame past the chunks imported fetches the rest here, a round trip on
            // the reader's thread; fetch them as the publisher grows once an executor is seen to
            // stall on it.
            if m.id() != f.id || m.reach(f.chunks).is_err() {
                return None;
            }
            if f.place == Place::Device && stream.is_none() {
                let list = arena::blocks(data, self.heap())?;
                return self.sh.copy_gpu_blocks(self.slot, self.ticket, m, list);
            }
            h.device = stream
                .filter(|s| s.device() == m.ordinal())
                .map_or(0, |_| m.device_ptr(0));
            h.host = m.host_base().unwrap_or(std::ptr::null());
            h.size = m.mapped_bytes();
        } else if stream.is_none() {
            return None;
        }
        if stream.is_some() {
            h.frame = self.device_ptr();
            // shortcut: an integrated GPU subscriber has no device address for host arena
            // fields; register the arena mapping once it is tried on such a board.
            if let Some(heap) = self.heap().filter(|_| self.sh.stages_to_device()) {
                let list = arena::blocks(data, Some(heap))?;
                let c = self
                    .sh
                    .copy_host_blocks(self.slot, self.ticket, heap, list)?;
                h.blocks = c.blocks;
                h.count = c.count;
            }
        }
        Some(h)
    }

    /// Empty when the payload is device memory and this subscriber declared a stream; read it
    /// through `device_ptr`.
    pub fn data(&self) -> &[u8] {
        let Some(p) = self.sh.host_slot(self.slot) else {
            return &[];
        };
        #[allow(clippy::cast_possible_truncation)]
        let len = self.meta.nbytes as usize;
        // SAFETY: the borrow keeps the publisher off this slot, and this channel's next copy into
        // it off its copy; take checked nbytes against slot_size.
        unsafe { std::slice::from_raw_parts(p, len) }
    }

    /// The frame's device address, 0 when the channel declared no stream. Read-only on a
    /// subscriber's mapping.
    pub fn device_ptr(&self) -> u64 {
        self.sh.device_ptr(self.slot)
    }

    /// The channel's stream, which the release waits for: queue the frame's work on it.
    pub fn stream(&self) -> Stream {
        self.sh.stream()
    }

    pub fn host_addressable(&self) -> bool {
        self.sh.host_slot(self.slot).is_some()
    }

    /// Copies `dst.len()` bytes from byte `at` of the frame into host `dst`. A device payload is
    /// copied on `stream()`, and this waits for that stream. False past the frame's bytes, or when
    /// the copy or the wait failed.
    pub fn copy_to(&self, at: usize, dst: &mut [u8]) -> bool {
        #[allow(clippy::cast_possible_truncation)]
        let len = self.meta.nbytes as usize;
        let Some(end) = at.checked_add(dst.len()).filter(|&end| end <= len) else {
            return false;
        };
        if dst.is_empty() {
            return true;
        }
        if self.host_addressable() {
            let Some(src) = self.data().get(at..end) else {
                return false;
            };
            dst.copy_from_slice(src);
            return true;
        }
        let src = self.device_ptr();
        let stream = self.stream();
        src != 0
            && stream.copy_to_host(dst.as_mut_ptr(), src + at as u64, dst.len())
            && stream.wait()
    }

    pub fn meta(&self) -> &FrameMeta {
        &self.meta
    }

    /// The frame's aux bytes, in host memory whatever the payload is.
    pub fn aux(&self) -> &[u8] {
        let len = usize::from(self.meta.aux_len);
        // SAFETY: the borrow keeps the publisher off this slot; take checked aux_len against
        // aux_size, which is the region's length.
        unsafe { std::slice::from_raw_parts(self.sh.seg.aux(self.slot), len) }
    }

    pub fn ticket(&self) -> u64 {
        self.ticket
    }

    /// The channel as trace events name it, for a binding's own events about this frame.
    pub fn trace_channel(&self) -> *const c_void {
        self.sh.trace_handle()
    }

    /// `CLOCK_MONOTONIC` nanoseconds at commit.
    pub fn publish_ns(&self) -> u64 {
        self.sh
            .seg
            .slot(self.slot)
            .publish_ns
            .load(Ordering::Relaxed)
    }
}

#[cfg(any(test, feature = "fuzz"))]
impl FrameView {
    pub(crate) fn lag(&self) -> u64 {
        let latest = self.sh.seg.ctrl().latest.load(Ordering::Acquire); // order: latest
        latest.wrapping_sub(self.ticket)
    }
}

impl Drop for FrameView {
    /// A failed fence keeps the borrow on purpose: a kernel may still read the slot, and the
    /// publisher must not rewrite it. `fence_failed` shows it. The channel is kept for the
    /// process's life too, so neither its mapping nor its reader number goes before the kernel.
    fn drop(&mut self) {
        if let Some(late) = self.sh.defer_release() {
            event!(flux_trace_release(self.sh.trace_handle(), self.ticket, 0));
            self.sh
                .defer(late, self.slot, self.ticket, Seam::Release, false);
            return;
        }
        let fenced = self.sh.fence(Seam::Release);
        let held = !matches!(fenced, Fenced::Done);
        event!(flux_trace_release(
            self.sh.trace_handle(),
            self.ticket,
            i32::from(held)
        ));
        match fenced {
            Fenced::Done => {
                if self.sh.gpu.is_some() {
                    ledger::fenced(self.sh.trace_handle(), self.slot);
                }
                self.sh.end_borrow(self.slot, self.ticket);
            }
            Fenced::Failed => {
                ledger::kept(self.sh.trace_handle(), self.slot);
                self.sh.leases_leaked.fetch_add(1, Ordering::Release); // order: leases_leaked
                std::mem::forget(Arc::clone(&self.sh));
            }
            Fenced::Late(late) => {
                self.sh.leases_leaked.fetch_add(1, Ordering::Release); // order: leases_leaked
                self.sh
                    .defer(late, self.slot, self.ticket, Seam::Release, true);
            }
        }
    }
}
