//! A borrowed frame.

use std::ffi::c_void;
use std::sync::Arc;
use std::sync::atomic::Ordering;

use crate::arena::{self};
use crate::gpu::{Fenced, Seam, Stream, ledger};
use crate::layout::FrameMeta;
use crate::trace::event;

use super::ChannelShared;

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
        let latest = self.sh.seg.ctrl().latest.load(Ordering::Acquire);
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
                self.sh.leases_leaked.fetch_add(1, Ordering::Release);
                std::mem::forget(Arc::clone(&self.sh));
            }
            Fenced::Late(late) => {
                self.sh.leases_leaked.fetch_add(1, Ordering::Release);
                self.sh
                    .defer(late, self.slot, self.ticket, Seam::Release, true);
            }
        }
    }
}
