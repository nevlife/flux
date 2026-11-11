//! The state one end of a channel shares with its views and loans: the segment, the claim and
//! borrow protocol, the GPU fences, and the arena memory frames own.

use std::ffi::{CString, c_void};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, OnceLock, Weak};

use crate::arena::{self, ArenaMap};
use crate::error::{Error, Result};
use crate::executor::Control;
use crate::gpu::{
    self, DevicePayload, FenceStats, Fenced, HostRegistration, Late, Route, Seam, Stream, ledger,
};
use crate::layout::{FrameMeta, TICKET_CLAIMED, TICKET_EMPTY, reader_lock_byte};
use crate::reader::ReaderLease;
use crate::segment::Segment;
use crate::sys::{self, PiMutex};
use crate::trace::event;

use super::{COUNT_CORRUPT, SWEEP_NS};

/// A channel that declared a stream, or a host subscriber of a device payload: the device address
/// of slot 0's payload and the stream both seams wait for.
pub(super) struct GpuState {
    pub(super) stream: Stream,
    pub(super) base: u64,
    pub(super) stats: Arc<FenceStats>,
    pub(super) staged: Staged,
}

/// A subscriber on a discrete GPU whose payload sits across `PCIe` reads its own copy, which each
/// take makes.
pub(super) enum Staged {
    InPlace,
    /// A host payload copied to the device. `GpuState::base` is this buffer's.
    Device(gpu::DeviceBuffer),
    /// A device payload, imported at `from`, copied to the host on a stream of flux's own. Such a
    /// subscriber declared no stream, so its views show neither a device address nor a stream.
    Host {
        buf: gpu::HostBuffer,
        from: u64,
    },
}

/// What views and loans keep alive past their Channel: the mapping and this consumer's number.
pub struct ChannelShared {
    /// Shared by every subscription of the process on the segment.
    pub(super) seg: Arc<Segment>,
    pub(super) lease: OnceLock<Arc<ReaderLease>>,
    pub(super) next_ticket: AtomicU64,
    pub(super) dropped: AtomicU64,
    pub(super) outstanding: AtomicU32,
    /// An executor whose take was refused for `max_borrow`, poked by the release that unblocks it.
    pub(super) release_waker: PiMutex<Weak<Control>>,
    pub(super) release_armed: AtomicBool,
    /// Publisher only: when `reclaim_dead` last ran for parked counts.
    pub(super) last_sweep_ns: AtomicU64,
    /// OFD locks do not exclude threads sharing the description: two reclaims would both take a
    /// dead number's byte, and one's unlock could hand it to a new subscriber the other then zeroes.
    pub(super) reclaiming: PiMutex<()>,
    /// The fork generation this was made in. A forked child's copy leaves shared counts alone:
    /// they are the parent's, and the parent may be reading the slot they hold.
    pub(super) born: u32,
    /// The segment name for trace events; empty for a heap channel.
    pub(super) trace_name: CString,
    /// Where `header.stamp` sits in a frame, for trace events; `trace::NO_STAMP` without one.
    pub(super) stamp_offset: i32,
    pub(super) gpu: Option<GpuState>,
    /// Views whose release fence failed: their borrows are never given back, unless the wait
    /// only timed out and the work finishes later.
    pub(super) leases_leaked: AtomicU32,
    /// Releases refused by `end_borrow`.
    pub(super) misreleased: AtomicU64,
    /// Deferred releases and timed-out fences, given back by the channel's next take, claim or
    /// executor pass once the waiter saw their work done.
    pub(super) late: PiMutex<Vec<Deferred>>,
    pub(super) late_count: AtomicU32,
    /// Set when the `Channel` goes: deferred work from views still out is handed to the waiter
    /// instead, since nobody is left to reap it.
    pub(super) orphaned_late: AtomicBool,
    /// The executor serving this channel, poked by the waiter when such work is done.
    pub(super) gpu_waker: PiMutex<Weak<Control>>,
    pub(super) gpu_waker_of: AtomicUsize,
    /// Publisher only, per slot: the arena memory its frame points into, freed when the slot is
    /// claimed again.
    pub(super) owned: PiMutex<Vec<Vec<Owned>>>,
    /// Slots with arena memory, so a claim of a channel that never used the arena skips the lock.
    pub(super) owned_slots: AtomicU32,
    /// Subscriber side: the publisher's arena, mapped by the first frame that named it. One
    /// publisher process, so one arena, per segment.
    pub(super) arena_map: OnceLock<Arc<ArenaMap>>,
}

/// Arena memory a frame owns.
pub enum Owned {
    Block(u64),
    /// Memory a binding allocated in the arena and keeps alive until `release(ctx)`.
    Adopted(unsafe extern "C" fn(*mut c_void), *mut c_void),
}

// SAFETY: the adopter promises its release may run on whichever thread claims the slot.
unsafe impl Send for Owned {}

/// A slot whose work nobody waited for yet, kept until it is done. It keeps the channel too, as a
/// failed fence does, so the mapping and the reader number stay while the device may use them.
pub(super) struct Deferred {
    pub(super) late: Late,
    pub(super) slot: u32,
    /// A release's view ticket; a commit has none.
    pub(super) ticket: u64,
    pub(super) seam: Seam,
    /// A timed-out release, counted in `leases_leaked` meanwhile; a deferred one is not.
    pub(super) leaked: bool,
    pub(super) keep: Arc<ChannelShared>,
}

impl Deferred {
    pub(super) fn orphan(self) {
        let late = self.late.clone();
        late.hand_off(Box::new(self));
    }
}

impl ChannelShared {
    pub(super) fn new(seg: Arc<Segment>, stamp_offset: i32) -> ChannelShared {
        sys::watch_fork();
        ChannelShared {
            born: sys::fork_gen(),
            trace_name: CString::new(seg.name()).unwrap_or_default(),
            stamp_offset,
            seg,
            lease: OnceLock::new(),
            next_ticket: AtomicU64::new(1),
            dropped: AtomicU64::new(0),
            outstanding: AtomicU32::new(0),
            release_waker: PiMutex::new(Weak::new()),
            release_armed: AtomicBool::new(false),
            last_sweep_ns: AtomicU64::new(0),
            reclaiming: PiMutex::new(()),
            gpu: None,
            leases_leaked: AtomicU32::new(0),
            misreleased: AtomicU64::new(0),
            late: PiMutex::new(Vec::new()),
            late_count: AtomicU32::new(0),
            orphaned_late: AtomicBool::new(false),
            gpu_waker: PiMutex::new(Weak::new()),
            gpu_waker_of: AtomicUsize::new(0),
            owned: PiMutex::new(Vec::new()),
            owned_slots: AtomicU32::new(0),
            arena_map: OnceLock::new(),
        }
    }

    pub(super) fn own(&self, slot: u32, o: Owned) {
        let mut owned = self.owned.lock();
        if owned.is_empty() {
            owned.resize_with(self.seg.layout().slot_count as usize, Vec::new);
        }
        if let Some(v) = owned.get_mut(slot as usize) {
            if v.is_empty() {
                self.owned_slots.fetch_add(1, Ordering::Relaxed);
            }
            v.push(o);
        }
    }

    pub(super) fn owns_any(&self, slot: u32) -> bool {
        self.owned_slots.load(Ordering::Relaxed) != 0
            && self
                .owned
                .lock()
                .get(slot as usize)
                .is_some_and(|v| !v.is_empty())
    }

    /// Outside the lock: a release may run a binding's code, which may publish.
    pub(super) fn free_owned(&self, slot: u32) {
        if self.owned_slots.load(Ordering::Relaxed) == 0 {
            return;
        }
        let mut list = match self.owned.lock().get_mut(slot as usize) {
            Some(v) if !v.is_empty() => std::mem::take(v),
            _ => return,
        };
        self.owned_slots.fetch_sub(1, Ordering::Relaxed);
        let a = arena::existing();
        for o in list.drain(..) {
            match o {
                Owned::Block(off) => {
                    if let Some(a) = a {
                        a.free(off);
                    }
                }
                // SAFETY: the adopter's contract.
                Owned::Adopted(release, ctx) => unsafe { release(ctx) },
            }
        }
        // Back with its capacity, so the slot's next frame records its blocks without allocating.
        // Nothing owns this slot meanwhile: its claim is this call's.
        if let Some(v) = self.owned.lock().get_mut(slot as usize) {
            *v = list;
        }
    }

    /// Judges the stream against the payload and makes it device-addressable. Every refusal is
    /// permanent: retrying never turns a host payload into a device one.
    pub(super) fn attach_device(&mut self, stream: &Stream, stats: &Arc<FenceStats>) -> Result<()> {
        if !stream.declared() {
            if self.seg.host_payload() {
                return Ok(());
            }
            return self.attach_host_copy(stats);
        }
        let seg = &self.seg;
        let route = gpu::require_route(stream.device())?.route;
        let mut staged = Staged::InPlace;
        let base = if seg.host_payload() {
            let payload = seg.payload(0);
            match route {
                Route::ShmDirect => payload as u64,
                Route::ShmRegistered => seg
                    .device_payload(|| {
                        let bytes = seg.layout().payload_bytes();
                        HostRegistration::new(
                            payload,
                            bytes,
                            seg.payload_readonly(),
                            stream.device(),
                        )
                        .map(DevicePayload::Registered)
                    })?
                    .base(),
                // A discrete GPU reads host memory across `PCIe` at every access, so the frame is
                // copied once. Registering the mapping keeps that copy off the calling thread.
                Route::DeviceHandle | Route::None => {
                    if !seg.payload_readonly() || gpu::registers_read_only(stream.device()) {
                        seg.device_payload(|| {
                            HostRegistration::new(
                                payload,
                                seg.layout().payload_bytes(),
                                seg.payload_readonly(),
                                stream.device(),
                            )
                            .map(DevicePayload::Registered)
                        })?;
                    }
                    let buf =
                        gpu::DeviceBuffer::new(seg.layout().payload_bytes(), stream.device())?;
                    let base = buf.base();
                    staged = Staged::Device(buf);
                    base
                }
            }
        } else {
            if route != Route::DeviceHandle {
                return Err(Error::Mismatch(format!(
                    "the payload of '{}' is a device allocation, which this host's {route} route \
                     does not import",
                    seg.name()
                )));
            }
            let ordinal = self.payload_device()?;
            if ordinal != stream.device() {
                return Err(Error::Mismatch(format!(
                    "the payload of '{}' is on device {ordinal} and the stream on device {}, \
                     which cannot reach it",
                    seg.name(),
                    stream.device()
                )));
            }
            self.import(ordinal)?
        };
        self.gpu = Some(GpuState {
            stream: stream.clone(),
            base,
            stats: Arc::clone(stats),
            staged,
        });
        Ok(())
    }

    /// A host subscriber of a device payload: the payload is imported and each take copies the
    /// frame to pinned host memory of this subscriber's.
    pub(super) fn attach_host_copy(&mut self, stats: &Arc<FenceStats>) -> Result<()> {
        let ordinal = self.payload_device()?;
        let route = gpu::require_route(ordinal)?.route;
        if route != Route::DeviceHandle {
            return Err(Error::Mismatch(format!(
                "the payload of '{}' is a device allocation, which this host's {route} route does \
                 not import",
                self.seg.name()
            )));
        }
        let from = self.import(ordinal)?;
        let buf = gpu::HostBuffer::new(self.seg.layout().payload_bytes(), ordinal)?;
        self.gpu = Some(GpuState {
            stream: Stream::on_device(ordinal)?,
            base: 0,
            stats: Arc::clone(stats),
            staged: Staged::Host { buf, from },
        });
        Ok(())
    }

    /// This process's ordinal of the device the payload was allocated on.
    pub(super) fn payload_device(&self) -> Result<i32> {
        gpu::device_for_uuid(&self.seg.header().device_uuid).ok_or_else(|| {
            Error::Mismatch(format!(
                "the payload of '{}' is in GPU memory, and no device in this process has the UUID \
                 it was allocated on",
                self.seg.name()
            ))
        })
    }

    pub(super) fn import(&self, ordinal: i32) -> Result<u64> {
        let seg = &self.seg;
        Ok(seg
            .device_payload(|| {
                gpu::DeviceImport::open(
                    &gpu::endpoint(seg.id()),
                    seg.layout().payload_bytes(),
                    ordinal,
                )
                .map(DevicePayload::Imported)
            })?
            .base())
    }

    pub(super) fn slot_offset(&self, slot: u32) -> usize {
        slot as usize * self.seg.layout().payload_stride()
    }

    pub(super) fn device_ptr(&self, slot: u32) -> u64 {
        match &self.gpu {
            Some(g) if !matches!(g.staged, Staged::Host { .. }) => {
                g.base + self.slot_offset(slot) as u64
            }
            _ => 0,
        }
    }

    pub(super) fn stream(&self) -> Stream {
        match &self.gpu {
            Some(g) if !matches!(g.staged, Staged::Host { .. }) => g.stream.clone(),
            _ => Stream::default(),
        }
    }

    /// The bytes of `slot` the host reads: the mapping, or this subscriber's copy of a device
    /// payload.
    pub(super) fn host_slot(&self, slot: u32) -> Option<*mut u8> {
        if self.seg.host_payload() {
            return Some(self.seg.payload(slot));
        }
        match &self.gpu {
            Some(GpuState {
                staged: Staged::Host { buf, .. },
                ..
            }) => {
                // SAFETY: the copy is payload_bytes long, as the payload.
                Some(unsafe { buf.base().add(self.slot_offset(slot)) })
            }
            _ => None,
        }
    }

    /// Copies the borrowed `slot` to where this subscriber reads it. A host copy is waited for
    /// here, since the caller reads it once the take returns; a device copy is queued on the
    /// channel's stream, which the caller's work follows. False once the driver refused, when the
    /// copy may have run in part.
    pub(super) fn stage(&self, slot: u32, nbytes: usize) -> bool {
        let Some(g) = &self.gpu else {
            return true;
        };
        let at = self.slot_offset(slot);
        match &g.staged {
            Staged::InPlace => true,
            Staged::Device(buf) => {
                g.stream
                    .copy_to_device(buf.base() + at as u64, self.seg.payload(slot), nbytes)
            }
            Staged::Host { buf, from } => {
                // SAFETY: at + nbytes is inside the copy, as inside the payload.
                let to = unsafe { buf.base().add(at) };
                g.stream.copy_to_host(to, from + at as u64, nbytes) && g.stream.wait()
            }
        }
    }

    /// Before a commit or a release declares done what the stream may still be doing. A forked
    /// child has no CUDA and returns no borrow anyway.
    pub(super) fn fence(&self, seam: Seam) -> Fenced {
        match &self.gpu {
            Some(g) if !self.inherited() => g.stats.fence(&g.stream, seam, self.waker()),
            _ => Fenced::Done,
        }
    }

    pub(super) fn waker(&self) -> Weak<Control> {
        self.gpu_waker.lock().clone()
    }

    /// A deferred release's mark, or None to wait as usual: no stream, a forked child, or a mark
    /// the driver refused.
    pub(super) fn defer_release(&self) -> Option<Late> {
        let g = self.gpu.as_ref()?;
        if self.inherited() || !g.stats.deferred.load(Ordering::Relaxed) {
            return None;
        }
        g.stream.mark(self.waker())
    }

    pub(super) fn defer(
        self: &Arc<Self>,
        late: Late,
        slot: u32,
        ticket: u64,
        seam: Seam,
        leaked: bool,
    ) {
        let keep = Arc::clone(self);
        let d = Deferred {
            late,
            slot,
            ticket,
            seam,
            leaked,
            keep,
        };
        let mut l = self.late.lock();
        if self.orphaned_late.load(Ordering::Relaxed) {
            drop(l);
            d.orphan();
            return;
        }
        l.push(d);
        self.late_count.fetch_add(1, Ordering::Release);
    }

    /// The channel is going: what it already reaped is given back here, on its thread; the rest
    /// waits on the waiter, which only lets it go. The borrows it held stay counted until the
    /// process's subscriber number is let go, as for a subscriber that died.
    pub(super) fn orphan_late(&self) {
        self.reap_late();
        let pending = {
            let mut l = self.late.lock();
            self.orphaned_late.store(true, Ordering::Relaxed);
            std::mem::take(&mut *l)
        };
        for d in pending {
            d.orphan();
        }
    }

    /// Gives back what deferred work held once it is done: a borrow, or a claim that was never
    /// published. Work that failed keeps its slot for the process's life, as a failed fence does.
    pub(super) fn reap_late(&self) {
        if self.late_count.load(Ordering::Acquire) == 0 {
            return;
        }
        // Asked with the lock let go: `done` may call into the driver, and a release on another
        // thread takes this lock to defer. Only this channel's thread reaps, so what it took out
        // is its alone.
        let all = std::mem::take(&mut *self.late.lock());
        let (ended, wait): (Vec<Deferred>, Vec<Deferred>) = all
            .into_iter()
            .partition(|d| d.late.done() || d.late.failed());
        self.late.lock().extend(wait);
        #[allow(clippy::cast_possible_truncation)]
        let n = ended.len() as u32;
        for d in ended {
            if d.late.failed() {
                if d.seam == Seam::Release {
                    ledger::kept(self.trace_handle(), d.slot);
                }
                if !d.leaked && d.seam == Seam::Release {
                    self.leases_leaked.fetch_add(1, Ordering::Release);
                }
                if let Some(g) = &self.gpu {
                    g.stats.count_failed();
                }
                std::mem::forget(d.keep);
                continue;
            }
            match d.seam {
                Seam::Release => {
                    if d.leaked {
                        self.leases_leaked.fetch_sub(1, Ordering::Release);
                    }
                    ledger::fenced(self.trace_handle(), d.slot);
                    self.end_borrow(d.slot, d.ticket);
                }
                Seam::Commit => self.abort(d.slot),
            }
        }
        self.late_count.fetch_sub(n, Ordering::Release);
    }

    pub(super) fn trace_handle(&self) -> *const c_void {
        std::ptr::from_ref(self).cast()
    }

    /// After the Arc exists: the event names the channel by its address.
    pub(super) fn traced(self: Arc<Self>) -> Arc<Self> {
        let id = self.seg.id();
        let l = self.seg.layout();
        event!(flux_trace_channel_init(
            self.trace_handle(),
            id.dev,
            id.ino,
            self.trace_name.as_ptr(),
            i32::from(self.seg.is_publisher()),
            l.slot_count,
            l.slot_size,
            self.stamp_offset,
        ));
        self
    }

    /// The `header.stamp` bytes of the frame in `slot`, 0 when the type has none or the frame is
    /// too short or in device memory. The caller owns the slot: claimed or borrowed.
    pub(super) fn stamp(&self, slot: u32, nbytes: u64) -> u64 {
        let Ok(at) = usize::try_from(self.stamp_offset) else {
            return 0;
        };
        let Some(base) = self.host_slot(slot) else {
            return 0;
        };
        if at as u64 + 8 > nbytes {
            return 0;
        }
        // SAFETY: at + 8 <= nbytes <= slot_size: inside the slot.
        let p = unsafe { base.add(at) };
        // SAFETY: the caller owns the slot, so nobody writes it meanwhile.
        u64::from_le_bytes(unsafe { std::ptr::read_unaligned(p.cast::<[u8; 8]>()) })
    }

    pub(super) fn inherited(&self) -> bool {
        sys::fork_gen() != self.born
    }

    pub(super) fn gpu_wedged(&self) -> bool {
        self.gpu.is_some() && gpu::wedged().is_some()
    }

    pub(super) fn index(&self) -> u32 {
        self.lease.get().map_or(0, |l| l.index)
    }

    pub(super) fn reader(&self, slot: u32) -> &AtomicU32 {
        self.seg.reader(slot, self.index())
    }

    pub(super) fn readers_free(&self, slot: u32, order: Ordering) -> bool {
        let n = self.seg.readers_in_use(order);
        self.seg
            .readers(slot)
            .iter()
            .take(n)
            .all(|r| r.load(order) == 0)
    }

    /// Claims `s` if no reader holds it, and leaves it as it was otherwise.
    pub(super) fn try_claim(&self, s: u32) -> bool {
        let sl = self.seg.slot(s);
        let old = sl.commit_ticket.load(Ordering::Relaxed);
        if old == TICKET_CLAIMED || !self.readers_free(s, Ordering::Acquire) {
            return false;
        }
        if sl
            .commit_ticket
            .compare_exchange(old, TICKET_CLAIMED, Ordering::SeqCst, Ordering::Relaxed)
            .is_err()
        {
            return false;
        }
        if !self.readers_free(s, Ordering::SeqCst) {
            // Borrowed under us before a byte was written: the frame is still whole.
            sl.commit_ticket.store(old, Ordering::Release);
            return false;
        }
        true
    }

    /// When starved, and at most once a `SWEEP_NS` while anyone is parked. A number whose byte
    /// lock can be taken belongs to a dead process.
    pub(super) fn reclaim_dead(&self) -> bool {
        let Some(fd) = self.seg.publisher_fd() else {
            return false;
        };
        let _reclaiming = self.reclaiming.lock();
        let slots = self.seg.layout().slot_count;
        let mut reclaimed = false;
        #[allow(clippy::cast_possible_truncation)]
        let numbers = self.seg.readers_in_use(Ordering::Acquire) as u32;
        for i in 0..numbers {
            let held = self.seg.waiter(i).load(Ordering::Acquire) != 0
                || (0..slots).any(|s| self.seg.reader(s, i).load(Ordering::Acquire) != 0);
            if !held || !sys::lock_write(fd, reader_lock_byte(i)) {
                continue;
            }
            self.seg.clear_reader(i);
            sys::unlock(fd, reader_lock_byte(i));
            reclaimed = true;
        }
        reclaimed
    }

    pub(super) fn finish_commit(&self, s: u32, meta: &FrameMeta) {
        let sl = self.seg.slot(s);
        // SAFETY: the slot is claimed, so no reader holds or reads this meta.
        unsafe { sl.meta.get().write(*meta) };
        let ticket = self.next_ticket.fetch_add(1, Ordering::Relaxed);
        let now = sys::monotonic_ns();
        sl.publish_ns.store(now, Ordering::Relaxed);
        self.seg.index_entry(ticket).store(s, Ordering::Relaxed);
        sl.commit_ticket.store(ticket, Ordering::Release);

        let ctrl = self.seg.ctrl();
        ctrl.latest.store(ticket, Ordering::Release);
        // Dekkers with wait(): bump, then read waiters; wait announces, then reads the word.
        ctrl.wakeup.fetch_add(1, Ordering::SeqCst);
        let woke = ctrl.waiters.load(Ordering::SeqCst) > 0
            || self
                .seg
                .waiters()
                .iter()
                .take(self.seg.readers_in_use(Ordering::SeqCst))
                .any(|w| w.load(Ordering::SeqCst) > 0);
        if woke {
            sys::futex_wake(&ctrl.wakeup);
            // A subscriber killed while parked never takes its count back, and then every
            // publish pays the wake syscall.
            if now - self.last_sweep_ns.load(Ordering::Relaxed) >= SWEEP_NS {
                self.last_sweep_ns.store(now, Ordering::Relaxed);
                self.reclaim_dead();
            }
        }
        event!(flux_trace_commit(
            self.trace_handle(),
            ticket,
            s,
            meta.nbytes,
            i32::from(woke),
            self.stamp(s, meta.nbytes),
        ));
    }

    pub(super) fn abort(&self, s: u32) {
        if self.inherited() {
            return;
        }
        // The loan may have overwritten the bytes, so the old frame goes too.
        self.seg
            .slot(s)
            .commit_ticket
            .store(TICKET_EMPTY, Ordering::SeqCst);
    }

    /// A release the borrow protocol cannot have made, a view's storage copied and released twice
    /// say, is refused: it changes no count, so it can neither underflow this number's counter nor
    /// give back a borrow another view still stands on.
    pub(super) fn end_borrow(&self, slot: u32, ticket: u64) {
        if self.inherited() {
            return;
        }
        // While the borrow stands no claim succeeds, so another ticket means it was given back
        // already. A claim about to fail shows `TICKET_CLAIMED` for a moment.
        let now = self.seg.slot(slot).commit_ticket.load(Ordering::Relaxed);
        if now != ticket && now != TICKET_CLAIMED {
            self.misreleased(slot, ticket, "the slot holds another frame");
            return;
        }
        let counter = self.reader(slot);
        let prev = counter.fetch_sub(1, Ordering::Release);
        if prev.wrapping_sub(1) >= COUNT_CORRUPT {
            // Undone. A count found at 0 is left at 1 instead: the slot stays held until the
            // segment goes, a leak rather than a rewrite under a view that still reads it.
            counter.fetch_add(if prev == 0 { 2 } else { 1 }, Ordering::Relaxed);
            self.misreleased(
                slot,
                ticket,
                "this subscriber's count on the slot was not above 0",
            );
            return;
        }
        if self.gpu.is_some() {
            ledger::returned(self.trace_handle(), slot);
        }
        // Dekkers with arm_release: drop, then read the flag; arm sets it, then reads the count.
        self.outstanding.fetch_sub(1, Ordering::SeqCst);
        if self.release_armed.load(Ordering::SeqCst)
            && self.release_armed.swap(false, Ordering::SeqCst)
            && let Some(ctl) = self.release_waker.lock().upgrade()
        {
            ctl.poke();
        }
    }

    #[cold]
    pub(super) fn misreleased(&self, slot: u32, ticket: u64, why: &str) {
        static WARNED: AtomicBool = AtomicBool::new(false);
        self.misreleased.fetch_add(1, Ordering::Relaxed);
        if !WARNED.swap(true, Ordering::Relaxed) {
            eprintln!(
                "flux: refused a second release of the view of ticket {ticket} in slot {slot} of \
                 '{}': {why}. Counted in misreleased(); later ones are only counted",
                self.seg.name()
            );
        }
    }

    pub(super) fn arm_release(&self, ctl: &Arc<Control>) {
        *self.release_waker.lock() = Arc::downgrade(ctl);
        self.release_armed.store(true, Ordering::SeqCst);
    }

    pub fn wake_seq(&self) -> u32 {
        self.seg.ctrl().wakeup.load(Ordering::Acquire)
    }

    pub fn wake_word(&self) -> &AtomicU32 {
        &self.seg.ctrl().wakeup
    }

    /// One store, under this process's subscriber number if it has one, so a process killed at
    /// any point leaves nothing the sweep cannot take back.
    pub(crate) fn add_waiter(&self) -> Waiter {
        let number = self.lease.get().map(|l| l.index);
        if !self.inherited() {
            self.waiter_count(number).fetch_add(1, Ordering::SeqCst);
        }
        Waiter(number)
    }

    pub(crate) fn remove_waiter(&self, w: Waiter) {
        if !self.inherited() {
            self.waiter_count(w.0).fetch_sub(1, Ordering::SeqCst);
        }
    }

    pub(super) fn waiter_count(&self, number: Option<u32>) -> &AtomicI32 {
        number.map_or(&self.seg.ctrl().waiters, |i| self.seg.waiter(i))
    }

    /// False only on a timeout. `timeout_ns` < 0 waits forever.
    pub fn wait(&self, last_seq: u32, timeout_ns: i64) -> bool {
        let w = self.add_waiter();
        let word = &self.seg.ctrl().wakeup;
        let woken =
            word.load(Ordering::SeqCst) != last_seq || sys::futex_wait(word, last_seq, timeout_ns);
        self.remove_waiter(w);
        woken
    }
}

impl Drop for ChannelShared {
    fn drop(&mut self) {
        if self.gpu.is_some() {
            ledger::channel_gone(self.trace_handle());
        }
        // A frame left in a slot can still be taken after the publisher is gone, so its arena
        // memory goes only with a claim, which a borrow refuses.
        // shortcut: a slot still borrowed now keeps its blocks until this process ends (at most
        // the subscribers' max_borrow); free them once its last reader leaves, when a process that
        // makes and drops publishers in a loop runs out of arena.
        if self.owned_slots.load(Ordering::Relaxed) != 0 && !self.inherited() {
            for s in 0..self.seg.layout().slot_count {
                if self.owns_any(s) && self.try_claim(s) {
                    self.abort(s);
                    self.free_owned(s);
                }
            }
        }
    }
}

/// Where an `add_waiter` counted: a lease can arrive before the matching remove.
#[derive(Clone, Copy)]
pub(crate) struct Waiter(Option<u32>);
