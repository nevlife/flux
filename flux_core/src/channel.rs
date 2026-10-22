//! Publish and take over one segment.
//!
//! Dekker between the publisher (CAS the slot's ticket to claimed, then read every reader counter)
//! and a subscriber (own counter +1, then read the ticket): both pairs are `SeqCst`, so at least
//! one side sees the other. The commit stores the ticket with `Release`, and the subscriber's load
//! after its +1 acquires it.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use std::cell::Cell;
use std::collections::VecDeque;
use std::ffi::{CString, c_char, c_void};
use std::fmt;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, OnceLock, Weak};

use crate::arena::{self, TRAILER_BYTES};
use crate::discovery::{
    SignpostMap, open_publisher_segment, segment_publisher_dead, sweep_dead_once,
};
use crate::error::{Error, Result};
use crate::executor::Control;
use crate::gpu::{
    self, DevicePayload, FenceStats, FenceWait, Fenced, HostRegistration, Late, Route, Seam,
    Stream, ledger,
};
use crate::layout::{
    DType, FrameMeta, MAX_DIMS, MIN_SLOT_COUNT, TICKET_CLAIMED, TICKET_EMPTY, reader_lock_byte,
};
use crate::qos::{MemoryPolicy, QoS};
use crate::reader::{self, ReaderLease};
use crate::segment::{Mapped, Segment};
use crate::sys::{self, PiMutex};
use crate::trace::{self, event};

const MAX_RETRIES: u32 = 64;
// Bounds one park inside take_blocking: the next take is what notices a publisher restart.
const MAX_PARK_NS: i64 = 100_000_000;
const PROBE_EVERY: u32 = 8;
// A live publisher that sends rarely would otherwise pay for a probe every PROBE_EVERY takes,
// and an executor takes every quiet source on each wake.
const PROBE_NS: u64 = 100_000_000;
const SWEEP_NS: u64 = 1_000_000_000;
/// A reader count at or past this is no count the protocol makes: a counter taken below 0, or one
/// a peer overwrote. Each process holds far fewer borrows.
const COUNT_CORRUPT: u32 = 1 << 31;

/// When a view of a channel with a stream gives its slot back.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum ReleaseMode {
    /// The release waits for the stream, so the slot is back when it returns.
    #[default]
    Wait = 0,
    /// The release returns at once; the slot is back once the device is done with it.
    Deferred = 1,
}

#[must_use]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Published {
    Ok,
    /// Every slot is borrowed. `dropped()` counts it.
    Backpressure,
    /// The frame does not fit the slot or `MAX_DIMS`, or the bytes do not match the shape.
    TooLarge,
    /// This channel was opened as a subscriber, or before a `fork()` this process came out of.
    NotPublisher,
    /// The payload is device memory, which a host copy cannot reach: loan and write through
    /// `device_ptr`.
    WrongDevice,
    /// The declared stream could not be waited for. The slot stays claimed and is never reused.
    FenceFailed,
    /// The process's GPU is wedged (`gpu::wedged`): nothing GPU works until it restarts.
    GpuWedged,
}

impl Published {
    pub fn faulted(self) -> bool {
        !matches!(self, Published::Ok | Published::Backpressure)
    }
}

impl fmt::Display for Published {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self, f)
    }
}

/// Why a take returned nothing although frames may exist.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Refused {
    /// This consumer already holds `max_borrow` views.
    pub max_borrow: u64,
    /// No subscriber number was free, or the channel was opened before a `fork()`.
    pub no_reader: u64,
    /// The check after the borrow lost its whole retry budget.
    pub contended: u64,
    /// Meta outside the slot.
    pub bad_frame: u64,
    /// Views whose release fence failed keep their borrows for good. Once they fill `max_borrow`,
    /// every take is refused, and counted here rather than as `max_borrow`. So is every take
    /// after the process's GPU is wedged (`gpu::wedged`).
    pub fence: u64,
}

/// A channel that declared a stream: the device address of slot 0's payload and the stream both
/// seams wait for.
struct GpuState {
    stream: Stream,
    base: u64,
    stats: Arc<FenceStats>,
}

/// What views and loans keep alive past their Channel: the mapping and this consumer's number.
pub struct ChannelShared {
    /// Shared by every subscription of the process on the segment.
    seg: Arc<Segment>,
    lease: OnceLock<Arc<ReaderLease>>,
    next_ticket: AtomicU64,
    dropped: AtomicU64,
    outstanding: AtomicU32,
    /// An executor whose take was refused for `max_borrow`, poked by the release that unblocks it.
    release_waker: PiMutex<Weak<Control>>,
    release_armed: AtomicBool,
    /// Publisher only: when `reclaim_dead` last ran for parked counts.
    last_sweep_ns: AtomicU64,
    /// OFD locks do not exclude threads sharing the description: two reclaims would both take a
    /// dead number's byte, and one's unlock could hand it to a new subscriber the other then zeroes.
    reclaiming: PiMutex<()>,
    /// The fork generation this was made in. A forked child's copy leaves shared counts alone:
    /// they are the parent's, and the parent may be reading the slot they hold.
    born: u32,
    /// The segment name for trace events; empty for a heap channel.
    trace_name: CString,
    /// Where `header.stamp` sits in a frame, for trace events; `trace::NO_STAMP` without one.
    stamp_offset: i32,
    gpu: Option<GpuState>,
    /// Views whose release fence failed: their borrows are never given back, unless the wait
    /// only timed out and the work finishes later.
    leases_leaked: AtomicU32,
    /// Releases refused by `end_borrow`.
    misreleased: AtomicU64,
    /// Deferred releases and timed-out fences, given back by the channel's next take, claim or
    /// executor pass once the waiter saw their work done.
    late: PiMutex<Vec<Deferred>>,
    late_count: AtomicU32,
    /// Set when the `Channel` goes: deferred work from views still out is handed to the waiter
    /// instead, since nobody is left to reap it.
    orphaned_late: AtomicBool,
    /// The executor serving this channel, poked by the waiter when such work is done.
    gpu_waker: PiMutex<Weak<Control>>,
    gpu_waker_of: AtomicUsize,
    /// Publisher only, per slot: the arena memory its frame points into, freed when the slot is
    /// claimed again.
    owned: PiMutex<Vec<Vec<Owned>>>,
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
struct Deferred {
    late: Late,
    slot: u32,
    /// A release's view ticket; a commit has none.
    ticket: u64,
    seam: Seam,
    /// A timed-out release, counted in `leases_leaked` meanwhile; a deferred one is not.
    leaked: bool,
    keep: Arc<ChannelShared>,
}

impl Deferred {
    fn orphan(self) {
        let late = self.late.clone();
        late.hand_off(Box::new(self));
    }
}

impl ChannelShared {
    fn new(seg: Arc<Segment>, stamp_offset: i32) -> ChannelShared {
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
        }
    }

    fn own(&self, slot: u32, o: Owned) {
        let mut owned = self.owned.lock();
        if owned.is_empty() {
            owned.resize_with(self.seg.layout().slot_count as usize, Vec::new);
        }
        if let Some(v) = owned.get_mut(slot as usize) {
            v.push(o);
        }
    }

    fn owns_any(&self, slot: u32) -> bool {
        self.owned
            .lock()
            .get(slot as usize)
            .is_some_and(|v| !v.is_empty())
    }

    /// Outside the lock: a release may run a binding's code, which may publish.
    fn free_owned(&self, slot: u32) {
        let list = match self.owned.lock().get_mut(slot as usize) {
            Some(v) if !v.is_empty() => std::mem::take(v),
            _ => return,
        };
        let a = arena::arena();
        for o in list {
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
    }

    /// Judges the stream against the payload and makes it device-addressable. Every refusal is
    /// permanent: retrying never turns a host payload into a device one.
    fn attach_device(&mut self, stream: &Stream, stats: &Arc<FenceStats>) -> Result<()> {
        let seg = &self.seg;
        if !stream.declared() {
            if seg.host_payload() {
                return Ok(());
            }
            return Err(Error::Mismatch(format!(
                "the payload of '{}' lives in GPU memory and cannot be read from the host; declare \
                 a stream",
                seg.name()
            )));
        }
        let route = gpu::require_route(stream.device())?.route;
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
                Route::DeviceHandle | Route::None => {
                    return Err(Error::InvalidArgument(
                        "this host reaches the GPU through the device handle path, so a channel \
                         that declares a stream needs a device payload; this one's is host memory"
                            .into(),
                    ));
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
            let ordinal = gpu::device_for_uuid(&seg.header().device_uuid).ok_or_else(|| {
                Error::Mismatch(format!(
                    "no device in this process has the UUID '{}' was allocated on",
                    seg.name()
                ))
            })?;
            if ordinal != stream.device() {
                return Err(Error::Mismatch(format!(
                    "the payload of '{}' is on device {ordinal} and the stream on device {}, \
                     which cannot reach it",
                    seg.name(),
                    stream.device()
                )));
            }
            seg.device_payload(|| {
                gpu::DeviceImport::open(
                    &gpu::endpoint(seg.id()),
                    seg.layout().payload_bytes(),
                    ordinal,
                )
                .map(DevicePayload::Imported)
            })?
            .base()
        };
        self.gpu = Some(GpuState {
            stream: stream.clone(),
            base,
            stats: Arc::clone(stats),
        });
        Ok(())
    }

    fn device_ptr(&self, slot: u32) -> u64 {
        self.gpu.as_ref().map_or(0, |g| {
            g.base + u64::from(slot) * self.seg.layout().payload_stride() as u64
        })
    }

    fn stream(&self) -> Stream {
        self.gpu
            .as_ref()
            .map(|g| g.stream.clone())
            .unwrap_or_default()
    }

    /// Before a commit or a release declares done what the stream may still be doing. A forked
    /// child has no CUDA and returns no borrow anyway.
    fn fence(&self, seam: Seam) -> Fenced {
        match &self.gpu {
            Some(g) if !self.inherited() => g.stats.fence(&g.stream, seam, self.waker()),
            _ => Fenced::Done,
        }
    }

    fn waker(&self) -> Weak<Control> {
        self.gpu_waker.lock().clone()
    }

    /// A deferred release's mark, or None to wait as usual: no stream, a forked child, or a mark
    /// the driver refused.
    fn defer_release(&self) -> Option<Late> {
        let g = self.gpu.as_ref()?;
        if self.inherited() || !g.stats.deferred.load(Ordering::Relaxed) {
            return None;
        }
        g.stream.mark(self.waker())
    }

    fn defer(self: &Arc<Self>, late: Late, slot: u32, ticket: u64, seam: Seam, leaked: bool) {
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
    fn orphan_late(&self) {
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
    fn reap_late(&self) {
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

    fn trace_handle(&self) -> *const c_void {
        std::ptr::from_ref(self).cast()
    }

    /// After the Arc exists: the event names the channel by its address.
    fn traced(self: Arc<Self>) -> Arc<Self> {
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
    fn stamp(&self, slot: u32, nbytes: u64) -> u64 {
        let Ok(at) = usize::try_from(self.stamp_offset) else {
            return 0;
        };
        if !self.seg.host_payload() || at as u64 + 8 > nbytes {
            return 0;
        }
        // SAFETY: at + 8 <= nbytes <= slot_size: inside the slot.
        let p = unsafe { self.seg.payload(slot).add(at) };
        // SAFETY: the caller owns the slot, so nobody writes it meanwhile.
        u64::from_le_bytes(unsafe { std::ptr::read_unaligned(p.cast::<[u8; 8]>()) })
    }

    fn inherited(&self) -> bool {
        sys::fork_gen() != self.born
    }

    fn gpu_wedged(&self) -> bool {
        self.gpu.is_some() && gpu::wedged().is_some()
    }

    fn index(&self) -> u32 {
        self.lease.get().map_or(0, |l| l.index)
    }

    fn reader(&self, slot: u32) -> &AtomicU32 {
        self.seg.reader(slot, self.index())
    }

    fn readers_free(&self, slot: u32, order: Ordering) -> bool {
        let n = self.seg.readers_in_use(order);
        self.seg
            .readers(slot)
            .iter()
            .take(n)
            .all(|r| r.load(order) == 0)
    }

    /// Claims `s` if no reader holds it, and leaves it as it was otherwise.
    fn try_claim(&self, s: u32) -> bool {
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
    fn reclaim_dead(&self) -> bool {
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

    fn finish_commit(&self, s: u32, meta: &FrameMeta) {
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

    fn abort(&self, s: u32) {
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
    fn end_borrow(&self, slot: u32, ticket: u64) {
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
    fn misreleased(&self, slot: u32, ticket: u64, why: &str) {
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

    fn arm_release(&self, ctl: &Arc<Control>) {
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

    fn waiter_count(&self, number: Option<u32>) -> &AtomicI32 {
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
        // Arena memory is not freed here: a frame left in a slot can still be taken after the
        // publisher is gone, so it stays until this process ends.
    }
}

/// Where an `add_waiter` counted: a lease can arrive before the matching remove.
#[derive(Clone, Copy)]
pub(crate) struct Waiter(Option<u32>);

/// A borrowed frame. While it lives, the publisher does not rewrite its slot. The header is the
/// copy `take` checked: another process can still write the one in shared memory.
pub struct FrameView {
    sh: Arc<ChannelShared>,
    slot: u32,
    ticket: u64,
    meta: FrameMeta,
}

impl FrameView {
    /// The arena the frame's trailer names, read-only. None for a frame without one.
    pub fn heap(&self) -> Option<&'static [u8]> {
        arena::resolve(self.data())
    }

    /// Empty when the payload is device memory; read it through `device_ptr`.
    pub fn data(&self) -> &[u8] {
        if !self.sh.seg.host_payload() {
            return &[];
        }
        #[allow(clippy::cast_possible_truncation)]
        let len = self.meta.nbytes as usize;
        // SAFETY: the borrow keeps the publisher off this slot, and take checked nbytes against
        // slot_size.
        unsafe { std::slice::from_raw_parts(self.sh.seg.payload(self.slot), len) }
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
        self.sh.seg.host_payload()
    }

    pub fn meta(&self) -> &FrameMeta {
        &self.meta
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

/// A claimed slot. Dropped without commit, it is aborted, and the frame the slot held before is
/// gone either way, since `data_mut` aliased it. It may be committed or dropped on another thread:
/// both touch only the shared state, as the bindings' loans do.
pub struct WriteSlot {
    sh: Arc<ChannelShared>,
    slot: u32,
    meta: FrameMeta,
    done: bool,
}

/// A loan's handle on the process arena: plain data, so a binding copies it with its writer.
/// Arena memory it hands out belongs to the loan's frame and is freed when its slot is claimed
/// again. Use it only while the loan lives.
#[derive(Clone, Copy)]
pub struct Heap {
    sh: *const ChannelShared,
    slot: u32,
    pub base: *mut u8,
    pub size: usize,
}

impl Heap {
    /// What a binding keeps: the channel and the slot.
    pub fn raw(&self) -> (*const c_void, u32) {
        (self.sh.cast(), self.slot)
    }

    /// The handle `raw` and the arena fields describe.
    pub fn from_raw(channel: *const c_void, slot: u32, base: *mut u8, size: usize) -> Heap {
        Heap {
            sh: channel.cast(),
            slot,
            base,
            size,
        }
    }

    /// `bytes` of arena memory and their arena offset.
    ///
    /// # Safety
    /// The loan this came from is live.
    pub unsafe fn alloc(&self, bytes: usize, align: usize) -> Option<(*mut u8, u64)> {
        let a = arena::arena()?;
        let off = a.alloc(bytes, align)?;
        // SAFETY: the live loan holds the channel.
        unsafe { &*self.sh }.own(self.slot, Owned::Block(off));
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
        let off = arena::arena()?.offset_of(p, bytes)?;
        // SAFETY: the live loan holds the channel.
        unsafe { &*self.sh }.own(self.slot, Owned::Adopted(release, ctx));
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

    /// The loan's way into the arena; None with no arena or on a device payload.
    pub fn heap(&self) -> Option<Heap> {
        let a = arena::arena().filter(|_| self.sh.seg.host_payload())?;
        Some(Heap {
            sh: Arc::as_ptr(&self.sh),
            slot: self.slot,
            base: a.base(),
            size: a.size(),
        })
    }

    pub fn commit(mut self) -> Published {
        let (published, end) = self.settle();
        self.end(end);
        published
    }

    fn settle(&self) -> (Published, i32) {
        if self.sh.inherited() {
            return (Published::NotPublisher, trace::LOAN_NOT_PUBLISHER);
        }
        if !fits(&self.meta, self.sh.seg.layout().slot_size) {
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

    fn end(&mut self, result: i32) {
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
        if self.sh.owns_any(self.slot)
            && let Some(a) = arena::arena()
        {
            let at = nbytes.next_multiple_of(8);
            if at + TRAILER_BYTES > self.capacity() {
                self.sh.abort(self.slot);
                self.end(trace::LOAN_TOO_LARGE);
                return Published::TooLarge;
            }
            let t = a.trailer();
            // SAFETY: at + 64 <= capacity, inside the slot.
            let dst = unsafe { self.sh.seg.payload(self.slot).add(at) };
            // SAFETY: the claim makes this the only access to the slot.
            unsafe { std::ptr::copy_nonoverlapping(t.as_ptr(), dst, TRAILER_BYTES) };
            nbytes = at + TRAILER_BYTES;
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
fn fence_failed(failed: i32, wedged: i32) -> (Published, i32) {
    if gpu::wedged().is_some() {
        (Published::GpuWedged, wedged)
    } else {
        (Published::FenceFailed, failed)
    }
}

/// Also run on every take, against a header any process could have written.
fn fits(meta: &FrameMeta, slot_size: u32) -> bool {
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
    n == Some(meta.nbytes) && meta.nbytes <= u64::from(slot_size)
}

/// itemsize follows from dtype and ndim from the shape, so they cannot contradict each other. An
/// overflowing byte count becomes `u64::MAX`, which the commit refuses.
fn meta_from(dt: DType, shape: &[u64]) -> FrameMeta {
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

/// The publisher's slots, oldest frame first. Only the publisher writes tickets, so commit order is
/// ticket order, and a claim takes the first free slot here instead of scanning for the oldest.
#[derive(Default)]
struct ClaimOrder {
    queue: VecDeque<u32>,
    /// Claimed for a loan; back in `queue` once committed or aborted.
    loaned: Vec<u32>,
    built: bool,
}

impl ClaimOrder {
    fn settle(&mut self, seg: &Segment) {
        let ticket = |s: u32| seg.slot(s).commit_ticket.load(Ordering::Relaxed);
        if !self.built {
            let mut all: Vec<(u64, u32)> = (0..seg.layout().slot_count)
                .map(|s| (ticket(s), s))
                .filter(|&(t, _)| t != TICKET_CLAIMED)
                .collect();
            all.sort_unstable();
            self.queue = all.into_iter().map(|(_, s)| s).collect();
            self.built = true;
        }
        let ClaimOrder { queue, loaned, .. } = self;
        loaned.retain(|&s| {
            let t = ticket(s);
            if t == TICKET_CLAIMED {
                return true;
            }
            match queue.iter().rposition(|&q| ticket(q) <= t) {
                Some(i) if i + 1 == queue.len() => queue.push_back(s),
                Some(i) => queue.insert(i + 1, s),
                None => queue.push_front(s),
            }
            false
        });
    }
}

/// One thread per Channel. Cursor, counters and re-attach state are plain fields; the atomics
/// cover the cross-process protocol only.
pub struct Channel {
    sh: Arc<ChannelShared>,
    qos: QoS,
    mem: MemoryPolicy,
    cursor: u64,
    lost: u64,
    stalled_polls: u32,
    last_probe_ns: u64,
    attach_gen: u32,
    signpost: Option<Arc<SignpostMap>>,
    signpost_epoch: u32,
    /// The caller's, for a re-attach: the old segment's header is writable by any peer.
    fingerprint: u64,
    orphaned: bool,
    last_seen_ticket: u64,
    /// Set by the first frame taken, so `set_qos` no longer moves the cursor.
    taken: bool,
    refused: Refused,
    /// Why the publisher of this signpost epoch refused a re-attach. Not retried until the epoch
    /// moves again.
    attach_error: Option<(u32, Error)>,
    order: ClaimOrder,
    /// Declared at construction and kept across attaches, with the fence statistics.
    stream: Stream,
    fence: Arc<FenceStats>,
    _not_sync: PhantomData<Cell<()>>,
}

fn next_attach_generation() -> u32 {
    static NEXT: AtomicU32 = AtomicU32::new(0);
    NEXT.fetch_add(1, Ordering::Relaxed)
}

fn check_slot_count(slot_count: u32) -> Result<()> {
    if slot_count < MIN_SLOT_COUNT {
        return Err(Error::InvalidArgument(format!(
            "slot_count must be >= {MIN_SLOT_COUNT}: a subscriber holding the only slot stalls the \
             publisher"
        )));
    }
    Ok(())
}

impl Channel {
    /// A process-private channel on the heap.
    pub fn new(slot_size: u32, slot_count: u32) -> Result<Channel> {
        check_slot_count(slot_count)?;
        Ok(Channel::from_segment(
            Segment::create_heap(slot_size, slot_count, 0)?,
            MemoryPolicy::default(),
            trace::NO_STAMP,
        ))
    }

    /// `name` is a signpost name (`signpost_name`). Fails with `PublisherExists` if another
    /// publisher is live on it. `max_subscribers` counts subscribing processes, not
    /// subscriptions; each one adds a counter that every claim and publish reads.
    pub fn create(
        name: &str,
        slot_size: u32,
        slot_count: u32,
        max_subscribers: u32,
        fingerprint: u64,
        mem: MemoryPolicy,
    ) -> Result<Channel> {
        Channel::create_on(
            name,
            slot_size,
            slot_count,
            max_subscribers,
            fingerprint,
            mem,
            Stream::default(),
            trace::NO_STAMP,
        )
    }

    /// As `create`, with frames produced on `stream`, declared once for the channel's life. On the
    /// device-handle route the payload is a device allocation. `stamp_offset` is where the type's
    /// `header.stamp` sits, for trace events (`trace::NO_STAMP` without one).
    #[allow(clippy::too_many_arguments)]
    pub fn create_on(
        name: &str,
        slot_size: u32,
        slot_count: u32,
        max_subscribers: u32,
        fingerprint: u64,
        mem: MemoryPolicy,
        stream: Stream,
        stamp_offset: i32,
    ) -> Result<Channel> {
        check_slot_count(slot_count)?;
        let device = if stream.declared() {
            let route = gpu::require_route(stream.device())?.route;
            (route == Route::DeviceHandle).then_some(stream.device())
        } else {
            None
        };
        let seg = open_publisher_segment(
            name,
            slot_size,
            slot_count,
            max_subscribers,
            fingerprint,
            device,
        )?;
        seg.apply(mem)?;
        let fence = Arc::new(FenceStats::default());
        let mut sh = ChannelShared::new(Arc::new(seg), stamp_offset);
        sh.attach_device(&stream, &fence)?;
        let mut ch = Channel::with_shared(Arc::new(sh).traced(), mem);
        ch.stream = stream;
        ch.fence = fence;
        Ok(ch)
    }

    /// `NotReady` until a publisher is up; retry.
    pub fn open(name: &str, fingerprint: u64, mem: MemoryPolicy) -> Result<Channel> {
        Channel::open_on(name, fingerprint, mem, Stream::default(), trace::NO_STAMP)
    }

    /// As `open`, with frames consumed on `stream`. A device payload needs one.
    pub fn open_on(
        name: &str,
        fingerprint: u64,
        mem: MemoryPolicy,
        stream: Stream,
        stamp_offset: i32,
    ) -> Result<Channel> {
        sweep_dead_once();
        sys::start_reaper();
        let signpost = Arc::new(SignpostMap::open(name)?);
        let fence = Arc::new(FenceStats::default());
        let (sh, epoch) =
            attach_subscriber(&signpost, fingerprint, mem, &stream, &fence, stamp_offset)?;
        let mut ch = Channel::attached(sh, epoch, signpost, fingerprint, mem);
        ch.stream = stream;
        ch.fence = fence;
        Ok(ch)
    }

    /// As `open`, through a watch that outlives the channel. Once the watch has attached, the next
    /// publisher's frames all came after this consumer joined, so they are read from the first.
    pub fn open_watched(
        watch: &PublisherWatch,
        fingerprint: u64,
        qos: QoS,
        mem: MemoryPolicy,
    ) -> Result<Channel> {
        Channel::open_watched_on(
            watch,
            fingerprint,
            qos,
            mem,
            Stream::default(),
            trace::NO_STAMP,
        )
    }

    pub fn open_watched_on(
        watch: &PublisherWatch,
        fingerprint: u64,
        qos: QoS,
        mem: MemoryPolicy,
        stream: Stream,
        stamp_offset: i32,
    ) -> Result<Channel> {
        qos.validate()?;
        sweep_dead_once();
        sys::start_reaper();
        watch.arm();
        let fence = Arc::new(FenceStats::default());
        let (sh, epoch) =
            attach_subscriber(&watch.map, fingerprint, mem, &stream, &fence, stamp_offset)?;
        let mut ch = Channel::attached(sh, epoch, Arc::clone(&watch.map), fingerprint, mem);
        ch.stream = stream;
        ch.fence = fence;
        ch.qos = qos;
        if watch.joined.swap(true, Ordering::Relaxed) {
            ch.cursor = 0;
        } else {
            ch.position_cursor();
        }
        Ok(ch)
    }

    fn attached(
        sh: Arc<ChannelShared>,
        epoch: u32,
        signpost: Arc<SignpostMap>,
        fingerprint: u64,
        mem: MemoryPolicy,
    ) -> Channel {
        let mut ch = Channel::with_shared(sh, mem);
        ch.signpost_epoch = epoch;
        ch.signpost = Some(signpost);
        ch.fingerprint = fingerprint;
        ch
    }

    pub(crate) fn from_segment(seg: Segment, mem: MemoryPolicy, stamp_offset: i32) -> Channel {
        Channel::with_shared(
            Arc::new(ChannelShared::new(Arc::new(seg), stamp_offset)).traced(),
            mem,
        )
    }

    fn with_shared(sh: Arc<ChannelShared>, mem: MemoryPolicy) -> Channel {
        let mut ch = Channel {
            sh,
            qos: QoS::default(),
            mem,
            cursor: 0,
            lost: 0,
            stalled_polls: 0,
            last_probe_ns: 0,
            attach_gen: next_attach_generation(),
            signpost: None,
            signpost_epoch: 0,
            fingerprint: 0,
            orphaned: false,
            last_seen_ticket: 0,
            taken: false,
            refused: Refused::default(),
            attach_error: None,
            order: ClaimOrder::default(),
            stream: Stream::default(),
            fence: Arc::new(FenceStats::default()),
            _not_sync: PhantomData,
        };
        ch.position_cursor();
        ch
    }

    fn can_publish(&self) -> bool {
        (self.sh.seg.is_publisher() || self.sh.seg.is_heap()) && !self.sh.inherited()
    }

    pub fn publish(&mut self, data: &[u8]) -> Published {
        self.publish_meta(data, meta_from(DType::U8, &[data.len() as u64]))
    }

    pub fn publish_typed(&mut self, data: &[u8], dt: DType, shape: &[u64]) -> Published {
        self.publish_meta(data, meta_from(dt, shape))
    }

    fn publish_meta(&mut self, data: &[u8], meta: FrameMeta) -> Published {
        let refused = if !self.can_publish() {
            Some((Published::NotPublisher, trace::CLAIM_NOT_PUBLISHER))
        } else if !self.sh.seg.host_payload() {
            Some((Published::WrongDevice, trace::CLAIM_WRONG_DEVICE))
        } else if !fits(&meta, self.slot_size()) || data.len() as u64 != meta.nbytes {
            Some((Published::TooLarge, trace::CLAIM_TOO_LARGE))
        } else if !matches!(self.sh.fence(Seam::Commit), Fenced::Done) {
            // The stream is the caller's statement of where frames come from, so its work is
            // waited for before the source is read.
            Some(fence_failed(
                trace::CLAIM_FENCE_FAILED,
                trace::CLAIM_GPU_WEDGED,
            ))
        } else {
            None
        };
        if let Some((published, why)) = refused {
            event!(flux_trace_claim(self.trace_handle(), trace::NONE, why, 0));
            return published;
        }
        let Some(s) = self.claim(0) else {
            return Published::Backpressure;
        };
        // SAFETY: the claim makes this the only access to the slot, and nbytes <= slot_size.
        unsafe { std::ptr::copy_nonoverlapping(data.as_ptr(), self.sh.seg.payload(s), data.len()) };
        self.sh.finish_commit(s, &meta);
        self.order.queue.push_back(s);
        Published::Ok
    }

    /// The whole slot as one u8 run; commit the prefix written with `commit_len`.
    pub fn loan(&mut self) -> Option<WriteSlot> {
        self.loan_typed(DType::U8, &[u64::from(self.slot_size())])
    }

    /// The shape is checked at commit, the one place the caller already reads an outcome.
    pub fn loan_typed(&mut self, dt: DType, shape: &[u64]) -> Option<WriteSlot> {
        // A wedged GPU fails every commit, and each failed commit keeps its slot.
        let refused = if !self.can_publish() {
            trace::CLAIM_NOT_PUBLISHER
        } else if self.sh.gpu_wedged() {
            trace::CLAIM_GPU_WEDGED
        } else {
            trace::CLAIM_OK
        };
        if refused != trace::CLAIM_OK {
            event!(flux_trace_claim(
                self.trace_handle(),
                trace::NONE,
                refused,
                1
            ));
            return None;
        }
        let slot = self.claim(1)?;
        self.order.loaned.push(slot);
        event!(flux_trace_loan(self.trace_handle(), slot));
        Some(WriteSlot {
            sh: Arc::clone(&self.sh),
            slot,
            meta: meta_from(dt, shape),
            done: false,
        })
    }

    /// The oldest free frame goes first, so the newest are overwritten last. `loan` is for the
    /// trace event.
    fn claim(&mut self, loan: i32) -> Option<u32> {
        self.sh.reap_late();
        self.order.settle(&self.sh.seg);
        for result in [trace::CLAIM_OK, trace::CLAIM_RECLAIMED] {
            let found = self.order.queue.iter().position(|&s| self.sh.try_claim(s));
            if let Some(slot) = found.and_then(|i| self.order.queue.remove(i)) {
                self.sh.free_owned(slot);
                event!(flux_trace_claim(self.trace_handle(), slot, result, loan));
                return Some(slot);
            }
            if result == trace::CLAIM_OK && self.sh.reclaim_dead() {
                continue;
            }
            break;
        }
        self.sh.dropped.fetch_add(1, Ordering::Relaxed);
        event!(flux_trace_claim(
            self.trace_handle(),
            trace::NONE,
            trace::CLAIM_BACKPRESSURE,
            loan
        ));
        None
    }

    /// The oldest frame newer than the cursor, within `depth` of the newest.
    pub fn take(&mut self) -> Option<FrameView> {
        self.sh.reap_late();
        if !self.begin_borrow() {
            return None;
        }
        let mut attempt = 0;
        while attempt < MAX_RETRIES {
            attempt += 1;
            let latest = self.sh.seg.ctrl().latest.load(Ordering::Acquire);
            if latest == 0 {
                break;
            }
            let floor = latest.saturating_sub(u64::from(self.qos.depth()));
            if self.cursor < floor {
                // Counted here: this take may still return nothing. Saturating: `latest` is
                // whatever a peer wrote, and debug builds check overflow.
                self.lost = self.lost.saturating_add(floor - self.cursor);
                self.cursor = floor;
            }

            // Never infer a restart from tickets: a commit stores the slot's ticket before it moves
            // `latest`, so the cursor may sit above it for a moment. A restart is a new segment.
            let Some((s, t)) = self.next_frame(latest) else {
                break;
            };

            let sl = self.sh.seg.slot(s);
            let counter = self.sh.reader(s);
            counter.fetch_add(1, Ordering::SeqCst);
            if sl.commit_ticket.load(Ordering::SeqCst) != t {
                counter.fetch_sub(1, Ordering::Release);
                continue;
            }
            // SAFETY: borrowed with the ticket unchanged, so the publisher is off this slot.
            let meta = unsafe { *sl.meta.get() };
            if t > self.cursor + 1 {
                self.lost = self.lost.saturating_add(t - self.cursor - 1);
            }
            self.cursor = t;
            if !fits(&meta, self.slot_size()) {
                counter.fetch_sub(1, Ordering::Release);
                self.refused.bad_frame += 1;
                event!(flux_trace_refused(
                    self.sh.trace_handle(),
                    trace::REFUSED_BAD_FRAME
                ));
                continue;
            }
            self.stalled_polls = 0;
            self.last_seen_ticket = t;
            self.taken = true;
            if self.sh.gpu.is_some() {
                ledger::borrowed(self.sh.trace_handle(), s);
            }
            event!(flux_trace_take(
                self.sh.trace_handle(),
                t,
                s,
                self.lost,
                self.sh.stamp(s, meta.nbytes),
            ));
            return Some(FrameView {
                sh: Arc::clone(&self.sh),
                slot: s,
                ticket: t,
                meta,
            });
        }
        if attempt == MAX_RETRIES {
            self.refused.contended += 1;
            event!(flux_trace_refused(
                self.sh.trace_handle(),
                trace::REFUSED_CONTENDED
            ));
        }
        self.sh.outstanding.fetch_sub(1, Ordering::Release);
        self.stalled_polls += 1;
        None
    }

    /// The oldest frame in (cursor, latest], found through the ticket index. A cursor further
    /// behind than the index reaches scans every slot instead.
    fn next_frame(&self, latest: u64) -> Option<(u32, u64)> {
        let seg = &self.sh.seg;
        let n = seg.layout().slot_count;
        let ticket = |s: u32| seg.slot(s).commit_ticket.load(Ordering::Relaxed);
        if latest.saturating_sub(self.cursor) > seg.layout().index_len() as u64 {
            return (0..n)
                .map(|s| (s, ticket(s)))
                .filter(|&(_, t)| t != TICKET_CLAIMED && t > self.cursor)
                .min_by_key(|&(_, t)| t);
        }
        (self.cursor.saturating_add(1)..=latest.min(TICKET_CLAIMED - 1)).find_map(|t| {
            let s = seg.index_entry(t).load(Ordering::Relaxed);
            (s < n && ticket(s) == t).then_some((s, t))
        })
    }

    /// `timeout_ns` < 0 waits forever.
    pub fn take_blocking(&mut self, timeout_ns: i64) -> Option<FrameView> {
        let deadline = (timeout_ns >= 0).then(|| sys::monotonic_ns() + timeout_ns.unsigned_abs());
        loop {
            let seq = self.wake_seq();
            if let Some(v) = self.take() {
                return Some(v);
            }
            // Only the caller can free a lease, and not while parked here. An inherited channel
            // never borrows, and a refused re-attach waits for a restart, so neither would end.
            if !self.can_borrow()
                || self.sh.inherited()
                || self.attach_error.is_some()
                || self.sh.gpu_wedged()
            {
                return None;
            }
            let park = match deadline {
                None => MAX_PARK_NS,
                Some(d) => {
                    let now = sys::monotonic_ns();
                    if now >= d {
                        return None;
                    }
                    i64::try_from(d - now)
                        .unwrap_or(MAX_PARK_NS)
                        .min(MAX_PARK_NS)
                }
            };
            self.sh.wait(seq, park);
        }
    }

    fn begin_borrow(&mut self) -> bool {
        if self.sh.inherited() {
            self.refused.no_reader += 1;
            event!(flux_trace_refused(
                self.sh.trace_handle(),
                trace::REFUSED_NO_READER
            ));
            return false;
        }
        if self.sh.gpu_wedged() {
            self.refused.fence += 1;
            event!(flux_trace_refused(
                self.sh.trace_handle(),
                trace::REFUSED_GPU_WEDGED
            ));
            return false;
        }
        self.maybe_reattach();
        if self.sh.lease.get().is_none() {
            let Ok(l) = reader::lease_for(&self.sh.seg) else {
                self.refused.no_reader += 1;
                event!(flux_trace_refused(
                    self.sh.trace_handle(),
                    trace::REFUSED_NO_READER
                ));
                return false;
            };
            let _ = self.sh.lease.set(l);
        }
        if self.sh.outstanding.fetch_add(1, Ordering::AcqRel) >= self.qos.borrow_limit() {
            self.sh.outstanding.fetch_sub(1, Ordering::Release);
            // Leaked borrows are never given back, so no release the caller makes can help.
            if self.sh.leases_leaked.load(Ordering::Acquire) >= self.qos.borrow_limit() {
                self.refused.fence += 1;
                event!(flux_trace_refused(
                    self.sh.trace_handle(),
                    trace::REFUSED_FENCE
                ));
            } else {
                self.refused.max_borrow += 1;
                event!(flux_trace_refused(
                    self.sh.trace_handle(),
                    trace::REFUSED_MAX_BORROW
                ));
            }
            return false;
        }
        true
    }

    /// On joining the stream, so volatile means from where this consumer joined.
    fn position_cursor(&mut self) {
        let newest = self.sh.seg.ctrl().latest.load(Ordering::Acquire);
        self.cursor = newest.saturating_sub(u64::from(self.qos.durability().replay));
    }

    /// A rotation check is one load from a mapping already held, so every stalled take does it,
    /// and so does a take at `max_borrow`, which never stalls. The orphan probe costs syscalls:
    /// every `PROBE_EVERY` stalled takes, at most once per `PROBE_NS`.
    fn maybe_reattach(&mut self) {
        let Some(signpost) = self.signpost.as_ref() else {
            return;
        };
        if self.stalled_polls == 0 && self.can_borrow() {
            return;
        }
        let rotated = self.attachable(signpost.epoch());
        if !rotated && self.stalled_polls < PROBE_EVERY {
            return;
        }
        if !rotated {
            self.stalled_polls = 0;
            let now = sys::monotonic_ns();
            if now.saturating_sub(self.last_probe_ns) < PROBE_NS {
                return;
            }
            self.last_probe_ns = now;
        }
        if self.reattach_if_replaced() {
            self.last_seen_ticket = 0;
            self.stalled_polls = 0;
        }
    }

    /// An epoch other than the one attached to, and not one that already refused this channel.
    fn attachable(&self, epoch: u32) -> bool {
        epoch != self.signpost_epoch && self.attach_error.as_ref().is_none_or(|(e, _)| *e != epoch)
    }

    /// Views still out keep the old mapping alive through their own `Arc` and give their borrow
    /// back to it, so they do not count against `max_borrow` on the new one. The new publisher's
    /// frames all came after this consumer joined, so it reads them from the first.
    fn reattach_if_replaced(&mut self) -> bool {
        let Some(signpost) = self.signpost.as_ref() else {
            return false;
        };
        let epoch = signpost.epoch();
        if !self.attachable(epoch) {
            if !self.orphaned {
                self.orphaned = segment_publisher_dead(self.sh.seg.name());
            }
            return false;
        }
        let attached = attach_subscriber(
            signpost,
            self.fingerprint,
            self.mem,
            &self.stream,
            &self.fence,
            self.sh.stamp_offset,
        );
        let (sh, epoch) = match attached {
            Ok(attached) => attached,
            Err(Error::NotReady(_)) => return false,
            Err(e) => {
                self.attach_error = Some((epoch, e));
                return false;
            }
        };
        self.sh.orphan_late();
        sys::retire(std::mem::replace(&mut self.sh, sh));
        self.signpost_epoch = epoch;
        self.cursor = 0;
        self.orphaned = false;
        self.attach_error = None;
        self.attach_gen = next_attach_generation();
        true
    }

    /// Advisory: a lapped or refused frame makes the next take empty.
    pub fn ready(&self) -> bool {
        self.sh.seg.ctrl().latest.load(Ordering::Acquire) > self.cursor
    }

    /// A take refused for this cannot be unblocked by a publish. Deferred releases whose work is
    /// done are given back first.
    pub fn can_borrow(&self) -> bool {
        self.sh.reap_late();
        self.sh.outstanding.load(Ordering::SeqCst) < self.qos.borrow_limit()
    }

    /// For an executor about to wait while this channel is at `max_borrow`: the view release that
    /// frees a borrow pokes `ctl`. False if one already did, so there is nothing to wait for.
    pub(crate) fn wake_on_release(&self, ctl: &Arc<Control>) -> bool {
        if self.sh.inherited() {
            return true;
        }
        self.sh.arm_release(ctl);
        if self.can_borrow() {
            self.sh.release_armed.store(false, Ordering::Relaxed);
            return false;
        }
        true
    }

    /// Before the first take it also sets where reading starts. After it, a new depth applies at
    /// the next take, which counts what falls behind it in `lost`, and the replay is unused.
    pub fn set_qos(&mut self, q: QoS) -> Result<()> {
        q.validate()?;
        self.qos = q;
        if !self.taken {
            self.position_cursor();
        }
        Ok(())
    }

    pub fn qos(&self) -> &QoS {
        &self.qos
    }
    pub fn lost(&self) -> u64 {
        self.lost
    }
    pub fn dropped(&self) -> u64 {
        self.sh.dropped.load(Ordering::Relaxed)
    }
    pub fn cursor(&self) -> u64 {
        self.cursor
    }
    pub fn refused(&self) -> &Refused {
        &self.refused
    }
    /// Unique within the process. A re-attach changes it.
    pub fn attach_generation(&self) -> u32 {
        self.attach_gen
    }
    /// Why the restarted publisher refused this consumer, until it restarts again. Takes find
    /// nothing meanwhile.
    pub fn attach_error(&self) -> Option<&Error> {
        self.attach_error.as_ref().map(|(_, e)| e)
    }
    /// The publisher is dead and no rotation has come.
    pub fn orphaned(&self) -> bool {
        self.orphaned
    }
    /// Opened before a `fork()` this process came out of: it borrows and publishes nothing.
    pub fn inherited(&self) -> bool {
        self.sh.inherited()
    }

    /// Sample before the take that finds nothing, then pass to `wait`: no lost wakeup.
    pub fn wake_seq(&self) -> u32 {
        self.sh.wake_seq()
    }
    pub(crate) fn trace_handle(&self) -> *const c_void {
        self.sh.trace_handle()
    }
    pub(crate) fn trace_segment(&self) -> *const c_char {
        self.sh.trace_name.as_ptr()
    }
    /// Keeps the wake word mapped across a re-attach or the Channel's end.
    pub(crate) fn wait_handle(&self) -> Arc<ChannelShared> {
        Arc::clone(&self.sh)
    }
    pub fn wait(&self, last_seq: u32, timeout_ns: i64) -> bool {
        self.sh.wait(last_seq, timeout_ns)
    }

    pub fn slot_size(&self) -> u32 {
        self.sh.seg.layout().slot_size
    }
    pub fn slot_count(&self) -> u32 {
        self.sh.seg.layout().slot_count
    }
    pub fn payload_readonly(&self) -> bool {
        self.sh.seg.payload_readonly()
    }
    pub fn pages_committed(&self) -> bool {
        self.sh.seg.pages_committed()
    }
    /// The stream declared at construction; undeclared for a host channel.
    pub fn stream(&self) -> &Stream {
        &self.stream
    }

    /// Commits and releases that could not wait for the stream, each holding a slot for good, or
    /// until its work is done when only the fence timeout ran out.
    /// Releases refused because the borrow they would give back was already gone: a view released
    /// twice. Each one found at 0 holds its slot until the publisher restarts.
    pub fn misreleased(&self) -> u64 {
        self.sh.misreleased.load(Ordering::Relaxed)
    }

    pub fn fence_failed(&self) -> u64 {
        self.fence.failed()
    }

    pub fn fence_wait(&self) -> FenceWait {
        self.fence.snapshot()
    }

    /// Bounds each commit and release fence, 0 for none (the default). A wait that runs out counts
    /// as a failed fence and keeps its slot; once the work is done, the next take or claim gives
    /// the slot back.
    pub fn set_fence_timeout(&self, timeout_ns: u64) {
        self.fence.timeout_ns.store(timeout_ns, Ordering::Relaxed);
        if timeout_ns > 0 {
            self.stream.prepare_waiter();
        }
    }

    /// `Deferred`: a view's release records an event and returns at once, and the borrow goes
    /// back only after the device finished with the slot, at the next take or executor pass.
    /// Until then it counts against `max_borrow`. The default, `Wait`, waits in the release.
    pub fn set_release_mode(&self, mode: ReleaseMode) {
        let deferred = mode == ReleaseMode::Deferred;
        self.fence.deferred.store(deferred, Ordering::Relaxed);
        if deferred {
            self.stream.prepare_waiter();
        }
    }

    pub fn release_mode(&self) -> ReleaseMode {
        if self.fence.deferred.load(Ordering::Relaxed) {
            ReleaseMode::Deferred
        } else {
            ReleaseMode::Wait
        }
    }

    /// For the executor serving this channel, each pass: the waiter pokes it when deferred work
    /// is done, and the pass gives back what that work held.
    pub(crate) fn serve_late(&self, ctl: &Arc<Control>) {
        let id = Arc::as_ptr(ctl) as usize;
        if self.sh.gpu_waker_of.load(Ordering::Relaxed) != id {
            *self.sh.gpu_waker.lock() = Arc::downgrade(ctl);
            self.sh.gpu_waker_of.store(id, Ordering::Relaxed);
        }
        self.sh.reap_late();
    }

    /// The payload is device memory: `data` is empty and `device_ptr` is the only way in.
    pub fn device_backed(&self) -> bool {
        !self.sh.seg.host_payload()
    }

    pub fn pages_locked(&self) -> bool {
        self.sh.seg.pages_locked()
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        self.sh.orphan_late();
    }
}

/// A signpost held by a subscription that has no publisher to attach to, so a waiter can sleep
/// until one comes: a publisher wakes the epoch word when it stores its epoch.
pub struct PublisherWatch {
    map: Arc<SignpostMap>,
    armed: AtomicU32,
    joined: AtomicBool,
}

impl PublisherWatch {
    /// Creates the signpost if absent.
    pub fn open(name: &str) -> Result<PublisherWatch> {
        Ok(PublisherWatch {
            map: Arc::new(SignpostMap::open(name)?),
            armed: AtomicU32::new(0),
            joined: AtomicBool::new(false),
        })
    }

    /// 0 until the first publisher; it changes with every restart.
    pub fn epoch(&self) -> u32 {
        self.map.epoch()
    }

    /// Notes the epoch now, before an attempt to attach. Returns it.
    pub fn arm(&self) -> u32 {
        let e = self.epoch();
        self.armed.store(e, Ordering::Relaxed);
        e
    }

    /// The epoch the last `arm` saw. A wait returns once the word has moved from it.
    pub fn armed(&self) -> u32 {
        self.armed.load(Ordering::Relaxed)
    }

    pub(crate) fn word(&self) -> &AtomicU32 {
        self.map.epoch_word()
    }

    pub(crate) fn signpost(&self) -> &Arc<SignpostMap> {
        &self.map
    }

    /// Until the epoch moves from `armed`, or a signal or stray wake: true asks for a look. False
    /// only on a timeout; `timeout_ns` < 0 waits forever.
    pub fn wait(&self, timeout_ns: i64) -> bool {
        let armed = self.armed();
        self.epoch() != armed || sys::futex_wait(self.word(), armed, timeout_ns)
    }
}

fn attach_subscriber(
    signpost: &SignpostMap,
    fingerprint: u64,
    mem: MemoryPolicy,
    stream: &Stream,
    fence: &Arc<FenceStats>,
    stamp_offset: i32,
) -> Result<(Arc<ChannelShared>, u32)> {
    let (seg, lease, epoch) = reader::subscribe(signpost, fingerprint, mem)?;
    let mut sh = ChannelShared::new(seg, stamp_offset);
    sh.attach_device(stream, fence)?;
    let _ = sh.lease.set(lease);
    Ok((Arc::new(sh).traced(), epoch))
}

/// The real orderings under Miri (`cargo miri test`). Subscribers stand for processes, each with
/// its own `ChannelShared` and subscriber number. A broken claim, commit or release ordering is a
/// data race on the payload or the meta; a broken wake Dekker leaves a waiter asleep forever, which
/// Miri reports as a deadlock.
#[cfg(test)]
mod race {
    use std::collections::VecDeque;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    use super::{Channel, ChannelShared, Published};
    use crate::executor::Control;
    use crate::qos::{MemoryPolicy, QoS};
    use crate::reader::ReaderLease;
    use crate::segment::Segment;

    const SLOT: usize = 16;
    const SLOTS: u32 = 3;
    const FRAMES: u8 = if cfg!(miri) { 12 } else { 200 };

    fn channel(sh: &Arc<ChannelShared>) -> Channel {
        let mut ch = Channel::with_shared(Arc::clone(sh), MemoryPolicy::default());
        let q = QoS::new(SLOTS).and_then(|q| q.max_borrow(2)).unwrap();
        ch.set_qos(q).unwrap();
        ch
    }

    fn subscriber(seg: &Segment, number: u32) -> Arc<ChannelShared> {
        let sh = ChannelShared::new(Arc::new(seg.alias()), crate::trace::NO_STAMP);
        let _ = sh.lease.set(Arc::new(ReaderLease::unlocked(number)));
        Arc::new(sh)
    }

    #[test]
    fn a_held_view_is_never_rewritten() {
        let publisher_side = Arc::new(ChannelShared::new(
            Arc::new(Segment::create_heap(u32::try_from(SLOT).unwrap(), SLOTS, 0).unwrap()),
            crate::trace::NO_STAMP,
        ));
        let subs = [1, 2].map(|n| subscriber(&publisher_side.seg, n));
        let done = AtomicBool::new(false);
        std::thread::scope(|s| {
            let mut publisher = channel(&publisher_side);
            let done = &done;
            s.spawn(move || {
                for id in 1..=FRAMES {
                    let r = publisher.publish(&[id; SLOT]);
                    assert!(matches!(r, Published::Ok | Published::Backpressure));
                }
                done.store(true, Ordering::Release);
            });
            for (sh, number) in subs.iter().zip([1, 2]) {
                let mut sub = channel(sh);
                s.spawn(move || {
                    // While the publisher claims, as a subscriber joining a running channel does.
                    sh.seg.raise_readers(number);
                    let mut held = VecDeque::new();
                    while !done.load(Ordering::Acquire) || sub.ready() {
                        if let Some(v) = sub.take() {
                            let first = v.data()[0];
                            assert!(v.data().iter().all(|&b| b == first), "a torn frame");
                            held.push_back((v, first));
                        }
                        if (held.len() == 2 || done.load(Ordering::Acquire))
                            && let Some((v, first)) = held.pop_front()
                        {
                            assert!(v.data().iter().all(|&b| b == first), "rewritten while held");
                        }
                        std::thread::yield_now();
                    }
                });
            }
        });
    }

    /// One slot, so every publish claims the slot every take borrows: the Dekker pair meets on
    /// each frame, and a claim that misses a borrow writes the payload under it. The depth is
    /// past the ticket index, so a take that falls behind scans the slot and can find a ticket
    /// newer than the `latest` it read: only the ticket's own Release orders that frame.
    #[test]
    fn a_claim_and_a_borrow_of_one_slot_exclude_each_other() {
        let publisher_side = Arc::new(ChannelShared::new(
            Arc::new(Segment::create_heap(u32::try_from(SLOT).unwrap(), 1, 0).unwrap()),
            crate::trace::NO_STAMP,
        ));
        let sh = subscriber(&publisher_side.seg, 1);
        let done = AtomicBool::new(false);
        let one = |sh: &Arc<ChannelShared>| {
            let mut ch = Channel::with_shared(Arc::clone(sh), MemoryPolicy::default());
            ch.set_qos(QoS::new(64).unwrap()).unwrap();
            ch
        };
        let mut sub = one(&sh);
        sh.seg.raise_readers(1);
        std::thread::scope(|s| {
            let mut publisher = one(&publisher_side);
            let done = &done;
            s.spawn(move || {
                for id in 1..=FRAMES {
                    let r = publisher.publish(&[id; SLOT]);
                    assert!(matches!(r, Published::Ok | Published::Backpressure));
                }
                done.store(true, Ordering::Release);
            });
            while !done.load(Ordering::Acquire) {
                if let Some(v) = sub.take() {
                    let first = v.data()[0];
                    assert!(v.data().iter().all(|&b| b == first), "a torn frame");
                }
            }
        });
    }

    fn heap() -> Arc<ChannelShared> {
        Arc::new(ChannelShared::new(
            Arc::new(Segment::create_heap(u32::try_from(SLOT).unwrap(), SLOTS, 0).unwrap()),
            crate::trace::NO_STAMP,
        ))
    }

    /// The wake gate: the publisher bumps the word and then reads the waiter counts, the waiter
    /// counts itself and then reads the word. One waiter counts under its subscriber number, the
    /// other, in the publisher's process, under the header. The waits have no timeout.
    #[test]
    fn a_waiting_subscriber_sees_the_last_frame() {
        fn wait_for_last(sh: &Arc<ChannelShared>, mut sub: Channel) {
            let mut last = 0;
            while last < FRAMES {
                let seq = sh.wake_seq();
                match sub.take() {
                    Some(v) => last = v.data()[0],
                    None => {
                        sh.wait(seq, -1);
                    }
                }
            }
        }
        let publisher_side = heap();
        let sh = subscriber(&publisher_side.seg, 1);
        sh.seg.raise_readers(1);
        let (numbered, numberless) = (channel(&sh), channel(&publisher_side));
        std::thread::scope(|s| {
            let mut publisher = channel(&publisher_side);
            s.spawn(move || {
                for id in 1..=FRAMES {
                    // A stale counter can make every slot look held for a moment.
                    while publisher.publish(&[id; SLOT]) == Published::Backpressure {
                        std::thread::yield_now();
                    }
                }
            });
            s.spawn(|| wait_for_last(&publisher_side, numberless));
            wait_for_last(&sh, numbered);
        });
    }

    /// A view released on another thread while the executor arms the wake at `max_borrow`.
    #[test]
    fn a_release_wakes_an_executor_at_max_borrow() {
        let sh = heap();
        let mut publisher = channel(&sh);
        let mut sub = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
        sub.set_qos(QoS::new(SLOTS).and_then(|q| q.max_borrow(1)).unwrap())
            .unwrap();
        let ctl = Control::new();
        for id in 1..=16 {
            assert_eq!(publisher.publish(&[id; SLOT]), Published::Ok);
            let v = sub.take().unwrap();
            std::thread::scope(|s| {
                s.spawn(move || drop(v));
                while sub.wake_on_release(&ctl) {
                    ctl.wait(-1);
                }
            });
        }
    }

    /// The commit writes the ticket index before it moves `latest`, so a take that read `latest`
    /// finds every frame through the index. Nothing is overwritten, so nothing may be lost.
    #[test]
    fn a_take_finds_every_frame_through_the_index() {
        let publisher_side = heap();
        let sh = subscriber(&publisher_side.seg, 1);
        let done = AtomicBool::new(false);
        // Reading starts where the subscription joined, so it joins first.
        let mut sub = channel(&sh);
        sh.seg.raise_readers(1);
        std::thread::scope(|s| {
            let mut publisher = channel(&publisher_side);
            let done = &done;
            s.spawn(move || {
                for id in 1..=u8::try_from(SLOTS).unwrap() {
                    assert_eq!(publisher.publish(&[id; SLOT]), Published::Ok);
                }
                done.store(true, Ordering::Release);
            });
            let mut next = 1;
            while !done.load(Ordering::Acquire) || sub.ready() {
                if let Some(v) = sub.take() {
                    assert_eq!(v.data()[0], next, "a frame was skipped");
                    next += 1;
                }
                std::thread::yield_now();
            }
            assert_eq!(sub.lost(), 0);
        });
    }
}

#[cfg(test)]
mod header {
    use std::sync::Arc;
    use std::sync::atomic::Ordering;

    use super::{Channel, ChannelShared, Published};
    use crate::layout::{DType, MAX_DIMS};
    use crate::qos::{MemoryPolicy, QoS};
    use crate::segment::Segment;

    fn pair() -> (Arc<ChannelShared>, Channel, Channel) {
        let sh = Arc::new(ChannelShared::new(
            Arc::new(Segment::create_heap(64, 2, 0).unwrap()),
            crate::trace::NO_STAMP,
        ));
        let p = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
        let s = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
        (sh, p, s)
    }

    fn scribble(sh: &ChannelShared, slot: u32) {
        // SAFETY: single-threaded test; stands in for another process writing the header.
        let m = unsafe { &mut *sh.seg.slot(slot).meta.get() };
        m.ndim = 8;
        m.shape = [1 << 20; MAX_DIMS];
        m.nbytes = u64::MAX;
    }

    /// A device-backed segment maps no payload, so a peer that flips its storage kind afterwards
    /// would have a view read past the mapping.
    #[test]
    fn a_storage_kind_rewritten_after_attach_is_not_believed() {
        use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};

        use crate::layout::{FrameMeta, SegmentLayout, StorageKind};
        let layout = SegmentLayout {
            slot_size: 64,
            slot_count: 2,
            max_subscribers: 1,
        };
        // SAFETY: a valid C string and no other arguments.
        let raw = unsafe { libc::memfd_create(c"flux-kind".as_ptr(), libc::MFD_CLOEXEC) };
        // SAFETY: a fresh descriptor this process owns.
        let file = std::fs::File::from(unsafe { OwnedFd::from_raw_fd(raw) });
        file.set_len(layout.payload_offset() as u64).unwrap();
        let mine = crate::sys::map_shared(file.as_raw_fd(), layout.payload_offset()).unwrap();
        // SAFETY: a fresh mapping of payload_offset bytes, as a device-backed header needs.
        unsafe { crate::segment::init_header(mine, layout, 7, Some((0, [1; 16]))) };
        let (base, len, header) =
            crate::discovery::map_validate(file.as_raw_fd(), "kind", 7).unwrap();
        let id = crate::segment::SegmentId::default();
        let seg = Segment::adopt_shm(base, len, header, String::new(), id);
        let mut ch = Channel::from_segment(seg, MemoryPolicy::default(), crate::trace::NO_STAMP);
        let seg = &ch.sh.seg;
        let meta = FrameMeta {
            ndim: 1,
            itemsize: 1,
            nbytes: 64,
            shape: [64, 0, 0, 0, 0, 0, 0, 0],
            ..FrameMeta::default()
        };
        // SAFETY: nothing else uses the slot; stands in for the publisher's commit.
        unsafe { seg.slot(0).meta.get().write(meta) };
        seg.slot(0).commit_ticket.store(1, Ordering::Release);
        seg.ctrl().latest.store(1, Ordering::Release);
        let kind = std::mem::offset_of!(crate::layout::ControlHeader, storage_kind);
        let at = mine.as_ptr().wrapping_add(kind);
        // SAFETY: inside the peer's mapping of the header; stands in for its write.
        unsafe { at.write(StorageKind::HostInline as u8) };
        let v = ch.take().unwrap();
        assert!(v.data().is_empty());
        // SAFETY: mapped above with this length; nothing refers to it.
        unsafe { crate::sys::munmap(mine, layout.payload_offset()) };
    }

    #[test]
    fn a_view_keeps_the_header_take_checked() {
        let (sh, mut p, mut s) = pair();
        assert_eq!(
            p.publish_typed(&[0; 24], DType::F32, &[2, 3]),
            Published::Ok
        );
        let v = s.take().unwrap();
        scribble(&sh, v.slot);
        assert_eq!(v.meta().shape(), &[2, 3]);
        assert_eq!(v.data().len(), 24);
    }

    #[test]
    fn a_header_that_disagrees_with_itself_is_refused() {
        let (sh, mut p, mut s) = pair();
        assert_eq!(p.publish(&[0; 8]), Published::Ok);
        for slot in 0..2 {
            // SAFETY: as scribble.
            let m = unsafe { &mut *sh.seg.slot(slot).meta.get() };
            m.shape[0] = 9;
        }
        assert!(s.take().is_none());
        assert_eq!(s.refused().bad_frame, 1);
    }

    #[test]
    fn a_bad_frame_is_passed_over_once() {
        let (sh, mut p, mut s) = pair();
        s.set_qos(QoS::new(2).unwrap()).unwrap();
        assert_eq!(p.publish(&[0; 8]), Published::Ok);
        assert_eq!(p.publish(&[1; 8]), Published::Ok);
        let first = (0..2)
            .find(|&slot| sh.seg.slot(slot).commit_ticket.load(Ordering::Relaxed) == 1)
            .unwrap();
        // SAFETY: as scribble.
        unsafe { &mut *sh.seg.slot(first).meta.get() }.dtype = 0xEE;
        assert_eq!(s.take().unwrap().data(), &[1; 8]);
        assert!(s.take().is_none());
        assert_eq!(s.refused().bad_frame, 1);
        assert_eq!(s.lost(), 0);
    }

    #[test]
    fn a_qos_set_mid_stream_neither_repeats_nor_hides_frames() {
        let (_sh, mut p, mut s) = pair();
        s.set_qos(QoS::new(2).unwrap()).unwrap();
        assert_eq!(p.publish(&[0; 8]), Published::Ok);
        assert_eq!(s.take().unwrap().data(), &[0; 8]);
        s.set_qos(QoS::new(1).unwrap().transient_local(1)).unwrap();
        assert_eq!(p.publish(&[1; 8]), Published::Ok);
        assert_eq!(p.publish(&[2; 8]), Published::Ok);
        assert_eq!(s.take().unwrap().data(), &[2; 8]);
        assert_eq!(s.lost(), 1);
        assert!(s.take().is_none());
    }

    /// A skipped frame spends a retry as a lost race does, so a run of bad frames as long as
    /// the budget ends the take as contended.
    #[test]
    fn a_take_out_of_retries_counts_contended() {
        let sh = Arc::new(ChannelShared::new(
            Arc::new(Segment::create_heap(8, 64, 0).unwrap()),
            crate::trace::NO_STAMP,
        ));
        let mut p = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
        let mut s = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
        s.set_qos(QoS::new(64).unwrap()).unwrap();
        for i in 0..64 {
            assert_eq!(p.publish(&[i; 8]), Published::Ok);
        }
        for slot in 0..64 {
            // SAFETY: as scribble.
            unsafe { &mut *sh.seg.slot(slot).meta.get() }.dtype = 0xEE;
        }
        assert!(s.take().is_none());
        assert_eq!((s.refused().bad_frame, s.refused().contended), (64, 1));
    }

    #[test]
    fn a_frame_of_unknown_dtype_is_refused() {
        let (sh, mut p, mut s) = pair();
        assert_eq!(p.publish(&[0; 8]), Published::Ok);
        for slot in 0..2 {
            // SAFETY: as scribble.
            unsafe { &mut *sh.seg.slot(slot).meta.get() }.dtype = 0xEE;
        }
        assert!(s.take().is_none());
        assert_eq!(s.refused().bad_frame, 1);
    }

    #[test]
    fn a_loan_commits_on_another_thread() {
        let (_sh, mut p, mut s) = pair();
        let mut w = p.loan().unwrap();
        w.data_mut()[..8].copy_from_slice(&[3; 8]);
        let published = std::thread::spawn(move || w.commit_len(8)).join().unwrap();
        assert_eq!(published, Published::Ok);
        assert_eq!(s.take().unwrap().data(), &[3; 8]);
    }

    #[test]
    fn commit_and_take_carry_the_frames_stamp() {
        let sh = Arc::new(ChannelShared::new(
            Arc::new(Segment::create_heap(64, 2, 0).unwrap()),
            4,
        ));
        let mut p = Channel::with_shared(Arc::clone(&sh), MemoryPolicy::default());
        let mut s = Channel::with_shared(sh, MemoryPolicy::default());
        let stamp = 0x1122_3344_5566_7788u64;
        let mut frame = [0u8; 12];
        frame[4..].copy_from_slice(&stamp.to_le_bytes());
        crate::trace::drain();
        assert_eq!(p.publish(&frame), Published::Ok);
        drop(s.take().unwrap());
        assert_eq!(p.publish(&frame[..11]), Published::Ok);
        drop(s.take().unwrap());
        let stamps: Vec<_> = crate::trace::drain()
            .into_iter()
            .filter(|(n, _)| ["commit", "take"].contains(n))
            .map(|(n, a)| (n, *a.last().unwrap()))
            .collect();
        assert_eq!(
            stamps,
            [
                ("commit", stamp),
                ("take", stamp),
                ("commit", 0),
                ("take", 0)
            ]
        );
    }

    #[test]
    fn a_committed_loan_can_keep_its_memory_past_the_channel() {
        let (sh, mut p, s) = pair();
        let mut w = p.loan().unwrap();
        w.data_mut()[0] = 7;
        let at = w.data_mut().as_ptr();
        let keep = w.mapped();
        assert_eq!(w.commit_len(1), Published::Ok);
        drop((sh, p, s));
        // SAFETY: keep holds the memory; Miri reports a use after free otherwise.
        assert_eq!(unsafe { *at }, 7);
        drop(keep);
    }
}

/// A view's storage copied and both copies released, as a C caller can do with `flux_view`.
#[cfg(test)]
mod release_twice {
    use std::sync::Arc;
    use std::sync::atomic::Ordering;

    use super::{Channel, ChannelShared, FrameView, Published};
    use crate::qos::{MemoryPolicy, QoS};
    use crate::reader::ReaderLease;
    use crate::segment::Segment;

    fn pair(slots: u32) -> (Channel, Channel) {
        let seg = Segment::create_heap(8, slots, 0).unwrap();
        let sub = ChannelShared::new(Arc::new(seg.alias()), crate::trace::NO_STAMP);
        let _ = sub.lease.set(Arc::new(ReaderLease::unlocked(1)));
        sub.seg.raise_readers(1);
        let p = ChannelShared::new(Arc::new(seg), crate::trace::NO_STAMP);
        let p = Channel::with_shared(Arc::new(p), MemoryPolicy::default());
        let mut s = Channel::with_shared(Arc::new(sub), MemoryPolicy::default());
        s.set_qos(QoS::new(slots).unwrap()).unwrap();
        (p, s)
    }

    fn copy(v: &FrameView) -> FrameView {
        FrameView {
            sh: Arc::clone(&v.sh),
            slot: v.slot,
            ticket: v.ticket,
            meta: v.meta,
        }
    }

    #[test]
    fn a_count_found_at_zero_is_refused_and_its_slot_held() {
        let (mut p, mut s) = pair(2);
        assert_eq!(p.publish(b"a"), Published::Ok);
        let v = s.take().unwrap();
        let (slot, twin) = (v.slot, copy(&v));
        drop(v);
        drop(twin);
        for _ in 0..4 {
            assert_eq!(p.publish(b"b"), Published::Ok);
            assert_eq!(s.take().map(|v| v.data().to_vec()), Some(b"b".to_vec()));
        }
        assert_eq!(s.sh.reader(slot).load(Ordering::Relaxed), 1);
        assert_eq!(s.sh.seg.slot(slot).commit_ticket.load(Ordering::Relaxed), 1);
        assert_eq!(s.misreleased(), 1);
    }

    #[test]
    fn a_release_of_a_rewritten_slot_leaves_the_new_borrow_standing() {
        let (mut p, mut s) = pair(1);
        assert_eq!(p.publish(b"a"), Published::Ok);
        let v = s.take().unwrap();
        let twin = copy(&v);
        drop(v);
        assert_eq!(p.publish(b"b"), Published::Ok);
        let held = s.take().unwrap();
        drop(twin);
        assert_eq!(p.publish(b"c"), Published::Backpressure);
        assert_eq!(held.data(), b"b");
        assert_eq!(s.misreleased(), 1);
    }
}
