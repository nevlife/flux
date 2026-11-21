//! Publish and take over one segment.
//!
//! Dekker between the publisher (CAS the slot's ticket to claimed, then read every reader counter)
//! and a subscriber (own counter +1, then read the ticket): both pairs are `SeqCst`, so at least
//! one side sees the other. The commit stores the ticket with `Release`, and the subscriber's load
//! after its +1 acquires it.

#![allow(
    clippy::disallowed_types,
    reason = "the slot, reader and wake words; orderings tagged `order:`"
)]
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
use std::ffi::{c_char, c_void};
use std::fmt;
use std::marker::PhantomData;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::arena::{self};
use crate::discovery::{
    SegmentPayload, SignpostMap, open_publisher_segment_in, segment_publisher_dead, sweep_dead_once,
};
use crate::error::{Error, Result};
use crate::executor::Control;
use crate::gpu::{self, FenceStats, FenceWait, Fenced, Route, Seam, Stream, ledger};
use crate::layout::{
    DEFAULT_MAX_SUBSCRIBERS, DType, FrameMeta, MIN_SLOT_COUNT, TICKET_CLAIMED, TICKET_EMPTY,
    reader_lock_byte,
};
use crate::qos::{MemoryPolicy, QoS};
use crate::reader::{self};
use crate::segment::Segment;
use crate::sizing;
use crate::sys::{self};
use crate::trace::{self, event};

pub use loan::{Heap, Loan};
use loan::{fence_failed, fits, meta_from};
pub(crate) use shared::{ChannelShared, Owned, Waiter};
pub use view::{Frame, GpuBlock, GpuHeap};

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

/// Where a publisher's loans put the variable-length fields of a frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum Alloc {
    /// Past the scalar block in the process's arena, the slot holding only what must be there.
    #[default]
    Arena = 0,
    /// All in the slot. A frame that does not fit is refused. The slot's GPU registration covers
    /// every field, so an integrated GPU reads them without a copy.
    Slot = 1,
}

/// Whether a fixed-slot publisher's slots are image slots: on an integrated GPU, each slot a
/// dmabuf of its own that engines such as VIC read in place (`gpu::ImageExport`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum ImageSlots {
    /// Image slots when the publisher declares an image format.
    // shortcut: no rule from the attached consumers yet, add one with the engine subscribers.
    #[default]
    Auto = 0,
    Off = 1,
    On = 2,
}

/// A publisher's settings. A size or count of 0 is `sizing`'s for its `alloc`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PublisherConfig {
    pub alloc: Alloc,
    pub slot_size: u32,
    pub slot_count: u32,
    /// `Alloc::Slot` and GPU memory channels only: the largest slot and the most slots it may
    /// grow to.
    pub max_slot_size: u32,
    pub max_slot_count: u32,
    pub max_subscribers: u32,
    pub aux_size: u32,
    /// `Alloc::Slot` and GPU memory channels only: the slots get memory once a subscriber
    /// attaches, not at creation, and loans fail until then. For a program that makes publishers
    /// for topics nobody may subscribe to.
    pub alloc_on_subscriber: bool,
    /// `On` needs fixed slots on an integrated GPU. An image slot's size is rounded up to 2 MiB.
    pub image_slots: ImageSlots,
    /// Image slots in this format: the slot size defaults to a frame's and must hold one, and
    /// subscribers read the layout (`Frame::image_layout`). Refused with `ImageSlots::Off`.
    pub image_format: crate::image_format::ImageFormat,
}

impl Default for PublisherConfig {
    fn default() -> Self {
        PublisherConfig {
            alloc: Alloc::Arena,
            slot_size: 0,
            slot_count: 0,
            max_slot_size: 0,
            max_slot_count: 0,
            max_subscribers: DEFAULT_MAX_SUBSCRIBERS,
            aux_size: 0,
            alloc_on_subscriber: false,
            image_slots: ImageSlots::Auto,
            image_format: crate::image_format::ImageFormat::default(),
        }
    }
}

fn wants_image(config: &PublisherConfig) -> Result<bool> {
    let format = config.image_format.is_some();
    match config.image_slots {
        ImageSlots::On => Ok(true),
        ImageSlots::Auto => Ok(format),
        ImageSlots::Off if format => Err(Error::InvalidArgument(
            "an image format needs image slots, which image_slots Off turns off".into(),
        )),
        ImageSlots::Off => Ok(false),
    }
}

/// An image segment's payload and slot size, or why `plan` cannot have one.
/// `config`'s slot size: with an image format, a frame's when 0, refused when smaller.
fn image_slot_size(config: &PublisherConfig) -> Result<u32> {
    let f = config.image_format;
    if !f.is_some() {
        return Ok(config.slot_size);
    }
    let bytes = f.layout()?.bytes;
    match config.slot_size {
        0 => Ok(bytes),
        n if n < bytes => Err(Error::InvalidArgument(format!(
            "a slot of {n} bytes does not hold a {:?} frame of {}x{}, {bytes} bytes",
            f.encoding, f.width, f.height
        ))),
        n => Ok(n),
    }
}

fn image_payload(
    plan: &sizing::Plan,
    stream: &Stream,
    format: crate::image_format::ImageFormat,
) -> Result<(SegmentPayload, u32)> {
    if !plan.fixed {
        return Err(Error::InvalidArgument(
            "image slots are fixed slots: use Alloc::Slot or a GPU memory channel".into(),
        ));
    }
    let ordinal = if stream.declared() {
        stream.device()
    } else {
        0
    };
    let p = gpu::probe(ordinal);
    if !p.integrated {
        return Err(Error::Unsupported(format!(
            "image slots need an integrated GPU, and device {ordinal} is not one ({})",
            p.reason
        )));
    }
    let size = (plan.slot_size as usize).next_multiple_of(gpu::IMAGE_SLOT_ALIGN);
    let size = u32::try_from(size).map_err(|_| {
        Error::InvalidArgument(format!("an image slot of {size} bytes is too large"))
    })?;
    Ok((SegmentPayload::Image(ordinal, format), size))
}

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
    /// after the process's GPU is wedged (`gpu::wedged`), and a frame whose device copy the
    /// device had no memory for.
    pub fence: u64,
}

/// The publisher's slots, oldest frame first. Only the publisher writes tickets, so commit order is
/// ticket order, and a claim takes the first free slot here instead of scanning for the oldest.
#[derive(Default)]
struct ClaimOrder {
    queue: VecDeque<u32>,
    /// Claimed for a loan; back in `queue` once committed or aborted.
    loaned: Vec<u32>,
    built: bool,
    /// The slots with memory this order knows; ones `filler` added since go first, unwritten.
    known: u32,
}

impl ClaimOrder {
    fn settle(&mut self, seg: &Segment) {
        let ticket = |s: u32| seg.slot(s).commit_ticket.load(Ordering::Relaxed);
        if !self.built {
            let mut all: Vec<(u64, u32)> = (0..seg.slots_ready())
                .map(|s| (ticket(s), s))
                .filter(|&(t, _)| t != TICKET_CLAIMED)
                .collect();
            all.sort_unstable();
            self.queue = all.into_iter().map(|(_, s)| s).collect();
            self.built = true;
            self.known = seg.slots_ready();
        }
        let ready = seg.slots_ready();
        for s in self.known..ready {
            self.queue.push_front(s);
        }
        self.known = self.known.max(ready);
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
    /// Why the publisher of this signpost epoch refused a re-attach, or the slot of it that would
    /// not lock. Not retried until the epoch moves again.
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
    /// A process-private channel on the heap. A size or count of 0 is `sizing`'s.
    pub fn new(slot_size: u32, slot_count: u32) -> Result<Channel> {
        let slot_size = sizing::slot_size(slot_size, None, false);
        let slot_count = sizing::slot_count(slot_count, slot_size);
        check_slot_count(slot_count)?;
        Ok(Channel::from_segment(
            Segment::create_heap(slot_size, slot_count, 0, 0)?,
            MemoryPolicy::default(),
            trace::NO_STAMP,
        ))
    }

    /// `name` is a signpost name (`signpost_name`). Fails with `PublisherExists` if another
    /// publisher is live on it. `max_subscribers` counts subscribing processes, not
    /// subscriptions; each one adds a counter that every claim and publish reads. Each slot
    /// carries `aux_size` bytes of host memory beside its payload, at most `MAX_AUX_SIZE`.
    pub fn create(
        name: &str,
        slot_size: u32,
        slot_count: u32,
        max_subscribers: u32,
        aux_size: u32,
        fingerprint: u64,
        mem: MemoryPolicy,
    ) -> Result<Channel> {
        Channel::create_on(
            name,
            slot_size,
            slot_count,
            max_subscribers,
            aux_size,
            fingerprint,
            mem,
            Stream::default(),
            trace::NO_STAMP,
            None,
        )
    }

    /// As `create`, with frames produced on `stream`, declared once for the channel's life. On the
    /// device-handle route the payload is a device allocation, unless the frames are a generated
    /// type's (`scalar_bytes`, its scalar block): then the slots stay in host memory and a frame's
    /// device fields go to the GPU arena. A size or count of 0 is `sizing`'s. `stamp_offset` is
    /// where the type's `header.stamp` sits, for trace events (`trace::NO_STAMP` without one).
    #[allow(clippy::too_many_arguments)]
    pub fn create_on(
        name: &str,
        slot_size: u32,
        slot_count: u32,
        max_subscribers: u32,
        aux_size: u32,
        fingerprint: u64,
        mem: MemoryPolicy,
        stream: Stream,
        stamp_offset: i32,
        scalar_bytes: Option<u32>,
    ) -> Result<Channel> {
        let config = PublisherConfig {
            slot_size,
            slot_count,
            max_subscribers,
            aux_size,
            ..PublisherConfig::default()
        };
        Channel::create_config(
            name,
            &config,
            fingerprint,
            mem,
            stream,
            stamp_offset,
            scalar_bytes,
        )
    }

    /// As `create_on`, with every publisher setting in `config`.
    pub fn create_config(
        name: &str,
        config: &PublisherConfig,
        fingerprint: u64,
        mem: MemoryPolicy,
        stream: Stream,
        stamp_offset: i32,
        scalar_bytes: Option<u32>,
    ) -> Result<Channel> {
        let device = if stream.declared() && scalar_bytes.is_none() {
            let route = gpu::require_route(stream.device())?.route;
            (route == Route::DeviceHandle).then_some(stream.device())
        } else {
            None
        };
        let image = wants_image(config)?;
        if config.image_format.is_some() && scalar_bytes.is_some() {
            return Err(Error::InvalidArgument(
                "an image format places the image at the start of the slot, which a generated \
                 type's frame does not: declare it on a byte or GPU memory channel"
                    .into(),
            ));
        }
        let plan = sizing::plan(
            config.alloc,
            image_slot_size(config)?,
            config.slot_count,
            config.max_slot_size,
            config.max_slot_count,
            scalar_bytes,
            stream.declared(),
        )?;
        check_slot_count(plan.slot_count)?;
        let filled = plan.fixed && filler::available();
        let later = filled && config.alloc_on_subscriber;
        let (payload, slot_size) = if image {
            image_payload(&plan, &stream, config.image_format)?
        } else {
            (
                device.map_or(SegmentPayload::Host, SegmentPayload::Device),
                plan.slot_size,
            )
        };
        let seg = open_publisher_segment_in(
            name,
            slot_size,
            plan.slot_count,
            // 0 is the default, as for the other settings.
            if config.max_subscribers == 0 {
                DEFAULT_MAX_SUBSCRIBERS
            } else {
                config.max_subscribers
            },
            config.aux_size,
            fingerprint,
            payload,
            if later { 0 } else { plan.start },
        )?;
        seg.apply(mem)?;
        if seg.host_payload() {
            arena::request(mem)?;
        }
        let fence = Arc::new(FenceStats::default());
        let mut sh = ChannelShared::new(Arc::new(seg), stamp_offset);
        sh.attach_device(&stream, &fence, true)?;
        sh.filled = filled;
        if later {
            sh.fill_start.store(plan.start, Ordering::Relaxed);
        }
        let mut ch = Channel::with_shared(Arc::new(sh).traced(), mem);
        ch.stream = stream;
        ch.fence = fence;
        ch.sh
            .slot_only
            .store(config.alloc == Alloc::Slot, Ordering::Relaxed);
        if filled && !filler::watch(&ch.sh, mem) {
            return Err(Error::Unsupported(
                "the thread that gives fixed slots their memory stopped".into(),
            ));
        }
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
        ch.announce_depth();
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
        } else if !fits(&meta, self.sh.seg.layout()) || data.len() as u64 != meta.nbytes {
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
        if !self.sh.seg.host_payload() {
            return self.publish_to_device(s, data, &meta);
        }
        // SAFETY: the claim makes this the only access to the slot, and nbytes <= slot_size.
        unsafe { std::ptr::copy_nonoverlapping(data.as_ptr(), self.sh.seg.payload(s), data.len()) };
        self.sh.finish_commit(s, &meta);
        self.order.queue.push_back(s);
        Published::Ok
    }

    /// Copies host `data` into the claimed device slot `s` on the channel's stream, which the
    /// commit then waits for. A failed copy or fence keeps the claim, as a loan's commit does.
    fn publish_to_device(&mut self, s: u32, data: &[u8], meta: &FrameMeta) -> Published {
        let dst = self.sh.device_ptr(s);
        let copied = self
            .sh
            .gpu
            .as_ref()
            .is_some_and(|g| g.stream.copy_to_device(dst, data.as_ptr(), data.len()));
        let fenced = if copied {
            self.sh.fence(Seam::Commit)
        } else {
            Fenced::Failed
        };
        match fenced {
            Fenced::Done => {
                self.sh.finish_commit(s, meta);
                self.order.queue.push_back(s);
                return Published::Ok;
            }
            Fenced::Failed => std::mem::forget(Arc::clone(&self.sh)),
            Fenced::Late(late) => self.sh.defer(late, s, 0, Seam::Commit, false),
        }
        self.order.loaned.push(s);
        fence_failed(trace::CLAIM_FENCE_FAILED, trace::CLAIM_GPU_WEDGED).0
    }

    /// The whole slot as one u8 run; commit the prefix written with `commit_len`.
    pub fn loan(&mut self) -> Option<Loan> {
        self.loan_typed(DType::U8, &[u64::from(self.slot_size())])
    }

    /// The shape is checked at commit, the one place the caller already reads an outcome.
    pub fn loan_typed(&mut self, dt: DType, shape: &[u64]) -> Option<Loan> {
        self.try_loan_typed(dt, shape).ok()
    }

    /// As `loan_typed`, with why there is no loan: `NotPublisher`, `GpuWedged` or `Backpressure`.
    pub fn try_loan_typed(
        &mut self,
        dt: DType,
        shape: &[u64],
    ) -> std::result::Result<Loan, Published> {
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
            return Err(if refused == trace::CLAIM_GPU_WEDGED {
                Published::GpuWedged
            } else {
                Published::NotPublisher
            });
        }
        let slot = self.claim(1).ok_or(Published::Backpressure)?;
        self.order.loaned.push(slot);
        event!(flux_trace_loan(self.trace_handle(), slot));
        Ok(Loan {
            sh: Arc::clone(&self.sh),
            slot,
            meta: meta_from(dt, shape),
            done: false,
        })
    }

    /// The oldest free frame goes first, so the newest are overwritten last. A slot is added
    /// instead while that frame is within a subscriber's depth, or when none is free. `loan` is
    /// for the trace event.
    fn claim(&mut self, loan: i32) -> Option<u32> {
        self.sh.reap_late();
        self.order.settle(&self.sh.seg);
        // A fixed-slot channel asks `filler` once per claim, here or below.
        let asked = self.sh.filled && self.oldest_within_depth();
        if asked {
            filler::ask(&self.sh);
        } else if self.oldest_within_depth()
            && let Some(slot) = self.add_slot(loan)
        {
            return Some(slot);
        }
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
        if !asked && let Some(slot) = self.add_slot(loan) {
            return Some(slot);
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

    /// Whether the oldest frame is one a subscriber's depth still reaches: the newest
    /// `depth_high` frames.
    fn oldest_within_depth(&self) -> bool {
        let seg = &self.sh.seg;
        let Some(&s) = self.order.queue.front() else {
            return false;
        };
        let t = seg.slot(s).commit_ticket.load(Ordering::Relaxed);
        // order: depth_high
        let depth = seg.ctrl().depth_high.load(Ordering::Relaxed);
        let latest = seg.ctrl().latest.load(Ordering::Relaxed); // order: latest
        t != TICKET_EMPTY && t != TICKET_CLAIMED && latest.saturating_sub(t) < u64::from(depth)
    }

    /// A slot given its memory just now, claimed. None when the segment cannot add one, and for a
    /// fixed-slot channel, whose claim asks `filler` instead and takes the slot once it is there.
    fn add_slot(&mut self, loan: i32) -> Option<u32> {
        if self.sh.filled {
            filler::ask(&self.sh);
            return None;
        }
        let slot = self.sh.seg.add_slot(self.mem)?;
        self.order.known = slot + 1;
        // Nothing reads a slot before its first ticket, so this cannot fail.
        if !self.sh.try_claim(slot) {
            return None;
        }
        event!(flux_trace_claim(
            self.trace_handle(),
            slot,
            trace::CLAIM_OK,
            loan
        ));
        Some(slot)
    }

    /// The oldest frame newer than the cursor, within `depth` of the newest.
    pub fn take(&mut self) -> Option<Frame> {
        self.sh.reap_late();
        if !self.begin_borrow() {
            return None;
        }
        let mut attempt = 0;
        while attempt < MAX_RETRIES {
            attempt += 1;
            let latest = self.sh.seg.ctrl().latest.load(Ordering::Acquire); // order: latest
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
            counter.fetch_add(1, Ordering::SeqCst); // order: readers
            // order: ticket
            if sl.commit_ticket.load(Ordering::SeqCst) != t {
                counter.fetch_sub(1, Ordering::Release); // order: readers
                continue;
            }
            // SAFETY: borrowed with the ticket unchanged, so the publisher is off this slot.
            let meta = unsafe { *sl.meta.get() };
            if let Err(e) = self.sh.seg.cover_slot(s, self.mem) {
                counter.fetch_sub(1, Ordering::Release); // order: readers
                self.attach_error = Some((self.signpost_epoch, e));
                break;
            }
            if t > self.cursor + 1 {
                self.lost = self.lost.saturating_add(t - self.cursor - 1);
            }
            self.cursor = t;
            if !fits(&meta, self.sh.seg.layout()) {
                counter.fetch_sub(1, Ordering::Release); // order: readers
                self.refused.bad_frame += 1;
                event!(flux_trace_refused(
                    self.sh.trace_handle(),
                    trace::REFUSED_BAD_FRAME
                ));
                continue;
            }
            if !self.sh.reach(s) {
                counter.fetch_sub(1, Ordering::Release); // order: readers
                self.refused.fence += 1;
                event!(flux_trace_refused(
                    self.sh.trace_handle(),
                    trace::REFUSED_FENCE
                ));
                break;
            }
            if self.sh.gpu.is_some() {
                ledger::borrowed(self.sh.trace_handle(), s);
            }
            #[allow(clippy::cast_possible_truncation)]
            if !self.sh.stage(s, meta.nbytes as usize) {
                // The copy may still read the slot: the borrow stays, as a failed fence's does.
                ledger::kept(self.sh.trace_handle(), s);
                self.sh.leases_leaked.fetch_add(1, Ordering::Release); // order: leases_leaked
                std::mem::forget(Arc::clone(&self.sh));
                self.refused.fence += 1;
                event!(flux_trace_refused(
                    self.sh.trace_handle(),
                    trace::REFUSED_FENCE
                ));
                return None;
            }
            self.stalled_polls = 0;
            self.last_seen_ticket = t;
            self.taken = true;
            event!(flux_trace_take(
                self.sh.trace_handle(),
                t,
                s,
                self.lost,
                self.sh.stamp(s, meta.nbytes),
            ));
            return Some(Frame {
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
        self.sh.outstanding.fetch_sub(1, Ordering::Release); // order: outstanding
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
    pub fn take_blocking(&mut self, timeout_ns: i64) -> Option<Frame> {
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
        if self.attach_error.is_some() {
            return false;
        }
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
        // order: outstanding
        if self.sh.outstanding.fetch_add(1, Ordering::AcqRel) >= self.qos.borrow_limit() {
            self.sh.outstanding.fetch_sub(1, Ordering::Release); // order: outstanding
            // Leaked borrows are never given back, so no release the caller makes can help.
            // order: leases_leaked
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
    /// So the publisher keeps this consumer's depth of frames, adding slots for them (`claim`).
    fn announce_depth(&self) {
        self.sh
            .seg
            .ctrl()
            .depth_high
            .fetch_max(self.qos.depth(), Ordering::Relaxed); // order: depth_high
    }

    fn position_cursor(&mut self) {
        let newest = self.sh.seg.ctrl().latest.load(Ordering::Acquire); // order: latest
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
        // The executor serving the old channel serves this one: a release deferred before its next
        // pass, the one that would tell this channel, pokes it all the same.
        let waker = self.sh.gpu_waker.lock().clone();
        *sh.gpu_waker.lock() = waker;
        sh.gpu_waker_of.store(
            self.sh.gpu_waker_of.load(Ordering::Relaxed),
            Ordering::Relaxed,
        );
        self.sh.orphan_late();
        sys::retire(std::mem::replace(&mut self.sh, sh));
        self.signpost_epoch = epoch;
        self.cursor = 0;
        self.orphaned = false;
        self.attach_error = None;
        self.attach_gen = next_attach_generation();
        self.announce_depth();
        true
    }

    /// Advisory: a lapped or refused frame makes the next take empty.
    pub fn ready(&self) -> bool {
        self.sh.seg.ctrl().latest.load(Ordering::Acquire) > self.cursor // order: latest
    }

    /// A take refused for this cannot be unblocked by a publish. Deferred releases whose work is
    /// done are given back first.
    pub fn can_borrow(&self) -> bool {
        self.sh.reap_late();
        self.sh.outstanding.load(Ordering::SeqCst) < self.qos.borrow_limit() // order: outstanding
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
        self.announce_depth();
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
    /// Subscriber processes on this publisher's channel now: the numbers whose lock another
    /// description holds, so a dead process is already out. One `F_OFD_GETLK` per number ever
    /// taken. A subscription in the publisher's own process counts from its first wait or take.
    /// A heap channel counts 1 once read from; a subscriber's channel counts 0.
    pub fn subscriber_count(&self) -> u32 {
        let seg = &self.sh.seg;
        #[allow(clippy::cast_possible_truncation)]
        let numbers = seg.readers_in_use(Ordering::Acquire) as u32; // order: readers_high
        if seg.is_heap() {
            return numbers;
        }
        let Some(fd) = seg.publisher_fd() else {
            return 0;
        };
        #[allow(clippy::cast_possible_truncation)]
        let held = (0..numbers)
            .filter(|&i| sys::lock_held(fd, reader_lock_byte(i)) == libc::F_WRLCK)
            .count() as u32;
        held
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
    /// Why this consumer cannot read the publisher's frames, until it restarts: the restarted
    /// publisher refused it, or a slot the publisher added would not commit or lock as the
    /// consumer's `MemoryPolicy` asks. Takes find nothing meanwhile.
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
    #[cfg(test)]
    pub(crate) fn device_copy_backed(&self) -> usize {
        self.sh.device_copy_backed()
    }
    #[cfg(test)]
    pub(crate) fn registered_slots(&self) -> usize {
        self.sh
            .seg
            .registrations()
            .map_or(0, crate::gpu::SlotRegistrations::registered)
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
    /// Slots with payload memory now, at most `slot_count`: a publisher left at count 0 adds them.
    pub fn slots_ready(&self) -> u32 {
        self.sh.seg.slots_ready()
    }
    pub fn aux_size(&self) -> u32 {
        self.sh.seg.layout().aux_size
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

    pub fn alloc(&self) -> Alloc {
        if self.sh.slot_only() {
            Alloc::Slot
        } else {
            Alloc::Arena
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
    sh.attach_device(stream, fence, false)?;
    let _ = sh.lease.set(lease);
    Ok((Arc::new(sh).traced(), epoch))
}

mod filler;
#[cfg(test)]
pub(crate) use filler::{loan_soon, soon};
#[cfg(test)]
mod grow;
#[cfg(test)]
mod header;
mod loan;
#[cfg(test)]
mod race;
#[cfg(test)]
mod release_twice;
mod shared;
mod view;
