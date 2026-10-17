use std::cell::UnsafeCell;
use std::collections::BTreeMap;
use std::ffi::{c_char, c_void};
use std::mem::{align_of, offset_of, size_of};
use std::ops::{Deref, DerefMut};
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use flux2_core::{
    Channel, DType, Error, FrameMeta, FrameView, Mapped, MemoryPolicy, Published, PublisherWatch,
    QoS, ReleaseMode, WriteSlot, signpost_name,
};

use crate::gpu::{flux2_stream, stream_of};
use crate::status::{copy_out, flux2_status, in_use, status, status_of, text};
use crate::store::{Stored, stored, stored_ref};

pub struct flux2_channel {
    ch: UnsafeCell<Channel>,
    /// Set while a call holds the channel, so a second thread is refused instead of let in.
    busy: AtomicBool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct flux2_memory_policy {
    pub precommit: bool,
    /// Implies precommit.
    pub lock: bool,
}

impl From<flux2_memory_policy> for MemoryPolicy {
    fn from(m: flux2_memory_policy) -> Self {
        MemoryPolicy {
            precommit: m.precommit,
            lock: m.lock,
        }
    }
}

/// `replay` 0 is volatile, N is `transient_local(N)`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct flux2_qos {
    pub depth: u32,
    pub replay: u32,
    pub max_borrow: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct flux2_refused {
    pub max_borrow: u64,
    pub no_reader: u64,
    pub contended: u64,
    pub bad_frame: u64,
    pub fence: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct flux2_fence_wait {
    pub commit_ns: u64,
    pub commit_waits: u64,
    pub commit_max_ns: u64,
    pub release_ns: u64,
    pub release_waits: u64,
    pub release_max_ns: u64,
}

/// Same bytes as the frame header in shared memory; `dtype` is a `FLUX2_DTYPE_*`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct flux2_frame_meta {
    pub ndim: u8,
    pub dtype: u8,
    pub reserved0: [u8; 2],
    pub itemsize: u32,
    pub nbytes: u64,
    pub shape: [u64; crate::FLUX2_MAX_DIMS],
}

pub const FLUX2_DTYPE_U8: u8 = 0;
pub const FLUX2_DTYPE_I8: u8 = 1;
pub const FLUX2_DTYPE_U16: u8 = 2;
pub const FLUX2_DTYPE_I16: u8 = 3;
pub const FLUX2_DTYPE_U32: u8 = 4;
pub const FLUX2_DTYPE_I32: u8 = 5;
pub const FLUX2_DTYPE_U64: u8 = 6;
pub const FLUX2_DTYPE_I64: u8 = 7;
pub const FLUX2_DTYPE_F16: u8 = 8;
pub const FLUX2_DTYPE_F32: u8 = 9;
pub const FLUX2_DTYPE_F64: u8 = 10;
pub const FLUX2_DTYPE_BF16: u8 = 11;

const _: () = {
    assert!(FLUX2_DTYPE_U8 == DType::U8 as u8);
    assert!(FLUX2_DTYPE_I8 == DType::I8 as u8);
    assert!(FLUX2_DTYPE_U16 == DType::U16 as u8);
    assert!(FLUX2_DTYPE_I16 == DType::I16 as u8);
    assert!(FLUX2_DTYPE_U32 == DType::U32 as u8);
    assert!(FLUX2_DTYPE_I32 == DType::I32 as u8);
    assert!(FLUX2_DTYPE_U64 == DType::U64 as u8);
    assert!(FLUX2_DTYPE_I64 == DType::I64 as u8);
    assert!(FLUX2_DTYPE_F16 == DType::F16 as u8);
    assert!(FLUX2_DTYPE_F32 == DType::F32 as u8);
    assert!(FLUX2_DTYPE_F64 == DType::F64 as u8);
    assert!(FLUX2_DTYPE_BF16 == DType::BF16 as u8);
};

#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum flux2_published {
    FLUX2_PUBLISHED_OK = 0,
    FLUX2_PUBLISHED_BACKPRESSURE = 1,
    FLUX2_PUBLISHED_TOO_LARGE = 2,
    FLUX2_PUBLISHED_NOT_PUBLISHER = 3,
    /// An unknown dtype, or a commit of an empty loan (already committed or aborted).
    FLUX2_PUBLISHED_INVALID_ARGUMENT = 4,
    FLUX2_PUBLISHED_IN_USE = 5,
    /// The payload is device memory; loan and write through `flux2_loan_device_ptr`.
    FLUX2_PUBLISHED_WRONG_DEVICE = 6,
    /// The stream could not be waited for. The slot stays claimed and is never reused.
    FLUX2_PUBLISHED_FENCE_FAILED = 7,
    FLUX2_PUBLISHED_GPU_WEDGED = 8,
}

impl From<Published> for flux2_published {
    fn from(p: Published) -> Self {
        match p {
            Published::Ok => flux2_published::FLUX2_PUBLISHED_OK,
            Published::Backpressure => flux2_published::FLUX2_PUBLISHED_BACKPRESSURE,
            Published::TooLarge => flux2_published::FLUX2_PUBLISHED_TOO_LARGE,
            Published::NotPublisher => flux2_published::FLUX2_PUBLISHED_NOT_PUBLISHER,
            Published::WrongDevice => flux2_published::FLUX2_PUBLISHED_WRONG_DEVICE,
            Published::FenceFailed => flux2_published::FLUX2_PUBLISHED_FENCE_FAILED,
            Published::GpuWedged => flux2_published::FLUX2_PUBLISHED_GPU_WEDGED,
        }
    }
}

/// A borrowed frame in caller memory, never copied. A move copies the bytes and zeroes the source.
#[repr(C)]
pub struct flux2_view {
    pub opaque: [u64; 16],
}

/// A claimed slot in caller memory, never copied. Commit it while no call is on its channel.
#[repr(C)]
pub struct flux2_loan {
    pub opaque: [u64; 16],
}

const _: () = {
    assert!(size_of::<Stored<FrameView>>() <= size_of::<flux2_view>());
    assert!(align_of::<Stored<FrameView>>() <= align_of::<flux2_view>());
    assert!(size_of::<LoanCell>() <= size_of::<flux2_loan>());
    assert!(align_of::<LoanCell>() <= align_of::<flux2_loan>());
    assert!(size_of::<flux2_frame_meta>() == size_of::<FrameMeta>());
    assert!(align_of::<flux2_frame_meta>() == align_of::<FrameMeta>());
    assert!(offset_of!(flux2_frame_meta, itemsize) == offset_of!(FrameMeta, itemsize));
    assert!(offset_of!(flux2_frame_meta, nbytes) == offset_of!(FrameMeta, nbytes));
    assert!(offset_of!(flux2_frame_meta, shape) == offset_of!(FrameMeta, shape));
};

struct Held<'a> {
    ch: &'a mut Channel,
    busy: &'a AtomicBool,
}

impl Deref for Held<'_> {
    type Target = Channel;
    fn deref(&self) -> &Channel {
        self.ch
    }
}

impl DerefMut for Held<'_> {
    fn deref_mut(&mut self) -> &mut Channel {
        self.ch
    }
}

impl Drop for Held<'_> {
    fn drop(&mut self) {
        self.busy.store(false, Ordering::Release);
    }
}

/// None while another thread is in a call on the same handle.
unsafe fn hold<'a>(p: *mut flux2_channel) -> Option<Held<'a>> {
    // SAFETY: the caller passes a live handle.
    let h = unsafe { &*p };
    if h.busy.swap(true, Ordering::Acquire) {
        return None;
    }
    Some(Held {
        // SAFETY: busy was clear, so no other call holds the channel.
        ch: unsafe { &mut *h.ch.get() },
        busy: &h.busy,
    })
}

pub(crate) unsafe fn chan_ref<'a>(p: *const flux2_channel) -> &'a Channel {
    // SAFETY: the caller passes a live handle.
    let h = unsafe { &*p };
    // SAFETY: readers race a holder only on the counters.
    unsafe { &*h.ch.get() }
}

pub(crate) unsafe fn watch_ref<'a>(p: *const flux2_watch) -> &'a PublisherWatch {
    // SAFETY: the caller passes a live handle.
    unsafe { &(*p).w }
}

unsafe fn view<'a>(p: *mut flux2_view) -> &'a mut Stored<FrameView> {
    // SAFETY: flux2_view is sized and aligned for Stored<FrameView> (asserted above).
    unsafe { stored(p) }
}

unsafe fn view_ref<'a>(p: *const flux2_view) -> &'a Stored<FrameView> {
    // SAFETY: as in view.
    unsafe { stored_ref(p) }
}

/// After the commit it keeps the memory the loan wrote mapped, for arrays a binding handed out
/// over it, until the storage is aborted or loaned again.
#[repr(C)]
struct LoanCell {
    slot: Stored<WriteSlot>,
    sent: Stored<Mapped>,
    /// Taken once per loan: a fresh `&mut` per `flux2_loan_data` would invalidate the pointer an
    /// earlier call handed out.
    payload: *mut u8,
}

impl LoanCell {
    fn put(&mut self, mut w: Option<WriteSlot>) {
        self.sent.put(None);
        self.payload = w
            .as_mut()
            .filter(|w| w.host_addressable())
            .map_or(std::ptr::null_mut(), |w| w.data_mut().as_mut_ptr());
        self.slot.put(w);
    }

    fn commit(&mut self, f: impl FnOnce(WriteSlot) -> Published) -> flux2_published {
        let Some(w) = self.slot.take() else {
            return flux2_published::FLUX2_PUBLISHED_INVALID_ARGUMENT;
        };
        self.sent.put(Some(w.mapped()));
        f(w).into()
    }
}

unsafe fn loan<'a>(p: *mut flux2_loan) -> &'a mut LoanCell {
    // SAFETY: flux2_loan is sized and aligned for LoanCell (asserted above), and zeroed is empty.
    unsafe { &mut *p.cast::<LoanCell>() }
}

unsafe fn shape<'a>(shape: *const u64, ndim: usize) -> &'a [u64] {
    if ndim == 0 {
        return &[];
    }
    // SAFETY: the caller passes ndim readable u64s.
    unsafe { std::slice::from_raw_parts(shape, ndim) }
}

unsafe fn bytes<'a>(data: *const u8, len: usize) -> &'a [u8] {
    if len == 0 {
        return &[];
    }
    // SAFETY: the caller passes len readable bytes.
    unsafe { std::slice::from_raw_parts(data, len) }
}

fn boxed(ch: Channel) -> *mut flux2_channel {
    Box::into_raw(Box::new(flux2_channel {
        ch: UnsafeCell::new(ch),
        busy: AtomicBool::new(false),
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_new(
    slot_size: u32,
    slot_count: u32,
    out: *mut *mut flux2_channel,
) -> flux2_status {
    status(Channel::new(slot_size, slot_count).map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

/// The publisher of signpost `name`, for up to `max_subscribers` subscribing processes.
// `stamp_offset` is where the type's `header.stamp` sits in a frame, for trace events; -1 without
// one (`FLUX2_NO_STAMP`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_create(
    name: *const c_char,
    slot_size: u32,
    slot_count: u32,
    max_subscribers: u32,
    fingerprint: u64,
    mem: flux2_memory_policy,
    stream: *const flux2_stream,
    stamp_offset: i32,
    out: *mut *mut flux2_channel,
) -> flux2_status {
    // SAFETY: stream is NULL or a live handle.
    let stream = unsafe { stream_of(stream) };
    // SAFETY: name is a valid string.
    let r = unsafe { text(name) }.and_then(|n| {
        Channel::create_on(
            n,
            slot_size,
            slot_count,
            max_subscribers,
            fingerprint,
            mem.into(),
            stream,
            stamp_offset,
        )
    });
    status(r.map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

// A subscription's hold on a signpost while it has no publisher; open creates the signpost.
pub struct flux2_watch {
    w: PublisherWatch,
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_watch_open(
    name: *const c_char,
    out: *mut *mut flux2_watch,
) -> flux2_status {
    // SAFETY: name is a valid string.
    let r = unsafe { text(name) }.and_then(PublisherWatch::open);
    status(r.map(|w| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(Box::into_raw(Box::new(flux2_watch { w }))) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_watch_free(w: *mut flux2_watch) {
    if !w.is_null() {
        // SAFETY: a handle from flux2_watch_open, freed once.
        drop(unsafe { Box::from_raw(w) });
    }
}

// 0 until the first publisher; it changes with every restart.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_watch_epoch(w: *const flux2_watch) -> u32 {
    // SAFETY: valid handle.
    unsafe { &(*w).w }.epoch()
}

// Notes the epoch for flux2_watch_wait and an executor to wait on; open_watched arms it too.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_watch_arm(w: *const flux2_watch) -> u32 {
    // SAFETY: valid handle.
    unsafe { &(*w).w }.arm()
}

// Until the epoch moves from the armed one, or a signal or stray wake: true asks for a look.
// False only on a timeout.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_watch_wait(w: *const flux2_watch, timeout_ns: i64) -> bool {
    // SAFETY: valid handle.
    unsafe { &(*w).w }.wait(timeout_ns)
}

/// After a first success through `w`, starts at its publisher's oldest frame. `w` is not kept.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_open_watched(
    w: *const flux2_watch,
    fingerprint: u64,
    q: flux2_qos,
    mem: flux2_memory_policy,
    stream: *const flux2_stream,
    stamp_offset: i32,
    out: *mut *mut flux2_channel,
) -> flux2_status {
    // SAFETY: stream is NULL or a live handle.
    let stream = unsafe { stream_of(stream) };
    let r = qos_of(q).and_then(|qos| {
        Channel::open_watched_on(
            // SAFETY: valid handle.
            unsafe { &(*w).w },
            fingerprint,
            qos,
            mem.into(),
            stream,
            stamp_offset,
        )
    });
    status(r.map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

fn qos_to_c(q: QoS) -> flux2_qos {
    flux2_qos {
        depth: q.depth(),
        replay: q.durability().replay,
        max_borrow: q.borrow_limit(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn flux2_qos_default() -> flux2_qos {
    qos_to_c(QoS::default())
}

fn qos_of(q: flux2_qos) -> flux2_core::Result<QoS> {
    QoS::new(q.depth)
        .and_then(|x| x.max_borrow(q.max_borrow))
        .map(|x| x.transient_local(q.replay))
}

/// A subscriber of signpost `name`. `FLUX2_NOT_READY` until a publisher is up.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_open(
    name: *const c_char,
    fingerprint: u64,
    mem: flux2_memory_policy,
    stream: *const flux2_stream,
    stamp_offset: i32,
    out: *mut *mut flux2_channel,
) -> flux2_status {
    // SAFETY: stream is NULL or a live handle.
    let stream = unsafe { stream_of(stream) };
    // SAFETY: name is a valid string.
    let r = unsafe { text(name) }
        .and_then(|n| Channel::open_on(n, fingerprint, mem.into(), stream, stamp_offset));
    status(r.map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_free(ch: *mut flux2_channel) {
    if !ch.is_null() {
        // SAFETY: a handle from one of the constructors, freed once.
        drop(unsafe { Box::from_raw(ch) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_publish(
    ch: *mut flux2_channel,
    data: *const u8,
    len: usize,
) -> flux2_published {
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return flux2_published::FLUX2_PUBLISHED_IN_USE;
    };
    // SAFETY: len readable bytes.
    let data = unsafe { bytes(data, len) };
    ch.publish(data).into()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_publish_typed(
    ch: *mut flux2_channel,
    data: *const u8,
    len: usize,
    dtype: u8,
    shape_: *const u64,
    ndim: usize,
) -> flux2_published {
    let Some(dt) = DType::from_raw(dtype) else {
        return flux2_published::FLUX2_PUBLISHED_INVALID_ARGUMENT;
    };
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return flux2_published::FLUX2_PUBLISHED_IN_USE;
    };
    // SAFETY: len readable bytes.
    let data = unsafe { bytes(data, len) };
    // SAFETY: ndim readable dims.
    let sh = unsafe { shape(shape_, ndim) };
    ch.publish_typed(data, dt, sh).into()
}

/// The whole slot as one u8 run. False when no slot is free, this is a subscriber, or in use.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_loan(ch: *mut flux2_channel, out: *mut flux2_loan) -> bool {
    // SAFETY: valid loan storage.
    let slot = unsafe { loan(out) };
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        slot.put(None);
        return false;
    };
    slot.put(ch.loan());
    slot.slot.get().is_some()
}

/// The shape is checked at commit. False also for an unknown dtype.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_loan_typed(
    ch: *mut flux2_channel,
    dtype: u8,
    shape_: *const u64,
    ndim: usize,
    out: *mut flux2_loan,
) -> bool {
    // SAFETY: valid loan storage.
    let slot = unsafe { loan(out) };
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        slot.put(None);
        return false;
    };
    // SAFETY: ndim readable dims.
    let sh = unsafe { shape(shape_, ndim) };
    slot.put(DType::from_raw(dtype).and_then(|dt| ch.loan_typed(dt, sh)));
    slot.slot.get().is_some()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_loan_data(l: *mut flux2_loan, capacity: *mut usize) -> *mut u8 {
    // SAFETY: valid loan storage.
    let cell = unsafe { loan(l) };
    let Some(w) = cell.slot.get() else {
        return std::ptr::null_mut();
    };
    // SAFETY: capacity is valid for a write.
    unsafe { capacity.write(w.capacity()) };
    cell.payload
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_loan_stream(l: *const flux2_loan) -> *mut c_void {
    // SAFETY: flux2_loan is sized and aligned for LoanCell (asserted above).
    unsafe { &*l.cast::<LoanCell>() }
        .slot
        .get()
        .map_or(std::ptr::null_mut(), |w| w.stream().native())
}

/// 0 when the channel declared no stream, as for `flux2_view_device_ptr`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_loan_device_ptr(l: *const flux2_loan) -> u64 {
    // SAFETY: flux2_loan is sized and aligned for LoanCell (asserted above).
    unsafe { &*l.cast::<LoanCell>() }
        .slot
        .get()
        .map_or(0, WriteSlot::device_ptr)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_loan_commit(l: *mut flux2_loan) -> flux2_published {
    // SAFETY: valid loan storage.
    unsafe { loan(l) }.commit(WriteSlot::commit)
}

/// Shrinks a one-dimensional loan to `nbytes`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_loan_commit_len(
    l: *mut flux2_loan,
    nbytes: usize,
) -> flux2_published {
    // SAFETY: valid loan storage.
    unsafe { loan(l) }.commit(|w| w.commit_len(nbytes))
}

/// Ends the loan (its slot's old frame is gone either way) or the mapping a commit kept.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_loan_abort(l: *mut flux2_loan) {
    // SAFETY: valid loan storage.
    unsafe { loan(l) }.put(None);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_loan_valid(l: *const flux2_loan) -> bool {
    // SAFETY: flux2_loan is sized and aligned for LoanCell (asserted above).
    unsafe { &*l.cast::<LoanCell>() }.slot.get().is_some()
}

/// The oldest frame newer than the cursor. A view already in `out` is released first.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_take(ch: *mut flux2_channel, out: *mut flux2_view) -> bool {
    // SAFETY: valid view storage.
    let slot = unsafe { view(out) };
    slot.put(None);
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return false;
    };
    slot.put(ch.take());
    slot.get().is_some()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_take_blocking(
    ch: *mut flux2_channel,
    timeout_ns: i64,
    out: *mut flux2_view,
) -> bool {
    // SAFETY: valid view storage.
    let slot = unsafe { view(out) };
    slot.put(None);
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return false;
    };
    slot.put(ch.take_blocking(timeout_ns));
    slot.get().is_some()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_data(v: *const flux2_view, len: *mut usize) -> *const u8 {
    // SAFETY: valid view storage.
    let Some(fv) = (unsafe { view_ref(v) }).get() else {
        return std::ptr::null();
    };
    if !fv.host_addressable() {
        return std::ptr::null();
    }
    // SAFETY: len is valid for a write.
    unsafe { len.write(fv.data().len()) };
    fv.data().as_ptr()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_stream(v: *const flux2_view) -> *mut c_void {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(std::ptr::null_mut(), |fv| fv.stream().native())
}

/// The frame's device address, 0 when the channel declared no stream.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_device_ptr(v: *const flux2_view) -> u64 {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(0, FrameView::device_ptr)
}

/// Points into `v`, so moving the view moves it. Size a typed read from `dtype`, not `itemsize`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_meta(v: *const flux2_view) -> *const flux2_frame_meta {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }.get().map_or(std::ptr::null(), |fv| {
        std::ptr::from_ref(fv.meta()).cast::<flux2_frame_meta>()
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_ticket(v: *const flux2_view) -> u64 {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }.get().map_or(0, FrameView::ticket)
}

// The channel as trace events name it (`flux2/trace.h`), NULL for an empty view.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_trace_channel(v: *const flux2_view) -> *const c_void {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(std::ptr::null(), FrameView::trace_channel)
}

/// `CLOCK_MONOTONIC` at commit.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_publish_ns(v: *const flux2_view) -> u64 {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(0, FrameView::publish_ns)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_release(v: *mut flux2_view) {
    // SAFETY: valid view storage.
    unsafe { view(v) }.put(None);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_valid(v: *const flux2_view) -> bool {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }.get().is_some()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_set_qos(
    ch: *mut flux2_channel,
    q: flux2_qos,
) -> flux2_status {
    let qos = qos_of(q);
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return in_use();
    };
    status(qos.and_then(|x| ch.set_qos(x)))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_qos(ch: *const flux2_channel) -> flux2_qos {
    // SAFETY: valid handle.
    qos_to_c(*unsafe { chan_ref(ch) }.qos())
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_refused(ch: *const flux2_channel) -> flux2_refused {
    // SAFETY: valid handle.
    let r = unsafe { chan_ref(ch) }.refused();
    flux2_refused {
        max_borrow: r.max_borrow,
        no_reader: r.no_reader,
        contended: r.contended,
        bad_frame: r.bad_frame,
        fence: r.fence,
    }
}

/// Releases refused because their borrow was already given back: a `flux2_view` released twice.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_misreleased(ch: *const flux2_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.misreleased()
}

/// Commits and releases that could not wait for the stream; each holds its slot (see `flux2_channel_set_fence_timeout`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_fence_failed(ch: *const flux2_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.fence_failed()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_fence_wait(ch: *const flux2_channel) -> flux2_fence_wait {
    // SAFETY: valid handle.
    let w = unsafe { chan_ref(ch) }.fence_wait();
    flux2_fence_wait {
        commit_ns: w.commit_ns,
        commit_waits: w.commit_waits,
        commit_max_ns: w.commit_max_ns,
        release_ns: w.release_ns,
        release_waits: w.release_waits,
        release_max_ns: w.release_max_ns,
    }
}

/// 0 = no bound (default). One that runs out is a failed fence until its work is done.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_set_fence_timeout(ch: *const flux2_channel, ns: u64) {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.set_fence_timeout(ns);
}

pub const FLUX2_RELEASE_WAIT: i32 = 0;
pub const FLUX2_RELEASE_DEFERRED: i32 = 1;

/// `FLUX2_RELEASE_DEFERRED`: a release returns at once; its slot goes back once the device is done.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_set_release_mode(
    ch: *const flux2_channel,
    mode: i32,
) -> flux2_status {
    let mode = match mode {
        FLUX2_RELEASE_WAIT => ReleaseMode::Wait,
        FLUX2_RELEASE_DEFERRED => ReleaseMode::Deferred,
        _ => {
            return status(Err(Error::InvalidArgument(format!(
                "release mode {mode} is neither FLUX2_RELEASE_WAIT nor FLUX2_RELEASE_DEFERRED"
            ))));
        }
    };
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.set_release_mode(mode);
    flux2_status::FLUX2_OK
}

/// Orders the view's channel stream after what `s` queued so far, on the device, not the host.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_view_after(v: *const flux2_view, s: *const flux2_stream) -> bool {
    // SAFETY: s is null or a live stream.
    let other = unsafe { stream_of(s) };
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .is_some_and(|fv| fv.stream().after(&other))
}

/// The payload is device memory: views and loans have no host bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_device_backed(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.device_backed()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_stream(ch: *const flux2_channel) -> *mut c_void {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.stream().native()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_ready(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.ready()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_can_borrow(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.can_borrow()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_lost(ch: *const flux2_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.lost()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_dropped(ch: *const flux2_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.dropped()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_cursor(ch: *const flux2_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.cursor()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_attach_generation(ch: *const flux2_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.attach_generation()
}

/// Why a restarted publisher refused this subscriber; takes find nothing until it restarts again.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_attach_status(ch: *mut flux2_channel) -> flux2_status {
    // SAFETY: valid handle.
    let Some(ch) = (unsafe { hold(ch) }) else {
        return in_use();
    };
    ch.attach_error().map_or(flux2_status::FLUX2_OK, status_of)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_orphaned(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.orphaned()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_inherited(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.inherited()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_wake_seq(ch: *const flux2_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.wake_seq()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_slot_size(ch: *const flux2_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.slot_size()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_slot_count(ch: *const flux2_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.slot_count()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_payload_readonly(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.payload_readonly()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_pages_committed(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.pages_committed()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_pages_locked(ch: *const flux2_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.pages_locked()
}

/// Sample `wake_seq` before the take that finds nothing. False only on a timeout.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_channel_wait(
    ch: *const flux2_channel,
    last_seq: u32,
    timeout_ns: i64,
) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.wait(last_seq, timeout_ns)
}

/// Where each generated type's `header.stamp` sits, by fingerprint: the generated code registers
/// it as it loads, so the bindings can pass it as `stamp_offset` from the fingerprint alone.
static STAMP_OFFSETS: Mutex<BTreeMap<u64, i32>> = Mutex::new(BTreeMap::new());

#[unsafe(no_mangle)]
pub extern "C" fn flux2_stamp_offset_register(fingerprint: u64, stamp_offset: i32) {
    STAMP_OFFSETS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .insert(fingerprint, stamp_offset);
}

// `FLUX2_NO_STAMP` for a fingerprint nothing registered.
#[unsafe(no_mangle)]
pub extern "C" fn flux2_stamp_offset(fingerprint: u64) -> i32 {
    STAMP_OFFSETS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .get(&fingerprint)
        .copied()
        .unwrap_or(crate::FLUX2_NO_STAMP)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_signpost_name(
    key: *const c_char,
    fingerprint: u64,
    out: *mut c_char,
    cap: usize,
    len: *mut usize,
) -> flux2_status {
    // SAFETY: key is a valid string.
    let r = unsafe { text(key) }.and_then(|k| signpost_name(k, fingerprint));
    // SAFETY: out holds cap bytes and len is valid for a write.
    status(r.and_then(|name| unsafe { copy_out(&name, out, cap, len) }))
}
