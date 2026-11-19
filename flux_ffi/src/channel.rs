#![allow(
    clippy::disallowed_types,
    reason = "the per-handle call guard; orderings tagged `order:`"
)]

use std::cell::UnsafeCell;
use std::collections::BTreeMap;
use std::ffi::{c_char, c_void};
use std::mem::{align_of, offset_of, size_of};
use std::ops::{Deref, DerefMut};
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use flux_core::{
    Alloc, Channel, DType, Error, FrameMeta, FrameView, ImageSlots, Mapped, MemoryPolicy,
    Published, PublisherConfig, PublisherWatch, QoS, ReleaseMode, WriteSlot, signpost_name,
};

use crate::gpu::{flux_stream, stream_of};
use crate::status::{copy_out, flux_status, in_use, status, status_of, text};
use crate::store::{Stored, stored, stored_ref};

pub struct flux_channel {
    ch: UnsafeCell<Channel>,
    /// Set while a call holds the channel, so a second thread is refused instead of let in.
    busy: AtomicBool,
}

pub const FLUX_MEMORY_DEFAULT: i32 = 0;
pub const FLUX_MEMORY_PRECOMMIT: i32 = 1;
/// Payload pages locked in RAM; PRECOMMIT only reserves them when made.
pub const FLUX_MEMORY_LOCK: i32 = 2;

fn memory_of(m: i32) -> flux_core::Result<MemoryPolicy> {
    match m {
        FLUX_MEMORY_DEFAULT => Ok(MemoryPolicy::default()),
        FLUX_MEMORY_PRECOMMIT => Ok(MemoryPolicy {
            precommit: true,
            lock: false,
        }),
        FLUX_MEMORY_LOCK => Ok(MemoryPolicy {
            precommit: true,
            lock: true,
        }),
        m => Err(Error::InvalidArgument(format!(
            "memory {m} is no FLUX_MEMORY_*"
        ))),
    }
}

/// `replay` 0 is volatile, N is `transient_local(N)`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct flux_qos {
    pub depth: u32,
    pub replay: u32,
    pub max_borrow: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct flux_refused {
    pub max_borrow: u64,
    pub no_reader: u64,
    pub contended: u64,
    pub bad_frame: u64,
    pub fence: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct flux_fence_wait {
    pub commit_ns: u64,
    pub commit_waits: u64,
    pub commit_max_ns: u64,
    pub release_ns: u64,
    pub release_waits: u64,
    pub release_max_ns: u64,
}

/// Same bytes as the frame header in shared memory; `dtype` is a `FLUX_DTYPE_*`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct flux_frame_meta {
    pub ndim: u8,
    pub dtype: u8,
    pub aux_len: u16,
    pub itemsize: u32,
    pub nbytes: u64,
    pub shape: [u64; crate::FLUX_MAX_DIMS],
}

pub const FLUX_DTYPE_U8: u8 = 0;
pub const FLUX_DTYPE_I8: u8 = 1;
pub const FLUX_DTYPE_U16: u8 = 2;
pub const FLUX_DTYPE_I16: u8 = 3;
pub const FLUX_DTYPE_U32: u8 = 4;
pub const FLUX_DTYPE_I32: u8 = 5;
pub const FLUX_DTYPE_U64: u8 = 6;
pub const FLUX_DTYPE_I64: u8 = 7;
pub const FLUX_DTYPE_F16: u8 = 8;
pub const FLUX_DTYPE_F32: u8 = 9;
pub const FLUX_DTYPE_F64: u8 = 10;
pub const FLUX_DTYPE_BF16: u8 = 11;

const _: () = {
    assert!(FLUX_DTYPE_U8 == DType::U8 as u8);
    assert!(FLUX_DTYPE_I8 == DType::I8 as u8);
    assert!(FLUX_DTYPE_U16 == DType::U16 as u8);
    assert!(FLUX_DTYPE_I16 == DType::I16 as u8);
    assert!(FLUX_DTYPE_U32 == DType::U32 as u8);
    assert!(FLUX_DTYPE_I32 == DType::I32 as u8);
    assert!(FLUX_DTYPE_U64 == DType::U64 as u8);
    assert!(FLUX_DTYPE_I64 == DType::I64 as u8);
    assert!(FLUX_DTYPE_F16 == DType::F16 as u8);
    assert!(FLUX_DTYPE_F32 == DType::F32 as u8);
    assert!(FLUX_DTYPE_F64 == DType::F64 as u8);
    assert!(FLUX_DTYPE_BF16 == DType::BF16 as u8);
};

#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum flux_published {
    FLUX_PUBLISHED_OK = 0,
    /// An unknown dtype, or a commit of an empty loan (already committed or aborted).
    FLUX_PUBLISHED_INVALID_ARGUMENT = 1,
    FLUX_PUBLISHED_IN_USE = 9,
    FLUX_PUBLISHED_BACKPRESSURE = 16,
    FLUX_PUBLISHED_TOO_LARGE = 17,
    FLUX_PUBLISHED_NOT_PUBLISHER = 18,
    /// The stream could not be waited for; the slot is held (`flux_channel_set_fence_timeout`).
    FLUX_PUBLISHED_FENCE_FAILED = 19,
    FLUX_PUBLISHED_GPU_WEDGED = 20,
}

impl From<Published> for flux_published {
    fn from(p: Published) -> Self {
        match p {
            Published::Ok => flux_published::FLUX_PUBLISHED_OK,
            Published::Backpressure => flux_published::FLUX_PUBLISHED_BACKPRESSURE,
            Published::TooLarge => flux_published::FLUX_PUBLISHED_TOO_LARGE,
            Published::NotPublisher => flux_published::FLUX_PUBLISHED_NOT_PUBLISHER,
            Published::FenceFailed => flux_published::FLUX_PUBLISHED_FENCE_FAILED,
            Published::GpuWedged => flux_published::FLUX_PUBLISHED_GPU_WEDGED,
        }
    }
}

/// A borrowed frame in caller memory, never copied. A move copies the bytes and zeroes the source.
#[repr(C)]
pub struct flux_frame {
    pub opaque: [u64; 32],
}

/// A claimed slot in caller memory, never copied. End it with `flux_loan_abort`, after a commit too.
#[repr(C)]
pub struct flux_loan {
    pub opaque: [u64; 32],
}

const _: () = {
    assert!(size_of::<Stored<FrameView>>() <= size_of::<flux_frame>());
    assert!(align_of::<Stored<FrameView>>() <= align_of::<flux_frame>());
    assert!(size_of::<LoanCell>() <= size_of::<flux_loan>());
    assert!(align_of::<LoanCell>() <= align_of::<flux_loan>());
    assert!(size_of::<flux_frame_meta>() == size_of::<FrameMeta>());
    assert!(align_of::<flux_frame_meta>() == align_of::<FrameMeta>());
    assert!(offset_of!(flux_frame_meta, aux_len) == offset_of!(FrameMeta, aux_len));
    assert!(offset_of!(flux_frame_meta, itemsize) == offset_of!(FrameMeta, itemsize));
    assert!(offset_of!(flux_frame_meta, nbytes) == offset_of!(FrameMeta, nbytes));
    assert!(offset_of!(flux_frame_meta, shape) == offset_of!(FrameMeta, shape));
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
        self.busy.store(false, Ordering::Release); // order: ffi_busy
    }
}

/// None while another thread is in a call on the same handle.
unsafe fn hold<'a>(p: *mut flux_channel) -> Option<Held<'a>> {
    // SAFETY: the caller passes a live handle.
    let h = unsafe { &*p };
    // order: ffi_busy
    if h.busy.swap(true, Ordering::Acquire) {
        return None;
    }
    Some(Held {
        // SAFETY: busy was clear, so no other call holds the channel.
        ch: unsafe { &mut *h.ch.get() },
        busy: &h.busy,
    })
}

pub(crate) unsafe fn chan_ref<'a>(p: *const flux_channel) -> &'a Channel {
    // SAFETY: the caller passes a live handle.
    let h = unsafe { &*p };
    // SAFETY: readers race a holder only on the counters.
    unsafe { &*h.ch.get() }
}

pub(crate) unsafe fn watch_ref<'a>(p: *const flux_watch) -> &'a PublisherWatch {
    // SAFETY: the caller passes a live handle.
    unsafe { &(*p).w }
}

unsafe fn view<'a>(p: *mut flux_frame) -> &'a mut Stored<FrameView> {
    // SAFETY: flux_frame is sized and aligned for Stored<FrameView> (asserted above).
    unsafe { stored(p) }
}

pub(crate) unsafe fn view_ref<'a>(p: *const flux_frame) -> &'a Stored<FrameView> {
    // SAFETY: as in view.
    unsafe { stored_ref(p) }
}

/// After the commit it keeps the memory the loan wrote mapped, for arrays a binding handed out
/// over it, until the storage is aborted or loaned again.
#[repr(C)]
pub(crate) struct LoanCell {
    pub(crate) slot: Stored<WriteSlot>,
    sent: Stored<Mapped>,
    /// Taken once per loan: a fresh `&mut` per `flux_loan_data` would invalidate the pointer an
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

    fn commit(&mut self, f: impl FnOnce(WriteSlot) -> Published) -> flux_published {
        let Some(w) = self.slot.take() else {
            return flux_published::FLUX_PUBLISHED_INVALID_ARGUMENT;
        };
        self.sent.put(Some(w.mapped()));
        f(w).into()
    }
}

unsafe fn loan<'a>(p: *mut flux_loan) -> &'a mut LoanCell {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above), and zeroed is empty.
    unsafe { &mut *p.cast::<LoanCell>() }
}

unsafe fn shape<'a>(shape: *const u64, ndim: usize) -> &'a [u64] {
    if ndim == 0 {
        return &[];
    }
    // SAFETY: the caller passes ndim readable u64s.
    unsafe { std::slice::from_raw_parts(shape, ndim) }
}

pub(crate) unsafe fn bytes<'a>(data: *const u8, len: usize) -> &'a [u8] {
    if len == 0 {
        return &[];
    }
    // SAFETY: the caller passes len readable bytes.
    unsafe { std::slice::from_raw_parts(data, len) }
}

fn boxed(ch: Channel) -> *mut flux_channel {
    Box::into_raw(Box::new(flux_channel {
        ch: UnsafeCell::new(ch),
        busy: AtomicBool::new(false),
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_new(
    slot_size: u32,
    slot_count: u32,
    out: *mut *mut flux_channel,
) -> flux_status {
    status(Channel::new(slot_size, slot_count).map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

/// A publisher's settings. A size or count of 0 is the default for `alloc` (`FLUX_ALLOC_*`).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct flux_publisher_config {
    pub size: u32,
    pub alloc: i32,
    pub slot_size: u32,
    pub slot_count: u32,
    pub max_slot_size: u32,
    pub max_slot_count: u32,
    pub max_subscribers: u32,
    pub aux_size: u32,
    /// Slots get memory once a subscriber attaches (`FLUX_ALLOC_SLOT` and GPU memory channels).
    pub alloc_on_subscriber: bool,
    /// `FLUX_IMAGE_SLOTS_*`: integrated-GPU slots, each a dmabuf engines read; AUTO with a format.
    pub image_slots: i32,
    /// `FLUX_IMAGE_*`: image slots in this format, the slot size defaulting to one frame.
    pub image_encoding: u32,
    pub image_width: u32,
    pub image_height: u32,
}

pub const FLUX_IMAGE_NONE: u32 = 0;
pub const FLUX_IMAGE_BGRA8: u32 = 1;
pub const FLUX_IMAGE_RGBA8: u32 = 2;
pub const FLUX_IMAGE_MONO8: u32 = 3;
pub const FLUX_IMAGE_MONO16: u32 = 4;
pub const FLUX_IMAGE_NV12: u32 = 5;
pub const FLUX_IMAGE_UYVY: u32 = 6;
pub const FLUX_IMAGE_YUYV: u32 = 7;

/// A frame's planes in an image slot. `pitch` is the row length, the `step` a message carries.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct flux_image_layout {
    pub encoding: u32,
    pub planes: u32,
    pub width: [u32; 4],
    pub height: [u32; 4],
    pub pitch: [u32; 4],
    pub offset: [u32; 4],
    pub bytes: u32,
}

fn image_layout_out(l: Option<flux_core::ImageLayout>, out: *mut flux_image_layout) -> bool {
    let Some(l) = l else {
        return false;
    };
    let four = |a: [u32; 2]| [a[0], a[1], 0, 0];
    let c = flux_image_layout {
        encoding: l.encoding as u32,
        planes: l.planes,
        width: four(l.width),
        height: four(l.height),
        pitch: four(l.pitch),
        offset: four(l.offset),
        bytes: l.bytes,
    };
    // SAFETY: the caller passes a writable flux_image_layout.
    unsafe { out.write(c) };
    true
}

#[unsafe(no_mangle)]
pub extern "C" fn flux_publisher_config_default() -> flux_publisher_config {
    let d = PublisherConfig::default();
    flux_publisher_config {
        size: CONFIG_SIZE,
        alloc: d.alloc as i32,
        slot_size: d.slot_size,
        slot_count: d.slot_count,
        max_slot_size: d.max_slot_size,
        max_slot_count: d.max_slot_count,
        max_subscribers: d.max_subscribers,
        aux_size: d.aux_size,
        alloc_on_subscriber: d.alloc_on_subscriber,
        image_slots: d.image_slots as i32,
        image_encoding: d.image_format.encoding as u32,
        image_width: d.image_format.width,
        image_height: d.image_format.height,
    }
}

#[allow(clippy::cast_possible_truncation)]
const CONFIG_SIZE: u32 = size_of::<flux_publisher_config>() as u32;
/// The size at ABI 6, the first with `size`. A later field leaves it, so an older caller's smaller
/// struct is still read, the new fields as 0.
const CONFIG_SIZE_V6: usize = size_of::<flux_publisher_config>();

/// `config` up to its `size`, over zeroes, which are the defaults. Bytes past what this library
/// knows must be zero.
unsafe fn config_of(
    config: *const flux_publisher_config,
) -> flux_core::Result<flux_publisher_config> {
    // SAFETY: the caller's struct holds its size bytes.
    let raw = unsafe { sized(config.cast::<u8>(), CONFIG_SIZE_V6, CONFIG_SIZE as usize) }
        .ok_or_else(|| {
            Error::InvalidArgument(
                "flux_publisher_config.size is not set: fill the config from \
                 flux_publisher_config_default()"
                    .into(),
            )
        })?;
    let raw = raw.map_err(|()| {
        Error::Unsupported(
            "the config sets fields this library does not know: it is older than the header".into(),
        )
    })?;
    let mut c = std::mem::MaybeUninit::<flux_publisher_config>::zeroed();
    // SAFETY: raw is at most sizeof bytes and does not overlap c.
    unsafe { std::ptr::copy_nonoverlapping(raw.as_ptr(), c.as_mut_ptr().cast::<u8>(), raw.len()) };
    // SAFETY: every field of the zeroed struct is valid, and raw wrote a caller's valid fields.
    Ok(unsafe { c.assume_init() })
}

/// The first `known` bytes of a struct whose leading u32 is its size: None below `min`, Err(())
/// when bytes past `known` are not zero.
pub(crate) unsafe fn sized<'a>(
    p: *const u8,
    min: usize,
    known: usize,
) -> Option<Result<&'a [u8], ()>> {
    // SAFETY: p is valid for a read of its first field.
    let size = unsafe { p.cast::<u32>().read_unaligned() } as usize;
    if size < min {
        return None;
    }
    // SAFETY: the caller's struct holds size bytes.
    let all = unsafe { std::slice::from_raw_parts(p, size) };
    let (head, extra) = all.split_at(size.min(known));
    Some(if extra.iter().any(|&b| b != 0) {
        Err(())
    } else {
        Ok(head)
    })
}

/// The publisher of signpost `name`, with its settings in `config` and `FLUX_MEMORY_*` `memory`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_create(
    name: *const c_char,
    config: *const flux_publisher_config,
    fingerprint: u64,
    memory: i32,
    stream: *const flux_stream,
    out: *mut *mut flux_channel,
) -> flux_status {
    // SAFETY: config is valid for a read of its size.
    let c = match unsafe { config_of(config) } {
        Ok(c) => c,
        Err(e) => return status_of(&e),
    };
    let mem = match memory_of(memory) {
        Ok(m) => m,
        Err(e) => return status_of(&e),
    };
    let alloc = match c.alloc {
        FLUX_ALLOC_ARENA => Alloc::Arena,
        FLUX_ALLOC_SLOT => Alloc::Slot,
        a => {
            return status(Err(Error::InvalidArgument(format!(
                "alloc {a} is neither FLUX_ALLOC_ARENA nor FLUX_ALLOC_SLOT"
            ))));
        }
    };
    let image_slots = match c.image_slots {
        FLUX_IMAGE_SLOTS_AUTO => ImageSlots::Auto,
        FLUX_IMAGE_SLOTS_OFF => ImageSlots::Off,
        FLUX_IMAGE_SLOTS_ON => ImageSlots::On,
        m => {
            return status(Err(Error::InvalidArgument(format!(
                "image_slots {m} is no FLUX_IMAGE_SLOTS_*"
            ))));
        }
    };
    let Some(encoding) = flux_core::ImageEncoding::from_u32(c.image_encoding) else {
        return status(Err(Error::InvalidArgument(format!(
            "image_encoding {} is no FLUX_IMAGE_*",
            c.image_encoding
        ))));
    };
    let config = PublisherConfig {
        alloc,
        slot_size: c.slot_size,
        slot_count: c.slot_count,
        max_slot_size: c.max_slot_size,
        max_slot_count: c.max_slot_count,
        max_subscribers: c.max_subscribers,
        aux_size: c.aux_size,
        alloc_on_subscriber: c.alloc_on_subscriber,
        image_slots,
        image_format: flux_core::ImageFormat {
            encoding,
            width: c.image_width,
            height: c.image_height,
        },
    };
    // SAFETY: stream is NULL or a live handle.
    let stream = unsafe { stream_of(stream) };
    // SAFETY: name is a valid string.
    let r = unsafe { text(name) }.and_then(|n| {
        Channel::create_config(
            n,
            &config,
            fingerprint,
            mem,
            stream,
            flux_stamp_offset(fingerprint),
            scalar_bytes_of(fingerprint),
        )
    });
    status(r.map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

/// A subscription's hold on a signpost while it has no publisher.
pub struct flux_watch {
    w: PublisherWatch,
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_watch_open(
    name: *const c_char,
    out: *mut *mut flux_watch,
) -> flux_status {
    // SAFETY: name is a valid string.
    let r = unsafe { text(name) }.and_then(PublisherWatch::open);
    status(r.map(|w| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(Box::into_raw(Box::new(flux_watch { w }))) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_watch_free(w: *mut flux_watch) {
    if !w.is_null() {
        // SAFETY: a handle from flux_watch_open, freed once.
        drop(unsafe { Box::from_raw(w) });
    }
}

// 0 until the first publisher; it changes with every restart.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_watch_epoch(w: *const flux_watch) -> u32 {
    // SAFETY: valid handle.
    unsafe { &(*w).w }.epoch()
}

// Notes the epoch for flux_watch_wait and an executor to wait on; open_watched arms it too.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_watch_arm(w: *const flux_watch) -> u32 {
    // SAFETY: valid handle.
    unsafe { &(*w).w }.arm()
}

// Until the epoch moves from the armed one, or a signal or stray wake: true asks for a look.
// False only on a timeout.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_watch_wait(w: *const flux_watch, timeout_ns: i64) -> bool {
    // SAFETY: valid handle.
    unsafe { &(*w).w }.wait(timeout_ns)
}

/// After a first success through `w`, starts at its publisher's oldest frame. `w` is not kept.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_open_watched(
    w: *const flux_watch,
    fingerprint: u64,
    q: flux_qos,
    memory: i32,
    stream: *const flux_stream,
    out: *mut *mut flux_channel,
) -> flux_status {
    // SAFETY: stream is NULL or a live handle.
    let stream = unsafe { stream_of(stream) };
    let r = qos_of(q).and_then(|qos| {
        Channel::open_watched_on(
            // SAFETY: valid handle.
            unsafe { &(*w).w },
            fingerprint,
            qos,
            memory_of(memory)?,
            stream,
            flux_stamp_offset(fingerprint),
        )
    });
    status(r.map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

fn qos_to_c(q: QoS) -> flux_qos {
    flux_qos {
        depth: q.depth(),
        replay: q.durability().replay,
        max_borrow: q.borrow_limit(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn flux_qos_default() -> flux_qos {
    qos_to_c(QoS::default())
}

fn qos_of(q: flux_qos) -> flux_core::Result<QoS> {
    QoS::new(q.depth)
        .and_then(|x| x.max_borrow(q.max_borrow))
        .map(|x| x.transient_local(q.replay))
}

/// A subscriber of signpost `name`. `FLUX_NOT_READY` until a publisher is up.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_open(
    name: *const c_char,
    fingerprint: u64,
    memory: i32,
    stream: *const flux_stream,
    out: *mut *mut flux_channel,
) -> flux_status {
    // SAFETY: stream is NULL or a live handle.
    let stream = unsafe { stream_of(stream) };
    // SAFETY: name is a valid string.
    let r = unsafe { text(name) }.and_then(|n| {
        Channel::open_on(
            n,
            fingerprint,
            memory_of(memory)?,
            stream,
            flux_stamp_offset(fingerprint),
        )
    });
    status(r.map(|ch| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(boxed(ch)) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_free(ch: *mut flux_channel) {
    if !ch.is_null() {
        // SAFETY: a handle from one of the constructors, freed once.
        drop(unsafe { Box::from_raw(ch) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_publish(
    ch: *mut flux_channel,
    data: *const u8,
    len: usize,
) -> flux_published {
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return flux_published::FLUX_PUBLISHED_IN_USE;
    };
    // SAFETY: len readable bytes.
    let data = unsafe { bytes(data, len) };
    ch.publish(data).into()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_publish_typed(
    ch: *mut flux_channel,
    data: *const u8,
    len: usize,
    dtype: u8,
    shape_: *const u64,
    ndim: usize,
) -> flux_published {
    let Some(dt) = DType::from_raw(dtype) else {
        return flux_published::FLUX_PUBLISHED_INVALID_ARGUMENT;
    };
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return flux_published::FLUX_PUBLISHED_IN_USE;
    };
    // SAFETY: len readable bytes.
    let data = unsafe { bytes(data, len) };
    // SAFETY: ndim readable dims.
    let sh = unsafe { shape(shape_, ndim) };
    ch.publish_typed(data, dt, sh).into()
}

/// The whole slot as one u8 run, OK with a slot in `out`. `BACKPRESSURE` when none is free.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_loan(
    ch: *mut flux_channel,
    out: *mut flux_loan,
) -> flux_published {
    // SAFETY: valid handle.
    let size = u64::from(unsafe { chan_ref(ch) }.slot_size());
    // SAFETY: valid handle and loan storage.
    unsafe { flux_channel_loan_typed(ch, FLUX_DTYPE_U8, &raw const size, 1, out) }
}

/// The shape is checked at commit.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_loan_typed(
    ch: *mut flux_channel,
    dtype: u8,
    shape_: *const u64,
    ndim: usize,
    out: *mut flux_loan,
) -> flux_published {
    // SAFETY: valid loan storage.
    let slot = unsafe { loan(out) };
    slot.put(None);
    let Some(dt) = DType::from_raw(dtype) else {
        return flux_published::FLUX_PUBLISHED_INVALID_ARGUMENT;
    };
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return flux_published::FLUX_PUBLISHED_IN_USE;
    };
    // SAFETY: ndim readable dims.
    let sh = unsafe { shape(shape_, ndim) };
    match ch.try_loan_typed(dt, sh) {
        Ok(w) => {
            slot.put(Some(w));
            flux_published::FLUX_PUBLISHED_OK
        }
        Err(p) => p.into(),
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_data(l: *mut flux_loan, capacity: *mut usize) -> *mut u8 {
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
pub unsafe extern "C" fn flux_loan_aux(l: *mut flux_loan, capacity: *mut usize) -> *mut u8 {
    // SAFETY: valid loan storage.
    let Some(w) = unsafe { loan(l) }.slot.get() else {
        return std::ptr::null_mut();
    };
    let aux = w.aux_region();
    // SAFETY: capacity is valid for a write.
    unsafe { capacity.write(aux.len()) };
    aux.cast::<u8>()
}

/// Bytes of the aux region the frame carries; past its capacity, the commit is `TOO_LARGE`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_set_aux_len(l: *mut flux_loan, len: usize) {
    // SAFETY: valid loan storage.
    if let Some(w) = unsafe { loan(l) }.slot.get_mut() {
        w.set_aux_len(len);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_stream(l: *const flux_loan) -> *mut c_void {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above).
    unsafe { &*l.cast::<LoanCell>() }
        .slot
        .get()
        .map_or(std::ptr::null_mut(), |w| w.stream().native())
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_device_ptr(l: *const flux_loan) -> u64 {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above).
    unsafe { &*l.cast::<LoanCell>() }
        .slot
        .get()
        .map_or(0, WriteSlot::device_ptr)
}

/// False but in an image slot with a declared format.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_image_layout(
    l: *const flux_loan,
    out: *mut flux_image_layout,
) -> bool {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above).
    let w = unsafe { &*l.cast::<LoanCell>() }.slot.get();
    image_layout_out(w.and_then(WriteSlot::image_layout), out)
}

/// The slot's dmabuf, owned by the channel; -1 but in an image slot.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_dmabuf_fd(l: *const flux_loan) -> i32 {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above).
    let w = unsafe { &*l.cast::<LoanCell>() }.slot.get();
    w.and_then(WriteSlot::dmabuf_fd).unwrap_or(-1)
}

// Host bytes into the payload at `at`: a plain copy into host memory, or a copy queued on the
// loan's stream into device memory, which the commit waits for. Pageable `src` is read before
// this returns; page-locked `src` must stay unchanged until the commit or abort. False past the
// capacity, for an ended loan, or when the copy was not queued. Not in flux.h, which is at its
// comment budget; the C++ `WriteSlot::copy_from` and `FrameView::copy_to` say the same.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_copy_from(
    l: *mut flux_loan,
    at: usize,
    src: *const u8,
    n: usize,
) -> bool {
    // SAFETY: valid loan storage.
    let Some(w) = unsafe { loan(l) }.slot.get_mut() else {
        return false;
    };
    let src: &[u8] = if n == 0 {
        &[]
    } else {
        // SAFETY: the caller gives n readable bytes at src.
        unsafe { std::slice::from_raw_parts(src, n) }
    };
    w.copy_from(at, src)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_commit(l: *mut flux_loan) -> flux_published {
    // SAFETY: valid loan storage.
    unsafe { loan(l) }.commit(WriteSlot::commit)
}

/// Shrinks a one-dimensional loan to `nbytes`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_commit_len(l: *mut flux_loan, nbytes: usize) -> flux_published {
    // SAFETY: valid loan storage.
    unsafe { loan(l) }.commit(|w| w.commit_len(nbytes))
}

/// Ends the loan (its slot's old frame is gone either way) or the mapping a commit kept.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_abort(l: *mut flux_loan) {
    // SAFETY: valid loan storage.
    unsafe { loan(l) }.put(None);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_loan_valid(l: *const flux_loan) -> bool {
    // SAFETY: flux_loan is sized and aligned for LoanCell (asserted above).
    unsafe { &*l.cast::<LoanCell>() }.slot.get().is_some()
}

fn taken(slot: &mut Stored<FrameView>, v: Option<FrameView>) -> flux_status {
    slot.put(v);
    if slot.get().is_some() {
        flux_status::FLUX_OK
    } else {
        flux_status::FLUX_NO_FRAME
    }
}

/// OK with the oldest frame newer than the cursor in `out`, else `NO_FRAME`; `out` is released first.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_take(
    ch: *mut flux_channel,
    out: *mut flux_frame,
) -> flux_status {
    // SAFETY: valid frame storage.
    let slot = unsafe { view(out) };
    slot.put(None);
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return in_use();
    };
    taken(slot, ch.take())
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_take_blocking(
    ch: *mut flux_channel,
    timeout_ns: i64,
    out: *mut flux_frame,
) -> flux_status {
    // SAFETY: valid frame storage.
    let slot = unsafe { view(out) };
    slot.put(None);
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return in_use();
    };
    taken(slot, ch.take_blocking(timeout_ns))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_data(v: *const flux_frame, len: *mut usize) -> *const u8 {
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

// The frame's bytes from `at` into host `dst`, as `flux_loan_copy_from` says. False past the
// frame's bytes, for an empty view, or when the copy or the wait failed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_copy_to(
    v: *const flux_frame,
    at: usize,
    dst: *mut u8,
    n: usize,
) -> bool {
    // SAFETY: valid view storage.
    let Some(fv) = (unsafe { view_ref(v) }).get() else {
        return false;
    };
    let dst: &mut [u8] = if n == 0 {
        &mut []
    } else {
        // SAFETY: the caller gives n writable bytes at dst.
        unsafe { std::slice::from_raw_parts_mut(dst, n) }
    };
    fv.copy_to(at, dst)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_aux(v: *const flux_frame, len: *mut usize) -> *const u8 {
    // SAFETY: valid view storage.
    let Some(fv) = (unsafe { view_ref(v) }).get() else {
        return std::ptr::null();
    };
    let aux = fv.aux();
    // SAFETY: len is valid for a write.
    unsafe { len.write(aux.len()) };
    aux.as_ptr()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_stream(v: *const flux_frame) -> *mut c_void {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(std::ptr::null_mut(), |fv| fv.stream().native())
}

/// The frame's device address, 0 when the channel declared no stream.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_device_ptr(v: *const flux_frame) -> u64 {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(0, FrameView::device_ptr)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_image_layout(
    v: *const flux_frame,
    out: *mut flux_image_layout,
) -> bool {
    // SAFETY: valid view storage.
    let f = unsafe { view_ref(v) }.get();
    image_layout_out(f.and_then(FrameView::image_layout), out)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_dmabuf_fd(v: *const flux_frame) -> i32 {
    // SAFETY: valid view storage.
    let f = unsafe { view_ref(v) }.get();
    f.and_then(FrameView::dmabuf_fd).unwrap_or(-1)
}

/// Points into `v`, so moving the frame moves it. Size a typed read from `dtype`, not `itemsize`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_get_meta(v: *const flux_frame) -> *const flux_frame_meta {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }.get().map_or(std::ptr::null(), |fv| {
        std::ptr::from_ref(fv.meta()).cast::<flux_frame_meta>()
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_ticket(v: *const flux_frame) -> u64 {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }.get().map_or(0, FrameView::ticket)
}

/// The channel as trace events name it (`flux/trace.h`), NULL for an empty frame.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_trace_channel(v: *const flux_frame) -> *const c_void {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(std::ptr::null(), FrameView::trace_channel)
}

/// `CLOCK_MONOTONIC` at commit.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_publish_ns(v: *const flux_frame) -> u64 {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .map_or(0, FrameView::publish_ns)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_release(v: *mut flux_frame) {
    // SAFETY: valid view storage.
    unsafe { view(v) }.put(None);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_valid(v: *const flux_frame) -> bool {
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }.get().is_some()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_set_qos(ch: *mut flux_channel, q: flux_qos) -> flux_status {
    let qos = qos_of(q);
    // SAFETY: valid handle.
    let Some(mut ch) = (unsafe { hold(ch) }) else {
        return in_use();
    };
    status(qos.and_then(|x| ch.set_qos(x)))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_qos(ch: *const flux_channel) -> flux_qos {
    // SAFETY: valid handle.
    qos_to_c(*unsafe { chan_ref(ch) }.qos())
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_refused(ch: *const flux_channel) -> flux_refused {
    // SAFETY: valid handle.
    let r = unsafe { chan_ref(ch) }.refused();
    flux_refused {
        max_borrow: r.max_borrow,
        no_reader: r.no_reader,
        contended: r.contended,
        bad_frame: r.bad_frame,
        fence: r.fence,
    }
}

/// Commits and releases that could not wait for the stream; each holds its slot (see `flux_channel_set_fence_timeout`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_fence_failed(ch: *const flux_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.fence_failed()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_fence_wait(ch: *const flux_channel) -> flux_fence_wait {
    // SAFETY: valid handle.
    let w = unsafe { chan_ref(ch) }.fence_wait();
    flux_fence_wait {
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
pub unsafe extern "C" fn flux_channel_set_fence_timeout(ch: *const flux_channel, ns: u64) {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.set_fence_timeout(ns);
}

pub const FLUX_RELEASE_WAIT: i32 = 0;
pub const FLUX_RELEASE_DEFERRED: i32 = 1;

/// `FLUX_RELEASE_DEFERRED`: a release returns at once; its slot goes back once the device is done.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_set_release_mode(
    ch: *const flux_channel,
    mode: i32,
) -> flux_status {
    let mode = match mode {
        FLUX_RELEASE_WAIT => ReleaseMode::Wait,
        FLUX_RELEASE_DEFERRED => ReleaseMode::Deferred,
        _ => {
            return status(Err(Error::InvalidArgument(format!(
                "release mode {mode} is neither FLUX_RELEASE_WAIT nor FLUX_RELEASE_DEFERRED"
            ))));
        }
    };
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.set_release_mode(mode);
    flux_status::FLUX_OK
}

pub const FLUX_ALLOC_ARENA: i32 = 0;
pub const FLUX_ALLOC_SLOT: i32 = 1;

pub const FLUX_IMAGE_SLOTS_AUTO: i32 = 0;
pub const FLUX_IMAGE_SLOTS_OFF: i32 = 1;
pub const FLUX_IMAGE_SLOTS_ON: i32 = 2;

/// Orders the frame's channel stream after what `s` queued so far, on the device, not the host.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_frame_after(v: *const flux_frame, s: *const flux_stream) -> bool {
    // SAFETY: s is null or a live stream.
    let other = unsafe { stream_of(s) };
    // SAFETY: valid view storage.
    unsafe { view_ref(v) }
        .get()
        .is_some_and(|fv| fv.stream().after(&other))
}

/// The payload is device memory: frames and loans have no host bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_device_backed(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.device_backed()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_ready(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.ready()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_can_borrow(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.can_borrow()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_lost(ch: *const flux_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.lost()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_dropped(ch: *const flux_channel) -> u64 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.dropped()
}

/// Live subscriber processes of a publisher's channel; 0 on a subscriber's channel.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_subscriber_count(ch: *const flux_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.subscriber_count()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_attach_generation(ch: *const flux_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.attach_generation()
}

/// Why takes find nothing until the publisher restarts: it refused us, or a slot missed our policy.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_attach_status(ch: *mut flux_channel) -> flux_status {
    // SAFETY: valid handle.
    let Some(ch) = (unsafe { hold(ch) }) else {
        return in_use();
    };
    ch.attach_error().map_or(flux_status::FLUX_OK, status_of)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_orphaned(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.orphaned()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_inherited(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.inherited()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_slot_size(ch: *const flux_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.slot_size()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_slot_count(ch: *const flux_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.slot_count()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_aux_size(ch: *const flux_channel) -> u32 {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.aux_size()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_payload_readonly(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.payload_readonly()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_pages_committed(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.pages_committed()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_channel_pages_locked(ch: *const flux_channel) -> bool {
    // SAFETY: valid handle.
    unsafe { chan_ref(ch) }.pages_locked()
}

/// Where each generated type's `header.stamp` sits, by fingerprint: the generated code registers
/// it as it loads, so the bindings can pass it as `stamp_offset` from the fingerprint alone.
static STAMP_OFFSETS: Mutex<BTreeMap<u64, i32>> = Mutex::new(BTreeMap::new());

#[unsafe(no_mangle)]
pub extern "C" fn flux_stamp_offset_register(fingerprint: u64, stamp_offset: i32) {
    STAMP_OFFSETS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .insert(fingerprint, stamp_offset);
}

/// `FLUX_NO_STAMP` for a fingerprint nothing registered.
#[unsafe(no_mangle)]
pub extern "C" fn flux_stamp_offset(fingerprint: u64) -> i32 {
    STAMP_OFFSETS
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .get(&fingerprint)
        .copied()
        .unwrap_or(crate::FLUX_NO_STAMP)
}

static GENERATED: Mutex<Vec<(u64, u32)>> = Mutex::new(Vec::new());

/// A generated type's fingerprint and scalar block: its stream publishers' host slots, its slot size.
#[unsafe(no_mangle)]
pub extern "C" fn flux_layout_register(fingerprint: u64, scalar_bytes: u32) {
    let mut g = GENERATED
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    if !g.iter().any(|(fp, _)| *fp == fingerprint) {
        g.push((fingerprint, scalar_bytes));
    }
}

fn scalar_bytes_of(fingerprint: u64) -> Option<u32> {
    if fingerprint == flux_core::NO_SCHEMA {
        return None;
    }
    GENERATED
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .iter()
        .find(|(fp, _)| *fp == fingerprint)
        .map(|(_, s)| *s)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_signpost_name(
    key: *const c_char,
    fingerprint: u64,
    out: *mut c_char,
    cap: usize,
    len: *mut usize,
) -> flux_status {
    // SAFETY: key is a valid string.
    let r = unsafe { text(key) }.and_then(|k| signpost_name(k, fingerprint));
    // SAFETY: out holds cap bytes and len is valid for a write.
    match r {
        // SAFETY: out holds cap bytes and len is valid for a write.
        Ok(name) => unsafe { copy_out(&name, out, cap, len) },
        Err(e) => status_of(&e),
    }
}
