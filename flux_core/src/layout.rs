//! Segment ABI. Processes built by other compilers read these bytes, so every type here is
//! `#[repr(C)]`, fixed width and free of bitfields. The words the publisher writes and the ones
//! parked subscribers write sit on separate cache lines. The asserts at the bottom pin sizes and
//! offsets.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use std::cell::UnsafeCell;
use std::mem::{align_of, offset_of, size_of};
use std::sync::atomic::{AtomicI32, AtomicU8, AtomicU32, AtomicU64};

pub const MAGIC: u32 = 0x464C_5832; // 'FLX2'
pub const SIGNPOST_MAGIC: u32 = 0x464C_5350; // 'FLSP'
pub const LAYOUT_VERSION: u32 = 6;
pub const CACHE_LINE: usize = 64;
// One page, so a subscriber can drop the payload to read-only while the headers it writes stay
// writable. Fixed rather than read from the kernel: the layout must match across processes.
pub const PAYLOAD_ALIGN: usize = 4096;
pub const MAX_DIMS: usize = 8;
pub const DEFAULT_MAX_SUBSCRIBERS: u32 = 16;
pub const MAX_SUBSCRIBERS: u32 = 4096;

pub const INIT_UNINIT: u32 = 0;
pub const INIT_READY: u32 = 2;

/// One slot held by a subscriber stalls the publisher. The public constructors check it; a
/// segment itself works with one.
pub const MIN_SLOT_COUNT: u32 = 2;
pub const MAX_SLOT_COUNT: u32 = 65_535;

/// `SlotHeader::commit_ticket` values besides a frame's ticket. Tickets start at 1.
pub const TICKET_EMPTY: u64 = 0;
pub const TICKET_CLAIMED: u64 = u64::MAX;

/// The raw path has no generated schema to fingerprint.
pub const NO_SCHEMA: u64 = 0;

/// OFD lock bytes. Locks are kernel state, so these offsets cost no file space.
pub const PUBLISHER_LOCK_BYTE: i64 = 0;
pub const fn reader_lock_byte(index: u32) -> i64 {
    1 + index as i64
}
pub const SIGNPOST_BOOTSTRAP_BYTE: i64 = 0;
pub const SIGNPOST_PUBLISHER_BYTE: i64 = 1;
#[allow(clippy::cast_possible_wrap)]
pub const fn roster_lock_byte(entry: usize) -> i64 {
    2 + entry as i64
}
/// Read-locked by every process that has the signpost open. A write lock here proves no user.
pub const SIGNPOST_USER_BYTE: i64 = roster_lock_byte(ROSTER_ENTRIES);

pub const SIGNPOST_BYTES: usize = 16384;
pub const ROSTER_ENTRIES: usize = 62;
pub const MAX_LABEL_LEN: usize = 240;
pub const KEY_RECORD_BYTES: usize = 192;
pub const KEY_NONE: u32 = 0;
pub const KEY_WRITING: u32 = 1;
pub const KEY_READY: u32 = 2;
pub const ENDPOINT_PUBLISHER: u32 = 1;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageKind {
    HostInline = 0,
    /// The payload is a device allocation shared by fd; the segment holds the control plane only.
    CudaIpc = 1,
}

/// Wire ABI: append only. A frame with an unknown value is refused.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DType {
    U8 = 0,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    I64,
    F16,
    F32,
    F64,
    BF16,
}

impl DType {
    pub const fn size(self) -> u32 {
        match self {
            DType::U8 | DType::I8 => 1,
            DType::U16 | DType::I16 | DType::F16 | DType::BF16 => 2,
            DType::U32 | DType::I32 | DType::F32 => 4,
            DType::U64 | DType::I64 | DType::F64 => 8,
        }
    }

    pub const fn from_raw(v: u8) -> Option<DType> {
        Some(match v {
            0 => DType::U8,
            1 => DType::I8,
            2 => DType::U16,
            3 => DType::I16,
            4 => DType::U32,
            5 => DType::I32,
            6 => DType::U64,
            7 => DType::I64,
            8 => DType::F16,
            9 => DType::F32,
            10 => DType::F64,
            11 => DType::BF16,
            _ => return None,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameMeta {
    pub ndim: u8,
    pub dtype: u8,
    pub reserved0: [u8; 2],
    pub itemsize: u32,
    pub nbytes: u64,
    pub shape: [u64; MAX_DIMS],
}

impl FrameMeta {
    /// Size a typed view from this, not `itemsize`: a mismatched pair could reach past the frame.
    pub fn dtype(&self) -> Option<DType> {
        DType::from_raw(self.dtype)
    }

    pub fn shape(&self) -> &[u64] {
        self.shape
            .get(..usize::from(self.ndim))
            .unwrap_or(&self.shape)
    }
}

/// Followed in the segment by `max_subscribers` reader counters, packed into shared lines: a claim
/// reads every counter of the slot, so one line beats one per subscriber. Padding them gained
/// nothing with eight subscribers borrowing one frame at once, and slowed claims under cache load.
#[repr(C, align(64))]
pub struct SlotHeader {
    /// The frame's ticket, `TICKET_EMPTY`, or `TICKET_CLAIMED` while the publisher owns the
    /// slot. The publisher alone writes it; the rest of the header is written only while claimed.
    pub commit_ticket: AtomicU64,
    pub(crate) meta: UnsafeCell<FrameMeta>,
    /// `CLOCK_MONOTONIC` at commit.
    pub publish_ns: AtomicU64,
    _reserved: [u8; 32],
}

#[repr(C, align(64))]
pub struct ControlHeader {
    pub init_state: AtomicU32,
    pub magic: u32,
    pub version: u32,
    pub slot_size: u32,
    pub slot_count: u32,
    pub max_subscribers: u32,
    pub storage_kind: u8,
    _reserved0: [u8; 3],
    pub device_id: u32,
    pub fingerprint: u64,
    /// One past the highest subscriber number ever taken. A subscriber raises it before its first
    /// borrow or wait, so claims and the wake gate read only the counters below it.
    pub readers_high: AtomicU32,
    /// `CudaIpc`: the allocating device's UUID. Ordinals differ between processes.
    pub device_uuid: [u8; 16],
    _reserved1: [u8; 4],
    /// The newest committed ticket, 0 before the first.
    pub latest: AtomicU64,
    /// Futex word, bumped on every publish.
    pub wakeup: AtomicU32,
    _reserved2: [u8; 52],
    /// Waits by a process with no subscriber number here: the publisher's own, or a heap
    /// channel's. Others count under their number in the words after this header, one store
    /// each, so a dead number's count is taken back whole. The publisher skips the wake syscall
    /// while all of them are 0. A wait is a parked `take_blocking` or a source bound to an
    /// executor.
    pub waiters: AtomicI32,
    _reserved3: [u8; 60],
}

impl ControlHeader {
    pub(crate) const fn new(layout: SegmentLayout, fingerprint: u64, kind: StorageKind) -> Self {
        ControlHeader {
            init_state: AtomicU32::new(INIT_UNINIT),
            magic: MAGIC,
            version: LAYOUT_VERSION,
            slot_size: layout.slot_size,
            slot_count: layout.slot_count,
            max_subscribers: layout.max_subscribers,
            storage_kind: kind as u8,
            _reserved0: [0; 3],
            device_id: 0,
            fingerprint,
            readers_high: AtomicU32::new(0),
            device_uuid: [0; 16],
            _reserved1: [0; 4],
            latest: AtomicU64::new(0),
            wakeup: AtomicU32::new(0),
            _reserved2: [0; 52],
            waiters: AtomicI32::new(0),
            _reserved3: [0; 60],
        }
    }
}

/// Reads one plain field of the header at `$p` (a `*const ControlHeader`), volatile, once.
macro_rules! peer_field {
    ($p:expr, $field:ident) => {{
        // SAFETY: a field inside the mapped header that `HeaderConfig::read`'s caller vouches for;
        // no reference is made to it.
        let at = unsafe { &raw const (*$p).$field };
        // SAFETY: as above, and aligned, as its offset is asserted below.
        unsafe { at.read_volatile() }
    }};
}

/// The plain fields of a `ControlHeader`, copied once. Any process can write the header, so what
/// is checked is this copy, and nothing is read from the mapping's fields again.
#[derive(Clone, Copy, Debug)]
pub(crate) struct HeaderConfig {
    pub magic: u32,
    pub version: u32,
    pub slot_size: u32,
    pub slot_count: u32,
    pub max_subscribers: u32,
    pub storage_kind: u8,
    pub fingerprint: u64,
    pub device_uuid: [u8; 16],
}

impl HeaderConfig {
    /// # Safety
    /// `ctrl` must point at a mapped `ControlHeader`.
    pub(crate) unsafe fn read(ctrl: *const ControlHeader) -> HeaderConfig {
        HeaderConfig {
            magic: peer_field!(ctrl, magic),
            version: peer_field!(ctrl, version),
            slot_size: peer_field!(ctrl, slot_size),
            slot_count: peer_field!(ctrl, slot_count),
            max_subscribers: peer_field!(ctrl, max_subscribers),
            storage_kind: peer_field!(ctrl, storage_kind),
            fingerprint: peer_field!(ctrl, fingerprint),
            device_uuid: peer_field!(ctrl, device_uuid),
        }
    }

    /// Unchecked: `layout_config_ok` first.
    pub(crate) fn layout(&self) -> SegmentLayout {
        SegmentLayout {
            slot_size: self.slot_size,
            slot_count: self.slot_count,
            max_subscribers: self.max_subscribers,
        }
    }

    pub(crate) fn host_payload(&self) -> bool {
        self.storage_kind == StorageKind::HostInline as u8
    }
}

/// Fixed-name rendezvous object. `epoch` names the current segment; 0 = none yet.
#[repr(C, align(64))]
pub struct Signpost {
    pub init_state: AtomicU32,
    pub magic: AtomicU32,
    pub version: AtomicU32,
    pub epoch: AtomicU32,
    _reserved: [u8; 48],
}

/// The exact channel key, which the signpost name only gives back flattened. Written once by the
/// first announcer; a writer that dies mid-write leaves it `KEY_WRITING`, which costs exactness only.
#[repr(C, align(64))]
pub struct KeyRecord {
    pub state: AtomicU32,
    pub len: AtomicU32,
    pub bytes: [AtomicU8; KEY_RECORD_BYTES],
}

/// One announced endpoint, for enumeration only. Held by an OFD write lock on
/// `roster_lock_byte(index)`; the holder writes it under the `seq` seqlock.
#[repr(C, align(64))]
pub struct RosterEntry {
    pub seq: AtomicU32,
    /// 0 = empty.
    pub pid: AtomicU32,
    pub flags: AtomicU32,
    pub label_len: AtomicU32,
    pub label: [AtomicU8; MAX_LABEL_LEN],
}

#[repr(C, align(64))]
pub struct SignpostPage {
    pub head: Signpost,
    pub key: KeyRecord,
    _reserved: [u8; 192],
    pub roster: [RosterEntry; ROSTER_ENTRIES],
}

pub const fn align_up(n: usize, a: usize) -> usize {
    n.div_ceil(a) * a
}

/// Bounds every value an offset is derived from. Unbounded, a header written by another process
/// could make `total_bytes` wrap.
pub const fn layout_config_ok(slot_size: u32, slot_count: u32, max_subscribers: u32) -> bool {
    slot_size >= 1
        && slot_count >= 1
        && slot_count <= MAX_SLOT_COUNT
        && max_subscribers >= 1
        && max_subscribers <= MAX_SUBSCRIBERS
}

/// Offsets are derived, never stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SegmentLayout {
    pub slot_size: u32,
    pub slot_count: u32,
    pub max_subscribers: u32,
}

impl SegmentLayout {
    pub const WAITERS_OFFSET: usize = align_up(size_of::<ControlHeader>(), CACHE_LINE);

    pub const fn waiters_bytes(&self) -> usize {
        align_up(
            self.max_subscribers as usize * size_of::<AtomicI32>(),
            CACHE_LINE,
        )
    }
    /// Entries of the ticket index: a power of two, at least twice the slots, so a ticket's
    /// entry outlives its frame unless the take is that far behind.
    pub const fn index_len(&self) -> usize {
        (2 * self.slot_count as usize).next_power_of_two()
    }
    pub const fn index_offset(&self) -> usize {
        Self::WAITERS_OFFSET + self.waiters_bytes()
    }
    pub const fn slots_offset(&self) -> usize {
        self.index_offset() + align_up(self.index_len() * size_of::<AtomicU32>(), CACHE_LINE)
    }
    pub const fn readers_bytes(&self) -> usize {
        align_up(
            self.max_subscribers as usize * size_of::<AtomicU32>(),
            CACHE_LINE,
        )
    }
    pub const fn slot_stride(&self) -> usize {
        size_of::<SlotHeader>() + self.readers_bytes()
    }
    pub const fn slots_bytes(&self) -> usize {
        self.slot_stride() * self.slot_count as usize
    }
    pub const fn payload_stride(&self) -> usize {
        align_up(self.slot_size as usize, PAYLOAD_ALIGN)
    }
    pub const fn payload_offset(&self) -> usize {
        align_up(self.slots_offset() + self.slots_bytes(), PAYLOAD_ALIGN)
    }
    pub const fn payload_bytes(&self) -> usize {
        self.payload_stride() * self.slot_count as usize
    }
    pub const fn total_bytes(&self) -> usize {
        self.payload_offset() + self.payload_bytes()
    }
}

const _: () = {
    assert!(size_of::<FrameMeta>() == 80);
    assert!(align_of::<FrameMeta>() == 8);

    assert!(size_of::<SlotHeader>() == 128);
    assert!(align_of::<SlotHeader>() == CACHE_LINE);
    assert!(offset_of!(SlotHeader, commit_ticket) == 0);
    assert!(offset_of!(SlotHeader, meta) == 8);
    assert!(offset_of!(SlotHeader, publish_ns) == 88);

    assert!(size_of::<ControlHeader>() == 192);
    assert!(align_of::<ControlHeader>() == CACHE_LINE);
    assert!(offset_of!(ControlHeader, init_state) == 0);
    assert!(offset_of!(ControlHeader, magic) == 4);
    assert!(offset_of!(ControlHeader, max_subscribers) == 20);
    assert!(offset_of!(ControlHeader, storage_kind) == 24);
    assert!(offset_of!(ControlHeader, device_id) == 28);
    assert!(offset_of!(ControlHeader, fingerprint) == 32);
    assert!(offset_of!(ControlHeader, readers_high) == 40);
    assert!(offset_of!(ControlHeader, device_uuid) == 44);
    assert!(offset_of!(ControlHeader, latest) == 64);
    assert!(offset_of!(ControlHeader, wakeup) == 72);
    assert!(offset_of!(ControlHeader, waiters) == 128);

    assert!(size_of::<Signpost>() == CACHE_LINE);
    assert!(offset_of!(Signpost, init_state) == 0);
    assert!(offset_of!(Signpost, magic) == 4);
    assert!(offset_of!(Signpost, epoch) == 12);

    assert!(size_of::<KeyRecord>() == 256);
    assert!(size_of::<RosterEntry>() == 256);
    assert!(offset_of!(SignpostPage, key) == 64);
    assert!(offset_of!(SignpostPage, roster) == 512);
    assert!(size_of::<SignpostPage>() == SIGNPOST_BYTES);

    let small = SegmentLayout {
        slot_size: 1,
        slot_count: 1,
        max_subscribers: 1,
    };
    assert!(small.payload_offset().is_multiple_of(PAYLOAD_ALIGN));
    assert!(small.payload_stride().is_multiple_of(PAYLOAD_ALIGN));
    assert!(small.slot_stride().is_multiple_of(CACHE_LINE));
    assert!(small.slots_offset().is_multiple_of(CACHE_LINE));

    let widest = SegmentLayout {
        slot_size: u32::MAX,
        slot_count: MAX_SLOT_COUNT,
        max_subscribers: MAX_SUBSCRIBERS,
    };
    assert!(layout_config_ok(
        widest.slot_size,
        widest.slot_count,
        widest.max_subscribers
    ));
    assert!(widest.total_bytes() < (1 << 50));
    // Every derived region starts on a cache line, so the atomics in it keep their alignment.
    assert!(widest.slots_offset().is_multiple_of(CACHE_LINE));
    assert!(widest.slot_stride().is_multiple_of(CACHE_LINE));
    assert!(widest.index_offset().is_multiple_of(CACHE_LINE));
};
