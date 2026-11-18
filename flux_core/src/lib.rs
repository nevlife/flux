//! flux engine: same-host zero-copy frames over POSIX shm, one publisher per channel.

pub mod arena;
mod channel;
mod discovery;
mod error;
mod executor;
#[cfg(any(test, feature = "fuzz"))]
#[doc(hidden)]
pub mod fuzz;
pub mod gpu;
mod image_format;
mod layout;
mod manifest;
mod partitioned;
mod qos;
mod reader;
mod segment;
mod sizing;
#[cfg(test)]
mod storm;
mod sys;
pub mod trace;

pub use channel::{
    Alloc, Channel, FrameView, GpuBlock, GpuHeap, Heap, ImageSlots, Published, PublisherConfig,
    PublisherWatch, Refused, ReleaseMode, WriteSlot,
};
pub use discovery::{flatten_key, signpost_name};
pub use error::{Error, Result};
pub use executor::{DEFAULT_TICK_NS, DEFAULT_TURN_NS, Executor, ExecutorHandle, MAX_DRAIN, Source};
pub use image_format::{IMAGE_PITCH_ALIGN, ImageEncoding, ImageFormat, ImageLayout};
pub use layout::{
    DEFAULT_MAX_SUBSCRIBERS, DType, FrameMeta, MAX_AUX_SIZE, MAX_DIMS, MAX_LABEL_LEN,
    MAX_SUBSCRIBERS, NO_SCHEMA, ROSTER_ENTRIES,
};
pub use manifest::{
    Announcement, ChannelStats, Endpoint, ManifestEntry, Topic, enumerate_topics,
    read_channel_stats,
};
pub use partitioned::PartitionedExecutor;
pub use qos::{Durability, MemoryPolicy, QoS};
pub use segment::Mapped;
pub use sizing::{
    Bound, DEFAULT_SLOT_COUNT, DEFAULT_SLOT_SIZE, HOST_BOUND, Plan, SLOT_ALLOC_MAX_SLOT_SIZE,
    SLOT_ALLOC_MAX_SLOTS, SLOT_ALLOC_SLOT_SIZE, SLOT_ALLOC_START_SLOTS, SLOT_STRING_BYTES,
    START_SLOTS, UNTYPED_SLOT_SIZE, max_slots, plan, slot_count, slot_size, start_slots,
    typed_slot_size,
};
