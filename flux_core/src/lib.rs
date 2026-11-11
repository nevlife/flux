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
mod layout;
mod manifest;
mod partitioned;
mod qos;
mod reader;
mod segment;
#[cfg(test)]
mod storm;
mod sys;
pub mod trace;

pub use channel::{
    Channel, FrameView, GpuBlock, GpuHeap, Heap, Published, PublisherWatch, Refused, ReleaseMode,
    WriteSlot,
};
pub use discovery::{flatten_key, signpost_name};
pub use error::{Error, Result};
pub use executor::{DEFAULT_TICK_NS, DEFAULT_TURN_NS, Executor, ExecutorHandle, MAX_DRAIN, Source};
pub use layout::{
    DEFAULT_MAX_SUBSCRIBERS, DEFAULT_SLOT_COUNT, DEFAULT_SLOT_SIZE, DType, FrameMeta, MAX_AUX_SIZE,
    MAX_DIMS, MAX_LABEL_LEN, MAX_SUBSCRIBERS, NO_SCHEMA, ROSTER_ENTRIES,
};
pub use manifest::{
    Announcement, ChannelStats, Endpoint, ManifestEntry, Topic, enumerate_topics,
    read_channel_stats,
};
pub use partitioned::PartitionedExecutor;
pub use qos::{Durability, MemoryPolicy, QoS};
pub use segment::Mapped;
