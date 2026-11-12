//! C ABI over `flux_core`. The contracts every function shares are stated once, in the header
//! preamble (`cbindgen.toml`), rather than per function.
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

mod arena;
mod channel;
mod executor;
mod gpu;
mod manifest;
mod schema;
mod status;
mod store;

pub use arena::*;
pub use channel::*;
pub use executor::*;
pub use gpu::*;
pub use manifest::*;
pub use schema::*;
pub use status::*;

// Raised on every change to the C ABI; the soname carries it (`build.rs`, `CMakeLists.txt`).
pub const FLUX_ABI_VERSION: u32 = 4;

// A caller compares it with its header's FLUX_ABI_VERSION.
#[unsafe(no_mangle)]
pub extern "C" fn flux_abi_version() -> u32 {
    FLUX_ABI_VERSION
}

// Literals, so cbindgen can write them; the asserts tie each to its core value.
pub const FLUX_MAX_DIMS: usize = 8;
pub const FLUX_NO_SCHEMA: u64 = 0;
pub const FLUX_MAX_LABEL_LEN: usize = 240;
pub const FLUX_MAX_DRAIN: u32 = 64;
pub const FLUX_DEFAULT_TICK_NS: i64 = 100_000_000;
pub const FLUX_DEFAULT_TURN_NS: i64 = 1_000_000;
pub const FLUX_DEFAULT_MAX_SUBSCRIBERS: u32 = 16;
pub const FLUX_DEFAULT_SLOT_SIZE: u32 = 8 << 20;
pub const FLUX_DEFAULT_SLOT_COUNT: u32 = 16;
pub const FLUX_MAX_SUBSCRIBERS: u32 = 4096;
pub const FLUX_MAX_AUX_SIZE: u32 = 32 * 1024;
pub const FLUX_NO_STAMP: i32 = -1;
pub const FLUX_HEAP_BASE: u64 = 0x8000_0000;
pub const FLUX_GPU_HEAP_BASE: u64 = 0xC000_0000;
// Arena descriptor offsets count 16-byte units. Readers use byte positions (GPU arena from POS).
pub const FLUX_ARENA_UNIT: u64 = 16;
pub const FLUX_GPU_HEAP_POS: u64 = 0x4_8000_0000;

const _: () = {
    assert!(FLUX_MAX_DIMS == flux_core::MAX_DIMS);
    assert!(FLUX_NO_SCHEMA == flux_core::NO_SCHEMA);
    assert!(FLUX_MAX_LABEL_LEN == flux_core::MAX_LABEL_LEN);
    assert!(FLUX_MAX_DRAIN == flux_core::MAX_DRAIN);
    assert!(FLUX_DEFAULT_TICK_NS == flux_core::DEFAULT_TICK_NS);
    assert!(FLUX_DEFAULT_TURN_NS == flux_core::DEFAULT_TURN_NS);
    assert!(FLUX_DEFAULT_MAX_SUBSCRIBERS == flux_core::DEFAULT_MAX_SUBSCRIBERS);
    assert!(FLUX_DEFAULT_SLOT_SIZE == flux_core::DEFAULT_SLOT_SIZE);
    assert!(FLUX_DEFAULT_SLOT_COUNT == flux_core::DEFAULT_SLOT_COUNT);
    assert!(FLUX_MAX_SUBSCRIBERS == flux_core::MAX_SUBSCRIBERS);
    assert!(FLUX_MAX_AUX_SIZE == flux_core::MAX_AUX_SIZE);
    assert!(FLUX_NO_STAMP == flux_core::trace::NO_STAMP);
    assert!(FLUX_HEAP_BASE == flux_core::arena::HEAP_BASE);
    assert!(FLUX_GPU_HEAP_BASE == flux_core::arena::GPU_BASE);
    assert!(FLUX_ARENA_UNIT == flux_core::arena::UNIT);
    assert!(FLUX_GPU_HEAP_POS == flux_core::arena::GPU_POS);
    assert!(flux_gen::wire::HEAP_POS as u64 == flux_core::arena::HEAP_POS);
    assert!(flux_gen::wire::GPU_POS as u64 == flux_core::arena::GPU_POS);
    assert!(flux_gen::wire::UNIT as u64 == flux_core::arena::UNIT);
};
