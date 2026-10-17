//! C ABI over `flux2_core`. The contracts every function shares are stated once, in the header
//! preamble (`cbindgen.toml`), rather than per function.
#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

mod channel;
mod executor;
mod gpu;
mod manifest;
mod schema;
mod status;
mod store;

pub use channel::*;
pub use executor::*;
pub use gpu::*;
pub use manifest::*;
pub use schema::*;
pub use status::*;

// Raised on every change to the C ABI; the soname carries it (`build.rs`, `CMakeLists.txt`).
pub const FLUX2_ABI_VERSION: u32 = 3;

// A caller compares it with its header's FLUX2_ABI_VERSION.
#[unsafe(no_mangle)]
pub extern "C" fn flux2_abi_version() -> u32 {
    FLUX2_ABI_VERSION
}

// Literals, so cbindgen can write them; the asserts tie each to its core value.
pub const FLUX2_MAX_DIMS: usize = 8;
pub const FLUX2_NO_SCHEMA: u64 = 0;
pub const FLUX2_MAX_LABEL_LEN: usize = 240;
pub const FLUX2_MAX_DRAIN: u32 = 64;
pub const FLUX2_DEFAULT_TICK_NS: i64 = 100_000_000;
pub const FLUX2_DEFAULT_MAX_SUBSCRIBERS: u32 = 16;
pub const FLUX2_MAX_SUBSCRIBERS: u32 = 4096;
pub const FLUX2_NO_STAMP: i32 = -1;

const _: () = {
    assert!(FLUX2_MAX_DIMS == flux2_core::MAX_DIMS);
    assert!(FLUX2_NO_SCHEMA == flux2_core::NO_SCHEMA);
    assert!(FLUX2_MAX_LABEL_LEN == flux2_core::MAX_LABEL_LEN);
    assert!(FLUX2_MAX_DRAIN == flux2_core::MAX_DRAIN);
    assert!(FLUX2_DEFAULT_TICK_NS == flux2_core::DEFAULT_TICK_NS);
    assert!(FLUX2_DEFAULT_MAX_SUBSCRIBERS == flux2_core::DEFAULT_MAX_SUBSCRIBERS);
    assert!(FLUX2_MAX_SUBSCRIBERS == flux2_core::MAX_SUBSCRIBERS);
    assert!(FLUX2_NO_STAMP == flux2_core::trace::NO_STAMP);
};
