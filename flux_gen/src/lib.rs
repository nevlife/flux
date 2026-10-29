//! `flux_gen`: parse a .msg, flatten it to leaves, place them at byte offsets, and emit the C++
//! and Python adapters of that layout with one fingerprint, so the two ends agree by construction.

pub mod ament;
pub mod cdr;
pub mod dtypes;
pub mod emit_cpp;
pub mod emit_py;
pub mod emit_ros;
pub mod fingerprint;
pub mod flatten;
#[cfg(any(test, feature = "fuzz"))]
pub mod fuzz;
pub mod layout;
pub mod model;
pub mod parse;
pub mod text;
pub mod wire;
