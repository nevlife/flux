//! Primitive dtypes for the flat wire, and the ROS .msg type map.
//!
//! Mirrors flux2's `DType` (`flux2_core` layout.rs) except `Bool` (a schema shape flatten uses,
//! never a frame dtype), F16 and BF16 (no .msg type maps to them). `token()` values are fingerprint
//! tokens: changing one changes every fingerprint, so bump `FINGERPRINT_SCHEMA_VERSION` with it.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DType {
    Bool,
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    I64,
    F32,
    F64,
}

impl DType {
    pub fn token(self) -> &'static str {
        match self {
            DType::Bool => "bool",
            DType::U8 => "u8",
            DType::I8 => "i8",
            DType::U16 => "u16",
            DType::I16 => "i16",
            DType::U32 => "u32",
            DType::I32 => "i32",
            DType::U64 => "u64",
            DType::I64 => "i64",
            DType::F32 => "f32",
            DType::F64 => "f64",
        }
    }

    pub fn size(self) -> usize {
        match self {
            DType::Bool | DType::U8 | DType::I8 => 1,
            DType::U16 | DType::I16 => 2,
            DType::U32 | DType::I32 | DType::F32 => 4,
            DType::U64 | DType::I64 | DType::F64 => 8,
        }
    }
}

/// string/wstring are a variable tail, handled in flatten.
pub fn ros_primitive(name: &str) -> Option<DType> {
    Some(match name {
        "bool" => DType::Bool,
        "byte" | "char" | "uint8" => DType::U8,
        "int8" => DType::I8,
        "int16" => DType::I16,
        "uint16" => DType::U16,
        "int32" => DType::I32,
        "uint32" => DType::U32,
        "int64" => DType::I64,
        "uint64" => DType::U64,
        "float32" => DType::F32,
        "float64" => DType::F64,
        _ => return None,
    })
}
