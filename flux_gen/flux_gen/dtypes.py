"""Primitive dtypes for the flat wire, and the ROS .msg type map.

Mirrors flux_core's DType (segment_layout.hpp) except at two points. BOOL has no counterpart
there: a bool leaf is a schema-level shape that flatten uses (and rejects as bool[]), not a frame
dtype the engine ever carries. BF16 goes the other way -- the engine carries it, but no ROS .msg
type maps to it, so a schema can never name one. Values are the tokens the fingerprint is
hashed from, so changing one changes every fingerprint -- bump FINGERPRINT_SCHEMA_VERSION with it.
"""

from enum import Enum


class DType(Enum):
    BOOL = "bool"
    U8 = "u8"
    I8 = "i8"
    U16 = "u16"
    I16 = "i16"
    U32 = "u32"
    I32 = "i32"
    U64 = "u64"
    I64 = "i64"
    F16 = "f16"
    F32 = "f32"
    F64 = "f64"


SIZE = {
    DType.BOOL: 1,
    DType.U8: 1,
    DType.I8: 1,
    DType.U16: 2,
    DType.I16: 2,
    DType.U32: 4,
    DType.I32: 4,
    DType.U64: 8,
    DType.I64: 8,
    DType.F16: 2,
    DType.F32: 4,
    DType.F64: 8,
}

# ROS 2 .msg primitive type name -> flux DType. string/wstring are not here; they are a
# variable tail, handled separately in flatten.
ROS_PRIMITIVE = {
    "bool": DType.BOOL,
    "byte": DType.U8,
    "char": DType.U8,
    "int8": DType.I8,
    "uint8": DType.U8,
    "int16": DType.I16,
    "uint16": DType.U16,
    "int32": DType.I32,
    "uint32": DType.U32,
    "int64": DType.I64,
    "uint64": DType.U64,
    "float32": DType.F32,
    "float64": DType.F64,
}
