"""flux_gen: turn ROS .msg schemas into flux flat-wire layouts, tiers and fingerprints.

Four stages, each the input of the next: parse a .msg, flatten it to an ordered leaf list,
assign those leaves byte offsets, and emit a C++ header and a Python module that read and write
that layout. The fingerprint is derived from the flattened leaves and emitted into both, so the
two ends agree by construction. See docs/en/message_shapes.en.md for the wire model.
"""

from .dtypes import ROS_PRIMITIVE, SIZE, DType
from .fingerprint import FINGERPRINT_SCHEMA_VERSION, canonical, fingerprint
from . import emit_cpp, emit_py, emit_ros  # noqa: F401
from .flatten import DYNAMIC_COUNT, FlatResult, Leaf, LeafKind, Reject, flatten_message
from .layout import Block, Layout, LayoutError, Placed, build_block, build_layout, walk
from .model import ArrayKind, Field, Message
from .parse import ParseError, Registry, load_dir, parse_msg, resolve

__all__ = [
    "DType",
    "SIZE",
    "ROS_PRIMITIVE",
    "ArrayKind",
    "Field",
    "Message",
    "ParseError",
    "Registry",
    "parse_msg",
    "load_dir",
    "resolve",
    "flatten_message",
    "FlatResult",
    "Leaf",
    "LeafKind",
    "DYNAMIC_COUNT",
    "Reject",
    "fingerprint",
    "canonical",
    "FINGERPRINT_SCHEMA_VERSION",
    "Block",
    "Layout",
    "LayoutError",
    "Placed",
    "build_block",
    "build_layout",
    "walk",
]
