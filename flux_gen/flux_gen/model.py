"""Parsed .msg model: a Message is an ordered list of Fields, each with an array kind."""

from dataclasses import dataclass, field
from enum import Enum


class ArrayKind(Enum):
    SCALAR = "scalar"    # T name
    FIXED = "fixed"      # T[N] name
    BOUNDED = "bounded"  # T[<=N] name  (rejected: never rides the fast path)
    DYNAMIC = "dynamic"  # T[] name


@dataclass(frozen=True)
class Field:
    name: str
    type_name: str  # raw base token: primitive, "string"/"wstring", or a message name
    kind: ArrayKind
    size: int = 0  # N for FIXED/BOUNDED, else 0


@dataclass(frozen=True)
class Constant:
    """`TYPE NAME=value`. Not wire data -- it is emitted as a named symbol beside the adapter so
    callers write `Msg.ERROR_GNSS` instead of 3, and it never enters the fingerprint."""

    name: str
    type_name: str
    value: object


@dataclass
class Message:
    name: str
    fields: list = field(default_factory=list)
    package: str = None  # owning ROS package; domains bare type references to it
    path: str = None  # source .msg, so a build can depend on every file a schema was read from
    constants: list = field(default_factory=list)
