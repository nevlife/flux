//! A Python adapter module for one schema, the twin of the C++ header.
//!
//! Same offsets, same field names, same allocate-in-any-order builder. Reads come back as numpy
//! views and lists, and scalar writes are attribute assignments rather than setter calls.

use std::fmt::Write;

use crate::dtypes::DType;
use crate::emit_cpp::at;
use crate::flatten::{DYNAMIC_COUNT, LeafKind};
use crate::layout::{Block, Layout, Placed, elem_class_name};
use crate::model::{ConstValue, Constant};
use crate::text::{py_float, py_ident, py_str};

pub fn np_type(dtype: DType) -> &'static str {
    match dtype {
        DType::Bool => "np.bool_",
        DType::U8 => "np.uint8",
        DType::I8 => "np.int8",
        DType::U16 => "np.uint16",
        DType::I16 => "np.int16",
        DType::U32 => "np.uint32",
        DType::I32 => "np.int32",
        DType::U64 => "np.uint64",
        DType::I64 => "np.int64",
        DType::F32 => "np.float32",
        DType::F64 => "np.float64",
    }
}

fn py_literal(c: &Constant) -> String {
    match &c.value {
        ConstValue::Str(s) => py_str(s),
        ConstValue::Bool(b) => if *b { "True" } else { "False" }.into(),
        ConstValue::Int(i) => i.to_string(),
        // repr() spells these as bare names, which Python does not define.
        ConstValue::Float(f) if f.is_nan() => "float('nan')".into(),
        ConstValue::Float(f) if f.is_infinite() => if *f < 0.0 {
            "-float('inf')"
        } else {
            "float('inf')"
        }
        .into(),
        ConstValue::Float(f) => py_float(*f),
    }
}

fn constants(layout: &Layout) -> String {
    if layout.constants.is_empty() {
        return String::new();
    }
    let lines: Vec<String> = layout
        .constants
        .iter()
        .map(|c| format!("{} = {}", py_ident(&c.name), py_literal(c)))
        .collect();
    format!("\n{}\n", lines.join("\n"))
}

const IND: &str = "    ";

fn dtype(p: &Placed) -> &'static str {
    np_type(p.leaf.dtype.expect("fixed leaves and columns have a dtype"))
}

fn property(out: &mut Vec<String>, name: &str, expr: &str) {
    out.push(format!("{IND}@_property"));
    out.push(format!("{IND}def {name}(self):"));
    out.push(format!("{IND}    return {expr}"));
}

fn view_body(block: &Block, base: &str, out: &mut Vec<String>, prefix: &[&Placed]) {
    for p in &block.placed {
        let n = py_ident(&p.name());
        let o = p.offset;
        out.push(String::new());
        match p.kind() {
            LeafKind::Fixed if p.leaf.count == 1 => {
                property(
                    out,
                    &n,
                    &format!("self._r.get({}, {})", at(base, o), dtype(p)),
                );
            }
            LeafKind::Fixed => {
                let expr = format!(
                    "self._r.block({}, {}, {})",
                    at(base, o),
                    dtype(p),
                    p.leaf.count
                );
                property(out, &n, &expr);
            }
            LeafKind::Column => {
                property(
                    out,
                    &n,
                    &format!("self._r.span({}, {})", at(base, o), dtype(p)),
                );
            }
            LeafKind::String => property(out, &n, &format!("self._r.string({})", at(base, o))),
            LeafKind::StringArray => {
                property(out, &n, &format!("self._r.strings({})", at(base, o)));
            }
            LeafKind::Stamp | LeafKind::Header => {
                let expr = format!(
                    "(int(self._r.get({}, np.int32)), int(self._r.get({}, np.uint32)))",
                    at(base, o),
                    at(base, o + 4)
                );
                property(out, &format!("{n}__stamp"), &expr);
                if p.kind() == LeafKind::Header {
                    out.push(String::new());
                    let expr = format!("self._r.string({})", at(base, p.frame_id_offset()));
                    property(out, &format!("{n}__frame_id"), &expr);
                }
            }
            LeafKind::RecordColumn | LeafKind::Jagged => {
                let mut path = prefix.to_vec();
                path.push(p);
                let e = p.elem();
                let expr = format!(
                    "_ElemArray(self._r, *self._r.elems({}, {}, {}), {}, {})",
                    at(base, o),
                    e.stride,
                    e.align,
                    e.stride,
                    elem_class_name(&path)
                );
                property(out, &n, &expr);
            }
        }
    }
}

fn setter(out: &mut Vec<String>, name: &str, arg: &str, body: &[String]) {
    out.push(format!("{IND}def _set__{name}(self, {arg}):"));
    out.extend(body.iter().map(|l| format!("{IND}    {l}")));
    out.push(format!("{IND}{name} = _property(fset=_set__{name})"));
}

fn builder_body(block: &Block, base: &str, out: &mut Vec<String>, prefix: &[&Placed]) {
    for p in &block.placed {
        let n = py_ident(&p.name());
        let o = p.offset;
        out.push(String::new());
        match p.kind() {
            LeafKind::Fixed if p.leaf.count == 1 => {
                let put = format!("self._w.put({}, {}, v)", at(base, o), dtype(p));
                setter(out, &n, "v", &[put]);
            }
            LeafKind::Fixed => {
                let expr = format!(
                    "self._w.block_view({}, {}, {})",
                    at(base, o),
                    dtype(p),
                    p.leaf.count
                );
                property(out, &n, &expr);
            }
            LeafKind::Column => {
                out.push(format!("{IND}def alloc__{n}(self, n):"));
                out.push(format!(
                    "{IND}    return self._w.alloc({}, n, {})",
                    at(base, o),
                    dtype(p)
                ));
            }
            LeafKind::String => {
                setter(
                    out,
                    &n,
                    "s",
                    &[format!("self._w.put_str({}, s)", at(base, o))],
                );
            }
            LeafKind::StringArray => {
                // A fixed string[N] passes N: the Writer refuses another length and latches bad.
                let count = if p.leaf.count == DYNAMIC_COUNT {
                    String::new()
                } else {
                    format!(", {}", p.leaf.count)
                };
                let put = format!("self._w.put_strs({}, items{count})", at(base, o));
                setter(out, &n, "items", &[put]);
            }
            LeafKind::Stamp | LeafKind::Header => {
                out.push(format!("{IND}def set__{n}__stamp(self, sec, nanosec):"));
                out.push(format!(
                    "{IND}    self._w.put({}, np.int32, sec)",
                    at(base, o)
                ));
                out.push(format!(
                    "{IND}    self._w.put({}, np.uint32, nanosec)",
                    at(base, o + 4)
                ));
                if p.kind() == LeafKind::Header {
                    out.push(String::new());
                    let put = format!("self._w.put_str({}, s)", at(base, p.frame_id_offset()));
                    setter(out, &format!("{n}__frame_id"), "s", &[put]);
                }
            }
            LeafKind::RecordColumn | LeafKind::Jagged => {
                let mut path = prefix.to_vec();
                path.push(p);
                let cls = elem_class_name(&path);
                let e = p.elem();
                // A fixed T[N] takes no count, as in C++: N is the schema's.
                let fixed = p.leaf.count != DYNAMIC_COUNT;
                out.push(format!(
                    "{IND}def alloc__{n}(self{}):",
                    if fixed { "" } else { ", n" }
                ));
                if fixed {
                    out.push(format!("{IND}    n = {}", p.leaf.count));
                }
                out.push(format!(
                    "{IND}    at = self._w.alloc_elems({}, n, {}, {})",
                    at(base, o),
                    e.stride,
                    e.align
                ));
                out.push(format!(
                    "{IND}    return _ElemArray(self._w, at, n, {}, {cls}Builder)",
                    e.stride
                ));
            }
        }
    }
}

fn elem_classes(block: &Block, out: &mut String, prefix: &[&Placed]) {
    for p in &block.placed {
        let Some(e) = &p.elem else { continue };
        let mut path = prefix.to_vec();
        path.push(p);
        elem_classes(e, out, &path);
        let cls = elem_class_name(&path);
        let dotted = path.iter().map(|x| x.name()).collect::<Vec<_>>().join(".");
        let (stride, align) = (e.stride, e.align);
        let mut body = Vec::new();
        view_body(e, "self._at + ", &mut body, &path);
        let body = body.join("\n");
        write!(
            out,
            r#"

class {cls}:
    """One element of `{dotted}`."""

    __slots__ = ("_r", "_at")
    STRIDE = {stride}
    ALIGN = {align}

    def __init__(self, reader, at):
        self._r, self._at = reader, at
{body}"#
        )
        .expect("writing to a String");
        let mut bbody = Vec::new();
        builder_body(e, "self._at + ", &mut bbody, &path);
        let bbody = bbody.join("\n");
        write!(
            out,
            r#"

class {cls}Builder:
    __slots__ = ("_w", "_at")
    STRIDE = {stride}
    ALIGN = {align}

    def __init__(self, writer, at):
        self._w, self._at = writer, at
{bbody}"#
        )
        .expect("writing to a String");
    }
}

const VIEW_INIT: &str = r#"    __slots__ = ("_r",)

    def __init__(self, frame):
        self._r = _Reader(frame)"#;

/// A tensor message's View also takes a CUDA Frame: its fields are read through the host
/// mapping, and `data` goes out on the device.
const TENSOR_VIEW_INIT: &str = r#"    __slots__ = ("_r", "_frame")

    def __init__(self, frame):
        self._frame = frame if isinstance(frame, _Frame) else None
        self._r = _Reader(frame if self._frame is None else frame._host())"#;

/// The module constant, View methods and Builder method of a tensor message; empty for the rest.
fn tensor(layout: &Layout) -> (String, String, String) {
    let Some(at) = layout.tensor_at() else {
        return Default::default();
    };
    let at = at.map(|o| o.to_string()).join(", ");
    let view = r#"

    def as_tensor__(self):
        """`data` as the tensor the other fields describe, without a copy: a numpy view, or for a
        dtype numpy has no name for (bfloat16, float8) a flux.wire.Tensor, read through .bits or
        __dlpack__. WireError if the fields put an element outside `data` or off its alignment."""
        return self._r.tensor(TENSOR_AT__)

    def __dlpack__(self, **kwargs):
        if self._frame is None:
            return self._r.tensor(TENSOR_AT__, False).__dlpack__()
        return self._frame._tensor_dlpack(TENSOR_AT__, **kwargs)

    def __dlpack_device__(self):
        return (1, 0) if self._frame is None else self._frame.__dlpack_device__()"#;
    let builder = r#"

    def alloc_tensor__(self, dtype, shape):
        """Lay `data` out as a C-contiguous tensor of `shape` on a 64-byte boundary and write the
        fields that describe it. Returns it writable, as View.as_tensor__() hands it out."""
        return self._w.tensor(TENSOR_AT__, dtype, shape)"#;
    (
        format!("\nTENSOR_AT__ = ({at})"),
        view.into(),
        builder.into(),
    )
}

/// The Python module text for a layout.
pub fn emit(layout: &Layout, source: &str) -> String {
    let mut elems = String::new();
    elem_classes(&layout.root, &mut elems, &[]);
    let (mut view, mut builder) = (Vec::new(), Vec::new());
    view_body(&layout.root, "", &mut view, &[]);
    builder_body(&layout.root, "", &mut builder, &[]);
    let (tensor_at, tensor_view, tensor_builder) = tensor(layout);
    let (view_init, frame_import) = if tensor_at.is_empty() {
        (VIEW_INIT, "")
    } else {
        (TENSOR_VIEW_INIT, "\nfrom flux import Frame as _Frame")
    };
    let view = view.join("\n") + &tensor_view;
    let builder = builder.join("\n") + &tensor_builder;
    let consts = constants(layout);
    let aliases = layout.constants.iter().fold(String::new(), |mut s, c| {
        writeln!(s, "    {0} = {0}", py_ident(&c.name)).expect("writing to a String");
        s
    });
    let (name, type_name, tier) = (&layout.name, layout.type_name(), layout.tier);
    let fp = layout.fingerprint;
    let scalar = layout.scalar_bytes();
    let stamp = layout.stamp_offset();
    let register = if stamp < 0 {
        ""
    } else {
        "\n_register_stamp_offset(FINGERPRINT__, STAMP_OFFSET__)"
    };
    let ns = layout.package.as_deref().unwrap_or("flux_msg");
    format!(
        r#""""{type_name} as a flux flat frame ({tier} tier).

generated by flux_gen from {source} -- do not edit.
"""

import numpy as np

from flux import Published as _Published
from flux import _register_stamp_offset{frame_import}
from flux.wire import ElemArray as _ElemArray, NullWriter as _NullWriter
from flux.wire import Reader as _Reader, Writer as _Writer

# Private names, so that no field or message name can rebind what the classes below call.
_property = property

TYPE_NAME__ = "{type_name}"
FINGERPRINT__ = 0x{fp:016X}
SCALAR_BYTES__ = {scalar}
STAMP_OFFSET__ = {stamp}{register}{tensor_at}
{consts}{elems}


class View:
    """Read side. Aliases the frame: valid only while the flux view it came from is alive."""

{view_init}
{view}


class Builder:
    """Write side over a loaned slot: fields are written straight into shared memory."""

    __slots__ = ("_w", "_loan")

    def __init__(self, buf, loan=None):
        # No buffer is a build__ that found no free slot: writes go nowhere, as in C++.
        self._w = _Writer(buf, SCALAR_BYTES__) if buf is not None else _NullWriter()
        self._loan = loan

    def __bool__(self):
        """False with no slot or once a field did not fit; commit__() then refuses."""
        return not self._w.bad

    @_property
    def size__(self):
        return self._w.size
{builder}

    def commit__(self):
        """Publish what has been written, as a flux.Published. The engine sees one opaque byte
        run, so the frame is stamped as u8[size]; the schema lives in the fingerprint, not in
        the frame metadata. flux.faulted(p) separates a dropped frame from a fault."""
        if self._loan is None:
            if isinstance(self._w, _NullWriter):
                return _Published.BACKPRESSURE  # the loan never happened
            return _Published.TOO_LARGE  # a caller's own buffer: nothing to publish
        if self._w.bad:
            return _Published.TOO_LARGE  # a write failed: the frame is torn, as C++ refuses it
        p = self._loan.commit(nbytes=self._w.size)
        self._w, self._loan = _NullWriter(), None  # the slot is the subscribers' now
        return p


def build__(publisher):
    """Loan a slot from `publisher` and return a Builder over it. With no free slot the Builder
    is false, writes into it go nowhere, and commit__() returns BACKPRESSURE, as in C++."""
    loan = publisher.loan(dtype="uint8")
    return Builder(None) if loan is None else Builder(loan.array, loan)


class {name}:
    """Namespace mirroring the C++ `{ns}::flux_msg::{name}`."""

    TYPE_NAME__ = TYPE_NAME__
    FINGERPRINT__ = FINGERPRINT__
    SCALAR_BYTES__ = SCALAR_BYTES__
    STAMP_OFFSET__ = STAMP_OFFSET__
    View = View
    Builder = Builder
    build__ = staticmethod(build__)
{aliases}
"#
    )
}
