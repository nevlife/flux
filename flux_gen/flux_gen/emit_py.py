"""Emit a Python adapter module for one schema -- the twin of emit_cpp.

Same offsets, same field names, same allocate-in-any-order builder as the C++ header. The surface
differs where the languages do: reads come back as numpy views and lists, and scalar writes are
attribute assignments rather than setter calls.
"""

import keyword

from .dtypes import DType
from .flatten import DYNAMIC_COUNT, LeafKind
from .layout import STAMP_SIZE, elem_class_name as _elem_class

NP_TYPE = {
    DType.BOOL: "np.bool_",
    DType.U8: "np.uint8", DType.I8: "np.int8",
    DType.U16: "np.uint16", DType.I16: "np.int16",
    DType.U32: "np.uint32", DType.I32: "np.int32",
    DType.U64: "np.uint64", DType.I64: "np.int64",
    DType.F16: "np.float16", DType.F32: "np.float32", DType.F64: "np.float64",
}


def ident(name):
    return name + "_" if keyword.iskeyword(name) or name in ("np", "self") else name


def np_type(dtype):
    if dtype not in NP_TYPE:
        raise ValueError(f"no numpy type for {dtype}")
    return NP_TYPE[dtype]


def py_literal(const):
    if const.type_name in ("string", "wstring"):
        return repr(const.value)
    if const.type_name == "bool":
        return "True" if const.value else "False"
    return repr(const.value)


def _constants(layout):
    if not layout.constants:
        return ""
    return "\n" + "\n".join(
        f"{ident(c.name)} = {py_literal(c)}" for c in layout.constants) + "\n"


def _view_body(block, base, out, prefix=(), ind="    "):
    def at(off):
        return f"{base}{off}" if base else str(off)

    for p in block.placed:
        n, o = ident(p.name), p.offset
        out.append("")
        if p.kind == LeafKind.FIXED:
            t = np_type(p.dtype)
            expr = (f"self._r.get({at(o)}, {t})" if p.count == 1
                    else f"self._r.block({at(o)}, {t}, {p.count})")
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    return {expr}")
        elif p.kind == LeafKind.COLUMN:
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    return self._r.span({at(o)}, {np_type(p.dtype)})")
        elif p.kind == LeafKind.STRING:
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    return self._r.string({at(o)})")
        elif p.kind == LeafKind.STRING_ARRAY:
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    return self._r.strings({at(o)})")
        elif p.kind in (LeafKind.STAMP, LeafKind.HEADER):
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}__stamp(self):")
            out.append(f"{ind}    return (int(self._r.get({at(o)}, np.int32)), "
                       f"int(self._r.get({at(o + 4)}, np.uint32)))")
            if p.kind == LeafKind.HEADER:
                out.append("")
                out.append(f"{ind}@property")
                out.append(f"{ind}def {n}__frame_id(self):")
                out.append(f"{ind}    return self._r.string({at(o + STAMP_SIZE)})")
        else:  # RECORD_COLUMN / JAGGED
            cls = _elem_class(prefix + (p,))
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    return ElemSeq(self._r, {at(o)}, {p.elem.stride}, "
                       f"{p.elem.align}, {cls})")


def _builder_body(block, base, out, prefix=(), ind="    "):
    def at(off):
        return f"{base}{off}" if base else str(off)

    for p in block.placed:
        n, o = ident(p.name), p.offset
        out.append("")
        if p.kind == LeafKind.FIXED and p.count == 1:
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    raise AttributeError('{n} is write-only on a Builder')")
            out.append("")
            out.append(f"{ind}@{n}.setter")
            out.append(f"{ind}def {n}(self, v):")
            out.append(f"{ind}    self._w.put({at(o)}, {np_type(p.dtype)}, v)")
        elif p.kind == LeafKind.FIXED:
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    return self._w.block_view({at(o)}, {np_type(p.dtype)}, "
                       f"{p.count})")
        elif p.kind == LeafKind.COLUMN:
            out.append(f"{ind}def alloc__{n}(self, n):")
            out.append(f"{ind}    return self._w.alloc({at(o)}, n, {np_type(p.dtype)})")
        elif p.kind == LeafKind.STRING:
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    raise AttributeError('{n} is write-only on a Builder')")
            out.append("")
            out.append(f"{ind}@{n}.setter")
            out.append(f"{ind}def {n}(self, s):")
            out.append(f"{ind}    self._w.put_str({at(o)}, s)")
        elif p.kind == LeafKind.STRING_ARRAY:
            out.append(f"{ind}@property")
            out.append(f"{ind}def {n}(self):")
            out.append(f"{ind}    raise AttributeError('{n} is write-only on a Builder')")
            out.append("")
            out.append(f"{ind}@{n}.setter")
            out.append(f"{ind}def {n}(self, items):")
            if p.count != DYNAMIC_COUNT:
                out.append(f"{ind}    items = list(items)")
                out.append(f"{ind}    if len(items) != {p.count}:")
                out.append(f"{ind}        raise ValueError('{n} expects exactly {p.count} "
                           f"strings, got %d' % len(items))")
            out.append(f"{ind}    self._w.put_strs({at(o)}, items)")
        elif p.kind in (LeafKind.STAMP, LeafKind.HEADER):
            out.append(f"{ind}def set__{n}__stamp(self, sec, nanosec):")
            out.append(f"{ind}    self._w.put({at(o)}, np.int32, sec)")
            out.append(f"{ind}    self._w.put({at(o + 4)}, np.uint32, nanosec)")
            if p.kind == LeafKind.HEADER:
                out.append("")
                out.append(f"{ind}@property")
                out.append(f"{ind}def {n}__frame_id(self):")
                out.append(f"{ind}    raise AttributeError('{n}__frame_id is write-only')")
                out.append("")
                out.append(f"{ind}@{n}__frame_id.setter")
                out.append(f"{ind}def {n}__frame_id(self, s):")
                out.append(f"{ind}    self._w.put_str({at(o + STAMP_SIZE)}, s)")
        else:  # RECORD_COLUMN / JAGGED
            cls = _elem_class(prefix + (p,))
            out.append(f"{ind}def alloc__{n}(self, n):")
            if p.count != DYNAMIC_COUNT:
                out.append(f"{ind}    if n != {p.count}:")
                out.append(f"{ind}        raise ValueError('{n} expects exactly {p.count} "
                           f"elements, got %d' % n)")
            out.append(f"{ind}    at = self._w.alloc_elems({at(o)}, n, {p.elem.stride}, "
                       f"{p.elem.align})")
            out.append(f"{ind}    return ElemArray(self._w, at, n, {p.elem.stride}, {cls}Builder)")


def _elem_classes(block, out, prefix=()):
    for p in block.placed:
        if p.elem is None:
            continue
        path = prefix + (p,)
        _elem_classes(p.elem, out, path)
        cls = _elem_class(path)
        body = []
        _view_body(p.elem, "self._at + ", body, path)
        out.append(f'''

class {cls}:
    """One element of `{'.'.join(x.name for x in path)}`."""

    __slots__ = ("_r", "_at")
    STRIDE = {p.elem.stride}
    ALIGN = {p.elem.align}

    def __init__(self, reader, at):
        self._r, self._at = reader, at
{chr(10).join(body)}''')
        bbody = []
        _builder_body(p.elem, "self._at + ", bbody, path)
        out.append(f'''

class {cls}Builder:
    __slots__ = ("_w", "_at")
    STRIDE = {p.elem.stride}
    ALIGN = {p.elem.align}

    def __init__(self, writer, at):
        self._w, self._at = writer, at
{chr(10).join(bbody)}''')


def emit(layout, source):
    """Return the Python module text for a Layout."""
    if layout.rejected:
        raise ValueError(f"{layout.type_name}: {layout.rejected}")

    elems = []
    _elem_classes(layout.root, elems)
    view, builder = [], []
    _view_body(layout.root, "", view)
    _builder_body(layout.root, "", builder)

    return f'''"""{layout.type_name} as a flux flat frame ({layout.tier} tier).

generated by flux_gen from {source} -- do not edit.
"""

import numpy as np

from flux_gen.wire import ElemArray, ElemSeq, Reader, Writer

TYPE_NAME__ = "{layout.type_name}"
FINGERPRINT__ = 0x{layout.fingerprint:016X}
SCALAR_BYTES__ = {layout.scalar_bytes}
{_constants(layout)}{"".join(elems)}


class View:
    """Read side. Aliases the frame: valid only while the flux view it came from is alive."""

    __slots__ = ("_r",)

    def __init__(self, frame):
        self._r = Reader(frame)
{chr(10).join(view)}


class Builder:
    """Write side over a loaned slot: fields are written straight into shared memory."""

    __slots__ = ("_w", "_loan")

    def __init__(self, buf, loan=None):
        self._w = Writer(buf, SCALAR_BYTES__)
        self._loan = loan

    @property
    def size__(self):
        return self._w.size
{chr(10).join(builder)}

    def commit__(self):
        """Publish what has been written, as a flux.Published. The engine sees one opaque byte
        run, so the frame is stamped as u8[size]; the schema lives in the fingerprint, not in
        the frame metadata. flux.faulted(p) separates a dropped frame from a fault."""
        if self._loan is None:
            # A caller's own buffer: nothing to publish. Imported here rather than at module
            # domain so a Builder over a plain bytearray needs no flux runtime at all.
            from flux import Published

            return Published.TooLarge
        return self._loan.commit(nbytes=self._w.size)


def build__(publisher):
    """Loan a slot from `publisher` and return a Builder over it, or None if none is free."""
    loan = publisher.loan(publisher.slot_size, dtype="uint8")
    return None if loan is None else Builder(loan.array, loan)


class {layout.name}:
    """Namespace mirroring the C++ `{layout.package or "flux_msg"}::flux_msg::{layout.name}`."""

    TYPE_NAME__ = TYPE_NAME__
    FINGERPRINT__ = FINGERPRINT__
    SCALAR_BYTES__ = SCALAR_BYTES__
    View = View
    Builder = Builder
    build__ = staticmethod(build__)
{"".join(f"    {ident(c.name)} = {ident(c.name)}" + chr(10) for c in layout.constants)}
'''
