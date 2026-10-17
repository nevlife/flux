"""Runtime for generated Python adapters. Reader and Writer are flux2/wire.hpp, the runtime C++
adapters use, bound with numpy views; what is left here only hands out per-element objects.
"""

import numpy as np

from ._flux2 import Reader, Tensor, WireError, Writer, _scratch_tensor

__all__ = ["ElemArray", "NullWriter", "Reader", "Tensor", "WireError", "Writer"]


class ElemArray:
    """Element views (Reader.elems) or builders (Writer.alloc_elems) of a record/jagged array."""

    __slots__ = ("_w", "_at", "_n", "_stride", "_cls")

    def __init__(self, writer, at, n, stride, cls):
        self._w, self._at, self._n, self._stride, self._cls = writer, at, n, stride, cls

    def __len__(self):
        return self._n

    def __getitem__(self, i):
        if i < 0:
            i += self._n
        if not 0 <= i < self._n:
            raise IndexError(f"element {i} out of range (length {self._n})")
        return self._cls(self._w, self._at + i * self._stride)

    def __iter__(self):
        for i in range(self._n):
            yield self[i]


class NullWriter:
    """The writer of a Builder whose loan found no free slot, as the C++ Writer with no buffer:
    bad from the start, and every write goes nowhere. alloc hands back a scratch array of the
    size asked for, so filling it the usual way does not fail."""

    __slots__ = ()
    bad = True
    size = 0

    def poison(self):
        pass

    def put(self, off, dtype, value):
        pass

    def block_view(self, off, dtype, count):
        return np.zeros(count, dtype=dtype)

    def alloc(self, off, n, dtype):
        return np.empty(n, dtype=dtype)

    def put_str(self, off, s):
        pass

    def put_strs(self, off, items, count=None):
        pass

    def alloc_elems(self, off, n, stride, align):
        return 0

    def tensor(self, at, dtype, shape):
        return _scratch_tensor(dtype, shape)
