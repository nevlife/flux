"""Runtime for generated Python adapters: the mirror of flux_core's flux/wire.hpp.

Same frame layout, same offsets, same descriptors -- both ends are generated from one tree, so
the only thing that has to match by hand is this file and its C++ twin. Reads hand back numpy
views that alias the slot; nothing is copied out.

Where C++ latches a bad() flag (its callers are noexcept), this raises WireError. A frame is
written by another process, so every descriptor is still bounds-checked before it is followed.
"""

import numpy as np

DESC_SIZE = 8
DESC_ALIGN = 4
MAX_FRAME_BYTES = 0xFFFFFFFF

_DESC = np.dtype([("off", np.uint32), ("len", np.uint32)])

# flux_gen.DType.value -> numpy dtype. Native byte order: the two ends are the same host.
NUMPY_DTYPE = {
    "u8": np.uint8, "i8": np.int8,
    "u16": np.uint16, "i16": np.int16,
    "u32": np.uint32, "i32": np.int32,
    "u64": np.uint64, "i64": np.int64,
    "f16": np.float16, "f32": np.float32, "f64": np.float64,
}


class WireError(Exception):
    pass


def _align_up(n, a):
    return (n + a - 1) // a * a


def _readonly(arr):
    # A received frame is borrowed, not owned: writing through it would corrupt a live frame
    # that other subscribers are reading.
    arr.flags.writeable = False
    return arr


class Reader:
    """Read side of a received frame. Construct over a read-only buffer (a flux view)."""

    __slots__ = ("_buf", "_n")

    def __init__(self, buf):
        self._buf = memoryview(buf).cast("B")
        self._n = len(self._buf)

    def __len__(self):
        return self._n

    @property
    def buffer(self):
        return self._buf

    def _region(self, off, n, esize, align):
        if off % align or off > self._n or n > (self._n - off) // esize:
            raise WireError(f"frame region [{off}, +{n}x{esize}) escapes a {self._n}-byte frame")

    def get(self, off, dtype):
        """One fixed value at a compile-time offset."""
        dt = np.dtype(dtype)
        self._region(off, 1, dt.itemsize, 1)
        return np.frombuffer(self._buf, dtype=dt, count=1, offset=off)[0]

    def block(self, off, dtype, count):
        """A fixed T[N] block, as a view."""
        dt = np.dtype(dtype)
        self._region(off, count, dt.itemsize, 1)
        return _readonly(np.frombuffer(self._buf, dtype=dt, count=count, offset=off))

    def desc(self, off):
        self._region(off, 1, DESC_SIZE, 1)
        d = np.frombuffer(self._buf, dtype=_DESC, count=1, offset=off)[0]
        return int(d["off"]), int(d["len"])

    def length(self, off):
        return self.desc(off)[1]

    def span(self, off, dtype):
        """A variable array, as a view aliasing the frame."""
        dt = np.dtype(dtype)
        at, n = self.desc(off)
        self._region(at, n, dt.itemsize, dt.itemsize)
        return _readonly(np.frombuffer(self._buf, dtype=dt, count=n, offset=at))

    def string(self, off):
        at, n = self.desc(off)
        self._region(at, n, 1, 1)
        return str(self._buf[at:at + n], "utf-8")

    def strings(self, off):
        at, n = self.desc(off)
        self._region(at, n, DESC_SIZE, DESC_ALIGN)
        return [self.string(at + i * DESC_SIZE) for i in range(n)]

    def elem(self, off, i, stride, align):
        """Frame offset of element i of a record/jagged array."""
        at, n = self.desc(off)
        self._region(at, n, stride, align)
        if not 0 <= i < n:
            raise WireError(f"element {i} out of range (length {n})")
        return at + i * stride

    def elems(self, off, stride, align):
        at, n = self.desc(off)
        self._region(at, n, stride, align)
        return at, n


class ElemSeq:
    """Elements of a record/jagged array: len(), indexing and iteration over element views."""

    __slots__ = ("_reader", "_at", "_n", "_stride", "_cls")

    def __init__(self, reader, off, stride, align, cls):
        self._reader = reader
        self._at, self._n = reader.elems(off, stride, align)
        self._stride, self._cls = stride, cls

    def __len__(self):
        return self._n

    def __getitem__(self, i):
        if isinstance(i, slice):
            return [self[k] for k in range(*i.indices(self._n))]
        if i < 0:
            i += self._n
        if not 0 <= i < self._n:
            raise IndexError(f"element {i} out of range (length {self._n})")
        return self._cls(self._reader, self._at + i * self._stride)

    def __iter__(self):
        for i in range(self._n):
            yield self[i]


class ElemArray:
    """Builders for the elements of a record/jagged array, as reserved by Writer.alloc_elems."""

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


class Writer:
    """Write side: a bump allocator over a loaned slot."""

    __slots__ = ("_buf", "_cap", "_used")

    def __init__(self, buf, scalar_bytes):
        self._buf = memoryview(buf).cast("B")
        self._cap = len(self._buf)
        if scalar_bytes > self._cap:
            raise WireError(f"scalar block ({scalar_bytes}B) exceeds the slot ({self._cap}B)")
        # The scalar block must start clean: a descriptor its writer never sets would otherwise
        # be last frame's, and point at bytes this frame does not own.
        self._buf[:scalar_bytes] = b"\0" * scalar_bytes
        self._used = scalar_bytes

    @property
    def size(self):
        """Bytes to commit."""
        return self._used

    @property
    def buffer(self):
        return self._buf

    def put(self, off, dtype, value):
        dt = np.dtype(dtype)
        if off + dt.itemsize > self._cap:
            raise WireError(f"scalar at {off} escapes the slot")
        np.frombuffer(self._buf, dtype=dt, count=1, offset=off)[0] = value

    def put_block(self, off, dtype, values):
        dst = self.block_view(off, dtype, len(values))
        dst[:] = values

    def block_view(self, off, dtype, count):
        dt = np.dtype(dtype)
        if off + count * dt.itemsize > self._cap:
            raise WireError(f"block at {off} escapes the slot")
        return np.frombuffer(self._buf, dtype=dt, count=count, offset=off)

    def _reserve(self, off, n, esize, align, zero):
        if off + DESC_SIZE > self._cap:
            raise WireError(f"descriptor at {off} escapes the slot")
        start = _align_up(self._used, align)
        if start > self._cap or n > (self._cap - start) // esize:
            raise WireError(
                f"frame needs {start + n * esize}B but the slot holds {self._cap}B")
        if start + n * esize > MAX_FRAME_BYTES:
            raise WireError("frame exceeds the 4 GiB a uint32 descriptor can address")
        np.frombuffer(self._buf, dtype=_DESC, count=1, offset=off)[0] = (start, n)
        self._used = start + n * esize
        if zero:
            self._buf[start:self._used] = b"\0" * (n * esize)
        return start

    def alloc(self, off, n, dtype):
        """Reserve n elements and return a writable view aliasing the slot (0-copy write path).

        Contents are left as they were -- the caller is about to overwrite them, and zeroing
        first would double the write traffic.
        """
        dt = np.dtype(dtype)
        start = self._reserve(off, n, dt.itemsize, dt.itemsize, False)
        return np.frombuffer(self._buf, dtype=dt, count=n, offset=start)

    def put_str(self, off, s):
        b = s.encode("utf-8") if isinstance(s, str) else bytes(s)
        start = self._reserve(off, len(b), 1, 1, False)
        self._buf[start:start + len(b)] = b

    def put_strs(self, off, items):
        items = list(items)
        start = self._reserve(off, len(items), DESC_SIZE, DESC_ALIGN, True)
        for i, s in enumerate(items):
            self.put_str(start + i * DESC_SIZE, s)

    def alloc_elems(self, off, n, stride, align):
        """Reserve n element blocks and return the first one's frame offset.

        Zeroed: an element block holds descriptors, and one its writer skips must read as empty
        rather than as a stale offset into this frame.
        """
        return self._reserve(off, n, stride, align, True)
