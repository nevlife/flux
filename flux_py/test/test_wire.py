import struct
import sys

import numpy as np
import pytest

from flux import wire


def test_a_dtype_a_frame_cannot_hold_is_refused():
    r = wire.Reader(bytearray(64))
    w = wire.Writer(bytearray(64), 16)
    for dt in (np.complex128, "V8", object):
        with pytest.raises(TypeError):
            r.get(0, dt)
        with pytest.raises(TypeError):
            r.span(0, dt)
        with pytest.raises(TypeError):
            w.alloc(0, 1, dt)


def test_a_zero_stride_or_align_is_refused():
    buf = bytearray(64)
    struct.pack_into("<II", buf, 0, 8, 2)
    r = wire.Reader(buf)
    w = wire.Writer(bytearray(64), 8)
    for stride, align in ((0, 1), (8, 0)):
        with pytest.raises(ValueError):
            r.elems(0, stride, align)
        with pytest.raises(ValueError):
            r.elem(0, 0, stride, align)
        with pytest.raises(ValueError):
            w.alloc_elems(0, 1, stride, align)


def test_a_dtype_named_at_run_time_is_not_kept():
    r = wire.Reader(bytearray(64))
    name = "".join(["float", "32"])
    before = sys.getrefcount(name)
    for _ in range(3):
        r.get(8, name)
    assert sys.getrefcount(name) == before


def test_a_string_field_takes_only_str_as_in_rclpy():
    for bad in (b"ab", bytearray(b"ab"), memoryview(b"ab"), np.arange(3, dtype=np.uint8)):
        w = wire.Writer(bytearray(256), 16)
        with pytest.raises(TypeError):
            w.put_str(0, bad)
        with pytest.raises(TypeError):
            w.put_strs(8, ["a", bad])
    with pytest.raises(TypeError):
        wire.Writer(bytearray(256), 16).put_strs(8, "ab")
    with pytest.raises(UnicodeEncodeError):
        wire.Writer(bytearray(256), 16).put_str(0, "\ud800")


INTS = [np.int8, np.int16, np.int32, np.int64, np.uint8, np.uint16, np.uint32, np.uint64]


@pytest.mark.parametrize("dt", INTS)
def test_an_int_out_of_range_raises_on_every_numpy(dt):
    # numpy 1.x wraps a Python int that does not fit, and numpy 2 a numpy integer.
    lo, hi = int(np.iinfo(dt).min), int(np.iinfo(dt).max)
    buf = bytearray(64)
    w = wire.Writer(buf, 16)
    w.put(0, dt, lo)
    w.put(8, dt, np.uint64(hi) if dt is np.uint64 else np.int64(hi))
    assert not w.bad
    r = wire.Reader(buf)
    assert (int(r.get(0, dt)), int(r.get(8, dt))) == (lo, hi)
    bad = [lo - 1, hi + 1]
    if dt is np.int64:
        bad.append(np.uint64(hi + 1))
    elif dt is np.uint64:
        bad.append(np.int64(-1))
    else:
        bad += [np.int64(lo - 1), np.int64(hi + 1)]
    for v in bad:
        w = wire.Writer(bytearray(64), 16)
        with pytest.raises(OverflowError):
            w.put(0, dt, v)
        assert w.bad, v


def test_a_write_that_raises_latches_bad():
    for put in (
        lambda w: w.put(0, np.uint32, None),
        lambda w: w.put(0, np.float32, "x"),
        lambda w: w.put_str(0, None),
        lambda w: w.put_strs(8, ["a", "b", "c"], 2),
        lambda w: w.put_strs(8, (s for s in ["a", 1])),
        lambda w: w.alloc(8, -1, np.float32),
        lambda w: w.alloc_elems(8, "2", 8, 8),
    ):
        w = wire.Writer(bytearray(256), 16)
        with pytest.raises((TypeError, ValueError, OverflowError)):
            put(w)
        assert w.bad


def test_a_string_ends_at_its_first_nul_as_in_rclpy():
    buf = bytearray(256)
    w = wire.Writer(buf, 16)
    w.put_str(0, "a\0b")
    w.put_strs(8, ["c\0d", "e"])
    r = wire.Reader(buf)
    assert r.string(0) == "a" and r.strings(8) == ["c", "e"]
    other = bytearray(64)
    struct.pack_into("<II", other, 0, 8, 3)
    other[8:11] = b"x\0y"
    assert wire.Reader(other).string(0) == "x"


def test_string_array_entries_that_share_bytes_are_refused():
    n, blob = 64, 8 + 8 * 64
    buf = bytearray(blob + 256)
    struct.pack_into("<II", buf, 0, 8, n)
    for i in range(n):
        struct.pack_into("<II", buf, 8 + 8 * i, blob, 256)
    with pytest.raises(wire.WireError, match="same bytes"):
        wire.Reader(buf).strings(0)
