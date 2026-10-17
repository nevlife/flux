"""Tensor messages (DLPack's fields, as tensor_msgs/ExperimentalTensor has them) through real
channels: the generated as_tensor__() reads the slot in place, and alloc_tensor__() writes it.
"""

import itertools
import os
import subprocess
import sys

import numpy as np
import pytest
import rclpy
from rclpy.node import Node

import flux2
from flux2.wire import Tensor, WireError
from test_emit import CORE_INCLUDE_FLAGS, CORE_LINK, MSG_DIR, generate, needs_cxx

try:
    import torch
except ImportError:
    torch = None

needs_torch = pytest.mark.skipif(torch is None, reason="needs torch")

DRIVER = r"""
#include "test/flux2/dl_tensor.hpp"
#include "flux2/channel.hpp"
#include "flux2/detail/subscriber.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <string>

using test::flux2_msg::DlTensor;

template <class T>
bool publish(flux2::detail::Channel & ch, std::array<std::int64_t, 2> shape, T first)
{
  auto b = DlTensor::build__(ch);
  auto t = b.alloc_tensor__<T>({shape.data(), shape.size()});
  for (std::size_t i = 0; i < t.size(); ++i) t[i] = static_cast<T>(first + static_cast<T>(i));
  return b.commit__() == flux2::Published::Ok;
}

template <class T>
void dump(const flux2::wire::Tensor & t, const T * p)
{
  std::size_t n = 1;
  for (const std::int64_t s : t.shape) n *= static_cast<std::size_t>(s);
  for (std::size_t i = 0; i < n; ++i) {
    std::size_t rest = i;
    std::int64_t at = 0;
    std::int64_t step = 1;
    for (std::size_t d = t.shape.size(); d-- > 0;) {
      const auto k = static_cast<std::int64_t>(rest % static_cast<std::size_t>(t.shape[d]));
      rest /= static_cast<std::size_t>(t.shape[d]);
      at += k * (t.strides.empty() ? step : t.strides[d]);
      step *= t.shape[d];
    }
    std::cout << ' ' << +p[at];
  }
}

// pub TOPIC: three tensors, then waits for stdin to close. sub TOPIC N: prints N frames.
int main(int, char ** argv)
{
  const std::string mode = argv[1];
  const std::string name = flux2::detail::signpost_name(argv[2], DlTensor::kFingerprint);
  if (mode == "pub") {
    auto ch = flux2::detail::Channel::create(name, 1 << 16, 4, DlTensor::kFingerprint);
    const bool ok = publish<float>(ch, {2, 3}, 0.5f) &&
                    publish<std::uint16_t>(ch, {1, 4}, 65530) &&
                    publish<std::int64_t>(ch, {2, 2}, -3000000000000LL);
    std::cout << (ok ? "ready" : "failed") << std::endl;
    std::string line;
    std::getline(std::cin, line);
    return ok ? 0 : 2;
  }
  flux2::detail::Subscriber sub(name, DlTensor::kFingerprint, flux2::QoS(8).transient_local(8), {});
  for (int i = 0; i < std::stoi(argv[3]); ++i) {
    const flux2::FrameView v = sub.take_blocking(5000000000LL, [] { return false; });
    const DlTensor::View view(v);
    const flux2::wire::Tensor t = view.as_tensor__();
    if (!view.ok__()) {
      std::cout << "bad\n";
      continue;
    }
    std::cout << +t.dtype.code << ' ' << +t.dtype.bits << " |";
    for (const std::int64_t s : t.shape) std::cout << ' ' << s;
    std::cout << " |";
    if (const auto * p = t.as<float>()) dump(t, p);
    if (const auto * p = t.as<std::int64_t>()) dump(t, p);
    if (t.dtype.code == flux2::wire::kDlBfloat || t.as<std::uint16_t>() != nullptr) {
      dump(t, static_cast<const std::uint16_t *>(t.data));
    }
    std::cout << '\n';
  }
  return 0;
}
"""


@pytest.fixture(scope="module")
def mod(tmp_path_factory):
    out = tmp_path_factory.mktemp("tensor")
    generate(os.path.join(MSG_DIR, "DlTensor.msg"), "test", str(out / "include"), str(out / "py"))
    sys.path.insert(0, str(out / "py"))
    import test_flux2.dl_tensor as m

    m.include = str(out / "include")
    yield m
    sys.path.remove(str(out / "py"))


@pytest.fixture(scope="module")
def driver(mod, tmp_path_factory):
    out = tmp_path_factory.mktemp("driver")
    src, exe = out / "driver.cpp", out / "driver"
    src.write_text(DRIVER)
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", f"-I{mod.include}"]
        + [*CORE_INCLUDE_FLAGS, str(src), *CORE_LINK, "-pthread", "-o", str(exe)],
        check=True,
        capture_output=True,
        text=True,
    )
    return str(exe)


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = Node("flux2_tensor_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


_unique = itertools.count()


def topic(name):
    return f"/tt{os.getpid()}_{name}{next(_unique)}"


def channel(node, mod, name, depth=8):
    t = topic(name)
    pub = flux2.Publisher(
        node, t, fingerprint=mod.FINGERPRINT__, slot_size=1 << 16, slot_count=depth
    )
    sub = flux2.Subscription(
        node, t, fingerprint=mod.FINGERPRINT__, qos=flux2.QoS(depth=depth, max_borrow=2)
    )
    return pub, sub


def send(pub, sub, mod, fill):
    b = mod.build__(pub)
    fill(b)
    assert b.commit__() == flux2.Published.OK
    frame = sub.take()
    return frame, mod.View(frame)


def raw(b, code, bits, shape, strides, byte_offset, nbytes, lanes=1):
    b.dtype_code, b.dtype_bits, b.dtype_lanes, b.byte_offset = code, bits, lanes, byte_offset
    b.alloc__shape(len(shape))[:] = shape
    b.alloc__strides(len(strides))[:] = strides
    return b.alloc__data(nbytes)


def shm_path(addr):
    with open("/proc/self/maps") as f:
        for line in f:
            lo, hi = (int(x, 16) for x in line.split()[0].split("-"))
            if lo <= addr < hi:
                return line.split()[-1]
    return None


def in_place(t, frame):
    """t aliases the slot: the same bytes as the frame numpy took, in the shared memory mapping."""
    lo = frame.ctypes.data
    return (
        np.shares_memory(t, frame)
        and lo <= t.ctypes.data < lo + frame.nbytes
        and shm_path(t.ctypes.data).startswith("/dev/shm/")
    )


@pytest.mark.parametrize(
    "dtype,shape,values",
    [
        (np.float32, (2, 3), np.arange(6, dtype=np.float32).reshape(2, 3) / 4),
        (np.uint16, (5,), np.array([0, 1, 2, 65534, 65535], np.uint16)),
        (np.int64, (2, 1, 2), np.array([-(2**62), -1, 0, 2**62], np.int64).reshape(2, 1, 2)),
    ],
)
def test_a_tensor_round_trips_in_place(node, mod, dtype, shape, values):
    pub, sub = channel(node, mod, f"rt_{np.dtype(dtype).name}")

    def fill(b):
        a = b.alloc_tensor__(dtype, shape)
        assert a.flags.writeable and a.ctypes.data % 64 == 0
        a[...] = values

    frame, v = send(pub, sub, mod, fill)
    t = v.as_tensor__()
    assert t.dtype == dtype and t.shape == shape and not t.flags.writeable
    np.testing.assert_array_equal(t, values)
    assert in_place(t, frame)
    assert v.dtype_lanes == 1 and len(v.strides) == 0 and v.byte_offset == 0
    if torch is not None:
        tt = torch.from_dlpack(v)
        assert tt.data_ptr() == t.ctypes.data and tt.shape == shape
        np.testing.assert_array_equal(tt.numpy(), values)


def test_strides_and_byte_offset_pick_the_elements_out_of_data(node, mod):
    pub, sub = channel(node, mod, "strided")
    m = np.arange(24, dtype=np.float32).reshape(4, 6)

    def fill(b):
        raw(b, 2, 32, (2, 3), (12, 2), 4 * 7, m.nbytes).view(np.float32)[:] = m.ravel()

    frame, v = send(pub, sub, mod, fill)
    t = v.as_tensor__()
    np.testing.assert_array_equal(t, m[1::2, 1::2])
    assert t.strides == (48, 8) and in_place(t, frame)
    if torch is not None:
        tt = torch.from_dlpack(v)
        assert tt.stride() == (12, 2) and tt.data_ptr() == t.ctypes.data
        np.testing.assert_array_equal(tt.numpy(), m[1::2, 1::2])

    def backwards(b):
        raw(b, 2, 32, (6,), (-1,), 4 * 5, 24).view(np.float32)[:] = m[0]

    frame, v = send(pub, sub, mod, backwards)
    np.testing.assert_array_equal(v.as_tensor__(), m[0, ::-1])


@needs_torch
def test_bfloat16_goes_through_dlpack_and_raw_bits(node, mod):
    pub, sub = channel(node, mod, "bf16")
    values = torch.tensor([[1.0, -2.5], [0.15625, 3e38]], dtype=torch.bfloat16)
    bits = values.view(torch.uint16).numpy()

    def fill(b):
        t = b.alloc_tensor__("bfloat16", (2, 2))
        assert isinstance(t, Tensor) and t.dtype == (4, 16, 1) and t.shape == (2, 2)
        t.bits[...] = bits
        with pytest.raises(TypeError, match="bfloat16"):
            t.array

    frame, v = send(pub, sub, mod, fill)
    t = v.as_tensor__()
    assert isinstance(t, Tensor)
    with pytest.raises(TypeError, match="__dlpack__"):
        t.array
    assert t.bits.dtype == np.uint16 and in_place(t.bits, frame)
    np.testing.assert_array_equal(t.bits, bits)
    for got in (torch.from_dlpack(v), torch.from_dlpack(t)):
        assert got.dtype == torch.bfloat16 and torch.equal(got, values)
        assert got.data_ptr() == t.bits.ctypes.data


def test_float8_reads_as_its_raw_bits(node, mod):
    pub, sub = channel(node, mod, "fp8")
    frame, v = send(pub, sub, mod, lambda b: b.alloc_tensor__("float8_e4m3fn", (3,)).bits.fill(7))
    t = v.as_tensor__()
    assert t.dtype == (10, 8, 1) and t.bits.dtype == np.uint8
    np.testing.assert_array_equal(t.bits, [7, 7, 7])


@pytest.mark.parametrize(
    "fields,why",
    [
        ((2, 32, (4,), (), 0, 12), "past the data"),
        ((2, 32, (2,), (), 12, 16), "past the data"),
        ((2, 32, (2,), (), 17, 16), "byte_offset past"),
        ((2, 32, (2,), (), 2, 16), "alignment"),
        ((2, 64, (1,), (), 4, 16), "alignment"),
        ((2, 32, (2, 2), (2,), 0, 16), "strides"),
        ((2, 32, (2,), (-1,), 0, 16), "past the data"),
        ((2, 32, (2, 2), (2, 2), 0, 16), "past the data"),
        ((2, 32, (-1,), (), 0, 16), "negative"),
        ((2, 32, (2**62, 4), (), 0, 16), "overflows"),
        ((3, 64, (1,), (), 0, 16), "dtype"),
        ((2, 8, (1,), (), 0, 16), "dtype"),
        ((99, 8, (1,), (), 0, 16), "dtype"),
        ((17, 4, (1,), (), 0, 16), "dtype"),
    ],
)
def test_a_tensor_the_frame_cannot_hold_raises(node, mod, fields, why):
    # Another process wrote the frame: no field of it is trusted, and nothing is copied instead.
    pub, sub = channel(node, mod, "bad")
    frame, v = send(pub, sub, mod, lambda b: raw(b, *fields))
    with pytest.raises(WireError, match=why):
        v.as_tensor__()
    with pytest.raises(WireError, match=why):
        v.__dlpack__()
    assert len(v.data) == fields[5]  # the fields are still readable one by one


def test_lanes_other_than_one_raise(node, mod):
    pub, sub = channel(node, mod, "lanes")
    frame, v = send(pub, sub, mod, lambda b: raw(b, 2, 32, (2,), (), 0, 32, lanes=2))
    with pytest.raises(WireError, match="dtype"):
        v.as_tensor__()


def test_a_tensor_dtype_flux2_cannot_name_is_refused_before_anything_is_written(mod):
    b = mod.Builder(bytearray(4096))
    with pytest.raises(TypeError, match="tensor"):
        b.alloc_tensor__(np.dtype("U4"), (2,))
    assert not b  # as any setter that raised: commit__() now refuses
    with pytest.raises(ValueError, match="negative"):
        mod.Builder(bytearray(4096)).alloc_tensor__(np.float32, (-1,))
    with pytest.raises(WireError):
        mod.Builder(bytearray(256)).alloc_tensor__(np.float64, (64,))


def test_a_builder_with_no_free_slot_hands_out_scratch(mod):
    class FullRing:
        def loan(self, *args, **kwargs):
            return None

    b = mod.build__(FullRing())
    a = b.alloc_tensor__(np.float32, (2, 2))
    a[...] = 1.0
    assert a.shape == (2, 2) and a.ctypes.data % 64 == 0
    assert b.alloc_tensor__("bfloat16", (3,)).bits.shape == (3,)
    assert b.commit__() == flux2.Published.BACKPRESSURE


@needs_cxx
def test_cpp_publishes_and_python_reads_in_place(node, mod, driver):
    t = topic("from_cpp")
    proc = subprocess.Popen(
        [driver, "pub", node.resolve_topic_name(t)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    try:
        assert proc.stdout.readline().strip() == "ready"
        sub = flux2.Subscription(
            node, t, fingerprint=mod.FINGERPRINT__, qos=flux2.QoS(depth=4, replay=4, max_borrow=2)
        )
        want = [
            np.arange(6, dtype=np.float32).reshape(2, 3) + 0.5,
            np.arange(65530, 65534, dtype=np.uint16).reshape(1, 4),
            np.arange(4, dtype=np.int64).reshape(2, 2) - 3_000_000_000_000,
        ]
        for w in want:
            frame = sub.take_blocking(5_000_000_000)
            got = mod.View(frame).as_tensor__()
            assert got.dtype == w.dtype and got.ctypes.data % 64 == 0
            np.testing.assert_array_equal(got, w)
            assert in_place(got, frame)
    finally:
        proc.stdin.close()
        assert proc.wait(timeout=30) == 0


@needs_cxx
def test_python_publishes_and_cpp_reads(node, mod, driver):
    pub, _ = channel(node, mod, "to_cpp")
    m = np.arange(24, dtype=np.float32).reshape(4, 6)

    def build(fill):
        b = mod.build__(pub)
        fill(b)
        assert b.commit__() == flux2.Published.OK

    build(lambda b: b.alloc_tensor__(np.float32, (2, 3)).__setitem__(..., m[:2, :3]))
    build(lambda b: b.alloc_tensor__(np.uint16, (4,)).__setitem__(..., [1, 2, 65535, 0]))
    build(lambda b: b.alloc_tensor__(np.int64, (1, 2)).__setitem__(..., [[-(2**40), 2**40]]))

    def strided(b):
        raw(b, 2, 32, (2, 3), (12, 2), 4 * 7, m.nbytes).view(np.float32)[:] = m.ravel()

    build(strided)
    build(lambda b: b.alloc_tensor__("bfloat16", (2,)).bits.__setitem__(..., [0x3F80, 0xC020]))
    build(lambda b: raw(b, 2, 32, (2,), (), 2, 16))
    r = subprocess.run(
        [driver, "sub", pub.topic_name, "6"], capture_output=True, text=True, timeout=60
    )
    assert r.returncode == 0, r.stderr
    assert r.stdout.splitlines() == [
        "2 32 | 2 3 | 0 1 2 6 7 8",
        "1 16 | 4 | 1 2 65535 0",
        "0 64 | 1 2 | -1099511627776 1099511627776",
        "2 32 | 2 3 | 7 9 11 19 21 23",
        "4 16 | 2 | 16256 49184",
        "bad",
    ]
