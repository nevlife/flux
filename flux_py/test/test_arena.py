"""Experimental arena: fields past the slot, named by offset, between Python and C++. The adopting
test runs a child under LD_PRELOAD=libflux_heaphook.so.
"""

import itertools
import os
import subprocess
import sys
import textwrap

import numpy as np
import pytest
import rclpy
from rclpy.node import Node

import flux
from test_emit import CORE_INCLUDE_FLAGS, CORE_LIB_DIR, CORE_LINK, GEN, generate, needs_cxx
from test_emit import MSG_DIR as GEN_MSG_DIR

os.environ["FLUX_ARENA_BYTES"] = str(64 << 20)  # before the first loan makes the arena

MSG_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "flux_cpp", "test", "msg")
SLOT = 4096
RUN = 1 << 20

DRIVER = r"""
#include "flux_test/flux/blob.hpp"
#include "flux/channel.hpp"
#include "flux/detail/subscriber.hpp"

#include <cstdint>
#include <iostream>
#include <string>

using flux_test::flux_msg::Blob;

// pub TOPIC: one Blob past its slot, then waits for stdin to close. sub TOPIC: prints one.
int main(int, char ** argv)
{
  const std::string mode = argv[1];
  const std::string name = flux::detail::signpost_name(argv[2], Blob::kFingerprint);
  if (mode == "pub") {
    auto ch = flux::detail::Channel::create(name, 4096, 4, Blob::kFingerprint);
    auto b = Blob::build__(ch);
    b.set__name("from c++");
    auto d = b.alloc__data(1 << 20);
    for (std::size_t i = 0; i < d.size(); ++i) d[i] = static_cast<std::uint8_t>(i * 7);
    b.alloc__tags(300);
    for (std::size_t i = 0; i < 300; ++i) b.set__tags(i, "tag" + std::to_string(i));
    auto items = b.alloc__items(200);
    for (std::size_t i = 0; i < items.size(); ++i) {
      items[i].set__label("item" + std::to_string(i));
      items[i].alloc__ids(3)[2] = static_cast<std::uint16_t>(i);
    }
    const bool ok = b.commit__() == flux::Published::Ok;
    std::cout << (ok ? "ready" : "failed") << std::endl;
    std::string line;
    std::getline(std::cin, line);
    return ok ? 0 : 2;
  }
  flux::detail::Subscriber sub(name, Blob::kFingerprint, flux::QoS(8).transient_local(8), {});
  const flux::FrameView v = sub.take_blocking(5000000000LL, [] { return false; });
  const Blob::View view(v);
  std::uint64_t sum = 0;
  for (const std::uint8_t x : view.data()) sum += x;
  std::cout << view.name() << '|' << view.data().size() << '|' << sum << '|' << view.tags(299)
            << '|' << view.items(199).label() << '|' << view.items(199).ids()[2] << '|'
            << view.ok__() << std::endl;
  return 0;
}
"""


@pytest.fixture(scope="module")
def mod(tmp_path_factory):
    out = tmp_path_factory.mktemp("arena")
    cmd = [GEN, "--pkg", "flux_test"]
    for m in ("Item", "Blob"):
        cmd += ["--msg", os.path.join(MSG_DIR, f"{m}.msg")]
    cmd += ["--out-cpp", str(out / "include"), "--out-py", str(out / "py")]
    subprocess.run(cmd, check=True, capture_output=True, text=True)
    sys.path.insert(0, str(out / "py"))
    import flux_test_flux.blob as m

    m.include = str(out / "include")
    m.py = str(out / "py")
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
    n = Node("flux_arena_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


_unique = itertools.count()


def topic(name):
    return f"/ar{os.getpid()}_{name}{next(_unique)}"


def channel(node, mod, t):
    pub = flux.Publisher(node, t, fingerprint=mod.FINGERPRINT__, slot_size=SLOT, slot_count=4)
    sub = flux.Subscription(node, t, fingerprint=mod.FINGERPRINT__)
    return pub, sub


def fill(b):
    b.name = "from python"
    b.alloc__data(RUN)[:] = (np.arange(RUN) * 7).astype(np.uint8)
    b.tags = [f"tag{i}" for i in range(300)]
    items = b.alloc__items(200)
    for i in range(200):
        items[i].label = f"item{i}"
        items[i].alloc__ids(3)[2] = i


def check(v, name):
    assert v.name == name
    assert len(v.data) == RUN
    assert np.array_equal(v.data, (np.arange(RUN) * 7).astype(np.uint8))
    assert v.tags[299] == "tag299"
    assert v.items[199].label == "item199"
    assert v.items[199].ids[2] == 199


@pytest.fixture(scope="module")
def tensor_mod(tmp_path_factory):
    out = tmp_path_factory.mktemp("arena_tensor")
    generate(
        os.path.join(GEN_MSG_DIR, "DlTensor.msg"), "test", str(out / "include"), str(out / "py")
    )
    sys.path.insert(0, str(out / "py"))
    import test_flux.dl_tensor as m

    yield m
    sys.path.remove(str(out / "py"))


def test_a_tensor_past_its_slot_is_read_from_the_arena(node, tensor_mod):
    t = topic("tensor")
    pub = flux.Publisher(
        node, t, fingerprint=tensor_mod.FINGERPRINT__, slot_size=SLOT, slot_count=4
    )
    sub = flux.Subscription(node, t, fingerprint=tensor_mod.FINGERPRINT__)
    values = np.arange(512 * 256, dtype=np.float32).reshape(512, 256)
    b = tensor_mod.build__(pub)
    b.alloc_tensor__(np.float32, values.shape)[...] = values
    assert b.commit__() == flux.Published.OK
    frame = sub.take()
    assert frame.nbytes <= SLOT
    got = tensor_mod.View(frame).as_tensor__()
    np.testing.assert_array_equal(got, values)


def test_a_frame_larger_than_its_slot_goes_through_the_arena(node, mod):
    pub, sub = channel(node, mod, topic("py"))
    b = mod.build__(pub)
    fill(b)
    assert b.commit__() == flux.Published.OK
    frame = sub.take()
    assert frame.nbytes <= SLOT
    check(mod.View(frame), "from python")


@needs_cxx
def test_cpp_reads_what_python_put_in_the_arena(node, mod, driver):
    t = topic("tocpp")
    pub = flux.Publisher(node, t, fingerprint=mod.FINGERPRINT__, slot_size=SLOT, slot_count=4)
    b = mod.build__(pub)
    fill(b)
    assert b.commit__() == flux.Published.OK
    out = subprocess.run([driver, "sub", t], check=True, capture_output=True, text=True).stdout
    total = int((np.arange(RUN) * 7).astype(np.uint8).sum())
    assert out.strip() == f"from python|{RUN}|{total}|tag299|item199|199|1"


@needs_cxx
def test_python_reads_what_cpp_put_in_the_arena(node, mod, driver):
    t = topic("fromcpp")
    # Replayed: the frame may be committed before the subscription attaches.
    qos = flux.QoS(depth=4, replay=4)
    sub = flux.Subscription(node, t, fingerprint=mod.FINGERPRINT__, qos=qos)
    with subprocess.Popen(
        [driver, "pub", t], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True
    ) as p:
        assert p.stdout.readline().strip() == "ready"
        check(mod.View(sub.take_blocking(5_000_000_000)), "from c++")
        p.stdin.close()


CHILD = """
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

import flux

sys.path.insert(0, {py!r})
import flux_test_flux.blob as mod

rclpy.init()
node = Node("flux_arena_child")
pub = flux.Publisher(node, {topic!r}, fingerprint=mod.FINGERPRINT__, slot_size=4096, slot_count=2)
with flux.arena_scope() as opened:
    a = np.full({run}, 9, dtype=np.uint8)  # as a library making its own array would
b = mod.build__(pub)
adopted = b.adopt__data(a)
del a
print(opened, adopted, b.commit__() == flux.Published.OK, flush=True)
sys.stdin.readline()
"""


def test_an_array_made_in_a_scope_goes_out_without_a_copy(node, mod):
    hook = os.path.join(CORE_LIB_DIR, "libflux_heaphook.so")
    t = topic("adopt")
    qos = flux.QoS(depth=4, replay=4)
    sub = flux.Subscription(node, t, fingerprint=mod.FINGERPRINT__, qos=qos)
    env = dict(os.environ, LD_PRELOAD=hook)
    code = textwrap.dedent(CHILD.format(py=mod.py, topic=t, run=RUN))
    with subprocess.Popen(
        [sys.executable, "-c", code],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        env=env,
        text=True,
    ) as p:
        assert p.stdout.readline().split() == ["True", "True", "True"]
        v = mod.View(sub.take_blocking(5_000_000_000))
        assert len(v.data) == RUN
        assert v.data[0] == 9 and v.data[-1] == 9
        p.stdin.close()
