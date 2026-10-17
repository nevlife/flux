"""On the host GPU with torch; skipped where either is missing.

The device-handle route runs in a child process under a stand-in libcuda.so.1 that reports the
GPU as discrete. It is still an integrated GPU underneath, so that route is emulated, not measured.
"""

import os
import subprocess
import sys
import textwrap
import time
import warnings
from pathlib import Path

import numpy as np
import pytest
import rclpy
from rclpy.node import Node

import flux2
from flux2.wire import WireError

try:
    import torch
except ImportError:
    torch = None
# Skipped test by test: a module skip collects nothing, which pytest reports as exit code 5.
pytestmark = pytest.mark.skipif(
    torch is None or not torch.cuda.is_available() or flux2.probe().route == flux2.Route.NONE,
    reason="needs torch and a CUDA device",
)

SMALL = {"slot_size": 1 << 16, "slot_count": 4}


def topic(name):
    return f"/g{os.getpid()}_{name}"


KERNEL_LOG = Path(__file__).resolve().parents[2] / "scripts" / "kernel_log.sh"


@pytest.fixture(scope="module", autouse=True)
def kernel_log():
    """Fails the module on a GPU or IOMMU fault in the kernel log, which may leave no CUDA error."""
    mark = subprocess.run([KERNEL_LOG, "mark"], capture_output=True, text=True)
    yield
    if mark.returncode != 0:
        warnings.warn(
            "the kernel log is not readable without root; the GPU tests are not judged by it"
        )
        return
    check = subprocess.run(
        [KERNEL_LOG, "check", mark.stdout.strip()], capture_output=True, text=True
    )
    assert check.returncode == 0, check.stderr


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = Node("flux2_gpu_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def write_on_device(loan, value):
    with torch.cuda.stream(torch.cuda.ExternalStream(loan.stream)):
        torch.as_tensor(loan, device="cuda").fill_(value)


def test_the_probe_names_the_route_and_its_attributes():
    p = flux2.probe()
    assert p.route != flux2.Route.NONE and p.reason == ""
    assert flux2.gpu_wedged() == 0 and flux2.Published.GPU_WEDGED.value == 8
    print(p.route, p.integrated, p.pageable_access, p.host_register, p.vmm, p.posix_fd_handle)


def test_a_frame_written_on_the_device_reaches_host_and_device_subscribers(node):
    if flux2.probe().route == flux2.Route.DEVICE_HANDLE:
        pytest.skip("host subscribers need a host payload")
    pub = flux2.Publisher(node, topic("rt"), device="cuda", **SMALL)
    cpu = flux2.Subscription(node, topic("rt"), qos=flux2.QoS(depth=4))
    dev = flux2.Subscription(node, topic("rt"), qos=flux2.QoS(depth=4), device="cuda")
    loan = pub.loan((256,), "float32")
    write_on_device(loan, 3.5)
    assert loan.commit() == flux2.Published.OK
    assert pub.fence_wait.commit_waits == 1

    np.testing.assert_array_equal(cpu.take(), np.full(256, 3.5, np.float32))

    frame = dev.take()
    assert frame.shape == (256,) and frame.dtype == np.float32 and frame.nbytes == 1024
    with pytest.raises(RuntimeError, match="with sub.take"):
        _ = frame.__cuda_array_interface__
    with frame as f:
        with torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
            t = torch.from_dlpack(f)
            assert t.device.type == "cuda" and float(t.sum()) == 3.5 * 256
        # Read-only, as the subscriber's mapping is on the device; torch takes no such CAI.
        cai = f.__cuda_array_interface__
        assert cai["data"][1] is True and cai["data"][0] != 0 and cai["typestr"] == "<f4"
    # Its CUDA array interface's consumers hold the frame itself, so it goes with the frame.
    assert frame.valid
    with pytest.raises(RuntimeError, match="with sub.take"):
        _ = frame.__cuda_array_interface__
    del t, f, frame
    assert dev.take() is None  # refreshes the counters, which are as of the last take
    assert dev.fence_wait.release_waits == 1 and dev.fence_failed == 0


def test_a_callback_is_a_frames_scope(node):
    pub = flux2.Publisher(node, topic("cb"), device="cuda", **SMALL)
    seen = []
    ex = flux2.Executor()

    def cb(frame):
        with torch.cuda.stream(torch.cuda.ExternalStream(frame.stream)):
            seen.append(float(torch.from_dlpack(frame).max()))
        seen.append(frame)
        ex.stop()

    sub = flux2.Subscription(node, topic("cb"), cb, device="cuda")
    ex.add(sub)
    loan = pub.loan((64,), "int32")
    write_on_device(loan, 9)
    assert loan.commit() == flux2.Published.OK
    ex.spin()
    assert seen[0] == 9 and not seen[1].valid


def test_a_tensor_keeps_its_frame_past_the_scope(node):
    pub = flux2.Publisher(node, topic("keep"), device="cuda", slot_size=1 << 16, slot_count=2)
    sub = flux2.Subscription(node, topic("keep"), qos=flux2.QoS(depth=2), device="cuda")
    for value in range(1, 5):
        loan = pub.loan((256,), "float32")
        write_on_device(loan, value)
        assert loan.commit() == flux2.Published.OK
        if value == 1:
            with sub.take() as f:
                t = torch.from_dlpack(f)
            assert not f.valid
    assert float(t.sum()) == 256.0


def test_a_tensor_on_its_own_stream_is_waited_for_before_the_release(node):
    pub = flux2.Publisher(node, topic("own"), device="cuda", slot_size=1 << 22, slot_count=2)
    sub = flux2.Subscription(node, topic("own"), qos=flux2.QoS(depth=2), device="cuda")
    loan = pub.loan((1 << 20,), "float32")
    write_on_device(loan, 1)
    assert loan.commit() == flux2.Published.OK
    mine = torch.cuda.Stream()
    with sub.take() as f, torch.cuda.stream(mine):
        torch.cuda._sleep(200_000_000)
        out = torch.from_dlpack(f).clone()
    for value in range(2, 5):
        loan = pub.loan((1 << 20,), "float32")
        write_on_device(loan, value)
        assert loan.commit() == flux2.Published.OK
    mine.synchronize()
    assert out.unique().tolist() == [1.0]


def test_a_deferred_release_returns_at_once_and_keeps_the_slot_until_the_device_is_done(node):
    pub = flux2.Publisher(node, topic("dfr"), device="cuda", slot_size=1 << 22, slot_count=2)
    sub = flux2.Subscription(
        node, topic("dfr"), qos=flux2.QoS(depth=2), device="cuda", release="deferred"
    )
    torch.empty(1, device="cuda").clone()  # loads the kernel, which would wait for the sleep
    torch.cuda.synchronize()
    loan = pub.loan((1 << 20,), "float32")
    write_on_device(loan, 1)
    assert loan.commit() == flux2.Published.OK
    mine = torch.cuda.Stream()
    t0 = time.perf_counter()
    with sub.take() as f, torch.cuda.stream(mine):
        torch.cuda._sleep(200_000_000)
        out = torch.from_dlpack(f).clone()
    left = time.perf_counter() - t0
    for value in range(2, 5):
        loan = pub.loan((1 << 20,), "float32")
        write_on_device(loan, value)
        assert loan.commit() == flux2.Published.OK
    mine.synchronize()
    assert out.unique().tolist() == [1.0]
    assert left < 0.04, left
    assert sub.fence_wait.release_waits == 0


def test_a_loan_written_on_its_own_stream_is_waited_for_before_the_commit(node):
    pub = flux2.Publisher(node, topic("lown"), device="cuda", slot_size=1 << 22, slot_count=2)
    sub = flux2.Subscription(node, topic("lown"), qos=flux2.QoS(depth=2), device="cuda")
    loan = pub.loan((1 << 20,), "float32")
    mine = torch.cuda.Stream()
    with torch.cuda.stream(mine):
        torch.cuda._sleep(200_000_000)
        torch.from_dlpack(loan).fill_(7)
    assert loan.commit() == flux2.Published.OK
    with sub.take() as f, torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
        assert torch.from_dlpack(f).unique().tolist() == [7.0]


def test_a_loan_dropped_unfinished_waits_for_its_stream_before_the_slot_goes_back(node):
    pub = flux2.Publisher(node, topic("ldrop"), device="cuda", slot_size=1 << 22, slot_count=2)
    sub = flux2.Subscription(node, topic("ldrop"), qos=flux2.QoS(depth=2), device="cuda")
    torch.empty(1, device="cuda").fill_(0)  # loads the kernel, which would wait for the sleep
    torch.cuda.synchronize()
    loan = pub.loan((1 << 20,), "float32")
    mine = torch.cuda.Stream()
    with torch.cuda.stream(mine):
        torch.cuda._sleep(200_000_000)
        torch.from_dlpack(loan).fill_(9)
    del loan
    for value in range(1, 5):
        loan = pub.loan((1 << 20,), "float32")
        write_on_device(loan, value)
        assert loan.commit() == flux2.Published.OK
    mine.synchronize()
    for value in (3, 4):
        with sub.take() as f, torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
            assert torch.from_dlpack(f).unique().tolist() == [float(value)]


# The stream's next work is a host callback that needs the GIL: a release or commit that waited
# for it with the GIL held would never return.
GIL_WAIT = textwrap.dedent(
    """
    import ctypes, os, sys, rclpy, torch, flux2
    from rclpy.node import Node
    cuda = ctypes.CDLL("libcuda.so.1")
    ran = []
    callback = ctypes.CFUNCTYPE(None, ctypes.c_void_p)(lambda _: ran.append(1))
    def gil_work_on(stream):
        assert cuda.cuLaunchHostFunc(ctypes.c_void_p(stream.cuda_stream), callback, None) == 0
    rclpy.init()
    node = Node("flux2_gpu_gil")
    t = f"/gil{os.getpid()}"
    pub = flux2.Publisher(node, t, device="cuda", slot_size=1 << 16, slot_count=4)
    sub = flux2.Subscription(node, t, qos=flux2.QoS(depth=4), device="cuda")
    mine = torch.cuda.Stream()
    loan = pub.loan((256,), "float32")
    if sys.argv[1] == "commit":
        with torch.cuda.stream(mine):
            torch.from_dlpack(loan).fill_(1)
            torch.cuda._sleep(50_000_000)
            gil_work_on(mine)
    assert loan.commit() == flux2.Published.OK
    if sys.argv[1] == "tensor":
        with sub.take() as f, torch.cuda.stream(mine):
            x = torch.from_dlpack(f)
            torch.cuda._sleep(50_000_000)
            gil_work_on(mine)
        del x, f
    torch.cuda.synchronize()
    print(len(ran))
    """
)


@pytest.mark.parametrize("case", ["tensor", "commit"])
def test_a_stream_wait_lets_go_of_the_gil(case):
    try:
        r = subprocess.run(
            [sys.executable, "-c", GIL_WAIT, case], capture_output=True, text=True, timeout=60
        )
    except subprocess.TimeoutExpired:
        pytest.fail(f"the {case} wait held the GIL its stream's host callback needed")
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.split() == ["1"]


def test_a_bounded_fence_waits_for_work_that_ends_in_time(node):
    pub = flux2.Publisher(node, topic("bound"), device="cuda", fence_timeout=5.0, **SMALL)
    sub = flux2.Subscription(
        node, topic("bound"), qos=flux2.QoS(depth=4), device="cuda", fence_timeout=5.0
    )
    loan = pub.loan((256,), "float32")
    write_on_device(loan, 2)
    assert loan.commit() == flux2.Published.OK
    with sub.take() as f, torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
        assert torch.from_dlpack(f).unique().tolist() == [2.0]
    assert sub.take() is None
    assert pub.fence_failed == 0 and sub.fence_failed == 0
    with pytest.raises(ValueError, match="positive"):
        flux2.Publisher(node, topic("bound0"), device="cuda", fence_timeout=0, **SMALL)


def test_bfloat16_goes_through_dlpack_only(node):
    pub = flux2.Publisher(node, topic("bf16"), device="cuda", **SMALL)
    loan = pub.loan((8,), "bfloat16") if hasattr(np, "bfloat16") else None
    if loan is None:
        try:
            import ml_dtypes  # noqa: F401
        except ImportError:
            pytest.skip("no bfloat16 dtype for numpy")
        loan = pub.loan((8,), "bfloat16")
    with pytest.raises(TypeError, match="__dlpack__"):
        _ = loan.__cuda_array_interface__
    t = torch.from_dlpack(loan)
    assert t.dtype == torch.bfloat16 and t.shape == (8,)
    loan.abort()


@pytest.fixture(scope="module")
def tensor_msg(tmp_path_factory):
    from test_emit import MSG_DIR, generate

    out = tmp_path_factory.mktemp("tensor")
    generate(os.path.join(MSG_DIR, "DlTensor.msg"), "test", py=str(out))
    sys.path.insert(0, str(out))
    import test_flux2.dl_tensor as m

    yield m
    sys.path.remove(str(out))


def tensor_channel(node, m, name):
    """A host publisher and a CUDA subscription of a tensor message."""
    if flux2.probe().route == flux2.Route.DEVICE_HANDLE:
        pytest.skip("the fields are read through a host mapping")
    pub = flux2.Publisher(node, topic(name), fingerprint=m.FINGERPRINT__, **SMALL)
    qos = flux2.QoS(depth=4, max_borrow=1)
    sub = flux2.Subscription(node, topic(name), fingerprint=m.FINGERPRINT__, qos=qos, device="cuda")
    return pub, sub


def test_a_tensor_message_hands_out_its_data_on_the_device(node, tensor_msg):
    pub, sub = tensor_channel(node, tensor_msg, "tensor")
    values = np.arange(48, dtype=np.float32).reshape(6, 8)
    b = tensor_msg.build__(pub)
    b.alloc_tensor__(np.float32, (6, 8))[...] = values
    assert b.commit__() == flux2.Published.OK
    with sub.take() as f:
        v = tensor_msg.View(f)
        assert v.__dlpack_device__() == f.__dlpack_device__() and v.dtype_bits == 32
        host = v.as_tensor__()
        with torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
            t = torch.from_dlpack(v)
            base = torch.from_dlpack(f).data_ptr()
        assert t.device.type == "cuda" and t.shape == (6, 8)
        # The registered mapping's device address, at the offset `data` has in the frame.
        assert t.data_ptr() - base == host.ctypes.data - f._host().ctypes.data
        np.testing.assert_array_equal(host, values)
    with pytest.raises(RuntimeError, match="already released"):
        tensor_msg.View(f)
    # Past the scope the tensor and the View's host view still hold the slot: max_borrow is 1.
    b = tensor_msg.build__(pub)
    b.alloc_tensor__(np.float32, (1,))
    assert b.commit__() == flux2.Published.OK
    assert sub.take() is None
    np.testing.assert_array_equal(t.cpu().numpy(), values)
    del t, v, host
    with sub.take() as g:
        assert g.valid and sub.fence_failed == 0


def test_a_tensor_messages_strides_and_bfloat16_reach_the_device(node, tensor_msg):
    pub, sub = tensor_channel(node, tensor_msg, "tensor_bf16")
    m = torch.arange(24, dtype=torch.bfloat16).reshape(4, 6)
    b = tensor_msg.build__(pub)
    b.dtype_code, b.dtype_bits, b.dtype_lanes, b.byte_offset = 4, 16, 1, 2 * 7
    b.alloc__shape(2)[:] = (2, 3)
    b.alloc__strides(2)[:] = (12, 2)
    b.alloc__data(48).view(np.uint16)[:] = m.view(torch.uint16).flatten().numpy()
    assert b.commit__() == flux2.Published.OK
    with sub.take() as f, torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
        t = torch.from_dlpack(tensor_msg.View(f))
        assert t.device.type == "cuda" and t.dtype == torch.bfloat16 and t.stride() == (12, 2)
        assert torch.equal(t.cpu(), m[1::2, 1::2])


def test_a_tensor_message_the_frame_cannot_hold_raises_on_the_device_too(node, tensor_msg):
    pub, sub = tensor_channel(node, tensor_msg, "tensor_bad")
    b = tensor_msg.build__(pub)
    b.dtype_code, b.dtype_bits, b.dtype_lanes = 2, 32, 1
    b.alloc__shape(1)[:] = (5,)
    b.alloc__data(16)
    assert b.commit__() == flux2.Published.OK
    with sub.take() as f:
        with pytest.raises(WireError, match="past the data"):
            torch.from_dlpack(tensor_msg.View(f))


def test_a_device_array_is_refused_by_publish(node):
    pub = flux2.Publisher(node, topic("refuse"), device="cuda", **SMALL)
    with pytest.raises(TypeError, match="loan"):
        pub.publish(torch.zeros(4, device="cuda"))
    host = flux2.Published.WRONG_DEVICE if pub.device_backed else flux2.Published.OK
    assert pub.publish(np.zeros(4, np.float32)) == host


def test_a_device_name_other_than_cpu_or_cuda_is_refused(node):
    with pytest.raises(ValueError):
        flux2.Publisher(node, topic("name"), device="gpu", **SMALL)


PEER = textwrap.dedent(
    """
    import sys, rclpy, torch, flux2
    from rclpy.node import Node
    rclpy.init()
    node = Node("flux2_gpu_peer")
    sub = flux2.Subscription(node, sys.argv[1], qos=flux2.QoS(depth=4, replay=4), device="cuda")
    for _ in range(4):
        with sub.take_blocking(5_000_000_000) as f:
            with torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
                print(float(torch.from_dlpack(f).sum()))
    """
)


def test_a_frame_written_on_the_device_reaches_a_device_subscriber_in_another_process(node):
    t = topic("peer")
    pub = flux2.Publisher(node, t, device="cuda", **SMALL)
    for value in range(1, 5):
        loan = pub.loan((256,), "float32")
        write_on_device(loan, value)
        assert loan.commit() == flux2.Published.OK
    assert pub.fence_wait.commit_waits == 4
    r = subprocess.run(
        [sys.executable, "-c", PEER, node.resolve_topic_name(t)],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert r.returncode == 0, r.stdout + r.stderr
    assert [float(x) for x in r.stdout.split()] == [256.0 * v for v in range(1, 5)]


SHIM = r"""
#include <dlfcn.h>
typedef int (*attr_fn)(int *, int, int);
int cuDeviceGetAttribute(int * v, int attrib, int dev)
{
  static attr_fn real;
  if (!real) real = (attr_fn)dlsym(dlopen(REAL, RTLD_LAZY | RTLD_NOLOAD), "cuDeviceGetAttribute");
  int r = real(v, attrib, dev);
  if (r == 0 && (attrib == 18 || attrib == 88)) *v = 0;
  return r;
}
"""

CHILD = textwrap.dedent(
    """
    import os, numpy as np, rclpy, torch, flux2
    from rclpy.node import Node
    assert flux2.probe().route == flux2.Route.DEVICE_HANDLE, flux2.probe().route
    rclpy.init()
    node = Node("flux2_dgpu")
    t = f"/d{os.getpid()}"
    pub = flux2.Publisher(node, t, slot_size=1 << 16, slot_count=4, device="cuda")
    assert pub.device_backed
    try:
        flux2.Subscription(node, t)
        raise SystemExit("a host subscription attached to a device payload")
    except flux2.Error as e:
        assert e.status == flux2.Status.MISMATCH, e
    sub = flux2.Subscription(node, t, qos=flux2.QoS(depth=4), device="cuda")
    loan = pub.loan((128,), "float32")
    try:
        loan.array
        raise SystemExit("a device loan handed out a host array")
    except RuntimeError:
        pass
    assert pub.publish(np.zeros(4, np.float32)) == flux2.Published.WRONG_DEVICE
    with torch.cuda.stream(torch.cuda.ExternalStream(loan.stream)):
        torch.as_tensor(loan, device="cuda").fill_(1.25)
    assert loan.commit() == flux2.Published.OK
    with sub.take() as f:
        with torch.cuda.stream(torch.cuda.ExternalStream(f.stream)):
            assert float(torch.from_dlpack(f).sum()) == 1.25 * 128
    print("ok")
    """
)


def real_libcuda():
    out = subprocess.run(["ldconfig", "-p"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if "libcuda.so.1 " in line:
            return os.path.realpath(line.rsplit(" ", 1)[1])
    return None


def test_the_device_handle_route_under_a_discrete_stand_in(tmp_path):
    real = real_libcuda()
    if real is None:
        pytest.skip("libcuda.so.1 not in the loader cache")
    # The stand-in carries DT_NEEDED on the real driver by absolute path: a stub with that soname
    # gives the link one. flux2 dlopens libcuda.so.1 and finds every other symbol through it.
    (tmp_path / "stub.c").write_text("void cuInit(void) {}\n")
    (tmp_path / "shim.c").write_text(SHIM)
    lib = tmp_path / "lib"
    lib.mkdir()
    cc = ["cc", "-shared", "-fPIC"]
    try:
        subprocess.run(
            cc + [f"-Wl,-soname,{real}", "-o", str(tmp_path / "stub.so"), str(tmp_path / "stub.c")],
            check=True,
        )
        subprocess.run(
            cc
            + [f'-DREAL="{real}"', "-o", str(lib / "libcuda.so.1"), str(tmp_path / "shim.c")]
            + ["-Wl,--no-as-needed", str(tmp_path / "stub.so"), "-ldl"],
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        pytest.skip("no C compiler for the stand-in")
    env = dict(os.environ, LD_LIBRARY_PATH=f"{lib}:{os.environ.get('LD_LIBRARY_PATH', '')}")
    r = subprocess.run(
        [sys.executable, "-c", CHILD], env=env, capture_output=True, text=True, timeout=120
    )
    assert r.returncode == 0 and r.stdout.strip().endswith("ok"), r.stdout + r.stderr
