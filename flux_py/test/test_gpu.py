"""flux_py CUDA path: declaration, device views, and the scope a fenced release requires.

Every test here skips on a host with no usable CUDA route. That is the normal state of a CI
runner, so the skip must not be silent about which half ran -- `cuda_or_skip` reports the reason
the constructor gave rather than a bare skip.

What is exercised is the binding, not the driver: that a declaration reaches the channel, that a
device view names the slot, and that a frame is unusable outside `with`. The fence itself is
flux_core's and is covered there.
"""

import numpy as np
import pytest

import flux

FP = 0xC0DE0607


def fill(loan, values):
    """Write `values` into `loan`, or state why this route cannot.

    An integrated GPU's slot is host memory as well, so numpy fills it. A discrete GPU's slot is
    VRAM, and filling it from Python needs cupy or torch -- so where neither is installed the
    assertion this makes is the refusal itself, which is the behaviour under test on that route.
    """
    if loan.host_addressable:
        loan.array[:] = values
        return True
    with pytest.raises(RuntimeError, match="needs host memory"):
        loan.array
    return False


def cuda_or_skip(topic, **kwargs):
    """A Publisher on `topic`, or skip naming why this host refused the declaration."""
    try:
        return flux.Publisher(topic, fingerprint=FP, device="cuda", **kwargs)
    except (ValueError, RuntimeError) as exc:
        pytest.skip(f"flux-cap:gpu-device no usable CUDA route here: {exc}")


def test_device_accepts_a_string_and_the_enum():
    pub = cuda_or_skip("/pytest/gpu/decl", slot_size=1 << 16, slot_count=4)
    same = flux.Publisher(
        "/pytest/gpu/decl", slot_size=1 << 16, slot_count=4, fingerprint=FP,
        device=flux.Device.Cuda,
    )
    assert pub.stream is not None
    assert same.stream is not None


def test_an_unknown_device_is_refused_not_guessed():
    with pytest.raises(ValueError):
        flux.Publisher("/pytest/gpu/bad", fingerprint=FP, device="gpu")
    with pytest.raises(ValueError):
        flux.Subscription("/pytest/gpu/bad", fingerprint=FP, device="opencl")


def test_a_host_channel_declares_no_stream():
    pub = flux.Publisher("/pytest/gpu/host", slot_size=1 << 16, slot_count=2, fingerprint=FP)
    sub = flux.Subscription("/pytest/gpu/host", fingerprint=FP)
    assert pub.stream is None
    assert sub.stream is None
    assert pub.fence_failed == 0
    assert sub.fence_failed == 0


def test_loan_exposes_the_slot_as_a_device_array():
    pub = cuda_or_skip("/pytest/gpu/loan", slot_size=1 << 16, slot_count=4)
    loan = pub.loan((4, 8), dtype="float32")
    assert loan is not None

    cai = loan.__cuda_array_interface__
    assert cai["shape"] == (4, 8)
    assert cai["typestr"] == "<f4"
    assert cai["version"] == 3
    assert cai["strides"] is None  # C-contiguous
    ptr, read_only = cai["data"]
    assert ptr != 0
    assert read_only is False  # the producer writes here
    assert loan.stream is not None

    # commit() waits on the stream, so the two seams are closed before anything is published.
    fill(loan, np.arange(32, dtype=np.float32).reshape(4, 8))
    assert loan.commit() == flux.Published.Ok
    assert pub.fence_failed == 0


def test_a_consumed_loan_has_no_device_address():
    pub = cuda_or_skip("/pytest/gpu/loan_done", slot_size=1 << 16, slot_count=4)
    loan = pub.loan((16,), dtype="uint8")
    assert loan.commit() == flux.Published.Ok
    with pytest.raises(RuntimeError):
        loan.__cuda_array_interface__


def test_a_host_publisher_has_no_device_address():
    pub = flux.Publisher("/pytest/gpu/host_loan", slot_size=1 << 16, slot_count=2, fingerprint=FP)
    loan = pub.loan((16,), dtype="uint8")
    with pytest.raises(RuntimeError):
        loan.__cuda_array_interface__
    assert loan.stream is None


def test_a_device_subscription_hands_back_a_scoped_frame():
    pub = cuda_or_skip("/pytest/gpu/frame", slot_size=1 << 16, slot_count=4)
    sub = flux.Subscription("/pytest/gpu/frame", fingerprint=FP, device="cuda")

    loan = pub.loan((6,), dtype="int32")
    fill(loan, np.arange(6, dtype=np.int32))
    assert loan.commit() == flux.Published.Ok

    frame = sub.take()
    assert isinstance(frame, flux.Frame)
    assert frame.shape == (6,)
    assert frame.dtype == "<i4"
    assert frame.nbytes == 24
    assert frame.stream is not None
    assert frame.released is False

    with frame as v:
        cai = v.__cuda_array_interface__
        assert cai["shape"] == (6,)
        ptr, read_only = cai["data"]
        assert ptr != 0
        assert read_only is True  # a subscriber mapping drops payload to PROT_READ

    assert frame.released is True
    assert sub.fence_failed == 0


def test_a_device_frame_is_unusable_outside_a_scope():
    pub = cuda_or_skip("/pytest/gpu/scope", slot_size=1 << 16, slot_count=4)
    sub = flux.Subscription("/pytest/gpu/scope", fingerprint=FP, device="cuda")
    pub.loan((4,), dtype="uint8").commit()

    frame = sub.take()
    assert frame is not None
    # Releasing this frame fences a stream; letting that land whenever the collector runs is
    # exactly the non-determinism the scope exists to remove, so reading it outside is refused.
    with pytest.raises(RuntimeError):
        frame.__cuda_array_interface__
    # The stream is a property of the channel, not of the borrow, so it reads either way.
    assert frame.stream is not None


def test_leaving_the_scope_returns_the_slot():
    # slot_count 2 with max_borrow 2: holding both frames starves the publisher, so a publish
    # that succeeds after the scopes close is what proves the borrows were actually returned.
    pub = cuda_or_skip("/pytest/gpu/return", slot_size=1 << 12, slot_count=2)
    # depth=2 so both frames are delivered: at depth 1 the cursor is pulled to the newest each
    # take and the second one has nothing to hand back (docs/en/qos.en.md, X-002).
    sub = flux.Subscription(
        "/pytest/gpu/return", fingerprint=FP, device="cuda",
        qos=flux.QoS(depth=2, max_borrow=2),
    )

    for _ in range(2):
        loan = pub.loan((8,), dtype="uint8")
        assert loan is not None
        assert loan.commit() == flux.Published.Ok

    held = [sub.take(), sub.take()]
    assert all(f is not None for f in held)
    for f in held:
        f.__enter__()
    assert pub.loan((8,), dtype="uint8") is None  # every slot borrowed
    for f in held:
        f.__exit__(None, None, None)

    assert pub.loan((8,), dtype="uint8") is not None
    assert sub.fence_failed == 0


def test_whether_a_host_subscriber_may_read_a_device_channel_follows_the_route():
    """The two routes answer this oppositely, so the test asserts the pairing, not one answer.

    On an integrated GPU the slot is host memory as well, so a CPU consumer of the same channel
    needs to know nothing about the publisher's declaration. On a
    discrete GPU the slot is VRAM and there is nothing for a host read to follow, so the attach is
    refused outright rather than handing back an address that faults.
    """
    pub = cuda_or_skip("/pytest/gpu/mixed", slot_size=1 << 16, slot_count=4)

    if not pub.host_addressable:
        with pytest.raises(flux.SegmentMismatch):
            flux.Subscription("/pytest/gpu/mixed", fingerprint=FP)
        return

    sub = flux.Subscription("/pytest/gpu/mixed", fingerprint=FP)  # host
    payload = np.arange(64, dtype=np.uint8)
    loan = pub.loan((64,), dtype="uint8")
    loan.array[:] = payload
    assert loan.commit() == flux.Published.Ok

    got = sub.take()
    assert isinstance(got, np.ndarray)  # a host subscription is unchanged by the publisher
    np.testing.assert_array_equal(got, payload)


def device_array_or_skip():
    """A 16-byte device array from whichever library this host has. Either will do -- what is
    under test is flux's refusal, not theirs, and pinning it to one of them made this skip on a
    machine that could run it."""
    try:
        import cupy as cp

        return cp.zeros(16, dtype=cp.uint8)
    except ImportError:
        pass
    try:
        import torch

        if torch.cuda.is_available():
            return torch.zeros(16, dtype=torch.uint8, device="cuda")
    except ImportError:
        pass
    pytest.skip("flux-cap:cupy-or-torch needs a library that allocates device memory (cupy or torch)")


def test_publish_refuses_a_device_array_by_name():
    arr = device_array_or_skip()
    pub = cuda_or_skip("/pytest/gpu/devarr", slot_size=1 << 16, slot_count=2)
    with pytest.raises(ValueError, match="loan"):
        pub.publish(arr)


def test_publish_of_a_host_array_into_a_device_channel_is_refused():
    """The Python half of S-001: the same refusal the C++ engine reports as an enum value.

    flux_core returns Published::WrongDevice because publish() is noexcept for the hard-RT path.
    The binding raises instead, so a Python caller cannot ignore a return value it never has to
    look at.
    """
    pub = cuda_or_skip("/pytest/gpu/hostpub", slot_size=1 << 16, slot_count=4)
    if pub.host_addressable:
        pytest.skip(
            "flux-cap:gpu-device-handle this route's slots are host memory, so publish() has "
            "somewhere to copy to"
        )
    with pytest.raises(ValueError, match="nowhere to copy to"):
        pub.publish(np.arange(64, dtype=np.uint8))


def test_fence_wait_is_zero_on_a_host_channel():
    pub = flux.Publisher("/pytest/gpu/nowait", slot_size=1 << 16, slot_count=2, fingerprint=FP)
    pub.publish(np.zeros(16, dtype=np.uint8))
    w = pub.fence_wait
    assert w.commit_ns == 0
    assert w.commit_count == 0
    assert w.release_ns == 0


def test_the_executor_delivers_a_scoped_frame_too():
    # The callback path must not be a hole in the scope rule: same object, same contract,
    # wherever the frame came from.
    pub = cuda_or_skip("/pytest/gpu/exec", slot_size=1 << 12, slot_count=4)
    sub = flux.Subscription("/pytest/gpu/exec", fingerprint=FP, device="cuda")
    ex = flux.Executor(max_channels=2, poll_tick_ns=1_000_000)

    seen = []

    def on_frame(frame):
        assert isinstance(frame, flux.Frame)
        with pytest.raises(RuntimeError):
            frame.__cuda_array_interface__
        with frame as v:
            seen.append(v.__cuda_array_interface__["shape"])

    ex.add(sub, on_frame)
    ex.dispatch()  # attach before publishing, so Volatile does not skip the frame

    loan = pub.loan((5,), dtype="uint8")
    assert loan.commit() == flux.Published.Ok
    ex.dispatch()

    assert seen == [(5,)]
    assert sub.fence_failed == 0


# bfloat16 on the device path. This is the pair the CAI route cannot serve at all -- its typestr
# is numpy's, and numpy has no bf16 -- so a GPU consumer of a bf16 frame reaches it through
# __dlpack__ or not at all. torch is the consumer that reads DLPack; it is not a flux
# dependency, so its absence skips rather than fails.


def torch_or_skip():
    try:
        import torch
    except ImportError:
        pytest.skip("flux-cap:torch torch is not installed; the DLPack consumer side cannot be exercised")
    if not torch.cuda.is_available():
        pytest.skip("flux-cap:torch-cuda torch has no CUDA device here")
    return torch


def test_bfloat16_crosses_the_device_seam_through_dlpack():
    torch = torch_or_skip()
    pub = cuda_or_skip("/pytest/gpu/bf16", slot_size=1 << 16, slot_count=4)
    sub = flux.Subscription("/pytest/gpu/bf16", fingerprint=FP, device="cuda")

    vals = torch.tensor([1.0, -2.5, 3.75, 100.0], dtype=torch.bfloat16, device="cuda")
    loan = pub.loan((4,), dtype="bfloat16")
    assert loan.__dlpack_device__() == (2, 0)  # kDLCUDA: the slot itself is device memory
    written = torch.from_dlpack(loan)
    assert written.dtype == torch.bfloat16 and written.is_cuda
    written.copy_(vals)
    torch.cuda.synchronize()
    assert loan.commit() == flux.Published.Ok

    frame = sub.take()
    assert frame is not None
    assert frame.dtype == "bfloat16"
    assert frame.__dlpack_device__() == (2, 0)
    with frame as v:
        read = torch.from_dlpack(v)
        assert read.dtype == torch.bfloat16 and read.is_cuda
        assert torch.equal(read, vals)
        # No copy anywhere: the tensor names the slot. On ShmDirect the device address IS the
        # host mapping, so the address torch got is the one .bits reports.
        assert read.data_ptr() == np.asarray(v.bits).__array_interface__["data"][0]
    assert sub.fence_failed == 0


def test_a_bfloat16_device_frame_refuses_the_cuda_array_interface():
    # The refusal is the point: handing back 'u2' or 'f2' would give cupy numbers that are not
    # the ones in the slot, and nothing downstream could notice.
    torch_or_skip()
    pub = cuda_or_skip("/pytest/gpu/bf16cai", slot_size=1 << 16, slot_count=4)
    sub = flux.Subscription("/pytest/gpu/bf16cai", fingerprint=FP, device="cuda")
    pub.loan((2,), dtype="bfloat16").commit()

    frame = sub.take()
    assert frame is not None
    with frame as v:
        with pytest.raises(RuntimeError, match="bfloat16"):
            v.__cuda_array_interface__
        assert type(v.__dlpack__(stream=None)).__name__ == "PyCapsule"


def test_a_device_frame_capsule_is_refused_outside_the_scope():
    # __dlpack__ carries the same rule as __cuda_array_interface__: the tensor it produces aliases
    # the slot, so handing one out unscoped puts the release back on the collector's schedule.
    torch_or_skip()
    pub = cuda_or_skip("/pytest/gpu/bf16scope", slot_size=1 << 16, slot_count=4)
    sub = flux.Subscription("/pytest/gpu/bf16scope", fingerprint=FP, device="cuda")
    pub.loan((2,), dtype="bfloat16").commit()

    frame = sub.take()
    assert frame is not None
    with pytest.raises(RuntimeError, match="with"):
        frame.__dlpack__()
    assert frame.shape == (2,)  # shape is a property of the frame, readable either way
