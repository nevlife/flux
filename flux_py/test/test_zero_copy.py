"""flux_py zero-copy round-trip and borrow-lifecycle tests.

Publish a numpy array from Python, take it back as a read-only zero-copy view, and verify
the view aliases the shared segment and that the borrow is tied to the array's lifetime.
"""

import gc
import time

import numpy as np
import pytest

import flux

FP = 0xC0DE1234


def test_zero_copy_roundtrip():
    pub = flux.Publisher("/pytest/zc", slot_size=1 << 20, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/zc", fingerprint=FP)

    a = np.arange(1000, dtype=np.float32).reshape(10, 100)
    assert pub.publish(a) == flux.Published.Ok

    v = sub.take()
    assert v is not None
    assert v.shape == (10, 100)
    assert v.dtype == np.float32
    np.testing.assert_array_equal(v, a)


def test_view_is_readonly_and_not_a_copy():
    pub = flux.Publisher("/pytest/ro", slot_size=1 << 20, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/ro", fingerprint=FP)

    a = np.full(2048, 7, dtype=np.uint8)
    assert pub.publish(a) == flux.Published.Ok

    v = sub.take()
    assert v is not None
    # A view into the publisher's slot must not be writable.
    assert v.flags.writeable is False
    # It borrows external memory (the capsule owner), so it is not numpy's own buffer.
    assert v.base is not None


def test_dtypes_roundtrip():
    # float16 is in the list because it is the one type the binding cannot template on -- it is
    # carried as u16 with the dtype overridden, and nothing was covering that branch.
    for dt in (np.uint8, np.int16, np.uint32, np.int64, np.float16, np.float32, np.float64):
        pub = flux.Publisher(f"/pytest/dt_{dt.__name__}", slot_size=1 << 16,
                             slot_count=2, fingerprint=FP)
        sub = flux.Subscription(f"/pytest/dt_{dt.__name__}", fingerprint=FP)
        a = (np.arange(64) % 17).astype(dt)
        assert pub.publish(a) == flux.Published.Ok
        v = sub.take()
        assert v is not None
        assert v.dtype == dt
        np.testing.assert_array_equal(v, a)


def test_take_empty_returns_none():
    sub = flux.Subscription("/pytest/never", fingerprint=FP)
    assert sub.take() is None
    assert sub.peek() is None


def test_view_outlives_subscription():
    # The returned view pins its Subscription (keepalive), so it stays valid even after the
    # Subscription goes out of scope. Without that, the view would alias an unmapped segment
    # and reading it (or its GC) would crash.
    pub = flux.Publisher("/pytest/outlive", slot_size=4096, slot_count=4, fingerprint=FP)
    a = np.arange(256, dtype=np.uint8)
    assert pub.publish(a) == flux.Published.Ok

    # The subscription joins after the publish, so volatile take() would skip it; peek() reads
    # current state regardless.
    sub = flux.Subscription("/pytest/outlive", fingerprint=FP)
    v = sub.peek()
    assert v is not None

    del sub
    gc.collect()  # Subscription is pinned by v, so its mapping survives

    np.testing.assert_array_equal(v, a)
    del v
    gc.collect()


def test_borrow_blocks_publish_then_releases_on_gc():
    # One slot: while the view is alive it borrows the only slot, so publish must drop.
    # Releasing the view (GC) frees the slot and publish succeeds again.
    pub = flux.Publisher("/pytest/borrow", slot_size=4096, slot_count=1, fingerprint=FP)
    sub = flux.Subscription("/pytest/borrow", fingerprint=FP)
    a = np.zeros(1024, dtype=np.uint8)

    assert pub.publish(a) == flux.Published.Ok
    v = sub.take()
    assert v is not None

    assert pub.publish(a) == flux.Published.Backpressure  # only slot borrowed -> dropped
    assert pub.dropped >= 1

    del v
    gc.collect()  # releases the borrow (FrameView destructor -> refcount--)

    assert pub.publish(a) == flux.Published.Ok  # slot free again


def test_oversized_publish_rejected():
    # A payload larger than slot_size must be rejected outright, not silently truncated with
    # the full shape kept -- a shape-driven numpy view would otherwise read past the slot.
    pub = flux.Publisher("/pytest/oversize", slot_size=4096, slot_count=2, fingerprint=FP)
    big = np.zeros(8192, dtype=np.uint8)  # 8192 > 4096
    with pytest.raises(ValueError):
        pub.publish(big)
    ok = np.zeros(4096, dtype=np.uint8)  # exactly slot_size still works
    assert pub.publish(ok) == flux.Published.Ok


def test_relative_topic_rejected():
    # flux_py has no ROS node to resolve names, so the topic must already be the absolute
    # resolved name a C++ node would use; a relative name would name a different segment.
    with pytest.raises(ValueError):
        flux.Publisher("pytest/relative", slot_size=4096, slot_count=2, fingerprint=FP)
    with pytest.raises(ValueError):
        flux.Subscription("pytest/relative", fingerprint=FP)


def test_config_mismatch_is_distinct_from_an_absent_segment():
    # The two ways an attach fails are not one failure. No publisher yet is a retryable miss the
    # caller sits through; a live publisher on another ring config can never become right. Without
    # a type of its own the second arrives as a bare RuntimeError, and a caller that cannot tell
    # them apart either spins forever on a misconfiguration or gives up on a slow publisher.
    assert issubclass(flux.SegmentMismatch, RuntimeError)

    # Absent: the subscription constructs, and stays empty until a publisher shows up.
    sub = flux.Subscription("/pytest/mismatch_absent", fingerprint=FP)
    assert sub.take() is None

    pub = flux.Publisher("/pytest/mismatch", slot_size=4096, slot_count=2, fingerprint=FP)
    with pytest.raises(flux.SegmentMismatch):
        flux.Publisher("/pytest/mismatch", slot_size=8192, slot_count=4, fingerprint=FP)
    assert pub.dropped == 0


def test_uncommitted_loan_is_safe_to_drop():
    # Loan pins the Publisher through a keepalive, but members destroy in reverse declaration
    # order: with the WriteSlot declared first it was destroyed LAST, so abort() wrote the slot
    # after the mapping was already gone. Dropping a loan the documented way segfaulted.
    def scope():
        pub = flux.Publisher("/pytest/loan/drop", slot_size=4096, slot_count=4, fingerprint=FP)
        loan = pub.loan((16,), dtype="uint8")
        assert loan is not None
        loan.array[:] = 7  # written, never committed

    scope()  # both die here; the test failing means the process crashed


def test_publish_rejects_rank_above_max_dims():
    # ndim was only checked on the read side, where a bad frame is dropped WITHOUT advancing the
    # cursor -- so one such publish wedged take() on that slot forever. Reject on write instead.
    # A rank flux cannot carry is a wiring mistake, so it raises and leaves dropped alone.
    pub = flux.Publisher("/pytest/loan/rank", fingerprint=FP, slot_size=4096, slot_count=4)
    sub = flux.Subscription("/pytest/loan/rank", fingerprint=FP)

    before = pub.dropped
    with pytest.raises(ValueError):
        pub.publish(np.zeros((1,) * 9, dtype=np.uint8))
    assert pub.dropped == before

    assert pub.publish(np.full(8, 3, dtype=np.uint8)) == flux.Published.Ok  # channel still works
    v = sub.take()
    assert v is not None and int(v[0]) == 3


def test_take_blocking_does_not_park_when_max_borrow_is_held():
    # Only the caller can free a lease, and it cannot while parked in take_blocking. Treating
    # "lease exhausted" as "no data" slept through every publish until the timeout.
    pub = flux.Publisher("/pytest/loan/lease", slot_size=64, slot_count=4, fingerprint=FP)
    sub = flux.Subscription(
        "/pytest/loan/lease", fingerprint=FP, qos=flux.QoS(depth=4, max_borrow=1))
    assert pub.publish(np.full(8, 1, dtype=np.uint8)) == flux.Published.Ok
    assert pub.publish(np.full(8, 2, dtype=np.uint8)) == flux.Published.Ok

    held = sub.take()
    assert held is not None
    t0 = time.monotonic()
    assert sub.take_blocking(1_000_000_000) is None
    assert time.monotonic() - t0 < 0.2, "parked instead of reporting the exhausted lease"

    del held
    assert sub.take_blocking(1_000_000_000) is not None


def test_an_exhausted_lease_is_visible_and_counted():
    # None from a held lease and None from an idle stream are the same value to the caller, and
    # neither .lost nor .dropped moves for the first one. can_borrow answers it before the call,
    # refused counts it after; without both, a retry loop here just burns CPU.
    pub = flux.Publisher("/pytest/loan/refused", slot_size=64, slot_count=4, fingerprint=FP)
    sub = flux.Subscription(
        "/pytest/loan/refused", fingerprint=FP, qos=flux.QoS(depth=4, max_borrow=1))
    assert pub.publish(np.full(8, 1, dtype=np.uint8)) == flux.Published.Ok

    assert sub.can_borrow is True
    assert sub.refused.total == 0

    held = sub.take()
    assert held is not None
    assert sub.can_borrow is False
    assert sub.take() is None
    assert sub.refused.max_borrow == 1
    assert sub.refused.total == 1
    assert sub.lost == 0

    del held
    assert sub.can_borrow is True
    assert sub.refused.max_borrow == 1  # cumulative, not a level


def test_an_idle_stream_is_not_counted_as_refused():
    sub = flux.Subscription("/pytest/loan/idle", fingerprint=FP)
    assert sub.take() is None
    assert sub.peek() is None
    assert sub.refused.total == 0


def test_loan_rejects_a_shape_that_overflows_the_byte_count():
    # itemsize * prod(shape) is u64: unchecked it wraps to a small value and slips past the
    # slot_size cap, leaving a Loan that claims 0 bytes over a huge shape.
    pub = flux.Publisher("/pytest/loan/ovf", slot_size=1 << 20, slot_count=4, fingerprint=FP)
    with pytest.raises(ValueError):
        pub.loan((2**32, 2**32), dtype="uint8")


def test_take_blocking_recovers_from_a_publisher_restart():
    # A restart binds the name to a new object. An unbounded park slept on the old segment's
    # wake word, which nothing will ever bump again -- take() has to run for the subscription
    # to notice the swap, so the park is capped.
    topic = "/pytest/loan/restart"
    pub = flux.Publisher(topic, slot_size=64, slot_count=2, fingerprint=FP)
    sub = flux.Subscription(topic, fingerprint=FP)
    assert pub.publish(np.full(8, 1, dtype=np.uint8)) == flux.Published.Ok
    assert sub.take() is not None

    del pub
    gc.collect()
    pub2 = flux.Publisher(topic, slot_size=64, slot_count=2, fingerprint=FP)

    for _ in range(40):
        pub2.publish(np.full(8, 2, dtype=np.uint8))
        v = sub.take_blocking(150_000_000)
        if v is not None:
            assert int(v[0]) == 2
            return
    raise AssertionError("take_blocking never saw the restarted publisher")


# X-020. Both ends report the domain they resolved, and both resolve it the one way core does.
# Previously a core caller took "0" while this binding took ROS_DOMAIN_ID, so on a host with a
# domain set the two never met and neither said anything. The name is what decides whether they
# meet, so the check ends there rather than at the string.
def test_endpoints_report_the_domain_they_resolved_and_core_agrees():
    pub = flux.Publisher("/pytest/domain", slot_size=1 << 16, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/domain", fingerprint=FP)

    core = flux.process_domain()
    assert pub.domain == core
    assert sub.domain == core
    assert pub.segment_name == sub.segment_name
    assert f".s{core}." in pub.segment_name


# resolve_domain is a query and follows the environment; process_domain is the answer names are
# built from and does not. rcl latches ROS_DOMAIN_ID the same way, so one process cannot have its
# ROS half in one domain and its flux half in another.
def test_process_domain_is_latched_while_resolve_domain_follows_the_environment(monkeypatch):
    latched = flux.process_domain()

    monkeypatch.delenv("FLUX_DOMAIN", raising=False)
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    assert flux.resolve_domain() == "7"
    assert flux.resolve_domain(None) == "0"  # the opt-out still opts out
    assert flux.process_domain() == latched

    pub = flux.Publisher("/pytest/domain/latched", slot_size=4096, slot_count=2, fingerprint=FP)
    assert pub.domain == latched
    assert f".s{latched}." in pub.segment_name


def test_subscription_drops_a_dead_publisher_mapping():
    pub = flux.Publisher("/pytest/orphan", slot_size=1 << 16, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/orphan", fingerprint=FP)
    assert sub.attached

    del pub
    gc.collect()
    for _ in range(64):
        if not sub.attached:
            break
        sub.take()
    assert not sub.attached

    pub = flux.Publisher("/pytest/orphan", slot_size=1 << 16, slot_count=4, fingerprint=FP)
    sub.take()  # re-attaches to the new stream; nothing to deliver yet
    assert sub.attached
    assert pub.publish(np.ones(16, dtype=np.uint8)) == flux.Published.Ok
    assert sub.take() is not None


def test_take_blocking_delivers_keyboard_interrupt():
    # A pending SIGINT must surface as KeyboardInterrupt within one park slice, not sleep until
    # the next frame arrives and then corrupt the view construction on the way out.
    import signal
    import subprocess
    import sys
    import time

    code = (
        "import flux\n"
        "pub = flux.Publisher('/pytest/zc/sigint', slot_size=4096, slot_count=4, fingerprint=1)\n"
        "sub = flux.Subscription('/pytest/zc/sigint', fingerprint=1)\n"
        "print('ready', flush=True)\n"
        "try:\n"
        "    sub.take_blocking(-1)\n"
        "    print('frame', flush=True)\n"
        "except KeyboardInterrupt:\n"
        "    print('kbint', flush=True)\n"
    )
    p = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)
    try:
        assert p.stdout.readline().strip() == "ready"
        time.sleep(0.3)  # well inside the infinite park
        p.send_signal(signal.SIGINT)
        out, _ = p.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        p.kill()
        raise AssertionError("SIGINT did not interrupt take_blocking(-1)")
    finally:
        if p.poll() is None:
            p.kill()
    assert "kbint" in out


def test_restart_is_picked_up_without_a_stall_run():
    # Rotation is a seqlock read of a signpost page the subscriber keeps mapped, so a restart is
    # noticed on the next stalled take rather than after a run of them. Volatile durability means
    # the frame published before the consumer rejoins is legitimately missed, so the restarted
    # publisher keeps publishing -- what is pinned here is that a small, bounded number of takes
    # is enough (it was 10 behind the old threshold, 3 now).
    import gc

    import numpy as np

    topic = "/pytest/zc/fastreattach"
    pub = flux.Publisher(topic, slot_size=4096, slot_count=4, fingerprint=1)
    sub = flux.Subscription(topic, fingerprint=1)
    pub.publish(np.array([1], dtype=np.uint8))
    assert sub.take() is not None

    del pub
    gc.collect()
    pub2 = flux.Publisher(topic, slot_size=4096, slot_count=4, fingerprint=1)

    takes = 0
    while True:
        pub2.publish(np.array([2], dtype=np.uint8))
        takes += 1
        v = sub.take()
        if v is not None and int(v[0]) == 2:
            break
        assert takes < 100, "the subscriber never followed the rotation"
    assert takes <= 5, f"restart took {takes} takes to surface; the cheap rotation check regressed"


# bfloat16: the one dtype flux carries that numpy has no name for. These pin the shape of
# that exception -- what a caller gets, and what it is refused -- because the ordinary numpy path
# cannot express it and would otherwise hand back u2 while calling it bf16.


def _bf16_bits(x):
    """float32 -> bfloat16 bit pattern, round-to-nearest-even. Keeps the test free of ml_dtypes."""
    u = np.asarray(x, dtype=np.float32).view(np.uint32)
    return ((u + ((u >> 16) & 1) + 0x7FFF) >> 16).astype(np.uint16)


def _from_bf16_bits(b):
    return (np.asarray(b).astype(np.uint32) << 16).view(np.float32)


def test_bfloat16_roundtrips_through_bits():
    pub = flux.Publisher("/pytest/bf16", slot_size=1 << 16, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/bf16", fingerprint=FP)

    vals = np.array([1.0, -2.5, 3.75, 100.0], dtype=np.float32)
    loan = pub.loan((4,), dtype="bfloat16")
    assert loan.bits.dtype == np.uint16  # equal width, unsigned: the caller supplies the meaning
    assert loan.bits.flags.writeable is True
    loan.bits[:] = _bf16_bits(vals)
    assert loan.commit() == flux.Published.Ok

    v = sub.take()
    assert isinstance(v, flux.Frame)  # not a numpy array: numpy cannot name this dtype
    assert v.dtype == "bfloat16"
    assert v.shape == (4,)
    assert v.nbytes == 8
    assert v.bits.dtype == np.uint16
    assert v.bits.flags.writeable is False
    np.testing.assert_allclose(_from_bf16_bits(v.bits), vals, rtol=0.01)


def test_bfloat16_is_reachable_through_dlpack_and_not_through_numpy():
    pub = flux.Publisher("/pytest/bf16dl", slot_size=1 << 16, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/bf16dl", fingerprint=FP)

    loan = pub.loan((2,), dtype="bfloat16")
    assert loan.__dlpack_device__() == (1, 0)  # kDLCPU on a host publisher
    with pytest.raises(RuntimeError, match="numpy"):
        loan.array  # .array promises numpy, which has no bfloat16
    loan.bits[:] = _bf16_bits([1.0, 2.0])
    assert loan.commit() == flux.Published.Ok

    v = sub.take()
    assert v.__dlpack_device__() == (1, 0)
    # A fresh capsule per call, so a consumer that rejects one can still be handed another.
    assert type(v.__dlpack__(stream=None)).__name__ == "PyCapsule"
    assert type(v.__dlpack__()).__name__ == "PyCapsule"
    # numpy is the consumer that cannot take it, and it must fail rather than reinterpret.
    with pytest.raises(Exception):
        np.from_dlpack(v)


def test_bfloat16_host_frame_needs_no_scope():
    # A GPU frame is scope-bound because releasing it fences a stream. This one is a host frame
    # that merely happens to be unnameable, so it releases on collection like the numpy view.
    pub = flux.Publisher("/pytest/bf16domain", slot_size=1 << 16, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/bf16domain", fingerprint=FP)
    pub.loan((2,), dtype="bfloat16").commit()

    v = sub.take()
    assert v.bits.shape == (2,)  # readable without `with`
    with v as scoped:
        assert scoped.bits.shape == (2,)
    with pytest.raises(RuntimeError, match="already released"):
        v.bits


def test_loan_rejects_a_dtype_neither_numpy_nor_flux_knows():
    pub = flux.Publisher("/pytest/baddt", slot_size=1 << 16, slot_count=2, fingerprint=FP)
    with pytest.raises(Exception):
        pub.loan((2,), dtype="complex64")


def test_publish_and_commit_report_the_outcome_not_a_bool():
    """A dropped frame and a leaked slot must not arrive as the same answer.

    commit() used to return False for both "already consumed" and a failed fence, and a fence
    failure costs the slot for good. flux.Published says which, and flux.faulted(p) folds the
    five values back into the one question a publisher actually asks.
    """
    pub = flux.Publisher("/pytest/zc/published", slot_size=64, slot_count=1, fingerprint=FP)
    sub = flux.Subscription("/pytest/zc/published", fingerprint=FP)

    a = np.zeros(8, dtype=np.uint8)
    assert pub.publish(a) == flux.Published.Ok
    assert not flux.faulted(flux.Published.Ok)
    assert bool(flux.Published.Ok) is True

    held = sub.take()  # the only slot is borrowed, so the next publish is dropped
    assert held is not None
    dropped = pub.publish(a)
    assert dropped == flux.Published.Backpressure
    assert not flux.faulted(dropped), "backpressure is a rate, not a fault"
    assert bool(dropped) is False, "only Ok is truthy, so `if not p` reads as written"
    assert pub.dropped == 1
    del held

    loan = pub.loan((8,), dtype="uint8")
    assert loan.commit() == flux.Published.Ok
    spent = loan.commit()
    assert spent == flux.Published.TooLarge, "a spent handle must not report Ok"
    assert flux.faulted(spent)


def test_publish_raises_for_what_it_can_see_in_the_argument():
    # The engine outcomes come back as values; a bad argument raises instead, where the message
    # can name the path that does work.
    pub = flux.Publisher("/pytest/zc/argcheck", slot_size=16, slot_count=2, fingerprint=FP)
    with pytest.raises(ValueError, match="slot_size"):
        pub.publish(np.zeros(64, dtype=np.uint8))


# A pointer must not outlive the permission that made it valid. Both directions of that used to be
# reachable through the documented API and corrupted live frames without any error.
def test_a_loan_array_cannot_write_after_commit():
    pub = flux.Publisher("/pytest/zc/revoke", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/zc/revoke", fingerprint=FP)

    loan = pub.loan((8,), dtype="uint8")
    arr = loan.array
    arr[:] = 0xAA
    assert loan.commit() == flux.Published.Ok

    v = sub.take()
    assert v is not None
    assert bytes(v[:4]) == b"\xaa\xaa\xaa\xaa"

    # The bytes belong to a published frame now, and this array still aliases them.
    with pytest.raises(ValueError, match="read-only"):
        arr[:] = 0xBB
    assert bytes(v[:4]) == b"\xaa\xaa\xaa\xaa", "a committed frame was rewritten through the loan"


def test_a_host_frame_view_keeps_its_bytes_after_the_with_block():
    # bf16 comes back as flux.Frame rather than a numpy view. Leaving the `with` ends this
    # object's borrow, but an array it already handed out still points at the slot, so the borrow
    # has to outlive the block for as long as that array does.
    pub = flux.Publisher("/pytest/zc/anchor", slot_size=4096, slot_count=2, fingerprint=FP)
    sub = flux.Subscription("/pytest/zc/anchor", fingerprint=FP)

    loan = pub.loan((4,), dtype="bfloat16")
    loan.bits[:] = _bf16_bits([1.0, 1.0, 1.0, 1.0])
    assert loan.commit() == flux.Published.Ok

    frame = sub.take()
    with frame as f:
        escaped = f.bits
        first = _from_bf16_bits(escaped)[0]
    assert frame.released
    assert first == 1.0

    # Refill the ring. Without the anchor the escaped array reads whatever landed in the slot.
    for value in (2.0, 3.0, 4.0, 5.0, 6.0):
        ln = pub.loan((4,), dtype="bfloat16")
        if ln is None:
            continue
        ln.bits[:] = _bf16_bits([value] * 4)
        ln.commit()
    assert _from_bf16_bits(escaped)[0] == 1.0, "the view read a frame published after its borrow"
