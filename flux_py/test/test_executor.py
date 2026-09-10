"""flux_py Executor tests: one wait, many subscriptions.

The executor's job is to block on every registered channel at once and dispatch a callback per
frame. These tests cover what that promise means in practice -- frames actually arrive, two
channels really are served by one spin, QoS depth decides how much of a burst arrives, a late
publisher is picked up, and spin() can be stopped from inside a callback.
"""

import gc
import threading
import time

import numpy as np

import flux

FP = 0xE7EC0DE


def test_dispatches_a_frame():
    pub = flux.Publisher("/pytest/ex/one", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/ex/one", fingerprint=FP)

    seen = []
    ex = flux.Executor()
    ex.add(sub, lambda v: seen.append(np.array(v)))
    assert len(ex) == 1

    a = np.arange(64, dtype=np.uint8)
    assert pub.publish(a) == flux.Published.Ok

    assert ex.spin_once(timeout_ns=500_000_000) == 1
    np.testing.assert_array_equal(seen[0], a)


def test_no_publisher_times_out_without_dispatching():
    sub = flux.Subscription("/pytest/ex/absent", fingerprint=FP)
    ex = flux.Executor()
    ex.add(sub, lambda v: None)

    t0 = time.monotonic()
    assert ex.spin_once(timeout_ns=50_000_000) == 0
    assert time.monotonic() - t0 >= 0.02  # it really blocked rather than spinning


def test_two_channels_share_one_wait():
    # The whole point of an executor: one spin_once services both topics. If the wait were
    # per-channel, only the channel that happened to be waited on would deliver.
    pub_a = flux.Publisher("/pytest/ex/a", slot_size=4096, slot_count=4, fingerprint=FP)
    pub_b = flux.Publisher("/pytest/ex/b", slot_size=4096, slot_count=4, fingerprint=FP)
    sub_a = flux.Subscription("/pytest/ex/a", fingerprint=FP)
    sub_b = flux.Subscription("/pytest/ex/b", fingerprint=FP)

    got = {"a": 0, "b": 0}
    ex = flux.Executor()
    ex.add(sub_a, lambda v: got.__setitem__("a", got["a"] + 1))
    ex.add(sub_b, lambda v: got.__setitem__("b", got["b"] + 1))

    assert pub_a.publish(np.zeros(16, dtype=np.uint8)) == flux.Published.Ok
    assert pub_b.publish(np.ones(16, dtype=np.uint8)) == flux.Published.Ok

    dispatched = 0
    for _ in range(10):
        dispatched += ex.spin_once(timeout_ns=200_000_000)
        if got["a"] and got["b"]:
            break
    assert got == {"a": 1, "b": 1}
    assert dispatched == 2


def test_priority_orders_the_pass():
    # priority reaches flux::Executor::add through the binding: the higher source is visited
    # first in a pass regardless of the order it was registered in. Ordering only -- nothing
    # preempts, so the whole visit runs before the next one starts.
    pub_lo = flux.Publisher("/pytest/ex/prio_lo", slot_size=4096, slot_count=4, fingerprint=FP)
    pub_hi = flux.Publisher("/pytest/ex/prio_hi", slot_size=4096, slot_count=4, fingerprint=FP)
    sub_lo = flux.Subscription("/pytest/ex/prio_lo", fingerprint=FP)
    sub_hi = flux.Subscription("/pytest/ex/prio_hi", fingerprint=FP)

    order = []
    ex = flux.Executor()
    ex.add(sub_lo, lambda v: order.append("lo"))
    ex.add(sub_hi, lambda v: order.append("hi"), priority=10)

    assert pub_lo.publish(np.zeros(16, dtype=np.uint8)) == flux.Published.Ok
    assert pub_hi.publish(np.ones(16, dtype=np.uint8)) == flux.Published.Ok

    for _ in range(10):
        ex.spin_once(timeout_ns=200_000_000)
        if len(order) == 2:
            break
    assert order == ["hi", "lo"]


def test_depth_preserves_publish_order():
    # depth=8 lets the consumer sit up to 8 frames behind, so a burst published between two
    # spins arrives in order instead of collapsing to the newest frame.
    pub = flux.Publisher("/pytest/ex/ordered", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/ex/ordered", fingerprint=FP, qos=flux.QoS(depth=8))

    seen = []
    ex = flux.Executor()
    ex.add(sub, lambda v: seen.append(int(v[0])))

    for i in range(5):
        assert pub.publish(np.full(8, i, dtype=np.uint8)) == flux.Published.Ok

    for _ in range(10):
        ex.spin_once(timeout_ns=200_000_000)
        if len(seen) >= 5:
            break
    assert seen == [0, 1, 2, 3, 4]


def test_depth_one_collapses_a_burst():
    # The default depth=1 never falls behind, so a burst published between spins yields the
    # newest frame only. This is the contrast that makes depth meaningful.
    pub = flux.Publisher("/pytest/ex/latest", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/ex/latest", fingerprint=FP)

    seen = []
    ex = flux.Executor()
    ex.add(sub, lambda v: seen.append(int(v[0])))

    for i in range(5):
        assert pub.publish(np.full(8, i, dtype=np.uint8)) == flux.Published.Ok

    assert ex.spin_once(timeout_ns=200_000_000) == 1
    assert seen == [4]
    assert sub.lost == 4  # the four it skipped getting there


def test_late_publisher_is_picked_up():
    # The subscription is registered before the segment exists; the executor must keep retrying
    # the attach instead of giving up at add() time.
    # transient_local(1): the attach necessarily lands after the publish, and volatile would
    # (correctly) join from there and see nothing.
    sub = flux.Subscription(
        "/pytest/ex/late", fingerprint=FP,
        qos=flux.QoS(durability=flux.TransientLocal(1)))
    seen = []
    ex = flux.Executor()
    ex.add(sub, lambda v: seen.append(int(v[0])))

    assert ex.spin_once(timeout_ns=20_000_000) == 0  # nothing to attach to yet

    pub = flux.Publisher("/pytest/ex/late", slot_size=4096, slot_count=4, fingerprint=FP)
    assert pub.publish(np.full(4, 42, dtype=np.uint8)) == flux.Published.Ok

    for _ in range(20):
        if ex.spin_once(timeout_ns=100_000_000):
            break
    assert seen == [42]


def test_publisher_restart_keeps_delivering():
    # A restart moves the subscription to a new segment, killing everything armed on the old
    # one. Before that was handled the channel went silent for good here.
    topic = "/pytest/ex/restart"
    pub = flux.Publisher(topic, slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription(topic, fingerprint=FP)

    seen = []
    ex = flux.Executor()
    ex.add(sub, lambda v: seen.append(int(v[0])))

    assert pub.publish(np.full(4, 1, dtype=np.uint8)) == flux.Published.Ok
    for _ in range(10):
        if ex.spin_once(timeout_ns=200_000_000):
            break
    assert seen == [1]

    del pub  # last publisher out unlinks the name
    gc.collect()

    pub2 = flux.Publisher(topic, slot_size=4096, slot_count=4, fingerprint=FP)
    for _ in range(100):
        pub2.publish(np.full(4, 2, dtype=np.uint8))
        ex.spin_once(timeout_ns=20_000_000)
        if len(seen) > 1:
            break
    assert seen[-1] == 2, "the executor went silent after the publisher restarted"


def test_spin_stops_from_callback():
    pub = flux.Publisher("/pytest/ex/stop", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/ex/stop", fingerprint=FP)

    ex = flux.Executor()
    calls = []

    def cb(v):
        calls.append(int(v[0]))
        ex.stop()

    ex.add(sub, cb)
    assert pub.publish(np.full(4, 9, dtype=np.uint8)) == flux.Published.Ok

    done = threading.Event()

    def run():
        ex.spin(tick_ns=50_000_000)
        done.set()

    t = threading.Thread(target=run)
    t.start()
    assert done.wait(timeout=5.0), "spin() did not return after stop()"
    t.join()
    assert calls == [9]


def test_stop_from_another_thread_breaks_a_blocking_spin():
    # No publisher at all, so spin() is parked in the kernel. stop() must break that wait
    # through the control eventfd rather than waiting out the tick.
    sub = flux.Subscription("/pytest/ex/idle", fingerprint=FP)
    ex = flux.Executor()
    ex.add(sub, lambda v: None)

    done = threading.Event()

    def run():
        ex.spin(tick_ns=10_000_000_000)  # 10 s tick: only stop() can end this promptly
        done.set()

    t = threading.Thread(target=run)
    t.start()
    time.sleep(0.1)
    ex.stop()
    assert done.wait(timeout=5.0), "stop() did not break the blocking wait"
    t.join()


def test_self_referencing_callback_is_collectable():
    # `lambda v: ex.stop()` is the ordinary way to write a stop condition, and it makes
    # Executor -> callback -> Executor a cycle. If the type were not GC-aware that cycle would
    # be uncollectable, pinning the Subscription and its shm mapping for the life of the
    # process. The sentinel is reachable only through the cycle, so its finalizer firing after
    # gc.collect() proves the whole cycle was reclaimed.
    finalized = []

    class Sentinel:
        def __del__(self):
            finalized.append(True)

    def build():
        ex = flux.Executor()
        sub = flux.Subscription("/pytest/ex/cycle", fingerprint=FP)
        sentinel = Sentinel()

        def cb(v):
            _ = sentinel  # a real closure capture: the cycle is what keeps this alive
            ex.stop()

        ex.add(sub, cb)
        return ex

    ex = build()
    del ex
    gc.collect()
    assert finalized, "the Executor <-> callback cycle was not collected"


def test_capacity_is_enforced():
    ex = flux.Executor(max_channels=1)
    ex.add(flux.Subscription("/pytest/ex/cap1", fingerprint=FP), lambda v: None)
    try:
        # std::length_error on the C++ side, which is what flux_cpp's test asserts too: one
        # refusal, spelled the way each language spells a bad argument.
        ex.add(flux.Subscription("/pytest/ex/cap2", fingerprint=FP), lambda v: None)
    except ValueError as e:
        assert "max_channels" in str(e)
    else:
        raise AssertionError("expected the executor to reject a channel past max_channels")


def test_add_rejects_a_non_subscription_with_type_error():
    # nb::cast raises std::bad_cast, which nanobind maps to RuntimeError -- an unhelpful message
    # for the common `ex.add(ros_sub, cb)` mistake. flux.ros.Executor.add_flux already raised
    # TypeError, so the two surfaces disagreed.
    ex = flux.Executor()
    try:
        ex.add(object(), lambda v: None)
    except TypeError as e:
        assert "Subscription" in str(e)
    else:
        raise AssertionError("expected TypeError for a non-Subscription")


def test_fallback_parker_wakes_without_a_tick(monkeypatch):
    # FLUX_DISABLE_IO_URING forces the parker-thread fallback (the path pre-6.7 kernels run).
    # A 5 s poll tick would gate the old fixed-tick polling; the parker must deliver in ~0.15 s.
    monkeypatch.setenv("FLUX_DISABLE_IO_URING", "1")
    pub = flux.Publisher("/pytest/ex/fbpark", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/ex/fbpark", fingerprint=FP)

    seen = []
    ex = flux.Executor(poll_tick_ns=5_000_000_000)
    ex.add(sub, lambda v: seen.append(int(v[0])))
    assert ex.uses_io_uring is False

    assert ex.spin_once(timeout_ns=0) == 0  # attach and start the parker

    def late_publish():
        time.sleep(0.15)
        pub.publish(np.array([7], dtype=np.uint8))

    t = threading.Thread(target=late_publish)
    t0 = time.monotonic()
    t.start()
    n = ex.spin_once(timeout_ns=-1)
    dt = time.monotonic() - t0
    t.join()

    assert n == 1 and seen == [7]
    assert dt < 2.0  # woken by the parker, not by the 5 s tick


def test_fallback_survives_publisher_restart(monkeypatch):
    monkeypatch.setenv("FLUX_DISABLE_IO_URING", "1")
    pub = flux.Publisher("/pytest/ex/fbrestart", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/ex/fbrestart", fingerprint=FP)

    seen = []
    ex = flux.Executor()
    ex.add(sub, lambda v: seen.append(int(v[0])))
    assert ex.uses_io_uring is False

    pub.publish(np.array([1], dtype=np.uint8))
    ex.spin_once(timeout_ns=500_000_000)
    before = len(seen)
    assert before == 1

    del pub
    gc.collect()
    pub = flux.Publisher("/pytest/ex/fbrestart", slot_size=4096, slot_count=4, fingerprint=FP)
    deadline = time.monotonic() + 10.0
    while len(seen) == before and time.monotonic() < deadline:
        pub.publish(np.array([2], dtype=np.uint8))
        ex.spin_once(timeout_ns=100_000_000)
    assert len(seen) > before


def test_interrupt_ends_at_most_one_call():
    # One interrupt() ends the call it targets and nothing later. A flag left set by the
    # infinite-wait path used to end the NEXT bounded call at its first spurious wake.
    pub = flux.Publisher("/pytest/ex/intr", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/ex/intr", fingerprint=FP)
    held = []
    ex = flux.Executor()
    ex.add(sub, held.append)

    while True:  # saturate max_borrow: from here a publish wakes the wait but delivers nothing
        pub.publish(np.zeros(8, dtype=np.uint8))
        if ex.spin_once(timeout_ns=200_000_000) == 0:
            break

    t = threading.Timer(0.2, ex.interrupt)
    t.start()
    t0 = time.monotonic()
    assert ex.spin_once(timeout_ns=-1) == 0  # ended by the interrupt
    assert time.monotonic() - t0 < 2.0
    t.join()

    t2 = threading.Timer(0.1, lambda: pub.publish(np.ones(8, dtype=np.uint8)))
    t2.start()
    t0 = time.monotonic()
    n = ex.spin_once(timeout_ns=600_000_000)  # undeliverable wake at 0.1 s, no interrupt
    dt = time.monotonic() - t0
    t2.join()
    assert n == 0
    assert dt >= 0.55  # blocked to its deadline: the previous call consumed its interrupt


def test_waiter_gate_balances_across_a_self_reattach():
    # The executor's +1 on ControlHeader.waiters must come off the segment it was placed on.
    # After the subscription re-attaches on its own (direct take()s between executor passes),
    # removing on the CURRENT segment would drive a live gate negative and publishers would
    # skip the wake syscall for every parked subscriber. waiters sits at offset 128
    # (segment_layout.hpp static_assert).
    import glob
    import struct

    pub = flux.Publisher("/pytest/ex/gate", slot_size=4096, slot_count=4, fingerprint=FP)

    signpost = pub.segment_name  # derived, not spelled out: the name shape is not this test's
    prefix = "/dev/shm" + signpost + "."  # unique segment = <signpost>.<pid>.<starttime>

    def gate():
        segs = sorted(glob.glob(prefix + "*"))
        assert segs
        with open(segs[-1], "rb") as f:
            f.seek(128)
            return struct.unpack("<i", f.read(4))[0]

    pub.publish(np.zeros(8, dtype=np.uint8))
    sub = flux.Subscription("/pytest/ex/gate", fingerprint=FP)
    ex = flux.Executor()
    ex.add(sub, lambda v: None)
    ex.spin_once(timeout_ns=200_000_000)
    if ex.uses_io_uring:
        assert gate() == 1  # armed: the executor announced itself

    del pub
    gc.collect()
    # Held, never read: the segment exists for exactly as long as this publisher does, and the
    # takes below are what re-attach to it.
    pub = flux.Publisher(  # noqa: F841
        "/pytest/ex/gate", slot_size=4096, slot_count=4, fingerprint=FP
    )
    for _ in range(12):  # direct takes: the stall probe re-attaches without the executor
        sub.take()
    del ex
    gc.collect()
    assert gate() == 0


# "spinning" and "stop requested" are two flags. A stop() that lands before spin() is
# entered must survive the gap -- the PartitionedExecutor parent stops a child and moves straight to join(), so
# nobody asks a second time.
def test_stop_before_spin_is_not_erased():
    sub = flux.Subscription("/pytest/exec/stop_first", fingerprint=0)
    ex = flux._flux.Executor(max_channels=1)
    ex.add(sub, lambda v: None)

    ex.stop()
    done = threading.Event()
    t = threading.Thread(target=lambda: (ex.spin(20_000_000), done.set()), daemon=True)
    t.start()
    assert done.wait(timeout=5.0), "spin() re-armed the run flag and swallowed the stop"
    t.join(timeout=5.0)


# Consuming the request on the way out is what keeps the executor reusable.
def test_the_same_executor_spins_again_after_stop():
    sub = flux.Subscription("/pytest/exec/reuse", fingerprint=0)
    ex = flux._flux.Executor(max_channels=1)
    ex.add(sub, lambda v: None)

    for _ in range(2):
        done = threading.Event()
        t = threading.Thread(target=lambda: (ex.spin(20_000_000), done.set()), daemon=True)
        t.start()
        while not ex.is_spinning:
            time.sleep(0.01)
        ex.stop()
        assert done.wait(timeout=5.0)
        t.join(timeout=5.0)
        assert not ex.is_spinning


def test_concurrent_spin_is_refused():
    sub = flux.Subscription("/pytest/exec/reentry", fingerprint=0)
    ex = flux._flux.Executor(max_channels=1)
    ex.add(sub, lambda v: None)

    done = threading.Event()
    t = threading.Thread(target=lambda: (ex.spin(20_000_000), done.set()), daemon=True)
    t.start()
    try:
        while not ex.is_spinning:
            time.sleep(0.01)
        try:
            ex.spin(20_000_000)
            raise AssertionError("a second concurrent spin() was allowed")
        except RuntimeError as exc:
            assert "already spinning" in str(exc)
    finally:
        ex.stop()
        assert done.wait(timeout=5.0)
        t.join(timeout=5.0)
