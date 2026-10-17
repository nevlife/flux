import gc
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node

import flux2

SMALL = {"slot_size": 4096, "slot_count": 4}


def topic(name):
    return f"/t{os.getpid()}_{name}"


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = Node("flux2_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_qos_fields_left_out_take_the_core_default():
    d = flux2.QoS()
    q = flux2.QoS(depth=d.depth + 3)
    assert (q.depth, q.replay, q.max_borrow) == (d.depth + 3, d.replay, d.max_borrow)
    q = flux2.QoS(max_borrow=d.max_borrow + 1)
    assert (q.depth, q.replay, q.max_borrow) == (d.depth, d.replay, d.max_borrow + 1)


def test_a_frame_reads_back_as_a_read_only_view(node):
    pub = flux2.Publisher(node, topic("rt"), **SMALL)
    sub = flux2.Subscription(node, topic("rt"))
    sent = np.arange(12, dtype=np.float32).reshape(3, 4)
    assert pub.publish(sent) == flux2.Published.OK
    got = sub.take()
    assert got.dtype == np.float32 and got.shape == (3, 4)
    np.testing.assert_array_equal(got, sent)
    with pytest.raises(ValueError):
        got[0, 0] = 1
    assert sub.take() is None


def test_bytes_and_every_dtype_round_trip(node):
    pub = flux2.Publisher(node, topic("dtypes"), **SMALL)
    sub = flux2.Subscription(node, topic("dtypes"), qos=flux2.QoS(depth=4, max_borrow=2))
    assert pub.publish(b"abc")
    np.testing.assert_array_equal(sub.take(), np.frombuffer(b"abc", np.uint8))
    for t in ["u1", "i1", "u2", "i2", "u4", "i4", "u8", "i8", "f2", "f4", "f8"]:
        a = np.arange(5).astype(t)
        assert pub.publish(a), t
        got = sub.take()
        assert got.dtype == np.dtype(t), t
        np.testing.assert_array_equal(got, a)
    assert pub.publish(np.float64(2.5))
    assert sub.take().shape == ()


def test_a_held_view_holds_its_slot_until_collected(node):
    pub = flux2.Publisher(node, topic("hold"), slot_size=64, slot_count=2)
    sub = flux2.Subscription(node, topic("hold"), qos=flux2.QoS(depth=2, max_borrow=2))
    assert pub.publish(b"a")
    assert pub.publish(b"b")
    held = [sub.take(), sub.take()]
    del sub
    assert pub.publish(b"c") == flux2.Published.BACKPRESSURE
    assert bytes(held[0]) == b"a"
    del held
    gc.collect()
    assert pub.publish(b"c")


def test_a_loan_publishes_only_on_commit_and_seals_its_arrays(node):
    pub = flux2.Publisher(node, topic("loan"), slot_size=64, slot_count=2)
    sub = flux2.Subscription(node, topic("loan"), qos=flux2.QoS(depth=2))
    loan = pub.loan((2, 3), "f4")
    a = loan.array
    a[:] = 7
    assert sub.take() is None
    assert loan.commit()
    assert not loan
    with pytest.raises(ValueError):
        a[0, 0] = 1
    np.testing.assert_array_equal(sub.take(), np.full((2, 3), 7, np.float32))

    whole = pub.loan()
    assert whole.capacity == 64 and whole.array.shape == (64,)
    whole.array[:3] = [1, 2, 3]
    assert whole.commit(nbytes=3)
    np.testing.assert_array_equal(sub.take(), [1, 2, 3])


def test_a_refused_commit_ends_the_loan_as_in_cpp(node):
    pub = flux2.Publisher(node, topic("loanbad"), slot_size=64, slot_count=2)
    for loan, nbytes in ((pub.loan((4,), "f4"), 3), (pub.loan((2, 2), "f4"), 8)):
        a = loan.array
        assert loan.commit(nbytes=nbytes) == flux2.Published.TOO_LARGE
        assert not loan
        with pytest.raises(ValueError):
            a[0] = 1
    # Both slots are back in the ring.
    assert pub.loan() and pub.loan()


def test_publish_refusals_are_those_of_cpp(node):
    pub = flux2.Publisher(node, topic("pubbad"), **SMALL)
    assert pub.publish(np.zeros((1,) * 9, np.uint8)) == flux2.Published.TOO_LARGE
    for bad in (np.zeros(2, np.complex64), np.zeros(2, bool)):
        with pytest.raises(TypeError):
            pub.publish(bad)
    with pytest.raises(TypeError):
        pub.loan((2,), "c8")


def test_a_loan_in_a_foreign_byte_order_is_refused(node):
    pub = flux2.Publisher(node, topic("loanbe"), slot_size=64, slot_count=2)
    with pytest.raises(TypeError):
        pub.publish(np.arange(4, dtype=">u4"))
    with pytest.raises(TypeError, match="native byte order"):
        pub.loan((4,), ">u4")


def test_a_loan_past_the_slot_is_refused(node):
    pub = flux2.Publisher(node, topic("loanbig"), slot_size=64, slot_count=2)
    with pytest.raises(ValueError, match="exceeds slot_size"):
        pub.loan((17,), "f4")
    with pytest.raises(ValueError, match="overflows"):
        pub.loan((1 << 62, 8), "u1")
    assert pub.loan((16,), "f4").commit()


def test_a_loan_array_outlives_its_publisher(node):
    pub = flux2.Publisher(node, topic("loanlife"), slot_size=1 << 20, slot_count=2)
    loan = pub.loan((16,))
    a = loan.array
    a[:] = 5
    assert loan.commit()
    loan.abort()
    del pub, loan
    gc.collect()
    assert a.sum() == 80
    # The array holds the memory, not the publisher: the topic is free for a new one.
    flux2.Publisher(node, topic("loanlife"), slot_size=64, slot_count=2)


@pytest.mark.filterwarnings("ignore:This process .* is multi-threaded")
def test_a_subscription_inherited_across_fork_refuses_to_wait(node):
    pub = flux2.Publisher(node, topic("fork"), **SMALL)
    sub = flux2.Subscription(node, topic("fork"))
    assert pub.publish(b"a")
    assert sub.take() is not None
    pid = os.fork()
    if pid == 0:
        try:
            sub.take_blocking()
        except RuntimeError:
            os._exit(0)
        os._exit(1)
    _, status = os.waitpid(pid, 0)
    assert os.waitstatus_to_exitcode(status) == 0
    assert pub.publish(b"b")
    assert bytes(sub.take()) == b"b"


def test_misuse_fails_with_its_status(node):
    pub = flux2.Publisher(node, topic("misuse"), **SMALL)
    with pytest.raises(flux2.Error) as e:
        flux2.Publisher(node, topic("misuse"), **SMALL)
    assert e.value.status == flux2.Status.PUBLISHER_EXISTS
    with pytest.raises(flux2.Error) as e:
        flux2.Subscription(node, topic("misuse"), qos=flux2.QoS(depth=2, replay=3))
    assert e.value.status == flux2.Status.INVALID_ARGUMENT
    assert pub.publish(np.zeros(5000, np.uint8)) == flux2.Published.TOO_LARGE
    assert flux2.faulted(flux2.Published.TOO_LARGE)
    assert not flux2.faulted(flux2.Published.BACKPRESSURE)


def test_the_publisher_sets_how_many_processes_may_subscribe(node):
    pub = flux2.Publisher(node, topic("maxsubs"), max_subscribers=64, **SMALL)
    assert flux2.read_channel_stats(pub.signpost_name).max_subscribers == 64
    with pytest.raises(flux2.Error) as e:
        flux2.Publisher(node, topic("maxbad"), max_subscribers=0, **SMALL)
    assert e.value.status == flux2.Status.INVALID_ARGUMENT


def test_topics_resolve_against_the_node(node):
    ns = Node("n", namespace=f"/ns{os.getpid()}")
    pub = flux2.Publisher(ns, "cam", **SMALL)
    assert pub.topic_name == f"/ns{os.getpid()}/cam"
    sub = flux2.Subscription(node, pub.topic_name)
    assert sub.signpost_name == pub.signpost_name
    ns.destroy_node()


def test_endpoints_are_listed_with_their_node(node):
    pub = flux2.Publisher(node, topic("listed"), fingerprint=7, **SMALL)
    sub = flux2.Subscription(node, pub.topic_name, fingerprint=7)  # noqa: F841 listed while alive
    (t,) = [t for t in flux2.enumerate_topics() if t.signpost == pub.signpost_name]
    assert (t.key, t.key_exact, t.fingerprint) == (pub.topic_name, True, 7)
    assert sorted(e.publisher for e in t.endpoints) == [False, True]
    assert {(e.pid, e.label) for e in t.endpoints} == {
        (os.getpid(), node.get_fully_qualified_name())
    }
    stats = flux2.read_channel_stats(pub.signpost_name)
    assert stats.live and stats.slot_size == SMALL["slot_size"]
    assert stats.max_subscribers == flux2.DEFAULT_MAX_SUBSCRIBERS
    assert not flux2.read_channel_stats(pub.signpost_name + "x").live


def test_take_blocking_waits_for_a_late_publisher(node):
    sub = flux2.Subscription(node, topic("late"))
    assert not sub.attached and sub.take() is None
    held = []

    def late():
        time.sleep(0.15)
        held.append(flux2.Publisher(node, topic("late"), **SMALL))
        time.sleep(0.15)
        held[0].publish(b"hi")

    t = threading.Thread(target=late)
    t.start()
    got = sub.take_blocking(timeout_ns=2_000_000_000)
    t.join()
    assert bytes(got) == b"hi"
    assert sub.take_blocking(timeout_ns=10_000_000) is None


def test_take_blocking_returns_as_soon_as_a_frame_lands(node):
    pub = flux2.Publisher(node, topic("wake"), **SMALL)
    sub = flux2.Subscription(node, topic("wake"))
    assert pub.publish(b"0") and sub.take() is not None
    late = []
    for _ in range(9):
        sent = []

        def publish(sent=sent):
            time.sleep(0.02)
            sent.append(time.monotonic())
            pub.publish(b"x")

        t = threading.Thread(target=publish)
        t.start()
        got = sub.take_blocking(timeout_ns=2_000_000_000)
        late.append(time.monotonic() - sent[0])
        t.join()
        del got
    # Woken, the wait returns in microseconds; polling, it would sleep out its 100 ms slice.
    assert sorted(late)[4] < 0.02


def test_take_blocking_ends_when_the_context_shuts_down(node):
    # As rclpy's wait_for_message: a shut down context ends the wait, attached or not.
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    other = Node("flux2_ctx", context=ctx)
    pub = flux2.Publisher(node, topic("ctx_on"), **SMALL)
    attached = flux2.Subscription(other, topic("ctx_on"))
    waiting = flux2.Subscription(other, topic("ctx_off"))
    got = {}
    threads = [
        threading.Thread(target=lambda: got.update(a=attached.take_blocking())),
        threading.Thread(target=lambda: got.update(w=waiting.take_blocking(3_000_000_000))),
    ]
    for t in threads:
        t.start()
    time.sleep(0.15)
    rclpy.shutdown(context=ctx)
    for t in threads:
        t.join(timeout=1)
    alive = [t.is_alive() for t in threads]
    pub.publish(b"x")  # ends the wait if the shutdown did not
    for t in threads:
        t.join()
    assert alive == [False, False]
    assert got == {"a": None, "w": None}


def test_sigterm_ends_a_take_blocking_loop():
    # rclpy's SIGTERM handler shuts the context down; Ctrl-C raises KeyboardInterrupt as before.
    code = (
        "import rclpy, flux2\n"
        "rclpy.init()\n"
        "sub = flux2.Subscription(rclpy.create_node('term'), '/term_take')\n"
        "print('ready', flush=True)\n"
        "while rclpy.ok():\n"
        "    sub.take_blocking()\n"
        "print('returned', flush=True)\n"
    )
    p = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)
    assert p.stdout.readline() == "ready\n"
    time.sleep(0.2)
    p.send_signal(signal.SIGTERM)
    try:
        out, _ = p.communicate(timeout=5)
    finally:
        p.kill()
    assert p.returncode == 0 and out == "returned\n"


def test_take_blocking_is_none_at_once_while_max_borrow_views_are_held(node):
    pub = flux2.Publisher(node, topic("heldtb"), **SMALL)
    sub = flux2.Subscription(node, topic("heldtb"), qos=flux2.QoS(depth=4, max_borrow=1))
    assert pub.publish(b"a") and pub.publish(b"b")
    held = sub.take()
    start = time.monotonic()
    done = threading.Event()
    t = threading.Thread(target=lambda: (sub.take_blocking(), done.set()), daemon=True)
    t.start()
    assert done.wait(1.0)
    assert sub.take_blocking(timeout_ns=2_000_000_000) is None
    assert time.monotonic() - start < 0.5
    del held


def test_lost_counts_over_the_subscriptions_life(node):
    pub = flux2.Publisher(node, topic("lost"), **SMALL)
    sub = flux2.Subscription(node, topic("lost"), qos=flux2.QoS(depth=1))
    for _ in range(3):
        assert pub.publish(b"a")
    assert sub.take() is not None and sub.lost == 2
    del pub
    gc.collect()
    end = time.monotonic() + 2
    while sub.attached and time.monotonic() < end:
        sub.take()
    assert not sub.attached and sub.lost == 2
    pub = flux2.Publisher(node, topic("lost"), **SMALL)
    while not sub.attached and time.monotonic() < end + 2:
        sub.take()
    for _ in range(3):
        assert pub.publish(b"b")
    assert sub.take() is not None and sub.lost == 4


def test_exit_with_a_thread_in_take_blocking_is_quiet(node):
    code = (
        "import threading, time, rclpy, flux2\n"
        "rclpy.init()\n"
        "sub = flux2.Subscription(rclpy.create_node('quiet'), '/quiet_exit')\n"
        "threading.Thread(target=sub.take_blocking, daemon=True).start()\n"
        "time.sleep(0.2)\n"
    )
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=20)
    assert p.returncode == 0 and "leaked" not in p.stderr


def test_a_second_thread_in_one_subscription_is_refused(node):
    sub = flux2.Subscription(node, topic("two"))
    # With no publisher, take_blocking stays inside the subscription until its timeout.
    t = threading.Thread(target=sub.take_blocking, kwargs={"timeout_ns": 300_000_000})
    t.start()
    time.sleep(0.05)
    with pytest.raises(RuntimeError, match="two threads"):
        sub.take()
    t.join()
    assert sub.take() is None
