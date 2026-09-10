"""flux.ros.Executor: one rclpy spin loop serving both ROS messages and flux frames.

The point of the unified executor is that neither transport needs its own thread or its own
loop. These tests check exactly that -- a flux frame and a ROS message both reach callbacks on
the spin thread, and the flux side is genuinely event-driven (a frame published while the loop
is parked wakes it) rather than picked up by a polling tick.
"""

import gc
import threading
import time
import weakref

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

import flux  # noqa: E402
import flux.ros  # noqa: E402

FP = 0xB817DE


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node("flux_ros_exec_test")
    yield n
    n.destroy_node()
    rclpy.try_shutdown()  # a test may have shut the context down itself


def spin_in_thread(ex):
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    return t


def test_flux_frame_reaches_a_callback(node):
    pub = flux.Publisher("/pytest/rosex/a", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/rosex/a", fingerprint=FP)

    seen = []
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    ex.add_flux(sub, lambda v: seen.append(int(v[0])))

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while not seen and time.monotonic() < deadline:
            pub.publish(np.full(8, 7, dtype=np.uint8))
            time.sleep(0.02)
        assert seen and seen[0] == 7
    finally:
        ex.stop()
        t.join(timeout=5.0)


def test_ros_and_flux_share_one_loop(node):
    from std_msgs.msg import Int32

    pub = flux.Publisher("/pytest/rosex/b", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/rosex/b", fingerprint=FP)

    got = {"flux": 0, "ros": 0, "threads": set()}
    ex = flux.ros.Executor()
    ex.add_ros_node(node)

    def on_flux(v):
        got["flux"] += 1
        got["threads"].add(threading.get_ident())

    def on_ros(msg):
        got["ros"] += 1
        got["threads"].add(threading.get_ident())

    ex.add_flux(sub, on_flux)
    node.create_subscription(Int32, "/pytest/rosex/ros_topic", on_ros, 10)
    ros_pub = node.create_publisher(Int32, "/pytest/rosex/ros_topic", 10)

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 10.0
        while (not got["flux"] or not got["ros"]) and time.monotonic() < deadline:
            pub.publish(np.full(8, 1, dtype=np.uint8))
            ros_pub.publish(Int32(data=1))
            time.sleep(0.05)
        assert got["flux"] > 0, "no flux frame was dispatched"
        assert got["ros"] > 0, "no ROS message was dispatched"
        # Both transports must land on the SAME thread -- that is what "unified" means. If flux
        # were serviced by its own executor this set would have two entries.
        assert len(got["threads"]) == 1, f"callbacks ran on {len(got['threads'])} threads"
    finally:
        ex.stop()
        t.join(timeout=5.0)


def test_flux_wakes_a_parked_loop(node):
    # No ROS traffic at all, so the rclpy wait set has nothing to fire on. A flux frame must
    # still get through -- that is the guard condition doing its job. If flux were polled at the
    # spin tick this would still pass, so the test also bounds the latency well under a tick.
    pub = flux.Publisher("/pytest/rosex/c", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/rosex/c", fingerprint=FP)

    arrived = threading.Event()
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    ex.add_flux(sub, lambda v: arrived.set())

    t = spin_in_thread(ex)
    try:
        time.sleep(0.3)  # let the loop settle into its parked state
        assert pub.publish(np.full(8, 5, dtype=np.uint8)) == flux.Published.Ok
        assert arrived.wait(timeout=5.0), "a published frame never woke the parked loop"
    finally:
        ex.stop()
        t.join(timeout=5.0)


def test_stop_ends_spin(node):
    sub = flux.Subscription("/pytest/rosex/idle", fingerprint=FP)
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    ex.add_flux(sub, lambda v: None)

    done = threading.Event()

    def run():
        ex.spin()
        done.set()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(0.2)
    ex.stop()
    assert done.wait(timeout=5.0), "spin() did not return after stop()"
    t.join(timeout=5.0)


def test_add_ros_node_is_idempotent(node):
    # rclpy writes node.executor on add_node, so adding the same node twice through two
    # executors silently steals it from the first. Adding it twice here must be a no-op rather
    # than a second add_node.
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    ex.add_ros_node(node)
    assert node.executor is not None
    ex.close()


def test_registration_is_refused_while_spinning(node):
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    done = threading.Event()

    def run():
        ex.spin()
        done.set()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(0.2)
    with pytest.raises(RuntimeError):
        ex.add_ros_node(node)
    ex.stop()
    assert done.wait(timeout=5.0)
    t.join(timeout=5.0)
    ex.close()


def test_topic_is_resolved_against_the_node_namespace():
    # A relative topic means nothing on its own: the node's namespace decides the real name, and
    # flux derives the segment name from that. flux.Subscription alone cannot do this (no node),
    # which is why the relative form is rejected there and resolved here.
    rclpy.init()
    try:
        n = rclpy.create_node("resolver", namespace="/robot1")
        try:
            pub = flux.ros.Publisher(n, "image", slot_size=4096, slot_count=2, fingerprint=FP)
            sub = flux.ros.Subscription(n, "image", fingerprint=FP)
            assert flux.ros.resolve(n, "image") == "/robot1/image"
            # Same resolved name on both ends -> same segment -> they meet.
            assert pub.segment_name == sub.segment_name
            # And a namespace-less node would NOT meet it.
            assert pub.publish(np.full(4, 3, dtype=np.uint8)) == flux.Published.Ok
            v = sub.take()
            assert v is not None and int(v[0]) == 3
        finally:
            n.destroy_node()
    finally:
        rclpy.shutdown()


def test_relative_topic_without_a_node_is_rejected():
    # The failure this prevents: a relative name silently naming a different segment than the
    # C++ node it is supposed to pair with, so nothing ever arrives and nothing ever errors.
    with pytest.raises(ValueError):
        flux.Subscription("image", fingerprint=FP)


def test_spin_once_rejected_while_spinning(node):
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    t = spin_in_thread(ex)
    deadline = time.monotonic() + 5.0
    while not ex._running and time.monotonic() < deadline:
        time.sleep(0.01)
    try:
        with pytest.raises(RuntimeError):
            ex.spin_once()
    finally:
        ex.stop()
        t.join(timeout=5.0)
    assert not t.is_alive()


def test_frames_arrive_under_the_events_executor(node):
    """The bridge must run on any rclpy executor, not just the wait-set one.

    EventsExecutor has no wait set and rejects guard conditions, so a bridge built on one
    raises here instead of delivering. Tasks are executor-level, which is why this passes.
    """
    events = pytest.importorskip("rclpy.experimental")

    pub = flux.Publisher("/pytest/rosex/events", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/rosex/events", fingerprint=FP)

    seen = []
    ex = flux.ros.Executor(rclpy_executor=events.EventsExecutor())
    ex.add_ros_node(node)
    ex.add_flux(sub, lambda v: seen.append(int(v[0])))

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while not seen and time.monotonic() < deadline:
            pub.publish(np.full(8, 9, dtype=np.uint8))
            time.sleep(0.02)
        assert seen and seen[0] == 9
    finally:
        ex.stop()
        t.join(timeout=5.0)


def test_stop_is_idempotent_and_close_detaches(node):
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    t = spin_in_thread(ex)
    time.sleep(0.1)
    ex.stop()
    ex.stop()  # second stop must be a no-op, not an error
    t.join(timeout=5.0)
    assert not t.is_alive()

    ex.close()
    ex.close()  # idempotent
    with pytest.raises(RuntimeError):
        ex.spin()  # a closed executor refuses to spin rather than queue onto a detached node


@pytest.mark.parametrize("kind", ["wait_set", "events"])
def test_teardown_survives_an_external_shutdown(node, kind):
    """Ctrl-C order: the context dies first, and stop()/close() still have to run.

    Both wake the spin thread with create_task, and by then the context behind the rclpy
    executor is gone. Neither may raise -- teardown that throws leaves the bridge thread
    attached to a node the caller is about to destroy.
    """
    rclpy_executor = None
    if kind == "events":
        rclpy_executor = pytest.importorskip("rclpy.experimental").EventsExecutor()

    ex = flux.ros.Executor(rclpy_executor=rclpy_executor)
    ex.add_ros_node(node)
    raised = []

    def spin():
        try:
            ex.spin()
        except BaseException as e:  # noqa: BLE001 - the type is the assertion
            # The name, not the exception: its traceback holds the frames of everything on the
            # stack, and a flux Channel kept alive that way outlives the module that owns it.
            raised.append(type(e).__name__)

    t = threading.Thread(target=spin, daemon=True)
    t.start()
    time.sleep(0.2)

    rclpy.shutdown()
    t.join(timeout=5.0)
    assert not t.is_alive()
    # rclpy's own spin() reports an external shutdown rather than returning normally, and an
    # application distinguishes it from stop(). Passing it through keeps that distinction.
    assert raised == ["ExternalShutdownException"]

    ex.stop()
    ex.close()


def test_a_queued_task_does_not_pin_the_executor(node):
    """A task the spin thread never runs must not keep the channels mapped.

    The bridge queues a dispatch task per wake, and one queued as the context goes down is
    never run and never dropped -- the rclpy executor holds it, and an EventsExecutor is
    itself held by the context it registered a shutdown callback with. If that task carried a
    strong reference back here, close() would not be the end of the segment mapping.
    """
    ex = flux.ros.Executor()
    ex.add_ros_node(node)
    stranded = ex._exec.create_task(ex._dispatch_task)  # the object the bridge queues
    ex.stop()
    ex.close()

    ref = weakref.ref(ex)
    del ex
    gc.collect()
    assert ref() is None, "a queued task kept the executor alive past close()"
    stranded()  # and running it afterwards is a no-op, not an attribute error on a dead object
