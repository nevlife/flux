import os
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from std_msgs.msg import String

import flux

SMALL = {"slot_size": 4096, "slot_count": 4}


def topic(name):
    return f"/q{os.getpid()}_{name}"


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Node("flux_part_test")
    yield n
    n.destroy_node()


def eventually(done, limit=3.0):
    end = time.monotonic() + limit
    while not done() and time.monotonic() < end:
        time.sleep(0.005)
    return done()


class Spinning:
    """Spins on a thread; stop() ends it and returns what spin() raised, if anything."""

    def __init__(self, ex):
        self.ex = ex
        self.error = None
        self.thread = threading.Thread(target=self._run)
        self.thread.start()

    def _run(self):
        try:
            self.ex.spin()
        except BaseException as e:  # noqa: B036 reported by stop()
            self.error = e

    def stop(self):
        self.ex.stop()
        self.thread.join()
        return self.error


def test_a_blocked_group_does_not_stall_another(node):
    pa = flux.Publisher(node, topic("a"), **SMALL)
    pb = flux.Publisher(node, topic("b"), **SMALL)
    release = threading.Event()
    a_in, b_got = [], []

    def slow(v):
        a_in.append(1)
        release.wait(3)

    ex = flux.PartitionedExecutor()
    ex.add(flux.Subscription(node, pa.topic_name, callback=slow), MutuallyExclusiveCallbackGroup())
    ex.add(
        flux.Subscription(node, pb.topic_name, callback=lambda v: b_got.append(1)),
        MutuallyExclusiveCallbackGroup(),
    )
    s = Spinning(ex)
    pa.publish(b"x")
    assert eventually(lambda: a_in)
    pb.publish(b"y")
    assert eventually(lambda: b_got)
    release.set()
    assert s.stop() is None


def test_a_lifecycle_node_carries_frames():
    node = LifecycleNode("flux_part_lifecycle")
    pub = flux.Publisher(node, topic("lifecycle"), **SMALL)
    got = []
    ex = flux.PartitionedExecutor()
    ex.add(
        flux.Subscription(node, pub.topic_name, callback=lambda v: got.append(1)),
        MutuallyExclusiveCallbackGroup(),
    )
    ex.add_node(node)
    s = Spinning(ex)
    pub.publish(b"x")
    assert eventually(lambda: got)
    assert s.stop() is None
    node.destroy_node()


def test_groups_and_nodes_get_threads_of_their_own(node):
    pa = flux.Publisher(node, topic("own"), **SMALL)
    seen = {}
    ex = flux.PartitionedExecutor()
    group = MutuallyExclusiveCallbackGroup()
    ex.add(
        flux.Subscription(
            node, pa.topic_name, callback=lambda v: seen.setdefault("flux", threading.get_ident())
        ),
        group,
    )
    node.create_timer(0.005, lambda: seen.setdefault("timer", threading.get_ident()))
    ex.add_node(node)
    hooked = {}
    ex.on_thread_start(group, lambda: hooked.setdefault("group", threading.get_ident()))
    s = Spinning(ex)
    assert eventually(lambda: pa.publish(b"x") and "flux" in seen and "timer" in seen)
    assert s.stop() is None
    assert seen["flux"] != seen["timer"]
    assert hooked["group"] == seen["flux"]


def test_a_flux_group_that_holds_a_ros_entity_is_refused(node):
    group = MutuallyExclusiveCallbackGroup()
    ex = flux.PartitionedExecutor()
    ex.add(flux.Subscription(node, topic("mixed"), callback=print), group)
    ex.add_node(node)
    s = Spinning(ex)
    time.sleep(0.05)
    node.create_subscription(String, topic("ros"), print, 10, callback_group=group)
    assert eventually(lambda: not s.thread.is_alive())
    assert isinstance(s.stop(), ValueError)


def test_misuse_is_refused(node):
    ex = flux.PartitionedExecutor()
    sub = flux.Subscription(node, topic("misuse"), callback=print)
    with pytest.raises(TypeError):
        ex.add(sub, "group")
    with pytest.raises(ValueError):
        ex.add(sub, ReentrantCallbackGroup())
    with pytest.raises(ValueError):
        ex.add(flux.Subscription(node, topic("nocb")), MutuallyExclusiveCallbackGroup())
    ex.add(sub, MutuallyExclusiveCallbackGroup())
    with pytest.raises(ValueError):
        ex.add(sub, MutuallyExclusiveCallbackGroup())
    ex.on_thread_start(MutuallyExclusiveCallbackGroup(), print)
    with pytest.raises(ValueError):
        ex.spin()


def test_a_callback_error_stops_every_thread_and_reaches_spin(node):
    pa = flux.Publisher(node, topic("raise"), **SMALL)

    def boom(v):
        raise KeyError("in a group")

    ex = flux.PartitionedExecutor()
    ex.add(flux.Subscription(node, pa.topic_name, callback=boom), MutuallyExclusiveCallbackGroup())
    ex.add_node(node)
    pa.publish(b"x")
    with pytest.raises(KeyError):
        ex.spin()
    ex.stop()
    ex.spin()


def test_context_shutdown_ends_spin():
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    other = Node("in_ctx", context=ctx)
    ex = flux.PartitionedExecutor()
    ex.add_node(other)
    s = Spinning(ex)
    time.sleep(0.05)
    rclpy.shutdown(context=ctx)
    assert eventually(lambda: not s.thread.is_alive())
    assert s.stop() is None


def test_a_flux_only_spin_ends_when_its_context_shuts_down():
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    other = Node("flux_only_ctx", context=ctx)
    ex = flux.PartitionedExecutor()
    ex.add(flux.Subscription(other, topic("ctx"), callback=print), MutuallyExclusiveCallbackGroup())
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    time.sleep(0.05)
    rclpy.shutdown(context=ctx)
    t.join(timeout=3)
    assert not t.is_alive()


def test_a_shutdown_racing_a_node_threads_wait_set_ends_spin_quietly(monkeypatch):
    class ShutDownInWait(SingleThreadedExecutor):
        # rclpy raises RCLError, not ExternalShutdownException, when a signal shuts the context
        # down while it builds a wait set.
        def spin_once(self, timeout_sec=None):
            rclpy.shutdown(context=self.context)
            raise _rclpy.RCLError("the given context is not valid")

    monkeypatch.setattr(flux, "SingleThreadedExecutor", ShutDownInWait)
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    other = Node("racing_shutdown", context=ctx)
    ex = flux.PartitionedExecutor()
    ex.add_node(other)
    ex.spin()


def test_frames_reach_a_group_beside_a_busy_node_thread(node):
    name = topic("busyros")
    clock = os.path.join(os.path.dirname(__file__), "clock_pub.py")
    p = subprocess.Popen(
        [sys.executable, clock, name, "100", "0.01"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
    )
    assert p.stdout.readline() == "ready\n"
    got = []
    sub = flux.Subscription(
        node, name, callback=lambda v: got.append(1), qos=flux.QoS(depth=8, max_borrow=4)
    )

    def busy():
        end = time.perf_counter() + 0.001
        while time.perf_counter() < end:
            pass

    # Always ready, so the node thread holds the GIL nearly all the time.
    node.create_timer(0.0001, busy)
    ex = flux.PartitionedExecutor()
    ex.add(sub, MutuallyExclusiveCallbackGroup())
    ex.add_node(node)
    s = Spinning(ex)
    assert eventually(lambda: sub.attached)
    p.stdin.write("go\n")
    p.stdin.flush()
    p.wait(timeout=20)
    time.sleep(0.1)
    assert s.stop() is None
    assert len(got) >= 95
