"""flux.ros.PartitionedExecutor: one thread per isolation unit.

The single flux.ros.Executor runs every callback on the rclpy spin thread, so a slow callback
delays every other channel. These tests check that the partitioned executor removes exactly that
coupling, and that it refuses the layouts rclpy cannot honor rather than downgrading them
quietly -- rclpy has no add_callback_group, so a flux group that also holds ROS entities would
have its mutual exclusion broken with nothing said.
"""

import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402

import flux  # noqa: E402
import flux.ros  # noqa: E402

FP = 0xC1E0DE


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node("flux_cie_test")
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def spin_in_thread(ex, tick_ns=20_000_000):
    t = threading.Thread(target=ex.spin, args=(tick_ns,), daemon=True)
    t.start()
    return t


def pump(publisher, payload, stop):
    while not stop.is_set():
        publisher.publish(payload)
        time.sleep(0.005)


# The point of the whole class: a callback blocking in one group must not delay delivery in
# another. On one flux.ros.Executor the fast channel starves behind the blocked callback.
def test_blocked_group_does_not_stall_others(node):
    slow_pub = flux.Publisher("/pytest/part/slow", slot_size=4096, slot_count=8, fingerprint=FP)
    fast_pub = flux.Publisher("/pytest/part/fast", slot_size=4096, slot_count=8, fingerprint=FP)
    slow_sub = flux.Subscription("/pytest/part/slow", fingerprint=FP)
    fast_sub = flux.Subscription("/pytest/part/fast", fingerprint=FP)

    release = threading.Event()
    entered = threading.Event()
    fast_seen = []

    def slow(_view):
        entered.set()
        release.wait(timeout=10.0)

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(slow_sub, MutuallyExclusiveCallbackGroup(), slow)
    ex.add_flux(fast_sub, MutuallyExclusiveCallbackGroup(), lambda v: fast_seen.append(int(v[0])))

    t = spin_in_thread(ex)
    stop = threading.Event()
    feeders = [
        threading.Thread(target=pump, args=(slow_pub, np.full(8, 1, np.uint8), stop), daemon=True),
        threading.Thread(target=pump, args=(fast_pub, np.full(8, 9, np.uint8), stop), daemon=True),
    ]
    for f in feeders:
        f.start()
    try:
        assert entered.wait(timeout=5.0), "the slow callback never ran"
        # The slow group is parked inside its callback right now. The fast group must keep
        # delivering anyway -- that is the isolation being tested.
        deadline = time.monotonic() + 5.0
        while len(fast_seen) < 5 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert len(fast_seen) >= 5, f"fast group stalled behind the blocked group: {fast_seen}"
        assert set(fast_seen) == {9}
    finally:
        stop.set()
        release.set()
        for f in feeders:
            f.join(timeout=5.0)
        ex.stop()
        t.join(timeout=10.0)
        assert not t.is_alive()


# Each group gets its own thread, and no callback runs on the thread that called spin().
def test_each_group_gets_its_own_thread(node):
    pubs = [
        flux.Publisher(f"/pytest/part/t{i}", slot_size=4096, slot_count=8, fingerprint=FP)
        for i in range(3)
    ]
    subs = [flux.Subscription(f"/pytest/part/t{i}", fingerprint=FP) for i in range(3)]

    seen = {}
    lock = threading.Lock()

    def record(index):
        def cb(_view):
            with lock:
                seen.setdefault(index, set()).add(threading.get_ident())

        return cb

    ex = flux.ros.PartitionedExecutor()
    for i, sub in enumerate(subs):
        ex.add_flux(sub, MutuallyExclusiveCallbackGroup(), record(i))

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while len(seen) < 3 and time.monotonic() < deadline:
            for p in pubs:
                p.publish(np.full(8, 3, np.uint8))
            time.sleep(0.02)
        assert len(seen) == 3, f"not every group delivered: {sorted(seen)}"
        threads = [next(iter(v)) for v in seen.values()]
        assert all(len(v) == 1 for v in seen.values()), "a group ran on more than one thread"
        assert len(set(threads)) == 3, "groups shared a thread"
        assert t.ident not in threads, "a callback ran on the spin thread"
    finally:
        ex.stop()
        t.join(timeout=10.0)


# A PartitionedExecutor holding only a node: every child is a stock rclpy executor and ROS delivery is unchanged.
def test_pure_ros_node_is_served(node):
    from std_msgs.msg import UInt64

    got = []
    node.create_subscription(UInt64, "/pytest/part/ros", lambda m: got.append(m.data), 10)
    pub = node.create_publisher(UInt64, "/pytest/part/ros", 10)

    ex = flux.ros.PartitionedExecutor()
    ex.add_ros_node(node)
    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while not got and time.monotonic() < deadline:
            pub.publish(UInt64(data=7))
            time.sleep(0.02)
        assert got and got[0] == 7
    finally:
        ex.stop()
        t.join(timeout=10.0)


# The node's ROS callbacks and a flux group run on different threads: that is the isolation the
# single flux.ros.Executor cannot give, since it merges both onto the rclpy spin thread.
def test_ros_node_and_flux_group_do_not_share_a_thread(node):
    from std_msgs.msg import UInt64

    pub = flux.Publisher("/pytest/part/mix", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/part/mix", fingerprint=FP)
    ros_pub = node.create_publisher(UInt64, "/pytest/part/mixros", 10)

    threads = {}
    ros_seen = threading.Event()
    flux_seen = threading.Event()

    def on_ros(_msg):
        threads["ros"] = threading.get_ident()
        ros_seen.set()

    node.create_subscription(UInt64, "/pytest/part/mixros", on_ros, 10)

    def on_flux(_view):
        threads["flux"] = threading.get_ident()
        flux_seen.set()

    ex = flux.ros.PartitionedExecutor()
    ex.add_ros_node(node)
    ex.add_flux(sub, MutuallyExclusiveCallbackGroup(), on_flux)

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while not (ros_seen.is_set() and flux_seen.is_set()) and time.monotonic() < deadline:
            pub.publish(np.full(8, 5, np.uint8))
            ros_pub.publish(UInt64(data=5))
            time.sleep(0.02)
        assert ros_seen.is_set() and flux_seen.is_set(), threads
        assert threads["ros"] != threads["flux"]
        assert t.ident not in threads.values()
    finally:
        ex.stop()
        t.join(timeout=10.0)


# One thread per group is exactly what a Reentrant group asks not to have. Refuse rather than
# serialize callbacks that declared they may overlap.
def test_reentrant_group_is_refused(node):
    sub = flux.Subscription("/pytest/part/reent", fingerprint=FP)
    ex = flux.ros.PartitionedExecutor()
    with pytest.raises(ValueError, match="Reentrant"):
        ex.add_flux(sub, ReentrantCallbackGroup(), lambda v: None)


# rclpy cannot hand a callback group to a child executor, so a group holding ROS entities would
# keep serving them on the node's thread while flux frames ran here. Refuse instead.
def test_group_holding_ros_entities_is_refused(node):
    from std_msgs.msg import UInt64

    group = MutuallyExclusiveCallbackGroup()
    node.create_subscription(UInt64, "/pytest/part/occupied", lambda m: None, 10, callback_group=group)

    sub = flux.Subscription("/pytest/part/occupied_flux", fingerprint=FP)
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, group, lambda v: None)  # entities are checked at spin, not at add
    with pytest.raises(ValueError, match="no ROS entities"):
        ex.spin(20_000_000)


# The same rule holds after spin() started: the parent's tick re-checks, because a timer created
# in a flux group mid-run breaks the group just as thoroughly as one created before it.
def test_group_that_grows_a_ros_entity_after_spin_raises(node):
    group = MutuallyExclusiveCallbackGroup()
    sub = flux.Subscription("/pytest/part/grows", fingerprint=FP)
    ex = flux.ros.PartitionedExecutor()
    ex.add_ros_node(node)
    ex.add_flux(sub, group, lambda v: None)

    error = []

    def run():
        try:
            ex.spin(20_000_000)
        except BaseException as exc:  # noqa: BLE001
            error.append(exc)

    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(0.3)
    node.create_timer(1.0, lambda: None, callback_group=group)
    t.join(timeout=10.0)
    assert not t.is_alive()
    assert error and isinstance(error[0], ValueError)
    assert "no ROS entities" in str(error[0])


# Registration is a pre-spin affair: the spawn scan reads the lists without a lock.
def test_add_after_spin_raises(node):
    sub = flux.Subscription("/pytest/part/late", fingerprint=FP)
    other = flux.Subscription("/pytest/part/late2", fingerprint=FP)
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, MutuallyExclusiveCallbackGroup(), lambda v: None)
    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 2.0
        while not ex._spinning and time.monotonic() < deadline:
            time.sleep(0.01)
        with pytest.raises(RuntimeError, match="before spin"):
            ex.add_flux(other, MutuallyExclusiveCallbackGroup(), lambda v: None)
        with pytest.raises(RuntimeError, match="before spin"):
            ex.add_ros_node(node)
        with pytest.raises(RuntimeError, match="already spinning"):
            ex.spin()
    finally:
        ex.stop()
        t.join(timeout=10.0)


def test_rejects_a_subscription_assigned_twice(node):
    sub = flux.Subscription("/pytest/part/dup", fingerprint=FP)
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, MutuallyExclusiveCallbackGroup(), lambda v: None)
    with pytest.raises(ValueError, match="already assigned"):
        ex.add_flux(sub, MutuallyExclusiveCallbackGroup(), lambda v: None)


def test_rejects_a_non_group_token(node):
    sub = flux.Subscription("/pytest/part/token", fingerprint=FP)
    ex = flux.ros.PartitionedExecutor()
    with pytest.raises(TypeError, match="callback group"):
        ex.add_flux(sub, "cam0", lambda v: None)


# An exception on a child thread stops every child and surfaces at the parent's spin().
def test_child_exception_reaches_spin(node):
    pub = flux.Publisher("/pytest/part/boom", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/part/boom", fingerprint=FP)

    def explode(_view):
        raise RuntimeError("callback exploded")

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, MutuallyExclusiveCallbackGroup(), explode)

    error = []

    def run():
        try:
            ex.spin(20_000_000)
        except BaseException as exc:  # noqa: BLE001
            error.append(exc)

    t = threading.Thread(target=run, daemon=True)
    t.start()
    deadline = time.monotonic() + 5.0
    while t.is_alive() and time.monotonic() < deadline:
        pub.publish(np.full(8, 1, np.uint8))
        time.sleep(0.02)
    t.join(timeout=5.0)
    assert not t.is_alive()
    assert error and "callback exploded" in str(error[0])


def test_stop_ends_spin_and_close_bars_restart(node):
    sub = flux.Subscription("/pytest/part/stop", fingerprint=FP)
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, MutuallyExclusiveCallbackGroup(), lambda v: None)
    t = spin_in_thread(ex)
    time.sleep(0.2)
    ex.stop()
    t.join(timeout=5.0)
    assert not t.is_alive()
    ex.close()
    with pytest.raises(RuntimeError, match="closed"):
        ex.spin()
