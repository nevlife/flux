"""flux.ros.message_filters: flux topics and DDS topics synchronized in one graph, from Python.

The C++ side proves the same thing in flux_cpp/test/test_message_filters.cpp. What is different
here, and what these tests are for: the object a Python callback receives already owns its
borrow, so the filter queues it directly instead of taking the frame first -- and rclpy has no
add_callback_group, so the mixed graph that PartitionedExecutor accepts in C++ has to be refused
here rather than run with its inputs on two threads.
"""

import sys
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
cli = pytest.importorskip("flux_gen.cli")
upstream = pytest.importorskip("message_filters")

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402

import flux  # noqa: E402
import flux.ros  # noqa: E402
import flux.ros.message_filters as fmf  # noqa: E402

STAMPED_MSG = """std_msgs/Header header
uint32 seq
"""

BARE_MSG = """uint32 seq
"""


@pytest.fixture(scope="module")
def adapters(tmp_path_factory):
    """Generated adapters for a message with a Header and one without."""
    from flux_gen import load_dir

    src = tmp_path_factory.mktemp("msg") / "msg"
    src.mkdir()
    (src / "Stamped.msg").write_text(STAMPED_MSG)
    (src / "Bare.msg").write_text(BARE_MSG)

    out = str(tmp_path_factory.mktemp("adapters"))
    reg = cli.ament_registry()
    if reg.get("std_msgs/Header") is None:
        pytest.skip("std_msgs is not on AMENT_PREFIX_PATH")
    load_dir(str(src), package="mf_pkg", into=reg)
    cli.generate(str(src / "Stamped.msg"), "mf_pkg", None, out, reg)
    cli.generate(str(src / "Bare.msg"), "mf_pkg", None, out, reg)

    sys.path.insert(0, out)
    try:
        from mf_pkg_flux.bare import Bare
        from mf_pkg_flux.stamped import Stamped

        yield Stamped, Bare
    finally:
        sys.path.remove(out)


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node("flux_mf_test")
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def publish(pub, adapter, sec, nanosec, seq):
    b = adapter.build__(pub)
    assert b is not None, "no free slot"
    b.set__header__stamp(sec, nanosec)
    b.seq = seq
    return b.commit__()


def spin_in_thread(ex, *args):
    t = threading.Thread(target=ex.spin, args=args, daemon=True)
    t.start()
    return t


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


# Two flux topics into one synchronizer. The pairing key is the header stamp the publisher wrote,
# so a match proves the filter read the stamp out of the segment and not out of arrival order:
# the two topics are published in opposite orders below.
def test_two_flux_inputs_pair_on_the_header_stamp(node, adapters):
    Stamped, _ = adapters
    fp = Stamped.FINGERPRINT__
    left_pub = flux.Publisher("/pytest/mf/l", slot_size=4096, slot_count=8, fingerprint=fp)
    right_pub = flux.Publisher("/pytest/mf/r", slot_size=4096, slot_count=8, fingerprint=fp)

    left = fmf.Subscriber(node, Stamped, "/pytest/mf/l", qos=flux.QoS(depth=8, max_borrow=32))
    right = fmf.Subscriber(node, Stamped, "/pytest/mf/r", qos=flux.QoS(depth=8, max_borrow=32))

    pairs = []
    sync = upstream.TimeSynchronizer([left, right], 10)
    sync.registerCallback(lambda a, b: pairs.append((a.view().seq, b.view().seq)))

    ex = flux.ros.Executor()
    ex.add_flux(left)
    ex.add_flux(right)
    spin_in_thread(ex, 20_000_000)
    try:
        # Right first, so arrival order and stamp order disagree.
        for i in range(3):
            publish(right_pub, Stamped, 100 + i, 0, 1000 + i)
        time.sleep(0.05)
        for i in range(3):
            publish(left_pub, Stamped, 100 + i, 0, i)
        assert wait_for(lambda: len(pairs) >= 3)
    finally:
        ex.stop()

    assert pairs[:3] == [(0, 1000), (1, 1001), (2, 1002)]
    assert left.forwarded == 3 and right.forwarded == 3
    assert left.unreadable == 0 and right.unreadable == 0


# The mixed graph: one flux topic, one DDS topic, one synchronizer, one thread. This is the
# arrangement Python had no way to express at all before.
def test_flux_and_dds_inputs_pair_in_one_synchronizer(node, adapters):
    TimeReference = pytest.importorskip("sensor_msgs.msg").TimeReference

    Stamped, _ = adapters
    fp = Stamped.FINGERPRINT__
    # slot_count well over the synchronizer's queue: a queued frame is a held borrow, and a
    # publisher whose slots are all held has nowhere to write (docs/en/api.en.md 3).
    flux_pub = flux.Publisher("/pytest/mf/mix", slot_size=4096, slot_count=64, fingerprint=fp)
    dds_pub = node.create_publisher(TimeReference, "/pytest/mf/dds", 10)

    fast = fmf.Subscriber(node, Stamped, "/pytest/mf/mix", qos=flux.QoS(depth=8, max_borrow=32))
    slow = upstream.Subscriber(node, TimeReference, "/pytest/mf/dds")

    pairs = []
    sync = upstream.ApproximateTimeSynchronizer([fast, slow], 10, 0.05)
    sync.registerCallback(lambda a, b: pairs.append((a.view().seq, b.source)))

    ex = flux.ros.Executor()
    ex.add_flux(fast)
    ex.add_ros_node(node)
    spin_in_thread(ex, 20_000_000)
    try:
        deadline = time.monotonic() + 10.0
        i = 0
        while not pairs and time.monotonic() < deadline:
            publish(flux_pub, Stamped, 200 + i, 0, i)
            m = TimeReference()
            m.header.stamp.sec = 200 + i
            m.source = f"dds{i}"
            dds_pub.publish(m)
            i += 1
            time.sleep(0.02)
        assert pairs, "no flux/DDS pair matched"
    finally:
        ex.stop()

    seq, source = pairs[0]
    assert source == f"dds{seq}"


# The borrow travels with the queued object: that is the whole reason this needs no take().
# A frame still sitting in a synchronizer queue must still read correctly after the callback
# that delivered it returned.
def test_a_queued_frame_outlives_the_callback_that_delivered_it(node, adapters):
    Stamped, _ = adapters
    fp = Stamped.FINGERPRINT__
    pub = flux.Publisher("/pytest/mf/hold", slot_size=4096, slot_count=4, fingerprint=fp)
    sub = fmf.Subscriber(node, Stamped, "/pytest/mf/hold", qos=flux.QoS(depth=8, max_borrow=8))

    held = []
    sub.registerCallback(held.append)

    ex = flux.ros.Executor()
    ex.add_flux(sub)
    spin_in_thread(ex, 20_000_000)
    try:
        for i in range(3):
            publish(pub, Stamped, 300 + i, i, i)
        assert wait_for(lambda: len(held) >= 3)
    finally:
        ex.stop()

    # Read every frame only now, long after its callback returned and out of delivery order.
    assert [m.view().seq for m in reversed(held[:3])] == [2, 1, 0]
    assert [(m.header.stamp.sec, m.header.stamp.nanosec) for m in held[:3]] == [
        (300, 0), (301, 1), (302, 2)
    ]


# A schema with no Header has no key to synchronize on. C++ stops at compile time; the earliest
# this language can stop is the first frame, and it must say why rather than pair everything.
def test_a_schema_without_a_header_is_refused_at_the_first_frame(node, adapters):
    _, Bare = adapters
    fp = Bare.FINGERPRINT__
    pub = flux.Publisher("/pytest/mf/bare", slot_size=4096, slot_count=4, fingerprint=fp)
    sub = fmf.Subscriber(node, Bare, "/pytest/mf/bare", qos=flux.QoS(max_borrow=4))

    b = Bare.build__(pub)
    b.seq = 7
    b.commit__()

    ex = flux.ros.Executor()
    ex.add_flux(sub)
    with pytest.raises(TypeError, match="no std_msgs/Header"):
        ex.spin_once(50_000_000)


# PartitionedExecutor: flux inputs in one group are on one thread, so the graph runs.
def test_partitioned_accepts_flux_inputs_in_one_group(node, adapters):
    Stamped, _ = adapters
    fp = Stamped.FINGERPRINT__
    left_pub = flux.Publisher("/pytest/mf/pl", slot_size=4096, slot_count=8, fingerprint=fp)
    right_pub = flux.Publisher("/pytest/mf/pr", slot_size=4096, slot_count=8, fingerprint=fp)

    left = fmf.Subscriber(node, Stamped, "/pytest/mf/pl", qos=flux.QoS(depth=8, max_borrow=32))
    right = fmf.Subscriber(node, Stamped, "/pytest/mf/pr", qos=flux.QoS(depth=8, max_borrow=32))

    pairs = []
    sync = upstream.TimeSynchronizer([left, right], 10)
    sync.registerCallback(lambda a, b: pairs.append((a.view().seq, b.view().seq)))

    group = MutuallyExclusiveCallbackGroup()
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(left, group)
    ex.add_flux(right, group)
    ex.add_sync_group(left, right)
    spin_in_thread(ex, 20_000_000)
    try:
        for i in range(3):
            publish(left_pub, Stamped, 400 + i, 0, i)
            publish(right_pub, Stamped, 400 + i, 0, 1000 + i)
        assert wait_for(lambda: len(pairs) >= 3)
    finally:
        ex.stop()

    assert pairs[:3] == [(0, 1000), (1, 1001), (2, 1002)]
    assert ex.unplaced_sync_inputs() == 0


# Two groups is two threads, and the sync policy's lock would be what joins them.
def test_partitioned_refuses_sync_inputs_split_across_groups(node, adapters):
    Stamped, _ = adapters
    left = fmf.Subscriber(node, Stamped, "/pytest/mf/gl")
    right = fmf.Subscriber(node, Stamped, "/pytest/mf/gr")

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(left, MutuallyExclusiveCallbackGroup())
    ex.add_flux(right, MutuallyExclusiveCallbackGroup())
    ex.add_sync_group(left, right)
    with pytest.raises(ValueError, match="spread across threads"):
        ex.spin(20_000_000)


# The refusal that is a language fact, not a policy: rclpy cannot move a DDS subscription onto a
# flux group's thread, so a mixed graph under PartitionedExecutor cannot be placed at all.
def test_partitioned_refuses_a_mixed_flux_and_dds_synchronizer(node, adapters):
    from std_msgs.msg import Header

    Stamped, _ = adapters
    fast = fmf.Subscriber(node, Stamped, "/pytest/mf/ml")
    slow = upstream.Subscriber(node, Header, "/pytest/mf/mr")

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(fast, MutuallyExclusiveCallbackGroup())
    ex.add_ros_node(node)
    ex.add_sync_group(fast, slow)
    with pytest.raises(ValueError, match="flux.ros.Executor"):
        ex.spin(20_000_000)


# An input nobody drives never pairs, and the partner waits forever. Saying so beats a graph that
# is simply silent.
def test_partitioned_refuses_an_unassigned_flux_input(node, adapters):
    Stamped, _ = adapters
    left = fmf.Subscriber(node, Stamped, "/pytest/mf/ul")
    right = fmf.Subscriber(node, Stamped, "/pytest/mf/ur")

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(left, MutuallyExclusiveCallbackGroup())
    ex.add_sync_group(left, right)
    with pytest.raises(ValueError, match="not assigned to any group"):
        ex.spin(20_000_000)


# A chain-middle filter names no subscription, so there is no thread to compare. It is counted,
# not guessed at.
def test_an_unplaceable_input_is_counted_not_judged(node, adapters):
    Stamped, _ = adapters
    left = fmf.Subscriber(node, Stamped, "/pytest/mf/cl")
    cache = upstream.Cache(left, 10)

    group = MutuallyExclusiveCallbackGroup()
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(left, group)
    ex.add_sync_group(left, cache)
    spin_in_thread(ex, 20_000_000)
    try:
        assert wait_for(lambda: ex.unplaced_sync_inputs() == 1)
    finally:
        ex.stop()


def test_a_bare_module_is_not_an_adapter(node):
    with pytest.raises(TypeError, match="flux_gen adapter"):
        fmf.Subscriber(node, np, "/pytest/mf/bad")
