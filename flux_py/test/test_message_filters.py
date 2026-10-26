import os
import subprocess
import sys
import threading
import time

import message_filters
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Temperature

import flux
from flux.message_filters import Subscriber


MSG = os.path.join(os.path.dirname(__file__), "msg")


@pytest.fixture(scope="module")
def adapters(tmp_path_factory):
    """The adapters flux_gen generates, so generator, wire runtime and filter are checked
    together."""
    out = tmp_path_factory.mktemp("adapters")
    subprocess.run(
        [
            os.path.join(get_package_prefix("flux_gen"), "lib", "flux_gen", "flux_gen"),
            "--pkg",
            "flux_test",
            "--msg",
            os.path.join(MSG, "Stamped.msg"),
            "--msg",
            os.path.join(MSG, "Plain.msg"),
            "--out-py",
            str(out),
        ],
        check=True,
    )
    sys.path.insert(0, str(out))


@pytest.fixture(scope="module")
def Stamped(adapters):  # noqa: N802 the generated class
    from flux_test_flux.stamped import Stamped

    return Stamped


def send(Stamped, pub, sec, payload, frame_id=""):  # noqa: N803
    b = Stamped.build__(pub)
    b.set__header__stamp(sec, 0)
    b.header__frame_id = frame_id
    b.payload = payload
    return b.commit__()


def topic(name):
    return f"/f{os.getpid()}_{name}"


QOS = flux.QoS(depth=8, max_borrow=8)


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = Node("flux_mf_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def pub(node, Stamped, name):  # noqa: N803
    return flux.Publisher(
        node, topic(name), fingerprint=Stamped.FINGERPRINT__, slot_size=64, slot_count=8
    )


def spin_until(ex, done, limit=3.0, each=None):
    t = threading.Thread(target=ex.spin)
    t.start()
    end = time.monotonic() + limit
    while not done() and time.monotonic() < end:
        if each is not None:
            each()
        time.sleep(0.01)
    ex.stop()
    t.join()
    return done()


def test_two_flux_topics_are_paired_by_stamp(node, Stamped):  # noqa: N803
    pa, pb = pub(node, Stamped, "a"), pub(node, Stamped, "b")
    a = Subscriber(node, Stamped, pa.topic_name, 4, qos=QOS)
    b = Subscriber(node, Stamped, pb.topic_name, 4, qos=QOS)
    pairs = []
    sync = message_filters.TimeSynchronizer([a, b], 4)
    sync.registerCallback(lambda x, y: pairs.append((x.view().payload, y.view().payload)))
    ex = flux.Executor()
    ex.add(a)
    ex.add(b)
    send(Stamped, pa, 1, 0xA0)
    send(Stamped, pb, 2, 0xB2)
    send(Stamped, pb, 1, 0xB0)
    assert spin_until(ex, lambda: pairs)
    assert pairs[0] == (0xA0, 0xB0)


def test_a_frame_whose_header_does_not_read_is_counted_not_forwarded(node, Stamped):  # noqa: N803
    pa = pub(node, Stamped, "bad")
    a = Subscriber(node, Stamped, pa.topic_name, 4, qos=QOS)
    got = []
    a.registerCallback(got.append)
    ex = flux.Executor()
    ex.add(a)
    pa.publish(b"abc")
    send(Stamped, pa, 1, 1)
    assert spin_until(ex, lambda: a.forwarded == 1)
    assert a.unreadable == 1 and len(got) == 1


def test_a_queued_frame_carries_its_frame_id(node, Stamped):  # noqa: N803
    pa = pub(node, Stamped, "frame")
    a = Subscriber(node, Stamped, pa.topic_name, 4, qos=QOS)
    got = []
    a.registerCallback(got.append)
    ex = flux.Executor()
    ex.add(a)
    send(Stamped, pa, 1, 1, frame_id="cam")
    assert spin_until(ex, lambda: got)
    assert got[0].header.frame_id == "cam"


def test_a_negative_stamp_is_counted_not_forwarded(node, Stamped):  # noqa: N803
    pa = pub(node, Stamped, "neg")
    a = Subscriber(node, Stamped, pa.topic_name, 4, qos=QOS)
    cache = message_filters.Cache(a, 4)
    ex = flux.Executor()
    ex.add(a)
    send(Stamped, pa, -1, 1)
    send(Stamped, pa, 2, 2)
    assert spin_until(ex, lambda: a.forwarded == 1)
    assert a.unreadable == 1 and cache.getLast().view().payload == 2


def test_an_adapter_without_a_header_is_refused(node, adapters):
    from flux_test_flux.plain import Plain

    with pytest.raises(TypeError, match="Header"):
        Subscriber(node, Plain, topic("plain"), 4)


def test_a_cache_with_the_default_qos_keeps_up(node, Stamped):  # noqa: N803
    pa = flux.Publisher(
        node, topic("cache"), fingerprint=Stamped.FINGERPRINT__, slot_size=64, slot_count=16
    )
    a = Subscriber(node, Stamped, pa.topic_name, 10)
    cache = message_filters.Cache(a, 10)
    ex = flux.Executor()
    ex.add(a)
    sent = [0]

    def more():
        if sent[0] < 30 and send(Stamped, pa, sent[0], sent[0]) == flux.Published.OK:
            sent[0] += 1

    def last():
        m = cache.getLast()
        return m is not None and m.view().payload == 29

    assert spin_until(ex, last, each=more)


def test_a_publisher_with_no_more_slots_than_the_queue_is_refused(node, Stamped):  # noqa: N803
    pa = pub(node, Stamped, "few")
    with pytest.raises(flux.Error, match="message_filters queue needs 9"):
        Subscriber(node, Stamped, pa.topic_name, 8).subscription.take()


def test_a_flux_topic_pairs_with_a_ros_topic(node, Stamped):  # noqa: N803
    pa = pub(node, Stamped, "mixed")
    a = Subscriber(node, Stamped, pa.topic_name, 4, qos=QOS)
    r = message_filters.Subscriber(node, Temperature, topic("temp"))
    paired = []
    sync = message_filters.TimeSynchronizer([a, r], 4)
    sync.registerCallback(lambda x, y: paired.append(1))
    temp = node.create_publisher(Temperature, topic("temp"), 10)
    ex = flux.Executor()
    ex.add(a)
    ex.add_node(node)

    def both():
        send(Stamped, pa, 5, 1)
        t = Temperature()
        t.header.stamp.sec = 5
        temp.publish(t)

    assert spin_until(ex, lambda: paired, each=both)


def test_a_partitioned_synchronizer_keeps_its_inputs_in_one_flux_group(node, Stamped):  # noqa: N803
    pa, pb = pub(node, Stamped, "pa"), pub(node, Stamped, "pb")
    a = Subscriber(node, Stamped, pa.topic_name, 4, qos=QOS)
    b = Subscriber(node, Stamped, pb.topic_name, 4, qos=QOS)
    g, other = MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()

    split = flux.PartitionedExecutor()
    split.add(a, g)
    split.add(b, other)
    split.add_sync_group(a, b)
    with pytest.raises(ValueError):
        split.spin()

    mixed = flux.PartitionedExecutor()
    mixed.add(a, g)
    mixed.add_sync_group(a, message_filters.Subscriber(node, Temperature, topic("t2")))
    with pytest.raises(ValueError):
        mixed.spin()

    paired = []
    sync = message_filters.TimeSynchronizer([a, b], 4)
    sync.registerCallback(lambda x, y: paired.append(1))
    one = flux.PartitionedExecutor()
    one.add(a, g)
    one.add(b, g)
    one.add_sync_group(a, b)

    def both():
        send(Stamped, pa, 7, 1)
        send(Stamped, pb, 7, 2)

    assert spin_until(one, lambda: paired, each=both)
