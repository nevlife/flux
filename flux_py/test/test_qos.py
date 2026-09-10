"""Consumer QoS from Python: peek vs take, depth, durability.

The names follow ROS 2 (docs/en/qos.en.md). What these pin down is the part a ROS user cannot guess
from the name alone -- peek() does not consume, depth is a lag bound rather than a buffer, and
volatile is measured from where the subscription joined the stream.
"""

import numpy as np
import pytest

import flux

FP = 0x0050EDEF


def test_peek_repeats_and_take_consumes():
    pub = flux.Publisher("/pytest/qos/peek", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/qos/peek", fingerprint=FP)

    assert pub.publish(np.full(8, 3, dtype=np.uint8)) == flux.Published.Ok

    assert int(sub.peek()[0]) == 3
    assert int(sub.peek()[0]) == 3  # nothing new published: the same frame again
    assert int(sub.take()[0]) == 3
    assert sub.take() is None  # consumed
    assert sub.peek() is not None  # still readable as state


def test_depth_one_delivers_newest_only():
    pub = flux.Publisher("/pytest/qos/d1", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/qos/d1", fingerprint=FP)
    assert sub.qos.depth == 1

    for i in range(5):
        assert pub.publish(np.full(8, i, dtype=np.uint8)) == flux.Published.Ok

    assert int(sub.take()[0]) == 4
    assert sub.lost == 4
    assert sub.take() is None


def test_depth_n_catches_up_in_order():
    pub = flux.Publisher("/pytest/qos/dn", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/qos/dn", fingerprint=FP, qos=flux.QoS(depth=8))

    for i in range(5):
        assert pub.publish(np.full(8, i, dtype=np.uint8)) == flux.Published.Ok

    seen = []
    while (v := sub.take()) is not None:
        seen.append(int(v[0]))
    assert seen == [0, 1, 2, 3, 4]


def test_volatile_skips_the_backlog_and_transient_local_replays_it():
    pub = flux.Publisher("/pytest/qos/dur", slot_size=4096, slot_count=8, fingerprint=FP)
    for i in range(3):
        assert pub.publish(np.full(8, i, dtype=np.uint8)) == flux.Published.Ok

    late = flux.Subscription("/pytest/qos/dur", fingerprint=FP, qos=flux.QoS(depth=8))
    assert late.take() is None  # joined after those three

    replay = flux.Subscription(
        "/pytest/qos/dur", fingerprint=FP,
        qos=flux.QoS(depth=8, durability=flux.TransientLocal(2)))
    assert [int(replay.take()[0]) for _ in range(2)] == [1, 2]
    assert replay.take() is None


def test_unhonourable_qos_is_rejected():
    with pytest.raises(ValueError):
        flux.QoS(depth=2, durability=flux.TransientLocal(3))  # replay deeper than the lag window
    with pytest.raises(ValueError):
        flux.QoS(depth=0)
    with pytest.raises(ValueError):
        flux.QoS(max_borrow=0)
    with pytest.raises(ValueError):
        flux.QoS(reliability=flux.Reliability.RELIABLE)


def test_take_blocking_waits_for_the_next_frame():
    pub = flux.Publisher("/pytest/qos/block", slot_size=4096, slot_count=4, fingerprint=FP)
    sub = flux.Subscription("/pytest/qos/block", fingerprint=FP)

    assert sub.take_blocking(timeout_ns=20_000_000) is None  # nothing published yet

    assert pub.publish(np.full(8, 9, dtype=np.uint8)) == flux.Published.Ok
    v = sub.take_blocking(timeout_ns=500_000_000)
    assert v is not None and int(v[0]) == 9
