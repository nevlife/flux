"""flux.rt and PartitionedExecutor.set_thread_scheduling: which thread the kernel prefers.

Not real-time. Python is not an RT target, and nothing here claims a bound. What
is checked is that a declared policy and cpu set reach the thread that was supposed to get them,
that a refusal arrives as an error instead of as a thread running at the wrong priority, and that
a declaration which would never apply is refused rather than accepted.

Affinity carries most of the checking because it needs no privilege. The SCHED_FIFO paths depend
on RLIMIT_RTPRIO, so they are asked of the host rather than assumed.
"""

import os
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402

import flux  # noqa: E402
import flux.ros  # noqa: E402
import flux.rt  # noqa: E402

FP = 0xC1E07A


def rtprio_limit():
    import resource

    return resource.getrlimit(resource.RLIMIT_RTPRIO)[0]


spare_cpus = sorted(os.sched_getaffinity(0))
needs_two_cpus = pytest.mark.skipif(
    len(spare_cpus) < 2, reason="needs at least two cpus to move a thread between them"
)
needs_rtprio = pytest.mark.skipif(
    rtprio_limit() < 10, reason="RLIMIT_RTPRIO too low for SCHED_FIFO on this host"
)


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node("flux_rt_test")
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def spin_in_thread(ex, tick_ns=20_000_000):
    t = threading.Thread(target=ex.spin, args=(tick_ns,), daemon=True)
    t.start()
    return t


# `import flux.rt` and `flux.rt` must name the same module: the extension submodule is reachable
# as an attribute for free, as an import only because __init__ registers it.
def test_the_submodule_is_importable_and_attribute_reachable():
    import flux.rt as by_import

    assert by_import is flux.rt


# Asking for nothing does nothing, and says so with an empty report rather than a host verdict
# no caller asked for.
def test_an_empty_request_is_a_no_op():
    before = flux.rt.current()
    assert flux.rt.apply() == ""
    after = flux.rt.current()
    assert (after.policy, after.priority, after.cpus) == (
        before.policy, before.priority, before.cpus,
    )


# The thread that asked is the thread that changes, and the change is read back from the kernel
# rather than inferred from the call returning.
@needs_two_cpus
def test_affinity_lands_on_the_calling_thread_only():
    target = spare_cpus[0]
    main_before = flux.rt.current().cpus
    got = {}

    def worker():
        flux.rt.apply(cpus=[target])
        got["cpus"] = flux.rt.current().cpus
        got["tid"] = flux.rt.this_tid()

    t = threading.Thread(target=worker)
    t.start()
    t.join(timeout=5.0)

    assert got["cpus"] == [target]
    assert got["tid"] != flux.rt.this_tid()
    assert flux.rt.current().cpus == main_before


# A request the kernel cannot satisfy is refused with the reason, not applied partially and not
# downgraded to something weaker.
def test_a_cpu_this_host_does_not_have_is_refused():
    absent = os.cpu_count() + 64
    before = flux.rt.current().cpus
    with pytest.raises((RuntimeError, OSError)):
        flux.rt.apply(cpus=[absent])
    assert flux.rt.current().cpus == before


# Validation is the same as the C++ Options::validate: out of range is rejected, not clamped.
def test_an_out_of_range_priority_is_rejected():
    with pytest.raises(ValueError):
        flux.rt.apply(policy=flux.rt.Policy.Fifo, priority=200)


# A group's child applies its own declaration before running any callback, so what the callback
# observes is already the declared placement.
@needs_two_cpus
def test_a_group_child_runs_where_it_was_declared(node):
    target = spare_cpus[-1]
    pub = flux.Publisher("/pytest/rt/g", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/rt/g", fingerprint=FP)
    group = MutuallyExclusiveCallbackGroup()
    seen = []

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, group, lambda _v: seen.append(flux.rt.current().cpus))
    ex.set_thread_scheduling(group, cpus=[target])

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while not seen and time.monotonic() < deadline:
            pub.publish(np.full(8, 3, np.uint8))
            time.sleep(0.01)
        assert seen, "the callback never ran"
        assert seen[0] == [target]
    finally:
        ex.stop()
        t.join(timeout=10.0)


# The other half of the split: a node is a partition here too, so it takes a declaration the
# same way a group does.
@needs_two_cpus
def test_a_node_child_runs_where_it_was_declared(node):
    target = spare_cpus[-1]
    seen = []
    node.create_timer(0.01, lambda: seen.append(flux.rt.current().cpus))

    ex = flux.ros.PartitionedExecutor()
    ex.add_ros_node(node)
    ex.set_thread_scheduling(node, cpus=[target])

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while not seen and time.monotonic() < deadline:
            time.sleep(0.01)
        assert seen, "the timer never fired"
        assert seen[0] == [target]
    finally:
        ex.stop()
        t.join(timeout=10.0)


# Two groups, two declarations, two placements. Nothing leaks between the children or onto the
# thread that called spin().
@needs_two_cpus
def test_two_groups_get_the_cpus_each_declared(node):
    left, right = spare_cpus[0], spare_cpus[-1]
    parent_before = flux.rt.current().cpus
    pubs = [
        flux.Publisher(f"/pytest/rt/two{i}", slot_size=4096, slot_count=8, fingerprint=FP)
        for i in range(2)
    ]
    subs = [flux.Subscription(f"/pytest/rt/two{i}", fingerprint=FP) for i in range(2)]
    groups = [MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()]
    seen = {}
    lock = threading.Lock()

    def record(index):
        def cb(_v):
            with lock:
                seen.setdefault(index, flux.rt.current().cpus)
        return cb

    ex = flux.ros.PartitionedExecutor()
    for index, (sub, group) in enumerate(zip(subs, groups)):
        ex.add_flux(sub, group, record(index))
    ex.set_thread_scheduling(groups[0], cpus=[left])
    ex.set_thread_scheduling(groups[1], cpus=[right])

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while len(seen) < 2 and time.monotonic() < deadline:
            for pub in pubs:
                pub.publish(np.full(8, 5, np.uint8))
            time.sleep(0.01)
        assert seen == {0: [left], 1: [right]}
        assert flux.rt.current().cpus == parent_before
    finally:
        ex.stop()
        t.join(timeout=10.0)


# A declaration for a unit no child serves would read as applied and never run. That is the
# silent downgrade the surface exists to prevent, so spin() refuses it.
def test_a_declaration_for_an_unserved_unit_is_refused(node):
    ex = flux.ros.PartitionedExecutor()
    ex.set_thread_scheduling(MutuallyExclusiveCallbackGroup(), cpus=[spare_cpus[0]])
    with pytest.raises(ValueError, match="does not serve"):
        ex.spin(20_000_000)


# Which of two declarations wins must never be a question of call order.
def test_a_second_declaration_for_one_unit_is_refused():
    ex = flux.ros.PartitionedExecutor()
    group = MutuallyExclusiveCallbackGroup()
    ex.set_thread_scheduling(group, cpus=[spare_cpus[0]])
    with pytest.raises(ValueError, match="already has a thread scheduling"):
        ex.set_thread_scheduling(group, cpus=[spare_cpus[0]])


def test_declaring_after_spin_started_is_refused(node):
    pub = flux.Publisher("/pytest/rt/late", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/rt/late", fingerprint=FP)
    group = MutuallyExclusiveCallbackGroup()
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, group, lambda _v: None)

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 2.0
        while not ex._spinning and time.monotonic() < deadline:
            time.sleep(0.01)
        with pytest.raises(RuntimeError, match="before spin"):
            ex.set_thread_scheduling(group, cpus=[spare_cpus[0]])
    finally:
        ex.stop()
        t.join(timeout=10.0)
        del pub


# A child that cannot take its declared policy must end the spin with that error. A thread
# running callbacks at a priority nobody asked for is the outcome this rules out.
@pytest.mark.skipif(rtprio_limit() >= 99, reason="this host grants any RT priority")
def test_a_refused_policy_ends_the_spin_rather_than_running_anyway(node):
    pub = flux.Publisher("/pytest/rt/deny", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/rt/deny", fingerprint=FP)
    group = MutuallyExclusiveCallbackGroup()
    ran = []

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, group, lambda _v: ran.append(1))
    ex.set_thread_scheduling(group, policy=flux.rt.Policy.Fifo, priority=99)

    with pytest.raises((RuntimeError, OSError)):
        ex.spin(20_000_000)
    assert not ran
    del pub


@needs_rtprio
def test_a_granted_policy_reaches_the_child(node):
    pub = flux.Publisher("/pytest/rt/fifo", slot_size=4096, slot_count=8, fingerprint=FP)
    sub = flux.Subscription("/pytest/rt/fifo", fingerprint=FP)
    group = MutuallyExclusiveCallbackGroup()
    seen = []

    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, group, lambda _v: seen.append(flux.rt.current()))
    ex.set_thread_scheduling(group, policy=flux.rt.Policy.Fifo, priority=5)

    t = spin_in_thread(ex)
    try:
        deadline = time.monotonic() + 5.0
        while not seen and time.monotonic() < deadline:
            pub.publish(np.full(8, 1, np.uint8))
            time.sleep(0.01)
        assert seen, "the callback never ran"
        assert seen[0].policy == os.SCHED_FIFO
        assert seen[0].priority == 5
    finally:
        ex.stop()
        t.join(timeout=10.0)
