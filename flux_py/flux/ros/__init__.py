"""Unified executor: rclpy subscriptions and flux channels dispatched by one loop, one thread.

The C++ side merges the two by pulling ROS into flux's io_uring -- rclcpp hands out an
on-new-message callback, so a subscription can be bridged to an eventfd and armed in the same
ring. rclpy does not hand that callback to Python, so that direction is closed here. This module
goes the other way: a bridge thread waits on the io_uring and hands the dispatch to the spin
thread as an executor task.

    flux channel --- io_uring --- [bridge] --- create_task ---+
                                                              +--> rclpy executor --> callback
    rclpy subscription ---------------------------------------+

`create_task` rather than a GuardCondition: a guard condition is a wait-set entity, and rclpy's
EventsExecutor has no wait set and rejects it outright. Tasks belong to the executor itself, so
the same bridge runs on every rclpy executor.

The bridge thread never touches a flux Channel. It only blocks on the io_uring and queues the
task; every take() and every callback runs on the rclpy spin thread. That matters: a re-attach
after a publisher restart swaps the mapping the wake word lives in, so reading it from a second
thread would race a munmap.

wait_for_work() and dispatch() must not overlap (they share the io_uring). The handshake below
enforces it: the bridge waits, queues the task, then blocks on `drained` until the spin thread
has finished dispatching.
"""

import ctypes
import threading
import warnings
import weakref

from .._flux import Device, Durability, FenceWait, Frame, Published, QoS, Refused, Reliability
from .._flux import faulted
from .._flux import SegmentMismatch, TransientLocal, Volatile
from .._flux import Executor as _FluxExecutor
from .._flux import NO_SCHEMA
from .._flux import Publisher as _Publisher
from .._flux import Subscription as _Subscription

__all__ = [
    "PartitionedExecutor",
    "Device",
    "Durability",
    "Executor",
    "FenceWait",
    "Frame",
    "NO_SCHEMA",
    "Published",
    "Publisher",
    "QoS",
    "Refused",
    "Reliability",
    "SegmentMismatch",
    "Subscription",
    "TransientLocal",
    "Volatile",
    "faulted",
    "resolve",
]


class Executor:
    """One spin loop for both transports.

    Wraps an rclpy executor, so ROS timers, services and subscriptions keep working exactly as
    they do without flux. flux frames arrive as callbacks on the same thread, between ROS
    callbacks -- no second executor, no locking between the two.

    Assembled the way `flux::ros::Executor` is, and with the same names: construct it, hand it
    the flux subscriptions and the nodes, then spin.

        ex = flux.ros.Executor()
        ex.add_flux(sub)
        ex.add_ros_node(node)
        ex.spin()
    """

    def __init__(self, *, rclpy_executor=None, max_channels=32, poll_tick_ns=2_000_000):
        from rclpy.executors import SingleThreadedExecutor

        self._nodes = []
        self._flux = _FluxExecutor(max_channels=max_channels, poll_tick_ns=poll_tick_ns)
        self._exec = rclpy_executor or SingleThreadedExecutor()

        self._drained = threading.Event()
        self._drained.set()
        self._running = False
        self._closed = False
        self._thread = None
        # One callable, reused for every task: rclpy allocates a Task per create_task and there
        # is no way around that (a Task is one-shot -- it refuses to run again once finished),
        # but the body it carries need not be a new object. Weak, because a task queued just as
        # the context goes down is never run and never dropped, and a strong reference there
        # would keep this executor's channels mapped for the life of the process.
        self._dispatch_task = _weak_dispatch(weakref.ref(self))

    # ---- registration ----

    def add_flux(self, subscription, callback=None, priority=0):
        """Register a flux Subscription; its own callback runs per frame.

        Same contract as flux.Executor.add. `callback` is for a bare flux.Subscription, which
        has nowhere to carry one; a flux.ros.Subscription brings its own. `priority` orders the
        visit within one dispatch pass, higher first, ties in registration order.

        A `flux.ros.message_filters.Subscriber` is accepted here in place of the Subscription it
        drives, so a filter graph is registered the way C++ registers one (`ex.add(left)`).
        """
        if self._running:
            raise RuntimeError("flux: add_flux() must be called before spin()")
        subscription = _flux_source_of(subscription)
        self._flux.add(subscription, _callback_of(subscription, callback), priority)

    def add_ros_node(self, node):
        """Register a node: its ROS callbacks run on this executor's spin thread.

        Do not also hand the node to another executor -- rclpy keeps one `node.executor`, and the
        second add silently takes the node away from the first. Idempotent per node.
        """
        if self._running:
            raise RuntimeError("flux: add_ros_node() must be called before spin()")
        if self._closed:
            raise RuntimeError("flux.ros.Executor is closed")
        for n in self._nodes:
            if n is node:
                return
        self._nodes.append(node)
        self._exec.add_node(node)

    # ---- spin ----

    def spin(self, tick_ns=100_000_000):
        """Block until stop(). ROS and flux callbacks both run on this thread.

        `tick_ns` bounds each wait so a stop is noticed promptly (negative = block until an
        event). Nanoseconds, like every other flux timeout.
        """
        if self._running:
            raise RuntimeError("flux.ros.Executor.spin() is already running")
        if self._closed:
            raise RuntimeError("flux.ros.Executor is closed")
        self._running = True
        if not self._nodes:
            # No ROS side to serve: this is the flux half of a split (see the executor example),
            # and the C++ counterpart with no node does exactly this too. Driving an empty rclpy
            # executor instead would add nothing and raise ExternalShutdownException on the way
            # down, on whatever thread this is.
            try:
                while self._running:
                    self._flux.spin_once(tick_ns)
            finally:
                self._running = False
            return
        self._start_bridge()
        try:
            while self._running:
                self._exec.spin_once(timeout_sec=_sec(tick_ns))
        finally:
            self._stop_bridge()

    def spin_once(self, timeout_ns=100_000_000):
        """One pass. Dispatches ready flux frames first, then serves ROS for up to timeout_ns."""
        if self._running:
            # dispatch() would race the bridge's wait_for_work() on the shared io_uring.
            raise RuntimeError("spin_once() must not run concurrently with spin()")
        self._flux.dispatch()
        self._exec.spin_once(timeout_sec=_sec(timeout_ns))

    def stop(self):
        """End spin(). Safe from a callback or another thread. Idempotent."""
        self._running = False
        self._flux.interrupt()
        # Nothing to wake once the context is down: rclpy's spin_once raises out of the loop on
        # its own, so the task would only sit in the queue of an executor nobody spins again.
        if not self._closed and any(n.context.ok() for n in self._nodes):
            self._exec.create_task(_nothing)  # wake the spin thread out of its wait

    def interrupt(self):
        """Break the current wait without ending the loop. Safe from another thread."""
        self._flux.interrupt()
        if not self._closed and any(n.context.ok() for n in self._nodes):
            self._exec.create_task(_nothing)

    def close(self):
        """Detach from the rclpy executor. Idempotent.

        Call after the final stop(), before destroying the nodes. A closed executor cannot spin
        again. A bridge thread that failed to stop (wedged in a callback) keeps the executor
        attached: detaching a node another thread may still queue work against is worse.
        """
        self.stop()
        self._stop_bridge()
        if self._thread is not None:
            return
        if not self._closed:
            self._closed = True
            for node in self._nodes:
                self._exec.remove_node(node)

    def __len__(self):
        """Registered flux subscriptions. `ex.size()` on the C++ side."""
        return len(self._flux)

    @property
    def uses_io_uring(self):
        """True if every flux channel is waited on through one io_uring (Linux 6.7+).

        False means the kernel lacks the opcode and the parker-thread fallback is active.
        """
        return self._flux.uses_io_uring

    # ---- internals ----

    def _dispatch_on_spin(self):
        # Runs on the rclpy spin thread: the only thread allowed to touch flux channels.
        try:
            self._flux.dispatch()
        finally:
            self._drained.set()  # release the bridge even if a callback raised

    def _bridge(self):
        _name_this_thread("flux-bridge")
        while self._running:
            self._flux.wait_for_work(200_000_000)  # 200 ms: bounds shutdown, not latency
            if not self._running:
                break
            self._drained.clear()
            # stop() can flip _running between the check above and this clear. If it did, the
            # spin thread has already left its loop and will never run the task, so
            # _drained.wait() below would block until _stop_bridge's join times out. Restore
            # the event (a concurrent _stop_bridge also sets it) and leave.
            if not self._running:
                self._drained.set()
                break
            self._exec.create_task(self._dispatch_task)
            # Do not re-enter wait_for_work until dispatch() has finished: both drive the
            # same io_uring, and concurrent submit/wait on one ring is a data race. Bounded,
            # so a spin thread that died without running the task cannot strand this bridge
            # forever -- a queued task stays queued, so nothing is lost by waiting again.
            while self._running and not self._drained.wait(timeout=0.2):
                pass

    def _start_bridge(self):
        self._flux.dispatch()  # attach and arm on this thread before the bridge starts waiting
        self._thread = threading.Thread(target=self._bridge, daemon=True, name="flux-bridge")
        self._thread.start()

    def _stop_bridge(self):
        self._running = False
        self._flux.interrupt()
        self._drained.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                return  # wedged: close() stays attached rather than detach under a live task
            self._thread = None


class PartitionedExecutor:
    """One thread per partition, so no unit's callback can delay another's.

    The C++ counterpart (`flux::ros::PartitionedExecutor`) makes the
    callback group the unit throughout: a child executor is handed the group with
    `add_callback_group`, so the group's ROS callbacks and its flux frames land on the same
    child thread and the group's mutual exclusion survives the split.

    rclpy has no `add_callback_group`. Its executor granularity is the node -- `add_node` writes
    `node.executor`, and a node belongs to one executor at a time. So the unit splits in two here,
    each as fine as rclpy allows it to be:

        add_ros_node(node)              -> one thread per NODE, serving all its ROS callbacks
        add_flux(sub, cb, group=g)      -> one thread per GROUP, serving its flux frames

    A group handed to add_flux must hold no ROS entities. Not a style rule: rclpy cannot move a
    timer or a subscription onto this group's thread, so its ROS callbacks would keep running on
    the node's thread while flux frames ran here -- the group's mutual exclusion broken with
    nothing said. Refusing is the same discipline the C++ side applies to a Reentrant group.
    A group that genuinely needs both transports on one thread is what `flux.ros.Executor` is.

    `set_thread_scheduling` settles which of those threads the kernel prefers and which cores it
    may use. Deliberately not named `schedule` like the C++ method: that one takes a Strictness
    and a control-loop priority and is part of a hard-RT chain declaration, and none of that
    carries here. The Python path is not an RT target -- GIL and GC are unbounded latency sources
    -- and a priority is not a bound.

    What the extra threads buy is real but conditional: they overlap only work that releases the
    GIL (numpy, zlib, decode, memcpy in an extension). Callbacks that are pure Python bytecode
    serialize on the GIL no matter how many threads serve them.
    """

    def __init__(self, *, poll_tick_ns=2_000_000):
        self._poll_tick_ns = poll_tick_ns
        self._assigned = []  # [(group, [(sub, callback, priority), ...])] -- insertion ordered
        self._sync_groups = []  # [[filter, ...]] -- declared synchronizer input sets
        self._unplaced_sync_inputs = 0
        self._sched = []  # [(unit, {policy, priority, cpus})] -- unit is a group or a node
        self._nodes = []
        self._children = []
        self._spinning = False
        self._closed = False
        self._running = False
        self._child_run = False
        # Parent's wait. A child sets it to report a failure or a context shutdown, so the parent
        # reacts at once instead of at the end of its tick.
        self._wake = threading.Event()
        self._error_lock = threading.Lock()
        self._error = None

    # ---- registration ----

    def add_flux(self, subscription, group, callback=None, priority=0):
        """Assign a flux Subscription to a partition group: same group, same thread.

        The callback comes from the subscription, as it does for Executor.add_flux. `group` is an
        rclpy callback group used purely as the partition token -- it must hold no ROS entities
        (see the class docstring). Subscriptions sharing a group share one thread, in the order
        they were added, unless `priority` reorders them: higher goes first within that group's
        pass, and it never crosses groups, which are separate threads.
        """
        from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup

        if self._spinning:
            raise RuntimeError("flux: add_flux() must be called before spin()")
        if self._closed:
            raise RuntimeError("flux: this PartitionedExecutor is closed")
        subscription = _flux_source_of(subscription)
        callback = _callback_of(subscription, callback)
        if not isinstance(group, CallbackGroup):
            raise TypeError(
                "add_flux(sub, group) expects an rclpy callback group as the second "
                "argument -- rclpy.callback_groups.MutuallyExclusiveCallbackGroup()"
            )
        # Same refusal as C++: one thread per group is exactly what a Reentrant group asks not to
        # have, and serializing its callbacks quietly is the downgrade this executor exists to
        # refuse. Split the work into separate MutuallyExclusive groups instead.
        if isinstance(group, ReentrantCallbackGroup):
            raise ValueError(
                "flux: a Reentrant callback group cannot be served here -- "
                "PartitionedExecutor gives each group exactly one thread, so its callbacks "
                "would serialize; split them into separate MutuallyExclusive groups"
            )
        for _g, subs in self._assigned:
            for s, _cb, _p in subs:
                if s is subscription:
                    raise ValueError("flux: this Subscription is already assigned to a group")
        for g, subs in self._assigned:
            if g is group:
                subs.append((subscription, callback, priority))
                return
        self._assigned.append((group, [(subscription, callback, priority)]))

    def add_sync_group(self, *inputs):
        """Declare that these filters are the inputs of one synchronizer, so spin() can check it.

        A sync policy runs the matched callback holding its own `threading.Lock`, so inputs
        served by two threads couple through that lock. A synchronizer does not tell anyone what
        its inputs are, so the set comes from outside:

            ex.add_flux(left, g)
            ex.add_flux(right, g)
            ex.add_sync_group(left, right)   # these two are one synchronizer
            ex.spin()                        # raises if they are not on one thread

        Accepts a flux Subscriber, a flux Subscription, and an upstream
        `message_filters.Subscriber`. A mix of flux and DDS inputs is refused: rclpy has no
        `add_callback_group`, so the DDS input stays on its node's thread and no group assignment
        can bring it here. `flux.ros.Executor` is what serves both on one thread.

        Declaring nothing checks nothing, as before.
        """
        if self._spinning:
            raise RuntimeError("flux: add_sync_group() must be called before spin()")
        if self._closed:
            raise RuntimeError("flux: this PartitionedExecutor is closed")
        if len(inputs) < 2:
            raise ValueError("flux: add_sync_group() needs at least two inputs")
        self._sync_groups.append(list(inputs))

    def unplaced_sync_inputs(self):
        """Declared inputs the last spin() could not place, and so did not judge.

        A chain-middle filter and a Cache are the cases: neither names a subscription, so there
        is no thread to compare. 0 means every declared input was judged.
        """
        return self._unplaced_sync_inputs

    def add_ros_node(self, node):
        """Register a node: its ROS callbacks get a thread of their own.

        Do not also hand the node to another executor -- rclpy keeps one `node.executor`, and the
        second add silently takes the node away from the first.
        """
        if self._spinning:
            raise RuntimeError("flux: add_ros_node() must be called before spin()")
        if self._closed:
            raise RuntimeError("flux: this PartitionedExecutor is closed")
        for n in self._nodes:
            if n is node:
                return
        self._nodes.append(node)

    def set_thread_scheduling(self, unit, *, policy=None, priority=0, cpus=()):
        """Declare the OS scheduling for the thread that will serve `unit`.

        `unit` is a partition of this executor: a callback group handed to `add_flux`, or a node
        handed to `add_ros_node`. Both, because rclpy splits the unit in two here and a
        node's thread is as worth prioritizing as a group's.

            control = MutuallyExclusiveCallbackGroup()
            ex.add_flux(front_camera, control)
            ex.set_thread_scheduling(control, cpus=[4, 5])
            ex.set_thread_scheduling(logging_node, cpus=[0])

        The child applies this to itself before running any callback, because a thread is the
        only one that can put itself on a policy. A refusal ends the spin with that error; it is
        never downgraded to a weaker setting.

        This is not a real-time guarantee and does not become one at any priority: the GIL and
        the GC stay unbounded latency sources. What it settles is which thread the
        kernel prefers when several are runnable, and which cores each may use -- which decides
        something real whenever the callbacks release the GIL, and nothing at all when they do
        not. `flux.rt.Policy.Fifo` needs RLIMIT_RTPRIO or CAP_SYS_NICE; affinity alone needs
        neither.

        One declaration per unit, and the unit must actually be served here: a second call for
        the same unit and a unit this executor never spawns are both refused, the first now and
        the second at spin(). A declaration that silently never applies is the failure this
        argues against everywhere else.
        """
        from .. import rt

        if self._spinning:
            raise RuntimeError("flux: set_thread_scheduling() must be called before spin()")
        if self._closed:
            raise RuntimeError("flux: this PartitionedExecutor is closed")
        for declared, _opts in self._sched:
            if declared is unit:
                raise ValueError(
                    "flux: this unit already has a thread scheduling declaration. One per unit, "
                    "so which one won is never a question of call order"
                )
        self._sched.append(
            (unit, {
                "policy": rt.Policy.Inherit if policy is None else policy,
                "priority": priority,
                "cpus": list(cpus),
            }),
        )

    # ---- spin ----

    def spin(self, tick_ns=100_000_000):
        """Spawn one thread per unit and block until stop().

        The calling thread runs no callback. It spawns the children, then re-checks every
        `tick_ns` that no flux group has grown a ROS entity behind its back. An exception on any
        child thread stops every child and is re-raised here.
        """
        if self._spinning:
            raise RuntimeError("flux: spin() called while already spinning")
        if self._closed:
            raise RuntimeError("flux: this PartitionedExecutor is closed")
        self._spinning = True
        self._running = True
        self._child_run = True
        with self._error_lock:
            self._error = None
        self._wake.clear()

        tick_sec = None if tick_ns < 0 else max(tick_ns, 1_000_000) / 1e9
        scan_error = None
        try:
            self._check_sync_groups()
            self._check_scheduling()
            self._spawn(tick_ns)
            while self._running and self._child_run:
                if self._wake.wait(timeout=tick_sec):
                    self._wake.clear()
                for group, _subs in self._assigned:
                    _require_flux_only(group)
        except BaseException as exc:  # noqa: BLE001 - re-raised after every child is joined
            scan_error = exc

        self._child_run = False
        for child in self._children:
            child.stop()
        for child in self._children:
            child.join()
        self._children = []
        self._spinning = False
        self._running = False

        if scan_error is not None:
            raise scan_error
        with self._error_lock:
            child_error = self._error
            self._error = None
        if child_error is not None:
            raise child_error

    def stop(self):
        """End spin(). Safe from a callback or another thread. Idempotent."""
        self._running = False
        self._wake.set()

    def interrupt(self):
        """Break the parent's current wait without ending the spin. Safe from another thread."""
        self._wake.set()

    def close(self):
        """Stop and refuse further use. Idempotent.

        Child executors are torn down by spin() itself, so this only bars a restart; call it
        after the final spin() returns and before destroying the nodes.
        """
        self.stop()
        self._closed = True

    # ---- internals ----

    def _check_sync_groups(self):
        placed = {}
        for group, subs in self._assigned:
            for sub, _cb, _priority in subs:
                placed[id(sub)] = group
        self._unplaced_sync_inputs = 0
        for inputs in self._sync_groups:
            threads = []
            for f in inputs:
                token = _sync_input_thread(f)
                if token is None:
                    self._unplaced_sync_inputs += 1
                    continue
                kind, owner = token
                if kind == "flux":
                    group = placed.get(id(owner))
                    if group is None:
                        raise ValueError(
                            "flux: a synchronizer input is not assigned to any group, so no "
                            "child executor drives it and its partners wait forever -- pass it "
                            "to add_flux(sub, group)"
                        )
                    threads.append(("flux", id(group), f))
                else:
                    threads.append(("ros", id(owner), f))
            if len({(kind, owner) for kind, owner, _f in threads}) < 2:
                continue
            if {kind for kind, _owner, _f in threads} == {"flux", "ros"}:
                raise ValueError(
                    "flux: this synchronizer mixes a flux input with a DDS input, and "
                    "PartitionedExecutor cannot put them on one thread -- rclpy has no "
                    "add_callback_group, so the DDS input is served by its node's thread while "
                    "the flux input runs on its group's. The sync policy's lock would join the "
                    "two, which is the isolation this executor exists to give. Use "
                    "flux.ros.Executor, which serves ROS and flux on one thread."
                )
            raise ValueError(
                "flux: the inputs of one synchronizer are spread across threads. Assign every "
                "flux input of this synchronizer to the same callback group; ROS inputs must "
                "come from one node, since each node gets its own thread here."
            )

    def _check_scheduling(self):
        """Refuse a declaration for a unit no child will serve.

        Such a declaration reads as applied and never runs, which is the silent downgrade the
        whole rt surface exists to prevent. A typo in the group variable is the usual way in.
        """
        for unit, _opts in self._sched:
            if any(group is unit for group, _subs in self._assigned):
                continue
            if any(node is unit for node in self._nodes):
                continue
            raise ValueError(
                "flux: set_thread_scheduling() was given a unit this executor does not serve, so "
                "no thread would ever apply it. Pass the callback group you handed to add_flux() "
                "or the node you handed to add_ros_node()"
            )

    def _sched_for(self, unit):
        for declared, opts in self._sched:
            if declared is unit:
                return opts
        return None

    def _spawn(self, tick_ns):
        from rclpy.executors import SingleThreadedExecutor

        for index, (group, subs) in enumerate(self._assigned):
            _require_flux_only(group)
            flux_ex = _FluxExecutor(max_channels=len(subs), poll_tick_ns=self._poll_tick_ns)
            for subscription, callback, priority in subs:
                flux_ex.add(subscription, callback, priority)
            name = f"flux-part-g{index}"
            self._start(
                _FluxChild(flux_ex, tick_ns, self._fail, name, self._sched_for(group)), name
            )

        for index, node in enumerate(self._nodes):
            ros_ex = SingleThreadedExecutor()
            ros_ex.add_node(node)
            # Not ros_ex.spin(): shutdown() is terminal and wake() is not sticky, so a stop racing
            # the thread's startup would be lost and the join would hang. A tick-bounded
            # spin_once loop re-reads the run flag the way the flux children do, which makes
            # shutdown finite without the race. Same reasoning as the C++ ROS child.
            name = f"flux-part-n{index}"
            self._start(
                _RosChild(
                    ros_ex, node, tick_ns, self._fail, self._child_running, self._shutdown_seen,
                    name, self._sched_for(node),
                ),
                name,
            )

    def _start(self, child, name):
        child.thread = threading.Thread(target=child.run, name=name, daemon=True)
        self._children.append(child)  # appended first: a start() that throws still gets joined
        child.thread.start()

    def _child_running(self):
        return self._child_run

    def _fail(self, exc):
        with self._error_lock:
            if self._error is None:
                self._error = exc
        self._child_run = False
        self._wake.set()

    def _shutdown_seen(self):
        # A child saw the context go down. Not an error: end the spin the way stop() would.
        self._child_run = False
        self._wake.set()


class _FluxChild:
    """A group's thread: one flux.Executor, spun directly. No rclpy bridge -- the group holds no
    ROS entity, so there is nothing on this thread to merge with."""

    def __init__(self, flux_ex, tick_ns, on_error, label, sched=None):
        self._flux = flux_ex
        self._tick_ns = tick_ns
        self._on_error = on_error
        self._label = label
        self._sched = sched
        self.thread = None

    def run(self):
        _name_this_thread(self._label)
        try:
            _apply_scheduling(self._sched)
            self._flux.spin(self._tick_ns if self._tick_ns >= 0 else 100_000_000)
        except BaseException as exc:  # noqa: BLE001 - carried to the parent's spin()
            self._on_error(exc)

    def stop(self):
        self._flux.stop()

    def join(self):
        if self.thread is None:
            return
        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            # Bounded so one wedged group cannot hang the whole teardown, but never silent: an
            # abandoned thread is what turns a stuck child into a SIGABRT at interpreter exit,
            # and that used to arrive with nothing said about where it came from.
            warnings.warn(
                f"flux: PartitionedExecutor child {self._label!r} did not stop within 5 s and was abandoned; "
                "the process may abort at interpreter shutdown",
                RuntimeWarning,
                stacklevel=2,
            )


class _RosChild:
    """A node's thread: a stock rclpy SingleThreadedExecutor, ticked so stopping is finite."""

    def __init__(self, ros_ex, node, tick_ns, on_error, running, on_shutdown, label, sched=None):
        self._exec = ros_ex
        self._node = node
        self._tick_sec = 0.1 if tick_ns < 0 else max(tick_ns, 1_000_000) / 1e9
        self._on_error = on_error
        self._running = running
        self._on_shutdown = on_shutdown
        self._label = label
        self._sched = sched
        self.thread = None

    def run(self):
        _name_this_thread(self._label)
        from rclpy.executors import ExternalShutdownException, ShutdownException

        try:
            _apply_scheduling(self._sched)
            while self._running():
                self._exec.spin_once(timeout_sec=self._tick_sec)
        except (ExternalShutdownException, ShutdownException):
            # Not an error. The parent has no other way to learn the context is gone, and
            # without this it would tick against a dead node until someone called stop().
            self._on_shutdown()
        except BaseException as exc:  # noqa: BLE001 - carried to the parent's spin()
            self._on_error(exc)

    def stop(self):
        try:
            self._exec.wake()
        except Exception:  # noqa: BLE001 - a torn-down context has nothing left to wake
            pass

    def join(self):
        if self.thread is None:
            return
        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            # Bounded so one wedged group cannot hang the whole teardown, but never silent: an
            # abandoned thread is what turns a stuck child into a SIGABRT at interpreter exit,
            # and that used to arrive with nothing said about where it came from.
            warnings.warn(
                f"flux: PartitionedExecutor child {self._label!r} did not stop within 5 s and was abandoned; "
                "the process may abort at interpreter shutdown",
                RuntimeWarning,
                stacklevel=2,
            )
        self._exec.remove_node(self._node)


def _require_flux_only(group):
    """Refuse a flux partition group that holds ROS entities.

    rclpy cannot hand a callback group to a child executor, so the entities in this group would
    keep being served on their node's thread while flux frames were served here. That is the
    group's mutual exclusion broken, and breaking it quietly is worse than refusing.
    """
    live = [ref() for ref in tuple(group.entities)]
    live = [entity for entity in live if entity is not None]
    if not live:
        return
    kinds = sorted({type(entity).__name__ for entity in live})
    raise ValueError(
        "flux: a callback group handed to add_flux() must hold no ROS entities, but this one "
        f"holds {len(live)} ({', '.join(kinds)}). rclpy has no add_callback_group, so those "
        "callbacks stay on their node's thread while flux frames run on this group's thread -- "
        "the group's mutual exclusion would be broken silently. Give the flux subscriptions a "
        "callback group of their own, or use flux.ros.Executor, which serves ROS and flux on one "
        "thread."
    )


try:
    _prctl = ctypes.CDLL(None, use_errno=True).prctl
    _prctl.argtypes = [ctypes.c_int, ctypes.c_char_p,
                       ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
    _prctl.restype = ctypes.c_int
except (OSError, AttributeError):
    _prctl = None


def _apply_scheduling(sched):
    """Put the calling child on its declared policy and cpus, before its first callback.

    Called on the child thread rather than by the parent because a thread is the only one that
    can set its own policy, and because a refusal has to reach the parent's spin() as an error
    rather than as a thread quietly running at the wrong priority.
    """
    if sched is None:
        return
    from .. import rt

    rt.apply(policy=sched["policy"], priority=sched["priority"], cpus=sched["cpus"])


def _name_this_thread(name):
    """Put `name` on the OS thread too. threading.Thread(name=) stops at Python.

    Worth the three lines: without it every executor thread this module starts shows as
    `python3` in top -H, perf and /proc, which is where anyone asks what the extra threads of
    a partitioned executor actually cost. PR_SET_NAME truncates at 15 bytes.
    """
    if _prctl is None:
        return
    try:
        _prctl(15, name.encode()[:15], 0, 0, 0)  # PR_SET_NAME
    except (OSError, ValueError):
        pass  # naming is a convenience; never fail a spin over it


def _sec(timeout_ns):
    """Nanoseconds to the seconds rclpy wants. Negative means block until an event, which rclpy
    spells as None."""
    return None if timeout_ns < 0 else timeout_ns / 1e9


def _nothing():
    """Task body whose only effect is waking the executor out of its wait."""


def _weak_dispatch(ref):
    """Task body that drops through if the Executor it belongs to is already gone."""

    def run():
        executor = ref()
        if executor is not None:
            executor._dispatch_on_spin()

    return run


def resolve(node, topic):
    """Expand a ROS topic name against a node's namespace and remap rules.

    flux derives the shm segment name from the FULLY RESOLVED topic, so both ends must resolve
    identically or they name different segments and never meet. A C++ flux node resolves through
    its rclcpp node automatically; this is the rclpy equivalent.
    """
    return node.resolve_topic_name(topic)


class Publisher(_Publisher):
    """flux.Publisher whose topic is resolved against `node` (namespace + remaps applied)."""

    def __init__(self, node, topic, **kwargs):
        super().__init__(resolve(node, topic), **kwargs)


class Subscription(_Subscription):
    """flux.Subscription that resolves its topic against `node` and carries its own callback.

    The callback belongs here, not to the executor call, for the reason it does in rclpy and in
    flux_cpp: a subscription is a topic plus what to do with it, and splitting the two lets the
    same subscription be registered twice with different callbacks. `add_flux(sub)` reads it.

    Leaving `callback` unset is allowed and gives the pull surface -- peek/take/take_blocking on
    your own schedule. Only `add_flux` requires one.
    """

    def __init__(self, node, topic, callback=None, **kwargs):
        super().__init__(resolve(node, topic), **kwargs)
        if callback is not None and not callable(callback):
            raise TypeError("flux: callback must be callable")
        self.callback = callback


def _flux_source_of(subscription):
    """The flux Subscription behind whatever was handed to add_flux().

    A `flux.ros.message_filters.Subscriber` is not a Subscription -- nanobind refuses a second
    base class, so it owns one instead of being one. Unwrapping it here is what lets the same
    `add_flux(sub)` take either, as `ex.add(left)` does in C++.
    """
    inner = getattr(subscription, "subscription", None)
    return inner if isinstance(inner, _Subscription) else subscription


def _sync_input_thread(f):
    """Which thread would serve this synchronizer input, as a token to compare, or None.

    A flux Subscription is served by the callback group it was assigned to; an upstream
    message_filters.Subscriber by its node, which PartitionedExecutor gives a thread of its own.
    Anything else -- a chain-middle filter, a Cache -- cannot be placed and is not judged.
    """
    inner = getattr(f, "subscription", None)
    if isinstance(inner, _Subscription):
        return ("flux", inner)
    if isinstance(f, _Subscription):
        return ("flux", f)
    node = getattr(f, "node", None)
    return None if node is None else ("ros", node)


def _callback_of(subscription, callback):
    """Resolve which callback to run: the explicit one, else the subscription's own.

    A flux.ros.Subscription carries its callback, which is the shape flux_cpp and rclpy both
    have. A bare flux.Subscription cannot -- it is the pull surface and has nowhere to put one --
    so the explicit argument stays for that case.
    """
    if not isinstance(subscription, _Subscription):
        raise TypeError("add_flux expects a flux.Subscription")
    own = getattr(subscription, "callback", None)
    if callback is not None and own is not None and callback is not own:
        raise ValueError(
            "flux: this Subscription already carries a callback; pass one or the other, not two"
        )
    resolved = callback if callback is not None else own
    if resolved is None:
        raise TypeError(
            "flux: no callback -- construct the subscription as "
            "flux.ros.Subscription(node, topic, callback=fn), or pass one to add_flux()"
        )
    return resolved
