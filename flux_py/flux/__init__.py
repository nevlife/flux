"""flux for rclpy nodes: zero-copy frames between processes on one host.

A Subscription hands its callback a read-only numpy view of the frame in shared memory. The view
holds the frame's slot until it is garbage: copy out what is kept long, or the publisher runs
out of slots (QoS max_borrow).
"""

import atexit
import contextlib
import functools
import threading
import weakref

from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

from . import _flux
from ._flux import _register_layout, _register_stamp_offset  # noqa: F401  generated code calls
from ._flux import (
    NO_SCHEMA,
    Error,
    FenceWait,
    Frame,
    Loan,
    MemoryPolicy,
    Platform,
    Published,
    QoS,
    Refused,
    Route,
    Status,
    aux,
    enumerate_topics,
    faulted,
    gpu_wedged,
    probe,
    read_channel_stats,
    gpu_fingerprint,
    ros_type_of,
)

__all__ = [
    "NO_SCHEMA",
    "Error",
    "Executor",
    "FenceWait",
    "Frame",
    "Loan",
    "MemoryPolicy",
    "PartitionedExecutor",
    "Platform",
    "Published",
    "Publisher",
    "QoS",
    "Refused",
    "Route",
    "Status",
    "Subscription",
    "arena_scope",
    "aux",
    "enumerate_topics",
    "faulted",
    "gpu_wedged",
    "probe",
    "read_channel_stats",
    "gpu_fingerprint",
    "ros_type_of",
]

# 0: the slot of the fingerprint's generated type (its scalar block, SLOT_STRING_BYTES for its
# own strings and the arena trailer, in pages), or UNTYPED_SLOT_SIZE without one.
DEFAULT_SLOT_SIZE = _flux.DEFAULT_SLOT_SIZE
UNTYPED_SLOT_SIZE = _flux.UNTYPED_SLOT_SIZE
SLOT_STRING_BYTES = _flux.SLOT_STRING_BYTES
DEFAULT_SLOT_COUNT = _flux.DEFAULT_SLOT_COUNT
# Subscribing processes, not subscriptions; each adds a counter that every publish reads.
DEFAULT_MAX_SUBSCRIBERS = _flux.DEFAULT_MAX_SUBSCRIBERS
_TICK_NS = _flux.DEFAULT_TICK_NS
# rclpy raises it, not ExternalShutdownException, when the context shuts down as it builds a wait set.
_RCLError = _rclpy.RCLError
_InvalidHandle = _rclpy.InvalidHandle
# How long one dispatch may run before the ROS side gets its turn.
_TURN_NS = _flux.DEFAULT_TURN_NS
# Spins still running, as at exit they may be on daemon threads.
_live = weakref.WeakSet()
_MIRROR_POLL_S = 0.1


def _cuda(device):
    if device in ("cpu", "cuda"):
        return device == "cuda"
    raise ValueError(f"flux: device is 'cpu' or 'cuda', not {device!r}")


def _on_device(array):
    if hasattr(array, "__cuda_array_interface__"):
        return True
    dlpack_device = getattr(array, "__dlpack_device__", None)
    return dlpack_device is not None and dlpack_device()[0] != 1


@contextlib.contextmanager
def arena_scope():
    """This thread's allocations go to the process arena while open.

    Needs LD_PRELOAD=libflux_heaphook.so and the arena on (FLUX_ARENA_BYTES not 0); yields False
    without them. An array made inside can be handed to a Builder's adopt__<field>() without a
    copy.
    """
    opened = _flux._arena_scope_enter()
    try:
        yield opened
    finally:
        if opened:
            _flux._arena_scope_leave()


def _timeout_ns(seconds):
    if seconds is None:
        return 0
    if seconds <= 0:
        raise ValueError("flux: fence_timeout must be positive, or None for no bound")
    return max(1, int(seconds * 1e9))


def _start_mirror(publisher, node):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    mirror = _flux._RosMirror(publisher, publisher.signpost_name, publisher._fingerprint)
    ros_pub = node.create_publisher(
        get_message(mirror.ros_type),
        publisher.topic_name,
        QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
        ),
    )
    stop = threading.Event()
    # The thread holds the mirror, not the publisher, so dropping the publisher stops it.
    weakref.finalize(publisher, stop.set)
    threading.Thread(
        target=_mirror,
        args=(mirror, ros_pub, stop, node.get_logger()),
        name=f"flux mirror {publisher.topic_name}",
        daemon=True,
    ).start()


def _mirror(mirror, ros_pub, stop, logger):
    try:
        while not stop.is_set():
            if ros_pub.get_subscription_count() == 0:
                mirror.detach()
                stop.wait(_MIRROR_POLL_S)
                continue
            data = mirror.next(int(_MIRROR_POLL_S * 1e9))
            if data is not None:
                ros_pub.publish(data)
    except (_InvalidHandle, _RCLError):
        pass  # the node or its context went first
    except Exception as e:
        # Raised on a thread, it would only reach stderr.
        logger.error(f"{ros_pub.topic_name}: ROS 2 mirror stopped: {e}")


class Publisher(_flux._Publisher):
    """One per topic on the host: Error with Status.PUBLISHER_EXISTS while another is live.

    device="cuda" declares a stream flux creates: a commit waits for it, and a loan is written on
    the device through __cuda_array_interface__ or __dlpack__.
    """

    def __init__(
        self,
        node,
        topic,
        fingerprint=NO_SCHEMA,
        slot_size=DEFAULT_SLOT_SIZE,
        slot_count=DEFAULT_SLOT_COUNT,
        memory=None,
        max_subscribers=DEFAULT_MAX_SUBSCRIBERS,
        device="cpu",
        fence_timeout=None,
        mirror_to_ros=False,
        aux_size=0,
    ):
        """fence_timeout, in seconds, bounds each commit's wait for the stream; None waits as long
        as it takes. A commit that runs out is FENCE_FAILED, and its slot comes back once the
        work is done. mirror_to_ros also publishes every frame as its ROS 2 message on the same
        topic while a ROS 2 subscriber wants it (depth 1, best effort, volatile). aux_size gives
        each slot that many bytes of host memory beside its payload: a loan writes them through
        loan.aux and loan.aux_len, and flux.aux(frame) reads them."""
        bound = _timeout_ns(fence_timeout)
        self.topic_name = node.resolve_topic_name(topic)
        self.signpost_name = _flux.signpost_name(self.topic_name, fingerprint)
        self._fingerprint = fingerprint
        super().__init__(
            self.signpost_name,
            self.topic_name,
            node.get_fully_qualified_name(),
            fingerprint,
            slot_size,
            slot_count,
            memory if memory is not None else MemoryPolicy(),
            max_subscribers,
            _cuda(device),
            aux_size,
        )
        self._set_fence_timeout(bound)
        if mirror_to_ros:
            _start_mirror(self, node)

    def publish(self, array):
        """A host array. A device array is refused: loan() a slot and write it on the device."""
        if _on_device(array):
            raise TypeError("flux: publish() takes host arrays; loan() a slot for a device array")
        return super().publish(array)


class Subscription(_flux._Subscription):
    """Attaches once a publisher is up, and again after it restarts.

    Raises on a QoS the channel would refuse, before anything is announced. Under an executor, a
    subscription refused when it attaches is logged once and left out until its publisher
    restarts; the others keep running.
    """

    def __init__(
        self,
        node,
        topic,
        callback=None,
        fingerprint=NO_SCHEMA,
        qos=None,
        memory=None,
        device="cpu",
        fence_timeout=None,
        release="wait",
    ):
        """device="cuda" hands out Frames instead of numpy views: their memory is reachable
        inside `with sub.take() as frame:` or the callback, and leaving either waits for the
        subscription's stream before the slot goes back. fence_timeout, in seconds, bounds that
        wait; None waits as long as it takes. A release that runs out keeps its slot until the
        work is done, and counts in fence_failed. release="deferred" does not wait: the slot goes
        back once the device is done, at the next take or executor pass, and counts against
        max_borrow until then."""
        if callback is not None and not callable(callback):
            raise TypeError("flux: callback must be callable")
        if release not in ("wait", "deferred"):
            raise ValueError('flux: release must be "wait" or "deferred"')
        bound = _timeout_ns(fence_timeout)
        self.topic_name = node.resolve_topic_name(topic)
        self.callback = callback
        self._logger = node.get_logger()
        # Weak: a thread that still holds the subscription at exit must not keep rclpy's context,
        # and so its middleware, alive.
        self._context = weakref.ref(node.context)
        super().__init__(
            _flux.signpost_name(self.topic_name, fingerprint),
            self.topic_name,
            node.get_fully_qualified_name(),
            fingerprint,
            qos if qos is not None else QoS(),
            memory if memory is not None else MemoryPolicy(),
            _cuda(device),
        )
        self._set_fence_timeout(bound)
        self._set_release_deferred(release == "deferred")

    def take_blocking(self, timeout_ns=-1):
        """With no publisher yet it also waits for one; a negative timeout waits forever. None at
        the timeout, once the node's context is shut down, and at once while max_borrow views
        are held."""
        return super().take_blocking(timeout_ns, self._context_ok)

    def _context_ok(self):
        return _all_ok([self._context])

    def _left_out(self, why):
        self._logger.error(
            f"flux: {self.topic_name} is left out until its publisher restarts: {why}"
        )


class Executor:
    """flux subscriptions and rclpy nodes, with every callback on the thread that calls spin().

    With an rclpy executor, given or made by add_node(), spin() spins it. Helper threads wait for
    frames and hand each flux pass to it as a task, so any rclpy executor works, EventsExecutor
    included; a pass no task has taken yet runs between rclpy callbacks. A pass ends after
    _TURN_NS: frames that keep coming would otherwise starve the ROS callbacks.
    """

    def __init__(self, rclpy_executor=None):
        self._flux = _flux._Executor()
        # Made only once a node is added: flux alone needs no rclpy executor.
        self._ros = rclpy_executor
        self._nodes = []
        self._contexts = []
        self._lock = threading.Lock()
        self._spinning = False
        self._stop = False
        self._error = None
        self._ended = threading.Event()

    def add(self, source, priority=0):
        """A Subscription with a callback, or a flux.message_filters.Subscriber."""
        # Under the lock spin() takes, so a spin starting meanwhile cannot walk the sources.
        with self._lock:
            if self._spinning:
                raise RuntimeError("flux: add() while spinning")
            sub = _with_callback(source)
            self._flux.add(sub, sub.callback, priority)
            _add_context(self._contexts, sub._context)

    def add_node(self, node):
        with self._lock:
            if self._spinning:
                raise RuntimeError("flux: add_node() while spinning")
            if node not in self._nodes:
                if self._ros is None:
                    self._ros = SingleThreadedExecutor(context=node.context)
                self._ros.add_node(node)
                self._nodes.append(node)

    def __len__(self):
        return len(self._flux)

    def spin(self):
        """Runs until stop(), a callback raises, or the rclpy context shuts down."""
        with self._lock:
            if self._spinning:
                raise RuntimeError("flux: spin() while already spinning")
            self._spinning = True
        self._ended.clear()
        _live.add(self)
        try:
            if self._ros is not None:
                self._spin_with_ros()
            else:
                self._flux.serve(self._contexts_ok)
        finally:
            self._stop = False
            self._spinning = False
            _live.discard(self)
            self._ended.set()

    def stop(self):
        """Ends spin() from any thread, callback or signal handler. Held if spin() has not
        started."""
        self._stop = True
        self._flux.stop()

    def _contexts_ok(self):
        return _all_ok(self._contexts)

    def _spin_with_ros(self):
        generation = self._flux.open_bridge()
        self._error = None
        post = functools.partial(self._post, generation)
        helpers = [
            threading.Thread(target=self._flux.bridge, name="flux-bridge", daemon=True),
            threading.Thread(target=self._flux.poster, args=(post,), name="flux-post", daemon=True),
        ]
        for h in helpers:
            h.start()
        try:
            while not self._stop and self._ros.context.ok():
                self._flux.dispatch_pending(_TURN_NS)
                try:
                    self._ros.spin_once(timeout_sec=_TICK_NS / 1e9)
                except ExternalShutdownException:
                    break
                except _RCLError:
                    if self._ros.context.ok():
                        raise
                    break
                if self._error is not None:
                    error, self._error = self._error, None
                    raise error
        finally:
            self._flux.close_bridge()
            for h in helpers:
                h.join()

    # On the post thread. stop() leaves the waking to it: a signal handler that took the rclpy
    # executor's task lock would deadlock if the main thread held it.
    def _post(self, generation):
        if self._ros.context.ok():
            # Weak, so a task still queued when spin ends does not keep this executor alive.
            self._ros.create_task(functools.partial(_dispatch, weakref.ref(self), generation))

    def _dispatch(self, generation):
        try:
            self._flux.dispatch_posted(_TURN_NS, generation)
        except BaseException as e:  # noqa: B036 re-raised by spin() on its own thread
            self._error = e


class PartitionedExecutor:
    """One thread per callback group for flux subscriptions, and one per node for its ROS side.

    rclpy executors take nodes, not groups. So a group given flux subscriptions must hold no ROS
    entity: that entity would run on its node's thread, and the group's callbacks would no longer
    exclude each other. spin() refuses such a group, also one that gains an entity later.
    """

    def __init__(self):
        self._groups = {}
        self._nodes = []
        self._init = {}
        self._sync = []
        self._spinning = False
        self._stop = False
        self._failure = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._ended = threading.Event()

    def add(self, source, group, priority=0):
        """A Subscription with a callback, or a flux.message_filters.Subscriber."""
        self._idle("add")
        if not isinstance(group, CallbackGroup):
            raise TypeError("flux: add() takes an rclpy CallbackGroup")
        if isinstance(group, ReentrantCallbackGroup):
            raise ValueError("flux: a Reentrant group cannot run on one thread")
        sub = _with_callback(source)
        if any(s is sub for _, subs in self._groups.values() for s, _ in subs):
            raise ValueError("flux: subscription assigned twice")
        self._groups.setdefault(id(group), (group, []))[1].append((sub, priority))

    def add_node(self, node):
        self._idle("add_node")
        if node not in self._nodes:
            self._nodes.append(node)

    def on_thread_start(self, unit, init):
        """Runs `init` on the thread of `unit`, a group or a node, before it serves."""
        self._idle("on_thread_start")
        if id(unit) in self._init:
            raise ValueError("flux: this group or node already has a thread-start hook")
        self._init[id(unit)] = (unit, init)

    def add_sync_group(self, *inputs):
        """The inputs of one synchronizer: all flux and in one group, refused otherwise."""
        self._idle("add_sync_group")
        if len(inputs) < 2:
            raise ValueError("flux: a synchronizer has at least two inputs")
        self._sync.append(inputs)

    def spin(self):
        """The first error on any thread stops every thread and is raised here."""
        with self._lock:
            self._idle("spin")
            self._spinning = True
        self._failure = None
        self._ended.clear()
        _live.add(self)
        children = []
        try:
            self._check()
            children = [
                _FluxThread(self, f"flux-g{i}", subs, self._hook(group))
                for i, (group, subs) in enumerate(self._groups.values())
            ] + [
                _RosThread(self, f"flux-n{i}", node, self._hook(node))
                for i, node in enumerate(self._nodes)
            ]
            for c in children:
                c.start()
            while not self._stop and self._failure is None and self._contexts_ok():
                self._wake.wait(_TICK_NS / 1e9)
                self._wake.clear()
                self._check_flux_only()
        except BaseException as e:  # noqa: B036 raised below, after the threads stop
            self._fail(e)
        finally:
            for c in children:
                c.stop()
            for c in children:
                c.join()
            self._spinning = False
            self._stop = False
            _live.discard(self)
            self._ended.set()
        if self._failure is not None:
            failure, self._failure = self._failure, None
            raise failure

    def stop(self):
        """Ends spin() from any thread. Held if spin() has not started."""
        self._stop = True
        self._wake.set()

    def _fail(self, e):
        if self._failure is None:
            self._failure = e
        self._wake.set()

    def _idle(self, what):
        if self._spinning:
            raise RuntimeError(f"flux: {what}() while spinning")

    def _hook(self, unit):
        return self._init.get(id(unit), (None, None))[1]

    def _contexts_ok(self):
        contexts = [weakref.ref(node.context) for node in self._nodes]
        for _, subs in self._groups.values():
            for sub, _ in subs:
                _add_context(contexts, sub._context)
        return _all_ok(contexts)

    def _group_of(self, sub):
        for key, (_, subs) in self._groups.items():
            if any(s is sub for s, _ in subs):
                return key
        raise ValueError("flux: a synchronizer input was never assigned to a group")

    def _check(self):
        served = set(self._groups) | {id(n) for n in self._nodes}
        if any(key not in served for key in self._init):
            raise ValueError("flux: a thread-start hook names a group or node no thread serves")
        self._check_flux_only()
        for inputs in self._sync:
            if not all(isinstance(i, Subscription) or hasattr(i, "subscription") for i in inputs):
                raise ValueError(
                    "flux: a synchronizer mixing flux and ROS inputs needs one thread; "
                    "use flux.Executor"
                )
            if len({self._group_of(_subscription_of(i)) for i in inputs}) > 1:
                raise ValueError(
                    "flux: a synchronizer's inputs are in different callback groups; put them in one"
                )

    def _check_flux_only(self):
        kinds = ("subscriptions", "timers", "clients", "services", "guards", "waitables")
        for node in self._nodes:
            for kind in kinds:
                for entity in getattr(node, kind):
                    if id(entity.callback_group) in self._groups:
                        raise ValueError(
                            f"flux: a group with flux subscriptions also holds a ROS {kind[:-1]} "
                            f"of {node.get_name()}; rclpy would run it on another thread"
                        )


class _FluxThread:
    def __init__(self, owner, name, subs, init):
        self._ex = _flux._Executor()
        for sub, priority in subs:
            self._ex.add(sub, sub.callback, priority)
        self._thread = threading.Thread(
            target=self._run, args=(owner, init), name=name, daemon=True
        )

    def _run(self, owner, init):
        try:
            if init is not None:
                init()
            self._ex.serve()
        except BaseException as e:  # noqa: B036 handed to the spinning thread
            owner._fail(e)

    def start(self):
        self._thread.start()

    def stop(self):
        self._ex.stop()

    def join(self):
        self._thread.join()


class _RosThread:
    def __init__(self, owner, name, node, init):
        self._node = node
        self._ex = SingleThreadedExecutor(context=node.context)
        self._ex.add_node(node)
        self._stop = False
        self._thread = threading.Thread(
            target=self._run, args=(owner, init), name=name, daemon=True
        )

    def _run(self, owner, init):
        try:
            if init is not None:
                init()
            while not self._stop and self._ex.context.ok():
                self._ex.spin_once(timeout_sec=_TICK_NS / 1e9)
                _flux._step_aside()
        except ExternalShutdownException:
            pass
        except _RCLError as e:
            if self._ex.context.ok():
                owner._fail(e)
        except BaseException as e:  # noqa: B036 handed to the spinning thread
            owner._fail(e)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True
        if self._ex.context.ok():
            self._ex.wake()

    def join(self):
        self._thread.join()
        self._ex.remove_node(self._node)
        # After its context is gone rclpy can no longer release the executor's guards.
        if self._ex.context.ok():
            self._ex.shutdown()


def _dispatch(ref, generation):
    ex = ref()
    if ex is not None:
        ex._dispatch(generation)


def _add_context(contexts, context):
    if context not in contexts:
        contexts.append(context)


def _all_ok(contexts):
    return all((c := ref()) is not None and c.ok() for ref in contexts)


def _subscription_of(source):
    return source if isinstance(source, Subscription) else source.subscription


def _with_callback(source):
    sub = _subscription_of(source)
    if sub.callback is None:
        raise ValueError(
            "flux: a Subscription without a callback gives an executor nothing to run; "
            "read it with take() instead"
        )
    return sub


# Stops spins left on daemon threads while the interpreter is whole: a thread that takes the GIL
# back once finalization has begun is ended by force before Python 3.14, and aborts.
def _before_exit():
    live = list(_live)
    for ex in live:
        ex.stop()
    for ex in live:
        ex._ended.wait(1.0)
    _flux._before_exit()


atexit.register(_before_exit)
