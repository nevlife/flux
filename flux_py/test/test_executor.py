import gc
import os
import signal
import subprocess
import sys
import threading
import time
import weakref

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from std_msgs.msg import String

import flux

SMALL = {"slot_size": 4096, "slot_count": 4}

# Humble's rclpy has no EventsExecutor.
try:
    from rclpy.experimental import EventsExecutor

    ROS_EXECUTORS = [SingleThreadedExecutor, EventsExecutor]
except ImportError:
    ROS_EXECUTORS = [SingleThreadedExecutor]


def topic(name):
    return f"/t{os.getpid()}_{name}"


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    n = Node("flux_exec_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def spin_until(ex, done, limit=3.0):
    """Spins on a thread until `done()` or `limit` seconds, then stops."""
    t = threading.Thread(target=ex.spin)
    t.start()
    end = time.monotonic() + limit
    while not done() and time.monotonic() < end:
        time.sleep(0.005)
    ex.stop()
    t.join()
    return done()


# Holds the one subscriber place of a publisher made with max_subscribers=1.
HOLD = (
    "import sys\n"
    "from flux import _flux as f\n"
    "s = f._Subscription(sys.argv[1], sys.argv[2], '', int(sys.argv[3]), f.QoS(), f.MemoryPolicy())\n"
    "print('ready', flush=True)\n"
    "sys.stdin.read()\n"
)


def hold_the_only_place(pub, fingerprint=0):
    args = [sys.executable, "-c", HOLD, pub.signpost_name, pub.topic_name, str(fingerprint)]
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    assert p.stdout.readline() == "ready\n"
    return p


def parker_threads():
    tasks = os.listdir("/proc/self/task")
    return sum(open(f"/proc/self/task/{t}/comm").read().strip() == "flux-parker" for t in tasks)


# test_executor_parked runs this file again with FLUX_DISABLE_FUTEX_WAITV set: the fallback a
# kernel below 5.16 takes.
def test_the_wait_takes_the_path_the_environment_chose(node):
    pub = flux.Publisher(node, topic("path"), **SMALL)
    got = []
    ex = flux.Executor()
    ex.add(flux.Subscription(node, topic("path"), callback=lambda v: got.append(bytes(v))))
    pub.publish(b"x")
    assert spin_until(ex, lambda: got == [b"x"])
    assert (parker_threads() > 0) == ("FLUX_DISABLE_FUTEX_WAITV" in os.environ)


def test_frames_reach_the_callback_without_ros(node):
    pub = flux.Publisher(node, topic("plain"), **SMALL)
    got = []
    ex = flux.Executor()
    sub = flux.Subscription(node, topic("plain"), callback=lambda v: got.append(bytes(v)))
    ex.add(sub)
    pub.publish(b"x")
    assert spin_until(ex, lambda: got == [b"x"])


# Its publisher came up after it, with no place left: only it is left out, logged once, until a
# publisher with room replaces that one.
def test_a_subscription_refused_at_attach_is_left_out_until_its_publisher_restarts(node):
    why, got, got_full = [], [], []
    full = flux.Subscription(node, topic("full"), callback=lambda v: got_full.append(bytes(v)))
    full._left_out = why.append
    pub_ok = flux.Publisher(node, topic("ok"), **SMALL)
    ok = flux.Subscription(node, topic("ok"), callback=lambda v: got.append(bytes(v)))
    pub_full = flux.Publisher(node, topic("full"), max_subscribers=1, **SMALL)
    holder = hold_the_only_place(pub_full)
    ex = flux.Executor()
    ex.add(full)
    ex.add(ok)
    pub_ok.publish(b"x")
    assert spin_until(ex, lambda: got == [b"x"] and why)
    assert len(why) == 1 and not full.attached

    del pub_full
    gc.collect()
    pub_full = flux.Publisher(node, topic("full"), max_subscribers=2, **SMALL)

    def sent():
        pub_full.publish(b"y")
        return got_full

    assert spin_until(ex, sent)
    assert len(why) == 1
    holder.stdin.close()
    holder.wait()


def test_a_subscription_an_executor_serves_refuses_take(node):
    pub = flux.Publisher(node, topic("served"), **SMALL)
    got = []
    ex = flux.Executor()
    sub = flux.Subscription(node, topic("served"), callback=lambda v: got.append(bytes(v)))
    ex.add(sub)
    pub.publish(b"x")
    assert spin_until(ex, lambda: got == [b"x"])
    with pytest.raises(RuntimeError, match="executor serves"):
        sub.take()


@pytest.mark.parametrize("ros_executor", ROS_EXECUTORS)
def test_flux_and_ros_callbacks_share_the_spin_thread(node, ros_executor):
    pub = flux.Publisher(node, topic(f"mixed_{ros_executor.__name__}"), **SMALL)
    threads = {"frame": set(), "msg": set()}
    sub = flux.Subscription(
        node,
        pub.topic_name,
        callback=lambda v: threads["frame"].add(threading.get_ident()),
    )
    ros_node = Node(f"ros_side_{ros_executor.__name__}")
    ros_node.create_subscription(
        String, topic("ros"), lambda m: threads["msg"].add(threading.get_ident()), 10
    )
    ros_pub = ros_node.create_publisher(String, topic("ros"), 10)
    ex = flux.Executor(ros_executor())
    ex.add(sub)
    ex.add_node(ros_node)
    spinner = []

    def spin():
        spinner.append(threading.get_ident())
        ex.spin()

    t = threading.Thread(target=spin)
    t.start()
    end = time.monotonic() + 3
    while not (threads["frame"] and threads["msg"]) and time.monotonic() < end:
        pub.publish(b"f")
        ros_pub.publish(String())
        time.sleep(0.02)
    ex.stop()
    t.join()
    ros_node.destroy_node()
    assert threads["frame"] == set(spinner) and threads["msg"] == set(spinner)


# Not MultiThreadedExecutor: its pool runs the timer beside the frames whatever the turn does.
@pytest.mark.parametrize("ros_executor", ROS_EXECUTORS)
def test_frames_that_keep_coming_do_not_starve_ros(node, ros_executor):
    name = ros_executor.__name__
    pub = flux.Publisher(node, topic(f"busy_{name}"), **SMALL)
    # Always behind: 5 ms per frame against a frame every 2 ms.
    sub = flux.Subscription(node, pub.topic_name, callback=lambda v: time.sleep(0.005))
    ros_node = Node(f"busy_ros_{name}")
    ticks = []
    ros_node.create_timer(0.01, lambda: ticks.append(1))
    ex = flux.Executor(ros_executor())
    ex.add(sub)
    ex.add_node(ros_node)
    t = threading.Thread(target=ex.spin)
    t.start()
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        pub.publish(b"f")
        time.sleep(0.002)
    ex.stop()
    t.join()
    ros_node.destroy_node()
    # 100 timer periods; each waits out at most one frame callback.
    assert len(ticks) >= 50


def test_a_dispatch_cut_short_finishes_before_the_next_frame(node):
    pub = flux.Publisher(node, topic("cut"), **SMALL)
    got = [[], [], []]
    # 0.8 ms each: the 1 ms turn ends after two of the three, and the third must not wait for
    # the next frame, which would replace its own.
    subs = [
        flux.Subscription(
            node, pub.topic_name, callback=lambda v, g=g: (time.sleep(0.0008), g.append(1))
        )
        for g in got
    ]
    ros_node = Node("cut_ros")
    ex = flux.Executor()
    for sub in subs:
        ex.add(sub)
    ex.add_node(ros_node)
    t = threading.Thread(target=ex.spin)
    t.start()
    while not all(s.attached for s in subs):
        pub.publish(b"f")
        time.sleep(0.01)
    time.sleep(0.05)
    for g in got:
        g.clear()
    for _ in range(30):
        pub.publish(b"f")
        time.sleep(0.02)
    time.sleep(0.05)
    ex.stop()
    t.join()
    ros_node.destroy_node()
    assert [len(g) for g in got] == [30, 30, 30]


def test_a_callback_error_leaves_spin_and_the_next_spin_delivers(node):
    pub = flux.Publisher(node, topic("raise"), **SMALL)
    seen = []

    def cb(v):
        seen.append(bytes(v))
        if len(seen) == 1:
            raise KeyError("first")

    ex = flux.Executor()
    ex.add(flux.Subscription(node, pub.topic_name, callback=cb))
    pub.publish(b"a")
    with pytest.raises(KeyError):
        ex.spin()
    pub.publish(b"b")
    assert spin_until(ex, lambda: seen == [b"a", b"b"])


def test_a_stop_before_spin_is_held(node):
    ex = flux.Executor()
    ex.stop()
    ex.spin()

    pub = flux.Publisher(node, topic("held"), **SMALL)
    got = []
    sub = flux.Subscription(
        node, pub.topic_name, callback=got.append, qos=flux.QoS(depth=4, max_borrow=4)
    )
    ros = flux.Executor()
    ros.add(sub)
    ros_node = Node("held_ros")
    ros.add_node(ros_node)
    assert spin_until(ros, lambda: sub.attached)
    for _ in range(3):
        pub.publish(b"f")
    ros.stop()
    ros.spin()
    ros_node.destroy_node()
    assert got == []


def test_misuse_is_refused(node):
    ex = flux.Executor()
    with pytest.raises(ValueError):
        ex.add(flux.Subscription(node, topic("nocb")))
    ex.add(flux.Subscription(node, topic("cb"), callback=lambda v: ex.stop()))
    assert len(ex) == 1
    t = threading.Thread(target=ex.spin)
    t.start()
    time.sleep(0.05)
    with pytest.raises(RuntimeError):
        ex.add(flux.Subscription(node, topic("late"), callback=print))
    ex.stop()
    t.join()


def test_an_executor_its_callback_closes_over_is_collected(node):
    def cycle():
        ex = flux.Executor()
        ex.add(flux.Subscription(node, topic("cycle"), callback=lambda v: ex.stop()))
        return weakref.ref(ex)

    dead = cycle()
    gc.collect()
    assert dead() is None


def test_a_flux_only_spin_ends_when_its_context_shuts_down():
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    other = Node("flux_only_ctx", context=ctx)
    ex = flux.Executor()
    ex.add(flux.Subscription(other, topic("ctx"), callback=print))
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    time.sleep(0.05)
    rclpy.shutdown(context=ctx)
    t.join(timeout=3)
    assert not t.is_alive()


SPIN = """
import sys
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import flux
rclpy.init()
node = rclpy.create_node("flux_spin_child")
sub = flux.Subscription(node, sys.argv[2], callback=print)
if sys.argv[1] == "partitioned":
    ex = flux.PartitionedExecutor()
    ex.add(sub, MutuallyExclusiveCallbackGroup())
else:
    ex = flux.Executor()
    ex.add(sub)
"""


@pytest.mark.parametrize("kind", ["executor", "partitioned"])
def test_sigterm_ends_a_flux_only_spin(kind):
    # rclpy's SIGTERM handler shuts the context down without running its on_shutdown callbacks.
    code = SPIN + 'print("ready", flush=True)\nex.spin()\nprint("returned", flush=True)\n'
    p = subprocess.Popen(
        [sys.executable, "-c", code, kind, topic(f"term_{kind}")],
        stdout=subprocess.PIPE,
        text=True,
    )
    assert p.stdout.readline() == "ready\n"
    time.sleep(0.2)
    p.send_signal(signal.SIGTERM)
    try:
        out, _ = p.communicate(timeout=5)
    finally:
        p.kill()
    assert p.returncode == 0 and out == "returned\n"


SERVE = """
import sys, threading, time
from flux import _flux
sp = _flux.signpost_name(sys.argv[1], _flux.NO_SCHEMA)
pub = _flux._Publisher(sp, sys.argv[1], "n", _flux.NO_SCHEMA, 4096, 4, _flux.MemoryPolicy())
sub = _flux._Subscription(sp, sys.argv[1], "n", _flux.NO_SCHEMA, _flux.QoS(), _flux.MemoryPolicy())
ex = _flux._Executor()
ex.add(sub, print, 0)
threading.Thread(target=ex.serve, daemon=True).start()
time.sleep(0.3)
"""


def test_a_daemon_thread_in_serve_outlives_interpreter_exit():
    # Preloaded, an exit handler sleeps once Python has finalized, and the serving thread's next
    # tick tries to take the GIL meanwhile. A tick that lands during finalization is safe either
    # way, hence several runs.
    env = dict(os.environ, LD_PRELOAD=os.environ["FLUX_EXIT_DELAY"])
    runs = [
        subprocess.Popen([sys.executable, "-c", SERVE, topic(f"exit{i}")], env=env)
        for i in range(8)
    ]
    assert [p.wait(timeout=20) for p in runs] == [0] * 8


@pytest.mark.parametrize("ros_executor", ROS_EXECUTORS)
def test_frames_reach_a_spin_whose_ros_side_is_busy(node, ros_executor):
    name = topic(f"busyros_{ros_executor.__name__}")
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
    ros_node = Node(f"busy_side_{ros_executor.__name__}")

    def busy():
        end = time.perf_counter() + 0.001
        while time.perf_counter() < end:
            pass

    # Always ready, so the spin thread holds the GIL nearly all the time.
    ros_node.create_timer(0.0001, busy)
    ex = flux.Executor(ros_executor())
    ex.add(sub)
    ex.add_node(ros_node)
    t = threading.Thread(target=ex.spin)
    t.start()
    end = time.monotonic() + 3
    while not sub.attached and time.monotonic() < end:
        time.sleep(0.01)
    p.stdin.write("go\n")
    p.stdin.flush()
    p.wait(timeout=20)
    time.sleep(0.1)
    ex.stop()
    t.join()
    ros_node.destroy_node()
    assert len(got) >= 95


def test_a_ros_callback_error_leaves_no_flux_pass_behind(node):
    pub = flux.Publisher(node, topic("stale"), **SMALL)
    ros_node = Node("stale_ros")
    rex = SingleThreadedExecutor()
    ex = flux.Executor(rex)
    outside = []
    ex.add(
        flux.Subscription(node, pub.topic_name, callback=lambda v: outside.append(not ex._spinning))
    )

    def fail():
        pub.publish(b"f")
        # Time for the frame's pass to be queued in rclpy as a task.
        time.sleep(0.05)
        raise KeyError("ros")

    tmr = ros_node.create_timer(0.05, fail)
    ex.add_node(ros_node)
    with pytest.raises(KeyError):
        ex.spin()
    ros_node.destroy_timer(tmr)
    for _ in range(3):
        rex.spin_once(timeout_sec=0.02)
    ros_node.destroy_node()
    assert True not in outside


def test_a_node_already_in_the_given_rclpy_executor_is_spun(node):
    ros_node = Node("prefilled")
    ticks = []
    ros_node.create_timer(0.01, lambda: ticks.append(1))
    rex = SingleThreadedExecutor()
    rex.add_node(ros_node)
    ex = flux.Executor(rex)
    ex.add(flux.Subscription(node, topic("prefilled"), callback=print))
    assert spin_until(ex, lambda: len(ticks) >= 3)
    ros_node.destroy_node()


def test_stop_does_not_wait_for_the_rclpy_task_lock(node):
    ex = flux.Executor()
    ros_node = Node("task_lock")
    ex.add_node(ros_node)
    stopped = threading.Event()
    # A signal handler runs on the thread that may be holding this lock.
    with ex._ros._tasks_lock:
        threading.Thread(target=lambda: (ex.stop(), stopped.set()), daemon=True).start()
        assert stopped.wait(1)
    ros_node.destroy_node()


def test_two_threads_cannot_spin_at_once(node):
    class SlowCheck(flux.Executor):
        @property
        def _spinning(self):
            spinning = self.__dict__["spinning"]
            time.sleep(0.05)
            return spinning

        @_spinning.setter
        def _spinning(self, v):
            self.__dict__["spinning"] = v

    ex = SlowCheck()
    ex.add(flux.Subscription(node, topic("twice"), callback=print))
    errors = []

    def run():
        try:
            ex.spin()
        except RuntimeError as e:
            errors.append(e)

    threads = [threading.Thread(target=run, daemon=True) for _ in range(2)]
    for t in threads:
        t.start()
    time.sleep(0.3)
    ex.stop()
    for t in threads:
        t.join(timeout=2)
    assert len(errors) == 1 and not any(t.is_alive() for t in threads)


def test_a_shutdown_racing_the_wait_set_ends_spin_quietly():
    class ShutDownInWait(SingleThreadedExecutor):
        # rclpy raises RCLError, not ExternalShutdownException, when a signal shuts the context
        # down while it builds a wait set.
        def spin_once(self, timeout_sec=None):
            rclpy.shutdown(context=self.context)
            raise _rclpy.RCLError("the given context is not valid")

    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    other = Node("racing_shutdown", context=ctx)
    ex = flux.Executor(ShutDownInWait(context=ctx))
    ex.add_node(other)
    ex.spin()


def test_context_shutdown_ends_spin(node):
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    other = Node("in_ctx", context=ctx)
    ex = flux.Executor(SingleThreadedExecutor(context=ctx))
    ex.add_node(other)
    t = threading.Thread(target=ex.spin)
    t.start()
    time.sleep(0.05)
    rclpy.shutdown(context=ctx)
    t.join(timeout=3)
    assert not t.is_alive()


def test_a_view_kept_by_a_callback_stays_readable(node):
    pub = flux.Publisher(node, topic("keep"), **SMALL)
    kept = []
    ex = flux.Executor()
    ex.add(
        flux.Subscription(
            node, pub.topic_name, callback=kept.append, qos=flux.QoS(depth=4, max_borrow=3)
        )
    )
    for i in range(3):
        pub.publish(np.full(4, i, np.int32))
        assert spin_until(ex, lambda i=i: len(kept) == i + 1)
    # Three of the four slots are kept, so these all go through the fourth.
    for i in range(8):
        assert pub.publish(np.full(4, 100 + i, np.int32))
    assert [int(v[0]) for v in kept] == [0, 1, 2]
