# Companion to docs/en/api.en.md section 4. Every region between [doc:id] and [doc:/id] is the code
# the fence tagged `doc:id` in that document shows. Nothing here runs: the functions are never
# called and take everything they need as parameters.
#
# test/test_doc_examples.py compiles this file and resolves every flux attribute path it uses
# against the imported package, so a renamed binding fails a test instead of silently outdating
# the document. scripts/check_doc_examples.py compares the two sides, ignoring comments so the
# documents can keep their own. Change the code here first, then the fence.

# [doc:py_imports]
import flux
import flux.ros
# [doc:/py_imports]

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

FP = 0


def _sink(*_args):
    pass


def doc_publisher(node):
    # [doc:py_publisher]
    pub = flux.ros.Publisher(node, "img", fingerprint=FP, slot_size=16 << 20, slot_count=16)

    dropped = pub.dropped
    slot_size = pub.slot_size
    segment_name = pub.segment_name
    # [doc:/py_publisher]
    _sink(dropped, slot_size, segment_name)


def doc_subscription(node):
    # [doc:py_subscription]
    sub = flux.ros.Subscription(node, "img", callback=_sink, fingerprint=FP, qos=flux.QoS())

    newest = sub.peek()
    nxt = sub.take()
    blocking = sub.take_blocking(timeout_ns=-1)
    lost = sub.lost
    qos = sub.qos
    segment_name = sub.segment_name
    domain = sub.domain

    if nxt is None and not sub.can_borrow:
        held = sub.refused.max_borrow
    if nxt is None and not sub.attached:
        where = sub.domain
    # [doc:/py_subscription]
    _sink(newest, nxt, blocking, lost, qos, segment_name, domain, held, where)


def doc_qos(n):
    # [doc:py_qos]
    qos = flux.QoS(
        depth=1, durability=flux.Volatile(), max_borrow=2, reliability=flux.Reliability.BEST_EFFORT
    )

    volatile = flux.Volatile()
    transient = flux.TransientLocal(n)

    kind = qos.durability
    live_only = kind.is_volatile
    replay = kind.replay
    # [doc:/py_qos]
    _sink(qos, volatile, transient, live_only, replay)


def doc_ros_executor(node, sub):
    # [doc:py_ros_executor]
    ex = flux.ros.Executor()
    ex.add_flux(sub)                  # the subscription carries the callback
    ex.add_ros_node(node)             # the whole node, same as add_ros_node in C++
    ex.spin()                         # runs until stop(); tick_ns defaults to 100 ms
    ex.spin_once(timeout_ns=100_000_000)
    merged = ex.uses_io_uring
    ex.interrupt()                    # wakes the wait without ending the loop
    ex.stop()
    ex.close()                        # after the last stop(), before destroying the node

    resolved = flux.ros.resolve(node, "img")
    # [doc:/py_ros_executor]
    _sink(resolved, merged)



def doc_partitioned(node, sub_a, sub_b):
    # [doc:py_partitioned]
    ex = flux.ros.PartitionedExecutor()
    ex.add_ros_node(node)
    ex.add_flux(sub_a, MutuallyExclusiveCallbackGroup())
    ex.add_flux(sub_b, MutuallyExclusiveCallbackGroup(), priority=10)
    ex.spin(tick_ns=100_000_000)
    ex.stop()
    ex.close()
    # [doc:/py_partitioned]


def doc_rt(node, sub, group):
    # [doc:py_rt]
    ex = flux.ros.PartitionedExecutor()
    ex.add_flux(sub, group)
    ex.add_ros_node(node)
    ex.set_thread_scheduling(group, cpus=[4, 5])
    ex.set_thread_scheduling(node, policy=flux.rt.Policy.Fifo, priority=20)

    report = flux.rt.apply(cpus=[4])          # the CALLING thread, not a child
    state = flux.rt.current()                 # read back from the kernel
    klass, prio, where = state.policy, state.priority, state.cpus
    tid = flux.rt.this_tid()                  # what top -H and /proc name it by
    # [doc:/py_rt]
    _sink(report, klass, prio, where, tid)





def doc_gpu_publisher(node, cp, render_into):
    # [doc:py_gpu_publisher]
    pub = flux.ros.Publisher(node, "img", fingerprint=FP, device="cuda")

    loan = pub.loan((480, 640, 3), dtype="uint8")
    if loan:
        render_into(cp.asarray(loan), loan.stream)
        loan.commit()
    fence_failed = pub.fence_failed
    worst_commit_ns = pub.fence_wait.commit_max_ns
    # [doc:/py_gpu_publisher]
    _sink(fence_failed, worst_commit_ns)


def doc_gpu_subscription(node, cp, use):
    # [doc:py_gpu_subscription]
    sub = flux.ros.Subscription(node, "img", fingerprint=FP, qos=flux.QoS(), device="cuda")

    frame = sub.take()
    if frame:
        with frame as v:
            use(cp.asarray(v), v.stream)

    fence_failed = sub.fence_failed
    worst_release_ns = sub.fence_wait.release_max_ns
    # [doc:/py_gpu_subscription]
    _sink(fence_failed, worst_release_ns)
