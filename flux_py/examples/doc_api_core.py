# Companion to docs/en/core_api.en.md -- flux without ROS. Nothing here imports flux.ros, which is
# what keeps the document's claim honest. Every region between [doc:id] and [doc:/id] is the
# code the fence tagged `doc:id` in that document shows. Nothing here runs.

import flux

FP = 0


def _sink(*_args):
    pass


def doc_core_publisher(arr):
    # [doc:core_py_publisher]
    pub = flux.Publisher("/img", fingerprint=FP, slot_size=16 << 20, slot_count=16)
    pub.publish(arr)
    # [doc:/core_py_publisher]
    _sink(pub.dropped, pub.segment_name)


def doc_core_subscription():
    # [doc:core_py_subscription]
    sub = flux.Subscription("/img", fingerprint=FP, qos=flux.QoS())
    v = sub.take()
    if v is not None:
        _sink(v.shape)
    # [doc:/core_py_subscription]
    _sink(sub.lost)


def doc_core_memory_policy():
    # [doc:core_py_memory_policy]
    mem = flux.MemoryPolicy(precommit=True, lock=True)
    pub = flux.Publisher("/img", fingerprint=FP, slot_size=4096, slot_count=8, memory=mem)
    sub = flux.Subscription("/img", fingerprint=FP, memory=mem)
    # [doc:/core_py_memory_policy]
    _sink(pub.segment_name, sub.segment_name)


def doc_core_executor(sub, callback):
    # [doc:py_core_executor]
    ex = flux.Executor(max_channels=32, poll_tick_ns=2_000_000)
    ex.add(sub, callback, priority=0)
    ex.spin_once(timeout_ns=-1)
    ex.spin(tick_ns=100_000_000)
    ex.stop()
    ex.interrupt()
    merged = ex.uses_io_uring
    busy = ex.is_spinning
    # [doc:/py_core_executor]
    _sink(merged)
    _sink(busy)

    # [doc:py_split_wait]
    ex.wait_for_work(timeout_ns=-1)
    delivered = ex.dispatch()
    # [doc:/py_split_wait]
    _sink(delivered)


def doc_enumerate():
    # [doc:py_enumerate]
    for topic in flux.enumerate_topics():
        name = topic.key if topic.key_exact else topic.signpost
        for ep in topic.endpoints:
            role = "pub" if ep.publisher else "sub"
            _sink(name, topic.domain, topic.fingerprint, role, ep.pid, ep.starttime, ep.label)
    # [doc:/py_enumerate]


def doc_channel_stats(signpost):
    # [doc:py_channel_stats]
    domain = flux.process_domain()

    stats = flux.read_channel_stats(signpost)
    if stats.live:
        _sink(stats.slot_count, stats.slot_size, stats.storage_kind, stats.fingerprint)
        _sink(stats.publish_seq, stats.epoch, stats.waiters)
    # [doc:/py_channel_stats]
    _sink(domain)
