"""Page pre-commit and mlock from Python.

The residency itself is flux_core's contract and flux_core tests it against /proc/self/smaps.
What can go wrong here is the declaration not reaching the mapping at all, which looks like
nothing: the channel works, it just still faults. These read residency back for that reason.
"""

import os

import numpy as np
import pytest

import flux

FP = 0x03E30E7
SLOT = 1 << 20
SLOTS = 8
PAYLOAD_KB = (SLOT // 1024) * SLOTS


def _resident_kb(needle):
    """Summed Rss over every mapping in this process backed by a file matching `needle`.

    Summed rather than per-mapping: a subscriber's read-only payload is its own VMA and it also
    holds the signpost, whose shm name is a prefix of the segment's.
    """
    total = 0
    ours = False
    with open("/proc/self/smaps") as f:
        for line in f:
            head = line.split(" ", 1)[0]
            if "-" in head and ":" not in head and head.replace("-", "").isalnum():
                ours = needle in line
                continue
            if ours and line.startswith("Rss:"):
                total += int(line.split()[1])
    return total


def _needle(topic):
    # The shm name is the mangled signpost: '/' becomes '.' and the fingerprint is appended.
    return topic.strip("/").replace("/", ".")


def test_default_is_off():
    p = flux.MemoryPolicy()
    assert p.precommit is False
    assert p.lock is False


def test_precommit_reaches_the_mapping():
    topic = "/pytest/mem/precommit"
    pub = flux.Publisher(
        topic, fingerprint=FP, slot_size=SLOT, slot_count=SLOTS,
        memory=flux.MemoryPolicy(precommit=True),
    )
    assert _resident_kb(_needle(topic)) >= PAYLOAD_KB
    assert pub.publish(np.zeros(16, dtype=np.uint8)) == flux.Published.Ok


def test_without_it_the_segment_stays_sparse():
    topic = "/pytest/mem/sparse"
    pub = flux.Publisher(topic, fingerprint=FP, slot_size=SLOT, slot_count=SLOTS)
    assert _resident_kb(_needle(topic)) < PAYLOAD_KB // 2
    assert pub.publish(np.zeros(16, dtype=np.uint8)) == flux.Published.Ok


def test_a_subscriber_declares_its_own():
    # Per-process: the publisher's committed pages say nothing about the subscriber's mapping.
    topic = "/pytest/mem/sub"
    pub = flux.Publisher(topic, fingerprint=FP, slot_size=SLOT, slot_count=SLOTS)
    sub = flux.Subscription(topic, fingerprint=FP, memory=flux.MemoryPolicy(precommit=True))

    resident = _resident_kb(_needle(topic))
    assert resident >= PAYLOAD_KB, "the subscriber's mapping was not committed"
    assert resident < PAYLOAD_KB + PAYLOAD_KB // 2, "the publisher's mapping was committed too"
    assert pub.publish(np.full(8, 5, dtype=np.uint8)) == flux.Published.Ok
    assert int(sub.take()[0]) == 5


def test_a_refused_lock_raises_rather_than_downgrading():
    import resource

    soft, hard = resource.getrlimit(resource.RLIMIT_MEMLOCK)
    if soft == resource.RLIM_INFINITY and os.geteuid() == 0:
        pytest.skip("running as root with no memlock limit: nothing is refused")
    resource.setrlimit(resource.RLIMIT_MEMLOCK, (4096, hard))
    try:
        with pytest.raises(OSError):
            flux.Publisher(
                "/pytest/mem/refuse", fingerprint=FP, slot_size=SLOT, slot_count=SLOTS,
                memory=flux.MemoryPolicy(lock=True),
            )
    finally:
        resource.setrlimit(resource.RLIMIT_MEMLOCK, (soft, hard))
