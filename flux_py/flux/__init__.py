"""flux: same-host zero-copy shared-memory transport between C++ and Python nodes."""

import sys

from ._flux import (
    NO_SCHEMA,
    ChannelStats,
    Device,
    Durability,
    Endpoint,
    Executor,
    FenceWait,
    Frame,
    Loan,
    MemoryPolicy,
    Published,
    Publisher,
    QoS,
    Refused,
    Reliability,
    SegmentMismatch,
    Subscription,
    Topic,
    TransientLocal,
    Volatile,
    enumerate_topics,
    faulted,
    flatten_key,
    process_domain,
    read_channel_stats,
    resolve_domain,
    rt,
)

# `rt` is an extension submodule, so it arrives as an attribute of `_flux` and not as a name the
# import machinery knows. Without this line `flux.rt.apply(...)` works and `import flux.rt` does
# not, which is a difference no reader expects to find.
sys.modules.setdefault(__name__ + ".rt", rt)

__all__ = [
    "NO_SCHEMA",
    "ChannelStats",
    "Device",
    "Durability",
    "Endpoint",
    "Executor",
    "FenceWait",
    "Frame",
    "Loan",
    "MemoryPolicy",
    "Published",
    "Publisher",
    "QoS",
    "Refused",
    "Reliability",
    "SegmentMismatch",
    "Subscription",
    "Topic",
    "TransientLocal",
    "Volatile",
    "enumerate_topics",
    "faulted",
    "flatten_key",
    "process_domain",
    "read_channel_stats",
    "resolve_domain",
    "rt",
]
