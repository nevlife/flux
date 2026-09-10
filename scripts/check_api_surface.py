#!/usr/bin/env python3
"""Every name the C++ user surface exports must appear in one of the user documents.

The reverse direction of scripts/check_doc_examples.py. A fence that names a member that no
longer exists breaks the build, because the fences are compiled; a member added without a line in
the document breaks nothing, so nothing catches it. flux_py/test/test_doc_examples.py runs this
same check for Python by walking `__all__`; C++ has no runtime to walk, so the headers are parsed
instead.

The surface is listed here rather than derived. flux_cpp's headers are the surface whole, but
flux_core's are the engine, and only the handful of types those four headers hand out or take
belong in a user document.

Names are matched bare, not as Type::name, because that is how a document writes them --
`w.data()`, not `flux::WriteSlot::data`. A new member whose name some other type already
documents therefore passes. Widening it would mean spelling every owner in the document.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cxx_surface import public_names  # noqa: E402
from doc_symbols import documented_symbols  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
# The C++ user surface is documented across three files: api.md is the .msg path, raw_api.md the
# path without one, core_api.md flux_core with no ROS. A name documented in any of them counts.
USER_DOCS = [
    ROOT / "docs" / "en" / "api.en.md",
    ROOT / "docs" / "en" / "raw_api.en.md",
    ROOT / "docs" / "en" / "core_api.en.md",
]

# (header, type) with type None meaning the whole header. A new public class in a flux_cpp header
# is caught by the None entries without being listed here.
SURFACE = [
    ("flux_cpp/include/flux/ros/publisher.hpp", None),
    ("flux_cpp/include/flux/ros/subscription.hpp", None),
    ("flux_cpp/include/flux/ros/executor.hpp", None),
    ("flux_cpp/include/flux/ros/partitioned_executor.hpp", None),
    ("flux_cpp/include/flux/ros/rt_spec.hpp", None),
    ("flux_cpp/include/flux/ros/message_filters/subscriber.hpp", None),
    ("flux_core/include/flux/memory.hpp", None),
    ("flux_core/include/flux/qos.hpp", None),
    ("flux_core/include/flux/rt.hpp", None),
    ("flux_core/include/flux/discovery.hpp", "SegmentMismatch"),
    ("flux_core/include/flux/channel.hpp", "FrameView"),
    ("flux_core/include/flux/channel.hpp", "WriteSlot"),
    ("flux_core/include/flux/channel.hpp", "Refused"),
    ("flux_core/include/flux/channel.hpp", "FenceWait"),
    ("flux_core/include/flux/segment_layout.hpp", "FrameMeta"),
    ("flux_core/include/flux/segment_layout.hpp", "DType"),
]

# Public in C++ but not user surface, each with the reason it is not a documentation gap.
# Document the name instead of adding to this.
UNDOCUMENTED = {
    "none": "MemoryPolicy::none() is the engine's 'is this a no-op' test, not a knob a caller sets",
    "Callback": "the callback type is spelled inline in the Subscription fence",
    "attach": "driver hook: flux::ros::Executor calls it, a node never does",
    "deliver": "driver hook: flux::ros::Executor calls it, a node never does",
    "channel": "driver hook: hands the executor the engine object behind the subscription",
    "validate": "QoS::validate and rt::Options::validate run inside the constructors that take them",
    "reserved0": "FrameMeta padding, reserved for a future field",
}


def main():
    documented = set()
    for doc in USER_DOCS:
        documented |= documented_symbols(doc.read_text())
    missing = {}
    for rel, only_type in SURFACE:
        header = ROOT / rel
        for name, enclosing in public_names(header.read_text(), only_type).items():
            if name in documented or name in UNDOCUMENTED:
                continue
            owner = sorted(e for e in enclosing if e) or ["(namespace domain)"]
            missing.setdefault(f"{owner[0]}::{name}", rel)
    for dotted, rel in sorted(missing.items()):
        print(f"{rel}: {dotted} is public but absent from the user documents", file=sys.stderr)
    if missing:
        print(f"{len(missing)} undocumented name(s) on the C++ surface", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
