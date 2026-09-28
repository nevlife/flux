"""C++ and Python name the same things, or the difference is written down with its reason.

The C++ side is read off the headers (scripts/cxx_surface.py), the Python side off the live
module. Each C++ type is paired with its Python class and their public member names compared,
after the spellings the two ROS client libraries already differ by:

- enum values: `FenceFailed` is `FENCE_FAILED` (contracts S-005)
- a getter: `get_topic_name()` is `topic_name`, as rclcpp and rclpy spell it
- a setter: `set_pass_budget(n)` is an assignable `pass_budget` property

A difference not in the tables below fails, and so does a table entry the code no longer has, so
the tables cannot drift from what they describe. Names only: arguments, defaults and exception
types are not compared.
"""

import ast
import importlib
import inspect
import pathlib
import re
import sys

import pytest

import flux
import flux.ros

pytest.importorskip("rclpy")
pytest.importorskip("message_filters")
import flux.ros.message_filters  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from check_api_surface import SURFACE  # noqa: E402
from cxx_surface import public_names  # noqa: E402

MODULES = (flux, flux.ros, flux.ros.message_filters)

ROS = "flux_cpp/include/flux/ros/"
CORE = "flux_core/include/flux/"

# (label, header, C++ name, Python path). The C++ name is a type, a namespace-scope function or
# constant, or `Type::member` for a member that Python spells at module scope.
PAIRS = [
    ("flux::ros::Publisher", ROS + "publisher.hpp", "Publisher", "flux.ros.Publisher"),
    (
        "flux::ros::create_publisher",
        ROS + "publisher.hpp",
        "create_publisher",
        "flux.ros.create_publisher",
    ),
    ("flux::ros::Subscription", ROS + "subscription.hpp", "Subscription", "flux.ros.Subscription"),
    (
        "flux::ros::create_subscription",
        ROS + "subscription.hpp",
        "create_subscription",
        "flux.ros.create_subscription",
    ),
    ("flux::ros::Executor", ROS + "executor.hpp", "Executor", "flux.ros.Executor"),
    (
        "flux::ros::PartitionedExecutor",
        ROS + "partitioned_executor.hpp",
        "PartitionedExecutor",
        "flux.ros.PartitionedExecutor",
    ),
    (
        "message_filters::Subscriber",
        ROS + "message_filters/subscriber.hpp",
        "Subscriber",
        "flux.ros.message_filters.Subscriber",
    ),
    (
        "message_filters::StampedFrame",
        ROS + "message_filters/subscriber.hpp",
        "StampedFrame",
        "flux.ros.message_filters.StampedFrame",
    ),
    ("flux::Executor", CORE + "executor.hpp", "Executor", "flux.Executor"),
    ("flux::MemoryPolicy", CORE + "memory.hpp", "MemoryPolicy", "flux.MemoryPolicy"),
    ("flux::QoS", CORE + "qos.hpp", "QoS", "flux.QoS"),
    ("flux::Durability", CORE + "qos.hpp", "Durability", "flux.Durability"),
    (
        "flux::Durability::TransientLocal",
        CORE + "qos.hpp",
        "Durability::TransientLocal",
        "flux.TransientLocal",
    ),
    ("flux::Durability::Volatile", CORE + "qos.hpp", "Durability::Volatile", "flux.Volatile"),
    ("flux::WriteSlot", CORE + "channel.hpp", "WriteSlot", "flux.Loan"),
    ("flux::FrameView", CORE + "channel.hpp", "FrameView", "flux.Frame"),
    ("flux::Channel::Refused", CORE + "channel.hpp", "Refused", "flux.Refused"),
    ("flux::Channel::FenceWait", CORE + "channel.hpp", "FenceWait", "flux.FenceWait"),
    ("flux::Published", CORE + "channel.hpp", "Published", "flux.Published"),
    ("flux::faulted", CORE + "channel.hpp", "faulted", "flux.faulted"),
    ("flux::Device", CORE + "gpu_platform.hpp", "Device", "flux.Device"),
    ("flux::kNoSchema", CORE + "segment_layout.hpp", "kNoSchema", "flux.NO_SCHEMA"),
    ("flux::OwnerId", CORE + "owner.hpp", "OwnerId", "flux.OwnerId"),
    ("flux::SegmentMismatch", CORE + "discovery.hpp", "SegmentMismatch", "flux.SegmentMismatch"),
    ("flux::Topic", CORE + "discovery.hpp", "Topic", "flux.Topic"),
    ("flux::Endpoint", CORE + "discovery.hpp", "Endpoint", "flux.Endpoint"),
    ("flux::ChannelStats", CORE + "discovery.hpp", "ChannelStats", "flux.ChannelStats"),
    ("flux::enumerate_topics", CORE + "discovery.hpp", "enumerate_topics", "flux.enumerate_topics"),
    (
        "flux::read_channel_stats",
        CORE + "discovery.hpp",
        "read_channel_stats",
        "flux.read_channel_stats",
    ),
    ("flux::signpost_name", CORE + "discovery.hpp", "signpost_name", "flux.signpost_name"),
    ("flux::flatten_key", CORE + "discovery.hpp", "flatten_key", "flux.flatten_key"),
    ("flux::canonical_domain", CORE + "discovery.hpp", "canonical_domain", "flux.canonical_domain"),
    ("flux::resolve_domain", CORE + "discovery.hpp", "resolve_domain", "flux.resolve_domain"),
    ("flux::process_domain", CORE + "discovery.hpp", "process_domain", "flux.process_domain"),
    ("flux::version", CORE + "version.hpp.in", "version", "flux.__version__"),
]

S010 = "S-010: the direct-loop calls flux.ros.Executor does not have"

# Member names one side has and the other does not, per pair label.
CPP_ONLY = {
    "flux::ros::Publisher": {
        "kDefaultSlotSize": "Python states the default as the constructor's keyword default",
        "kDefaultSlotCount": "Python states the default as the constructor's keyword default",
    },
    "flux::ros::Subscription": {
        "Callback": "the std::function type of the callback; Python takes any callable",
    },
    "flux::ros::Executor": {
        "dispatch": S010,
        "has_more": S010,
        "wait_for_work": S010,
        "pump_ros": S010,
        "take_ros_ready": S010,
        "pass_budget": S010,
        "set_pass_budget": S010,
        "ros_budget": S010,
        "set_ros_budget": S010,
        "add_ros_callback_group": "rclpy executors take nodes, not callback groups",
    },
    "message_filters::Subscriber": {
        "Message": "the message type parameter; Python takes the adapter module instead (adapter)",
        "subscribe": "S-011",
        "subscribed": "S-011",
        "unsubscribe": "S-011",
    },
    "flux::QoS": {
        "keep_last": "S-005: C++ builds a QoS with setters, Python with keywords",
        "transient_local": "S-005: C++ builds a QoS with setters, Python with keywords",
        "durability_volatile": "S-005: C++ builds a QoS with setters, Python with keywords",
        "validate": "the Python constructor validates, so no unvalidated QoS exists to check",
    },
    "flux::Executor": {
        "waker": "for a C++ embedder merging its own readiness through spin_once(timeout, woken); "
        "Python's spin_once has no woken",
        "clear": "the teardown hook flux_py's cyclic-GC clear calls, not a call a user makes",
    },
    "flux::OwnerId": {
        "pad": "alignment padding, not a field",
        "valid": "every OwnerId Python receives names a live process",
    },
    "flux::SegmentMismatch": {
        "runtime_error": "the inherited std::runtime_error constructor, read as a member",
    },
    "flux::WriteSlot": {
        "data": "a raw pointer; Python writes through the numpy view (array)",
        "device_ptr": "Python hands the device address over as __cuda_array_interface__ and __dlpack__",
    },
    "flux::FrameView": {
        "data": "a host frame is a numpy array in Python; flux.Frame is the S-012 exception",
        "size": "flux.Frame states it as nbytes",
        "meta": "flux.Frame states FrameMeta's fields as its own attributes (dtype, shape, nbytes)",
        "device_ptr": "Python hands the device address over as __cuda_array_interface__ and __dlpack__",
        "release": "leaving the `with` block releases a flux.Frame (S-012)",
        "valid": "flux.Frame states the negation (released)",
    },
}

PY_ONLY = {
    "flux::ros::Executor": {
        "is_spinning": "C++ inherits rclcpp::Executor::is_spinning()",
        "shutdown": "S-006",
    },
    "message_filters::Subscriber": {
        "adapter": "the adapter module the constructor takes; C++ has it as the Message parameter",
        "topic": "the topic as given; getTopic() returns the same on both sides",
    },
    "flux::WriteSlot": {
        "array": "the numpy view over the slot, where C++ writes through data()",
        "bits": "S-012",
    },
    "flux::FrameView": {
        "bits": "S-012",
        "dtype": "one of FrameMeta's fields (C++ meta())",
        "shape": "one of FrameMeta's fields (C++ meta())",
        "nbytes": "C++ size()",
        "released": "the negation of C++ valid()",
    },
}

# The same thing under another name, per pair label: C++ member -> (Python member, reason).
RENAMED = {
    "flux::ros::Subscription": {
        "has_callback": ("callback", "Python exposes the callable itself, None when unset"),
    },
    "message_filters::Subscriber": {
        "getSubscriber": ("sub", "B-27: each side follows its upstream message_filters"),
    },
}

# A C++ surface type with no Python class, and why.
CPP_TYPES_ONLY = {
    "FrameMeta": "a Python frame is a numpy array, which carries its own dtype and shape",
    "DType": "Python names element types as numpy does",
}

# A Python export with no C++ counterpart in PAIRS, and why.
PY_EXPORTS_ONLY = {
    "flux.Publisher": "the engine layer: C++ has one flux::Channel for both roles (core_api 7). "
    "flux.ros.Publisher subclasses it and is paired",
    "flux.Subscription": "the engine layer: C++ has one flux::Channel for both roles (core_api 7). "
    "flux.ros.Subscription subclasses it and is paired",
}


def _resolve(path):
    module, _, name = path.rpartition(".")
    return getattr(importlib.import_module(module), name)


def _upper_snake(name):
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name).upper()


def _cpp_names(header):
    return public_names((ROOT / header).read_text())


def _cpp_members(header, cpp_type):
    return {n for n, owners in _cpp_names(header).items() if cpp_type in owners}


def _init_attributes(cls):
    """Public `self.x = ...` targets of a pure-Python __init__: attributes dir() does not show."""
    try:
        tree = ast.parse(inspect.getsource(cls))
    except (OSError, TypeError):
        return set()
    found = set()
    for fn in ast.walk(tree):
        if not (isinstance(fn, ast.FunctionDef) and fn.name == "__init__"):
            continue
        for node in ast.walk(fn):
            if (
                isinstance(node, ast.Attribute)
                and isinstance(node.ctx, ast.Store)
                and isinstance(node.value, ast.Name)
                and node.value.id == "self"
                and not node.attr.startswith("_")
            ):
                found.add(node.attr)
    return found


def _py_members(cls):
    """Public names flux itself defines on `cls`: not ones inherited from Exception or rclpy."""
    found = set()
    for klass in cls.__mro__:
        if not (klass.__module__ or "").startswith("flux"):
            continue
        found |= {n for n in vars(klass) if not n.startswith("_")}
        found |= _init_attributes(klass)
    return found


def _assignable(cls, name):
    attr = inspect.getattr_static(cls, name, None)
    return isinstance(attr, property) and attr.fset is not None


def _unexplained(label, header, cpp_type, py_path):
    cls = _resolve(py_path)
    cpp = _cpp_members(header, cpp_type)
    py = _py_members(cls)
    renamed = RENAMED.get(label, {})
    moved = {n.split("::", 1)[1] for _, h, n, _ in PAIRS if n.startswith(cpp_type + "::")}
    enum = inspect.isclass(cls) and issubclass(cls, int) or hasattr(cls, "__members__")

    matched_py = set()
    cpp_only = set()
    for name in cpp - moved:
        if name in renamed:
            matched_py.add(renamed[name][0])
            continue
        if enum and _upper_snake(name) in py:
            matched_py.add(_upper_snake(name))
        elif name in py:
            matched_py.add(name)
        elif name.startswith("get_") and name[4:] in py:
            matched_py.add(name[4:])
        elif name.startswith("set_") and name[4:] in py and _assignable(cls, name[4:]):
            matched_py.add(name[4:])
        else:
            cpp_only.add(name)
    py_only = py - matched_py - {n for n in py if n in cpp}

    problems = []
    listed_cpp = CPP_ONLY.get(label, {})
    listed_py = PY_ONLY.get(label, {})
    problems += [
        f"C++ {label}::{n} has no Python counterpart" for n in sorted(cpp_only - set(listed_cpp))
    ]
    problems += [
        f"Python {py_path}.{n} has no C++ counterpart" for n in sorted(py_only - set(listed_py))
    ]
    problems += [
        f"CPP_ONLY[{label!r}][{n!r}] no longer differs" for n in sorted(set(listed_cpp) - cpp_only)
    ]
    problems += [
        f"PY_ONLY[{label!r}][{n!r}] no longer differs" for n in sorted(set(listed_py) - py_only)
    ]
    for cpp_name, (py_name, _) in renamed.items():
        if cpp_name not in cpp or py_name not in py:
            problems.append(f"RENAMED[{label!r}][{cpp_name!r}] no longer holds")
    return problems


TYPE_PAIRS = [p for p in PAIRS if inspect.isclass(_resolve(p[3]))]


@pytest.mark.parametrize(
    "label, header, cpp_type, py_path", TYPE_PAIRS, ids=[p[0] for p in TYPE_PAIRS]
)
def test_a_pair_names_the_same_members(label, header, cpp_type, py_path):
    assert _unexplained(label, header, cpp_type, py_path) == []


def test_every_paired_cpp_name_exists():
    missing = []
    for label, header, cpp_name, _ in PAIRS:
        owner, _, member = cpp_name.rpartition("::")
        names = _cpp_names(header)
        if member not in names or (owner and owner not in names[member]):
            missing.append(label)
    assert missing == []


def test_every_python_export_is_paired_or_explained():
    covered = {id(_resolve(p)) for *_, p in PAIRS} | {id(_resolve(p)) for p in PY_EXPORTS_ONLY}
    unpaired = sorted(
        f"{m.__name__}.{n}" for m in MODULES for n in m.__all__ if id(getattr(m, n)) not in covered
    )
    assert unpaired == []


def test_every_cpp_surface_type_is_paired_or_explained():
    paired = {(h, n) for _, h, n, _ in PAIRS}
    unpaired = []
    for header, only_type in SURFACE:
        names = _cpp_names(header)
        wanted = {only_type} if only_type else {n for n, o in names.items() if None in o}
        for name in sorted(wanted):
            if (header, name) not in paired and name not in CPP_TYPES_ONLY:
                unpaired.append(f"{header}: {name}")
    assert unpaired == []


def test_the_tables_name_only_pairs_that_exist():
    labels = {p[0] for p in PAIRS}
    stale = sorted((set(CPP_ONLY) | set(PY_ONLY) | set(RENAMED)) - labels)
    assert stale == []
