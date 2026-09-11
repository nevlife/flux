"""Which generated adapter belongs to a channel.

A channel carries only its schema fingerprint. The type name lives in the adapter module flux_gen
emitted (`TYPE_NAME__`, `FINGERPRINT__`), so the map is built by importing every `<pkg>_flux`
package on the Python path. Channels whose adapter is not installed cannot be bridged; they are
reported, not guessed at.
"""

import importlib
import os
import sys

_SUFFIX = "_flux"


class Adapter:
    """One .msg type as flux_gen installed it: the frame adapter and its ROS message bridge."""

    def __init__(self, module_name, type_name, fingerprint):
        self.module_name = module_name
        self.type_name = type_name
        self.fingerprint = fingerprint
        self._module = None
        self._ros = None

    @property
    def ros_type(self):
        """`pkg/msg/Name`, the spelling ros2cli and rosidl_runtime_py use."""
        pkg, name = self.type_name.split("/")
        return f"{pkg}/msg/{name}"

    @property
    def view(self):
        if self._module is None:
            self._module = importlib.import_module(self.module_name)
        return self._module.View

    def _ros_module(self):
        if self._ros is None:
            self._ros = importlib.import_module(self.module_name + "_ros")
        return self._ros

    @property
    def message(self):
        return self._ros_module().MESSAGE

    @property
    def frame_to_msg(self):
        return self._ros_module().frame_to_msg


def _candidate_modules(paths):
    """`pkg.module` names of every adapter on `paths`, the `_ros` bridges excluded."""
    seen = set()
    for entry in paths:
        try:
            names = os.listdir(entry)
        except OSError:
            continue
        for pkg in sorted(names):
            if not pkg.endswith(_SUFFIX) or pkg in seen:
                continue
            pkg_dir = os.path.join(entry, pkg)
            if not os.path.isdir(pkg_dir):
                continue
            seen.add(pkg)
            for file in sorted(os.listdir(pkg_dir)):
                stem, ext = os.path.splitext(file)
                if ext != ".py" or stem == "__init__" or stem.endswith("_ros"):
                    continue
                yield f"{pkg}.{stem}"


def discover(paths=None):
    """fingerprint -> Adapter for every adapter importable from `paths` (default: sys.path)."""
    found = {}
    for name in _candidate_modules(sys.path if paths is None else paths):
        try:
            module = importlib.import_module(name)
        except Exception:
            continue
        fingerprint = getattr(module, "FINGERPRINT__", None)
        type_name = getattr(module, "TYPE_NAME__", None)
        if not isinstance(fingerprint, int) or not isinstance(type_name, str):
            continue
        found.setdefault(fingerprint, Adapter(name, type_name, fingerprint))
    return found
