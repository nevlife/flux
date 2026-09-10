"""flux_gen command line: turn .msg files into C++ and Python flux adapters.

    python3 -m flux_gen --pkg my_pkg --msg msg/Cloud.msg --out-cpp include --out-py python

Nested types resolve against the message's own directory first, then against every ROS package
on AMENT_PREFIX_PATH -- a schema's fingerprint depends on what its nested types flatten to, so an
unresolvable reference is an error rather than a guess.
"""

import argparse
import os
import sys

from . import Registry, emit_cpp, emit_py, emit_ros, flatten_message, load_dir
from .dtypes import ROS_PRIMITIVE
from .layout import build_layout
from .parse import resolve


def ament_registry(into=None):
    """Every installed .msg, keyed pkg/Msg, so nested references to other packages resolve.

    Merges without displacing: a message already in `into` was named by the caller, and an
    installed copy of it -- the same package rebuilt, or an adapter generated for a package that
    ships its own .msg -- must not be the one the layout is read from.
    """
    reg = into if into is not None else Registry()
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        share = os.path.join(prefix, "share")
        if not prefix or not os.path.isdir(share):
            continue
        for pkg in sorted(os.listdir(share)):
            msg_dir = os.path.join(share, pkg, "msg")
            if os.path.isdir(msg_dir):
                load_dir(msg_dir, package=pkg, into=reg, if_absent=True)
    return reg


def sources(msg, reg, seen=None):
    """Every .msg a schema is read from, itself included.

    A build has to depend on all of them: editing a nested type changes the carrier's layout and
    its fingerprint, and a stale adapter is not a build error but a segment name that no longer
    matches its peer.
    """
    seen = set() if seen is None else seen
    if msg is None or id(msg) in seen:
        return []
    seen.add(id(msg))
    out = [msg.path] if msg.path else []
    for fld in msg.fields:
        if fld.type_name in ROS_PRIMITIVE or fld.type_name in ("string", "wstring"):
            continue
        try:
            nested = resolve(reg, fld.type_name, msg.package)
        except Exception:
            continue
        out.extend(sources(nested, reg, seen))
    return out


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return path


def generate(msg_path, pkg, out_cpp=None, out_py=None, registry=None, depfile=None):
    """Generate adapters for one .msg. Returns the paths written."""
    reg = registry if registry is not None else Registry()
    stem = os.path.basename(msg_path)[:-4]
    msg = reg.get(f"{pkg}/{stem}")
    if msg is None:
        raise SystemExit(f"flux_gen: {pkg}/{stem} not found (looked beside {msg_path})")

    layout = build_layout(flatten_message(msg, reg))
    if layout.rejected:
        raise SystemExit(f"flux_gen: {pkg}/{stem} cannot ride the flat wire -- {layout.rejected}")

    source = f"{pkg}/msg/{stem}.msg"
    snake = emit_cpp.snake(stem)
    written = []
    if out_cpp:
        written.append(_write(
            os.path.join(out_cpp, pkg, "flux", snake + ".hpp"), emit_cpp.emit(layout, source)))
        written.append(_write(
            os.path.join(out_cpp, pkg, "flux", snake + "_ros.hpp"),
            emit_ros.emit_cpp(layout, source)))
    if out_py:
        written.append(_write(
            os.path.join(out_py, pkg + "_flux", snake + ".py"), emit_py.emit(layout, source)))
        written.append(_write(
            os.path.join(out_py, pkg + "_flux", snake + "_ros.py"),
            emit_ros.emit_py(layout, source)))
    if depfile:
        deps = " ".join(sorted(set(sources(msg, reg))))
        _write(depfile, "".join(f"{out}: {deps}\n" for out in written))
    return written


def main(argv=None):
    ap = argparse.ArgumentParser(prog="flux_gen", description=__doc__.splitlines()[0])
    ap.add_argument("--pkg", required=True, help="ROS package owning the messages")
    ap.add_argument("--msg", required=True, action="append", help=".msg file (repeatable)")
    ap.add_argument("--out-cpp", help="root for include/<pkg>/flux/<name>.hpp")
    ap.add_argument("--out-py", help="root for <pkg>_flux/<name>.py")
    ap.add_argument("--depfile", help="write a make depfile listing every .msg read")
    ap.add_argument("--no-ament", action="store_true",
                    help="do not resolve nested types against installed ROS packages")
    args = ap.parse_args(argv)

    if not args.out_cpp and not args.out_py:
        ap.error("nothing to do: pass --out-cpp and/or --out-py")

    # The named .msg files load first so that they win: generating for a package that is already
    # installed (its own rebuild, or an adapter for a message package like sensor_msgs) would
    # otherwise read the installed copy, or fail outright as a duplicate.
    reg = Registry()
    for d in {os.path.dirname(os.path.abspath(m)) for m in args.msg}:
        load_dir(d, package=args.pkg, into=reg)
    if not args.no_ament:
        ament_registry(into=reg)

    if args.depfile and len(args.msg) != 1:
        ap.error("--depfile describes one message; pass a single --msg")

    for msg in args.msg:
        for path in generate(msg, args.pkg, args.out_cpp, args.out_py, reg, args.depfile):
            print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
