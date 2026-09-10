"""Emit the optional ROS-message bridge for one schema: msg_to_frame / frame_to_msg.

Kept in its own header and module because it is the one part of a generated adapter that has to
include the rosidl-generated message type. The base adapter includes nothing but flux/wire.hpp,
so a message package that never touches this file needs no dependency on rclcpp or rclpy.

Both directions copy every field, which is what the rest of flux exists to avoid -- this is for
code that already holds a ROS message object (a bridge from a plain ROS subscription, or a
library whose signature takes one). Data that can be born in the slot should use the Builder
directly (docs/en/copy_model.en.md).
"""

from .emit_cpp import ident as cpp_ident
from .emit_cpp import snake
from .emit_py import ident as py_ident
from .flatten import LeafKind
from .layout import walk

RECORDS = (LeafKind.RECORD_COLUMN, LeafKind.JAGGED)


def ros_cpp_type(type_name):
    pkg, name = type_name.split("/")
    return f"{pkg}::msg::{name}"


def ros_cpp_header(type_name):
    pkg, name = type_name.split("/")
    return f"{pkg}/msg/{snake(name)}.hpp"


def ros_py_module(type_name):
    pkg, name = type_name.split("/")
    return f"{pkg}.msg", name


def _access(base, path, sep="."):
    out = base
    for part in path:
        out = f"{out}[{part}]" if isinstance(part, int) else f"{out}{sep}{part}"
    return out


def element_types(layout):
    """Every nested ROS type the bridge has to name, plus the message itself."""
    seen = {layout.type_name}
    order = [layout.type_name]
    for path in walk(layout.root):
        et = path[-1].elem_type
        if et and et not in seen:
            seen.add(et)
            order.append(et)
    return order


# --- C++ ------------------------------------------------------------------------------------ #

def _cpp_to_frame(block, out, ind, msg, bld, depth):
    for p in block.placed:
        n, src = cpp_ident(p.name), _access(msg, p.path)
        if p.kind == LeafKind.FIXED and p.count == 1:
            out.append(f"{ind}{bld}.set__{n}({src});")
        elif p.kind == LeafKind.FIXED:
            out.append(f"{ind}{{ auto s = {bld}.{n}(); "
                       f"std::copy({src}.begin(), {src}.end(), s.begin()); }}")
        elif p.kind == LeafKind.COLUMN:
            out.append(f"{ind}{{ auto s = {bld}.alloc__{n}({src}.size()); "
                       f"std::copy({src}.begin(), {src}.end(), s.begin()); }}")
        elif p.kind == LeafKind.STRING:
            out.append(f"{ind}{bld}.set__{n}({src});")
        elif p.kind == LeafKind.STRING_ARRAY:
            i = f"i{depth}"
            out.append(f"{ind}{bld}.alloc__{n}({src}.size());")
            out.append(f"{ind}for (std::size_t {i} = 0; {i} < {src}.size(); ++{i}) {{")
            out.append(f"{ind}  {bld}.set__{n}({i}, {src}[{i}]);")
            out.append(f"{ind}}}")
        elif p.kind == LeafKind.STAMP:
            out.append(f"{ind}{bld}.set__{n}__stamp({src}.sec, {src}.nanosec);")
        elif p.kind == LeafKind.HEADER:
            out.append(f"{ind}{bld}.set__{n}__stamp({src}.stamp.sec, {src}.stamp.nanosec);")
            out.append(f"{ind}{bld}.set__{n}__frame_id({src}.frame_id);")
        else:
            i, arr, elem = f"i{depth}", f"a{depth}", f"e{depth}"
            out.append(f"{ind}{{")
            out.append(f"{ind}  auto {arr} = {bld}.alloc__{n}({src}.size());")
            out.append(f"{ind}  for (std::size_t {i} = 0; {i} < {src}.size(); ++{i}) {{")
            out.append(f"{ind}    auto {elem} = {arr}[{i}];")
            _cpp_to_frame(p.elem, out, ind + "    ", f"{src}[{i}]", elem, depth + 1)
            out.append(f"{ind}  }}")
            out.append(f"{ind}}}")


def _cpp_to_msg(block, out, ind, msg, view, depth, tn):
    for p in block.placed:
        n, dst = cpp_ident(p.name), _access(msg, p.path)
        if p.kind == LeafKind.FIXED and p.count == 1:
            out.append(f"{ind}{dst} = {view}.{n}();")
        elif p.kind == LeafKind.FIXED:
            out.append(f"{ind}{{ auto s = {view}.{n}(); "
                       f"std::copy(s.begin(), s.end(), {dst}.begin()); }}")
        elif p.kind == LeafKind.COLUMN:
            out.append(f"{ind}{{ auto s = {view}.{n}(); {dst}.assign(s.begin(), s.end()); }}")
        elif p.kind == LeafKind.STRING:
            out.append(f"{ind}{dst} = std::string({view}.{n}());")
        elif p.kind == LeafKind.STAMP:
            out.append(f"{ind}{dst}.sec = {view}.{n}__sec();")
            out.append(f"{ind}{dst}.nanosec = {view}.{n}__nanosec();")
        elif p.kind == LeafKind.HEADER:
            out.append(f"{ind}{dst}.stamp.sec = {view}.{n}__sec();")
            out.append(f"{ind}{dst}.stamp.nanosec = {view}.{n}__nanosec();")
            out.append(f"{ind}{dst}.frame_id = std::string({view}.{n}__frame_id());")
        else:
            i, cnt = f"i{depth}", f"n{depth}"
            out.append(f"{ind}{{")
            out.append(f"{ind}  const std::size_t {cnt} = {view}.{n}__size();")
            if p.is_dynamic:
                out.append(f"{ind}  {dst}.resize({cnt});")
            else:
                # A fixed-size array is a std::array with no resize, and the frame's length is
                # data while the schema's N is not: a frame that disagrees cannot be represented,
                # and a clamped copy is a silently wrong message. Refuse it loudly.
                label = _access("", p.path).lstrip(".")
                out.append(f"{ind}  if ({cnt} != {p.count}) {{")
                out.append(
                    f'{ind}    throw std::length_error("flux: {tn}.{label}: frame holds " + '
                    f'std::to_string({cnt}) + " elements, schema fixes {p.count}");')
                out.append(f"{ind}  }}")
            out.append(f"{ind}  for (std::size_t {i} = 0; {i} < {cnt}; ++{i}) {{")
            if p.kind == LeafKind.STRING_ARRAY:
                out.append(f"{ind}    {dst}[{i}] = std::string({view}.{n}({i}));")
            else:
                elem = f"e{depth}"
                out.append(f"{ind}    auto {elem} = {view}.{n}({i});")
                _cpp_to_msg(p.elem, out, ind + "    ", f"{dst}[{i}]", elem, depth + 1, tn)
            out.append(f"{ind}  }}")
            out.append(f"{ind}}}")


def emit_cpp(layout, source):
    if layout.rejected:
        raise ValueError(f"{layout.type_name}: {layout.rejected}")
    ns = layout.package or "flux_msg"
    guard = f"{ns.upper()}__FLUX__{snake(layout.name).upper()}_ROS_HPP_"
    ros = ros_cpp_type(layout.type_name)

    includes = "".join(
        f'#include "{ros_cpp_header(t)}"\n' for t in element_types(layout))
    to_frame, to_msg = [], []
    _cpp_to_frame(layout.root, to_frame, "  ", "m", "b", 0)
    _cpp_to_msg(layout.root, to_msg, "  ", "m", "v", 0, layout.type_name)
    stdexcept = ("#include <stdexcept>\n"
                 if any("std::length_error" in l for l in to_msg) else "")

    return f"""// generated by flux_gen from {source} -- do not edit.
#ifndef {guard}
#define {guard}

#include "{ns}/flux/{snake(layout.name)}.hpp"
{includes}
#include <algorithm>
#include <cstddef>
{stdexcept}#include <string>

namespace {ns}::flux_msg
{{

// Copy every field of a {layout.type_name} into a loaned slot. This is the 1-copy path: the
// message owns its own buffers, so nothing can be aliased (docs/en/copy_model.en.md). Prefer writing
// into {layout.name}::Builder directly when the data has not been built as a ROS message yet.
inline void msg_to_frame(const {ros} & m, {layout.name}::Builder & b)
{{
{chr(10).join(to_frame)}
}}

// Copy a received frame back out into a ROS message. One copy per subscriber, which is the cost
// flux exists to avoid -- use {layout.name}::View directly unless an existing signature needs
// the message object.
inline void frame_to_msg(const {layout.name}::View & v, {ros} & m)
{{
{chr(10).join(to_msg)}
}}

inline {ros} frame_to_msg(const {layout.name}::View & v)
{{
  {ros} m;
  frame_to_msg(v, m);
  return m;
}}

}}  // namespace {ns}::flux_msg

#endif  // {guard}
"""


# --- Python --------------------------------------------------------------------------------- #

def _py_to_frame(block, out, ind, msg, bld, depth):
    for p in block.placed:
        n, src = py_ident(p.name), _access(msg, p.path)
        if p.kind == LeafKind.FIXED and p.count == 1:
            out.append(f"{ind}{bld}.{n} = {src}")
        elif p.kind == LeafKind.FIXED:
            out.append(f"{ind}{bld}.{n}[:] = {src}")
        elif p.kind == LeafKind.COLUMN:
            out.append(f"{ind}{bld}.alloc__{n}(len({src}))[:] = {src}")
        elif p.kind in (LeafKind.STRING, LeafKind.STRING_ARRAY):
            out.append(f"{ind}{bld}.{n} = {src}")
        elif p.kind == LeafKind.STAMP:
            out.append(f"{ind}{bld}.set__{n}__stamp({src}.sec, {src}.nanosec)")
        elif p.kind == LeafKind.HEADER:
            out.append(f"{ind}{bld}.set__{n}__stamp({src}.stamp.sec, {src}.stamp.nanosec)")
            out.append(f"{ind}{bld}.{n}__frame_id = {src}.frame_id")
        else:
            src_v, arr, i, elem = f"_s{depth}", f"_a{depth}", f"_i{depth}", f"_e{depth}"
            out.append(f"{ind}{src_v} = {src}")
            out.append(f"{ind}{arr} = {bld}.alloc__{n}(len({src_v}))")
            out.append(f"{ind}for {i}, {elem} in enumerate({arr}):")
            _py_to_frame(p.elem, out, ind + "    ", f"{src_v}[{i}]", elem, depth + 1)


def _py_to_msg(block, out, ind, msg, view, depth, alias, tn):
    for p in block.placed:
        n, dst = py_ident(p.name), _access(msg, p.path)
        if p.kind == LeafKind.STRING_ARRAY and not p.is_dynamic:
            # Same rule as the C++ side: a fixed-size array cannot hold a disagreeing frame
            # length, and rclpy's own assertion would fire too late and say too little.
            tmp, label = f"_t{depth}", _access("", p.path).lstrip(".")
            out.append(f"{ind}{tmp} = {view}.{n}")
            out.append(f"{ind}if len({tmp}) != {p.count}:")
            out.append(f'{ind}    raise ValueError(f"flux: {tn}.{label}: '
                       f'frame holds {{len({tmp})}} elements, schema fixes {p.count}")')
            out.append(f"{ind}{dst} = {tmp}")
        elif p.kind in (LeafKind.FIXED, LeafKind.COLUMN, LeafKind.STRING, LeafKind.STRING_ARRAY):
            # rclpy copies on assignment, so handing it a view over the slot is safe.
            out.append(f"{ind}{dst} = {view}.{n}")
        elif p.kind == LeafKind.STAMP:
            out.append(f"{ind}{dst}.sec, {dst}.nanosec = {view}.{n}__stamp")
        elif p.kind == LeafKind.HEADER:
            out.append(f"{ind}{dst}.stamp.sec, {dst}.stamp.nanosec = {view}.{n}__stamp")
            out.append(f"{ind}{dst}.frame_id = {view}.{n}__frame_id")
        else:
            seq, lst, i, elem = f"_q{depth}", f"_l{depth}", f"_i{depth}", f"_e{depth}"
            cls = alias[p.elem_type]
            out.append(f"{ind}{seq} = {view}.{n}")
            if not p.is_dynamic:
                label = _access("", p.path).lstrip(".")
                out.append(f"{ind}if len({seq}) != {p.count}:")
                out.append(f'{ind}    raise ValueError(f"flux: {tn}.{label}: '
                           f'frame holds {{len({seq})}} elements, schema fixes {p.count}")')
            out.append(f"{ind}{lst} = [{cls}() for _ in range(len({seq}))]")
            out.append(f"{ind}for {i}, {elem} in enumerate({seq}):")
            _py_to_msg(p.elem, out, ind + "    ", f"{lst}[{i}]", elem, depth + 1, alias, tn)
            out.append(f"{ind}{dst} = {lst}")


def emit_py(layout, source):
    if layout.rejected:
        raise ValueError(f"{layout.type_name}: {layout.rejected}")

    types = element_types(layout)
    alias = {t: f"_Ros{i}" for i, t in enumerate(types)}
    imports = "\n".join(
        f"from {ros_py_module(t)[0]} import {ros_py_module(t)[1]} as {alias[t]}" for t in types)

    to_frame, to_msg = [], []
    _py_to_frame(layout.root, to_frame, "    ", "m", "b", 0)
    _py_to_msg(layout.root, to_msg, "    ", "m", "v", 0, alias, layout.type_name)
    root = alias[layout.type_name]

    return f'''"""ROS message bridge for {layout.type_name}.

generated by flux_gen from {source} -- do not edit.

Importing this module pulls in the rclpy message type; the base adapter does not, so code that
never converts a ROS message object needs neither this module nor rclpy.
"""

{imports}

from .{snake(layout.name)} import Builder, View  # noqa: F401

MESSAGE = {root}


def msg_to_frame(m, b):
    """Copy every field of a {layout.type_name} into a Builder over a loaned slot.

    The 1-copy path: the message owns its own buffers, so nothing can be aliased
    (docs/en/copy_model.en.md). Write into the Builder directly when the data has not been built as a
    ROS message yet.
    """
{chr(10).join(to_frame)}
    return b


def frame_to_msg(v, m=None):
    """Copy a received frame back out into a ROS message.

    One copy per subscriber, which is the cost flux exists to avoid -- read the View directly
    unless an existing signature needs the message object.
    """
    if m is None:
        m = {root}()
{chr(10).join(to_msg)}
    return m
'''
