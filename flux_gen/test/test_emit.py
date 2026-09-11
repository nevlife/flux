"""Codegen tests: the generated adapters round-trip, and the two languages agree byte for byte.

The cross-language test is the one that matters. Everything else in flux_gen can be checked by
reading the output; that the C++ header and the Python module put a field at the same offset can
only be checked by having one write a frame and the other read it.
"""

import os
import shutil
import subprocess
import sys

import pytest

from flux_gen import build_layout, flatten_message, load_dir
from flux_gen.cli import generate

MSG_DIR = os.path.join(os.path.dirname(__file__), "msg")
CORE_INCLUDE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "flux_core", "include"))

GOOD = ["Vec3", "Pose", "FlatPose", "PointCloud", "Labeled", "NestedString", "Trajectory",
        "TensorList", "Tensor", "Event", "Names", "Status", "FixedNames"]

needs_cxx = pytest.mark.skipif(
    shutil.which("g++") is None or not os.path.isdir(CORE_INCLUDE),
    reason="needs g++ and flux_core headers")


def _core_lib():
    """The compiled flux_core. A test that only checks syntax does not need it; one that links
    does, because the generated Builder can own a WriteSlot and so needs its destructor."""
    env = os.environ.get("FLUX_CORE_LIB")
    if env and os.path.isfile(env):
        return env
    ws = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    for rel in ("install/flux_core/lib/libflux_core.a", "build/flux_core/libflux_core.a"):
        cand = os.path.join(ws, rel)
        if os.path.isfile(cand):
            return cand
    return None


CORE_LIB = _core_lib()

needs_link = pytest.mark.skipif(
    shutil.which("g++") is None or not os.path.isdir(CORE_INCLUDE) or CORE_LIB is None,
    reason="needs g++, flux_core headers and a built libflux_core")


@pytest.fixture(scope="module")
def reg():
    return load_dir(MSG_DIR)


@pytest.fixture(scope="module")
def built(tmp_path_factory, reg):
    """Generate every good schema once; return (cpp_root, py_root)."""
    out = tmp_path_factory.mktemp("gen")
    cpp, py = str(out / "include"), str(out / "python")
    for name in GOOD:
        generate(os.path.join(MSG_DIR, f"{name}.msg"), "test", cpp, py, reg)
    sys.path.insert(0, py)
    yield cpp, py
    sys.path.remove(py)


def module(name):
    from flux_gen.emit_cpp import snake
    return __import__(f"test_flux.{snake(name)}", fromlist=["Builder"])


@pytest.mark.parametrize("name", GOOD)
def test_generated_module_imports_and_declares_its_schema(built, reg, name):
    m = module(name)
    layout = build_layout(flatten_message(reg[name], reg))
    assert m.FINGERPRINT__ == layout.fingerprint
    assert m.SCALAR_BYTES__ == layout.scalar_bytes
    assert m.TYPE_NAME__ == layout.type_name
    assert getattr(m, name).FINGERPRINT__ == m.FINGERPRINT__


def test_columnar_round_trip(built):
    m = module("PointCloud")
    buf = bytearray(4096)
    b = m.Builder(buf)
    b.alloc__x(3)[:] = [1.0, 2.0, 3.0]
    b.alloc__y(3)[:] = [4.0, 5.0, 6.0]
    b.alloc__z(3)[:] = [7.0, 8.0, 9.0]
    b.width = 3
    b.height = 1

    v = m.View(memoryview(buf)[:b.size__])
    assert list(v.x) == [1.0, 2.0, 3.0]
    assert list(v.z) == [7.0, 8.0, 9.0]
    assert (v.width, v.height) == (3, 1)


def test_string_and_stamp_round_trip(built):
    m = module("Event")
    buf = bytearray(4096)
    b = m.Builder(buf)
    b.set__stamp__stamp(-7, 250000000)
    b.tags = ["a", "", "unicode å"]
    b.alloc__values(2)[:] = [0.5, -0.5]

    v = m.View(memoryview(buf)[:b.size__])
    assert v.stamp__stamp == (-7, 250000000)
    assert v.tags == ["a", "", "unicode å"]
    assert list(v.values) == [0.5, -0.5]


def test_jagged_round_trip(built):
    m = module("TensorList")
    buf = bytearray(8192)
    b = m.Builder(buf)
    elems = b.alloc__tensors(3)
    for i, e in enumerate(elems):
        e.alloc__shape(2)[:] = [i, i + 1]
        e.alloc__data(i)[:] = range(i)

    v = m.View(memoryview(buf)[:b.size__])
    assert len(v.tensors) == 3
    for i, e in enumerate(v.tensors):
        assert list(e.shape) == [i, i + 1]
        assert list(e.data) == list(range(i))


def test_record_round_trip(built):
    m = module("Trajectory")
    buf = bytearray(4096)
    b = m.Builder(buf)
    poses = b.alloc__poses(2)
    poses[0].position_x = 1.0
    poses[1].orientation_z = 2.0

    v = m.View(memoryview(buf)[:b.size__])
    assert len(v.poses) == 2
    assert v.poses[0].position_x == 1.0
    assert v.poses[1].orientation_z == 2.0
    assert v.poses[0].orientation_z == 0.0  # element blocks start zeroed


def test_received_views_are_read_only(built):
    m = module("PointCloud")
    buf = bytearray(1024)
    b = m.Builder(buf)
    b.alloc__x(2)[:] = [1.0, 2.0]
    v = m.View(memoryview(buf)[:b.size__])
    with pytest.raises(ValueError):
        v.x[0] = 9.0


def test_a_frame_larger_than_the_slot_is_refused(built):
    from flux_gen.wire import WireError
    m = module("PointCloud")
    b = m.Builder(bytearray(64))
    with pytest.raises(WireError):
        b.alloc__x(1000)


def test_a_truncated_frame_does_not_read_past_its_end(built):
    m = module("PointCloud")
    buf = bytearray(4096)
    b = m.Builder(buf)
    b.alloc__x(4)[:] = [1.0, 2.0, 3.0, 4.0]
    from flux_gen.wire import WireError
    v = m.View(memoryview(buf)[:b.size__ - 8])  # descriptor now points past the frame
    with pytest.raises(WireError):
        list(v.x)


def test_constants_are_re_emitted(built):
    m = module("Status")
    assert (m.ERROR_NONE, m.ERROR_GNSS, m.ON) == (0, 3, True)
    assert m.EPS == pytest.approx(1e-6)
    # String constants follow rosidl: matching quotes are stripped, an unquoted value is taken
    # verbatim with the comment removed. The parity matters -- the same .msg must yield the
    # same constant on the ROS side and the flux side.
    assert m.TAG == "unit=m/s#kept"
    assert m.RAW == "bare value"
    assert m.Status.ERROR_GNSS == 3


def test_f16_has_no_cpp_mapping_and_says_why():
    # C++17 has no standard 16-bit float type; the C++ emitter must refuse f16 with the reason
    # rather than fall through to an internal-looking KeyError or hand out raw bits.
    from flux_gen.dtypes import DType
    from flux_gen.emit_cpp import cpp_type
    with pytest.raises(ValueError, match="f16"):
        cpp_type(DType.F16)


def test_ros_bridge_refuses_a_fixed_length_mismatch(reg):
    # The frame's length is data; the schema's T[N] is not. The generated bridge must refuse a
    # frame that disagrees instead of clamping (old C++) or half-copying before rclpy's late
    # assertion (Python). One throw/raise per length-carrying fixed leaf: names and pair.
    from flux_gen import build_layout, flatten_message
    from flux_gen.emit_ros import emit_cpp as bridge_cpp
    from flux_gen.emit_ros import emit_py as bridge_py
    layout = build_layout(flatten_message(reg["FixedNames"], reg))
    cpp = bridge_cpp(layout, "test/msg/FixedNames.msg")
    assert "std::min" not in cpp
    assert cpp.count("throw std::length_error") == 2
    assert "#include <stdexcept>" in cpp
    py = bridge_py(layout, "test/msg/FixedNames.msg")
    assert py.count("raise ValueError") == 2


def test_py_bridge_raises_on_a_fixed_length_mismatch(built):
    # Executable half of the check above. A stand-in message class is enough here: the length
    # check fires before any field agreement with rosidl is at stake.
    import types
    saved = {k: sys.modules.get(k) for k in ("test", "test.msg")}
    pkg, msgs = types.ModuleType("test"), types.ModuleType("test.msg")

    class FixedNames:
        pass

    class Tensor:
        pass

    msgs.FixedNames, msgs.Tensor = FixedNames, Tensor
    pkg.msg = msgs
    sys.modules["test"], sys.modules["test.msg"] = pkg, msgs
    try:
        import test_flux.fixed_names_ros as fn_ros
        m = module("FixedNames")

        def frame(n_names, n_pair):
            # The Builder now refuses a wrong length, so a mismatched frame -- what a foreign
            # or older writer could still produce -- is crafted on the raw Writer.
            from flux_gen.wire import Writer
            buf = bytearray(8192)
            w = Writer(buf, m.SCALAR_BYTES__)
            w.put_strs(0, ["x"] * n_names)
            pair_at = w.alloc_elems(8, n_pair, m.PairElem.STRIDE, m.PairElem.ALIGN)
            for i in range(n_pair):
                e = m.PairElemBuilder(w, pair_at + i * m.PairElem.STRIDE)
                e.alloc__shape(1)[:] = [1]
                e.alloc__data(1)[:] = [0.5]
            return m.View(memoryview(buf)[:w.size])

        with pytest.raises(ValueError, match="names.*schema fixes 2"):
            fn_ros.frame_to_msg(frame(3, 2))
        with pytest.raises(ValueError, match="pair.*schema fixes 2"):
            fn_ros.frame_to_msg(frame(2, 3))
        assert fn_ros.frame_to_msg(frame(2, 2)).names == ["x", "x"]
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


def test_constants_stay_out_of_the_fingerprint(reg):
    # Adding a constant must not move the fingerprint: it is not wire data, so a peer built
    # before it was added still reads the same bytes and must still meet this one.
    from flux_gen import fingerprint, parse_msg
    body = "uint16 code\nfloat64[] values\n"
    bare = flatten_message(parse_msg(body, "Status", package="test"), reg)
    withc = flatten_message(
        parse_msg("uint16 EXTRA = 9\n" + body, "Status", package="test"), reg)
    assert withc.constants and not bare.constants
    assert bare.leaves == withc.leaves
    assert fingerprint(bare.leaves, bare.type_name) == fingerprint(
        withc.leaves, withc.type_name)


CXX_DRIVER = r"""
#include "test/flux/point_cloud.hpp"
#include "test/flux/tensor_list.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

using test::flux_msg::PointCloud;
using test::flux_msg::TensorList;

// Writes one PointCloud frame to argv[1], then reads back the TensorList frame at argv[2] that
// Python wrote and prints it. Both directions in one process, so a single compile covers them.
int main(int argc, char ** argv)
{
  std::vector<unsigned char> buf(8192);
  PointCloud::Builder b(buf.data(), buf.size());
  auto x = b.alloc__x(3);
  x[0] = 1.5f; x[1] = 2.5f; x[2] = 3.5f;
  auto z = b.alloc__z(2);
  z[0] = -1.0f; z[1] = -2.0f;
  b.set__width(640);
  b.set__height(480);
  if (!b.ok__()) { return 2; }
  FILE * out = std::fopen(argv[1], "wb");
  std::fwrite(buf.data(), 1, b.size__(), out);
  std::fclose(out);

  FILE * in = std::fopen(argv[2], "rb");
  std::vector<unsigned char> got(65536);
  const std::size_t n = std::fread(got.data(), 1, got.size(), in);
  std::fclose(in);
  TensorList::View v(got.data(), n);
  std::printf("%zu\n", v.tensors__size());
  for (std::size_t i = 0; i < v.tensors__size(); ++i) {
    auto e = v.tensors(i);
    for (auto s : e.shape()) { std::printf("%u ", s); }
    std::printf("| ");
    for (auto d : e.data()) { std::printf("%g ", d); }
    std::printf("\n");
  }
  return v.ok__() ? 0 : 3;
}
"""


@needs_cxx
def test_generated_headers_compile(built, tmp_path):
    cpp_root, _ = built
    src = tmp_path / "all.cpp"
    src.write_text("".join(
        f'#include "test/flux/{__import__("flux_gen.emit_cpp", fromlist=["snake"]).snake(n)}.hpp"\n'
        for n in GOOD) + "int main() { return 0; }\n")
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-fsyntax-only",
         f"-I{cpp_root}", f"-I{CORE_INCLUDE}", str(src)],
        check=True, capture_output=True, text=True)


@needs_link
def test_cpp_and_python_agree_on_the_bytes(built, tmp_path):
    cpp_root, _ = built
    src, exe = tmp_path / "driver.cpp", tmp_path / "driver"
    src.write_text(CXX_DRIVER)
    subprocess.run(
        ["g++", "-std=c++17", "-O1", f"-I{cpp_root}", f"-I{CORE_INCLUDE}",
         str(src), CORE_LIB, "-pthread", "-o", str(exe)],
        check=True, capture_output=True, text=True)

    # Python writes a jagged frame for C++ to read.
    tl = module("TensorList")
    buf = bytearray(8192)
    bb = tl.Builder(buf)
    for i, e in enumerate(bb.alloc__tensors(2)):
        e.alloc__shape(1)[:] = [i + 7]
        e.alloc__data(2)[:] = [i + 0.25, i + 0.5]
    from_py = tmp_path / "from_py.bin"
    from_py.write_bytes(bytes(memoryview(buf)[:bb.size__]))

    to_py = tmp_path / "from_cpp.bin"
    proc = subprocess.run([str(exe), str(to_py), str(from_py)],
                          check=True, capture_output=True, text=True)
    assert proc.stdout.split("\n")[0] == "2"
    assert proc.stdout.split("\n")[1] == "7 | 0.25 0.5 "
    assert proc.stdout.split("\n")[2] == "8 | 1.25 1.5 "

    # ...and reads back what C++ wrote.
    pc = module("PointCloud")
    v = pc.View(to_py.read_bytes())
    assert list(v.x) == [1.5, 2.5, 3.5]
    assert list(v.y) == []
    assert list(v.z) == [-1.0, -2.0]
    assert (v.width, v.height) == (640, 480)


# --- ROS bridge -------------------------------------------------------------------------------

def _has_rclpy():
    try:
        import geometry_msgs.msg  # noqa: F401
        return True
    except Exception:
        return False


needs_rclpy = pytest.mark.skipif(not _has_rclpy(), reason="needs ROS messages on the path")


@needs_rclpy
def test_ros_bridge_round_trips_real_messages(tmp_path):
    # The generated bridge is checked against the real rclpy types, not a stand-in: the whole
    # point is that it agrees with rosidl on every field, nesting and array.
    import numpy as np
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Image, PointCloud2, PointField

    from flux_gen.cli import ament_registry, generate
    reg = ament_registry()
    cpp, py = str(tmp_path / "inc"), str(tmp_path / "py")
    for key in ("geometry_msgs/PoseStamped", "sensor_msgs/Image", "sensor_msgs/PointCloud2"):
        pkg, stem = key.split("/")
        generate(reg[key].path, pkg, cpp, py, reg)
    for pkg in ("geometry_msgs_flux", "sensor_msgs_flux"):
        open(os.path.join(py, pkg, "__init__.py"), "a").close()
    sys.path.insert(0, py)
    try:
        import geometry_msgs_flux.pose_stamped_ros as ps_ros
        import sensor_msgs_flux.image_ros as img_ros
        import sensor_msgs_flux.point_cloud2_ros as pc2_ros

        def rt(mod, msg):
            buf = bytearray(1 << 20)
            b = mod.Builder(buf)
            mod.msg_to_frame(msg, b)
            back = mod.frame_to_msg(mod.View(memoryview(buf)[:b.size__]))
            # Equality is Python-level; only the C converter notices a numpy scalar in an int
            # field, and it aborts the process rather than raising.
            assert deserialize_message(serialize_message(back), type(msg)) == msg
            return back

        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp.sec, ps.header.stamp.nanosec = 12, 34
        ps.pose.position.x, ps.pose.position.z = 1.5, 3.5
        ps.pose.orientation.w = 1.0
        assert rt(ps_ros, ps) == ps  # fixed nesting

        im = Image()
        im.header.frame_id = "cam"
        im.height, im.width, im.encoding, im.step = 4, 8, "mono8", 8
        im.data = np.arange(32, dtype=np.uint8).tobytes()
        assert rt(img_ros, im) == im  # columnar + string

        pc = PointCloud2()
        pc.header.frame_id = "lidar"
        pc.height, pc.width = 1, 3
        pc.fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
                     for i, n in enumerate(("x", "y", "z"))]
        pc.point_step, pc.row_step = 12, 36
        pc.data = bytes(range(36))
        pc.is_dense = True
        assert rt(pc2_ros, pc) == pc  # jagged with a string inside each element
    finally:
        sys.path.remove(py)


def _ros_include_dirs():
    """Every installed ROS package's include dir, as rosidl headers reference each other."""
    dirs = []
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        inc = os.path.join(prefix, "include")
        if prefix and os.path.isdir(inc):
            dirs += [os.path.join(inc, d) for d in os.listdir(inc)
                     if os.path.isdir(os.path.join(inc, d))]
    return dirs


@needs_cxx
@needs_rclpy
def test_ros_bridge_compiles_against_rosidl_headers(tmp_path):
    # The Python half of the bridge is checked by round-tripping; the C++ half can only be
    # checked by handing it to a compiler with the real generated message headers.
    from flux_gen.cli import ament_registry, generate
    reg = ament_registry()
    cpp = str(tmp_path / "inc")
    for key in ("geometry_msgs/PoseStamped", "sensor_msgs/Image", "sensor_msgs/PointCloud2"):
        generate(reg[key].path, key.split("/")[0], cpp, None, reg)

    src = tmp_path / "bridge.cpp"
    src.write_text(
        '#include "geometry_msgs/flux/pose_stamped_ros.hpp"\n'
        '#include "sensor_msgs/flux/image_ros.hpp"\n'
        '#include "sensor_msgs/flux/point_cloud2_ros.hpp"\n'
        "int main() {\n"
        "  sensor_msgs::msg::PointCloud2 m;\n"
        "  unsigned char buf[4096];\n"
        "  sensor_msgs::flux_msg::PointCloud2::Builder b(buf, sizeof(buf));\n"
        "  sensor_msgs::flux_msg::msg_to_frame(m, b);\n"
        "  sensor_msgs::flux_msg::PointCloud2::View v(buf, b.size__());\n"
        "  return sensor_msgs::flux_msg::frame_to_msg(v).width == 0 ? 0 : 1;\n"
        "}\n")
    inc = [f"-I{d}" for d in _ros_include_dirs()]
    if not inc:
        pytest.skip("flux-cap:ros-headers no ROS include dirs on AMENT_PREFIX_PATH")
    subprocess.run(
        ["g++", "-std=c++17", "-fsyntax-only", f"-I{cpp}", f"-I{CORE_INCLUDE}", *inc, str(src)],
        check=True, capture_output=True, text=True)


def test_py_builder_rejects_a_fixed_length_mismatch(built):
    # message_shapes.md: a length that disagrees with the schema's T[N] is an explicit error at
    # the write too, so the bad frame is never produced.
    m = module("FixedNames")
    b = m.Builder(bytearray(8192))
    with pytest.raises(ValueError, match="names expects exactly 2"):
        b.names = ["x", "y", "z"]
    with pytest.raises(ValueError, match="pair expects exactly 2"):
        b.alloc__pair(3)
    b.names = ["x", "y"]


@needs_link
def test_cpp_build_loans_from_a_publisher_and_commits(built, tmp_path):
    # Msg::build__(pub) is the C++ counterpart of the Python Msg.build__(pub): one line for what was
    # loan() plus a Builder over it. The Builder owns the slot, so the caller keeps one object.
    # A build() whose loan found no free slot is false and its commit() reports Backpressure --
    # a dropped frame, not a torn one.
    cpp_root, _ = built
    src, exe = tmp_path / "build_helper.cpp", tmp_path / "build_helper"
    src.write_text(
        '#include "test/flux/vec3.hpp"\n'
        '#include "flux/channel.hpp"\n'
        "#include <cstdio>\n"
        "int main()\n"
        "{\n"
        "  flux::Channel ch(4096, 2);\n"
        "  auto b = test::flux_msg::Vec3::build__(ch);\n"
        '  std::printf("%d\\n", static_cast<bool>(b) ? 1 : 0);\n'
        "  b.set__x(1.5);\n"
        "  b.set__y(2.5);\n"
        "  b.set__z(3.5);\n"
        '  std::printf("%d\\n", b.commit__() == flux::Published::Ok ? 1 : 0);\n'
        "  auto v = ch.take();\n"
        "  test::flux_msg::Vec3::View view(v);\n"
        '  std::printf("%d\\n", (view.ok__() && view.x() == 1.5 && view.z() == 3.5) ? 1 : 0);\n'
        "  // Every slot borrowed: build() comes back false and refuses to publish.\n"
        "  auto h1 = ch.loan();\n"
        "  auto h2 = ch.loan();\n"
        "  auto full = test::flux_msg::Vec3::build__(ch);\n"
        '  std::printf("%d\\n", static_cast<bool>(full) ? 1 : 0);\n'
        '  std::printf("%d\\n", full.commit__() == flux::Published::Backpressure ? 1 : 0);\n'
        "  return 0;\n"
        "}\n")
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
         f"-I{cpp_root}", f"-I{CORE_INCLUDE}", str(src), CORE_LIB, "-pthread", "-o", str(exe)],
        check=True, capture_output=True, text=True)
    proc = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert proc.stdout == "1\n1\n1\n0\n1\n"


@needs_link
def test_cpp_builder_poisons_a_fixed_length_mismatch(built, tmp_path):
    cpp_root, _ = built
    src = tmp_path / "poison.cpp"
    src.write_text(
        '#include "test/flux/fixed_names.hpp"\n'
        "#include <cstdio>\n"
        "int main()\n"
        "{\n"
        "  unsigned char buf[8192];\n"
        "  test::flux_msg::FixedNames::Builder b(buf, sizeof buf);\n"
        "  b.alloc__names(3);\n"
        '  std::printf("%d\\n", b.ok__() ? 1 : 0);\n'
        "  test::flux_msg::FixedNames::Builder c(buf, sizeof buf);\n"
        "  c.alloc__pair(3);\n"
        '  std::printf("%d\\n", c.ok__() ? 1 : 0);\n'
        "  return 0;\n"
        "}\n")
    exe = tmp_path / "poison"
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
         f"-I{cpp_root}", f"-I{CORE_INCLUDE}", str(src), CORE_LIB, "-pthread", "-o", str(exe)],
        check=True, capture_output=True, text=True)
    proc = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert proc.stdout == "0\n0\n"
