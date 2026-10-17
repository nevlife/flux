"""Codegen tests: the generated adapters round-trip, and the two languages agree byte for byte.

The cross-language test is the one that matters. Everything else flux2_gen does is checked by its
own cargo tests; that the C++ header and the Python module put a field at the same offset can
only be checked by having one write a frame and the other read it.
"""

import os
import re
import shutil
import subprocess
import sys

import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory

_FLUX2 = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MSG_DIR = os.path.join(_FLUX2, "flux2_gen", "test", "msg")
GEN = os.path.join(get_package_prefix("flux2_gen"), "lib", "flux2_gen", "flux2_gen")
CORE_INCLUDES = [
    os.path.join(_FLUX2, "flux2_cpp", "include"),
    os.path.join(_FLUX2, "flux2_ffi", "include"),
]
CORE_INCLUDE_FLAGS = [f"-I{d}" for d in CORE_INCLUDES]

GOOD = [
    "Vec3",
    "Pose",
    "FlatPose",
    "PointCloud",
    "Labeled",
    "NestedString",
    "Trajectory",
    "TensorList",
    "Tensor",
    "Event",
    "Names",
    "Status",
    "FixedNames",
    "DlTensor",
]

needs_cxx = pytest.mark.skipif(
    shutil.which("g++") is None or not all(os.path.isdir(d) for d in CORE_INCLUDES),
    reason="needs g++ and the flux2 headers",
)


def _core_lib_dir(prefixes):
    """Where libflux2_ffi.so is. A test that only checks syntax does not need it; one that links
    does, because the generated Builder can own a WriteSlot and so needs its destructor. Looked up
    on the install prefixes, which an isolated and a merged install both put there."""
    env = os.environ.get("FLUX2_FFI_LIB_DIR")
    for d in [env] if env else [os.path.join(p, "lib") for p in prefixes.split(os.pathsep) if p]:
        if os.path.isfile(os.path.join(d, "libflux2_ffi.so")):
            return d
    return None


CORE_LIB_DIR = _core_lib_dir(os.environ.get("CMAKE_PREFIX_PATH", ""))
CORE_LINK = [f"-L{CORE_LIB_DIR}", "-lflux2_ffi", f"-Wl,-rpath,{CORE_LIB_DIR}"]


def test_the_ffi_library_is_found_in_a_merged_install(tmp_path, monkeypatch):
    # A test that cannot find the library fails rather than skips: --merge-install put it where
    # the old lookup did not search, and seven cross-language tests skipped without a word.
    assert CORE_LIB_DIR is not None, "libflux2_ffi.so is on no CMAKE_PREFIX_PATH entry"
    (tmp_path / "lib").mkdir()
    (tmp_path / "lib" / "libflux2_ffi.so").touch()
    monkeypatch.delenv("FLUX2_FFI_LIB_DIR", raising=False)
    merged = os.pathsep.join(["/nonexistent", str(tmp_path)])
    assert _core_lib_dir(merged) == str(tmp_path / "lib")


def generate(msg, pkg, cpp=None, py=None):
    cmd = [GEN, "--pkg", pkg, "--msg", msg]
    cmd += ["--out-cpp", cpp] if cpp else []
    cmd += ["--out-py", py] if py else []
    subprocess.run(cmd, check=True, capture_output=True, text=True)


def installed(key):
    pkg, name = key.split("/")
    return os.path.join(get_package_share_directory(pkg), "msg", f"{name}.msg")


def snake(name):
    """flux2_gen's file naming: PointCloud2 -> point_cloud2, GPSFix -> gps_fix."""
    return re.sub(r"(?<=.)([A-Z])(?=[a-z])|(?<=[a-z0-9])([A-Z])", r"_\1\2", name).lower()


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    """Generate every good schema once; return (cpp_root, py_root)."""
    out = tmp_path_factory.mktemp("gen")
    cpp, py = str(out / "include"), str(out / "python")
    for name in GOOD:
        generate(os.path.join(MSG_DIR, f"{name}.msg"), "test", cpp, py)
    sys.path.insert(0, py)
    yield cpp, py
    sys.path.remove(py)


def module(name):
    return __import__(f"test_flux2.{snake(name)}", fromlist=["Builder"])


@pytest.mark.parametrize("name", GOOD)
def test_generated_module_imports_and_declares_its_schema(built, name):
    # The two adapters of one schema must carry the same identity.
    m = module(name)
    with open(os.path.join(built[0], "test", "flux2", f"{snake(name)}.hpp")) as f:
        header = f.read()
    assert f"kFingerprint = 0x{m.FINGERPRINT__:016X}ull;" in header
    assert f"kScalarBytes = {m.SCALAR_BYTES__};" in header
    assert f'kTypeName = "{m.TYPE_NAME__}";' in header
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

    v = m.View(memoryview(buf)[: b.size__])
    assert list(v.x) == [1.0, 2.0, 3.0]
    assert list(v.z) == [7.0, 8.0, 9.0]
    assert (v.width, v.height) == (3, 1)


@pytest.mark.parametrize(
    "name,field,value",
    [("PointCloud", "width", 3), ("Labeled", "label", "x"), ("Event", "tags", ["a"])],
)
def test_a_builder_field_is_write_only(built, name, field, value):
    # A Builder has no getter at all, as in C++: reading is Python's own AttributeError.
    m = module(name)
    b = m.Builder(bytearray(4096))
    setattr(b, field, value)
    assert getattr(m.Builder, field).fget is None
    with pytest.raises(AttributeError):
        getattr(b, field)


def test_a_builder_with_no_free_slot_is_false_and_harmless(built):
    # As in C++: build__ on a full ring hands back a false Builder. Writing into it goes nowhere
    # and commit__() reports BACKPRESSURE, so a caller that skipped the check drops one frame
    # instead of crashing under load.
    flux2 = pytest.importorskip("flux2")

    class FullRing:
        slot_size = 4096

        def loan(self, *args, **kwargs):
            return None

    m = module("TensorList")
    b = m.build__(FullRing())
    assert not b
    elems = b.alloc__tensors(2)
    for e in elems:
        e.alloc__shape(2)[:] = [1, 2]
    b2 = module("Event").build__(FullRing())
    b2.tags = ["a"]
    b2.set__stamp__stamp(1, 2)
    b2.alloc__values(3)[:] = [1.0, 2.0, 3.0]
    assert b.commit__() == flux2.Published.BACKPRESSURE
    assert b2.commit__() == flux2.Published.BACKPRESSURE


def test_a_string_that_is_not_utf8_reads_as_rclpy_reads_it(built):
    # rclpy decodes a string field with errors="replace": a byte that is not UTF-8 becomes U+FFFD
    # and the rest of the message still reads. A C++ publisher does not check what it wrote.
    m = module("Labeled")
    buf = bytearray(4096)
    b = m.Builder(buf)
    b.label = "ok xx"
    b.alloc__values(1)[:] = [2.5]
    at = buf.index(b"ok xx") + 3
    buf[at : at + 2] = b"\xed\x95"  # U+D55C cut after two of its three bytes
    v = m.View(memoryview(buf)[: b.size__])
    assert v.label == "ok \ufffd"
    assert list(v.values) == [2.5]


def test_string_and_stamp_round_trip(built):
    m = module("Event")
    buf = bytearray(4096)
    b = m.Builder(buf)
    b.set__stamp__stamp(-7, 250000000)
    b.tags = ["a", "", "unicode \u00e5"]
    b.alloc__values(2)[:] = [0.5, -0.5]

    v = m.View(memoryview(buf)[: b.size__])
    assert v.stamp__stamp == (-7, 250000000)
    assert v.tags == ["a", "", "unicode \u00e5"]
    assert list(v.values) == [0.5, -0.5]


def test_jagged_round_trip(built):
    m = module("TensorList")
    buf = bytearray(8192)
    b = m.Builder(buf)
    elems = b.alloc__tensors(3)
    for i, e in enumerate(elems):
        e.alloc__shape(2)[:] = [i, i + 1]
        e.alloc__data(i)[:] = range(i)

    v = m.View(memoryview(buf)[: b.size__])
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

    v = m.View(memoryview(buf)[: b.size__])
    assert len(v.poses) == 2
    assert v.poses[0].position_x == 1.0
    assert v.poses[1].orientation_z == 2.0
    assert v.poses[0].orientation_z == 0.0  # element blocks start zeroed


def test_received_views_are_read_only(built):
    m = module("PointCloud")
    buf = bytearray(1024)
    b = m.Builder(buf)
    b.alloc__x(2)[:] = [1.0, 2.0]
    v = m.View(memoryview(buf)[: b.size__])
    with pytest.raises(ValueError):
        v.x[0] = 9.0


def test_a_frame_larger_than_the_slot_is_refused(built):
    from flux2.wire import WireError

    m = module("PointCloud")
    b = m.Builder(bytearray(64))
    with pytest.raises(WireError):
        b.alloc__x(1000)


def test_a_truncated_frame_does_not_read_past_its_end(built):
    m = module("PointCloud")
    buf = bytearray(4096)
    b = m.Builder(buf)
    b.alloc__x(4)[:] = [1.0, 2.0, 3.0, 4.0]
    from flux2.wire import WireError

    v = m.View(memoryview(buf)[: b.size__ - 8])  # descriptor now points past the frame
    with pytest.raises(WireError):
        list(v.x)


def test_constants_are_re_emitted(built):
    m = module("Status")
    assert (m.ERROR_NONE, m.ERROR_GNSS, m.ON) == (0, 3, True)
    assert m.EPS == pytest.approx(1e-6)
    # String constants follow rosidl: matching quotes are stripped, an unquoted value is taken
    # verbatim with the comment removed, and a '#' ends the line even inside quotes. The parity
    # matters -- the same .msg must yield the same constant on the ROS side and the flux side.
    assert m.TAG == "unit=m/s"
    assert m.CUT == "'a"
    assert m.RAW == "bare value"
    assert m.Status.ERROR_GNSS == 3


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
        import test_flux2.fixed_names_ros as fn_ros

        m = module("FixedNames")

        def frame(n_names, n_pair):
            # The Builder now refuses a wrong length, so a mismatched frame -- what a foreign
            # or older writer could still produce -- is crafted on the raw Writer.
            from flux2.wire import Writer

            buf = bytearray(8192)
            w = Writer(buf, m.SCALAR_BYTES__)
            w.put_strs(0, ["x"] * n_names)
            pair_at = w.alloc_elems(8, n_pair, m.PairElem.STRIDE, m.PairElem.ALIGN)
            for i in range(n_pair):
                e = m.PairElemBuilder(w, pair_at + i * m.PairElem.STRIDE)
                e.alloc__shape(1)[:] = [1]
                e.alloc__data(1)[:] = [0.5]
            return m.View(memoryview(buf)[: w.size])

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


CXX_DRIVER = r"""
#include "test/flux2/point_cloud.hpp"
#include "test/flux2/tensor_list.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

using test::flux2_msg::PointCloud;
using test::flux2_msg::TensorList;

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
    src.write_text(
        "".join(f'#include "test/flux2/{snake(n)}.hpp"\n' for n in GOOD)
        + "int main() { return 0; }\n"
    )
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-fsyntax-only",
            f"-I{cpp_root}",
            *CORE_INCLUDE_FLAGS,
            str(src),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


@needs_cxx
def test_cpp_and_python_agree_on_the_bytes(built, tmp_path):
    cpp_root, _ = built
    src, exe = tmp_path / "driver.cpp", tmp_path / "driver"
    src.write_text(CXX_DRIVER)
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-O1",
            f"-I{cpp_root}",
            *CORE_INCLUDE_FLAGS,
            str(src),
            *CORE_LINK,
            "-pthread",
            "-o",
            str(exe),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    # Python writes a jagged frame for C++ to read.
    tl = module("TensorList")
    buf = bytearray(8192)
    bb = tl.Builder(buf)
    for i, e in enumerate(bb.alloc__tensors(2)):
        e.alloc__shape(1)[:] = [i + 7]
        e.alloc__data(2)[:] = [i + 0.25, i + 0.5]
    from_py = tmp_path / "from_py.bin"
    from_py.write_bytes(bytes(memoryview(buf)[: bb.size__]))

    to_py = tmp_path / "from_cpp.bin"
    proc = subprocess.run(
        [str(exe), str(to_py), str(from_py)], check=True, capture_output=True, text=True
    )
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
    from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

    cpp, py = str(tmp_path / "inc"), str(tmp_path / "py")
    keys = ("geometry_msgs/PoseStamped", "sensor_msgs/Image", "sensor_msgs/Imu")
    for key in (*keys, "sensor_msgs/PointCloud2"):
        generate(installed(key), key.split("/")[0], cpp, py)
    for pkg in ("geometry_msgs_flux2", "sensor_msgs_flux2"):
        open(os.path.join(py, pkg, "__init__.py"), "a").close()
    sys.path.insert(0, py)
    try:
        import geometry_msgs_flux2.pose_stamped_ros as ps_ros
        import sensor_msgs_flux2.image_ros as img_ros
        import sensor_msgs_flux2.imu_ros as imu_ros
        import sensor_msgs_flux2.point_cloud2_ros as pc2_ros

        def rt(mod, msg):
            buf = bytearray(1 << 20)
            b = mod.Builder(buf)
            mod.msg_to_frame(msg, b)
            back = mod.frame_to_msg(mod.View(memoryview(buf)[: b.size__]))
            # Equality is Python-level; only the C converter notices a numpy scalar in an int
            # field, and it aborts the process rather than raising.
            assert deserialize_message(serialize_message(back), type(msg)) == msg
            # A message that aliased the frame would pin a received slot for its lifetime.
            frame = np.frombuffer(buf, np.uint8)
            for name in back.get_fields_and_field_types():
                value = getattr(back, name)
                assert not (isinstance(value, np.ndarray) and np.shares_memory(value, frame)), name
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

        imu = Imu()
        imu.orientation_covariance[4] = 2.5
        assert rt(imu_ros, imu) == imu  # fixed arrays

        pc = PointCloud2()
        pc.header.frame_id = "lidar"
        pc.height, pc.width = 1, 3
        pc.fields = [
            PointField(name=n, offset=4 * i, datatype=7, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ]
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
            dirs += [
                os.path.join(inc, d) for d in os.listdir(inc) if os.path.isdir(os.path.join(inc, d))
            ]
    return dirs


@needs_cxx
@needs_rclpy
def test_ros_bridge_compiles_against_rosidl_headers(tmp_path):
    # The Python half of the bridge is checked by round-tripping; the C++ half can only be
    # checked by handing it to a compiler with the real generated message headers.
    cpp = str(tmp_path / "inc")
    for key in ("geometry_msgs/PoseStamped", "sensor_msgs/Image", "sensor_msgs/PointCloud2"):
        generate(installed(key), key.split("/")[0], cpp)

    src = tmp_path / "bridge.cpp"
    src.write_text(
        '#include "geometry_msgs/flux2/pose_stamped_ros.hpp"\n'
        '#include "sensor_msgs/flux2/image_ros.hpp"\n'
        '#include "sensor_msgs/flux2/point_cloud2_ros.hpp"\n'
        "int main() {\n"
        "  sensor_msgs::msg::PointCloud2 m;\n"
        "  unsigned char buf[4096];\n"
        "  sensor_msgs::flux2_msg::PointCloud2::Builder b(buf, sizeof(buf));\n"
        "  sensor_msgs::flux2_msg::msg_to_frame(m, b);\n"
        "  sensor_msgs::flux2_msg::PointCloud2::View v(buf, b.size__());\n"
        "  return sensor_msgs::flux2_msg::frame_to_msg(v).width == 0 ? 0 : 1;\n"
        "}\n"
    )
    inc = [f"-I{d}" for d in _ros_include_dirs()]
    if not inc:
        pytest.skip("flux-cap:ros-headers no ROS include dirs on AMENT_PREFIX_PATH")
    subprocess.run(
        ["g++", "-std=c++17", "-fsyntax-only", f"-I{cpp}", *CORE_INCLUDE_FLAGS, *inc, str(src)],
        check=True,
        capture_output=True,
        text=True,
    )


def _ros_bridges(tmp_path, keys, py=False):
    """Generate the adapter and bridge for installed messages; return (cpp_root, py_root)."""
    cpp, pyroot = str(tmp_path / "inc"), str(tmp_path / "py") if py else None
    for key in keys:
        generate(installed(key), key.split("/")[0], cpp, pyroot)
    return cpp, pyroot


def _fields_desc_offset(cpp):
    """Where PointCloud2's `fields` descriptor sits in the scalar block, read off the header."""
    with open(os.path.join(cpp, "sensor_msgs", "flux2", "point_cloud2.hpp")) as f:
        return int(
            re.search(r"fields__size\(\) const noexcept \{ return r_\.len\((\d+),", f.read())[1]
        )


# A descriptor whose elements run past the frame. Small enough that a reader trusting it still
# only allocates a little, so the failing form of the test fails instead of exhausting memory.
_BAD_FIELDS_LEN = 1000


@needs_cxx
@needs_rclpy
def test_cpp_bridge_writes_nothing_into_a_slot_too_small_for_the_message(tmp_path):
    # A slot that cannot hold the message makes the Builder hand back empty spans and latch. The
    # bridge must copy into what it was handed, not the message's length, or it writes through
    # a null span. Covers a fixed T[N] block (CameraInfo.k) and a variable column (data).
    cpp, _ = _ros_bridges(tmp_path, ("sensor_msgs/CameraInfo", "sensor_msgs/PointCloud2"))
    src, exe = tmp_path / "small_slot.cpp", tmp_path / "small_slot"
    src.write_text(
        '#include "sensor_msgs/flux2/camera_info_ros.hpp"\n'
        '#include "sensor_msgs/flux2/point_cloud2_ros.hpp"\n'
        "#include <cstdio>\n"
        "int main()\n"
        "{\n"
        "  alignas(8) unsigned char tiny[16];\n"
        "  sensor_msgs::flux2_msg::CameraInfo::Builder cb(tiny, sizeof(tiny));\n"
        "  sensor_msgs::flux2_msg::msg_to_frame(sensor_msgs::msg::CameraInfo{}, cb);\n"
        '  std::printf("%d\\n", cb.ok__() ? 1 : 0);\n'
        "  sensor_msgs::msg::PointCloud2 pc;\n"
        "  pc.data.assign(1 << 16, 7);\n"
        "  alignas(8) unsigned char small[512];\n"
        "  sensor_msgs::flux2_msg::PointCloud2::Builder pb(small, sizeof(small));\n"
        "  sensor_msgs::flux2_msg::msg_to_frame(pc, pb);\n"
        '  std::printf("%d\\n", pb.ok__() ? 1 : 0);\n'
        "  return 0;\n"
        "}\n"
    )
    inc = [f"-I{d}" for d in _ros_include_dirs()]
    if not inc:
        pytest.skip("flux-cap:ros-headers no ROS include dirs on AMENT_PREFIX_PATH")
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            f"-I{cpp}",
            *CORE_INCLUDE_FLAGS,
            *inc,
            str(src),
            *CORE_LINK,
            "-pthread",
            "-o",
            str(exe),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    proc = subprocess.run([str(exe)], capture_output=True, text=True)
    assert proc.returncode == 0, f"bridge crashed: {proc.returncode}"
    assert proc.stdout == "0\n0\n"


@needs_cxx
@needs_rclpy
def test_cpp_bridge_refuses_a_descriptor_that_runs_past_the_frame(tmp_path):
    # The reader trusts no descriptor (message_shapes.md 9). An array length is one too: the
    # View reports 0 and latches rather than hand a loop a count it cannot walk, and the bridge
    # throws rather than return a message filled with zeros.
    cpp, _ = _ros_bridges(tmp_path, ("sensor_msgs/PointCloud2",))
    src, exe = tmp_path / "bad_desc.cpp", tmp_path / "bad_desc"
    src.write_text(
        '#include "sensor_msgs/flux2/point_cloud2_ros.hpp"\n'
        "#include <cstdint>\n"
        "#include <cstdio>\n"
        "#include <cstring>\n"
        "#include <stdexcept>\n"
        "using sensor_msgs::flux2_msg::PointCloud2;\n"
        "int main()\n"
        "{\n"
        "  sensor_msgs::msg::PointCloud2 pc;\n"
        "  pc.fields.resize(2);\n"
        "  alignas(8) unsigned char buf[4096];\n"
        "  PointCloud2::Builder b(buf, sizeof(buf));\n"
        "  sensor_msgs::flux2_msg::msg_to_frame(pc, b);\n"
        f"  const std::uint32_t bad = {_BAD_FIELDS_LEN};\n"
        f"  std::memcpy(buf + {_fields_desc_offset(cpp)} + 4, &bad, sizeof(bad));\n"
        "  PointCloud2::View v(buf, b.size__());\n"
        '  std::printf("%zu\\n", v.fields__size());\n'
        '  std::printf("%d\\n", v.ok__() ? 1 : 0);\n'
        "  try {\n"
        "    sensor_msgs::flux2_msg::frame_to_msg(PointCloud2::View(buf, b.size__()));\n"
        '    std::printf("returned\\n");\n'
        "  } catch (const std::runtime_error &) {\n"
        '    std::printf("threw\\n");\n'
        "  }\n"
        "  return 0;\n"
        "}\n"
    )
    inc = [f"-I{d}" for d in _ros_include_dirs()]
    if not inc:
        pytest.skip("flux-cap:ros-headers no ROS include dirs on AMENT_PREFIX_PATH")
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            f"-I{cpp}",
            *CORE_INCLUDE_FLAGS,
            *inc,
            str(src),
            *CORE_LINK,
            "-pthread",
            "-o",
            str(exe),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    proc = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert proc.stdout == "0\n0\nthrew\n"


@needs_rclpy
def test_py_bridge_refuses_a_slot_too_small_and_a_descriptor_past_the_frame(tmp_path):
    # The Python half of the two tests above: both conditions raise WireError.
    from sensor_msgs.msg import PointCloud2, PointField

    from flux2.wire import WireError

    cpp, py = _ros_bridges(tmp_path, ("sensor_msgs/PointCloud2",), py=True)
    open(os.path.join(py, "sensor_msgs_flux2", "__init__.py"), "a").close()
    sys.path.insert(0, py)
    try:
        import sensor_msgs_flux2.point_cloud2_ros as pc2_ros

        pc = PointCloud2()
        pc.data = bytes(1 << 16)
        with pytest.raises(WireError):
            pc2_ros.msg_to_frame(pc, pc2_ros.Builder(bytearray(512)))

        pc = PointCloud2()
        pc.fields = [PointField(), PointField()]
        buf = bytearray(4096)
        b = pc2_ros.Builder(buf)
        pc2_ros.msg_to_frame(pc, b)
        at = _fields_desc_offset(cpp) + 4
        buf[at : at + 4] = _BAD_FIELDS_LEN.to_bytes(4, "little")
        with pytest.raises(WireError):
            pc2_ros.frame_to_msg(pc2_ros.View(memoryview(buf)[: b.size__]))
    finally:
        sys.path.remove(py)


@needs_rclpy
def test_py_bridge_refuses_a_frame_that_reads_the_same_bytes_twice(tmp_path):
    # The Python half of Cdr.FrameToMsgRefusesAFrameThatReadsTheSameBytesTwice: every field name
    # points at the longest one, so the copy grows with the square of the frame.
    from sensor_msgs.msg import PointCloud2, PointField

    from flux2.wire import WireError

    cpp, py = _ros_bridges(tmp_path, ("sensor_msgs/PointCloud2",), py=True)
    open(os.path.join(py, "sensor_msgs_flux2", "__init__.py"), "a").close()
    sys.path.insert(0, py)
    try:
        import sensor_msgs_flux2.point_cloud2_ros as pc2_ros

        n = 200
        pc = PointCloud2()
        pc.fields = [PointField(name="a" * (i + 1)) for i in range(n)]
        buf = bytearray(1 << 16)
        b = pc2_ros.Builder(buf)
        pc2_ros.msg_to_frame(pc, b)
        frame = memoryview(buf)[: b.size__]
        at = _fields_desc_offset(cpp)
        elems = int.from_bytes(buf[at : at + 4], "little")
        stride = pc2_ros.View(frame).fields[0].STRIDE
        last = elems + (n - 1) * stride  # each element starts with its name's descriptor
        for i in range(n - 1):
            buf[elems + i * stride : elems + i * stride + 8] = buf[last : last + 8]
        with pytest.raises(WireError, match="reads the same bytes twice"):
            pc2_ros.frame_to_msg(pc2_ros.View(frame))
    finally:
        sys.path.remove(py)


def test_py_builder_refuses_to_commit_after_a_failed_write(built):
    # C++ latches a failed write and commit__() refuses the frame. Python raises at the write and
    # must refuse the same commit, or a caller that caught the error publishes a torn frame.
    flux2 = pytest.importorskip("flux2")
    from flux2.wire import WireError

    committed = []

    class Loan:
        def commit(self, nbytes):
            committed.append(nbytes)
            return flux2.Published.OK

    b = module("PointCloud").Builder(bytearray(64), Loan())
    b.width = 3
    with pytest.raises(WireError):
        b.alloc__x(1000)
    assert b.commit__() == flux2.Published.TOO_LARGE

    c = module("FixedNames").Builder(bytearray(8192), Loan())
    with pytest.raises(ValueError):
        c.names = ["x", "y", "z"]
    assert c.commit__() == flux2.Published.TOO_LARGE
    assert committed == []


@pytest.mark.parametrize(
    "name,write,error",
    [
        ("PointCloud", lambda b: setattr(b, "width", -1), OverflowError),
        ("PointCloud", lambda b: setattr(b, "width", None), TypeError),
        ("PointCloud", lambda b: b.alloc__x(-1), OverflowError),
        ("Labeled", lambda b: setattr(b, "label", 5), TypeError),
        ("Labeled", lambda b: setattr(b, "label", "\ud800"), UnicodeEncodeError),
        ("Event", lambda b: setattr(b, "tags", ["a", None]), TypeError),
        ("Event", lambda b: b.set__stamp__stamp(2**31, 0), OverflowError),
        ("FixedNames", lambda b: setattr(b, "names", "xy"), TypeError),
        ("FixedNames", lambda b: setattr(b, "names", iter(["x"])), ValueError),
        ("TensorList", lambda b: b.alloc__tensors(None), TypeError),
    ],
)
def test_py_builder_refuses_to_commit_after_a_setter_raised(built, name, write, error):
    # A setter that raises (bad type, wrong length, out of range) leaves the frame torn: the
    # field is zero, stale or half written. As in C++, commit__() then refuses it.
    flux2 = pytest.importorskip("flux2")
    committed = []

    class Loan:
        def commit(self, nbytes):
            committed.append(nbytes)
            return flux2.Published.OK

    b = module(name).Builder(bytearray(8192), Loan())
    with pytest.raises(error):
        write(b)
    assert not b
    assert b.commit__() == flux2.Published.TOO_LARGE
    assert committed == []


def test_py_builder_writes_nowhere_after_commit(built):
    flux2 = pytest.importorskip("flux2")

    class Loan:
        def commit(self, nbytes):
            return flux2.Published.OK

    buf = bytearray(4096)
    b = module("Vec3").Builder(buf, Loan())
    b.x = 1.5
    assert b.commit__() == flux2.Published.OK
    sent = bytes(buf)
    b.x = 9.0
    assert bytes(buf) == sent
    assert not b
    assert b.commit__() == flux2.Published.BACKPRESSURE


def test_py_builder_rejects_a_fixed_length_mismatch(built):
    # message_shapes.md: a length that disagrees with the schema's T[N] is an explicit error at
    # the write too, so the bad frame is never produced.
    m = module("FixedNames")
    b = m.Builder(bytearray(8192))
    with pytest.raises(ValueError, match="string\\[2\\] takes exactly 2 strings, got 3"):
        b.names = ["x", "y", "z"]
    with pytest.raises(TypeError):
        b.names = "xy"
    # The count of a fixed record array is the schema's, so there is no argument to get wrong.
    with pytest.raises(TypeError):
        b.alloc__pair(3)
    assert len(b.alloc__pair()) == 2
    b.names = ["x", "y"]


@needs_cxx
def test_cpp_build_loans_from_a_publisher_and_commits(built, tmp_path):
    # Msg::build__(pub) is the C++ counterpart of the Python Msg.build__(pub): one line for what was
    # loan() plus a Builder over it. The Builder owns the slot, so the caller keeps one object.
    # A build() whose loan found no free slot is false and its commit() reports Backpressure --
    # a dropped frame, not a torn one.
    cpp_root, _ = built
    src, exe = tmp_path / "build_helper.cpp", tmp_path / "build_helper"
    src.write_text(
        '#include "test/flux2/vec3.hpp"\n'
        '#include "flux2/channel.hpp"\n'
        "#include <cstdio>\n"
        "int main()\n"
        "{\n"
        "  flux2::detail::Channel ch(4096, 2);\n"
        "  auto b = test::flux2_msg::Vec3::build__(ch);\n"
        '  std::printf("%d\\n", static_cast<bool>(b) ? 1 : 0);\n'
        "  b.set__x(1.5);\n"
        "  b.set__y(2.5);\n"
        "  b.set__z(3.5);\n"
        '  std::printf("%d\\n", b.commit__() == flux2::Published::Ok ? 1 : 0);\n'
        "  auto v = ch.take();\n"
        "  test::flux2_msg::Vec3::View view(v);\n"
        '  std::printf("%d\\n", (view.ok__() && view.x() == 1.5 && view.z() == 3.5) ? 1 : 0);\n'
        "  // Committed: the Builder no longer reaches the slot a subscriber holds.\n"
        "  b.set__x(9.0);\n"
        '  std::printf("%d\\n", (!b && view.x() == 1.5) ? 1 : 0);\n'
        "  // Every slot borrowed: build() comes back false and refuses to publish.\n"
        "  auto h1 = ch.loan();\n"
        "  auto h2 = ch.loan();\n"
        "  auto full = test::flux2_msg::Vec3::build__(ch);\n"
        '  std::printf("%d\\n", static_cast<bool>(full) ? 1 : 0);\n'
        '  std::printf("%d\\n", full.commit__() == flux2::Published::Backpressure ? 1 : 0);\n'
        "  return 0;\n"
        "}\n"
    )
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{cpp_root}",
            *CORE_INCLUDE_FLAGS,
            str(src),
            *CORE_LINK,
            "-pthread",
            "-o",
            str(exe),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    proc = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert proc.stdout == "1\n1\n1\n1\n0\n1\n"


@needs_cxx
def test_cpp_hands_over_string_bytes_that_are_not_utf8_unchanged(built, tmp_path):
    # The C++ half of test_a_string_that_is_not_utf8_reads_as_rclpy_reads_it: as rclcpp keeps a
    # std::string, the View returns the bytes as written and the frame stays ok__().
    cpp_root, _ = built
    src, exe = tmp_path / "not_utf8.cpp", tmp_path / "not_utf8"
    src.write_text(
        '#include "test/flux2/labeled.hpp"\n'
        "#include <cstdio>\n"
        "int main()\n"
        "{\n"
        "  unsigned char buf[4096];\n"
        "  test::flux2_msg::Labeled::Builder b(buf, sizeof buf);\n"
        '  b.set__label(std::string_view("ok \\xed\\x95", 5));\n'
        "  test::flux2_msg::Labeled::View v(buf, b.size__());\n"
        '  std::printf("%d %zu %02x\\n", v.ok__() ? 1 : 0, v.label().size(),\n'
        "              static_cast<unsigned char>(v.label()[4]));\n"
        "  return 0;\n"
        "}\n"
    )
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{cpp_root}",
            *CORE_INCLUDE_FLAGS,
            str(src),
            *CORE_LINK,
            "-pthread",
            "-o",
            str(exe),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    proc = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert proc.stdout == "1 5 95\n"


@needs_cxx
def test_cpp_builder_refuses_to_commit_after_a_failed_write(built, tmp_path):
    # The C++ half of test_py_builder_refuses_to_commit_after_a_failed_write: a write past the
    # slot latches ok__() false, and commit__() reports TooLarge and publishes nothing.
    cpp_root, _ = built
    src, exe = tmp_path / "too_large.cpp", tmp_path / "too_large"
    src.write_text(
        '#include "test/flux2/point_cloud.hpp"\n'
        '#include "flux2/channel.hpp"\n'
        "#include <cstdio>\n"
        "int main()\n"
        "{\n"
        "  flux2::detail::Channel ch(64, 2);\n"
        "  auto b = test::flux2_msg::PointCloud::build__(ch);\n"
        "  b.set__width(3);\n"
        "  b.alloc__x(1000);\n"
        '  std::printf("%d\\n", b.ok__() ? 1 : 0);\n'
        '  std::printf("%d\\n", b.commit__() == flux2::Published::TooLarge ? 1 : 0);\n'
        '  std::printf("%d\\n", static_cast<bool>(ch.take()) ? 1 : 0);\n'
        "  return 0;\n"
        "}\n"
    )
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{cpp_root}",
            *CORE_INCLUDE_FLAGS,
            str(src),
            *CORE_LINK,
            "-pthread",
            "-o",
            str(exe),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    proc = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert proc.stdout == "0\n1\n0\n"


@needs_cxx
def test_cpp_fixed_length_arrays_take_no_count(built, tmp_path):
    # N is the schema's, so alloc__x() takes no count and a wrong one is a compile error. It used
    # to be a runtime poison, and on a record array the empty handle it returned crashed on use.
    cpp_root, _ = built

    def build(body):
        src = tmp_path / "fixed.cpp"
        src.write_text(
            '#include "test/flux2/fixed_names.hpp"\n'
            "#include <cstdio>\n"
            "int main()\n"
            "{\n"
            "  unsigned char buf[8192];\n"
            "  test::flux2_msg::FixedNames::Builder b(buf, sizeof buf);\n"
            f"  {body}\n"
            "}\n"
        )
        exe = tmp_path / "fixed"
        return subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-I{cpp_root}",
                *CORE_INCLUDE_FLAGS,
                str(src),
                *CORE_LINK,
                "-pthread",
                "-o",
                str(exe),
            ],
            capture_output=True,
            text=True,
        ), exe

    for wrong in ("b.alloc__names(3);", "b.alloc__pair(3);"):
        compiled, _ = build(wrong)
        assert compiled.returncode != 0, f"{wrong} compiled"
    compiled, exe = build(
        "b.alloc__names(); auto a = b.alloc__pair(); "
        'std::printf("%zu %d\\n", a.size(), b.ok__() ? 1 : 0); return 0;'
    )
    assert compiled.returncode == 0, compiled.stderr
    proc = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
    assert proc.stdout == "2 1\n"


# --- names, bytes and packaging ---------------------------------------------------------------


def _write_msgs(root, msgs):
    os.makedirs(root, exist_ok=True)
    for name, body in msgs.items():
        with open(os.path.join(root, f"{name}.msg"), "w") as f:
            f.write(body)


def _module_name(name):
    import keyword

    s = snake(name)
    return f"{s}_" if keyword.iskeyword(s) else s


_ITEMS = "uint8 x\nItem[] items\n"
_NAMED = {
    # Names the generated module imports, a Python keyword as a module, and fields named like
    # what a class body calls.
    "Item": "uint8 v\n",
    "Reader": _ITEMS,
    "Writer": _ITEMS,
    "Published": _ITEMS,
    "NullWriter": _ITEMS,
    "ElemArray": _ITEMS,
    "Global": _ITEMS,
    "Fields": "float64 property\nfloat64 np\nfloat64 self\nfloat64 static_cast\n"
    "float64 linux\nfloat64 errno\nstring[] x\n",
}


@pytest.fixture(scope="module")
def named(tmp_path_factory):
    out = tmp_path_factory.mktemp("named")
    msg_dir = str(out / "names_pkg" / "msg")
    _write_msgs(msg_dir, _NAMED)
    cpp, py = str(out / "include"), str(out / "python")
    for name in _NAMED:
        generate(os.path.join(msg_dir, f"{name}.msg"), "names_pkg", cpp, py)
    sys.path.insert(0, py)
    yield cpp
    sys.path.remove(py)


@pytest.mark.parametrize("name", [n for n in _NAMED if n not in ("Item", "Fields")])
def test_a_message_may_be_named_like_what_its_module_uses(named, name):
    # The trailing `class <Msg>` rebound the module's Reader, Writer, Published or NullWriter, and
    # a keyword module (`global`) could not be imported at all.
    flux2 = pytest.importorskip("flux2")
    m = __import__(f"names_pkg_flux2.{_module_name(name)}", fromlist=[name])
    cls = getattr(m, name)
    buf = bytearray(256)
    b = cls.Builder(buf)
    b.x = 5
    b.alloc__items(2)[1].v = 7
    v = cls.View(memoryview(buf)[: b.size__])
    assert (v.x, len(v.items), v.items[1].v) == (5, 2, 7)

    class FullRing:
        def loan(self, *args, **kwargs):
            return None

    none = cls.build__(FullRing())
    assert not none
    assert none.commit__() == flux2.Published.BACKPRESSURE


def test_fields_named_like_python_builtins_keep_their_names(named):
    # A field `property` rebound property() in the class body and the module did not import; a
    # field `np` was renamed np_ in Python alone.
    import names_pkg_flux2.fields as m

    buf = bytearray(256)
    b = m.Builder(buf)
    b.property, b.np, b.self, b.static_cast, b.linux, b.errno = 1.5, 2.5, 3.5, 4.5, 5.5, 6.5
    b.x = ["a"]
    v = m.View(memoryview(buf)[: b.size__])
    assert (v.property, v.np, v.self, v.static_cast, v.linux, v.errno) == (
        1.5,
        2.5,
        3.5,
        4.5,
        5.5,
        6.5,
    )
    assert v.x == ["a"]


@needs_cxx
def test_named_messages_and_fields_compile(named, tmp_path):
    # The C++ half: a reserved word or a common macro as a field name gets a trailing '_'.
    src = tmp_path / "named.cpp"
    src.write_text(
        "#include <cassert>\n#include <cerrno>\n"
        + "".join(f'#include "names_pkg/flux2/{snake(n)}.hpp"\n' for n in _NAMED)
        + "int main() {\n"
        "  unsigned char buf[256] = {};\n"
        "  names_pkg::flux2_msg::Fields::View v(buf, sizeof buf);\n"
        "  return static_cast<int>(v.property() + v.np() + v.self() + v.static_cast_() +\n"
        "                          v.linux_() + v.errno_());\n"
        "}\n"
    )
    subprocess.run(
        ["g++", "-std=gnu++17", "-Wall", "-Wextra", "-Werror", "-fsyntax-only", f"-I{named}"]
        + CORE_INCLUDE_FLAGS
        + [str(src)],
        check=True,
        capture_output=True,
        text=True,
    )


def test_py_bridge_hands_rclpy_bytes_and_bools(tmp_path):
    # rclpy holds a byte as a one-byte bytes object and a bool[N] item as a Python bool; its C
    # converter asserts (SIGABRT) on an int or a numpy.bool. A stand-in class shows the types.
    import types

    msg_dir = str(tmp_path / "fb_pkg" / "msg")
    _write_msgs(msg_dir, {"FixedBool": "bool[3] flags\nbyte[2] raw\nbyte one\nbyte[] many\n"})
    py = str(tmp_path / "py")
    generate(os.path.join(msg_dir, "FixedBool.msg"), "fb_pkg", py=py)
    pkg, msgs = types.ModuleType("fb_pkg"), types.ModuleType("fb_pkg.msg")

    class FixedBool:
        pass

    msgs.FixedBool, pkg.msg = FixedBool, msgs
    saved = {k: sys.modules.get(k) for k in ("fb_pkg", "fb_pkg.msg")}
    sys.modules["fb_pkg"], sys.modules["fb_pkg.msg"] = pkg, msgs
    sys.path.insert(0, py)
    try:
        import fb_pkg_flux2.fixed_bool_ros as fb_ros

        m = FixedBool()
        m.flags, m.raw, m.one, m.many = [True, False, True], [b"\x01", b"3"], b"3", [b"\xff"]
        buf = bytearray(256)
        b = fb_ros.Builder(buf)
        fb_ros.msg_to_frame(m, b)
        back = fb_ros.frame_to_msg(fb_ros.View(memoryview(buf)[: b.size__]))
        assert back.flags == [True, False, True]
        assert all(type(x) is bool for x in back.flags)
        assert (back.raw, back.one, back.many) == ([b"\x01", b"3"], b"3", [b"\xff"])
    finally:
        sys.path.remove(py)
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


_BYTE_ROUND_TRIP = """
import sys
sys.path.insert(0, sys.argv[1])
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import Byte, ByteMultiArray
import diagnostic_msgs_flux2.diagnostic_status_ros as ds_ros
import std_msgs_flux2.byte_multi_array_ros as bma_ros
import std_msgs_flux2.byte_ros as b_ros

def rt(mod, msg):
    buf = bytearray(4096)
    b = mod.Builder(buf)
    mod.msg_to_frame(msg, b)
    back = mod.frame_to_msg(mod.View(memoryview(buf)[: b.size__]))
    assert deserialize_message(serialize_message(back), type(msg)) == msg
    return buf

assert rt(b_ros, Byte(data=b"3"))[0] == 0x33
rt(bma_ros, ByteMultiArray(data=[b"\\x01", b"\\xff"]))
rt(ds_ros, DiagnosticStatus(level=DiagnosticStatus.ERROR, name="n", message="m"))
print("ok")
"""


@needs_rclpy
def test_py_bridge_round_trips_byte_fields_through_rclpy(tmp_path):
    # Publishing a frame_to_msg result with a byte field aborted the process (rosidl asserts
    # PyBytes_Check), so it runs in a child.
    py = str(tmp_path / "py")
    for key in ("std_msgs/Byte", "std_msgs/ByteMultiArray", "diagnostic_msgs/DiagnosticStatus"):
        generate(installed(key), key.split("/")[0], py=py)
    for pkg in ("std_msgs_flux2", "diagnostic_msgs_flux2"):
        open(os.path.join(py, pkg, "__init__.py"), "a").close()
    proc = subprocess.run(
        [sys.executable, "-c", _BYTE_ROUND_TRIP, py], capture_output=True, text=True, timeout=60
    )
    assert (proc.returncode, proc.stdout) == (0, "ok\n"), proc.stderr


_PKG_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.0</version>
  <description>scratch</description>
  <maintainer email="none@example.com">none</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
"""

_PKG_CMAKE = """cmake_minimum_required(VERSION 3.16)
project({name} CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(flux2_gen REQUIRED)
find_package(flux2_cpp REQUIRED)
# Stands in for rosidl_generate_interfaces(), which leaves this list behind.
set(rosidl_generate_interfaces_ABS_IDL_FILES "${{CMAKE_CURRENT_BINARY_DIR}}/x/msg/WithRos.idl")
flux2_generate_adapters({msgs})
add_executable(use_adapter use_adapter.cpp)
target_link_libraries(use_adapter {name}_flux2_adapters flux2_cpp::flux2_cpp)
ament_package()
"""


def _scratch_package(root, name, msgs):
    src = root / name
    _write_msgs(str(src / "msg"), msgs)
    (src / "package.xml").write_text(_PKG_XML.format(name=name))
    listed = " ".join(f"msg/{m}.msg" for m in msgs)
    (src / "CMakeLists.txt").write_text(_PKG_CMAKE.format(name=name, msgs=listed))
    (src / "use_adapter.cpp").write_text(
        "".join(f'#include "{name}/flux2/{snake(m)}.hpp"\n' for m in msgs)
        + "int main() { return 0; }\n"
    )
    return src


def _cmake(*args):
    return subprocess.run(["cmake", *args], capture_output=True, text=True, timeout=300)


@pytest.mark.skipif(shutil.which("cmake") is None, reason="needs cmake")
def test_flux2_generate_adapters_in_a_package(tmp_path):
    # The package's own target includes its adapters (no include dir or build order before), a
    # message with no rosidl type gets no ROS bridge, a keyword module imports, and the package
    # exports each message class instead of star-importing their constants over each other.
    src = _scratch_package(
        tmp_path,
        "adapt_pkg",
        {
            "ModeA": "uint8 AUTO=1\nuint8 mode\n",
            "ModeB": "uint8 AUTO=7\nuint8 mode\n",
            "Global": "uint8 x\n",
            "WithRos": "uint8 x\n",
        },
    )
    build, inst = tmp_path / "build", tmp_path / "inst"
    for args in (
        ["-S", str(src), "-B", str(build), f"-DCMAKE_INSTALL_PREFIX={inst}"],
        ["--build", str(build), "-j", "4"],
        ["--install", str(build)],
    ):
        proc = _cmake(*args)
        assert proc.returncode == 0, proc.stdout + proc.stderr
    hpp = inst / "include" / "adapt_pkg" / "adapt_pkg" / "flux2"
    assert (hpp / "with_ros_ros.hpp").is_file()
    assert not (hpp / "mode_a_ros.hpp").exists()
    (site,) = (inst / "lib").glob("python3*/site-packages")
    assert not (site / "adapt_pkg_flux2" / "mode_a_ros.py").exists()
    probe = (
        "import adapt_pkg_flux2 as p\n"
        "print(p.ModeA.AUTO, p.ModeB.AUTO, hasattr(p, 'AUTO'), p.Global.TYPE_NAME__)\n"
    )
    proc = subprocess.run(
        [sys.executable, "-c", probe],
        capture_output=True,
        text=True,
        env={**os.environ, "PYTHONPATH": f"{site}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"},
    )
    assert proc.stdout == "1 7 False adapt_pkg/Global\n", proc.stderr


@pytest.mark.skipif(shutil.which("cmake") is None, reason="needs cmake")
def test_two_messages_writing_one_file_is_a_configure_error(tmp_path):
    # Foo's bridge foo_ros.* and FooRos's adapter foo_ros.* are one file: whichever ran last won.
    src = _scratch_package(tmp_path, "clash_pkg", {"Foo": "uint8 x\n", "FooRos": "uint8 y\n"})
    (src / "CMakeLists.txt").write_text(
        (src / "CMakeLists.txt").read_text().replace("WithRos.idl", "Foo.idl")
    )
    proc = _cmake("-S", str(src), "-B", str(tmp_path / "build"))
    assert proc.returncode != 0
    assert "foo_ros" in proc.stderr and "rename one" in proc.stderr, proc.stderr
