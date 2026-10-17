import os
import subprocess
import sys

from ament_index_python.packages import get_package_prefix

MSG = os.path.join(os.path.dirname(__file__), "msg")

CHILD = """
import sys

import rclpy
from rclpy.node import Node

import flux2
from flux2_test_flux2.plain import Plain
from flux2_test_flux2.stamped import Stamped

rclpy.init()
node = Node("trace_node", namespace="/trace_ns")
ex = flux2.Executor()
for i, kind in enumerate((Stamped, Plain)):
    topic = f"{sys.argv[1]}_{i}"
    pub = flux2.Publisher(node, topic, fingerprint=kind.FINGERPRINT__, slot_size=64, slot_count=4)
    sub = flux2.Subscription(
        node, topic, fingerprint=kind.FINGERPRINT__, callback=lambda v: ex.stop()
    )
    ex.add(sub)
    b = kind.build__(pub)
    if kind is Stamped:
        b.set__header__stamp(7, 250)
    assert b.commit__() == flux2.Published.OK
    ex.spin()
"""


def test_endpoints_frames_and_the_gil_are_traced(tmp_path):
    gen = os.path.join(get_package_prefix("flux2_gen"), "lib", "flux2_gen", "flux2_gen")
    subprocess.run(
        [gen, "--pkg", "flux2_test", "--msg", os.path.join(MSG, "Stamped.msg")]
        + ["--msg", os.path.join(MSG, "Plain.msg"), "--out-py", str(tmp_path)],
        check=True,
    )
    log = tmp_path / "trace.log"
    env = dict(
        os.environ,
        LD_PRELOAD=os.environ["FLUX2_TRACE_LOG_LIB"],
        FLUX2_TRACE_LOG=str(log),
        PYTHONPATH=os.pathsep.join([str(tmp_path), os.environ.get("PYTHONPATH", "")]),
    )
    topic = f"/t{os.getpid()}_trace"
    subprocess.run([sys.executable, "-c", CHILD, topic], env=env, check=True, timeout=60)
    events = [line.split() for line in log.read_text().splitlines()]

    endpoints = [e for e in events if e[0] == "endpoint_init"]
    assert [(e[2], e[3], e[6], e[7]) for e in endpoints] == [
        ("/trace_ns/trace_node", f"{topic}_{i}", role, "1") for i in "01" for role in "01"
    ]
    stamps = [(e[0], int(e[3])) for e in events if e[0] in ("commit", "take")]
    assert stamps == [
        ("commit", 7 | 250 << 32),
        ("take", 7 | 250 << 32),
        ("commit", 0),
        ("take", 0),
    ]
    takes = [e[1:3] for e in events if e[0] == "take"]
    assert [e[1:3] for e in events if e[0] == "py_ready"] == takes
    gil = [e[1] for e in events if e[0] == "py_gil"]
    assert gil[:2] == ["0", "1"] and gil.count("0") == gil.count("1")
