"""docs/en/api.en.md section 2's Python examples, run against real generator output.

flux_gen's own tests cover the generated surface right up to where it meets a Publisher, and
stop there: `build()` loans a slot and `commit()` publishes it, so neither exists without one.
Generating the document's adapter here and running its publish region is what puts those two
under a test instead of under review.
"""

import ast
import importlib.util
import pathlib
import sys

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
cli = pytest.importorskip("flux_gen.cli")

import flux  # noqa: E402
import flux.ros  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / "flux_py" / "examples" / "doc_api_adapter.py"
CLOUD_MSG = ROOT / "docs" / "examples" / "Cloud.msg"


@pytest.fixture(scope="module")
def example(tmp_path_factory):
    """The example module, with the adapters it imports generated onto sys.path first."""
    from flux_gen import load_dir

    out = str(tmp_path_factory.mktemp("adapters"))
    reg = cli.ament_registry()
    load_dir(str(CLOUD_MSG.parent), package="my_pkg", into=reg)
    cli.generate(str(CLOUD_MSG), "my_pkg", None, out, reg)

    image = reg.get("sensor_msgs/Image")
    if image is None:
        pytest.skip("flux-cap:ros-msgs sensor_msgs is not on AMENT_PREFIX_PATH")
    cli.generate(image.path, "sensor_msgs", None, out, reg)

    sys.path.insert(0, out)
    try:
        spec = importlib.util.spec_from_file_location("flux_doc_api_adapter", EXAMPLE)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        yield mod
    finally:
        sys.path.remove(out)


@pytest.fixture
def node():
    rclpy.init()
    n = rclpy.create_node("flux_doc_adapter_test")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_publish_region_loans_and_commits(example, node):
    cloud, n = example.Cloud, 8
    xs = np.arange(n, dtype=np.float32)

    pub, b = example.doc_publish(node, n, xs)
    assert b is not None, "build() found no free slot on an untouched channel"

    # A subscriber positions its cursor at the newest ticket, so it only sees what is published
    # after it attaches -- hence a second frame, written through the same generated names.
    sub = flux.ros.Subscription(node, "cloud", fingerprint=cloud.FINGERPRINT__)
    again = cloud.build__(pub)
    again.alloc__x(n)[:] = xs
    again.width = n
    again.label = "front"
    assert again.commit__() == flux.Published.Ok

    f = sub.take()
    assert f is not None, "commit() reported success but nothing arrived"
    c = cloud.View(f)
    assert list(c.x) == list(xs)
    assert c.width == n
    assert c.label == "front"


def _paths(tree, root):
    for node_ in ast.walk(tree):
        if not isinstance(node_, ast.Attribute):
            continue
        parts = []
        cur = node_
        while isinstance(cur, ast.Attribute):
            parts.append(cur.attr)
            cur = cur.value
        if isinstance(cur, ast.Name) and cur.id == root:
            yield (root, *reversed(parts))


@pytest.mark.parametrize("root", ["Cloud", "Image"])
def test_example_adapter_names_resolve(example, root):
    """The regions this file never calls still may not name a generated attribute that is gone."""
    paths = set(_paths(ast.parse(EXAMPLE.read_text(), str(EXAMPLE)), root))
    assert paths, f"the example uses no {root} attribute at all"
    for path in sorted(paths):
        obj = getattr(example, root)
        for part in path[1:]:
            dotted = ".".join(path)
            assert hasattr(obj, part), f"{dotted} does not exist"
            obj = getattr(obj, part)
