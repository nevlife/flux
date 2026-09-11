"""The decisions, without a live system.

Relaying itself is flux.Subscription plus rclpy publish, both covered elsewhere. What is pinned
here is which channels get a relay, which adapter a fingerprint selects, and how `ros2 bag
record` arguments turn into a channel filter.
"""

import argparse
import os
import textwrap

import pytest

from flux_bridge import adapters
from flux_bridge.activate import selector
from flux_bridge.bridge import plan


def test_plan_starts_wanted_live_channels_and_stops_the_rest():
    live = {"/a", "/b", "/c"}
    wanted = {"/a", "/b", "/gone"}
    active = {"/b", "/c", "/dead"}
    assert plan(live, wanted, active) == (["/a"], ["/c", "/dead"])


def test_plan_is_idle_when_nothing_changes():
    assert plan({"/a"}, {"/a"}, {"/a"}) == ([], [])
    assert plan(set(), set(), set()) == ([], [])


def _fake_adapter_package(root, pkg, modules):
    pkg_dir = root / pkg
    pkg_dir.mkdir()
    for name, body in modules.items():
        (pkg_dir / f"{name}.py").write_text(textwrap.dedent(body))
    return str(root)


def test_discover_maps_fingerprints_to_installed_adapters(tmp_path, monkeypatch):
    path = _fake_adapter_package(tmp_path, "demo_flux", {
        "__init__": "",
        "cloud": 'TYPE_NAME__ = "demo/Cloud"\nFINGERPRINT__ = 0x10\nclass View: pass\n',
        "cloud_ros": 'MESSAGE = "ros-cloud"\ndef frame_to_msg(v): return v\n',
        "broken": "raise ImportError('not an adapter')\n",
        "plain": "X = 1\n",
    })
    monkeypatch.syspath_prepend(path)
    found = adapters.discover([path])
    assert list(found) == [0x10]
    adapter = found[0x10]
    assert adapter.type_name == "demo/Cloud"
    assert adapter.ros_type == "demo/msg/Cloud"
    assert adapter.module_name == "demo_flux.cloud"
    assert adapter.message == "ros-cloud"
    assert adapter.frame_to_msg("frame") == "frame"
    assert adapter.view.__name__ == "View"


def test_discover_ignores_directories_that_are_not_adapter_packages(tmp_path):
    (tmp_path / "other").mkdir()
    (tmp_path / "loose_flux.py").write_text("FINGERPRINT__ = 1\n")
    assert adapters.discover([str(tmp_path), os.path.join(str(tmp_path), "missing")]) == {}


def _record_args(**overrides):
    fields = {"all": False, "all_topics": False, "topics": None, "topics_positional": None,
              "regex": None, "exclude_regex": None, "exclude_topics": None}
    fields.update(overrides)
    return argparse.Namespace(**fields)


def test_selector_follows_record_arguments():
    assert selector(_record_args(all=True))("/cam/left")
    assert selector(_record_args(topics=["/cam/left"]))("/cam/left")
    assert not selector(_record_args(topics=["/cam/left"]))("/cam/right")
    assert selector(_record_args(topics_positional=["/cam/right"]))("/cam/right")
    assert selector(_record_args(regex="^/cam/"))("/cam/left")
    assert not selector(_record_args(regex="^/cam/"))("/lidar")
    assert not selector(_record_args(all=True, exclude_topics=["/cam/left"]))("/cam/left")
    assert not selector(_record_args(all=True, exclude_regex="left$"))("/cam/left")


# rosbag2 records nothing without a selection, but the relay is cheap and the bag cannot record
# what the bridge did not publish, so the tie goes to the relay.
def test_selector_with_no_criteria_selects_everything():
    assert selector(_record_args())("/anything")


@pytest.mark.parametrize("name", ["/cam/left", ".cam.left"])
def test_find_channel_accepts_the_flattened_spelling(monkeypatch, name):
    import flux

    from flux_bridge.activate import find_channel

    class Topic:
        def __init__(self, key, domain):
            self.key = key
            self.domain = domain

    monkeypatch.setattr(flux, "enumerate_topics", lambda: [Topic("/cam/left", "0"), Topic("/cam/left", "1")])
    monkeypatch.setattr(flux, "flatten_key", lambda k: k.replace("/", "."))
    assert find_channel(name, domain="1").domain == "1"
    assert find_channel("/nothing", domain="1") is None
