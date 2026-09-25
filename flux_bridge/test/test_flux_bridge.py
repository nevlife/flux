"""The decisions, without a live system.

Relaying itself is flux.Subscription plus rclpy publish, both covered elsewhere. What is pinned
here is which channels get a relay and which adapter a fingerprint selects.
"""

import os
import textwrap

import pytest

from flux_bridge import adapters
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


def test_discover_maps_fingerprints_to_installed_adapters(tmp_path, monkeypatch, capsys):
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
    # A module that fails to import is skipped, but not silently: an adapter built against an
    # older flux_gen would otherwise read as "no adapter installed".
    err = capsys.readouterr().err
    assert "demo_flux.broken" in err and "not an adapter" in err
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


def test_a_relay_skips_an_unreadable_frame_and_keeps_relaying():
    # A frame that disagrees with its schema is a wrong message, not the end of the channel. The
    # relay counts it and relays the frames after it; it used to die on the first one.
    import time
    import types

    import numpy as np

    import flux
    from flux_bridge.bridge import Relay
    from flux_gen.wire import WireError

    sent = []

    class Pub:
        def publish(self, msg):
            sent.append(msg)

    node = types.SimpleNamespace(
        create_publisher=lambda *a: Pub(), destroy_publisher=lambda p: None)

    def to_msg(frame):
        if frame[0] == 0xFF:
            raise WireError("frame region escapes the frame")
        return int(frame[0])

    adapter = types.SimpleNamespace(message=object, view=lambda f: np.asarray(f), frame_to_msg=to_msg)
    key, fp = "/pytest/bridge/unreadable", 0xB21D6E
    pub = flux.Publisher(key, fingerprint=fp, slot_size=4096, slot_count=4)
    relay = Relay(node, types.SimpleNamespace(key=key, fingerprint=fp), adapter, None)
    try:
        for handled, first_byte in enumerate((1, 0xFF, 2), start=1):
            pub.publish(np.full(8, first_byte, dtype=np.uint8))
            deadline = time.monotonic() + 5.0
            while len(sent) + relay.unreadable < handled and time.monotonic() < deadline:
                time.sleep(0.01)
        assert sent == [1, 2]
        assert relay.unreadable == 1
    finally:
        relay.stop()
