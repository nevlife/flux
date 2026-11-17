"""ros2 bag record_flux / play_flux: registered, and handing the C++ programs the options ros2 bag
record / play builds from the same arguments."""

import argparse
from importlib.metadata import entry_points

import pytest
import rosbag2_py
import yaml

from flux_bag.verb import play_flux, record_flux, rosbag2_options


def parse(verb, argv):
    parser = argparse.ArgumentParser()
    verb.add_arguments(parser, "ros2 bag")
    return parser.parse_args([str(a) for a in argv])


def record_options(argv):
    built = record_flux.program_options(parse(record_flux.RecordFluxVerb(), argv))
    assert not isinstance(built, str), built
    text, ros_args = built
    return yaml.safe_load(text), ros_args


def play_options(argv):
    built = play_flux.program_options(parse(play_flux.PlayFluxVerb(), argv))
    assert not isinstance(built, str), built
    text, ros_args = built
    return yaml.safe_load(text), ros_args


def fields(options):
    return {a for a in dir(options) if not a.startswith("_")}


def test_both_verbs_are_registered_under_ros2_bag():
    assert {"record_flux", "play_flux"} <= {e.name for e in entry_points(group="ros2bag.verb")}


# A field rosbag2 adds is a test failure here, not an option dropped on the way to the program.
def test_every_rosbag2_option_field_is_handed_over():
    assert set(rosbag2_options.STORAGE_KEYS) == fields(rosbag2_py.StorageOptions(uri="x"))
    record = set(rosbag2_options.RECORD_KEYS.values())
    record |= {"topic_polling_interval", "topic_qos_profile_overrides"}
    assert record == fields(rosbag2_py.RecordOptions())
    play = set(rosbag2_options.PLAY_KEYS.values())
    play |= {"topic_qos_profile_overrides", "delay", "playback_duration", "start_offset"}
    play |= {"playback_until_timestamp", "wait_acked_timeout", "service_requests_source"}
    assert play == fields(rosbag2_py.PlayOptions())


def test_record_flux_hands_over_what_ros2_bag_record_builds(tmp_path):
    overrides = tmp_path / "qos.yaml"
    overrides.write_text("/imu:\n  reliability: best_effort\n  depth: 5\n")
    out = tmp_path / "out"
    o, ros_args = record_options(
        ["-a", "-o", out, "-b", 1000, "-d", 5, "--max-cache-size", 0, "--compression-mode", "file"]
        + ["--compression-format", "zstd", "--qos-profile-overrides-path", overrides, "-x", "^/c"]
        + ["--exclude-topics", "/x", "/y", "--no-discovery", "--custom-data", "k=v"]
        + ["--node-name", "rec", "--poll", 0.5, "--log-level", "warn"]
    )
    assert ros_args == ["--ros-args", "--log-level", "warn"]
    assert (o["node_name"], o["poll"]) == ("rec", 0.5)
    s = o["storage"]
    assert (s["uri"], s["max_bagfile_size"], s["max_bagfile_duration"]) == (str(out), 1000, 5)
    assert (s["max_cache_size"], s["custom_data"], s["storage_id"]) == (0, {"k": "v"}, "mcap")
    r = o["record"]
    assert r["all_topics"] and r["all_services"] and r["is_discovery_disabled"]
    assert (r["compression_mode"], r["compression_format"]) == ("FILE", "zstd")
    assert (r["exclude_regex"], r["exclude_topics"]) == ("^/c", ["/x", "/y"])
    assert r["topic_polling_interval"] == {"milliseconds": 100}
    qos = r["topic_qos_profile_overrides"]["/imu"]
    assert (qos["reliability"], qos["depth"], qos["history"]) == ("best_effort", 5, "keep_last")
    assert qos["deadline"] == {"sec": 0, "nsec": 0}


def test_record_flux_takes_ros2_bag_record_defaults(tmp_path):
    o, ros_args = record_options(["/cam", "-o", tmp_path / "out"])
    assert ros_args == ["--ros-args", "--log-level", "info"]
    assert o["record"]["topics"] == ["/cam"] and not o["record"]["all_topics"]
    assert o["storage"]["max_cache_size"] == 100 * 1024 * 1024
    assert (o["node_name"], o["poll"]) == ("rosbag2_recorder", 0.1)


def test_play_flux_hands_over_what_ros2_bag_play_builds(tmp_path):
    o, ros_args = play_options(
        [tmp_path, "-r", 0.5, "-l", "--remap", "/a:=/b", "--start-offset", 1.5, "--clock"]
        + ["--playback-duration", 2, "--playback-until-nsec", 7, "-d", 0.25, "--topics", "/a"]
        + ["--exclude-topics", "/x", "-x", "/y.*", "-p", "--disable-keyboard-controls"]
        + ["--wait-for-all-acked", 3000, "--publish-service-requests"]
        + ["--service-requests-source", "client_introspection"]
    )
    assert ros_args == ["--ros-args", "--log-level", "info"]
    assert o["storage"]["uri"] == str(tmp_path)
    p = o["play"]
    assert (p["rate"], p["loop"], p["start_paused"], p["disable_keyboard_controls"]) == (
        0.5,
        True,
        True,
        True,
    )
    assert p["topic_remapping_options"] == ["--ros-args", "--remap", "/a:=/b"]
    assert p["start_offset"] == {"sec": 1, "nsec": 500_000_000}
    assert p["delay"] == {"sec": 0, "nsec": 250_000_000}
    assert p["playback_duration"] == {"sec": 2, "nsec": 0}
    assert p["playback_until_timestamp"] == {"sec": 0, "nsec": 7}
    assert (p["topics_to_filter"], p["exclude_regex_to_filter"]) == (["/a"], "/y.*")
    # --clock leaves the bag's own /clock out, as in ros2 bag play.
    assert (p["clock_publish_frequency"], p["exclude_topics"]) == (40.0, ["/x", "/clock"])
    # Milliseconds in a Duration's place: the C++ decoder keeps the count, the player reads ms.
    assert p["wait_acked_timeout"] == {"sec": 0, "nsec": 3000}
    assert p["publish_service_requests"] and p["service_requests_source"] == 1


def test_play_flux_takes_ros2_bag_play_defaults(tmp_path):
    p = play_options([tmp_path])[0]["play"]
    assert (p["rate"], p["loop"], p["start_offset"], p["read_ahead_queue_size"]) == (
        1.0,
        False,
        {"sec": 0, "nsec": 0},
        1000,
    )
    assert p["playback_duration"] == {"sec": -1, "nsec": 0}
    assert (
        p["playback_until_timestamp"] == p["wait_acked_timeout"] == {"sec": -1, "nsec": 999_999_999}
    )


def test_a_negative_nanosecond_count_keeps_its_value():
    for ns in (-1, -1_500_000_000, 0, 7, 2_000_000_001):
        d = rosbag2_options.duration(ns)
        assert d["sec"] * 1_000_000_000 + d["nsec"] == ns and 0 <= d["nsec"] < 1_000_000_000
    assert rosbag2_options.seconds(-1.5) == {"sec": -2, "nsec": 500_000_000}


@pytest.mark.parametrize(
    "verb, argv",
    [
        (record_flux.RecordFluxVerb(), ["-a", "--poll", "0"]),
        (record_flux.RecordFluxVerb(), ["-a", "--poll", "abc"]),
        (record_flux.RecordFluxVerb(), ["-a", "--poll", "nan"]),
        (record_flux.RecordFluxVerb(), ["-a", "--poll", "inf"]),
        (record_flux.RecordFluxVerb(), ["-a", "-b", "1k"]),
        (play_flux.PlayFluxVerb(), [".", "-r", "0"]),
        (play_flux.PlayFluxVerb(), [".", "-r", "2abc"]),
    ],
)
def test_a_bad_argument_is_refused_before_anything_runs(verb, argv):
    with pytest.raises(SystemExit):
        parse(verb, argv)


@pytest.mark.parametrize(
    "argv, why",
    [
        ([], "Need to specify at least one option"),
        (["-a", "--compression-format", "zstd"], "without a compression mode"),
    ],
)
def test_record_flux_refuses_what_ros2_bag_record_refuses(tmp_path, argv, why):
    built = record_flux.program_options(
        parse(record_flux.RecordFluxVerb(), [*argv, "-o", tmp_path / "out"])
    )
    assert isinstance(built, str) and why in built


def test_record_flux_refuses_a_bag_that_is_there(tmp_path):
    built = record_flux.program_options(parse(record_flux.RecordFluxVerb(), ["-a", "-o", tmp_path]))
    assert isinstance(built, str) and "already exists" in built


@pytest.mark.parametrize(
    "argv, why",
    [
        (["-r", "nan"], "Value must be finite"),
        (["-r", "inf"], "Value must be finite"),
        (["-i", ".", "-i", "."], "plays one bag"),
        ([], "no input bags"),
    ],
)
def test_play_flux_refuses(tmp_path, monkeypatch, argv, why):
    monkeypatch.chdir(tmp_path)
    built = play_flux.program_options(parse(play_flux.PlayFluxVerb(), argv))
    assert isinstance(built, str) and why in built
