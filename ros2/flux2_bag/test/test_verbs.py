"""ros2 bag record_flux2 / play_flux2: registered, and handing the C++ programs what they parsed."""

import argparse
from importlib.metadata import entry_points

import pytest

from flux2_bag.verb import play_flux2, record_flux2


def parse(verb, argv):
    parser = argparse.ArgumentParser()
    verb.add_arguments(parser, "ros2 bag")
    return parser.parse_args(argv)


def handed_over(module, verb, argv, monkeypatch):
    seen = []
    monkeypatch.setattr(module, "run", lambda executable, a: seen.append((executable, a)))
    verb.main(args=parse(verb, argv))
    return seen


def test_both_verbs_are_registered_under_ros2_bag():
    assert {"record_flux2", "play_flux2"} <= {e.name for e in entry_points(group="ros2bag.verb")}


def test_record_flux2_hands_the_recorder_what_it_parsed(monkeypatch):
    argv = ["-a", "/cam", "-o", "out", "--poll", "0.5", "--exclude-topics", "/x", "/y", "-e", "^/c"]
    argv += ["--max-cache-size", "1000"]
    assert handed_over(record_flux2, record_flux2.RecordFlux2Verb(), argv, monkeypatch) == [
        (
            "record",
            [
                "/cam",
                "-a",
                "-e",
                "^/c",
                "-o",
                "out",
                "--poll",
                "0.5",
                "--max-cache-size",
                "1000",
                "--exclude-topics",
                "/x",
                "/y",
            ],
        )
    ]


def test_record_flux2_leaves_unset_options_to_the_recorder(monkeypatch):
    verb = record_flux2.RecordFlux2Verb()
    assert handed_over(record_flux2, verb, ["/cam"], monkeypatch) == [("record", ["/cam"])]


def test_play_flux2_hands_the_player_what_it_parsed(monkeypatch):
    verb = play_flux2.PlayFlux2Verb()
    assert handed_over(play_flux2, verb, ["run1", "-r", "0.5"], monkeypatch) == [
        ("play", ["run1", "-r", "0.5"])
    ]
    assert handed_over(play_flux2, verb, ["run1"], monkeypatch)[-1] == ("play", ["run1"])


@pytest.mark.parametrize(
    "verb, argv",
    [
        (record_flux2.RecordFlux2Verb(), ["--poll", "0"]),
        (record_flux2.RecordFlux2Verb(), ["--poll", "abc"]),
        (play_flux2.PlayFlux2Verb(), ["run1", "-r", "0"]),
        (play_flux2.PlayFlux2Verb(), []),
    ],
)
def test_a_bad_argument_is_refused_before_anything_runs(verb, argv):
    with pytest.raises(SystemExit):
        parse(verb, argv)
