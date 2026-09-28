"""ros2 bag record_flux / play_flux: registered, and handing the C++ programs what they parsed."""

import argparse
from importlib.metadata import entry_points

import pytest

from flux_bag.verb import play_flux, record_flux


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
    assert {"record_flux", "play_flux"} <= {e.name for e in entry_points(group="ros2bag.verb")}


def test_record_flux_hands_the_recorder_what_it_parsed(monkeypatch):
    argv = ["-a", "/cam", "-o", "out", "--poll", "0.5", "--exclude-topics", "/x", "/y", "-e", "^/c"]
    assert handed_over(record_flux, record_flux.RecordFluxVerb(), argv, monkeypatch) == [
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
                "--exclude-topics",
                "/x",
                "/y",
            ],
        )
    ]


def test_record_flux_leaves_unset_options_to_the_recorder(monkeypatch):
    verb = record_flux.RecordFluxVerb()
    assert handed_over(record_flux, verb, ["/cam"], monkeypatch) == [("record", ["/cam"])]


def test_play_flux_hands_the_player_what_it_parsed(monkeypatch):
    verb = play_flux.PlayFluxVerb()
    assert handed_over(play_flux, verb, ["run1", "-r", "0.5"], monkeypatch) == [
        ("play", ["run1", "-r", "0.5"])
    ]
    assert handed_over(play_flux, verb, ["run1"], monkeypatch)[-1] == ("play", ["run1"])


@pytest.mark.parametrize(
    "verb, argv",
    [
        (record_flux.RecordFluxVerb(), ["--poll", "0"]),
        (record_flux.RecordFluxVerb(), ["--poll", "abc"]),
        (play_flux.PlayFluxVerb(), ["run1", "-r", "0"]),
        (play_flux.PlayFluxVerb(), []),
    ],
)
def test_a_bad_argument_is_refused_before_anything_runs(verb, argv):
    with pytest.raises(SystemExit):
        parse(verb, argv)
