"""flux_cli: the formatting rules and the parser, without a live system.

The commands themselves are thin over flux.enumerate_topics() and flux.read_channel_stats(),
which flux_core tests already cover. What is worth pinning here is what the text says -- an
inexact key must not read as a real name, and a channel nobody holds must not read as a fault.
"""

import pytest

import flux_cli.main as main_module
from flux_cli.format import display_name, endpoint_summary, human_bytes, storage_name, table
from flux_cli.main import build_parser


class FakeEndpoint:
    def __init__(self, publisher, pid=1, label=""):
        self.publisher = publisher
        self.pid = pid
        self.label = label


class FakeTopic:
    def __init__(self, key, key_exact=True, endpoints=(), domain="0", fingerprint=1):
        self.key = key
        self.key_exact = key_exact
        self.fingerprint = fingerprint
        self.endpoints = list(endpoints)
        self.domain = domain
        self.signpost = "/flux.v7.s" + domain + "." + key.strip("/").replace("/", ".")


class FakeStats:
    def __init__(self, live=False):
        self.live = live


# The name flattens every non-alnum character, so an unmarked `.a.b` reads as a channel actually
# called that. Marking it is the whole point of key_exact.
def test_an_inexact_key_is_marked_as_coming_from_the_name():
    assert display_name(FakeTopic("/a/b")) == "/a/b"
    assert display_name(FakeTopic(".a.b", key_exact=False)) == ".a.b (from name)"


# The state a channel is looked up in is the one where its publisher is gone: the key comes back
# flattened, and matching on == would refuse the only name the caller has for it.
def test_a_dead_channel_resolves_by_the_name_its_publisher_used():
    topics = [FakeTopic(".cam.left", key_exact=False)]
    assert main_module._match(topics, "/cam/left") is topics[0]
    assert main_module._match(topics, ".cam.left") is topics[0]


# An exact key wins, so a channel actually called `.a.b` still resolves to itself rather than to
# whichever of the two the flattened pass happens to reach first.
def test_an_exact_key_wins_over_one_that_only_flattens_alike():
    flat = FakeTopic(".a.b", key_exact=False)
    real = FakeTopic("/a/b")
    assert main_module._match([flat, real], "/a/b") is real


def test_a_name_matching_two_channels_is_refused_rather_than_guessed(capsys):
    topics = [FakeTopic("/a/b"), FakeTopic(".a.b", key_exact=False)]
    with pytest.raises(SystemExit):
        main_module._match(topics, "/a.b")
    assert "2 channels" in capsys.readouterr().err


def test_endpoint_summary_counts_roles_and_says_nothing_when_empty():
    assert endpoint_summary(FakeTopic("/t")) == "-"
    assert endpoint_summary(
        FakeTopic("/t", endpoints=[FakeEndpoint(True), FakeEndpoint(False)])) == "1 pub, 1 sub"
    assert endpoint_summary(
        FakeTopic("/t", endpoints=[FakeEndpoint(True), FakeEndpoint(True)])) == "2 pub"
    assert endpoint_summary(
        FakeTopic("/t", endpoints=[FakeEndpoint(False)])) == "1 sub"


def test_table_sizes_columns_to_their_contents():
    lines = table([["a", "loooong"], ["bbbb", "x"]], ["H1", "H2"])
    assert lines[0] == "H1    H2"
    assert lines[1] == "----  -------"
    assert lines[2] == "a     loooong"
    assert lines[3] == "bbbb  x"


def test_table_does_not_leave_trailing_spaces():
    for line in table([["a", "b"]], ["H1", "H2"]):
        assert line == line.rstrip()


def test_human_bytes_uses_binary_units():
    assert human_bytes(512) == "512 B"
    assert human_bytes(1024) == "1 KiB"
    assert human_bytes(1 << 20) == "1 MiB"
    assert human_bytes(3 << 30) == "3.0 GiB"


# This command is often what you run when something is wrong, so an unknown storage kind prints
# its number rather than being guessed into a name.
def test_storage_name_reports_an_unknown_kind_as_itself():
    assert storage_name(0) == "host"
    assert storage_name(1) == "cuda-ipc"
    assert storage_name(2) == "dma-buf"
    assert storage_name(9) == "unknown(9)"


# --domain and --all-domains belong to the verbs, not the root: `flux topic list --all-domains` is
# where people type them, and a root-level flag would only be accepted before the subcommand.
@pytest.mark.parametrize("argv", [
    ["topic", "list", "--all-domains"],
    ["topic", "info", "/x", "--all-domains"],
    ["topic", "hz", "/x", "--all-domains"],
])
def test_domain_flags_are_accepted_after_the_verb(argv):
    args = build_parser().parse_args(argv)
    assert args.all_domains is True


def test_domain_defaults_to_unset_so_main_can_resolve_it():
    args = build_parser().parse_args(["topic", "list"])
    assert args.domain is None
    assert args.all_domains is False


def test_hz_window_defaults_to_one_second():
    assert build_parser().parse_args(["topic", "hz", "/x"]).window == 1.0
    assert build_parser().parse_args(["topic", "hz", "/x", "--window", "0.25"]).window == 0.25


def test_a_bare_invocation_is_refused_rather_than_doing_something():
    with pytest.raises(SystemExit):
        build_parser().parse_args([])
    with pytest.raises(SystemExit):
        build_parser().parse_args(["topic"])
    with pytest.raises(SystemExit):
        build_parser().parse_args(["domain"])


# `domain list` reports which domain the environment puts this process in, so an override would
# answer a question the caller did not ask. main() still fills the resolved value in.
def test_domain_list_takes_no_domain_override():
    args = build_parser().parse_args(["domain", "list"])
    assert getattr(args, "domain", None) is None
    with pytest.raises(SystemExit):
        build_parser().parse_args(["domain", "list", "--domain", "7"])


# The resolved domain is always a row, empty or not: it is the value the caller came to check, and
# dropping it when nothing is in it hides exactly the case worth seeing.
def test_domain_list_shows_the_resolved_domain_even_when_it_is_empty(monkeypatch, capsys):
    monkeypatch.setattr(main_module.flux, "enumerate_topics", lambda: [])
    args = build_parser().parse_args(["domain", "list"])
    args.domain = "7"
    assert main_module.cmd_domain_list(args) == 0
    out = capsys.readouterr().out
    assert "7 (here)" in out
    assert "more than one domain" not in out


# Two domains on one host is the state that makes a subscriber look idle while its publisher is
# running, so the command says so rather than leaving the reader to compare rows.
def test_domain_list_names_the_split_when_there_is_more_than_one(monkeypatch, capsys):
    topics = [FakeTopic("/a", domain="0"), FakeTopic("/b", domain="7")]
    monkeypatch.setattr(main_module.flux, "enumerate_topics", lambda: topics)
    monkeypatch.setattr(
        main_module.flux, "read_channel_stats", lambda _signpost: FakeStats(live=True))
    args = build_parser().parse_args(["domain", "list"])
    args.domain = "7"
    assert main_module.cmd_domain_list(args) == 0
    out = capsys.readouterr().out
    assert "7 (here)" in out
    assert "\n0 " in out and "(here)" not in out.split("\n0 ")[1].split("\n")[0]
    assert "more than one domain" in out
