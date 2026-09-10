""".msg parser robustness: malformed input must be rejected, never guessed at.

The parser decides the flat wire layout and the fingerprint that both the C++ and the Python
adapter are generated from. A field silently dropped or an array size silently misread does not
fail loudly -- it yields a wrong layout that both ends then agree on, so nothing catches it.
These tests pin the rejection behaviour, and the corpus sweep pins the other side: hardening
must not start rejecting real messages.
"""

import glob
import os

import pytest

from flux_gen import ArrayKind, Field, Message, Registry, load_dir, parse_msg, resolve
from flux_gen.parse import MAX_FIXED_ARRAY, ParseError

ROS_MSG_GLOB = "/opt/ros/*/share/*/msg/*.msg"
ROS_MSG_DIR_GLOB = "/opt/ros/*/share/*/msg"


def one(text):
    return parse_msg(text, "T", path="t.msg")


@pytest.mark.parametrize(
    "text,needle",
    [
        ("float64[abc] x", "bad fixed array size"),
        ("float64[-1] x", "bad fixed array size"),
        ("float64[1_0] x", "bad fixed array size"),   # python int() accepts underscores; we must not
        ("float64[٣] x", "bad fixed array size"),     # str.isdigit() is true, int() returns 3
        ("float64[²] x", "bad fixed array size"),     # str.isdigit() is true, int() raises ValueError
        ("float64[ 3] x", "unterminated array"),      # whitespace splits the token
        ("float64[3 x", "unterminated array"),
        ("float64[3", "unterminated array"),
        ("float64[0] x", "zero-length fixed array"),
        (f"float64[{MAX_FIXED_ARRAY + 1}] x", "exceeds"),
        ("float64[99999999999999999999] x", "exceeds"),
        ("[] x", "no element type"),
        ("float64", "no field name"),
        ("int32[] 9bad", "malformed field name"),
        ("int32<=4 x", "'<=' bound is only valid"),
        ("float64 x\nfloat64 x", "duplicate field name"),
    ],
)
def test_malformed_is_rejected(text, needle):
    with pytest.raises(ParseError) as e:
        one(text)
    assert needle in str(e.value)


def test_error_names_file_and_line():
    with pytest.raises(ParseError) as e:
        one("float64 ok\n# a comment\nfloat64[bad] x")
    msg = str(e.value)
    assert "t.msg:3" in msg


@pytest.mark.parametrize(
    "text,expect",
    [
        ("float64 x", ("float64", ArrayKind.SCALAR, 0)),
        ("float64[] x", ("float64", ArrayKind.DYNAMIC, 0)),
        ("float64[36] x", ("float64", ArrayKind.FIXED, 36)),
        ("float64[<=4] x", ("float64", ArrayKind.BOUNDED, 4)),
        ("string<=10 x", ("string", ArrayKind.SCALAR, 0)),
        ("string<=10[] x", ("string", ArrayKind.DYNAMIC, 0)),
        ("geometry_msgs/Point[] x", ("geometry_msgs/Point", ArrayKind.DYNAMIC, 0)),
    ],
)
def test_valid_type_forms(text, expect):
    f = one(text).fields[0]
    assert (f.type_name, f.kind, f.size) == expect


@pytest.mark.parametrize("text", ["uint16 ERR=1", "uint16 ERR = 1", "string NAME='x'"])
def test_constants_are_ignored(text):
    assert one(text).fields == []


@pytest.mark.parametrize(
    "text,needle",
    [
        ("uint16 err=1", "malformed constant name"),
        ("uint16 Err=1", "malformed constant name"),
        ("uint16 E__F=1", "malformed constant name"),
        ("uint16 E_=1", "malformed constant name"),
        ("uint16 A=1\nuint16 A=2", "duplicate constant name"),
    ],
)
def test_bad_constant_names_are_rejected(text, needle):
    # rosidl requires UPPER_CASE, so being laxer would accept a .msg that plain ROS rejects.
    # The doubled and trailing underscore rows carry more than style: they are what keeps the
    # adapter's own FINGERPRINT__ and TYPE_NAME__ unreachable from a .msg.
    with pytest.raises(ParseError) as e:
        one(text)
    assert needle in str(e.value)


def test_a_constant_may_not_shadow_its_own_message():
    # An all-caps message name is legal, and its class sits in the same domains as the constants.
    with pytest.raises(ParseError) as e:
        parse_msg("uint16 IMU=1", "IMU", path="imu.msg")
    assert "collides" in str(e.value)


def test_default_value_containing_equals_is_still_a_field():
    # The bug this guards: deciding "constant" by "is there an '=' after the type" drops this
    # field, because its default value contains one. Dropping a field changes the wire layout
    # and the fingerprint, and both generated ends would agree on the wrong answer.
    m = one('string label "a=b"\nfloat64[] values')
    assert [f.name for f in m.fields] == ["label", "values"]


def test_hash_inside_a_quoted_default_is_not_a_comment():
    m = one('string tag "has # hash"\nint32 n')
    assert [f.name for f in m.fields] == ["tag", "n"]


def test_escaped_quote_in_default_does_not_end_the_string():
    m = one('string tag "a\\" # b"\nint32 n')
    assert [f.name for f in m.fields] == ["tag", "n"]


def test_comments_and_blank_lines_are_skipped():
    m = one("# leading\n\nfloat64 x  # trailing\n\n# tail\n")
    assert [f.name for f in m.fields] == ["x"]


def test_empty_message_is_legal():
    assert one("# nothing here\n").fields == []


def two_package_registry():
    """Two packages defining the same message name with different layouts -- the shape that made
    the old basename fallback return the wrong one."""
    reg = Registry()
    reg.add(Message("GoalStatus", [Field("status", "int8", ArrayKind.SCALAR)], package="action_msgs"))
    reg.add(Message("GoalStatus", [Field("text", "string", ArrayKind.SCALAR)], package="actionlib_msgs"))
    return reg


def test_qualified_lookup_returns_that_package():
    reg = two_package_registry()
    assert resolve(reg, "action_msgs/GoalStatus").fields[0].name == "status"
    assert resolve(reg, "actionlib_msgs/GoalStatus").fields[0].name == "text"


def test_qualified_lookup_never_falls_back_to_the_basename():
    # The bug this guards: an unresolvable 'pkg/Msg' fell back to any message with that basename,
    # so a missing package silently produced another package's layout -- and therefore another
    # package's fingerprint, which both generated ends then agree on.
    #
    # The unique-basename case is the one that bites: the bare key is present and populated, so a
    # fallback finds it and returns a confidently wrong answer. (With an ambiguous basename the
    # bare key is withdrawn, and even a fallback would miss.)
    reg = Registry()
    reg.add(Message("Odometry", [Field("x", "float64", ArrayKind.SCALAR)], package="nav_msgs"))
    assert resolve(reg, "nav_msgs/Odometry") is not None
    assert resolve(reg, "foo_msgs/Odometry") is None

    reg = two_package_registry()
    assert resolve(reg, "nav_msgs/GoalStatus") is None


def test_bare_name_domains_to_the_referring_package():
    reg = two_package_registry()
    assert resolve(reg, "GoalStatus", package="action_msgs").fields[0].name == "status"
    assert resolve(reg, "GoalStatus", package="actionlib_msgs").fields[0].name == "text"


def test_ambiguous_bare_name_is_an_error():
    reg = two_package_registry()
    with pytest.raises(ParseError) as e:
        resolve(reg, "GoalStatus")
    assert "ambiguous" in str(e.value)
    assert "action_msgs/GoalStatus" in str(e.value)
    assert "actionlib_msgs/GoalStatus" in str(e.value)


def test_unique_bare_name_still_resolves():
    reg = two_package_registry()
    reg.add(Message("Odometry", [Field("x", "float64", ArrayKind.SCALAR)], package="nav_msgs"))
    assert resolve(reg, "Odometry") is not None
    assert resolve(reg, "Odometry", package="nav_msgs") is not None


def test_duplicate_registration_is_an_error():
    reg = two_package_registry()
    with pytest.raises(ParseError) as e:
        reg.add(Message("GoalStatus", [], package="action_msgs"))
    assert "duplicate message" in str(e.value)


@pytest.mark.skipif(not glob.glob(ROS_MSG_DIR_GLOB), reason="no installed ROS messages to sweep")
def test_ros_corpus_keys_every_message_under_its_package():
    # load_dir used to key on the file stem alone, so a whole-distro registry silently lost one
    # of every duplicated basename -- 35 of them in ROS jazzy -- and no qualified key existed for
    # resolve to find. Every file must survive under its own 'pkg/Msg'.
    reg = Registry()
    dirs = sorted(glob.glob(ROS_MSG_DIR_GLOB))
    for d in dirs:
        load_dir(d, into=reg)
    files = sorted(glob.glob(ROS_MSG_GLOB))
    qualified = [k for k in reg if "/" in k]
    assert len(qualified) == len(files)
    for path in files:
        parts = path.split("/")
        key = parts[parts.index("share") + 1] + "/" + os.path.basename(path)[:-4]
        assert resolve(reg, key) is not None, key


@pytest.mark.skipif(not glob.glob(ROS_MSG_GLOB), reason="no installed ROS messages to sweep")
def test_installed_ros_messages_all_parse():
    # The other half of hardening: rejecting malformed input is only correct if no real message
    # is caught by it. Every .msg shipped with the installed ROS distro must still parse.
    files = sorted(glob.glob(ROS_MSG_GLOB))
    assert len(files) > 100, "expected a substantial corpus to sweep"
    for path in files:
        with open(path, encoding="utf-8") as f:
            parse_msg(f.read(), os.path.basename(path)[:-4], path=path)


@pytest.mark.parametrize("line", [
    "float64 _leading",   # the Python adapter names its own slots _r/_at/_w
    "float64 Upper",
    "float64 double__underscore",
    "float64 trailing_",
])
def test_field_names_follow_the_ros_rule(line):
    # ROS 2 requires lowercase, letter-initial, single underscores between alphanumerics. Being
    # laxer than ROS here would accept a .msg that rosidl rejects, so the flux path and the plain
    # ROS path would disagree about whether the message is even valid.
    with pytest.raises(ParseError):
        parse_msg(line + "\n", "M", package="p")


@pytest.mark.parametrize("line", ["float64 a_b", "float64 x1", "float64 point_cloud2"])
def test_ordinary_field_names_are_accepted(line):
    assert parse_msg(line + "\n", "M", package="p").fields


def test_string_constants_follow_rosidl_quoting():
    # Same .msg, same value on both sides: matching quotes are stripped and a comment ends the
    # value. A '#' inside quotes stays text (quote-aware, unlike rosidl's blind split).
    m = one(
        "string A='hello'   # note\n"
        'string B="hi there"\n'
        "string C=bare value   # note\n"
        "string D='a#b'\n"
        "uint8 x\n")
    assert {c.name: c.value for c in m.constants} == {
        "A": "hello", "B": "hi there", "C": "bare value", "D": "a#b"}


def test_a_duplicate_key_is_still_an_error():
    reg = Registry()
    reg.add(parse_msg("uint8 x\n", "M", package="p"))
    with pytest.raises(ParseError):
        reg.add(parse_msg("uint8 y\n", "M", package="p"))


def test_if_absent_keeps_the_message_already_registered():
    # The CLI loads the .msg files it was handed before merging the installed ones, so generating
    # for a package that is also installed reads the source it was pointed at rather than failing
    # as a duplicate or silently taking the installed copy.
    reg = Registry()
    reg.add(parse_msg("uint8 mine\n", "M", package="p"))
    reg.add(parse_msg("uint8 theirs\n", "M", package="p"), if_absent=True)
    assert [f.name for f in reg["p/M"].fields] == ["mine"]
