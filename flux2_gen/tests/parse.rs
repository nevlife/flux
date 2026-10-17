//! .msg parser robustness: malformed input must be rejected, never guessed at.
//!
//! A field silently dropped or an array size silently misread yields a wrong layout that both
//! generated ends then agree on, so nothing downstream catches it. The corpus sweep pins the
//! other side: hardening must not start rejecting real messages.

mod common;

use std::fs;
use std::path::Path;

use common::{field, message, ros_corpus, ros_msg_dirs, ros_msg_files};
use flux2_gen::model::{ArrayKind, ConstValue, Message};
use flux2_gen::parse::{ParseError, Registry, parse_msg, resolve};

fn one(text: &str) -> Result<Message, ParseError> {
    parse_msg(text, "T", Some(Path::new("t.msg")), None)
}

fn field_names(m: &Message) -> Vec<&str> {
    m.fields.iter().map(|f| f.name.as_str()).collect()
}

fn assert_rejected(text: &str, needle: &str) {
    let err = one(text).unwrap_err().to_string();
    assert!(err.contains(needle), "{text:?}: '{err}' lacks '{needle}'");
}

#[test]
fn malformed_is_rejected() {
    let cases = [
        ("float64[abc] x", "bad fixed array size"),
        ("float64[-1] x", "bad fixed array size"),
        ("float64[1_0] x", "bad fixed array size"),
        ("float64[\u{0663}] x", "bad fixed array size"),
        ("float64[\u{00b2}] x", "bad fixed array size"),
        // Whitespace splits the token.
        ("float64[ 3] x", "unterminated array"),
        ("float64[3 x", "unterminated array"),
        ("float64[3", "unterminated array"),
        ("float64[0] x", "zero-length fixed array"),
        ("[] x", "no element type"),
        ("float64", "no field name"),
        ("int32[] 9bad", "malformed field name"),
        ("int32<=4 x", "'<=' bound is only valid"),
        ("float64 x\nfloat64 x", "duplicate field name"),
    ];
    for (text, needle) in cases {
        assert_rejected(text, needle);
    }
}

#[test]
fn error_names_file_and_line() {
    let err = one("float64 ok\n# a comment\nfloat64[bad] x")
        .unwrap_err()
        .to_string();
    assert!(err.contains("t.msg:3"), "{err}");
}

#[test]
fn valid_type_forms() {
    let cases = [
        ("float64 x", ("float64", ArrayKind::Scalar, 0)),
        ("float64[] x", ("float64", ArrayKind::Dynamic, 0)),
        ("float64[36] x", ("float64", ArrayKind::Fixed, 36)),
        ("float64[<=4] x", ("float64", ArrayKind::Bounded, 4)),
        ("string<=10 x", ("string", ArrayKind::Scalar, 0)),
        ("string<=10[] x", ("string", ArrayKind::Dynamic, 0)),
        (
            "geometry_msgs/Point[] x",
            ("geometry_msgs/Point", ArrayKind::Dynamic, 0),
        ),
    ];
    for (text, expect) in cases {
        let m = one(text).unwrap();
        let f = &m.fields[0];
        assert_eq!((f.type_name.as_str(), f.kind, f.size), expect, "{text}");
    }
}

#[test]
fn constants_are_ignored() {
    for text in ["uint16 ERR=1", "uint16 ERR = 1", "string NAME='x'"] {
        assert!(one(text).unwrap().fields.is_empty(), "{text}");
    }
}

#[test]
fn bad_constant_names_are_rejected() {
    // rosidl requires UPPER_CASE, so being laxer would accept a .msg that plain ROS rejects.
    // The doubled and trailing underscore rows keep the adapter's own FINGERPRINT__ and
    // TYPE_NAME__ unreachable from a .msg.
    let cases = [
        ("uint16 err=1", "malformed constant name"),
        ("uint16 Err=1", "malformed constant name"),
        ("uint16 E__F=1", "malformed constant name"),
        ("uint16 E_=1", "malformed constant name"),
        ("uint16 A=1\nuint16 A=2", "duplicate constant name"),
    ];
    for (text, needle) in cases {
        assert_rejected(text, needle);
    }
}

#[test]
fn default_value_containing_equals_is_still_a_field() {
    // Deciding "constant" by "is there an '=' after the type" drops this field, because its
    // default value contains one.
    let m = one("string label \"a=b\"\nfloat64[] values").unwrap();
    assert_eq!(field_names(&m), ["label", "values"]);
}

#[test]
fn hash_inside_a_quoted_default_is_not_a_comment() {
    let m = one("string tag \"has # hash\"\nint32 n").unwrap();
    assert_eq!(field_names(&m), ["tag", "n"]);
}

#[test]
fn escaped_quote_in_default_does_not_end_the_string() {
    let m = one("string tag \"a\\\" # b\"\nint32 n").unwrap();
    assert_eq!(field_names(&m), ["tag", "n"]);
}

#[test]
fn comments_and_blank_lines_are_skipped() {
    let m = one("# leading\n\nfloat64 x  # trailing\n\n# tail\n").unwrap();
    assert_eq!(field_names(&m), ["x"]);
}

#[test]
fn empty_message_is_legal() {
    assert!(one("# nothing here\n").unwrap().fields.is_empty());
}

/// Two packages defining one message name with different layouts: the shape that made the old
/// basename fallback return the wrong one.
fn two_package_registry() -> Registry {
    let mut reg = Registry::new();
    for (pkg, name, ty) in [
        ("action_msgs", "status", "int8"),
        ("actionlib_msgs", "text", "string"),
    ] {
        let fields = vec![field(name, ty, ArrayKind::Scalar, 0)];
        reg.add(message("GoalStatus", fields, Some(pkg)), false)
            .unwrap();
    }
    reg
}

fn odometry() -> Message {
    let fields = vec![field("x", "float64", ArrayKind::Scalar, 0)];
    message("Odometry", fields, Some("nav_msgs"))
}

fn first_field(reg: &Registry, base: &str, package: Option<&str>) -> String {
    resolve(reg, base, package).unwrap().unwrap().fields[0]
        .name
        .clone()
}

#[test]
fn qualified_lookup_returns_that_package() {
    let reg = two_package_registry();
    assert_eq!(first_field(&reg, "action_msgs/GoalStatus", None), "status");
    assert_eq!(first_field(&reg, "actionlib_msgs/GoalStatus", None), "text");
}

#[test]
fn qualified_lookup_never_falls_back_to_the_basename() {
    // An unresolvable 'pkg/Msg' that fell back to any message with that basename would make a
    // missing package silently produce another package's layout and fingerprint. The
    // unique-basename case bites: the bare key is present, so a fallback finds it.
    let mut reg = Registry::new();
    reg.add(odometry(), false).unwrap();
    assert!(resolve(&reg, "nav_msgs/Odometry", None).unwrap().is_some());
    assert!(resolve(&reg, "foo_msgs/Odometry", None).unwrap().is_none());

    let reg = two_package_registry();
    assert!(
        resolve(&reg, "nav_msgs/GoalStatus", None)
            .unwrap()
            .is_none()
    );
}

#[test]
fn bare_name_domains_to_the_referring_package() {
    let reg = two_package_registry();
    assert_eq!(
        first_field(&reg, "GoalStatus", Some("action_msgs")),
        "status"
    );
    assert_eq!(
        first_field(&reg, "GoalStatus", Some("actionlib_msgs")),
        "text"
    );
}

#[test]
fn ambiguous_bare_name_is_an_error() {
    let reg = two_package_registry();
    let err = resolve(&reg, "GoalStatus", None).unwrap_err().to_string();
    assert!(err.contains("ambiguous"), "{err}");
    assert!(err.contains("action_msgs/GoalStatus"), "{err}");
    assert!(err.contains("actionlib_msgs/GoalStatus"), "{err}");
}

#[test]
fn unique_bare_name_still_resolves() {
    let mut reg = two_package_registry();
    reg.add(odometry(), false).unwrap();
    assert!(resolve(&reg, "Odometry", None).unwrap().is_some());
    assert!(
        resolve(&reg, "Odometry", Some("nav_msgs"))
            .unwrap()
            .is_some()
    );
}

#[test]
fn duplicate_registration_is_an_error() {
    let mut reg = two_package_registry();
    let err = reg
        .add(message("GoalStatus", vec![], Some("action_msgs")), false)
        .unwrap_err()
        .to_string();
    assert!(err.contains("duplicate message"), "{err}");
}

#[test]
fn ros_corpus_keys_every_message_under_its_package() {
    // Keying load_dir on the file stem alone loses one of every duplicated basename in a
    // whole-distro registry. Every file must survive under its own 'pkg/Msg'.
    if ros_msg_dirs().is_empty() {
        eprintln!("skipped: no installed ROS messages to sweep");
        return;
    }
    let reg = ros_corpus();
    let files = ros_msg_files();
    let qualified = reg.messages().filter(|(k, _)| k.contains('/')).count();
    assert_eq!(qualified, files.len());
    for path in files {
        let pkg = path
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .file_name()
            .unwrap();
        let stem = path.file_stem().unwrap();
        let key = format!("{}/{}", pkg.to_str().unwrap(), stem.to_str().unwrap());
        assert!(resolve(&reg, &key, None).unwrap().is_some(), "{key}");
    }
}

#[test]
fn installed_ros_messages_all_parse() {
    // Rejecting malformed input is only correct if no real message is caught by it.
    if ros_msg_dirs().is_empty() {
        eprintln!("skipped: no installed ROS messages to sweep");
        return;
    }
    let files = ros_msg_files();
    assert!(files.len() > 100, "expected a substantial corpus to sweep");
    for path in files {
        let text = fs::read_to_string(&path).unwrap();
        let stem = path.file_stem().unwrap().to_str().unwrap();
        if let Err(e) = parse_msg(&text, stem, Some(&path), None) {
            panic!("{e}");
        }
    }
}

#[test]
fn field_names_follow_the_ros_rule() {
    // Being laxer than ROS would accept a .msg that rosidl rejects, so the flux path and the
    // plain ROS path would disagree about whether the message is even valid.
    for line in [
        "float64 _leading",
        "float64 Upper",
        "float64 double__underscore",
        "float64 trailing_",
    ] {
        let parsed = parse_msg(&format!("{line}\n"), "M", None, Some("p"));
        assert!(parsed.is_err(), "{line}");
    }
}

#[test]
fn ordinary_field_names_are_accepted() {
    for line in ["float64 a_b", "float64 x1", "float64 point_cloud2"] {
        let m = parse_msg(&format!("{line}\n"), "M", None, Some("p")).unwrap();
        assert!(!m.fields.is_empty(), "{line}");
    }
}

#[test]
fn string_constants_follow_rosidl_quoting() {
    // Matching quotes are stripped and a comment ends the value. As in rosidl, the first '#'
    // ends the line even inside quotes, which leaves D an unmatched quote kept as text, and a
    // tab is a space. Values checked against rosidl_adapter's parse_message_string.
    let m = one(concat!(
        "string A='hello'   # note\n",
        "string B=\"hi there\"\n",
        "string C=bare value   # note\n",
        "string D='a#b'\n",
        "string E=\"a\tb\"\n",
        "string F=robot's arm # see\n",
        "uint8 x\n",
    ))
    .unwrap();
    let got: Vec<(&str, &ConstValue)> = m
        .constants
        .iter()
        .map(|c| (c.name.as_str(), &c.value))
        .collect();
    let s = |v: &str| ConstValue::Str(v.to_owned());
    assert_eq!(
        got,
        [
            ("A", &s("hello")),
            ("B", &s("hi there")),
            ("C", &s("bare value")),
            ("D", &s("'a")),
            ("E", &s("a b")),
            ("F", &s("robot's arm")),
        ]
    );
}

#[test]
fn a_duplicate_key_is_still_an_error() {
    let mut reg = Registry::new();
    reg.add(parse_msg("uint8 x\n", "M", None, Some("p")).unwrap(), false)
        .unwrap();
    let again = parse_msg("uint8 y\n", "M", None, Some("p")).unwrap();
    assert!(reg.add(again, false).is_err());
}

#[test]
fn if_absent_keeps_the_message_already_registered() {
    // The CLI loads the .msg files it was handed before merging the installed ones, so
    // generating for a package that is also installed reads the source it was pointed at.
    let mut reg = Registry::new();
    let mine = parse_msg("uint8 mine\n", "M", None, Some("p")).unwrap();
    let theirs = parse_msg("uint8 theirs\n", "M", None, Some("p")).unwrap();
    reg.add(mine, false).unwrap();
    reg.add(theirs, true).unwrap();
    assert_eq!(field_names(reg.get("p/M").unwrap()), ["mine"]);
}

fn constant(text: &str) -> Result<ConstValue, ParseError> {
    one(text).map(|m| m.constants[0].value.clone())
}

#[test]
fn every_python_line_boundary_ends_a_line() {
    // rosidl reads a .msg with str.splitlines(). Splitting on '\n' alone read "y" as a default
    // value of "x": one field silently gone, and a layout and fingerprint rosidl disagrees with.
    assert_eq!(field_names(&one("int32 x\rint32 y\n").unwrap()), ["x", "y"]);
    assert_eq!(
        field_names(&one("# note\x0cint32 y\nint32 x").unwrap()),
        ["y", "x"]
    );
    assert_eq!(field_names(&one("int32 x\x0bint32 y").unwrap()), ["x", "y"]);
    let err = one("int32 x\r\nint32 y\r\nfloat64[bad] z").unwrap_err();
    assert_eq!(err.lineno, Some(3), "a CRLF pair is one line break");
}

#[test]
fn integer_constants_read_as_rosidl_reads_them() {
    // rosidl tries int(text), then int(text, 0).
    let int = |v: i128| Ok(ConstValue::Int(v));
    for (text, want) in [
        ("int32 A=1_000", int(1000)),
        ("int32 A=007", int(7)),
        ("int32 A=0x_ff", int(255)),
        ("int32 A=-0x5", int(-5)),
        ("int32 A=+0b101", int(5)),
        ("int32 A=0O17", int(15)),
    ] {
        assert_eq!(constant(text).map_err(|e| e.message), want, "{text}");
    }
    for text in [
        "int32 A=-0x-5",
        "int32 A=0x+5",
        "int32 A=1__0",
        "int32 A=_1",
        "int32 A=1_",
        "int32 A=0x",
        "int32 A=1.5",
    ] {
        assert!(constant(text).is_err(), "{text}");
    }
}

#[test]
fn integer_constants_are_range_checked_like_rosidl() {
    // Unchecked, uint8 BIG=300 was 44 in C++ and 300 in Python.
    for text in [
        "uint8 A=300",
        "uint8 A=-1",
        "byte A=256",
        "char A=-1",
        "int8 A=-129",
        "uint32 A=-1",
        "int64 A=9223372036854775808",
        "uint64 A=18446744073709551616",
    ] {
        let err = constant(text).unwrap_err().message;
        assert!(err.contains("out of range"), "{text}: {err}");
    }
    assert_eq!(
        constant("int64 A=-9223372036854775808"),
        Ok(ConstValue::Int(i64::MIN.into()))
    );
    assert_eq!(
        constant("uint64 A=18446744073709551615"),
        Ok(ConstValue::Int(u64::MAX.into()))
    );
}

#[test]
fn other_constants_read_as_rosidl_reads_them() {
    let s = |v: &str| Ok(ConstValue::Str(v.to_owned()));
    assert_eq!(constant(r#"string Q="a\"b""#), s("a\"b"));
    assert_eq!(constant(r"string Q='it\'s'"), s("it's"));
    assert_eq!(constant(r#"string Q='say "hi"'"#), s("say \"hi\""));
    assert_eq!(constant(r#"string Q="a\nb""#), s(r"a\nb"));
    assert_eq!(constant(r#"string Q=""#), s(""));
    assert!(constant(r#"string Q="a"b""#).is_err());
    assert!(constant(r#"string<=3 Q="abcd""#).is_err());
    assert_eq!(constant(r#"string<=3 Q="abc""#), s("abc"));
    assert_eq!(constant("bool B=TRUE"), Ok(ConstValue::Bool(true)));
    assert_eq!(constant("bool B=False"), Ok(ConstValue::Bool(false)));
    assert_eq!(constant("float64 F=1_0.5"), Ok(ConstValue::Float(10.5)));
    assert!(constant("float64 F=1__0.5").is_err());
}

#[test]
fn a_constant_may_share_its_message_name() {
    // rosidl takes `uint8 ABC=1` in ABC.msg. Only the adapter cannot have it, so the parser keeps
    // the message: a carrier of ABC must still resolve it.
    let m = parse_msg("uint8 ABC=1\nuint8 value\n", "ABC", None, Some("p")).unwrap();
    assert_eq!(m.constants[0].name, "ABC");
}

#[test]
fn optional_fields_parse_as_rosidl_parses_them() {
    let m = one("@optional int32 x\n@optional\nint32 y\nint32 z").unwrap();
    let optional: Vec<bool> = m.fields.iter().map(|f| f.optional).collect();
    assert_eq!(optional, [true, true, false]);
    assert!(one("@optional\n@optional int32 x").is_err());
}

#[test]
fn a_bad_msg_fails_only_the_messages_that_read_it() {
    // One file this parser refuses anywhere on the path stopped every generation and the whole
    // bridge schema table.
    let dir = std::env::temp_dir().join(format!("flux2_gen_bad_{}", std::process::id()));
    fs::create_dir_all(&dir).unwrap();
    fs::write(dir.join("Good.msg"), "float64 v\n").unwrap();
    fs::write(dir.join("Bad.msg"), "int32 x\nfloat64[bad] y\n").unwrap();
    fs::write(dir.join("User.msg"), "Bad b\n").unwrap();
    let mut reg = Registry::new();
    let loaded = flux2_gen::parse::load_dir(&dir, "p", &mut reg, false);
    fs::remove_dir_all(&dir).unwrap();
    loaded.unwrap();

    assert!(resolve(&reg, "p/Good", None).unwrap().is_some());
    let err = resolve(&reg, "Bad", Some("p")).unwrap_err().to_string();
    assert!(err.contains("Bad.msg:2"), "{err}");
    let user = resolve(&reg, "p/User", None).unwrap().unwrap();
    assert!(flux2_gen::flatten::flatten_message(&user, &reg).is_err());
    let names: Vec<String> = flux2_gen::ament::layouts(&reg)
        .iter()
        .map(flux2_gen::layout::Layout::type_name)
        .collect();
    assert_eq!(names, ["p/Good"]);
}
