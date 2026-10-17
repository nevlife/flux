//! Minimal ROS 2 .msg parser (fields, arrays, nested types, constants; no rosidl dependency).
//!
//! Every ambiguous shape is a `ParseError`, never a guess: a misread field yields a wrong layout
//! and fingerprint that both generated ends then agree on.

use std::collections::HashMap;
use std::fmt;
use std::path::{Path, PathBuf};
use std::rc::Rc;

use crate::dtypes::{DType, ros_primitive};
use crate::model::{ArrayKind, ConstValue, Constant, Field, Message};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub message: String,
    pub path: Option<PathBuf>,
    pub lineno: Option<usize>,
    pub line: Option<String>,
}

impl ParseError {
    pub fn new(message: impl Into<String>) -> ParseError {
        ParseError {
            message: message.into(),
            path: None,
            lineno: None,
            line: None,
        }
    }
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "flux2_gen: ")?;
        if self.path.is_some() || self.lineno.is_some() {
            match &self.path {
                Some(p) => write!(f, "{}:", p.display())?,
                None => write!(f, "<msg>:")?,
            }
            match self.lineno {
                Some(n) => write!(f, "{n}: ")?,
                None => write!(f, "?: ")?,
            }
        }
        write!(f, "{}", self.message)?;
        if let Some(line) = self.line.as_deref().filter(|l| !l.is_empty()) {
            write!(f, " in '{}'", line.trim())?;
        }
        Ok(())
    }
}

impl std::error::Error for ParseError {}

struct Ctx<'a> {
    path: Option<&'a Path>,
    lineno: usize,
    line: &'a str,
}

impl Ctx<'_> {
    fn err(&self, message: impl Into<String>) -> ParseError {
        ParseError {
            message: message.into(),
            path: self.path.map(Path::to_path_buf),
            lineno: Some(self.lineno),
            line: Some(self.line.to_owned()),
        }
    }
}

fn is_ident(s: &str) -> bool {
    let mut chars = s.chars();
    chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn is_type_name(s: &str) -> bool {
    s.split('/').all(is_ident)
}

/// `^[a-z](?:_?[a-z0-9]+)*$` for `lower`, the same over upper case otherwise: no leading,
/// doubled or trailing underscore, which keeps the adapter's own names unreachable from a .msg.
fn is_snake(s: &str, lower: bool) -> bool {
    let letter = |c: char| {
        if lower {
            c.is_ascii_lowercase()
        } else {
            c.is_ascii_uppercase()
        }
    };
    let b = s.as_bytes();
    if b.is_empty() || !letter(b[0] as char) || b[b.len() - 1] == b'_' {
        return false;
    }
    b.windows(2).all(|w| !(w[0] == b'_' && w[1] == b'_'))
        && s.chars()
            .all(|c| letter(c) || c.is_ascii_digit() || c == '_')
}

/// rosidl cuts a line at its first '#', inside quotes too, so a constant's value ends there.
fn strip_comment(raw: &str) -> &str {
    raw.find('#').map_or(raw, |i| &raw[..i])
}

/// rosidl takes any positive size; flatten decides what the wire can carry.
fn parse_size(inner: &str, tok: &str, ctx: &Ctx) -> Result<usize, ParseError> {
    if inner.is_empty() || !inner.bytes().all(|b| b.is_ascii_digit()) {
        return Err(ctx.err(format!(
            "bad fixed array size '{inner}' in type '{tok}' (expected T[N], T[] or T[<=N])"
        )));
    }
    let size: usize = inner.parse().unwrap_or(usize::MAX);
    if size == 0 {
        return Err(ctx.err(format!(
            "zero-length fixed array in type '{tok}' (use T[] for a dynamic array)"
        )));
    }
    Ok(size)
}

fn parse_type(tok: &str, ctx: &Ctx) -> Result<(String, ArrayKind, usize), ParseError> {
    let mut base = tok;
    let mut kind = ArrayKind::Scalar;
    let mut size = 0;
    if tok.ends_with(']') {
        let Some(i) = tok.rfind('[') else {
            return Err(ctx.err(format!("unmatched ']' in type '{tok}'")));
        };
        let inner = tok[i + 1..tok.len() - 1].trim();
        base = &tok[..i];
        if base.is_empty() {
            return Err(ctx.err(format!("array with no element type: '{tok}'")));
        }
        if inner.is_empty() {
            kind = ArrayKind::Dynamic;
        } else if let Some(bound) = inner.strip_prefix("<=") {
            kind = ArrayKind::Bounded;
            size = parse_size(bound.trim(), tok, ctx)?;
        } else {
            kind = ArrayKind::Fixed;
            size = parse_size(inner, tok, ctx)?;
        }
    } else if tok.contains('[') {
        return Err(ctx.err(format!("unterminated array in type '{tok}' (missing ']')")));
    }

    // string<=N: the bound does not affect the flat wire. Split, not a prefix test, so that
    // "stringgg<=3" is not taken as a string.
    if let Some((bounded, _)) = base.split_once("<=") {
        if bounded != "string" && bounded != "wstring" {
            return Err(ctx.err(format!(
                "'<=' bound is only valid on string/wstring, got '{tok}'"
            )));
        }
        base = bounded;
    }

    if !is_type_name(base) {
        return Err(ctx.err(format!("malformed type name '{base}' in '{tok}'")));
    }
    Ok((base.to_owned(), kind, size))
}

/// Python's digit run: `_` only between two digits of `radix`, as `int()` and `float()` take it.
fn python_digits(s: &str, radix: u32) -> Option<String> {
    let b = s.as_bytes();
    let digit = |i: usize| b.get(i).is_some_and(|&c| char::from(c).is_digit(radix));
    let mut out = String::with_capacity(s.len());
    for (i, &c) in b.iter().enumerate() {
        if c == b'_' {
            if i == 0 || !digit(i - 1) || !digit(i + 1) {
                return None;
            }
        } else {
            out.push(char::from(c));
        }
    }
    Some(out)
}

/// rosidl's `int(text)`, then `int(text, 0)`: a sign, then decimal (leading zeros allowed) or a
/// 0x/0o/0b literal, with `_` between digits.
fn parse_int(text: &str) -> Option<i128> {
    let (neg, body) = match text.as_bytes().first() {
        Some(b'-') => (true, &text[1..]),
        Some(b'+') => (false, &text[1..]),
        _ => (false, text),
    };
    let lower = body.to_ascii_lowercase();
    let (radix, digits) = match lower.get(..2) {
        Some("0x") => (16, &lower[2..]),
        Some("0o") => (8, &lower[2..]),
        Some("0b") => (2, &lower[2..]),
        _ => (10, lower.as_str()),
    };
    let digits = match radix {
        10 => digits,
        _ => digits.strip_prefix('_').unwrap_or(digits),
    };
    let digits = python_digits(digits, radix)?;
    if digits.is_empty() || !digits.chars().all(|c| c.is_digit(radix)) {
        return None;
    }
    let v = i128::from_str_radix(&digits, radix).ok()?;
    Some(if neg { -v } else { v })
}

/// Python's `float(text)`; Rust's parser takes the same spellings once the `_` are gone.
fn parse_float(text: &str) -> Option<f64> {
    if text.contains('_') {
        python_digits(text, 10)?.parse().ok()
    } else {
        text.parse().ok()
    }
}

fn int_range(dt: DType) -> (i128, i128) {
    match dt {
        DType::U8 => (0, u8::MAX.into()),
        DType::I8 => (i8::MIN.into(), i8::MAX.into()),
        DType::U16 => (0, u16::MAX.into()),
        DType::I16 => (i16::MIN.into(), i16::MAX.into()),
        DType::U32 => (0, u32::MAX.into()),
        DType::I32 => (i32::MIN.into(), i32::MAX.into()),
        DType::U64 => (0, u64::MAX.into()),
        _ => (i64::MIN.into(), i64::MAX.into()),
    }
}

/// rosidl's string constant: one pair of matching outer quotes is removed, a quote of that kind
/// inside must be escaped, and `\<quote>` becomes the quote. Anything else is verbatim.
fn string_constant(text: &str, ctx: &Ctx) -> Result<String, ParseError> {
    for quote in ['"', '\''] {
        if !(text.starts_with(quote) && text.ends_with(quote)) {
            continue;
        }
        let inner = text.get(1..text.len() - 1).unwrap_or("");
        let mut prev = None;
        for c in inner.chars() {
            if c == quote && prev != Some('\\') {
                return Err(ctx.err(format!(
                    "string constant {text} has an unescaped {quote} inside"
                )));
            }
            prev = Some(c);
        }
        return Ok(inner.replace(&format!("\\{quote}"), &quote.to_string()));
    }
    Ok(text.to_owned())
}

fn constant_value(
    base: &str,
    bound: Option<usize>,
    text: &str,
    ctx: &Ctx,
) -> Result<ConstValue, ParseError> {
    let text = text.trim();
    if base == "string" || base == "wstring" {
        let s = string_constant(text, ctx)?;
        if let Some(n) = bound.filter(|&n| s.chars().count() > n) {
            return Err(ctx.err(format!("string constant {text} is longer than {n}")));
        }
        return Ok(ConstValue::Str(s));
    }
    if base == "bool" {
        return match text.to_ascii_lowercase().as_str() {
            "1" | "true" => Ok(ConstValue::Bool(true)),
            "0" | "false" => Ok(ConstValue::Bool(false)),
            _ => Err(ctx.err(format!(
                "bool constant must be 0/1/true/false, got '{text}'"
            ))),
        };
    }
    let Some(dt) = ros_primitive(base) else {
        return Err(ctx.err(format!(
            "only primitive and string types can be constants, got '{base}'"
        )));
    };
    if base.starts_with("float") {
        return parse_float(text)
            .map(ConstValue::Float)
            .ok_or_else(|| ctx.err(format!("constant value '{text}' is not a {base}")));
    }
    let (lo, hi) = int_range(dt);
    match parse_int(text) {
        Some(v) if (lo..=hi).contains(&v) => Ok(ConstValue::Int(v)),
        Some(_) => Err(ctx.err(format!(
            "constant value '{text}' is out of range for {base} ({lo}..={hi})"
        ))),
        None => Err(ctx.err(format!("constant value '{text}' is not a {base}"))),
    }
}

/// Python's `str.splitlines()`, which rosidl reads a .msg with: `\r`, `\f` and the other line
/// boundaries end a line as `\n` does.
fn lines(text: &str) -> impl Iterator<Item = &str> {
    const BREAKS: [char; 10] = [
        '\n', '\r', '\x0b', '\x0c', '\x1c', '\x1d', '\x1e', '\u{85}', '\u{2028}', '\u{2029}',
    ];
    text.split("\r\n").flat_map(|l| l.split(BREAKS))
}

/// Only an '=' right after the name makes a constant: a string default may contain '='.
fn split_field<'a>(rest: &'a str, ctx: &Ctx) -> Result<(&'a str, Option<&'a str>), ParseError> {
    let b = rest.as_bytes();
    if b.is_empty() || !(b[0].is_ascii_alphabetic() || b[0] == b'_') {
        return Err(ctx.err("missing or malformed field name after the type"));
    }
    let end = b
        .iter()
        .position(|&c| !(c.is_ascii_alphanumeric() || c == b'_'))
        .unwrap_or(b.len());
    let tail = rest[end..].trim_start();
    Ok((&rest[..end], tail.strip_prefix('=')))
}

pub fn parse_msg(
    text: &str,
    name: &str,
    path: Option<&Path>,
    package: Option<&str>,
) -> Result<Message, ParseError> {
    let mut msg = Message {
        name: name.to_owned(),
        package: package.map(str::to_owned),
        path: path.map(Path::to_path_buf),
        ..Message::default()
    };
    let mut seen = Vec::<String>::new();
    let mut optional = false;
    // As rosidl: a tab in a value is a space.
    let text = text.replace('\t', " ");
    for (i, raw) in lines(&text).enumerate() {
        let mut line = strip_comment(raw).trim();
        if line.is_empty() {
            continue;
        }
        let ctx = Ctx {
            path,
            lineno: i + 1,
            line: raw,
        };
        if let Some(rest) = line.strip_prefix("@optional") {
            if optional {
                return Err(ctx.err("@optional twice on one field"));
            }
            optional = true;
            line = rest.trim_start();
            if line.is_empty() {
                continue;
            }
        }
        let (type_tok, rest) = match line.find(char::is_whitespace) {
            Some(at) => (&line[..at], Some(line[at..].trim_start())),
            None => (line, None),
        };
        let (base, kind, size) = parse_type(type_tok, &ctx)?;
        let Some(rest) = rest else {
            return Err(ctx.err(format!("type '{type_tok}' with no field name")));
        };

        let (field_name, const_text) = split_field(rest, &ctx)?;
        if let Some(const_text) = const_text {
            if kind != ArrayKind::Scalar {
                return Err(ctx.err(format!(
                    "array constant '{field_name}' is not valid in a .msg"
                )));
            }
            if !is_snake(field_name, false) {
                return Err(ctx.err(format!(
                    "malformed constant name '{field_name}' (rosidl requires UPPER_CASE)"
                )));
            }
            if msg.constants.iter().any(|c| c.name == field_name) {
                return Err(ctx.err(format!("duplicate constant name '{field_name}'")));
            }
            let bound = type_tok.split_once("<=").and_then(|(_, n)| n.parse().ok());
            let value = constant_value(&base, bound, const_text, &ctx)?;
            msg.constants.push(Constant {
                name: field_name.to_owned(),
                type_name: base,
                value,
            });
            optional = false;
            continue;
        }

        if !is_snake(field_name, true) {
            return Err(ctx.err(format!("malformed field name '{field_name}'")));
        }
        if seen.iter().any(|s| s == field_name) {
            return Err(ctx.err(format!("duplicate field name '{field_name}'")));
        }
        seen.push(field_name.to_owned());
        msg.fields.push(Field {
            name: field_name.to_owned(),
            type_name: base,
            kind,
            size,
            optional,
        });
        optional = false;
    }
    Ok(msg)
}

/// A parsed message, or why its .msg did not parse.
pub type Entry = Result<Rc<Message>, ParseError>;

/// Keyed by 'pkg/Msg', and by bare 'Msg' only while one package defines it.
///
/// A second definition withdraws the bare key into `ambiguous`: same-basename messages
/// (`action_msgs/GoalStatus`, `actionlib_msgs/GoalStatus`) differ in layout.
///
/// A .msg that does not parse is kept as its error, so that it fails only the messages that read
/// it: rosidl takes files this parser refuses, and one of them anywhere on the path must not stop
/// every other message.
#[derive(Default)]
pub struct Registry {
    map: HashMap<String, Entry>,
    pub ambiguous: HashMap<String, Vec<String>>,
    owners: HashMap<String, Vec<String>>,
}

impl Registry {
    pub fn new() -> Registry {
        Registry::default()
    }

    /// A message that parsed.
    pub fn get(&self, key: &str) -> Option<&Rc<Message>> {
        self.map.get(key)?.as_ref().ok()
    }

    /// A message, or the error its .msg gave.
    pub fn entry(&self, key: &str) -> Option<&Entry> {
        self.map.get(key)
    }

    /// The messages that parsed.
    pub fn messages(&self) -> impl Iterator<Item = (&String, &Rc<Message>)> {
        self.map
            .iter()
            .filter_map(|(k, e)| Some((k, e.as_ref().ok()?)))
    }

    pub fn add(&mut self, msg: Message, if_absent: bool) -> Result<(), ParseError> {
        let (package, name) = (msg.package.clone(), msg.name.clone());
        self.insert(package.as_deref(), &name, Ok(msg), if_absent)
    }

    fn insert(
        &mut self,
        package: Option<&str>,
        bare: &str,
        parsed: Result<Message, ParseError>,
        if_absent: bool,
    ) -> Result<(), ParseError> {
        let key = package.map_or_else(|| bare.to_owned(), |p| format!("{p}/{bare}"));
        let owners = self.owners.entry(bare.to_owned()).or_default();
        if owners.contains(&key) {
            if if_absent {
                return Ok(());
            }
            return Err(ParseError::new(format!(
                "duplicate message '{key}' in the registry"
            )));
        }
        owners.push(key.clone());
        let single = owners.len() == 1;
        let owners = owners.clone();
        let entry = parsed.map(Rc::new);
        if package.is_some() {
            self.map.insert(key, entry.clone());
        }
        if single {
            self.map.insert(bare.to_owned(), entry);
        } else {
            self.map.remove(bare);
            self.ambiguous.insert(bare.to_owned(), owners);
        }
        Ok(())
    }
}

/// Every .msg in `dir`, as messages of `package`. `if_absent` lets an installed copy yield to a
/// message already named.
pub fn load_dir(
    dir: &Path,
    package: &str,
    reg: &mut Registry,
    if_absent: bool,
) -> Result<(), ParseError> {
    let mut entries: Vec<String> = std::fs::read_dir(dir)
        .map_err(|e| ParseError::new(format!("{}: {e}", dir.display())))?
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .filter(|n| Path::new(n).extension().is_some_and(|e| e == "msg"))
        .collect();
    entries.sort();
    for entry in entries {
        let full = dir.join(&entry);
        let name = &entry[..entry.len() - 4];
        let parsed = std::fs::read_to_string(&full)
            .map_err(|e| ParseError::new(format!("{}: {e}", full.display())))
            .and_then(|text| parse_msg(&text, name, Some(&full), Some(package)));
        reg.insert(Some(package), name, parsed, if_absent)?;
    }
    Ok(())
}

/// A qualified name never falls back to the basename; a bare one tries the referring package,
/// then the registry-wide alias, and errs if that alias is ambiguous.
pub fn resolve(
    reg: &Registry,
    base: &str,
    package: Option<&str>,
) -> Result<Option<Rc<Message>>, ParseError> {
    let found = |e: &Entry| e.clone().map(Some);
    if base.contains('/') {
        return reg.entry(base).map_or(Ok(None), found);
    }
    if let Some(p) = package.filter(|p| !p.is_empty())
        && let Some(e) = reg.entry(&format!("{p}/{base}"))
    {
        return found(e);
    }
    if let Some(clash) = reg.ambiguous.get(base).filter(|c| !c.is_empty()) {
        return Err(ParseError::new(format!(
            "ambiguous type name '{base}': defined by {} -- qualify it as pkg/Msg",
            clash.join(", ")
        )));
    }
    reg.entry(base).map_or(Ok(None), found)
}
