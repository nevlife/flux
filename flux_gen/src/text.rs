//! Spellings the emitted code shares: Python's `repr()` of a constant, and identifier mangling.

use std::fmt::Write;

/// Python's `repr(float)`: the shortest round-trip digits, positional between 1e-4 and 1e16.
///
/// # Panics
/// Never: `{:e}` always writes an integer exponent.
pub fn py_float(x: f64) -> String {
    if x.is_nan() {
        return "nan".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf" } else { "-inf" }.into();
    }
    let sci = format!("{:e}", x.abs());
    let (mantissa, exp) = sci.split_once('e').expect("{:e} has an exponent");
    let exp: i32 = exp.parse().expect("integer exponent");
    let digits: String = mantissa.chars().filter(char::is_ascii_digit).collect();
    let sign = if x.is_sign_negative() { "-" } else { "" };
    // Position of the decimal point after the first digit, as CPython's `decpt`.
    let decpt = exp + 1;
    if decpt <= -4 || decpt > 16 {
        let (first, rest) = digits.split_at(1);
        let dot = if rest.is_empty() {
            String::new()
        } else {
            format!(".{rest}")
        };
        let esign = if exp < 0 { '-' } else { '+' };
        return format!("{sign}{first}{dot}e{esign}{:02}", exp.unsigned_abs());
    }
    let point = decpt.unsigned_abs() as usize;
    let body = if decpt <= 0 {
        format!("0.{}{digits}", "0".repeat(point))
    } else if point >= digits.len() {
        format!("{digits}{}.0", "0".repeat(point - digits.len()))
    } else {
        let (a, b) = digits.split_at(point);
        format!("{a}.{b}")
    };
    format!("{sign}{body}")
}

/// Python's `repr(str)`.
pub fn py_str(s: &str) -> String {
    let quote = if s.contains('\'') && !s.contains('"') {
        '"'
    } else {
        '\''
    };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(quote);
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\t' => out.push_str("\\t"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            c if c == quote => {
                out.push('\\');
                out.push(c);
            }
            c if !py_printable(c) => {
                write!(out, "\\x{:02x}", u32::from(c)).expect("writing to a String");
            }
            c => out.push(c),
        }
    }
    out.push(quote);
    out
}

/// ASCII as `str.isprintable()` decides it. No installed .msg constant holds anything else.
fn py_printable(c: char) -> bool {
    !(c < ' ' || c == '\x7f')
}

const PY_KEYWORDS: [&str; 35] = [
    "False", "None", "True", "and", "as", "assert", "async", "await", "break", "class", "continue",
    "def", "del", "elif", "else", "except", "finally", "for", "from", "global", "if", "import",
    "in", "is", "lambda", "nonlocal", "not", "or", "pass", "raise", "return", "try", "while",
    "with", "yield",
];

/// C++20 keywords and alternative tokens, and the lowercase macros a translation unit commonly
/// has: glibc's `assert` and `errno`, GCC's `linux` and `unix` in GNU mode.
const CPP_RESERVED: [&str; 96] = [
    "alignas",
    "alignof",
    "and",
    "and_eq",
    "asm",
    "auto",
    "bitand",
    "bitor",
    "bool",
    "break",
    "case",
    "catch",
    "char",
    "char8_t",
    "char16_t",
    "char32_t",
    "class",
    "compl",
    "concept",
    "const",
    "consteval",
    "constexpr",
    "constinit",
    "const_cast",
    "continue",
    "co_await",
    "co_return",
    "co_yield",
    "decltype",
    "default",
    "delete",
    "do",
    "double",
    "dynamic_cast",
    "else",
    "enum",
    "explicit",
    "export",
    "extern",
    "false",
    "float",
    "for",
    "friend",
    "goto",
    "if",
    "inline",
    "int",
    "long",
    "mutable",
    "namespace",
    "new",
    "noexcept",
    "not",
    "not_eq",
    "nullptr",
    "operator",
    "or",
    "or_eq",
    "private",
    "protected",
    "public",
    "register",
    "reinterpret_cast",
    "requires",
    "return",
    "short",
    "signed",
    "sizeof",
    "static",
    "static_assert",
    "static_cast",
    "struct",
    "switch",
    "template",
    "this",
    "thread_local",
    "throw",
    "true",
    "try",
    "typedef",
    "typeid",
    "typename",
    "union",
    "unsigned",
    "using",
    "virtual",
    "void",
    "volatile",
    "wchar_t",
    "while",
    "xor",
    "xor_eq",
    "assert",
    "errno",
    "linux",
    "unix",
];

/// A field name that is a Python keyword.
pub fn py_ident(name: &str) -> String {
    if PY_KEYWORDS.contains(&name) {
        format!("{name}_")
    } else {
        name.to_owned()
    }
}

pub fn cpp_ident(name: &str) -> String {
    if CPP_RESERVED.contains(&name) {
        format!("{name}_")
    } else {
        name.to_owned()
    }
}

/// The Python module of a message: its snake name, escaped when that is a keyword (`Global`).
pub fn py_module(name: &str) -> String {
    py_ident(&snake(name))
}

/// `CamelCase` to `snake_case`, keeping an acronym together: `PointCloud2` to `point_cloud2`,
/// `GPSFix` to `gps_fix`.
pub fn snake(name: &str) -> String {
    let c: Vec<char> = name.chars().collect();
    let mut out = String::with_capacity(name.len() + 4);
    for (i, &ch) in c.iter().enumerate() {
        if ch.is_uppercase()
            && i > 0
            && (!c[i - 1].is_uppercase() || c.get(i + 1).is_some_and(|n| n.is_lowercase()))
        {
            out.push('_');
        }
        out.extend(ch.to_lowercase());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floats_spell_as_python_repr() {
        for (x, s) in [
            (1.0, "1.0"),
            (0.1, "0.1"),
            (-2.5, "-2.5"),
            (1e16, "1e+16"),
            (1e15, "1000000000000000.0"),
            (1.5e16, "1.5e+16"),
            (0.0001, "0.0001"),
            (0.00001, "1e-05"),
            (1.25e-7, "1.25e-07"),
            (123.456, "123.456"),
            (-0.0, "-0.0"),
            (0.0, "0.0"),
            (f64::INFINITY, "inf"),
            (std::f64::consts::PI, "3.141592653589793"),
            (1e100, "1e+100"),
        ] {
            assert_eq!(py_float(x), s, "{x}");
        }
    }

    #[test]
    fn strings_spell_as_python_repr() {
        assert_eq!(py_str("abc"), "'abc'");
        assert_eq!(py_str("it's"), "\"it's\"");
        assert_eq!(py_str("both ' \""), "'both \\' \"'");
        assert_eq!(py_str("a\\b\n\t\x01"), "'a\\\\b\\n\\t\\x01'");
    }

    #[test]
    fn snake_keeps_acronyms() {
        assert_eq!(snake("PointCloud2"), "point_cloud2");
        assert_eq!(snake("GPSFix"), "gps_fix");
        assert_eq!(snake("Imu"), "imu");
        assert_eq!(snake("CameraInfo"), "camera_info");
    }
}
