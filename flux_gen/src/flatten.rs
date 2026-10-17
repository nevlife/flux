//! Flatten a parsed message into an ordered leaf layout and classify its tier.
//!
//! The wire is one pointer-free blob, so nesting is resolved here. A shape that cannot ride it
//! is rejected and the caller falls back to plain ROS.

use std::fmt;

use crate::dtypes::{DType, ros_primitive};
use crate::model::{ArrayKind, Constant, Field, Message};
use crate::parse::{ParseError, Registry, resolve};

/// Nested fixed arrays multiply past the parser's per-field bound, so only a total cap stops
/// the blowup. ROS jazzy's largest message flattens to 107 leaves.
pub const MAX_LEAVES: usize = 4096;

/// flatten expands a fixed array of structs leaf by leaf and layout multiplies N into offsets, so a
/// huge N exhausts memory or overflows. ROS jazzy's largest is [36].
pub const MAX_FIXED_ARRAY: usize = 1 << 16;

/// `Leaf::count` of every per-frame-length leaf. The default 1 would fingerprint T[] like T[1].
pub const DYNAMIC_COUNT: usize = 0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeafKind {
    /// Compile-time offset scalar block entry (dtype x count).
    Fixed,
    /// Per-frame dynamic primitive array.
    Column,
    /// Per-frame dynamic array of a fixed struct.
    RecordColumn,
    /// Single string (variable tail).
    String,
    /// Array of strings (variable tail).
    StringArray,
    /// Array of a variable-length element.
    Jagged,
    /// Top-level `std_msgs/Header`: stamp + `frame_id`.
    Header,
    /// Top-level bare `builtin_interfaces/Time`.
    Stamp,
}

impl LeafKind {
    pub fn token(self) -> &'static str {
        match self {
            LeafKind::Fixed => "fixed",
            LeafKind::Column => "column",
            LeafKind::RecordColumn => "record",
            LeafKind::String => "string",
            LeafKind::StringArray => "sarray",
            LeafKind::Jagged => "jagged",
            LeafKind::Header => "header",
            LeafKind::Stamp => "stamp",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PathPart {
    Name(String),
    Index(usize),
}

impl fmt::Display for PathPart {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PathPart::Name(n) => f.write_str(n),
            PathPart::Index(i) => write!(f, "{i}"),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Leaf {
    pub kind: LeafKind,
    /// Set for Fixed and Column.
    pub dtype: Option<DType>,
    /// N for a fixed T[N], `DYNAMIC_COUNT` when the length is per-frame.
    pub count: usize,
    /// Flattened element layout for `RecordColumn` and Jagged.
    pub record_leaves: Vec<Leaf>,
    /// ("pose", "position", "x"), or ("poses", 0, "x") through a fixed array. Not fingerprinted.
    pub path: Vec<PathPart>,
    /// `RecordColumn`/Jagged: the element's `pkg/Msg`, for the ROS adapter.
    pub elem_type: Option<String>,
    /// A ROS `byte`: u8 on the wire, but rclpy holds it as `bytes`.
    pub ros_byte: bool,
}

impl Leaf {
    fn new(kind: LeafKind, path: Vec<PathPart>) -> Leaf {
        Leaf {
            kind,
            dtype: None,
            count: 1,
            record_leaves: Vec::new(),
            path,
            elem_type: None,
            ros_byte: false,
        }
    }

    pub fn name(&self) -> String {
        self.path
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join("_")
    }
}

#[derive(Clone, Debug)]
pub struct FlatResult {
    pub name: String,
    pub leaves: Vec<Leaf>,
    /// Why the message cannot ride the fast path.
    pub rejected: Option<String>,
    pub package: Option<String>,
    pub constants: Vec<Constant>,
}

impl FlatResult {
    pub fn type_name(&self) -> String {
        match &self.package {
            Some(p) => format!("{p}/{}", self.name),
            None => self.name.clone(),
        }
    }

    pub fn tier(&self) -> &'static str {
        if self.rejected.is_some() {
            "reject"
        } else if self.leaves.iter().any(|l| l.kind == LeafKind::Jagged) {
            "jagged"
        } else if self.leaves.iter().any(|l| l.kind != LeafKind::Fixed) {
            "columnar"
        } else {
            "fixed"
        }
    }
}

enum Stop {
    Reject(String),
    Parse(ParseError),
}

impl From<ParseError> for Stop {
    fn from(e: ParseError) -> Stop {
        Stop::Parse(e)
    }
}

fn all_fixed(leaves: &[Leaf]) -> bool {
    leaves.iter().all(|l| l.kind == LeafKind::Fixed)
}

fn reparent(leaves: &[Leaf], prefix: &[PathPart]) -> Vec<Leaf> {
    leaves
        .iter()
        .map(|l| {
            let mut l = l.clone();
            l.path.splice(0..0, prefix.iter().cloned());
            l
        })
        .collect()
}

fn expand(
    fld: &Field,
    reg: &Registry,
    seen: &[String],
    package: Option<&str>,
) -> Result<Vec<Leaf>, Stop> {
    let path = vec![PathPart::Name(fld.name.clone())];
    let reject = |m: String| Err(Stop::Reject(format!("{}: {m}", fld.name)));
    if fld.kind == ArrayKind::Bounded {
        return reject("bounded array [<=N] not supported".into());
    }
    if fld.optional {
        return reject("@optional not supported".into());
    }
    if fld.kind == ArrayKind::Fixed && fld.size > MAX_FIXED_ARRAY {
        return reject(format!(
            "fixed array size {} exceeds the {MAX_FIXED_ARRAY} limit (use a dynamic T[] instead)",
            fld.size
        ));
    }

    if let Some(dt) = ros_primitive(&fld.type_name) {
        let (kind, count) = match fld.kind {
            ArrayKind::Scalar => (LeafKind::Fixed, 1),
            ArrayKind::Fixed => (LeafKind::Fixed, fld.size),
            _ if dt == DType::Bool => return reject("bool[] not supported".into()),
            _ => (LeafKind::Column, DYNAMIC_COUNT),
        };
        let mut leaf = Leaf::new(kind, path);
        leaf.dtype = Some(dt);
        leaf.count = count;
        leaf.ros_byte = fld.type_name == "byte";
        return Ok(vec![leaf]);
    }

    if fld.type_name == "string" {
        if fld.kind == ArrayKind::Scalar {
            return Ok(vec![Leaf::new(LeafKind::String, path)]);
        }
        let mut leaf = Leaf::new(LeafKind::StringArray, path);
        leaf.count = if fld.kind == ArrayKind::Fixed {
            fld.size
        } else {
            DYNAMIC_COUNT
        };
        return Ok(vec![leaf]);
    }
    if fld.type_name == "wstring" {
        return reject("wstring not supported".into());
    }

    let Some(msg) = resolve(reg, &fld.type_name, package)? else {
        return reject(format!("unknown type '{}'", fld.type_name));
    };
    // Keyed on the resolved message: two packages can define the same basename.
    let ident = msg.ident();
    if seen.contains(&ident) {
        return reject(format!("recursive nested type '{ident}'"));
    }
    let mut inner_seen = seen.to_vec();
    inner_seen.push(ident.clone());
    let sub = flatten(&msg, reg, &inner_seen, false)?;
    if sub.is_empty() {
        return reject(format!("nested type '{ident}' has no publishable fields"));
    }

    let element = |kind, count| {
        let mut leaf = Leaf::new(kind, path.clone());
        leaf.count = count;
        leaf.record_leaves.clone_from(&sub);
        leaf.elem_type = Some(ident.clone());
        vec![leaf]
    };
    match fld.kind {
        ArrayKind::Scalar => Ok(reparent(&sub, &path)),
        ArrayKind::Fixed if all_fixed(&sub) => {
            // Bound before materializing: the product is what blows up, not the operands.
            let total = sub.len() * fld.size;
            if total > MAX_LEAVES {
                return reject(format!(
                    "fixed nesting expands to {total} leaves (limit {MAX_LEAVES})"
                ));
            }
            let mut out = Vec::with_capacity(total);
            for i in 0..fld.size {
                let prefix = [path[0].clone(), PathPart::Index(i)];
                out.extend(reparent(&sub, &prefix));
            }
            Ok(out)
        }
        // record_leaves carries the element structure into the fingerprint.
        ArrayKind::Fixed => Ok(element(LeafKind::Jagged, fld.size)),
        _ if all_fixed(&sub) => Ok(element(LeafKind::RecordColumn, DYNAMIC_COUNT)),
        _ => Ok(element(LeafKind::Jagged, DYNAMIC_COUNT)),
    }
}

/// Only at the top level do these map to the frame's stamp and `frame_id`; nested deeper they
/// flatten as ordinary structs (the frame carries one stamp). A bare name counts only when it
/// resolves to the builtin type: a package's own Header.msg or Time.msg is an ordinary struct.
fn frame_meta(
    fld: &Field,
    reg: &Registry,
    package: Option<&str>,
) -> Result<Option<Leaf>, ParseError> {
    let t = fld.type_name.as_str();
    if fld.kind != ArrayKind::Scalar || !(t.ends_with("Header") || t.ends_with("Time")) {
        return Ok(None);
    }
    let ident = if t.contains('/') {
        t.to_owned()
    } else {
        match resolve(reg, t, package)? {
            Some(m) => m.ident(),
            None => return Ok(None),
        }
    };
    let kind = match ident.as_str() {
        "std_msgs/Header" => LeafKind::Header,
        "builtin_interfaces/Time" => LeafKind::Stamp,
        _ => return Ok(None),
    };
    Ok(Some(Leaf::new(
        kind,
        vec![PathPart::Name(fld.name.clone())],
    )))
}

fn flatten(msg: &Message, reg: &Registry, seen: &[String], top: bool) -> Result<Vec<Leaf>, Stop> {
    let package = msg.package.as_deref();
    let mut out = Vec::new();
    for fld in &msg.fields {
        let meta = if top {
            frame_meta(fld, reg, package)?
        } else {
            None
        };
        match meta {
            Some(meta) => out.push(meta),
            None => out.extend(expand(fld, reg, seen, package)?),
        }
        if out.len() > MAX_LEAVES {
            return Err(Stop::Reject(format!(
                "{}: flattens to more than {MAX_LEAVES} leaves",
                msg.name
            )));
        }
    }
    Ok(out)
}

/// A shape the wire cannot carry is a rejected result; an ambiguous type reference is an error.
pub fn flatten_message(msg: &Message, reg: &Registry) -> Result<FlatResult, ParseError> {
    let mut flat = FlatResult {
        name: msg.name.clone(),
        leaves: Vec::new(),
        rejected: None,
        package: msg.package.clone(),
        constants: msg.constants.clone(),
    };
    match flatten(msg, reg, &[], true) {
        Ok(leaves) if leaves.is_empty() => {
            flat.rejected = Some(format!("{}: no publishable fields", msg.name));
        }
        Ok(leaves) => flat.leaves = leaves,
        Err(Stop::Reject(why)) => flat.rejected = Some(why),
        Err(Stop::Parse(e)) => return Err(e),
    }
    Ok(flat)
}
