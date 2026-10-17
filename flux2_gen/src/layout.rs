//! Assign byte offsets: a leaf list becomes a concrete slot layout both languages agree on.
//!
//! A frame is one pointer-free blob of two regions:
//!
//! ```text
//! [ scalar block: compile-time offsets ][ var region: bump-allocated per frame ]
//! ```
//!
//! The scalar block holds fixed values inline and, for every variable leaf, an 8-byte descriptor
//! `{u32 off, u32 len}`, `off` measured from the blob start. Record and jagged arrays are the
//! same mechanism one level down: an array of fixed-stride element blocks in the var region,
//! whose blocks may hold descriptors pointing further out. Offsets are u32, so one frame of a
//! generated schema is capped at 4 GiB regardless of `slot_size`.

use std::collections::HashMap;
use std::fmt;

use crate::dtypes::DType;
use crate::fingerprint::fingerprint;
use crate::flatten::{DYNAMIC_COUNT, FlatResult, Leaf, LeafKind, PathPart};
use crate::model::Constant;

/// `{u32 off, u32 len}`
pub const DESC_SIZE: usize = 8;
pub const DESC_ALIGN: usize = 4;
/// `{i32 sec, u32 nanosec}`
pub const STAMP_SIZE: usize = 8;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LayoutError(pub String);

impl fmt::Display for LayoutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for LayoutError {}

/// One leaf, given an offset inside its enclosing block.
#[derive(Clone, Debug)]
pub struct Placed {
    pub leaf: Leaf,
    pub offset: usize,
    /// The element block of a `RecordColumn` or Jagged leaf.
    pub elem: Option<Block>,
}

impl Placed {
    pub fn kind(&self) -> LeafKind {
        self.leaf.kind
    }

    pub fn name(&self) -> String {
        self.leaf.name()
    }

    /// True when the length rides in the frame rather than in the schema.
    pub fn is_dynamic(&self) -> bool {
        self.leaf.count == DYNAMIC_COUNT
    }

    /// Offset of a Header's `frame_id` descriptor; its stamp occupies the first 8 bytes.
    pub fn frame_id_offset(&self) -> usize {
        self.offset + STAMP_SIZE
    }

    /// # Panics
    /// On a leaf that is neither a record nor a jagged array.
    pub fn elem(&self) -> &Block {
        self.elem
            .as_ref()
            .expect("record and jagged leaves have an element block")
    }
}

/// A fixed-size run of placed leaves: a message's scalar block, or one element of an array.
#[derive(Clone, Debug)]
pub struct Block {
    pub placed: Vec<Placed>,
    /// Size rounded up to align, so an array of these is addressable by index.
    pub stride: usize,
    pub align: usize,
}

#[derive(Clone, Debug)]
pub struct Layout {
    pub name: String,
    pub package: Option<String>,
    pub tier: &'static str,
    pub root: Block,
    pub fingerprint: u64,
    pub constants: Vec<Constant>,
}

impl Layout {
    pub fn type_name(&self) -> String {
        match &self.package {
            Some(p) => format!("{p}/{}", self.name),
            None => self.name.clone(),
        }
    }

    pub fn scalar_bytes(&self) -> usize {
        self.root.stride
    }

    /// Where the top-level `std_msgs/Header`'s stamp sits in a frame, -1 without one: the
    /// channel's `stamp_offset` for trace events.
    ///
    /// # Panics
    /// When the stamp sits 2 GiB or more into the frame.
    pub fn stamp_offset(&self) -> i32 {
        self.root
            .placed
            .iter()
            .find(|p| p.kind() == LeafKind::Header)
            .map_or(-1, |p| {
                i32::try_from(p.offset).expect("a frame offset fits i32")
            })
    }

    /// Offsets of the fields in `TENSOR_FIELDS` order when they are the message's only fields,
    /// as in `tensor_msgs/ExperimentalTensor`. Matched by name and type, so a copy under another
    /// package name is a tensor message too.
    pub fn tensor_at(&self) -> Option<[usize; 7]> {
        let placed = &self.root.placed;
        if placed.len() != TENSOR_FIELDS.len() {
            return None;
        }
        let mut at = [0; 7];
        for (slot, (name, kind, dtype)) in at.iter_mut().zip(TENSOR_FIELDS) {
            let p = placed.iter().find(
                |p| matches!(p.leaf.path.as_slice(), [PathPart::Name(n)] if n.as_str() == name),
            )?;
            let one = kind != LeafKind::Fixed || p.leaf.count == 1;
            if p.kind() != kind || p.leaf.dtype != Some(dtype) || !one || p.leaf.ros_byte {
                return None;
            }
            *slot = p.offset;
        }
        Some(at)
    }
}

/// The fields of a `DLTensor` (`DLPack`) as a .msg has them, in `flux2::wire::TensorAt` order.
pub const TENSOR_FIELDS: [(&str, LeafKind, DType); 7] = [
    ("dtype_code", LeafKind::Fixed, DType::U8),
    ("dtype_bits", LeafKind::Fixed, DType::U8),
    ("dtype_lanes", LeafKind::Fixed, DType::U16),
    ("shape", LeafKind::Column, DType::I64),
    ("strides", LeafKind::Column, DType::I64),
    ("byte_offset", LeafKind::Fixed, DType::U64),
    ("data", LeafKind::Column, DType::U8),
];

fn camel(name: &str) -> String {
    name.split('_')
        .filter(|p| !p.is_empty())
        .map(|p| {
            let mut c = p.chars();
            c.next()
                .map(|f| f.to_uppercase().chain(c).collect::<String>())
                .unwrap_or_default()
        })
        .collect()
}

/// Class name both emitters give the element adapter of `path`.
pub fn elem_class_name(path: &[&Placed]) -> String {
    let mut s: String = path.iter().map(|p| camel(&p.name())).collect();
    s.push_str("Elem");
    s
}

fn align_up(n: usize, a: usize) -> usize {
    n.div_ceil(a) * a
}

/// (align, size) this leaf occupies in its enclosing block.
fn entry_shape(leaf: &Leaf) -> (usize, usize) {
    match leaf.kind {
        LeafKind::Fixed => {
            let sz = leaf.dtype.expect("fixed leaves have a dtype").size();
            (sz, sz * leaf.count)
        }
        LeafKind::Stamp => (4, STAMP_SIZE),
        // Stamp inline, frame_id as a descriptor.
        LeafKind::Header => (4, STAMP_SIZE + DESC_SIZE),
        _ => (DESC_ALIGN, DESC_SIZE),
    }
}

/// Pack leaves in declaration order at natural alignment, C-struct style.
pub fn build_block(leaves: &[Leaf]) -> Block {
    let (mut off, mut align) = (0, 1);
    let mut placed = Vec::with_capacity(leaves.len());
    for leaf in leaves {
        let (a, size) = entry_shape(leaf);
        off = align_up(off, a);
        let elem = (!leaf.record_leaves.is_empty()).then(|| build_block(&leaf.record_leaves));
        placed.push(Placed {
            leaf: leaf.clone(),
            offset: off,
            elem,
        });
        off += size;
        align = align.max(a);
    }
    Block {
        placed,
        stride: align_up(off, align),
        align,
    }
}

fn check_names(block: &Block, path: &str) -> Result<(), LayoutError> {
    let mut seen = Vec::<String>::new();
    for p in &block.placed {
        let name = p.name();
        if seen.contains(&name) {
            return Err(LayoutError(format!(
                "{path}{name}: two fields flatten to the same name -- nesting is joined with \
                 '_', so 'a_b' and 'a.b' collide"
            )));
        }
        if let Some(elem) = &p.elem {
            check_names(elem, &format!("{path}{name}[]."))?;
        }
        seen.push(name);
    }
    Ok(())
}

/// Element class names camel-case their field path, which erases the '_' vs '.' boundary; two
/// distinct paths landing on one class silently alias in Python and fail to compile in C++. The
/// message's own name must not be one of them either, nor View or Builder: the C++ adapter nests
/// them in a struct of that name, and the Python module rebinds them.
fn check_class_names(block: &Block, msg: &str) -> Result<(), LayoutError> {
    fn rec<'a>(
        b: &'a Block,
        prefix: &mut Vec<&'a Placed>,
        seen: &mut HashMap<String, String>,
    ) -> Result<(), LayoutError> {
        for p in &b.placed {
            let Some(elem) = &p.elem else { continue };
            prefix.push(p);
            let cls = elem_class_name(prefix);
            let dotted = prefix
                .iter()
                .map(|x| x.name())
                .collect::<Vec<_>>()
                .join(".");
            for name in [cls.clone(), format!("{cls}Builder"), format!("{cls}Array")] {
                if let Some(prev) = seen.get(&name) {
                    return Err(LayoutError(format!(
                        "element class name collision: {prev} and field path '{dotted}' both \
                         generate {name}; rename one to disambiguate"
                    )));
                }
                seen.insert(name, format!("field path '{dotted}'"));
            }
            rec(elem, prefix, seen)?;
            prefix.pop();
        }
        Ok(())
    }
    if msg == "View" || msg == "Builder" {
        return Err(LayoutError(format!(
            "message name {msg} is a class its adapter defines; rename the message"
        )));
    }
    let mut seen = HashMap::from([(msg.to_owned(), format!("the message {msg}"))]);
    rec(block, &mut Vec::new(), &mut seen)
}

/// A rejected result is the caller's to report; it is refused here.
pub fn build_layout(flat: &FlatResult) -> Result<Layout, LayoutError> {
    if let Some(why) = &flat.rejected {
        return Err(LayoutError(why.clone()));
    }
    if flat.constants.iter().any(|c| c.name == flat.name) {
        return Err(LayoutError(format!(
            "constant name '{}' collides with the struct the generated adapter defines",
            flat.name
        )));
    }
    let root = build_block(&flat.leaves);
    check_names(&root, "")?;
    check_class_names(&root, &flat.name)?;
    Ok(Layout {
        name: flat.name.clone(),
        package: flat.package.clone(),
        tier: flat.tier(),
        root,
        fingerprint: fingerprint(&flat.leaves, &flat.type_name()),
        constants: flat.constants.clone(),
    })
}

/// Every placed leaf with the path of placed leaves that leads to it, element blocks included.
pub fn walk(block: &Block) -> Vec<Vec<&Placed>> {
    fn rec<'a>(b: &'a Block, path: &mut Vec<&'a Placed>, out: &mut Vec<Vec<&'a Placed>>) {
        for p in &b.placed {
            path.push(p);
            out.push(path.clone());
            if let Some(elem) = &p.elem {
                rec(elem, path, out);
            }
            path.pop();
        }
    }
    let mut out = Vec::new();
    rec(block, &mut Vec::new(), &mut out);
    out
}
