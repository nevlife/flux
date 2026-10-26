//! A frame of a generated schema as the CDR bytes rmw publishes for the same ROS message.
//!
//! The flat leaves are the message's fields in declaration order, nested structs inline, so one
//! walk over the layout reads the frame and writes CDR (little-endian, XCDR1: each primitive
//! aligned to its size from the end of the 4-byte encapsulation header). Two passes over the
//! same walk: the first counts, the second writes into memory the caller sized from the count.

use std::fmt;

use crate::dtypes::DType;
use crate::flatten::LeafKind;
use crate::layout::{Block, DESC_ALIGN, DESC_SIZE, Layout, Placed};
use crate::wire::Reader;

const ENCAPSULATION_CDR_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unreadable(pub String);

impl fmt::Display for Unreadable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Unreadable {}

trait Sink {
    fn pos(&self) -> usize;
    fn put(&mut self, bytes: &[u8]);
    fn zeros(&mut self, n: usize);
    /// Position measured from the end of the encapsulation header.
    fn align(&mut self, a: usize) {
        let body = self.pos() - ENCAPSULATION_CDR_LE.len();
        self.zeros((a - body % a) % a);
    }
    fn count(&mut self, n: usize) {
        self.align(4);
        #[allow(clippy::cast_possible_truncation)]
        self.put(&(n as u32).to_le_bytes());
    }
}

struct Counter(usize);

impl Sink for Counter {
    fn pos(&self) -> usize {
        self.0
    }
    fn put(&mut self, bytes: &[u8]) {
        self.0 += bytes.len();
    }
    fn zeros(&mut self, n: usize) {
        self.0 += n;
    }
}

/// Writes what fits and counts the rest, so a short `out` reports the size it needed.
struct Fill<'a> {
    out: &'a mut [u8],
    pos: usize,
}

impl Sink for Fill<'_> {
    fn pos(&self) -> usize {
        self.pos
    }
    fn put(&mut self, bytes: &[u8]) {
        if let Some(dst) = self.out.get_mut(self.pos..self.pos + bytes.len()) {
            dst.copy_from_slice(bytes);
        }
        self.pos += bytes.len();
    }
    fn zeros(&mut self, n: usize) {
        if let Some(dst) = self.out.get_mut(self.pos..self.pos + n) {
            dst.fill(0);
        }
        self.pos += n;
    }
}

struct Walk<'r, 'f, S> {
    r: &'r Reader<'f>,
    sink: S,
    type_name: &'r str,
    /// Frame bytes not yet read through a descriptor. A writer never points two descriptors at
    /// the same bytes, so a frame that does cannot make the CDR grow past its own size.
    unread: usize,
}

impl<S: Sink> Walk<'_, '_, S> {
    fn bad(&self) -> Unreadable {
        Unreadable(format!(
            "flux: {}: the frame does not match its schema",
            self.type_name
        ))
    }

    fn spend(&mut self, n: usize) -> Result<(), Unreadable> {
        self.unread = self.unread.checked_sub(n).ok_or_else(|| {
            Unreadable(format!(
                "flux: {}: the frame reads the same bytes twice",
                self.type_name
            ))
        })?;
        Ok(())
    }

    /// Up to the first NUL: rclpy's message holds a C string, so that is what it would publish.
    fn string(&mut self, bytes: &[u8]) {
        let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
        let bytes = &bytes[..end];
        self.sink.count(bytes.len() + 1);
        self.sink.put(bytes);
        self.sink.zeros(1);
    }

    /// A fixed T[N] cannot hold a frame of another length; a clamped copy would be a silently
    /// wrong message.
    fn length(&self, p: &Placed, n: usize) -> Result<(), Unreadable> {
        if p.is_dynamic() {
            return Ok(());
        }
        if n != p.leaf.count {
            return Err(Unreadable(format!(
                "flux: {}.{}: frame holds {n} elements, schema fixes {}",
                self.type_name,
                p.name(),
                p.leaf.count
            )));
        }
        Ok(())
    }

    fn primitives(&mut self, dt: DType, bytes: &[u8]) {
        self.sink.align(dt.size());
        if dt == DType::Bool {
            for &b in bytes {
                self.sink.put(&[u8::from(b != 0)]);
            }
        } else {
            self.sink.put(bytes);
        }
    }

    fn block(&mut self, block: &Block, base: usize) -> Result<(), Unreadable> {
        for p in &block.placed {
            let off = base + p.offset;
            match p.kind() {
                LeafKind::Fixed => {
                    let dt = p.leaf.dtype.expect("fixed leaves have a dtype");
                    let (n, size) = (p.leaf.count, dt.size());
                    // A scalar is read unaligned, as the C++ Reader's get() does.
                    let align = if n == 1 { 1 } else { size };
                    let bytes = self.r.region(off, n, size, align).unwrap_or_default();
                    self.primitives(dt, bytes);
                }
                LeafKind::Column => {
                    let dt = p.leaf.dtype.expect("columns have a dtype");
                    let bytes = self.r.span(off, dt.size(), dt.size()).unwrap_or_default();
                    self.spend(bytes.len())?;
                    self.sink.count(bytes.len() / dt.size());
                    if !bytes.is_empty() {
                        self.primitives(dt, bytes);
                    }
                }
                LeafKind::String => {
                    let s = self.r.str(off).unwrap_or_default();
                    self.spend(s.len())?;
                    self.string(s);
                }
                LeafKind::StringArray => {
                    let n = self.r.len_of(off, DESC_SIZE, DESC_ALIGN).unwrap_or(0);
                    self.length(p, n)?;
                    self.spend(n * DESC_SIZE)?;
                    if p.is_dynamic() {
                        self.sink.count(n);
                    }
                    for i in 0..n {
                        let s = self.r.str_at(off, i).unwrap_or_default();
                        self.spend(s.len())?;
                        self.string(s);
                    }
                }
                LeafKind::Stamp | LeafKind::Header => {
                    let stamp: [u8; 8] = self.r.bytes(off).unwrap_or_default();
                    self.sink.align(4);
                    self.sink.put(&stamp);
                    if p.kind() == LeafKind::Header {
                        let s = self.r.str(p.frame_id_offset()).unwrap_or_default();
                        self.spend(s.len())?;
                        self.string(s);
                    }
                }
                LeafKind::RecordColumn | LeafKind::Jagged => {
                    let elem = p.elem();
                    let n = self.r.len_of(off, elem.stride, elem.align).unwrap_or(0);
                    self.length(p, n)?;
                    self.spend(n * elem.stride)?;
                    if p.is_dynamic() {
                        self.sink.count(n);
                    }
                    for i in 0..n {
                        let at = self.r.elem(off, i, elem.stride, elem.align).unwrap_or(0);
                        self.block(elem, at)?;
                    }
                }
            }
            if self.r.bad() {
                return Err(self.bad());
            }
        }
        Ok(())
    }
}

fn walk<S: Sink>(layout: &Layout, frame: &[u8], mut sink: S) -> Result<S, Unreadable> {
    let r = Reader::new(frame);
    let type_name = layout.type_name();
    sink.put(&ENCAPSULATION_CDR_LE);
    let mut w = Walk {
        r: &r,
        sink,
        type_name: &type_name,
        unread: frame.len(),
    };
    w.block(&layout.root, 0)?;
    Ok(w.sink)
}

/// Bytes `write_cdr` will write for this frame, encapsulation header included.
pub fn cdr_size(layout: &Layout, frame: &[u8]) -> Result<usize, Unreadable> {
    walk(layout, frame, Counter(0)).map(|c| c.0)
}

/// Writes the frame as CDR into `out` and returns its size. A size past `out.len()` means `out` was
/// too small and holds no complete message.
pub fn write_cdr(layout: &Layout, frame: &[u8], out: &mut [u8]) -> Result<usize, Unreadable> {
    walk(layout, frame, Fill { out, pos: 0 }).map(|f| f.pos)
}
