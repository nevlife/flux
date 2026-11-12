//! A frame whose fields are in its publisher's arena, made into one that holds them itself.
//!
//! A bag cannot keep arena offsets: the arena is gone when the bag is played. The frame body is
//! copied, each field in the arena is appended after it, and its descriptor is rewritten to point
//! there. Element arrays are walked too, since their elements hold descriptors of their own. The
//! result is a frame of the same schema with no arena, which a slot reader reads as any other.

use crate::cdr::Unreadable;
use crate::flatten::LeafKind;
use crate::layout::{Block, DESC_ALIGN, DESC_SIZE, Layout};
use crate::wire::{GPU_POS, HEAP_POS};

struct Inline<'a> {
    heap: &'a [u8],
    out: Vec<u8>,
    type_name: String,
    /// Arena bytes left to append. A writer never points two descriptors at the same bytes, so a
    /// frame that does cannot make the result grow past the arena.
    unread: usize,
}

impl Inline<'_> {
    fn bad(&self) -> Unreadable {
        Unreadable(format!(
            "flux: {}: the frame does not match its schema",
            self.type_name
        ))
    }

    fn u32_at(&self, off: usize) -> Result<u32, Unreadable> {
        let b = self.out.get(off..off + 4).ok_or_else(|| self.bad())?;
        Ok(u32::from_ne_bytes(b.try_into().map_err(|_| self.bad())?))
    }

    /// The descriptor at `off` pointing at its `esize`-byte elements in the frame, after copying
    /// them out of the arena when they are there; their position and count.
    fn relocate(
        &mut self,
        off: usize,
        esize: usize,
        align: usize,
    ) -> Result<(usize, usize), Unreadable> {
        let wire = self.u32_at(off)?;
        let n = self.u32_at(off + 4)? as usize;
        let pos = crate::wire::position(wire);
        let bytes = n.checked_mul(esize).ok_or_else(|| self.bad())?;
        if pos >= GPU_POS {
            return Err(Unreadable(format!(
                "flux: {}: a field is in GPU memory",
                self.type_name
            )));
        }
        if pos < HEAP_POS {
            let fits =
                pos.is_multiple_of(align) && pos <= self.out.len() && bytes <= self.out.len() - pos;
            return if fits { Ok((pos, n)) } else { Err(self.bad()) };
        }
        let at = pos - HEAP_POS;
        if !at.is_multiple_of(align) || at > self.heap.len() || bytes > self.heap.len() - at {
            return Err(self.bad());
        }
        self.unread = self.unread.checked_sub(bytes).ok_or_else(|| {
            Unreadable(format!(
                "flux: {}: the frame reads the same bytes twice",
                self.type_name
            ))
        })?;
        let to = self.out.len().next_multiple_of(align.max(1));
        let end = to
            .checked_add(bytes)
            .filter(|&e| e < HEAP_POS)
            .ok_or_else(|| {
                Unreadable(format!(
                    "flux: {}: the frame with its arena fields passes 2 GiB",
                    self.type_name
                ))
            })?;
        self.out.resize(to, 0);
        self.out.extend_from_slice(&self.heap[at..at + bytes]);
        debug_assert_eq!(self.out.len(), end);
        #[allow(clippy::cast_possible_truncation)]
        self.out[off..off + 4].copy_from_slice(&(to as u32).to_ne_bytes());
        Ok((to, n))
    }

    fn block(&mut self, block: &Block, base: usize) -> Result<(), Unreadable> {
        for p in &block.placed {
            let off = base + p.offset;
            match p.kind() {
                LeafKind::Fixed | LeafKind::Stamp => {}
                LeafKind::Column => {
                    let size = p.leaf.dtype.expect("columns have a dtype").size();
                    self.relocate(off, size, size)?;
                }
                LeafKind::String => {
                    self.relocate(off, 1, 1)?;
                }
                LeafKind::Header => {
                    self.relocate(base + p.frame_id_offset(), 1, 1)?;
                }
                LeafKind::StringArray => {
                    let (at, n) = self.relocate(off, DESC_SIZE, DESC_ALIGN)?;
                    for i in 0..n {
                        self.relocate(at + i * DESC_SIZE, 1, 1)?;
                    }
                }
                LeafKind::RecordColumn | LeafKind::Jagged => {
                    let elem = p.elem();
                    let (at, n) = self.relocate(off, elem.stride, elem.align)?;
                    for i in 0..n {
                        self.block(elem, at + i * elem.stride)?;
                    }
                }
            }
        }
        Ok(())
    }
}

/// `body`, a frame without its arena trailer, with every field `heap` holds copied in. Fails for
/// a frame that does not match the schema, has a field in GPU memory, or would pass 2 GiB.
pub fn inline_arena(layout: &Layout, body: &[u8], heap: &[u8]) -> Result<Vec<u8>, Unreadable> {
    let mut w = Inline {
        heap,
        out: body.to_vec(),
        type_name: layout.type_name(),
        unread: heap.len(),
    };
    if body.len() < layout.root.stride {
        return Err(w.bad());
    }
    w.block(&layout.root, 0)?;
    // The copy reads as a frame of its own: the schema's walk finds every field in it.
    if crate::cdr::cdr_size(layout, &w.out).is_err() {
        return Err(w.bad());
    }
    Ok(w.out)
}
