//! The flat frame layout `flux_gen` computes, read side. Twin of `flux/wire.hpp`'s Reader.
//!
//! A frame is `[ scalar block | var region ]`. Fixed values sit inline in the scalar block;
//! everything variable is an 8-byte descriptor `{u32 off, u32 len}` pointing into the var
//! region, offsets measured from the frame start so the blob stays pointer-free.
//!
//! A frame is written by another process, so `Reader` trusts none of it: every descriptor is
//! checked against the frame size and the alignment its type needs. A violation latches `bad`
//! and turns further reads into `None`. Values are native-endian, as both ends share the host.
//!
//! A descriptor offset at or past `WIRE_HEAP` names the publisher's arena in `UNIT`s, from
//! `WIRE_GPU` its GPU arena. The reader works in positions, bytes from 0: the frame's below
//! `HEAP_POS`, the arena's from it (`flux_core::arena` has the same numbers).

use std::cell::Cell;

use crate::layout::{DESC_ALIGN, DESC_SIZE};

pub const WIRE_HEAP: u32 = 1 << 31;
pub const WIRE_GPU: u32 = 3 << 30;
pub const UNIT: usize = 16;
pub const HEAP_POS: usize = 1 << 31;
pub const GPU_POS: usize = HEAP_POS + (WIRE_GPU - WIRE_HEAP) as usize * UNIT;

/// The position a descriptor offset names.
pub fn position(wire: u32) -> usize {
    if wire >= WIRE_GPU {
        GPU_POS + (wire - WIRE_GPU) as usize * UNIT
    } else if wire >= WIRE_HEAP {
        HEAP_POS + (wire - WIRE_HEAP) as usize * UNIT
    } else {
        wire as usize
    }
}

fn fits(off: usize, bytes: usize, size: usize) -> bool {
    off <= size && bytes <= size - off
}

/// Read side of a received frame. Offsets assume the frame starts 8-byte aligned, as a flux
/// slot does (it starts on a page).
pub struct Reader<'a> {
    buf: &'a [u8],
    heap: &'a [u8],
    bad: Cell<bool>,
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8]) -> Reader<'a> {
        Reader::with_heap(buf, &[])
    }

    /// A frame whose descriptors may name `heap`, the arena its trailer names. GPU arena
    /// positions are not the CPU's: reading one latches bad.
    pub fn with_heap(buf: &'a [u8], heap: &'a [u8]) -> Reader<'a> {
        Reader {
            buf,
            heap,
            bad: Cell::new(false),
        }
    }

    pub fn bad(&self) -> bool {
        self.bad.get()
    }

    fn fail<T>(&self) -> Option<T> {
        self.bad.set(true);
        None
    }

    /// `n` elements of `esize` bytes at position `off`, which must be a multiple of `align`.
    pub fn region(&self, off: usize, n: usize, esize: usize, align: usize) -> Option<&'a [u8]> {
        let (mem, at) = if off >= GPU_POS {
            return self.fail();
        } else if off >= HEAP_POS {
            (self.heap, off - HEAP_POS)
        } else {
            (self.buf, off)
        };
        let size = mem.len();
        if self.bad() || !at.is_multiple_of(align) || at > size || n > (size - at) / esize {
            return self.fail();
        }
        Some(&mem[at..at + n * esize])
    }

    /// Bytes descriptors can name: the frame's and its arena's.
    pub fn budget(&self) -> usize {
        self.buf.len() + self.heap.len()
    }

    /// `N` bytes at position `off`, unaligned.
    pub fn bytes<const N: usize>(&self, off: usize) -> Option<[u8; N]> {
        let (mem, at) = if off >= GPU_POS {
            return self.fail();
        } else if off >= HEAP_POS {
            (self.heap, off - HEAP_POS)
        } else {
            (self.buf, off)
        };
        if self.bad() || !fits(at, N, mem.len()) {
            return self.fail();
        }
        mem[at..at + N].try_into().ok()
    }

    pub fn u32(&self, off: usize) -> Option<u32> {
        self.bytes(off).map(u32::from_ne_bytes)
    }

    /// `(position, len)` of the descriptor at `off`.
    pub fn desc(&self, off: usize) -> Option<(usize, usize)> {
        Some((position(self.u32(off)?), self.u32(off + 4)? as usize))
    }

    /// Element count of a variable array, checked to fit in the frame.
    pub fn len_of(&self, off: usize, esize: usize, align: usize) -> Option<usize> {
        let (at, n) = self.desc(off)?;
        self.region(at, n, esize, align).map(|_| n)
    }

    /// The bytes of a variable array whose elements are `esize` bytes.
    pub fn span(&self, off: usize, esize: usize, align: usize) -> Option<&'a [u8]> {
        let (at, n) = self.desc(off)?;
        self.region(at, n, esize, align)
    }

    pub fn str(&self, off: usize) -> Option<&'a [u8]> {
        self.span(off, 1, 1)
    }

    /// Entry `i` of a string array whose descriptor is at `off`.
    pub fn str_at(&self, off: usize, i: usize) -> Option<&'a [u8]> {
        let at = self.elem(off, i, DESC_SIZE, DESC_ALIGN)?;
        self.str(at)
    }

    /// Frame offset of element `i` of a record/jagged array whose descriptor is at `off`.
    pub fn elem(&self, off: usize, i: usize, stride: usize, align: usize) -> Option<usize> {
        let (at, n) = self.desc(off)?;
        self.region(at, n, stride, align)?;
        if i >= n {
            return self.fail();
        }
        Some(at + i * stride)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame of `size` bytes with the descriptor `(at, n)` at offset 0.
    fn frame(size: usize, at: u32, n: u32) -> Vec<u8> {
        let mut f = vec![0u8; size];
        f[..4].copy_from_slice(&at.to_ne_bytes());
        f[4..8].copy_from_slice(&n.to_ne_bytes());
        f
    }

    #[test]
    fn a_descriptor_inside_the_frame_reads() {
        let mut f = frame(16, 8, 3);
        f[8..11].copy_from_slice(b"map");
        let r = Reader::new(&f);
        assert_eq!(r.str(0), Some(&b"map"[..]));
        assert!(!r.bad());
    }

    #[test]
    fn a_descriptor_past_the_frame_latches_bad() {
        let f = frame(16, 8, 100);
        let r = Reader::new(&f);
        assert_eq!(r.str(0), None);
        assert!(r.bad());
        assert_eq!(r.u32(8), None, "a latched reader reads nothing more");
    }

    #[test]
    fn a_misaligned_span_latches_bad() {
        let f = frame(32, 9, 1);
        let r = Reader::new(&f);
        assert_eq!(r.span(0, 4, 4), None);
        assert!(r.bad());
    }

    #[test]
    fn an_index_past_the_array_latches_bad() {
        let f = frame(24, 8, 2);
        let r = Reader::new(&f);
        assert_eq!(r.elem(0, 1, 8, 8), Some(16));
        assert_eq!(r.elem(0, 2, 8, 8), None);
        assert!(r.bad());
    }

    #[test]
    fn a_string_array_entry_reads_through_its_own_descriptor() {
        let mut f = frame(32, 8, 1);
        f[8..12].copy_from_slice(&16u32.to_ne_bytes());
        f[12..16].copy_from_slice(&2u32.to_ne_bytes());
        f[16..18].copy_from_slice(b"ok");
        let r = Reader::new(&f);
        assert_eq!(r.len_of(0, DESC_SIZE, DESC_ALIGN), Some(1));
        assert_eq!(r.str_at(0, 0), Some(&b"ok"[..]));
    }
}
