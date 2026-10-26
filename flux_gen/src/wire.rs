//! The flat frame layout `flux_gen` computes, read side. Twin of `flux/wire.hpp`'s Reader.
//!
//! A frame is `[ scalar block | var region ]`. Fixed values sit inline in the scalar block;
//! everything variable is an 8-byte descriptor `{u32 off, u32 len}` pointing into the var
//! region, offsets measured from the frame start so the blob stays pointer-free.
//!
//! A frame is written by another process, so `Reader` trusts none of it: every descriptor is
//! checked against the frame size and the alignment its type needs. A violation latches `bad`
//! and turns further reads into `None`. Values are native-endian, as both ends share the host.

use std::cell::Cell;

use crate::layout::{DESC_ALIGN, DESC_SIZE};

fn fits(off: usize, bytes: usize, size: usize) -> bool {
    off <= size && bytes <= size - off
}

/// Read side of a received frame. Offsets assume the frame starts 8-byte aligned, as a flux
/// slot does (it starts on a page).
pub struct Reader<'a> {
    buf: &'a [u8],
    bad: Cell<bool>,
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8]) -> Reader<'a> {
        Reader {
            buf,
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

    /// `n` elements of `esize` bytes at `off`, which must be a multiple of `align`.
    pub fn region(&self, off: usize, n: usize, esize: usize, align: usize) -> Option<&'a [u8]> {
        let size = self.buf.len();
        if self.bad() || !off.is_multiple_of(align) || off > size || n > (size - off) / esize {
            return self.fail();
        }
        Some(&self.buf[off..off + n * esize])
    }

    pub fn bytes<const N: usize>(&self, off: usize) -> Option<[u8; N]> {
        if self.bad() || !fits(off, N, self.buf.len()) {
            return self.fail();
        }
        self.buf[off..off + N].try_into().ok()
    }

    pub fn u32(&self, off: usize) -> Option<u32> {
        self.bytes(off).map(u32::from_ne_bytes)
    }

    /// `(off, len)` of the descriptor at `off`.
    pub fn desc(&self, off: usize) -> Option<(usize, usize)> {
        Some((self.u32(off)? as usize, self.u32(off + 4)? as usize))
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
