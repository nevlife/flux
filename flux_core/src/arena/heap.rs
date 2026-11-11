//! The allocator both arenas use: segregated free lists over boundary-tagged blocks.

use std::io;
use std::ptr::NonNull;

// Block header: size | flags, then the previous block's offset (meaningful while that is free).
// A free block's body links its bin: next, prev.
const GRAIN: u64 = 16;
const HEADER: u64 = 16;
const MIN_BLOCK: u64 = 32;
const USED: u64 = 1;
const PREV_USED: u64 = 2;
const FLAGS: u64 = GRAIN - 1;
const NIL: u64 = u64::MAX;
const BINS: usize = 32;

/// Makes `[from, to)` of an allocator's memory usable before the allocator writes there.
pub(crate) type Grow<'a> = &'a mut dyn FnMut(u64, u64) -> io::Result<()>;

/// Segregated free lists over boundary-tagged blocks. The tags live at `base`: the memory itself
/// for the host arena, a host shadow of it for memory the CPU cannot write (the GPU arena).
pub(crate) struct Heap {
    base: NonNull<u8>,
    size: u64,
    pub(super) heads: [u64; BINS],
    pub(super) nonempty: u32,
    /// Bytes from the start made usable. Nothing past it is written.
    pub(super) committed: u64,
    /// How far `committed` moves at a time.
    step: u64,
}

// SAFETY: reached only under the arena's lock.
unsafe impl Send for Heap {}

#[allow(clippy::cast_possible_truncation)]
fn bin(size: u64) -> usize {
    (size.ilog2() as usize).min(BINS - 1)
}

fn round_up(n: u64, a: u64) -> u64 {
    n.div_ceil(a) * a
}

impl Heap {
    /// Over `size` bytes at `base`. With `grow`, they are made usable `step` at a time as blocks
    /// reach them; without, they all are already.
    pub(crate) fn new(
        base: NonNull<u8>,
        size: u64,
        step: u64,
        grow: Option<Grow<'_>>,
    ) -> io::Result<Heap> {
        let size = size / GRAIN * GRAIN;
        let mut h = Heap {
            base,
            size,
            heads: [NIL; BINS],
            nonempty: 0,
            committed: if grow.is_some() { 0 } else { size },
            step,
        };
        if let Some(grow) = grow {
            h.reach(MIN_BLOCK, grow)?;
        }
        h.make_free(0, h.size, true);
        Ok(h)
    }

    fn reach(&mut self, end: u64, grow: Grow<'_>) -> io::Result<()> {
        if end <= self.committed {
            return Ok(());
        }
        let to = round_up(end, self.step).min(self.size);
        grow(self.committed, to)?;
        self.committed = to;
        Ok(())
    }

    fn word(&self, off: u64) -> *mut u64 {
        // SAFETY: every offset passed is inside the arena, on an 8-byte boundary.
        #[allow(clippy::cast_possible_truncation)]
        unsafe {
            self.base.as_ptr().add(off as usize).cast()
        }
    }

    fn get(&self, off: u64) -> u64 {
        // SAFETY: see word.
        unsafe { self.word(off).read() }
    }

    fn set(&mut self, off: u64, v: u64) {
        debug_assert!(
            off + 8 <= self.committed,
            "arena store at {off} past its pages"
        );
        // SAFETY: see word; the lock makes this the only writer.
        unsafe { self.word(off).write(v) }
    }

    fn block_size(&self, b: u64) -> u64 {
        self.get(b) & !FLAGS
    }

    fn insert(&mut self, b: u64) {
        let i = bin(self.block_size(b));
        let next = self.heads[i];
        self.set(b + 16, next);
        self.set(b + 24, NIL);
        if next != NIL {
            self.set(next + 24, b);
        }
        self.heads[i] = b;
        self.nonempty |= 1 << i;
    }

    fn remove(&mut self, b: u64) {
        let i = bin(self.block_size(b));
        let (next, prev) = (self.get(b + 16), self.get(b + 24));
        if prev == NIL {
            self.heads[i] = next;
        } else {
            self.set(prev + 16, next);
        }
        if next != NIL {
            self.set(next + 24, prev);
        }
        if self.heads[i] == NIL {
            self.nonempty &= !(1 << i);
        }
    }

    fn make_free(&mut self, b: u64, size: u64, prev_used: bool) {
        self.set(b, size | if prev_used { PREV_USED } else { 0 });
        let n = b + size;
        if n < self.size {
            let h = self.get(n);
            self.set(n, h & !PREV_USED);
            self.set(n + 8, b);
        }
        self.insert(b);
    }

    fn make_used(&mut self, b: u64, size: u64, prev_used: bool) {
        self.set(b, size | USED | if prev_used { PREV_USED } else { 0 });
        let n = b + size;
        if n < self.size {
            let h = self.get(n);
            self.set(n, h | PREV_USED);
        }
    }

    /// First fit in `need`'s bin, else the head of a larger one, every block of which fits.
    fn find(&self, need: u64) -> Option<u64> {
        let i = bin(need);
        let mut b = self.heads[i];
        while b != NIL {
            if self.block_size(b) >= need {
                return Some(b);
            }
            b = self.get(b + 16);
        }
        let above = self.nonempty & !((2u32 << i).wrapping_sub(1));
        (above != 0).then(|| self.heads[above.trailing_zeros() as usize])
    }

    /// The payload offset, on an `align` boundary (a power of two). None when no block fits or
    /// its memory cannot be made usable.
    pub(crate) fn alloc(&mut self, bytes: u64, align: u64, grow: Grow<'_>) -> Option<u64> {
        let need = round_up(bytes.checked_add(HEADER)?, GRAIN).max(MIN_BLOCK);
        let align = align.max(GRAIN);
        let slack = if align > GRAIN { align + MIN_BLOCK } else { 0 };
        let b = self.find(need.checked_add(slack)?)?;
        // The block, its split-off tail's header and links: every byte this call writes.
        self.reach((b + slack + need + MIN_BLOCK).min(self.size), grow)
            .ok()?;
        self.remove(b);
        let mut size = self.block_size(b);
        let mut prev_used = self.get(b) & PREV_USED != 0;
        let mut start = b;
        if align > GRAIN {
            let mut p = round_up(b + HEADER, align);
            while p - HEADER != b && p - HEADER - b < MIN_BLOCK {
                p += align;
            }
            start = p - HEADER;
            if start > b {
                self.make_free(b, start - b, prev_used);
                size -= start - b;
                prev_used = false;
                self.set(start + 8, b);
            }
        }
        if size - need >= MIN_BLOCK {
            self.make_used(start, need, prev_used);
            self.make_free(start + need, size - need, true);
        } else {
            self.make_used(start, size, prev_used);
        }
        Some(start + HEADER)
    }

    pub(crate) fn free(&mut self, payload: u64) {
        let mut b = payload - HEADER;
        let h = self.get(b);
        debug_assert!(h & USED != 0, "arena block at {b} freed twice");
        let mut size = h & !FLAGS;
        let mut prev_used = h & PREV_USED != 0;
        let n = b + size;
        if n < self.size && self.get(n) & USED == 0 {
            self.remove(n);
            size += self.block_size(n);
        }
        if !prev_used {
            let p = self.get(b + 8);
            self.remove(p);
            size += self.block_size(p);
            prev_used = self.get(p) & PREV_USED != 0;
            b = p;
        }
        self.make_free(b, size, prev_used);
    }

    pub(crate) fn usable(&self, payload: u64) -> u64 {
        self.block_size(payload - HEADER) - HEADER
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn no_grow(_: u64, _: u64) -> io::Result<()> {
        unreachable!("all of a test heap is usable")
    }

    fn heap(bytes: usize) -> (Vec<u64>, Heap) {
        let mut mem = vec![0u64; bytes / 8];
        let base = NonNull::new(mem.as_mut_ptr().cast::<u8>()).unwrap();
        (mem, Heap::new(base, bytes as u64, GRAIN, None).unwrap())
    }

    #[test]
    fn blocks_do_not_overlap_and_coalesce_back_to_one() {
        let (_mem, mut h) = heap(1 << 16);
        let mut got = Vec::new();
        for (i, n) in [1u64, 17, 100, 4000, 64, 3, 900].iter().enumerate() {
            let align = if i % 2 == 0 { 16 } else { 64 };
            let p = h.alloc(*n, align, &mut no_grow).unwrap();
            assert_eq!(p % align, 0);
            assert!(h.usable(p) >= *n);
            got.push((p, *n));
        }
        let mut sorted = got.clone();
        sorted.sort_unstable();
        for w in sorted.windows(2) {
            assert!(w[0].0 + w[0].1 <= w[1].0);
        }
        for (i, (p, _)) in got.iter().enumerate() {
            if i % 2 == 0 {
                h.free(*p);
            }
        }
        for (i, (p, _)) in got.iter().enumerate() {
            if i % 2 == 1 {
                h.free(*p);
            }
        }
        assert_eq!(h.nonempty.count_ones(), 1);
        assert_eq!(h.block_size(h.heads[bin(h.size)]), h.size);
    }

    #[test]
    fn it_runs_out_and_recovers() {
        let (_mem, mut h) = heap(4096);
        let a = h.alloc(3000, 16, &mut no_grow).unwrap();
        assert!(h.alloc(3000, 16, &mut no_grow).is_none());
        h.free(a);
        assert!(h.alloc(4000, 16, &mut no_grow).is_some());
    }
}
