//! The allocator both arenas use: segregated free lists over boundary-tagged blocks, two levels
//! of them as in TLSF, so finding a block never walks a list.

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
/// Each power of two of block sizes is split into this many lists.
const SL_BITS: u32 = 3;
const SL: usize = 1 << SL_BITS;
/// Powers of two: blocks below 2^40 bytes, well past the largest arena.
const FL: usize = 40;

/// Makes `[from, to)` of an allocator's memory usable before the allocator writes there.
pub(crate) type Grow<'a> = &'a mut dyn FnMut(u64, u64) -> io::Result<()>;

/// Segregated free lists over boundary-tagged blocks. The tags live at `base`: the memory itself
/// for the host arena, a host shadow of it for memory the CPU cannot write (the GPU arena).
pub(crate) struct Heap {
    base: NonNull<u8>,
    size: u64,
    heads: [[u64; SL]; FL],
    /// Bit i: some list of power i has a block. Bit j of `sl_map[i]`: list j of it has one.
    fl_map: u64,
    sl_map: [u8; FL],
    /// Bytes from the start made usable. Nothing past it is written.
    pub(super) committed: u64,
    /// How far `committed` moves at a time.
    step: u64,
}

// SAFETY: reached only under the arena's lock.
unsafe impl Send for Heap {}

/// The list a free block of `size` bytes (at least `MIN_BLOCK`) goes on: its power of two, then
/// which eighth of that power.
#[allow(clippy::cast_possible_truncation)]
fn list_of(size: u64) -> (usize, usize) {
    let fl = size.ilog2();
    let sl = (size >> (fl - SL_BITS)) as usize & (SL - 1);
    ((fl as usize).min(FL - 1), sl)
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
            heads: [[NIL; SL]; FL],
            fl_map: 0,
            sl_map: [0; FL],
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
        let (f, s) = list_of(self.block_size(b));
        let next = self.heads[f][s];
        self.set(b + 16, next);
        self.set(b + 24, NIL);
        if next != NIL {
            self.set(next + 24, b);
        }
        self.heads[f][s] = b;
        self.sl_map[f] |= 1 << s;
        self.fl_map |= 1 << f;
    }

    fn remove(&mut self, b: u64) {
        let (f, s) = list_of(self.block_size(b));
        let (next, prev) = (self.get(b + 16), self.get(b + 24));
        if prev == NIL {
            self.heads[f][s] = next;
        } else {
            self.set(prev + 16, next);
        }
        if next != NIL {
            self.set(next + 24, prev);
        }
        if self.heads[f][s] == NIL {
            self.sl_map[f] &= !(1 << s);
            if self.sl_map[f] == 0 {
                self.fl_map &= !(1 << f);
            }
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

    /// The head of `need`'s own list when it fits, the common case of frames of one size; else
    /// the head of the first list at or above `need` rounded up to the next list, every block of
    /// which fits. Either way no list is walked.
    fn find(&self, need: u64) -> Option<u64> {
        let (f, s) = list_of(need);
        let head = self.heads[f][s];
        if head != NIL && self.block_size(head) >= need {
            return Some(head);
        }
        let up = need.checked_add((1 << (need.ilog2() - SL_BITS)) - 1)?;
        let (f, s) = list_of(up);
        let here = u32::from(self.sl_map[f]) & (u32::MAX << s);
        let (f, s) = if here != 0 {
            (f, here.trailing_zeros() as usize)
        } else {
            let above = self.fl_map
                & u64::MAX
                    .checked_shl(u32::try_from(f + 1).ok()?)
                    .unwrap_or(0);
            if above == 0 {
                return None;
            }
            let f = above.trailing_zeros() as usize;
            (f, self.sl_map[f].trailing_zeros() as usize)
        };
        Some(self.heads[f][s])
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

    /// Lists with a free block: one once everything has coalesced back.
    #[cfg(test)]
    pub(super) fn lists_in_use(&self) -> u32 {
        self.sl_map.iter().map(|m| m.count_ones()).sum()
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
        assert_eq!(h.lists_in_use(), 1);
        let (f, s) = list_of(h.size);
        assert_eq!(h.block_size(h.heads[f][s]), h.size);
    }

    /// A list full of free blocks too small for the request is not walked: the request goes to a
    /// later list, whose every block fits.
    #[test]
    fn a_request_skips_a_list_of_blocks_too_small_for_it() {
        let (_mem, mut h) = heap(1 << 20);
        let all: Vec<_> = (0..2000)
            .map(|_| h.alloc(80, 16, &mut no_grow).unwrap())
            .collect();
        for p in all.iter().step_by(2) {
            h.free(*p);
        }
        let p = h.alloc(96, 16, &mut no_grow).unwrap();
        assert!(h.usable(p) >= 96);
        assert!(
            all.iter().all(|&q| q + 80 <= p - HEADER || q >= p + 96),
            "overlaps a block"
        );
    }

    /// Freed blocks of one size between blocks still used cannot coalesce; once nothing else is
    /// left, a request of that size still gets one of them. 1024 bytes make a 1040-byte block,
    /// inside its list rather than at its start, so rounding up alone would pass them over.
    #[test]
    fn a_freed_block_between_used_ones_serves_a_request_of_its_size() {
        let (_mem, mut h) = heap(1 << 16);
        let mut mine = Vec::new();
        while let Some(p) = h.alloc(1024, 16, &mut no_grow) {
            mine.push(p);
            if h.alloc(100, 16, &mut no_grow).is_none() {
                break;
            }
        }
        for p in &mine {
            h.free(*p);
        }
        for _ in 0..mine.len() {
            assert!(
                h.alloc(1024, 16, &mut no_grow).is_some(),
                "a freed block fits"
            );
        }
    }

    /// ns per alloc and free with `n` free blocks too small for the request on its list, for
    /// `n` from 10 to 10000: flat when no list is walked. `cargo test --release -- --ignored`.
    #[test]
    #[ignore = "a timing run"]
    fn alloc_time_against_free_blocks_on_its_list() {
        for n in [10usize, 100, 1000, 10_000] {
            let (_mem, mut h) = heap(4 << 20);
            let all: Vec<_> = (0..2 * n)
                .map(|_| h.alloc(80, 16, &mut no_grow).unwrap())
                .collect();
            for p in all.iter().step_by(2) {
                h.free(*p);
            }
            // Fewer rounds where each is slow, so a run stays short on the timing cores.
            let rounds = (2_000_000 / u32::try_from(n).unwrap()).clamp(2000, 200_000);
            let t = std::time::Instant::now();
            for _ in 0..rounds {
                let p = h.alloc(96, 16, &mut no_grow).unwrap();
                h.free(std::hint::black_box(p));
            }
            let ns = t.elapsed().as_nanos() / u128::from(rounds);
            println!("free blocks {n:>6}: {ns} ns per alloc and free");
        }
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
