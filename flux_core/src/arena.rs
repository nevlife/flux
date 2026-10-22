//! Experimental: the process's arena, one shm file a frame's fields can live in past its slot.
//!
//! A frame names arena bytes by offset, never by pointer, so a subscriber maps the file wherever
//! it lands and a Python reader reads it as it reads a slot. A descriptor offset at or past
//! `HEAP_BASE` is `HEAP_BASE` + an arena offset; the frame ends in a trailer naming the file.
//! The publisher frees what a frame owns when it claims that frame's slot again, so the slot's
//! reader counts guard the arena bytes too.
//!
//! On only with `FLUX_ARENA_BYTES` set. Allocation takes no heap memory of its own, so a malloc
//! hook (`flux_heaphook`) can route a thread's allocations here.

use std::ffi::{CStr, c_void};
use std::os::fd::{AsRawFd, OwnedFd};
use std::ptr::NonNull;
use std::sync::OnceLock;

use crate::sys::{self, PiMutex};

pub const HEAP_BASE: u64 = 1 << 31;
pub const MAX_ARENA_BYTES: usize = 1 << 31;
pub const TRAILER_BYTES: usize = 64;
const MAGIC: [u8; 16] = *b"flux-arena-v1\0\0\0";
const PREFIX: &str = "/flux_arena_";
const NAME_BYTES: usize = TRAILER_BYTES - MAGIC.len();

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

/// Segregated free lists over boundary-tagged blocks, kept in the arena itself.
struct Heap {
    base: NonNull<u8>,
    size: u64,
    heads: [u64; BINS],
    nonempty: u32,
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
    fn new(base: NonNull<u8>, size: u64) -> Heap {
        let mut h = Heap {
            base,
            size: size / GRAIN * GRAIN,
            heads: [NIL; BINS],
            nonempty: 0,
        };
        h.make_free(0, h.size, true);
        h
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

    /// The payload offset, on an `align` boundary (a power of two).
    fn alloc(&mut self, bytes: u64, align: u64) -> Option<u64> {
        let need = round_up(bytes.checked_add(HEADER)?, GRAIN).max(MIN_BLOCK);
        let align = align.max(GRAIN);
        let slack = if align > GRAIN { align + MIN_BLOCK } else { 0 };
        let b = self.find(need.checked_add(slack)?)?;
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

    fn free(&mut self, payload: u64) {
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

    fn usable(&self, payload: u64) -> u64 {
        self.block_size(payload - HEADER) - HEADER
    }
}

/// The publisher side: this process's arena, read-write.
pub struct Arena {
    name: String,
    base: NonNull<u8>,
    size: usize,
    heap: PiMutex<Heap>,
    _fd: OwnedFd,
}

// SAFETY: the mapping is fixed for the process's life and the heap is behind its lock.
unsafe impl Sync for Arena {}
// SAFETY: as above.
unsafe impl Send for Arena {}

static ARENA: OnceLock<Option<Arena>> = OnceLock::new();

/// The process's arena, made on first use; None unless `FLUX_ARENA_BYTES` asks for one.
pub fn arena() -> Option<&'static Arena> {
    let a = ARENA.get_or_init(Arena::create).as_ref();
    if a.is_some() {
        attach_hook();
    }
    a
}

impl Arena {
    fn create() -> Option<Arena> {
        let bytes: usize = std::env::var("FLUX_ARENA_BYTES").ok()?.parse().ok()?;
        let size = bytes.min(MAX_ARENA_BYTES) & !4095;
        if size == 0 {
            return None;
        }
        sweep_stale();
        let name = format!("{PREFIX}{}_{:016x}", sys::getpid(), random());
        let fd = sys::shm_open(&name, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR).ok()?;
        let len = libc::off_t::try_from(size).ok()?;
        // Sparse: pages are backed as they are touched, not when the arena is made.
        // SAFETY: plain syscall on a descriptor we hold.
        if unsafe { libc::ftruncate(fd.as_raw_fd(), len) } != 0 {
            sys::shm_unlink(&name);
            return None;
        }
        let Ok(base) = sys::map_shared(fd.as_raw_fd(), size) else {
            sys::shm_unlink(&name);
            return None;
        };
        Some(Arena {
            heap: PiMutex::new(Heap::new(base, size as u64)),
            name,
            base,
            size,
            _fd: fd,
        })
    }

    pub fn alloc(&self, bytes: usize, align: usize) -> Option<u64> {
        if !align.is_power_of_two() {
            return None;
        }
        self.heap.lock().alloc(bytes as u64, align as u64)
    }

    pub fn free(&self, off: u64) {
        self.heap.lock().free(off);
    }

    pub fn ptr(&self, off: u64) -> *mut u8 {
        #[allow(clippy::cast_possible_truncation)]
        // SAFETY: offsets handed out are inside the mapping.
        unsafe {
            self.base.as_ptr().add(off as usize)
        }
    }

    /// The arena offset of `bytes` at `p`, if they are all inside the arena.
    pub fn offset_of(&self, p: *const u8, bytes: usize) -> Option<u64> {
        let off = (p as usize).checked_sub(self.base.as_ptr() as usize)?;
        (off <= self.size && bytes <= self.size - off).then_some(off as u64)
    }

    pub fn base(&self) -> *mut u8 {
        self.base.as_ptr()
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn trailer(&self) -> [u8; TRAILER_BYTES] {
        let mut t = [0u8; TRAILER_BYTES];
        t[..MAGIC.len()].copy_from_slice(&MAGIC);
        let n = self.name.len().min(NAME_BYTES - 1);
        t[MAGIC.len()..MAGIC.len() + n].copy_from_slice(&self.name.as_bytes()[..n]);
        t
    }
}

fn random() -> u64 {
    let mut b = [0u8; 8];
    // SAFETY: b is a valid buffer of its length.
    let n = unsafe { libc::getrandom(b.as_mut_ptr().cast(), b.len(), 0) };
    if n == 8 {
        u64::from_ne_bytes(b)
    } else {
        sys::monotonic_ns()
    }
}

/// Arenas of dead processes: nothing else unlinks them.
fn sweep_stale() {
    let Ok(dir) = std::fs::read_dir("/dev/shm") else {
        return;
    };
    for e in dir.flatten() {
        let name = e.file_name();
        let Some(rest) = name.to_str().and_then(|n| n.strip_prefix(&PREFIX[1..])) else {
            continue;
        };
        let Some(pid) = rest
            .split('_')
            .next()
            .and_then(|p| p.parse::<libc::pid_t>().ok())
        else {
            continue;
        };
        // SAFETY: signal 0 only probes.
        let dead = unsafe { libc::kill(pid, 0) } != 0
            && std::io::Error::last_os_error().raw_os_error() == Some(libc::ESRCH);
        if dead {
            sys::shm_unlink(&format!("/{rest}", rest = name.to_string_lossy()));
        }
    }
}

/// A subscriber's read-only mapping, kept for the process's life.
struct Mapping {
    name: [u8; NAME_BYTES],
    base: usize,
    size: usize,
}

static MAPPINGS: PiMutex<Vec<Mapping>> = PiMutex::new(Vec::new());

/// The arena a frame's trailer names, mapped read-only. None for a frame without one, and for a
/// name that is not an arena this user owns.
pub fn resolve(frame: &[u8]) -> Option<&'static [u8]> {
    let t = frame.get(frame.len().checked_sub(TRAILER_BYTES)?..)?;
    if t.get(..MAGIC.len())? != MAGIC {
        return None;
    }
    let mut key = [0u8; NAME_BYTES];
    key.copy_from_slice(t.get(MAGIC.len()..)?);
    let name = CStr::from_bytes_until_nul(&key).ok()?.to_str().ok()?;
    let id = name.strip_prefix(PREFIX)?;
    if id.is_empty() || !id.bytes().all(|c| c.is_ascii_hexdigit() || c == b'_') {
        return None;
    }
    let mut maps = MAPPINGS.lock();
    if let Some(m) = maps.iter().find(|m| m.name == key) {
        // SAFETY: the mapping lives for the process.
        return Some(unsafe { std::slice::from_raw_parts(m.base as *const u8, m.size) });
    }
    let fd = sys::shm_open(name, libc::O_RDONLY).ok()?;
    let st = sys::fstat(fd.as_raw_fd()).ok()?;
    if !st.ours() || st.size == 0 {
        return None;
    }
    let base = sys::map_readonly(fd.as_raw_fd(), st.size).ok()?;
    maps.push(Mapping {
        name: key,
        base: base.as_ptr() as usize,
        size: st.size,
    });
    // SAFETY: just mapped, never unmapped.
    Some(unsafe { std::slice::from_raw_parts(base.as_ptr(), st.size) })
}

// The malloc hook, when `libflux_heaphook.so` is preloaded. It owns the per-thread scope count;
// this side hands it the allocator.
type AttachFn = unsafe extern "C" fn(
    unsafe extern "C" fn(usize, usize) -> *mut c_void,
    unsafe extern "C" fn(*mut c_void),
    unsafe extern "C" fn(*mut c_void) -> usize,
    *mut c_void,
    usize,
);
type ScopeFn = unsafe extern "C" fn();

fn hook_symbol(name: &CStr) -> Option<*mut c_void> {
    // SAFETY: dlsym with a valid C string.
    let p = unsafe { libc::dlsym(libc::RTLD_DEFAULT, name.as_ptr()) };
    (!p.is_null()).then_some(p)
}

fn attach_hook() {
    static ATTACHED: OnceLock<()> = OnceLock::new();
    ATTACHED.get_or_init(|| {
        let (Some(a), Some(sym)) = (
            ARENA.get().and_then(Option::as_ref),
            hook_symbol(c"flux_heaphook_attach"),
        ) else {
            return;
        };
        // SAFETY: the hook exports this symbol with this signature.
        let attach: AttachFn = unsafe { std::mem::transmute(sym) };
        // SAFETY: the functions and range stay valid for the process.
        unsafe {
            attach(
                hook_alloc,
                hook_free,
                hook_usable,
                a.base().cast(),
                a.size(),
            );
        };
    });
}

unsafe extern "C" fn hook_alloc(bytes: usize, align: usize) -> *mut c_void {
    ARENA
        .get()
        .and_then(Option::as_ref)
        .and_then(|a| a.alloc(bytes, align).map(|off| a.ptr(off).cast()))
        .unwrap_or(std::ptr::null_mut())
}

unsafe extern "C" fn hook_free(p: *mut c_void) {
    if let Some(a) = ARENA.get().and_then(Option::as_ref)
        && let Some(off) = a.offset_of(p.cast(), 0)
    {
        a.free(off);
    }
}

unsafe extern "C" fn hook_usable(p: *mut c_void) -> usize {
    let Some(a) = ARENA.get().and_then(Option::as_ref) else {
        return 0;
    };
    #[allow(clippy::cast_possible_truncation)]
    a.offset_of(p.cast(), 0)
        .map_or(0, |off| a.heap.lock().usable(off) as usize)
}

/// Opens a scope on this thread: its allocations go to the arena until `scope_leave`. False when
/// the hook is not loaded or there is no arena.
pub fn scope_enter() -> bool {
    static ENTER: OnceLock<Option<usize>> = OnceLock::new();
    if arena().is_none() {
        return false;
    }
    let Some(f) = *ENTER.get_or_init(|| hook_symbol(c"flux_heaphook_enter").map(|p| p as usize))
    else {
        return false;
    };
    // SAFETY: the hook exports this symbol with this signature.
    let enter = unsafe { std::mem::transmute::<usize, ScopeFn>(f) };
    // SAFETY: no preconditions.
    unsafe { enter() };
    true
}

pub fn scope_leave() {
    static LEAVE: OnceLock<Option<usize>> = OnceLock::new();
    if let Some(f) = *LEAVE.get_or_init(|| hook_symbol(c"flux_heaphook_leave").map(|p| p as usize))
    {
        // SAFETY: the hook exports this symbol with this signature.
        let leave = unsafe { std::mem::transmute::<usize, ScopeFn>(f) };
        // SAFETY: no preconditions.
        unsafe { leave() };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn heap(bytes: usize) -> (Vec<u64>, Heap) {
        let mut mem = vec![0u64; bytes / 8];
        let base = NonNull::new(mem.as_mut_ptr().cast::<u8>()).unwrap();
        (mem, Heap::new(base, bytes as u64))
    }

    #[test]
    fn blocks_do_not_overlap_and_coalesce_back_to_one() {
        let (_mem, mut h) = heap(1 << 16);
        let mut got = Vec::new();
        for (i, n) in [1u64, 17, 100, 4000, 64, 3, 900].iter().enumerate() {
            let align = if i % 2 == 0 { 16 } else { 64 };
            let p = h.alloc(*n, align).unwrap();
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
        let a = h.alloc(3000, 16).unwrap();
        assert!(h.alloc(3000, 16).is_none());
        h.free(a);
        assert!(h.alloc(4000, 16).is_some());
    }

    #[test]
    fn a_trailer_names_an_arena_only_by_its_prefix() {
        let mut frame = vec![0u8; 128];
        frame[64..80].copy_from_slice(&MAGIC);
        frame[80..92].copy_from_slice(b"/etc/passwd\0");
        assert!(resolve(&frame).is_none());
        assert!(resolve(&frame[..10]).is_none());
    }
}
