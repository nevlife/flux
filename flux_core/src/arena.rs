//! The process's arena: one shm file a frame's fields can live in past its slot.
//!
//! A frame names arena bytes by offset, never by pointer, so a subscriber maps the file wherever
//! it lands and a Python reader reads it as it reads a slot. A descriptor offset at or past
//! `HEAP_BASE` is `HEAP_BASE` + an arena offset; the frame ends in a trailer naming the file.
//! The publisher frees what a frame owns when it claims that frame's slot again, so the slot's
//! reader counts guard the arena bytes too.
//!
//! On unless `FLUX_ARENA_BYTES` is 0, and made by the first frame that needs it. The file is
//! sized at once but its pages are reserved as the blocks reach them. Allocation takes no heap
//! memory of its own, so a malloc hook (`flux_heaphook`) can route a thread's allocations here.

use std::ffi::{CStr, c_void};
use std::io;
use std::os::fd::{AsRawFd, RawFd};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, OnceLock, Weak};

use crate::discovery::name_still_bound;
use crate::error::{Error, Result};
use crate::layout::LAYOUT_VERSION;
use crate::qos::MemoryPolicy;
use crate::sys::{self, LiveFd, PiMutex};

pub const HEAP_BASE: u64 = 1 << 31;
/// Descriptors are u32 and the arena has the upper half.
pub const MAX_ARENA_BYTES: usize = 1 << 31;
pub const TRAILER_BYTES: usize = 64;
const MAGIC: [u8; 16] = *b"flux-arena-v1\0\0\0";
const NAME_BYTES: usize = TRAILER_BYTES - MAGIC.len();
/// The creator holds a read lock on it while it lives; the sweep unlinks a file nobody holds.
pub(crate) const LOCK_BYTE: i64 = 0;
/// Pages are reserved this far at a time, ahead of the highest block, so a full `/dev/shm` fails
/// an allocation instead of raising SIGBUS on the first store into a page it cannot back.
const COMMIT_STEP: u64 = 4 << 20;

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

/// `flux.v<layout>.arena-<16 hex>`: a key cannot hold '-', so no channel name is one.
fn prefix() -> &'static str {
    static PREFIX: OnceLock<String> = OnceLock::new();
    PREFIX.get_or_init(|| format!("flux.v{LAYOUT_VERSION}.arena-"))
}

pub(crate) fn is_arena_entry(entry: &str) -> bool {
    entry
        .strip_prefix(prefix())
        .is_some_and(|id| id.len() == 16 && id.bytes().all(|b| b.is_ascii_hexdigit()))
}

/// Segregated free lists over boundary-tagged blocks, kept in the arena itself.
struct Heap {
    base: NonNull<u8>,
    size: u64,
    heads: [u64; BINS],
    nonempty: u32,
    /// Bytes from the start whose pages are reserved. Nothing past it is written.
    committed: u64,
    /// Where `committed` grows; None for memory that is all there.
    file: Option<RawFd>,
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
    fn new(base: NonNull<u8>, size: u64, file: Option<RawFd>) -> io::Result<Heap> {
        let size = size / GRAIN * GRAIN;
        let mut h = Heap {
            base,
            size,
            heads: [NIL; BINS],
            nonempty: 0,
            committed: if file.is_some() { 0 } else { size },
            file,
        };
        h.reach(MIN_BLOCK)?;
        h.make_free(0, h.size, true);
        Ok(h)
    }

    /// Reserves pages up to `end`, under the policy the publishers asked for.
    fn reach(&mut self, end: u64) -> io::Result<()> {
        let Some(fd) = self.file.filter(|_| end > self.committed) else {
            return Ok(());
        };
        let to = round_up(end, COMMIT_STEP).min(self.size);
        #[allow(clippy::cast_possible_truncation)]
        let (from, len) = (self.committed as usize, (to - self.committed) as usize);
        sys::allocate_range(fd, from, len)?;
        // SAFETY: from < to <= size, inside the mapping.
        policy_apply(unsafe { self.base.add(from) }, len, policy())?;
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
    /// its pages cannot be reserved.
    fn alloc(&mut self, bytes: u64, align: u64) -> Option<u64> {
        let need = round_up(bytes.checked_add(HEADER)?, GRAIN).max(MIN_BLOCK);
        let align = align.max(GRAIN);
        let slack = if align > GRAIN { align + MIN_BLOCK } else { 0 };
        let b = self.find(need.checked_add(slack)?)?;
        // The block, its split-off tail's header and links: every byte this call writes.
        self.reach((b + slack + need + MIN_BLOCK).min(self.size))
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

// The union of what publishers asked for, never undone, as a segment mapping's is.
const PRECOMMIT: u8 = 1;
const LOCK: u8 = 2;
static POLICY: AtomicU8 = AtomicU8::new(0);

fn policy() -> u8 {
    POLICY.load(Ordering::Relaxed)
}

fn policy_apply(p: NonNull<u8>, len: usize, policy: u8) -> io::Result<()> {
    const MADV_POPULATE_WRITE: libc::c_int = 23;
    let failed = if policy & LOCK != 0 {
        // SAFETY: the caller passes a range inside the mapping.
        unsafe { libc::mlock(p.as_ptr().cast(), len) != 0 }
    } else if policy & PRECOMMIT != 0 {
        // SAFETY: as above.
        unsafe { libc::madvise(p.as_ptr().cast(), len, MADV_POPULATE_WRITE) != 0 }
    } else {
        false
    };
    if failed {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

/// A publisher's `MemoryPolicy`, for the arena pages its frames may use: the ones reserved so far
/// now, later ones as they come.
pub(crate) fn request(mem: MemoryPolicy) -> Result<()> {
    let bits = if mem.lock { LOCK } else { 0 } | if mem.precommit { PRECOMMIT } else { 0 };
    let old = POLICY.fetch_or(bits, Ordering::Relaxed);
    let Some(a) = existing().filter(|_| old | bits != old) else {
        return Ok(());
    };
    let h = a.heap.lock();
    #[allow(clippy::cast_possible_truncation)]
    policy_apply(a.base, h.committed as usize, old | bits)
        .map_err(|e| Error::io("mlock or madvise of the arena", &a.name, e))
}

/// The publisher side: this process's arena, read-write.
pub struct Arena {
    name: String,
    base: NonNull<u8>,
    size: usize,
    heap: PiMutex<Heap>,
    live: LiveFd,
    born: u32,
}

// SAFETY: the mapping is fixed for the process's life and the heap is behind its lock.
unsafe impl Sync for Arena {}
// SAFETY: as above.
unsafe impl Send for Arena {}

static ARENA: OnceLock<Option<Arena>> = OnceLock::new();

/// The arena size `FLUX_ARENA_BYTES` asks for, `MAX_ARENA_BYTES` when unset; None for 0 or a
/// value that is not a number.
pub fn configured() -> Option<usize> {
    static SIZE: OnceLock<Option<usize>> = OnceLock::new();
    *SIZE.get_or_init(|| {
        let bytes = match std::env::var("FLUX_ARENA_BYTES") {
            Err(_) => MAX_ARENA_BYTES,
            Ok(v) => v.trim().parse::<usize>().ok()?,
        };
        Some(bytes.min(MAX_ARENA_BYTES) & !4095).filter(|&n| n > 0)
    })
}

/// The process's arena, made on first use. None when it is off or could not be made, and in a
/// forked child, whose copy of the allocator state would race its parent's.
pub fn arena() -> Option<&'static Arena> {
    configured()?;
    let a = ARENA.get_or_init(Arena::create).as_ref()?;
    attach_hook();
    (a.born == sys::fork_gen()).then_some(a)
}

/// The arena if one was made, without making it.
pub fn existing() -> Option<&'static Arena> {
    ARENA.get()?.as_ref().filter(|a| a.born == sys::fork_gen())
}

impl Arena {
    fn create() -> Option<Arena> {
        let size = configured()?;
        Arena::open(size)
            .inspect_err(|e| {
                eprintln!("flux: no arena, so frames larger than their slot are refused: {e}");
            })
            .ok()
    }

    fn open(size: usize) -> io::Result<Arena> {
        sys::watch_fork();
        for _ in 0..8 {
            let name = format!("/{}{:016x}", prefix(), random());
            let live = match LiveFd::open(|| {
                sys::shm_open(&name, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR)
            }) {
                Ok(live) => live,
                Err(e) if e.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(e) => return Err(e),
            };
            let raw = live.raw();
            // A sweeper may unlink a fresh file before it is locked.
            if !sys::lock_read(raw, LOCK_BYTE) || !name_still_bound(&name, raw) {
                continue;
            }
            let fail = |e: io::Error| {
                sys::shm_unlink(&name);
                e
            };
            let len = libc::off_t::try_from(size).map_err(|_| io::ErrorKind::InvalidInput)?;
            // SAFETY: plain syscall on a descriptor we hold.
            if unsafe { libc::ftruncate(raw, len) } != 0 {
                return Err(fail(io::Error::last_os_error()));
            }
            let base = sys::map_shared(raw, size).map_err(&fail)?;
            let heap = match Heap::new(base, size as u64, Some(raw)) {
                Ok(h) => h,
                Err(e) => {
                    // SAFETY: mapped above with size; nothing points into it yet.
                    unsafe { sys::munmap(base, size) };
                    return Err(fail(e));
                }
            };
            // SAFETY: registers a plain function.
            let registered = unsafe { libc::atexit(unlink_at_exit) } == 0;
            // It fails only for want of memory, and then the sweep unlinks the file after exit.
            let _ = registered;
            return Ok(Arena {
                name,
                base,
                size,
                heap: PiMutex::new(heap),
                live,
                born: sys::fork_gen(),
            });
        }
        Err(io::Error::other("no free arena name"))
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

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn trailer(&self) -> [u8; TRAILER_BYTES] {
        let mut t = [0u8; TRAILER_BYTES];
        t[..MAGIC.len()].copy_from_slice(&MAGIC);
        let n = self.name.len().min(NAME_BYTES - 1);
        t[MAGIC.len()..MAGIC.len() + n].copy_from_slice(&self.name.as_bytes()[..n]);
        t
    }
}

/// A clean exit leaves no file. Subscribers keep their mappings; a crash leaves it to the sweep.
extern "C" fn unlink_at_exit() {
    if let Some(a) = existing().filter(|a| a.live.owned_here()) {
        sys::shm_unlink(&a.name);
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

/// A subscriber's read-only mapping of a publisher's arena. It goes when the last holder does:
/// a channel that read through it, a binding's reader.
pub struct ArenaMap {
    name: [u8; NAME_BYTES],
    base: NonNull<u8>,
    size: usize,
}

// SAFETY: a read-only mapping, fixed while it lives.
unsafe impl Send for ArenaMap {}
// SAFETY: as above.
unsafe impl Sync for ArenaMap {}

impl ArenaMap {
    pub fn bytes(&self) -> &[u8] {
        // SAFETY: mapped with size and unmapped only on drop.
        unsafe { std::slice::from_raw_parts(self.base.as_ptr(), self.size) }
    }

    /// Whether `frame`'s trailer names this arena. Per frame, so it only compares bytes.
    pub fn names(&self, frame: &[u8]) -> bool {
        frame
            .len()
            .checked_sub(TRAILER_BYTES)
            .and_then(|at| frame.get(at..))
            .is_some_and(|t| t[..MAGIC.len()] == MAGIC && t[MAGIC.len()..] == self.name)
    }
}

impl Drop for ArenaMap {
    fn drop(&mut self) {
        // SAFETY: mapped in map_for with size; no slice outlives self.
        unsafe { sys::munmap(self.base, self.size) };
    }
}

/// Shared by every holder in the process, as segment mappings are.
static MAPS: PiMutex<Vec<Weak<ArenaMap>>> = PiMutex::new(Vec::new());

/// The arena name in `frame`'s trailer, if it has one naming an arena of this layout.
fn trailer_name(frame: &[u8]) -> Option<[u8; NAME_BYTES]> {
    let t = frame.get(frame.len().checked_sub(TRAILER_BYTES)?..)?;
    if t.get(..MAGIC.len())? != MAGIC {
        return None;
    }
    let mut key = [0u8; NAME_BYTES];
    key.copy_from_slice(t.get(MAGIC.len()..)?);
    let name = CStr::from_bytes_until_nul(&key).ok()?.to_str().ok()?;
    is_arena_entry(name.strip_prefix('/')?).then_some(key)
}

/// The arena `frame`'s trailer names, mapped read-only. None for a frame without one, and for a
/// name that is not an arena this user owns.
pub fn map_for(frame: &[u8]) -> Option<Arc<ArenaMap>> {
    let key = trailer_name(frame)?;
    let mut maps = MAPS.lock();
    maps.retain(|w| w.strong_count() > 0);
    if let Some(m) = maps
        .iter()
        .filter_map(Weak::upgrade)
        .find(|m| m.name == key)
    {
        return Some(m);
    }
    let name = CStr::from_bytes_until_nul(&key).ok()?.to_str().ok()?;
    let fd = sys::shm_open(name, libc::O_RDONLY).ok()?;
    let st = sys::fstat(fd.as_raw_fd()).ok()?;
    if !st.ours() || st.size == 0 || st.size > MAX_ARENA_BYTES {
        return None;
    }
    let base = sys::map_readonly(fd.as_raw_fd(), st.size).ok()?;
    let m = Arc::new(ArenaMap {
        name: key,
        base,
        size: st.size,
    });
    maps.push(Arc::downgrade(&m));
    Some(m)
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
        let (Some(a), Some(sym)) = (existing(), hook_symbol(c"flux_heaphook_attach")) else {
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
    existing()
        .and_then(|a| a.alloc(bytes, align).map(|off| a.ptr(off).cast()))
        .unwrap_or(std::ptr::null_mut())
}

unsafe extern "C" fn hook_free(p: *mut c_void) {
    if let Some(a) = existing()
        && let Some(off) = a.offset_of(p.cast(), 0)
    {
        a.free(off);
    }
}

unsafe extern "C" fn hook_usable(p: *mut c_void) -> usize {
    let Some(a) = existing() else {
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
        (mem, Heap::new(base, bytes as u64, None).unwrap())
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

    /// The pages of a block are reserved before it is written; a block whose pages cannot be has
    /// no effect, and the rest of the arena still serves.
    #[test]
    fn a_block_past_the_reserved_pages_is_refused_whole() {
        use std::os::fd::{FromRawFd, OwnedFd};
        let size = usize::try_from(4 * COMMIT_STEP).unwrap();
        // SAFETY: plain syscall.
        let raw = unsafe { libc::memfd_create(c"arena_test".as_ptr(), 0) };
        assert!(raw >= 0);
        // SAFETY: a fresh descriptor, owned here.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        let len = libc::off_t::try_from(size).unwrap();
        // SAFETY: plain syscall on a descriptor we hold.
        assert_eq!(unsafe { libc::ftruncate(fd.as_raw_fd(), len) }, 0);
        let base = sys::map_shared(fd.as_raw_fd(), size).unwrap();
        let mut h = Heap::new(base, size as u64, Some(fd.as_raw_fd())).unwrap();
        assert_eq!(h.committed, COMMIT_STEP);
        let a = h.alloc(COMMIT_STEP + 1, 16).unwrap();
        assert_eq!(h.committed, 2 * COMMIT_STEP);
        // A descriptor that cannot reserve: every later reservation fails.
        h.file = Some(-1);
        assert!(h.alloc(2 * COMMIT_STEP, 16).is_none());
        assert_eq!(h.committed, 2 * COMMIT_STEP);
        let b = h.alloc(1000, 16).unwrap();
        h.free(a);
        h.free(b);
        assert_eq!(h.nonempty.count_ones(), 1);
        // SAFETY: mapped above; nothing points into it any more.
        unsafe { sys::munmap(base, size) };
    }

    #[test]
    fn a_trailer_names_an_arena_only_by_its_prefix() {
        let mut frame = vec![0u8; 128];
        frame[64..80].copy_from_slice(&MAGIC);
        frame[80..92].copy_from_slice(b"/etc/passwd\0");
        assert!(map_for(&frame).is_none());
        assert!(map_for(&frame[..10]).is_none());
        let name = format!("/{}{:016x}", prefix(), 7);
        frame[80..80 + name.len()].copy_from_slice(name.as_bytes());
        assert!(trailer_name(&frame).is_some());
    }

    /// Liveness by lock, not by pid: a pid can be another namespace's, or reused.
    #[test]
    fn the_sweep_unlinks_an_arena_nobody_holds_and_keeps_a_held_one() {
        let exists = |n: &str| std::path::Path::new(&format!("/dev/shm{n}")).exists();
        let dead = format!("/{}{:016x}", prefix(), random());
        drop(sys::shm_open(&dead, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR).unwrap());
        let held = format!("/{}{:016x}", prefix(), random());
        let live =
            LiveFd::open(|| sys::shm_open(&held, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR))
                .unwrap();
        assert!(sys::lock_read(live.raw(), LOCK_BYTE));
        crate::discovery::sweep_dead();
        assert!(!exists(&dead));
        assert!(exists(&held));
        sys::shm_unlink(&held);
    }

    #[test]
    fn arena_names_are_not_channel_names() {
        let n = format!("{}{:016x}", prefix(), 0xabc);
        assert!(is_arena_entry(&n));
        assert!(!crate::discovery::is_signpost_entry(&n));
        assert!(crate::manifest::parse_signpost(&n).is_none());
        assert!(!is_arena_entry(&format!("{}{:015x}", prefix(), 0xabc)));
    }
}
