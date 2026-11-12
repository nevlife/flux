//! The process's arena: one shm file a frame's fields can live in past its slot.
//!
//! A frame names arena bytes by offset, never by pointer, so a subscriber maps the file wherever
//! it lands and a Python reader reads it as it reads a slot. A descriptor offset at or past
//! `HEAP_BASE` and below `GPU_BASE` is `HEAP_BASE` + an arena offset in `UNIT`s; the frame ends
//! in a trailer naming the file. Offsets from `GPU_BASE` up are the GPU arena's (`gpu::arena`).
//! Readers work in positions, bytes from 0: a slot's below `HEAP_POS`, the arena's from it, the
//! GPU arena's from `GPU_POS`. `position` and `wire` convert.
//! The publisher frees what a frame owns when it claims that frame's slot again, so the slot's
//! reader counts guard the arena bytes too.
//!
//! On unless `FLUX_ARENA_BYTES` is 0, and made by the first frame that needs it. The file is
//! sized at once but its pages are reserved as the blocks reach them. Allocation takes no heap
//! memory of its own, so a malloc hook (`flux_heaphook`) can route a thread's allocations here.

#![allow(
    clippy::disallowed_types,
    reason = "the memory policy bits, Relaxed only"
)]

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

mod heap;

pub(crate) use heap::Heap;

pub const HEAP_BASE: u64 = 1 << 31;
pub const GPU_BASE: u64 = 3 << 30;
/// Descriptors are u32: the arena has the third quarter of their range, the GPU arena the fourth,
/// each value one `UNIT`. Every block starts on a `UNIT` boundary (the allocator's grain).
pub const UNIT: u64 = 16;
pub const MAX_ARENA_BYTES: usize = 16 << 30;
/// The GPU arena reserves this much address space; chunks are mapped into it as blocks need them.
pub const MAX_GPU_ARENA_BYTES: usize = MAX_ARENA_BYTES;
pub const HEAP_POS: u64 = HEAP_BASE;
pub const GPU_POS: u64 = HEAP_POS + (GPU_BASE - HEAP_BASE) * UNIT;

/// The position a descriptor offset names.
pub fn position(wire: u32) -> u64 {
    let w = u64::from(wire);
    if w >= GPU_BASE {
        GPU_POS + (w - GPU_BASE) * UNIT
    } else if w >= HEAP_BASE {
        HEAP_POS + (w - HEAP_BASE) * UNIT
    } else {
        w
    }
}

/// The descriptor offset of a position; None for an arena position off a `UNIT` boundary or
/// past the range.
pub fn wire(pos: u64) -> Option<u32> {
    let (base, at) = if pos >= GPU_POS {
        (GPU_BASE, pos - GPU_POS)
    } else if pos >= HEAP_POS {
        (HEAP_BASE, pos - HEAP_POS)
    } else {
        return u32::try_from(pos).ok();
    };
    if !at.is_multiple_of(UNIT) {
        return None;
    }
    u32::try_from(base + at / UNIT).ok()
}
/// A frame with arena blocks ends in the GPU arena's trailer when it has GPU blocks, then this
/// one: its magic, the arena's name, and the list of the blocks, by descriptor offset and length.
/// The list is in the frame before the trailers when it fits the slot, else in the arena.
pub const TRAILER_BYTES: usize = 64;
pub const GPU_TRAILER_BYTES: usize = 2 * TRAILER_BYTES;
/// One entry of the list: the block's descriptor offset and length, each a little-endian u32.
pub const BLOCK_BYTES: usize = 8;
const MAGIC: [u8; 16] = *b"flux-arena-v2\0\0\0";
/// The same trailer, after a GPU arena's.
const MAGIC_GPU: [u8; 16] = *b"flux-arena-g2\0\0\0";
const NAME_BYTES: usize = 40;
const LIST_AT: usize = TRAILER_BYTES - 8;
/// The creator holds a read lock on it while it lives; the sweep unlinks a file nobody holds.
pub(crate) const LOCK_BYTE: i64 = 0;
/// Pages are reserved this far at a time, ahead of the highest block, so a full `/dev/shm` fails
/// an allocation instead of raising SIGBUS on the first store into a page it cannot back.
const COMMIT_STEP: u64 = 4 << 20;

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

/// Reserves the file's pages for `[from, to)` and applies the publishers' policy to them.
fn commit_file(fd: RawFd, base: NonNull<u8>, from: u64, to: u64) -> io::Result<()> {
    #[allow(clippy::cast_possible_truncation)]
    let (from, len) = (from as usize, (to - from) as usize);
    sys::allocate_range(fd, from, len)?;
    // SAFETY: from < to <= the mapping's size.
    policy_apply(unsafe { base.add(from) }, len, policy())
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
            let heap = match Heap::new(
                base,
                size as u64,
                COMMIT_STEP,
                Some(&mut |from, to| commit_file(raw, base, from, to)),
            ) {
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
        let (fd, base) = (self.live.raw(), self.base);
        self.heap
            .lock()
            .alloc(bytes as u64, align as u64, &mut |from, to| {
                commit_file(fd, base, from, to)
            })
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
}

/// The trailer naming `arena`, or no arena (a frame with only GPU blocks), and the list of
/// `blocks` at descriptor offset `list`; `gpu` when a GPU arena's trailer comes before it.
pub fn trailer(arena: Option<&Arena>, gpu: bool, list: u32, blocks: u32) -> [u8; TRAILER_BYTES] {
    let mut t = [0u8; TRAILER_BYTES];
    t[..MAGIC.len()].copy_from_slice(if gpu { &MAGIC_GPU } else { &MAGIC });
    if let Some(a) = arena {
        let n = a.name.len().min(NAME_BYTES - 1);
        t[MAGIC.len()..MAGIC.len() + n].copy_from_slice(&a.name.as_bytes()[..n]);
    }
    t[LIST_AT..LIST_AT + 4].copy_from_slice(&list.to_le_bytes());
    t[LIST_AT + 4..].copy_from_slice(&blocks.to_le_bytes());
    t
}

fn has_magic(t: &[u8]) -> bool {
    t.get(..MAGIC.len())
        .is_some_and(|m| m == MAGIC || m == MAGIC_GPU)
}

/// The list of the blocks `frame` owns, in either arena, as its trailer gives it: in the frame
/// before its trailers, or in `heap`, the arena the trailer names.
pub fn blocks<'a>(frame: &'a [u8], heap: Option<&'a [u8]>) -> Option<&'a [u8]> {
    let t = frame.get(frame.len().checked_sub(TRAILER_BYTES)?..)?;
    if !has_magic(t) {
        return None;
    }
    let u32_at = |i: usize| u32::from_le_bytes(t[i..i + 4].try_into().unwrap_or_default());
    let at = u64::from(u32_at(LIST_AT));
    let len = BLOCK_BYTES.checked_mul(u32_at(LIST_AT + 4) as usize)?;
    if at < HEAP_BASE {
        let behind = if t[..MAGIC.len()] == MAGIC_GPU {
            GPU_TRAILER_BYTES
        } else {
            TRAILER_BYTES
        };
        let at = usize::try_from(at).ok()?;
        let end = at.checked_add(len)?;
        return (end <= frame.len().checked_sub(behind)?).then(|| &frame[at..end]);
    }
    if at >= GPU_BASE {
        return None;
    }
    let at = usize::try_from((at - HEAP_BASE) * UNIT).ok()?;
    heap?.get(at..at.checked_add(len)?)
}

/// The entries of a block `list` whose positions are in `from..to`, as (position, length); None
/// when one of them runs past `size` bytes from `from`.
pub fn entries_in(
    list: &[u8],
    from: u64,
    to: u64,
    size: usize,
) -> Option<impl Iterator<Item = (u64, u64)> + '_> {
    let u32_at = |b: [u8; 4]| u32::from_le_bytes(b);
    let mine = list
        .as_chunks::<BLOCK_BYTES>()
        .0
        .iter()
        .map(move |&[o0, o1, o2, o3, l0, l1, l2, l3]| {
            (
                position(u32_at([o0, o1, o2, o3])),
                u64::from(u32_at([l0, l1, l2, l3])),
            )
        })
        .filter(move |&(off, _)| (from..to).contains(&off));
    let fits = mine
        .clone()
        .all(|(off, len)| off - from + len <= size as u64);
    fits.then_some(mine)
}

/// `frame` without its arena trailers (the GPU arena's too): the frame itself when it has none.
/// None for a GPU trailer the frame is too short to hold.
pub fn body(frame: &[u8]) -> Option<&[u8]> {
    let Some(at) = frame.len().checked_sub(TRAILER_BYTES) else {
        return Some(frame);
    };
    match &frame[at..at + MAGIC.len()] {
        m if *m == MAGIC => Some(&frame[..at]),
        m if *m == MAGIC_GPU => frame.get(..frame.len().checked_sub(GPU_TRAILER_BYTES)?),
        _ => Some(frame),
    }
}

/// The GPU arena's trailer in `frame`, if its own says one comes before it.
pub fn gpu_trailer(frame: &[u8]) -> Option<&[u8]> {
    let at = frame.len().checked_sub(GPU_TRAILER_BYTES)?;
    let t = frame.get(at..)?;
    (t[TRAILER_BYTES..TRAILER_BYTES + MAGIC_GPU.len()] == MAGIC_GPU).then(|| &t[..TRAILER_BYTES])
}

/// A clean exit leaves no file. Subscribers keep their mappings; a crash leaves it to the sweep.
extern "C" fn unlink_at_exit() {
    if let Some(a) = existing().filter(|a| a.live.owned_here()) {
        sys::shm_unlink(&a.name);
    }
}

pub(crate) fn random() -> u64 {
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
            .is_some_and(|t| has_magic(t) && t[MAGIC.len()..MAGIC.len() + NAME_BYTES] == self.name)
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
    if !has_magic(t) {
        return None;
    }
    let mut key = [0u8; NAME_BYTES];
    key.copy_from_slice(t.get(MAGIC.len()..MAGIC.len() + NAME_BYTES)?);
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
        let raw = fd.as_raw_fd();
        let mut grow = |from, to| commit_file(raw, base, from, to);
        let mut h = Heap::new(base, size as u64, COMMIT_STEP, Some(&mut grow)).unwrap();
        assert_eq!(h.committed, COMMIT_STEP);
        let a = h.alloc(COMMIT_STEP + 1, 16, &mut grow).unwrap();
        assert_eq!(h.committed, 2 * COMMIT_STEP);
        // A descriptor that cannot reserve: every later reservation fails.
        let mut broken = |from, to| commit_file(-1, base, from, to);
        assert!(h.alloc(2 * COMMIT_STEP, 16, &mut broken).is_none());
        assert_eq!(h.committed, 2 * COMMIT_STEP);
        let b = h.alloc(1000, 16, &mut broken).unwrap();
        h.free(a);
        h.free(b);
        assert_eq!(h.lists_in_use(), 1);
        // SAFETY: mapped above; nothing points into it any more.
        unsafe { sys::munmap(base, size) };
    }

    #[test]
    fn positions_and_descriptor_offsets_convert_both_ways() {
        let gib = 1u64 << 30;
        for pos in [
            0,
            4096,
            HEAP_POS,
            HEAP_POS + UNIT,
            HEAP_POS + gib,
            HEAP_POS + 16 * gib - UNIT,
            GPU_POS,
            GPU_POS + 256,
        ] {
            let w = wire(pos).expect("a descriptor offset");
            assert_eq!(position(w), pos, "{pos:#x}");
        }
        assert_eq!(wire(HEAP_POS + 8), None, "off a UNIT boundary");
        assert_eq!(wire(GPU_POS + 16 * gib), None, "past u32");
        assert_eq!(
            position(u32::try_from(HEAP_BASE).unwrap() - 1),
            HEAP_BASE - 1
        );
    }

    /// An arena past 1 GiB: a block there has a descriptor offset, and a list there is found and
    /// read back as positions. The memory is sparse, so only the touched pages are used.
    #[test]
    fn a_block_and_a_list_past_one_gib_are_named_and_found() {
        use std::os::fd::{FromRawFd, OwnedFd};
        let size = 2usize << 30;
        // SAFETY: plain syscall.
        let raw = unsafe { libc::memfd_create(c"arena_far".as_ptr(), 0) };
        assert!(raw >= 0);
        // SAFETY: a fresh descriptor, owned here.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        let len = libc::off_t::try_from(size).unwrap();
        // SAFETY: plain syscall on a descriptor we hold.
        assert_eq!(unsafe { libc::ftruncate(fd.as_raw_fd(), len) }, 0);
        let base = sys::map_shared(fd.as_raw_fd(), size).unwrap();
        let mut grow = |_, _| Ok(());
        let mut h = Heap::new(base, size as u64, COMMIT_STEP, Some(&mut grow)).unwrap();
        let big = h.alloc(3 << 29, 16, &mut grow).unwrap();
        let far = h.alloc(64, 16, &mut grow).unwrap();
        assert!(far > 1 << 30, "{far:#x}");
        let w = wire(HEAP_POS + far).expect("a descriptor offset past 1 GiB");
        assert_eq!(position(w), HEAP_POS + far);
        // A list of one entry naming `far` itself, in the arena at `far`.
        let mut entry = [0u8; BLOCK_BYTES];
        entry[..4].copy_from_slice(&w.to_le_bytes());
        entry[4..].copy_from_slice(&64u32.to_le_bytes());
        let far_at = usize::try_from(far).unwrap();
        // SAFETY: far has 64 bytes in the mapping.
        let dst = unsafe { base.as_ptr().add(far_at) };
        // SAFETY: as above; nothing else writes there.
        unsafe { std::ptr::copy_nonoverlapping(entry.as_ptr(), dst, BLOCK_BYTES) };
        let mut frame = vec![0u8; 128];
        frame[64..].copy_from_slice(&trailer(None, false, w, 1));
        // SAFETY: mapped above with size; read only below.
        let heap = unsafe { std::slice::from_raw_parts(base.as_ptr(), size) };
        let list = blocks(&frame, Some(heap)).expect("the list past 1 GiB");
        assert_eq!(list, &entry);
        let got: Vec<_> = entries_in(list, HEAP_POS, GPU_POS, size).unwrap().collect();
        assert_eq!(got, [(HEAP_POS + far, 64)]);
        h.free(far);
        h.free(big);
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
