//! Rendezvous through `/dev/shm`. A fixed-name signpost carries the epoch of the current segment,
//! and the segment is `<signpost>.e<epoch>`. One publisher per signpost. Every process that has a
//! signpost open holds a read lock on it; the last one out unlinks it.

#![allow(
    clippy::disallowed_types,
    reason = "the signpost words; orderings tagged `order:`"
)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use std::io;
use std::os::fd::{AsRawFd, OwnedFd, RawFd};
use std::os::unix::fs::MetadataExt;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Mutex, PoisonError};
use std::time::Duration;

use crate::arena;
use crate::error::{Error, Result};
use crate::gpu;
use crate::layout::{
    CACHE_LINE, ControlHeader, HeaderConfig, INIT_READY, LAYOUT_VERSION, MAGIC,
    PUBLISHER_LOCK_BYTE, SIGNPOST_BOOTSTRAP_BYTE, SIGNPOST_BYTES, SIGNPOST_MAGIC,
    SIGNPOST_PUBLISHER_BYTE, SIGNPOST_USER_BYTE, Signpost, SignpostPage, StorageKind,
    layout_config_ok,
};
use crate::segment::{Segment, SegmentId, checked_layout, init_header};
use crate::sys::{self, LiveFd};

pub const MAX_KEY_LEN: usize = 185;

const NAME_MAX: usize = 255;
const NAP: Duration = Duration::from_micros(100);
const BOOTSTRAP_SPINS: u32 = 5000;
const MAX_EPOCH_TRIES: u32 = 64;
const SKEW_PROBE_NS: u64 = 100_000_000;

const _: () = {
    // "flux.v<ver>." key ".<fp16>" ".e<epoch>", each at its widest.
    assert!(6 + 10 + 1 + MAX_KEY_LEN + 17 + 2 + 10 <= NAME_MAX);
};

/// Every byte but alnum and '_' becomes '.'. '_' stays so `/cam_left` and `/cam/left` differ.
pub fn flatten_key(key: &str) -> String {
    key.chars()
        .map(|c| if key_char(c) && c != '/' { c } else { '.' })
        .collect()
}

/// The bytes `flatten_key` keeps apart. ROS names use no others.
fn key_char(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_' || c == '/'
}

/// Layout version and fingerprint are both in the name: an object written under another value of
/// either is never opened. A key with a byte `flatten_key` would merge with '/' is refused, so two
/// keys never share a channel.
pub fn signpost_name(key: &str, fingerprint: u64) -> Result<String> {
    if key.len() > MAX_KEY_LEN {
        return Err(Error::InvalidArgument(format!(
            "channel key is {} characters, past the limit of {MAX_KEY_LEN}",
            key.len()
        )));
    }
    if !key.chars().all(key_char) {
        return Err(Error::InvalidArgument(format!(
            "channel key {key:?} has a character other than A-Z, a-z, 0-9, '_' and '/'"
        )));
    }
    Ok(format!(
        "/flux.v{LAYOUT_VERSION}.{}.{fingerprint:016x}",
        flatten_key(key)
    ))
}

pub fn segment_name(signpost: &str, epoch: u32) -> String {
    format!("{signpost}.e{epoch}")
}

pub(crate) fn is_signpost_entry(entry: &str) -> bool {
    entry
        .rsplit_once('.')
        .is_some_and(|(_, fp)| fp.len() == 16 && fp.bytes().all(|b| b.is_ascii_hexdigit()))
}

fn is_segment_entry(entry: &str) -> bool {
    entry
        .rsplit_once(".e")
        .is_some_and(|(_, ep)| !ep.is_empty() && ep.bytes().all(|b| b.is_ascii_digit()))
}

fn id_of(fd: RawFd) -> SegmentId {
    sys::fstat(fd)
        .map(|st| SegmentId {
            dev: st.dev,
            ino: st.ino,
        })
        .unwrap_or_default()
}

/// Called with a lock held on `fd`, which pins the binding: an unlinker needs the write lock.
/// By path, so it costs no descriptor: at the fd limit an open would fail and read as unbound.
pub(crate) fn name_still_bound(name: &str, fd: RawFd) -> bool {
    std::fs::metadata(format!("/dev/shm{name}")).is_ok_and(|m| {
        SegmentId {
            dev: m.dev(),
            ino: m.ino(),
        } == id_of(fd)
    })
}

/// An open signpost. Holds the user read lock for its lifetime, so no one unlinks it meanwhile.
pub(crate) struct SignpostMap {
    live: LiveFd,
    base: NonNull<u8>,
    name: String,
}

// SAFETY: a shared mapping read and written through atomics only.
unsafe impl Send for SignpostMap {}
// SAFETY: as above; &self hands out only atomics.
unsafe impl Sync for SignpostMap {}

impl SignpostMap {
    /// Creates and bootstraps the signpost if absent.
    pub(crate) fn open(name: &str) -> Result<SignpostMap> {
        for _ in 0..BOOTSTRAP_SPINS {
            let live = LiveFd::open(|| sys::shm_open(name, libc::O_CREAT | libc::O_RDWR))
                .map_err(|e| Error::io("signpost shm_open", name, e))?;
            let st = sys::fstat(live.raw()).map_err(|e| Error::io("signpost fstat", name, e))?;
            if !st.ours() {
                return Err(not_ours(name, &st));
            }
            // Refused: an unlinker holds the write lock. Unbound: it finished. Either way the name
            // is or will be another object.
            if sys::lock_read(live.raw(), SIGNPOST_USER_BYTE) && name_still_bound(name, live.raw())
            {
                return SignpostMap::bootstrap(name, live);
            }
            std::thread::sleep(NAP);
        }
        Err(Error::NotReady(format!(
            "signpost '{name}' kept being removed"
        )))
    }

    fn bootstrap(name: &str, live: LiveFd) -> Result<SignpostMap> {
        let raw = live.raw();
        let base = match SignpostMap::map(name, raw) {
            Ok(base) => base,
            Err(e) => {
                // As in drop: the last user out unlinks, here what this open may have created.
                leave(name, raw);
                return Err(e);
            }
        };
        let map = SignpostMap {
            live,
            base,
            name: name.to_owned(),
        };
        let sp = map.sp();
        // order: sp_init
        if sp.init_state.load(Ordering::Acquire) != INIT_READY {
            if sys::lock_write(raw, SIGNPOST_BOOTSTRAP_BYTE) {
                // order: sp_init
                if sp.init_state.load(Ordering::Acquire) != INIT_READY {
                    sp.magic.store(SIGNPOST_MAGIC, Ordering::Relaxed);
                    sp.version.store(LAYOUT_VERSION, Ordering::Relaxed);
                    sp.epoch.store(0, Ordering::Relaxed);
                    sp.init_state.store(INIT_READY, Ordering::Release); // order: sp_init
                }
                sys::unlock(raw, SIGNPOST_BOOTSTRAP_BYTE);
            } else {
                for _ in 0..BOOTSTRAP_SPINS {
                    // order: sp_init
                    if sp.init_state.load(Ordering::Acquire) == INIT_READY {
                        break;
                    }
                    std::thread::sleep(NAP);
                }
            }
        }
        // order: sp_init
        if sp.init_state.load(Ordering::Acquire) != INIT_READY {
            return Err(Error::NotReady(format!(
                "signpost '{name}' never became ready"
            )));
        }
        if sp.magic.load(Ordering::Relaxed) != SIGNPOST_MAGIC
            || sp.version.load(Ordering::Relaxed) != LAYOUT_VERSION
        {
            return Err(Error::Mismatch(format!(
                "'{name}' is not a flux v{LAYOUT_VERSION} signpost; remove /dev/shm{name} to recover"
            )));
        }
        Ok(map)
    }

    fn map(name: &str, raw: RawFd) -> Result<NonNull<u8>> {
        let st = sys::fstat(raw).map_err(|e| Error::io("signpost fstat", name, e))?;
        if st.size < SIGNPOST_BYTES {
            sys::allocate(raw, SIGNPOST_BYTES)
                .map_err(|e| Error::io("signpost fallocate", name, e))?;
        }
        sys::map_shared(raw, SIGNPOST_BYTES).map_err(|e| Error::io("signpost mmap", name, e))
    }

    pub(crate) fn fd(&self) -> &LiveFd {
        &self.live
    }

    pub(crate) fn name(&self) -> &str {
        &self.name
    }

    pub(crate) fn epoch(&self) -> u32 {
        self.sp().epoch.load(Ordering::Acquire) // order: sp_epoch
    }

    /// Woken by every publisher that stores a new epoch.
    pub(crate) fn epoch_word(&self) -> &AtomicU32 {
        &self.sp().epoch
    }

    fn sp(&self) -> &Signpost {
        &self.page().head
    }

    pub(crate) fn page(&self) -> &SignpostPage {
        // SAFETY: the mapping is SIGNPOST_BYTES long, zero-filled or initialized, and every
        // SignpostPage field is an atomic or padding.
        unsafe { self.base.cast::<SignpostPage>().as_ref() }
    }
}

impl Drop for SignpostMap {
    fn drop(&mut self) {
        // SAFETY: mapped in open with SIGNPOST_BYTES; no reference outlives self.
        unsafe { sys::munmap(self.base, SIGNPOST_BYTES) };
        if self.live.owned_here() {
            leave(&self.name, self.live.raw());
        }
    }
}

/// The last user out unlinks, under the write lock, before the fd closes. Upgrading the read lock
/// succeeds only if no other user holds one, so two users leaving at once would each see the
/// other's and leave the signpost behind: each lets go of its own and tries once more, and the
/// one that lets go last finds none. Meanwhile a sweeper may have removed the name and another
/// opener made it anew, so that try checks the name still binds this object.
fn leave(name: &str, raw: RawFd) {
    if sys::lock_write(raw, SIGNPOST_USER_BYTE) {
        sys::shm_unlink(name);
        return;
    }
    sys::unlock(raw, SIGNPOST_USER_BYTE);
    if sys::lock_write(raw, SIGNPOST_USER_BYTE) && name_still_bound(name, raw) {
        sys::shm_unlink(name);
    }
}

/// The returned segment holds the publisher locks for its lifetime and is unlinked on drop.
/// `device`: the ordinal to allocate the payload on, for the device-handle route. The segment then
/// holds the control plane only, and the allocation is served before the epoch advertises it.
/// The payload has memory for its first `ready` slots, the rest only once `add_slot` adds them.
#[allow(clippy::too_many_arguments)]
pub fn open_publisher_segment(
    name: &str,
    slot_size: u32,
    slot_count: u32,
    max_subscribers: u32,
    aux_size: u32,
    fingerprint: u64,
    device: Option<i32>,
    ready: u32,
) -> Result<Segment> {
    let layout = checked_layout(slot_size, slot_count, max_subscribers, aux_size)?;
    let device = match device {
        Some(ordinal) => Some((
            ordinal,
            gpu::device_uuid(ordinal)
                .ok_or_else(|| Error::Unsupported(format!("device {ordinal} has no UUID")))?,
        )),
        None => None,
    };
    let ready = ready.clamp(1, layout.slot_count);
    let bytes = if device.is_some() {
        layout.payload_offset()
    } else {
        layout.total_bytes()
    };
    let reserved = if device.is_some() {
        bytes
    } else {
        layout.payload_offset() + ready as usize * layout.payload_stride()
    };
    sweep_dead_once();

    let map = SignpostMap::open(name)?;
    if !sys::lock_write(map.fd().raw(), SIGNPOST_PUBLISHER_BYTE) {
        return Err(Error::PublisherExists(name.to_owned()));
    }
    let last = map.epoch();
    // A predecessor that crashed after this process swept. With the signpost lock held, no
    // segment under it has a live publisher.
    if last != 0 {
        remove_if_dead(&segment_name(name, last), PUBLISHER_LOCK_BYTE);
    }
    // 0 means "no publisher", so the count skips it on wrapping.
    let mut epoch = last.wrapping_add(1).max(1);
    for _ in 0..MAX_EPOCH_TRIES {
        let seg_name = segment_name(name, epoch);
        let live = match LiveFd::open(|| {
            sys::shm_open(&seg_name, libc::O_CREAT | libc::O_EXCL | libc::O_RDWR)
        }) {
            Ok(live) => live,
            // Not ours, or alive: anyone can create a name they can predict, so the next one is
            // drawn at random.
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                if !remove_if_dead(&seg_name, PUBLISHER_LOCK_BYTE) {
                    epoch = random_epoch(last);
                }
                continue;
            }
            Err(e) => return Err(Error::io("shm_open", &seg_name, e)),
        };
        let raw = live.raw();
        // A sweeper may unlink a fresh object before it is locked.
        if !sys::lock_read(raw, PUBLISHER_LOCK_BYTE) || !name_still_bound(&seg_name, raw) {
            continue;
        }
        if let Err(e) = sys::truncate(raw, bytes).and_then(|()| sys::allocate(raw, reserved)) {
            sys::shm_unlink(&seg_name);
            return Err(Error::io("fallocate (/dev/shm too small?)", &seg_name, e));
        }
        let base = sys::map_shared(raw, bytes).map_err(|e| {
            sys::shm_unlink(&seg_name);
            Error::io("mmap", &seg_name, e)
        })?;
        #[allow(clippy::cast_sign_loss)]
        let header_device = device.map(|(ordinal, uuid)| (ordinal as u32, uuid));
        // SAFETY: a fresh mapping of `bytes`, not yet advertised to anyone.
        let header = unsafe { init_header(base, layout, fingerprint, header_device, ready) };
        let mut seg = Segment::adopt_shm(base, bytes, header, seg_name, id_of(raw));
        seg.publisher = Some(live);
        if let Some((ordinal, _)) = device {
            // Dropping seg on failure unlinks it; nothing was advertised.
            let alloc = gpu::DeviceAlloc::create(
                &gpu::endpoint(seg.id()),
                layout.payload_bytes(),
                layout.payload_stride(),
                ready as usize * layout.payload_stride(),
                ordinal,
            )?;
            seg.device_payload(|| Ok(gpu::DevicePayload::Exported(alloc)))?;
        }
        map.sp().epoch.store(epoch, Ordering::Release); // order: sp_epoch
        // For subscriptions waiting for a publisher. Rare enough to need no gate.
        sys::futex_wake(&map.sp().epoch);
        seg.signpost = Some(map);
        return Ok(seg);
    }
    Err(Error::NotReady(format!(
        "no free segment name under '{name}'"
    )))
}

/// Neither 0 nor `last`, which subscribers would not see as a restart.
fn random_epoch(last: u32) -> u32 {
    loop {
        let mut b = [0u8; 4];
        // SAFETY: b is a valid 4-byte buffer.
        let n = unsafe { libc::getrandom(b.as_mut_ptr().cast(), b.len(), 0) };
        let e = u32::from_ne_bytes(b);
        if n == 4 && e != 0 && e != last {
            return e;
        }
    }
}

/// The layout version of another build's signpost for the same key and fingerprint, from a listing
/// of /dev/shm the process shares and renews at most once a `SKEW_PROBE_NS`: a subscriber with no
/// publisher yet asks on every attach.
fn other_layout(signpost: &str) -> Option<u32> {
    struct Listing {
        at_ns: u64,
        names: Vec<(u32, String)>,
    }
    static LISTING: Mutex<Listing> = Mutex::new(Listing {
        at_ns: 0,
        names: Vec::new(),
    });
    let ours = signpost.strip_prefix(&format!("/flux.v{LAYOUT_VERSION}."))?;
    sys::fork_safe(|| {
        let mut l = LISTING.lock().unwrap_or_else(PoisonError::into_inner);
        let now = sys::monotonic_ns();
        if l.at_ns == 0 || now.saturating_sub(l.at_ns) >= SKEW_PROBE_NS {
            l.at_ns = now;
            l.names = std::fs::read_dir("/dev/shm")
                .into_iter()
                .flatten()
                .flatten()
                .filter_map(|e| {
                    let n = e.file_name().into_string().ok()?;
                    let (v, rest) = n.strip_prefix("flux.v")?.split_once('.')?;
                    let v = v.parse().ok().filter(|&v| v != LAYOUT_VERSION)?;
                    Some((v, rest.to_owned()))
                })
                .collect();
        }
        l.names
            .iter()
            .find(|(_, rest)| rest == ours)
            .map(|&(v, _)| v)
    })
}

/// The current segment of `signpost`, opened but not mapped: a process maps a segment once for all
/// its subscriptions. The fd is the subscriber's own description, for its reader lock.
pub(crate) fn open_subscriber_fd(signpost: &SignpostMap) -> Result<(LiveFd, String, u32)> {
    let name = signpost.name();
    let epoch = signpost.epoch();
    if epoch == 0 {
        if let Some(theirs) = other_layout(name) {
            return Err(Error::Mismatch(format!(
                "'{name}' has no publisher of flux layout v{LAYOUT_VERSION}, and a flux build of \
                 layout v{theirs} uses this channel; the two never meet, so build both from one \
                 flux"
            )));
        }
        return Err(Error::NotReady(format!("no publisher on '{name}' yet")));
    }
    let seg_name = segment_name(name, epoch);
    let fd = LiveFd::open(|| sys::shm_open(&seg_name, libc::O_RDWR)).map_err(|e| {
        if e.kind() == io::ErrorKind::NotFound {
            Error::NotReady(format!("segment '{seg_name}' is gone"))
        } else {
            Error::io("shm_open", &seg_name, e)
        }
    })?;
    Ok((fd, seg_name, epoch))
}

/// Maps for a subscriber what `open_subscriber_fd` opened. Holds no publisher lock.
pub(crate) fn map_subscriber(fd: RawFd, name: &str, fingerprint: u64) -> Result<Segment> {
    let (base, bytes, header) = map_validate(fd, name, fingerprint)?;
    let mut seg = Segment::adopt_shm(base, bytes, header, name.to_owned(), id_of(fd));
    check_publisher(fd, name)?;
    seg.protect_payload();
    Ok(seg)
}

pub(crate) fn check_publisher(fd: RawFd, name: &str) -> Result<()> {
    if publisher_holds(fd) {
        return Ok(());
    }
    Err(Error::NotReady(format!(
        "segment '{name}' has no live publisher"
    )))
}

pub(crate) fn segment_id(fd: RawFd) -> SegmentId {
    id_of(fd)
}

/// Every header field it checks is read once into a `HeaderConfig`, and the checks and the layout
/// use that copy: a peer rewriting the header meanwhile changes nothing here.
pub(crate) fn map_validate(
    fd: RawFd,
    name: &str,
    fingerprint: u64,
) -> Result<(NonNull<u8>, usize, HeaderConfig)> {
    let st = sys::fstat(fd).map_err(|e| Error::io("fstat", name, e))?;
    if !st.ours() {
        return Err(not_ours(name, &st));
    }
    let bytes = st.size;
    if bytes < size_of::<ControlHeader>() {
        return Err(Error::Mismatch(format!(
            "segment '{name}' is shorter than its header"
        )));
    }
    let base = sys::map_shared(fd, bytes).map_err(|e| Error::io("mmap", name, e))?;
    let reject = |why: &str| {
        // SAFETY: mapped just above; nothing references it yet.
        unsafe { sys::munmap(base, bytes) };
        Err(Error::Mismatch(format!("{why} '{name}'")))
    };
    // Every atomic offset is a multiple of its size (layout.rs), so a page-aligned base aligns
    // them all.
    debug_assert!((base.as_ptr() as usize).is_multiple_of(CACHE_LINE));
    let ctrl = base.cast::<ControlHeader>();
    // SAFETY: at least a header's worth of mapped bytes.
    let init = unsafe { ctrl.as_ref() }.init_state.load(Ordering::Acquire); // order: seg_init
    if init != INIT_READY {
        return reject("segment not initialized");
    }
    // SAFETY: as above.
    let h = unsafe { HeaderConfig::read(ctrl.as_ptr()) };
    if h.magic != MAGIC {
        return reject("bad magic");
    }
    if h.version != LAYOUT_VERSION {
        return reject("layout version mismatch");
    }
    if h.fingerprint != fingerprint {
        return reject("fingerprint mismatch");
    }
    let device_backed = match h.storage_kind {
        k if k == StorageKind::HostInline as u8 => false,
        k if k == StorageKind::CudaIpc as u8 => true,
        _ => return reject("payload storage unknown to this build"),
    };
    // Bounded before any offset is derived from them. Within these bounds no offset wraps
    // (layout.rs asserts the widest layout).
    if !layout_config_ok(h.slot_size, h.slot_count, h.max_subscribers, h.aux_size) {
        return reject("slot_size/slot_count/max_subscribers/aux_size out of range");
    }
    let layout = h.layout();
    // A device-backed segment keeps only the control plane in shm.
    let expected = if device_backed {
        layout.payload_offset()
    } else {
        layout.total_bytes()
    };
    if expected != bytes {
        return reject("segment size does not match its header");
    }
    Ok((base, bytes, h))
}

fn not_ours(name: &str, st: &sys::Stat) -> Error {
    Error::Mismatch(format!(
        "'{name}' belongs to uid {}, not to this user; flux maps only its own objects",
        st.uid
    ))
}

/// A read lock on the publisher byte is a live publisher. A write lock there is a prober or a
/// sweeper, neither of which is one.
pub(crate) fn publisher_holds(fd: RawFd) -> bool {
    sys::lock_held(fd, PUBLISHER_LOCK_BYTE) == libc::F_RDLCK
}

pub fn segment_publisher_dead(segment: &str) -> bool {
    match sys::shm_open(segment, libc::O_RDWR) {
        Ok(fd) => !publisher_holds(fd.as_raw_fd()),
        Err(e) => e.kind() == io::ErrorKind::NotFound,
    }
}

/// Read-only mapping of a signpost. Rotations update it in place, so it stays valid.
pub struct SignpostView {
    base: Option<NonNull<u8>>,
}

// SAFETY: a read-only shared mapping read through atomics.
unsafe impl Send for SignpostView {}

impl SignpostView {
    pub fn new(name: &str) -> SignpostView {
        SignpostView::open(name).map_or(SignpostView { base: None }, |(v, _)| v)
    }

    /// Also returns the read-only fd, for probing locks on the signpost.
    pub(crate) fn open(name: &str) -> Option<(SignpostView, OwnedFd)> {
        let fd = sys::shm_open(name, libc::O_RDONLY).ok()?;
        // A created but unsized object raises SIGBUS on access.
        if sys::fstat(fd.as_raw_fd()).map_or(true, |st| st.size < SIGNPOST_BYTES || !st.ours()) {
            return None;
        }
        let base = sys::map_readonly(fd.as_raw_fd(), SIGNPOST_BYTES).ok()?;
        let view = SignpostView { base: Some(base) };
        let sp = view.sp()?;
        (sp.init_state.load(Ordering::Acquire) == INIT_READY // order: sp_init
            && sp.magic.load(Ordering::Relaxed) == SIGNPOST_MAGIC)
            .then_some((view, fd))
    }

    fn sp(&self) -> Option<&Signpost> {
        self.page().map(|p| &p.head)
    }

    pub(crate) fn page(&self) -> Option<&SignpostPage> {
        // SAFETY: a SIGNPOST_BYTES mapping whose SignpostPage fields are all atomics or padding.
        self.base
            .map(|b| unsafe { b.cast::<SignpostPage>().as_ref() })
    }

    pub fn epoch(&self) -> Option<u32> {
        self.sp().map(|sp| sp.epoch.load(Ordering::Acquire)) // order: sp_epoch
    }
}

impl Drop for SignpostView {
    fn drop(&mut self) {
        if let Some(b) = self.base {
            // SAFETY: mapped in new with SIGNPOST_BYTES; no reference outlives self.
            unsafe { sys::munmap(b, SIGNPOST_BYTES) };
        }
    }
}

/// Unlinks segments with no live publisher and signposts with no user: what crashed processes
/// left. A clean exit leaves neither. Sweepers need no turn: each proves an object dead by its own
/// lock, and checks the name still binds it.
pub fn sweep_dead() {
    let Ok(dir) = std::fs::read_dir("/dev/shm") else {
        return;
    };
    for entry in dir.flatten() {
        let file = entry.file_name();
        let Some(n) = file.to_str() else { continue };
        gpu::remove_dead_endpoint(n);
        // Liveness is judged by this layout's lock bytes; another version may hold others.
        if !n.starts_with(&format!("flux.v{LAYOUT_VERSION}.")) {
            continue;
        }
        let byte = if arena::is_arena_entry(n) {
            arena::LOCK_BYTE
        } else if is_signpost_entry(n) {
            SIGNPOST_USER_BYTE
        } else if is_segment_entry(n) {
            PUBLISHER_LOCK_BYTE
        } else {
            continue;
        };
        remove_if_dead(&format!("/{n}"), byte);
    }
}

/// Unlinks `name` if this user owns it and no process holds a read lock on `byte`.
pub(crate) fn remove_if_dead(name: &str, byte: i64) -> bool {
    let Ok(fd) = sys::shm_open(name, libc::O_RDWR) else {
        return false;
    };
    let raw = fd.as_raw_fd();
    // Exclusive: success proves no holder of the read lock, and blocks one from taking it until
    // the unlink is done. Bound: the name was not rebound since the open.
    let dead = sys::fstat(raw).is_ok_and(|st| st.ours())
        && sys::lock_write(raw, byte)
        && name_still_bound(name, raw);
    if dead {
        sys::shm_unlink(name);
    }
    dead
}

/// On the first publisher or subscription of a process, and again in a forked child. Nothing here
/// blocks, unlike a `Once`, which a child forked during the first sweep would find running forever.
pub fn sweep_dead_once() {
    static SWEPT_IN: AtomicU32 = AtomicU32::new(u32::MAX);
    let generation = sys::fork_gen();
    if SWEPT_IN.swap(generation, Ordering::Relaxed) != generation {
        sweep_dead();
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::Ordering;

    use super::{
        SignpostMap, open_publisher_segment, open_subscriber_fd, remove_if_dead, signpost_name,
        sweep_dead,
    };
    use crate::error::Error;
    use crate::layout::LAYOUT_VERSION;
    use crate::layout::{DEFAULT_MAX_SUBSCRIBERS, NO_SCHEMA};
    use crate::manifest::read_channel_stats;

    /// An end-to-end check needs a second uid (root or newuidmap), so this pins the rule every
    /// open applies before it maps.
    #[test]
    fn only_this_users_objects_are_ours() {
        let name = signpost_name(&format!("/d{}_own", std::process::id()), NO_SCHEMA).unwrap();
        let map = SignpostMap::open(&name).unwrap();
        let mut st = crate::sys::fstat(map.fd().raw()).unwrap();
        assert!(st.ours());
        st.uid = st.uid.wrapping_add(1);
        assert!(!st.ours());
    }

    /// Another build's signpost has the same key and fingerprint under its own layout version, so
    /// the two never share an object; the subscriber says so instead of waiting for good.
    #[test]
    fn a_publisher_of_another_layout_is_reported() {
        let name = signpost_name(&format!("/d{}_skew", std::process::id()), NO_SCHEMA).unwrap();
        let theirs = name.replacen(
            &format!("v{LAYOUT_VERSION}."),
            &format!("v{}.", LAYOUT_VERSION - 1),
            1,
        );
        let planted = crate::sys::shm_open(&theirs, libc::O_CREAT | libc::O_RDWR).unwrap();
        // In use, as the other build holds it; a sweep would remove it otherwise.
        assert!(crate::sys::lock_read(
            std::os::fd::AsRawFd::as_raw_fd(&planted),
            crate::layout::SIGNPOST_USER_BYTE
        ));
        let start = std::time::Instant::now();
        let err = loop {
            let map = SignpostMap::open(&name).unwrap();
            match open_subscriber_fd(&map) {
                Err(Error::NotReady(_)) if start.elapsed().as_millis() < 1000 => {
                    std::thread::sleep(std::time::Duration::from_millis(20));
                }
                Err(e) => break e,
                Ok(_) => panic!("attached to nothing"),
            }
        };
        crate::sys::shm_unlink(&theirs);
        drop(planted);
        let text = err.to_string();
        assert!(matches!(err, Error::Mismatch(_)), "{text}");
        assert!(
            text.contains(&format!("v{LAYOUT_VERSION}"))
                && text.contains(&format!("v{}", LAYOUT_VERSION - 1)),
            "{text}"
        );
    }

    #[test]
    fn a_sweep_leaves_another_layouts_objects_alone() {
        let name = signpost_name(&format!("/d{}_old", std::process::id()), NO_SCHEMA).unwrap();
        let theirs = name.replacen(
            &format!("v{LAYOUT_VERSION}."),
            &format!("v{}.", LAYOUT_VERSION - 1),
            1,
        );
        drop(crate::sys::shm_open(&theirs, libc::O_CREAT | libc::O_RDWR).unwrap());
        sweep_dead();
        let left = std::path::Path::new("/dev/shm").join(&theirs[1..]).exists();
        crate::sys::shm_unlink(&theirs);
        assert!(left);
    }

    /// Every other byte flattens to '.', as '/' does, so `/cam.left`, `/cam-left` and `/cam/left`
    /// would be one channel.
    #[test]
    fn a_key_that_would_share_a_name_with_another_is_refused() {
        for bad in [
            "/cam.left",
            "/cam-left",
            "/cam left",
            "/cam\u{1b}[31m",
            "/c\u{e9}m",
        ] {
            assert!(
                matches!(
                    signpost_name(bad, NO_SCHEMA),
                    Err(Error::InvalidArgument(_))
                ),
                "{bad:?}"
            );
        }
        let ok = signpost_name("/cam/left_1/A", NO_SCHEMA).unwrap();
        assert!(ok.contains(".cam.left_1.A."), "{ok}");
    }

    /// Two users leaving at once each see the other's read lock when they upgrade theirs.
    #[test]
    fn two_last_users_leaving_together_remove_the_signpost() {
        let name = signpost_name(&format!("/d{}_leave", std::process::id()), NO_SCHEMA).unwrap();
        let path = format!("/dev/shm{name}");
        for _ in 0..2000 {
            let barrier = std::sync::Barrier::new(2);
            let maps = [
                SignpostMap::open(&name).unwrap(),
                SignpostMap::open(&name).unwrap(),
            ];
            std::thread::scope(|s| {
                for m in maps {
                    let barrier = &barrier;
                    s.spawn(move || {
                        barrier.wait();
                        drop(m);
                    });
                }
            });
            assert!(!std::path::Path::new(&path).exists(), "left behind");
        }
    }

    /// A child that shares the publisher's open files without running the fork handlers, as
    /// `posix_spawn`'s does until it execs, does not keep a publisher that is gone alive.
    #[test]
    fn a_child_sharing_the_publishers_files_does_not_keep_it_alive() {
        let name = signpost_name(&format!("/d{}_spawned", std::process::id()), NO_SCHEMA).unwrap();
        let open =
            || open_publisher_segment(&name, 64, 2, DEFAULT_MAX_SUBSCRIBERS, 0, NO_SCHEMA, None, 2);
        // A subscriber keeps the signpost, so the next publisher locks the same file.
        let _keep = SignpostMap::open(&name).unwrap();
        let p = open().unwrap();
        // SAFETY: a raw clone runs no fork handler; the child only sleeps and exits, both plain
        // syscalls.
        let pid = unsafe { libc::syscall(libc::SYS_clone, libc::SIGCHLD, 0, 0, 0, 0) };
        if pid == 0 {
            let nap = libc::timespec {
                tv_sec: 0,
                tv_nsec: 300_000_000,
            };
            // SAFETY: plain syscall.
            unsafe { libc::nanosleep(&raw const nap, std::ptr::null_mut()) };
            // SAFETY: ends the child without running the parent's destructors.
            unsafe { libc::_exit(0) };
        }
        drop(p);
        let again = open();
        let mut status = 0;
        // SAFETY: reaps our own child.
        #[allow(clippy::cast_possible_truncation)]
        unsafe {
            libc::waitpid(pid as libc::pid_t, &raw mut status, 0)
        };
        assert!(again.is_ok(), "{:?}", again.err());
    }

    #[test]
    fn the_epoch_wraps_past_zero() {
        let name = signpost_name(&format!("/d{}_wrap", std::process::id()), NO_SCHEMA).unwrap();
        let keep = SignpostMap::open(&name).unwrap();
        keep.sp().epoch.store(u32::MAX - 1, Ordering::Release); // order: sp_epoch
        drop(
            open_publisher_segment(&name, 64, 1, DEFAULT_MAX_SUBSCRIBERS, 0, NO_SCHEMA, None, 1)
                .unwrap(),
        );
        assert_eq!(keep.epoch(), u32::MAX);
        let _p =
            open_publisher_segment(&name, 64, 1, DEFAULT_MAX_SUBSCRIBERS, 0, NO_SCHEMA, None, 1)
                .unwrap();
        assert_eq!(read_channel_stats(&name).epoch, 1);
    }

    /// An object of another user is left alone, dead or not: root would otherwise remove it, and
    /// any user could on a /dev/shm without the sticky bit.
    #[test]
    fn another_users_dead_object_is_not_removed() {
        let name = signpost_name(&format!("/d{}_theirs", std::process::id()), NO_SCHEMA).unwrap();
        let planted = crate::sys::shm_open(&name, libc::O_CREAT | libc::O_RDWR).unwrap();
        drop(planted);
        crate::sys::AS_ANOTHER_USER.set(true);
        let removed = remove_if_dead(&name, crate::layout::SIGNPOST_USER_BYTE);
        crate::sys::AS_ANOTHER_USER.set(false);
        crate::sys::shm_unlink(&name);
        assert!(!removed);
    }
}
