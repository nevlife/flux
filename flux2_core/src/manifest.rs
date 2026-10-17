//! Which channels exist and who announced them. Visibility only: nothing on the data path reads
//! the roster, so a lost or torn entry costs a row in a listing and nothing else.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

use std::os::fd::AsRawFd;
use std::sync::atomic::{AtomicI32, AtomicU8, Ordering, fence};

use crate::discovery::{SignpostMap, SignpostView, flatten_key, publisher_holds, segment_name};
use crate::error::{Error, Result};
use crate::layout::{
    ControlHeader, ENDPOINT_PUBLISHER, HeaderConfig, INIT_READY, KEY_NONE, KEY_READY,
    KEY_RECORD_BYTES, KEY_WRITING, KeyRecord, LAYOUT_VERSION, MAGIC, MAX_LABEL_LEN,
    MAX_SUBSCRIBERS, ROSTER_ENTRIES, RosterEntry, SegmentLayout, roster_lock_byte,
};
use crate::sys;

const READ_RETRIES: u32 = 64;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ManifestEntry {
    /// The channel's signpost name; carries key and fingerprint.
    pub signpost: String,
    /// The exact key, which the name only gives back flattened.
    pub key: String,
    /// Opaque to core. The ROS layer puts the node's fully qualified name here.
    pub label: String,
    pub publisher: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Endpoint {
    pub pid: u32,
    pub label: String,
    pub publisher: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Topic {
    pub signpost: String,
    pub key: String,
    pub fingerprint: u64,
    /// False: `key` is the flattened spelling read back from the name.
    pub key_exact: bool,
    pub endpoints: Vec<Endpoint>,
    /// Held entries that were being written for the whole read, and are missing from `endpoints`.
    pub busy: usize,
}

/// Read with no lock and no borrow: every field is write-once config or a publisher-written atomic.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ChannelStats {
    /// A publisher holds the advertised segment.
    pub live: bool,
    pub slot_size: u32,
    pub slot_count: u32,
    /// Subscribing processes the segment has room for.
    pub max_subscribers: u32,
    pub storage_kind: u32,
    /// A change means the publisher restarted.
    pub epoch: u32,
    pub fingerprint: u64,
    pub publish_seq: u64,
    pub waiters: i32,
}

/// Lists an endpoint for as long as it lives. The kernel drops the entry's lock at process death,
/// so a crashed endpoint stops being listed with no cleanup.
pub struct Announcement {
    map: SignpostMap,
    index: usize,
}

impl Announcement {
    /// Creates the signpost if absent, so a subscriber shows before its publisher. A label past
    /// `MAX_LABEL_LEN` bytes is cut at a character boundary.
    pub fn new(entry: &ManifestEntry) -> Result<Announcement> {
        let name = &entry.signpost;
        let map = SignpostMap::open(name)?;
        let index = (0..ROSTER_ENTRIES)
            .find(|&j| sys::lock_write(map.fd().raw(), roster_lock_byte(j)))
            .ok_or_else(|| Error::NotReady(format!("every roster entry of '{name}' is held")))?;
        let page = map.page();
        write_key(&page.key, &entry.key);
        let flags = if entry.publisher {
            ENDPOINT_PUBLISHER
        } else {
            0
        };
        let pid = u32::try_from(sys::getpid()).unwrap_or(0);
        if let Some(e) = page.roster.get(index) {
            write_entry(e, pid, flags, &entry.label);
        }
        Ok(Announcement { map, index })
    }
}

impl Drop for Announcement {
    fn drop(&mut self) {
        // A forked child's copy describes the parent's endpoint.
        if self.map.fd().owned_here()
            && let Some(e) = self.map.page().roster.get(self.index)
        {
            write_entry(e, 0, 0, "");
        }
    }
}

fn store_bytes(dst: &[AtomicU8], src: &[u8]) {
    for (d, &b) in dst.iter().zip(src) {
        d.store(b, Ordering::Relaxed);
    }
}

/// The entry's writer held `seq` odd through every retry: mid-write, or dead mid-write with the
/// lock already taken over by an owner that has not written yet.
#[derive(Debug, PartialEq, Eq)]
pub(crate) struct Busy;

fn load_bytes(src: &[AtomicU8], len: u32) -> Vec<u8> {
    src.iter()
        .take(len as usize)
        .map(|b| b.load(Ordering::Relaxed))
        .collect()
}

fn write_key(rec: &KeyRecord, key: &str) {
    if key.is_empty() || key.len() > KEY_RECORD_BYTES {
        return;
    }
    if rec
        .state
        .compare_exchange(KEY_NONE, KEY_WRITING, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        return;
    }
    store_bytes(&rec.bytes, key.as_bytes());
    rec.len
        .store(u32::try_from(key.len()).unwrap_or(0), Ordering::Relaxed);
    rec.state.store(KEY_READY, Ordering::Release);
}

pub(crate) fn read_key(rec: &KeyRecord) -> Option<String> {
    if rec.state.load(Ordering::Acquire) != KEY_READY {
        return None;
    }
    let len = rec.len.load(Ordering::Relaxed);
    if len as usize > KEY_RECORD_BYTES {
        return None;
    }
    String::from_utf8(load_bytes(&rec.bytes, len)).ok()
}

fn cut(label: &str) -> &str {
    let mut end = label.len().min(MAX_LABEL_LEN);
    while !label.is_char_boundary(end) {
        end -= 1;
    }
    &label[..end]
}

/// Only the lock holder writes. A holder that died mid-write left `seq` odd; the next one starts
/// from there.
fn write_entry(e: &RosterEntry, pid: u32, flags: u32, label: &str) {
    let label = cut(label);
    let start = e.seq.load(Ordering::Relaxed) | 1;
    e.seq.store(start, Ordering::Relaxed);
    fence(Ordering::Release);
    e.pid.store(pid, Ordering::Relaxed);
    e.flags.store(flags, Ordering::Relaxed);
    store_bytes(&e.label, label.as_bytes());
    e.label_len
        .store(u32::try_from(label.len()).unwrap_or(0), Ordering::Relaxed);
    e.seq.store(start.wrapping_add(1), Ordering::Release);
}

/// The whole label is copied on every try and its length is trusted only once the copy is known
/// consistent; a length past the label, which no writer makes, drops the entry.
pub(crate) fn read_entry(e: &RosterEntry) -> std::result::Result<Option<Endpoint>, Busy> {
    let mut label = [0u8; MAX_LABEL_LEN];
    for _ in 0..READ_RETRIES {
        let s1 = e.seq.load(Ordering::Acquire);
        if s1 & 1 != 0 {
            std::hint::spin_loop();
            continue;
        }
        let pid = e.pid.load(Ordering::Relaxed);
        let flags = e.flags.load(Ordering::Relaxed);
        let len = e.label_len.load(Ordering::Relaxed);
        for (d, s) in label.iter_mut().zip(&e.label) {
            *d = s.load(Ordering::Relaxed);
        }
        fence(Ordering::Acquire);
        if e.seq.load(Ordering::Relaxed) != s1 {
            continue;
        }
        let Some(label) = label.get(..len as usize) else {
            return Ok(None);
        };
        return Ok((pid != 0).then(|| Endpoint {
            pid,
            label: String::from_utf8_lossy(label).into_owned(),
            publisher: flags & ENDPOINT_PUBLISHER != 0,
        }));
    }
    Err(Busy)
}

/// `flux2.v<ver>.<flattened key>.<fp16>`. Segments end in `.e<epoch>` and fail here.
pub(crate) fn parse_signpost(entry: &str) -> Option<(String, u64)> {
    let rest = entry.strip_prefix(&format!("flux2.v{LAYOUT_VERSION}."))?;
    let (key, fp) = rest.rsplit_once('.')?;
    if fp.len() != 16 {
        return None;
    }
    let fp = u64::from_str_radix(fp, 16).ok()?;
    Some((key.to_owned(), fp))
}

/// Sorted by signpost. Mutates nothing: a dead endpoint is skipped, never cleared.
pub fn enumerate_topics() -> Vec<Topic> {
    let Ok(dir) = std::fs::read_dir("/dev/shm") else {
        return Vec::new();
    };
    let mut topics = Vec::new();
    for entry in dir.flatten() {
        let file = entry.file_name();
        let Some((flat, fingerprint)) = file.to_str().and_then(parse_signpost) else {
            continue;
        };
        let signpost = format!("/{}", file.to_string_lossy());
        // Another user's channel fails to open and is not listed.
        let Some((view, fd)) = SignpostView::open(&signpost) else {
            continue;
        };
        let Some(page) = view.page() else { continue };
        // Any process can write the record: a key that does not name this channel is not its.
        let (key, key_exact) = read_key(&page.key)
            .filter(|k| flatten_key(k) == flat)
            .map_or((flat, false), |k| (k, true));
        let mut endpoints = Vec::new();
        let mut busy = 0;
        for (j, e) in page.roster.iter().enumerate() {
            if sys::lock_held(fd.as_raw_fd(), roster_lock_byte(j)) != libc::F_WRLCK {
                continue;
            }
            match read_entry(e) {
                Ok(Some(ep)) => endpoints.push(ep),
                Ok(None) => {}
                Err(Busy) => busy += 1,
            }
        }
        topics.push(Topic {
            signpost,
            key,
            fingerprint,
            key_exact,
            endpoints,
            busy,
        });
    }
    topics.sort_by(|a, b| a.signpost.cmp(&b.signpost));
    topics
}

/// A missing or unreadable signpost or segment reads as not live.
pub fn read_channel_stats(signpost: &str) -> ChannelStats {
    let mut out = ChannelStats {
        epoch: SignpostView::new(signpost).epoch().unwrap_or(0),
        ..ChannelStats::default()
    };
    if out.epoch == 0 {
        return out;
    }
    let seg = segment_name(signpost, out.epoch);
    let Ok(fd) = sys::shm_open(&seg, libc::O_RDONLY) else {
        return out;
    };
    let raw = fd.as_raw_fd();
    // Created but not yet sized: touching the header would raise SIGBUS.
    let Ok(st) = sys::fstat(raw) else {
        return out;
    };
    if st.size < size_of::<ControlHeader>() || !st.ours() {
        return out;
    }
    let size = st.size;
    let len =
        size.min(SegmentLayout::WAITERS_OFFSET + MAX_SUBSCRIBERS as usize * size_of::<AtomicI32>());
    let held = publisher_holds(raw);
    let Ok(base) = sys::map_readonly(raw, len) else {
        return out;
    };
    // SAFETY: a mapping of at least a header; its plain fields are written before INIT_READY.
    let ctrl = unsafe { base.cast::<ControlHeader>().as_ref() };
    // SAFETY: as above.
    let h = unsafe { HeaderConfig::read(ctrl) };
    if held
        && ctrl.init_state.load(Ordering::Acquire) == INIT_READY
        && h.magic == MAGIC
        && h.version == LAYOUT_VERSION
    {
        out.live = true;
        out.slot_size = h.slot_size;
        out.slot_count = h.slot_count;
        out.max_subscribers = h.max_subscribers;
        out.storage_kind = u32::from(h.storage_kind);
        out.fingerprint = h.fingerprint;
        out.publish_seq = ctrl.latest.load(Ordering::Relaxed);
        let numbers = (h.max_subscribers as usize)
            .min((len - SegmentLayout::WAITERS_OFFSET.min(len)) / size_of::<AtomicI32>());
        // SAFETY: at most len bytes in, so inside the mapping or one past it.
        let at = unsafe { base.add(SegmentLayout::WAITERS_OFFSET.min(len)) };
        // SAFETY: `numbers` i32s from `at` lie inside the mapping, as just bounded.
        let per_number =
            unsafe { std::slice::from_raw_parts(at.cast::<AtomicI32>().as_ptr(), numbers) };
        out.waiters = per_number
            .iter()
            .fold(ctrl.waiters.load(Ordering::Relaxed), |n, w| {
                n.wrapping_add(w.load(Ordering::Relaxed))
            });
    }
    // SAFETY: mapped above with this length; ctrl is not used past here.
    unsafe { sys::munmap(base, len) };
    out
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::Ordering;

    use super::{Busy, read_entry, write_entry};
    use crate::layout::{MAX_LABEL_LEN, RosterEntry};

    fn entry() -> RosterEntry {
        // SAFETY: every RosterEntry field is an atomic, valid when zeroed.
        unsafe { std::mem::zeroed() }
    }

    /// An owner that died mid-write leaves `seq` odd. The next owner may hold the lock a while
    /// before it writes; until then the entry reads as busy, and its write ends that.
    #[test]
    fn an_odd_seq_reads_busy_until_the_next_owner_writes() {
        let e = entry();
        write_entry(&e, 7, 0, "/a");
        e.seq.fetch_add(1, Ordering::Relaxed);
        assert_eq!(read_entry(&e), Err(Busy));
        write_entry(&e, 8, 0, "/b");
        assert_eq!(e.seq.load(Ordering::Relaxed) % 2, 0);
        let got = read_entry(&e).unwrap().unwrap();
        assert_eq!((got.pid, got.label.as_str()), (8, "/b"));
    }

    #[test]
    fn a_label_length_past_the_label_drops_the_entry() {
        let e = entry();
        write_entry(&e, 7, 0, "/a");
        e.label_len
            .store(u32::try_from(MAX_LABEL_LEN).unwrap() + 1, Ordering::Relaxed);
        assert_eq!(read_entry(&e), Ok(None));
    }
}

/// The roster seqlock under Miri: every entry a reader accepts is one the writer wrote whole.
#[cfg(test)]
mod race {
    use super::{ENDPOINT_PUBLISHER, RosterEntry, read_entry, write_entry};

    #[test]
    fn a_read_entry_is_never_torn() {
        // SAFETY: every field is an atomic integer, for which zero bytes are valid.
        let e: RosterEntry = unsafe { std::mem::zeroed() };
        let writes = if cfg!(miri) { 8 } else { 2000 };
        std::thread::scope(|s| {
            let e = &e;
            s.spawn(move || {
                for pid in 1..=writes {
                    let flags = if pid % 2 == 0 { ENDPOINT_PUBLISHER } else { 0 };
                    write_entry(e, pid, flags, &"x".repeat(pid as usize % 7));
                }
            });
            for _ in 0..writes {
                if let Ok(Some(got)) = read_entry(e) {
                    assert_eq!(got.label.len(), got.pid as usize % 7, "{got:?}");
                    assert_eq!(got.publisher, got.pid % 2 == 0, "{got:?}");
                }
                std::thread::yield_now();
            }
        });
    }
}
