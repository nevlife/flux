use std::ffi::{CString, c_char};

use flux_core::{Announcement, ManifestEntry, Result, enumerate_topics, read_channel_stats};

use crate::status::{flux_status, status, text};

pub struct flux_announcement {
    _a: Announcement,
}

#[repr(C)]
pub struct flux_manifest_entry {
    pub signpost: *const c_char,
    /// The exact key; NULL or "" when the caller has none.
    pub key: *const c_char,
    pub label: *const c_char,
    pub publisher: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct flux_channel_stats {
    pub live: bool,
    pub slot_size: u32,
    pub slot_count: u32,
    pub max_subscribers: u32,
    pub storage_kind: u32,
    pub epoch: u32,
    pub fingerprint: u64,
    pub publish_seq: u64,
    pub waiters: i32,
}

#[repr(C)]
pub struct flux_endpoint_info {
    pub pid: u32,
    pub publisher: bool,
    pub label: *const c_char,
}

#[repr(C)]
pub struct flux_topic_info {
    pub signpost: *const c_char,
    pub key: *const c_char,
    pub fingerprint: u64,
    pub key_exact: bool,
    pub endpoints: *const flux_endpoint_info,
    pub endpoint_count: usize,
}

/// Every pointer inside stays valid until `flux_topic_list_free`.
pub struct flux_topic_list {
    _strings: Vec<CString>,
    _endpoints: Vec<Vec<flux_endpoint_info>>,
    topics: Vec<flux_topic_info>,
}

unsafe fn opt_text(p: *const c_char) -> Result<String> {
    if p.is_null() {
        return Ok(String::new());
    }
    // SAFETY: a non-NULL argument is a valid string.
    unsafe { text(p) }.map(str::to_owned)
}

/// Lists this endpoint until freed. A failure costs visibility only.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_announce(
    entry: *const flux_manifest_entry,
    out: *mut *mut flux_announcement,
) -> flux_status {
    // SAFETY: entry is valid, and so are its strings.
    let e = unsafe { &*entry };
    // SAFETY: as above.
    let signpost = unsafe { opt_text(e.signpost) };
    // SAFETY: as above.
    let key = unsafe { opt_text(e.key) };
    // SAFETY: as above.
    let label = unsafe { opt_text(e.label) };
    let r = match (signpost, key, label) {
        (Ok(signpost), Ok(key), Ok(label)) => Announcement::new(&ManifestEntry {
            signpost,
            key,
            label,
            publisher: e.publisher,
        }),
        (Err(x), _, _) | (_, Err(x), _) | (_, _, Err(x)) => Err(x),
    };
    status(r.map(|a| {
        // SAFETY: out is valid for a write.
        unsafe { out.write(Box::into_raw(Box::new(flux_announcement { _a: a }))) };
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_announcement_free(a: *mut flux_announcement) {
    if !a.is_null() {
        // SAFETY: a handle from flux_announce, freed once.
        drop(unsafe { Box::from_raw(a) });
    }
}

/// A missing or unreadable channel reads as not live.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_read_channel_stats(
    signpost: *const c_char,
    out: *mut flux_channel_stats,
) -> flux_status {
    // SAFETY: signpost is a valid string.
    status(unsafe { text(signpost) }.map(|n| {
        let s = read_channel_stats(n);
        let c = flux_channel_stats {
            live: s.live,
            slot_size: s.slot_size,
            slot_count: s.slot_count,
            max_subscribers: s.max_subscribers,
            storage_kind: s.storage_kind,
            epoch: s.epoch,
            fingerprint: s.fingerprint,
            publish_seq: s.publish_seq,
            waiters: s.waiters,
        };
        // SAFETY: out is valid for a write.
        unsafe { out.write(c) };
    }))
}

fn keep(strings: &mut Vec<CString>, s: &str) -> *const c_char {
    let c = CString::new(s.replace('\0', " ")).unwrap_or_default();
    let p = c.as_ptr();
    strings.push(c);
    p
}

/// Sorted by signpost. Free with `flux_topic_list_free`.
#[unsafe(no_mangle)]
pub extern "C" fn flux_enumerate_topics() -> *mut flux_topic_list {
    let mut strings = Vec::new();
    let mut all_endpoints = Vec::new();
    let mut topics = Vec::new();
    for t in enumerate_topics() {
        let endpoints: Vec<flux_endpoint_info> = t
            .endpoints
            .iter()
            .map(|e| flux_endpoint_info {
                pid: e.pid,
                publisher: e.publisher,
                label: keep(&mut strings, &e.label),
            })
            .collect();
        topics.push(flux_topic_info {
            signpost: keep(&mut strings, &t.signpost),
            key: keep(&mut strings, &t.key),
            fingerprint: t.fingerprint,
            key_exact: t.key_exact,
            endpoints: endpoints.as_ptr(),
            endpoint_count: endpoints.len(),
        });
        all_endpoints.push(endpoints);
    }
    Box::into_raw(Box::new(flux_topic_list {
        _strings: strings,
        _endpoints: all_endpoints,
        topics,
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_topic_list_size(l: *const flux_topic_list) -> usize {
    // SAFETY: a live list.
    unsafe { &*l }.topics.len()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_topic_list_get(
    l: *const flux_topic_list,
    i: usize,
) -> *const flux_topic_info {
    // SAFETY: a live list.
    unsafe { &*l }
        .topics
        .get(i)
        .map_or(std::ptr::null(), std::ptr::from_ref)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_topic_list_free(l: *mut flux_topic_list) {
    if !l.is_null() {
        // SAFETY: a list from flux_enumerate_topics, freed once.
        drop(unsafe { Box::from_raw(l) });
    }
}
