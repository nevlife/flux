use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard};

use flux_core::{
    Announcement, Channel, ChannelStats, DEFAULT_MAX_SUBSCRIBERS, ManifestEntry, MemoryPolicy,
    Published, Topic, enumerate_topics, read_channel_stats, signpost_name,
};

const CHILD_ENV: &str = "FLUX_TEST_MANIFEST_SIGNPOST";

// A spawned child holds copies of this process's lock fds until it execs.
static CHILDREN: RwLock<()> = RwLock::new(());

fn shared() -> RwLockReadGuard<'static, ()> {
    CHILDREN.read().unwrap_or_else(PoisonError::into_inner)
}

fn alone() -> RwLockWriteGuard<'static, ()> {
    CHILDREN.write().unwrap_or_else(PoisonError::into_inner)
}

struct Name {
    key: String,
    signpost: String,
}

impl Name {
    fn new(test: &str) -> Name {
        static N: AtomicU32 = AtomicU32::new(0);
        let key = format!(
            "/m{}_{}/{test}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        );
        let signpost = signpost_name(&key, 0xABCD).unwrap();
        Name { key, signpost }
    }

    fn entry(&self, label: &str, publisher: bool) -> ManifestEntry {
        ManifestEntry {
            signpost: self.signpost.clone(),
            key: self.key.clone(),
            label: label.to_owned(),
            publisher,
        }
    }

    fn listed(&self) -> Option<Topic> {
        enumerate_topics()
            .into_iter()
            .find(|t| t.signpost == self.signpost)
    }

    fn labels(&self) -> Vec<String> {
        let mut v: Vec<String> = self
            .listed()
            .map(|t| t.endpoints.into_iter().map(|e| e.label).collect())
            .unwrap_or_default();
        v.sort();
        v
    }
}

#[test]
fn a_subscriber_is_listed_before_its_publisher() {
    let _g = shared();
    let n = Name::new("early");
    let a = Announcement::new(&n.entry("/ns/listener", false)).unwrap();
    let t = n.listed().expect("the announcement created no signpost");
    assert_eq!(t.fingerprint, 0xABCD);
    assert!(t.key_exact);
    assert_eq!(t.key, n.key);
    assert_eq!(t.endpoints.len(), 1);
    let e = &t.endpoints[0];
    assert_eq!(e.label, "/ns/listener");
    assert!(!e.publisher);
    assert_eq!(e.pid, std::process::id());
    drop(a);
    assert!(
        n.labels().is_empty(),
        "a dropped announcement is still listed"
    );
}

#[test]
fn every_endpoint_is_listed_until_it_leaves() {
    let _g = shared();
    let n = Name::new("many");
    let p = Announcement::new(&n.entry("talker", true)).unwrap();
    let s1 = Announcement::new(&n.entry("one", false)).unwrap();
    let s2 = Announcement::new(&n.entry("two", false)).unwrap();
    assert_eq!(n.labels(), ["one", "talker", "two"]);
    let t = n.listed().unwrap();
    assert_eq!(t.endpoints.iter().filter(|e| e.publisher).count(), 1);
    drop(s1);
    assert_eq!(n.labels(), ["talker", "two"]);
    drop((p, s2));
}

#[test]
fn an_unannounced_channel_lists_its_flattened_key() {
    let _g = shared();
    let n = Name::new("flat");
    let _p = Channel::create(
        &n.signpost,
        64,
        2,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        0xABCD,
        MemoryPolicy::default(),
    )
    .unwrap();
    let t = n.listed().unwrap();
    assert!(!t.key_exact);
    assert_eq!(t.key, flux_core::flatten_key(&n.key));
    assert!(t.endpoints.is_empty());
}

/// The first announcer writes the exact key, and any process can announce. One naming another
/// channel would shadow it in every listing.
#[test]
fn a_key_that_does_not_name_its_channel_is_not_listed_as_exact() {
    let _g = shared();
    let n = Name::new("forged");
    let other = Name::new("honest");
    let forged = ManifestEntry {
        key: other.key.clone(),
        ..n.entry("liar", true)
    };
    let _a = Announcement::new(&forged).unwrap();
    let t = n.listed().unwrap();
    assert!(!t.key_exact);
    assert_eq!(t.key, flux_core::flatten_key(&n.key));
    assert_eq!(t.endpoints.len(), 1);
}

#[test]
fn a_long_label_is_cut_at_a_character_boundary() {
    let _g = shared();
    let n = Name::new("long");
    let label = "\u{e9}".repeat(200);
    let _a = Announcement::new(&n.entry(&label, false)).unwrap();
    let got = &n.labels()[0];
    assert!(got.len() <= flux_core::MAX_LABEL_LEN);
    assert!(label.starts_with(got.as_str()));
    assert!(!got.is_empty());
}

#[test]
fn a_full_roster_refuses_one_more() {
    let _g = shared();
    let n = Name::new("full");
    let held: Vec<Announcement> = (0..)
        .map_while(|i| Announcement::new(&n.entry(&i.to_string(), false)).ok())
        .collect();
    assert!(!held.is_empty());
    assert!(Announcement::new(&n.entry("late", false)).is_err());
    drop(held);
    assert!(Announcement::new(&n.entry("late", false)).is_ok());
}

#[test]
fn stats_follow_the_publisher() {
    let _g = shared();
    let n = Name::new("stats");
    assert_eq!(read_channel_stats(&n.signpost), ChannelStats::default());
    let mut p = Channel::create(
        &n.signpost,
        64,
        4,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        0xABCD,
        MemoryPolicy::default(),
    )
    .unwrap();
    for _ in 0..3 {
        assert_eq!(p.publish(b"x"), Published::Ok);
    }
    let s = read_channel_stats(&n.signpost);
    assert!(s.live);
    assert_eq!((s.slot_size, s.slot_count, s.fingerprint), (64, 4, 0xABCD));
    assert_eq!((s.publish_seq, s.waiters, s.storage_kind), (3, 0, 0));
    let epoch = s.epoch;
    assert!(epoch > 0);
    let watcher = Announcement::new(&n.entry("watcher", false)).unwrap();
    drop(p);
    let s = read_channel_stats(&n.signpost);
    assert!(!s.live);
    assert_eq!(
        s.epoch, epoch,
        "a remaining user keeps the signpost and its epoch"
    );
    drop(watcher);
    assert!(n.listed().is_none(), "the last user out left the signpost");
}

#[test]
fn child_announces_and_dies() {
    let Ok(signpost) = std::env::var(CHILD_ENV) else {
        return;
    };
    let a = Announcement::new(&ManifestEntry {
        signpost,
        key: String::new(),
        label: "doomed".into(),
        publisher: false,
    })
    .unwrap();
    std::mem::forget(a);
    println!("announced");
    std::process::exit(0);
}

#[test]
fn a_dead_endpoint_is_not_listed() {
    let _g = alone();
    let n = Name::new("dead");
    let _me = Announcement::new(&n.entry("alive", false)).unwrap();
    let out = Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "child_announces_and_dies", "--nocapture"])
        .env(CHILD_ENV, &n.signpost)
        .stdout(Stdio::piped())
        .output()
        .unwrap();
    assert!(String::from_utf8_lossy(&out.stdout).contains("announced"));
    assert_eq!(
        n.labels(),
        ["alive"],
        "the dead child's entry is still listed"
    );
}
