//! A publisher left at slot count 0 starts with `START_SLOTS` slots with memory and adds slots up to
//! its count; one given a count has them all from the start.

#![allow(clippy::disallowed_types, reason = "a test")]

use std::os::unix::fs::MetadataExt;

use super::{Channel, Published};
use crate::discovery::signpost_name;
use crate::layout::{DEFAULT_MAX_SUBSCRIBERS, NO_SCHEMA};
use crate::qos::{MemoryPolicy, QoS};
use crate::sizing::START_SLOTS;

const SLOT: u32 = 64 << 10;

fn name(what: &str) -> String {
    signpost_name(&format!("/grow{}_{what}", std::process::id()), NO_SCHEMA).unwrap()
}

fn publisher(name: &str, count: u32) -> Channel {
    Channel::create(
        name,
        SLOT,
        count,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        NO_SCHEMA,
        MemoryPolicy::default(),
    )
    .unwrap()
}

fn subscriber(name: &str, qos: QoS) -> Channel {
    let mut s = Channel::open(name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    s.set_qos(qos).unwrap();
    s
}

/// Bytes of /dev/shm the segment holds: the ones written or reserved, not its holes.
fn resident(p: &Channel) -> u64 {
    std::fs::metadata(format!("/dev/shm{}", p.sh.seg.name()))
        .unwrap()
        .blocks()
        * 512
}

#[test]
fn a_count_left_to_flux_reserves_only_its_start_slots() {
    let p = publisher(&name("start"), 0);
    assert_eq!(p.slot_count(), 256);
    assert_eq!(p.slots_ready(), START_SLOTS);
    let l = p.sh.seg.layout();
    let reserved = (l.payload_offset() + START_SLOTS as usize * l.payload_stride()) as u64;
    assert_eq!(resident(&p), reserved);
    assert!(reserved < l.total_bytes() as u64 / 20);
}

/// Without the added slots the ring of nine would lose all but the newest nine of the 30.
#[test]
fn a_deeper_subscriber_keeps_its_depth_of_frames() {
    let n = name("deep");
    let mut p = publisher(&n, 0);
    let mut s = subscriber(&n, QoS::new(20).unwrap());
    for i in 0..30u8 {
        assert_eq!(p.publish(&[i]), Published::Ok);
    }
    assert_eq!(p.slots_ready(), 21);
    let got: Vec<u8> = std::iter::from_fn(|| s.take().map(|v| v.data()[0])).collect();
    assert_eq!(got, (10..30).collect::<Vec<_>>());
    assert_eq!(s.lost(), 10);
}

#[test]
fn a_slot_is_added_when_every_slot_is_held() {
    let n = name("held");
    let mut p = publisher(&n, 0);
    let mut s = subscriber(&n, QoS::new(1).unwrap().max_borrow(12).unwrap());
    let mut held = Vec::new();
    for i in 0..12u8 {
        assert_eq!(p.publish(&[i]), Published::Ok);
        held.push(s.take().unwrap());
    }
    assert_eq!(p.slots_ready(), 12);
    assert_eq!(p.publish(&[12]), Published::Ok);
    assert_eq!(p.slots_ready(), 13);
}

#[test]
fn a_count_asked_for_has_every_slot_and_never_adds() {
    let n = name("asked");
    let mut p = publisher(&n, 4);
    assert_eq!(p.slots_ready(), 4);
    let mut s = subscriber(&n, QoS::new(4).unwrap().max_borrow(4).unwrap());
    let mut held = Vec::new();
    for i in 0..4u8 {
        assert_eq!(p.publish(&[i]), Published::Ok);
        held.push(s.take().unwrap());
    }
    assert_eq!(p.publish(&[4]), Published::Backpressure);
    assert_eq!(p.slots_ready(), 4);
}

#[test]
fn a_subscriber_that_commits_its_pages_commits_the_added_slots_it_reads() {
    let n = name("commit");
    let mut p = publisher(&n, 0);
    let mut s = Channel::open(
        &n,
        NO_SCHEMA,
        MemoryPolicy {
            precommit: true,
            lock: false,
        },
    )
    .unwrap();
    s.set_qos(QoS::new(20).unwrap()).unwrap();
    for i in 0..15u8 {
        assert_eq!(p.publish(&[i]), Published::Ok);
        assert_eq!(s.take().map(|v| v.data()[0]), Some(i));
    }
    let l = s.sh.seg.layout();
    let ready = p.slots_ready() as usize;
    assert!(ready > START_SLOTS as usize);
    assert_eq!(
        s.sh.seg.committed_bytes(),
        l.payload_offset() + ready * l.payload_stride()
    );
}
