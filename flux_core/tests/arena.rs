use flux_core::arena::{HEAP_BASE, TRAILER_BYTES};
use flux_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, MemoryPolicy, NO_SCHEMA, Published, signpost_name,
};

fn pair(test: &str, slots: u32) -> (Channel, Channel) {
    // SAFETY: set before any thread reads it.
    unsafe { std::env::set_var("FLUX_ARENA_BYTES", (64u32 << 20).to_string()) };
    let name = signpost_name(&format!("/a{}_{test}", std::process::id()), NO_SCHEMA).unwrap();
    let p = Channel::create(
        &name,
        4096,
        slots,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        NO_SCHEMA,
        MemoryPolicy::default(),
    )
    .unwrap();
    let s = Channel::open(&name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    (p, s)
}

/// One frame: an 8-byte offset into the arena, then a run there longer than the slot.
fn publish(p: &mut Channel, fill: u8, n: usize) -> u64 {
    let mut w = p.loan().unwrap();
    let heap = w.heap().unwrap();
    // SAFETY: w is live.
    let (dst, off) = unsafe { heap.alloc(n, 64) }.unwrap();
    // SAFETY: heap_alloc handed out n writable bytes.
    unsafe { std::ptr::write_bytes(dst, fill, n) };
    w.data_mut()[..8].copy_from_slice(&(HEAP_BASE + off).to_le_bytes());
    assert_eq!(w.commit_len(8), Published::Ok);
    off
}

#[test]
fn a_frame_carries_a_run_past_its_slot_and_frees_it_when_the_slot_comes_back() {
    let (mut p, mut s) = pair("run", 2);
    let off = publish(&mut p, 7, 1 << 20);
    let v = s.take().unwrap();
    assert_eq!(v.data().len(), 8 + TRAILER_BYTES);
    let heap = v.heap().expect("the trailer names the arena");
    let at = u64::from_le_bytes(v.data()[..8].try_into().unwrap()) - HEAP_BASE;
    assert_eq!(at, off);
    let run = &heap[usize::try_from(at).unwrap()..][..1 << 20];
    assert!(run.iter().all(|&b| b == 7));

    // Held: the other slot takes the next frame, and this run stays put.
    let other = publish(&mut p, 9, 1 << 20);
    assert_ne!(other, off);
    assert!(run.iter().all(|&b| b == 7));
    drop(v);
    // Both slots claimed again: the first run was freed, so the arena hands its offset out again.
    let again = publish(&mut p, 1, 1 << 20);
    assert_eq!(again, off);
}

/// Runs `body` alone in a child process with an arena of `bytes`: the arena is per process, so
/// tests that fill it or count its mappings would see each other in one.
fn in_child(test: &str, bytes: usize, body: impl FnOnce()) {
    if std::env::var("FLUX_ARENA_CHILD").as_deref() == Ok(test) {
        body();
        return;
    }
    let out = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", test, "--nocapture", "--test-threads=1"])
        .env("FLUX_ARENA_CHILD", test)
        .env("FLUX_ARENA_BYTES", bytes.to_string())
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{test} failed in its child:\n{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
}

fn channel(test: &str, slots: u32) -> (Channel, Channel) {
    let name = signpost_name(&format!("/a{}_{test}", std::process::id()), NO_SCHEMA).unwrap();
    let p = Channel::create(
        &name,
        4096,
        slots,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        NO_SCHEMA,
        MemoryPolicy::default(),
    )
    .unwrap();
    let s = Channel::open(&name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    (p, s)
}

/// The lines of /proc/self/maps that map an arena with `perms` ("r--s", "rw-s").
fn arena_mappings(perms: &str) -> usize {
    std::fs::read_to_string("/proc/self/maps")
        .unwrap()
        .lines()
        .filter(|l| l.contains(perms) && l.contains(".arena-"))
        .count()
}

#[test]
fn a_dropped_publisher_frees_the_blocks_no_reader_holds() {
    const RUN: usize = 12 << 20;
    in_child(
        "a_dropped_publisher_frees_the_blocks_no_reader_holds",
        64 << 20,
        || {
            let (mut p, mut s) = channel("drop", 4);
            let first = publish(&mut p, 1, RUN);
            for fill in 2..=4 {
                publish(&mut p, fill, RUN);
            }
            // Holds the first frame's block; the other three have none.
            let held = s.take().unwrap();
            drop(p);
            // 64 MiB less the held 12 MiB at the start: 40 MiB fits only if the other three went.
            let (mut q, _t) = channel("after", 2);
            let off = publish(&mut q, 5, 40 << 20);
            assert!(off > first);
            let heap = held.heap().unwrap();
            let at = usize::try_from(first).unwrap();
            assert!(heap[at..at + RUN].iter().all(|&b| b == 1));
        },
    );
}

#[test]
fn a_subscribers_mapping_of_an_arena_goes_with_its_channel() {
    in_child(
        "a_subscribers_mapping_of_an_arena_goes_with_its_channel",
        64 << 20,
        || {
            let (mut p, mut s) = channel("map", 2);
            publish(&mut p, 3, 1 << 20);
            assert_eq!(arena_mappings("r--s"), 0);
            let v = s.take().unwrap();
            assert!(v.heap().is_some());
            assert_eq!(arena_mappings("r--s"), 1);
            drop(v);
            // The view gave its frame back; the channel still holds the mapping for the next.
            assert_eq!(arena_mappings("r--s"), 1);
            drop(s);
            assert_eq!(arena_mappings("r--s"), 0);
            assert_eq!(arena_mappings("rw-s"), 1, "the publisher's own arena stays");
        },
    );
}

#[test]
fn arena_pages_follow_the_publishers_memory_policy() {
    fn locked_kib() -> u64 {
        let status = std::fs::read_to_string("/proc/self/status").unwrap();
        let line = status.lines().find(|l| l.starts_with("VmLck:")).unwrap();
        line.split_whitespace().nth(1).unwrap().parse().unwrap()
    }
    in_child(
        "arena_pages_follow_the_publishers_memory_policy",
        64 << 20,
        || {
            let mut lim = libc::rlimit {
                rlim_cur: 0,
                rlim_max: 0,
            };
            // SAFETY: plain syscall into a valid struct.
            let rc = unsafe { libc::getrlimit(libc::RLIMIT_MEMLOCK, &raw mut lim) };
            assert_eq!(rc, 0);
            if lim.rlim_cur < 64 << 20 {
                eprintln!("skipped: RLIMIT_MEMLOCK {} is below 64 MiB", lim.rlim_cur);
                return;
            }
            let name = signpost_name(&format!("/a{}_lock", std::process::id()), NO_SCHEMA).unwrap();
            let lock = MemoryPolicy {
                precommit: false,
                lock: true,
            };
            let mut p =
                Channel::create(&name, 4096, 2, DEFAULT_MAX_SUBSCRIBERS, 0, NO_SCHEMA, lock)
                    .unwrap();
            let before = locked_kib();
            publish(&mut p, 1, 8 << 20);
            assert!(
                locked_kib() >= before + (8 << 10),
                "{} -> {}",
                before,
                locked_kib()
            );
        },
    );
}

#[test]
fn an_arena_of_0_bytes_is_off() {
    in_child("an_arena_of_0_bytes_is_off", 0, || {
        let (mut p, _s) = channel("off", 2);
        let w = p.loan().unwrap();
        assert!(w.heap().is_none());
        assert_eq!(arena_mappings("r--s") + arena_mappings("rw-s"), 0);
    });
}

#[test]
fn a_later_publishers_memory_policy_reaches_the_pages_already_reserved() {
    fn locked_kib() -> u64 {
        let status = std::fs::read_to_string("/proc/self/status").unwrap();
        let line = status.lines().find(|l| l.starts_with("VmLck:")).unwrap();
        line.split_whitespace().nth(1).unwrap().parse().unwrap()
    }
    in_child(
        "a_later_publishers_memory_policy_reaches_the_pages_already_reserved",
        64 << 20,
        || {
            let mut lim = libc::rlimit {
                rlim_cur: 0,
                rlim_max: 0,
            };
            // SAFETY: plain syscall into a valid struct.
            let rc = unsafe { libc::getrlimit(libc::RLIMIT_MEMLOCK, &raw mut lim) };
            assert_eq!(rc, 0);
            if lim.rlim_cur < 64 << 20 {
                eprintln!("skipped: RLIMIT_MEMLOCK {} is below 64 MiB", lim.rlim_cur);
                return;
            }
            let (mut p, _s) = channel("unlocked", 2);
            publish(&mut p, 1, 8 << 20);
            let before = locked_kib();
            let name =
                signpost_name(&format!("/a{}_later", std::process::id()), NO_SCHEMA).unwrap();
            let lock = MemoryPolicy {
                precommit: false,
                lock: true,
            };
            let _q = Channel::create(&name, 4096, 2, DEFAULT_MAX_SUBSCRIBERS, 0, NO_SCHEMA, lock)
                .unwrap();
            assert!(
                locked_kib() >= before + (8 << 10),
                "{} -> {}",
                before,
                locked_kib()
            );
        },
    );
}
