use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard};
use std::time::{Duration, Instant};

use flux_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, Error, FrameView, MemoryPolicy, NO_SCHEMA, Published, QoS,
    flatten_key, read_channel_stats, signpost_name,
};

const CHILD_ENV: &str = "FLUX_TEST_CHILD_SIGNPOST";

// A spawned or forked child holds copies of this process's lock fds until it execs or exits, so a
// publisher dropped and re-created meanwhile is refused. Tests that make children run alone.
static CHILDREN: RwLock<()> = RwLock::new(());

fn shared() -> RwLockReadGuard<'static, ()> {
    CHILDREN.read().unwrap_or_else(PoisonError::into_inner)
}

fn alone() -> RwLockWriteGuard<'static, ()> {
    CHILDREN.write().unwrap_or_else(PoisonError::into_inner)
}

struct Topic(String);

impl Topic {
    fn new(test: &str) -> Topic {
        static N: AtomicU32 = AtomicU32::new(0);
        let key = format!(
            "/t{}_{}_{test}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        );
        Topic(signpost_name(&key, NO_SCHEMA).unwrap())
    }
}

fn exists(name: &str) -> bool {
    std::path::Path::new("/dev/shm").join(&name[1..]).exists()
}

fn publisher(t: &Topic, slots: u32) -> Channel {
    Channel::create(
        &t.0,
        64,
        slots,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    )
    .unwrap()
}

fn subscriber(t: &Topic) -> Channel {
    Channel::open(&t.0, NO_SCHEMA, MemoryPolicy::default()).unwrap()
}

/// Takes both frames of a two-slot channel, so a publish must wait on these views.
fn take_both(s: &mut Channel) -> (FrameView, FrameView) {
    s.set_qos(
        QoS::new(2)
            .unwrap()
            .transient_local(2)
            .max_borrow(2)
            .unwrap(),
    )
    .unwrap();
    (s.take().unwrap(), s.take().unwrap())
}

#[test]
fn subscriber_sees_publisher_frames() {
    let _g = shared();
    let t = Topic::new("basic");
    let mut p = publisher(&t, 4);
    let mut s = subscriber(&t);
    assert!(s.payload_readonly());
    assert_eq!(p.publish(b"frame"), Published::Ok);
    let v = s.take().unwrap();
    assert_eq!(v.data(), b"frame");
    assert!(v.publish_ns() > 0);
    assert_eq!(s.publish(b"no"), Published::NotPublisher);
}

#[test]
fn open_before_publisher_is_not_ready() {
    let _g = shared();
    let t = Topic::new("early");
    assert!(matches!(
        Channel::open(&t.0, NO_SCHEMA, MemoryPolicy::default()),
        Err(Error::NotReady(_))
    ));
}

#[test]
fn second_publisher_is_refused() {
    let _g = shared();
    let t = Topic::new("exclusive");
    let p = publisher(&t, 2);
    let second = Channel::create(
        &t.0,
        64,
        2,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    );
    assert!(matches!(second, Err(Error::PublisherExists(_))));
    drop(p);
    publisher(&t, 2);
}

#[test]
fn fingerprint_mismatch_is_refused() {
    let _g = shared();
    let t = Topic::new("fp");
    let _p = publisher(&t, 2);
    assert!(matches!(
        Channel::open(&t.0, 7, MemoryPolicy::default()),
        Err(Error::Mismatch(_))
    ));
}

#[test]
fn transient_local_replays_recent_frames() {
    let _g = shared();
    let t = Topic::new("replay");
    let mut p = publisher(&t, 4);
    for b in [b"a", b"b", b"c"] {
        assert_eq!(p.publish(b), Published::Ok);
    }
    let mut s = subscriber(&t);
    s.set_qos(QoS::new(3).unwrap().transient_local(2)).unwrap();
    assert_eq!(s.take().unwrap().data(), b"b");
    assert_eq!(s.take().unwrap().data(), b"c");
    assert!(s.take().is_none());
}

#[test]
fn live_subscriber_views_are_not_reclaimed() {
    let _g = shared();
    let t = Topic::new("live");
    let mut p = publisher(&t, 2);
    let mut s = subscriber(&t);
    assert_eq!(p.publish(b"a"), Published::Ok);
    assert_eq!(p.publish(b"b"), Published::Ok);
    let v = take_both(&mut s);
    assert_eq!(p.publish(b"c"), Published::Backpressure);
    drop(v);
    assert_eq!(p.publish(b"c"), Published::Ok);
}

/// The park is bounded (100 ms), so only the latency tells a futex wake from a poll.
#[test]
fn blocking_take_wakes_on_publish() {
    let _g = shared();
    let topic = Topic::new("block");
    let mut publ = publisher(&topic, 2);
    let mut sub = subscriber(&topic);
    let handle = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(10));
        let sent = Instant::now();
        assert_eq!(publ.publish(b"late"), Published::Ok);
        (publ, sent)
    });
    let view = sub
        .take_blocking(2_000_000_000)
        .expect("woken by the publish");
    let got = Instant::now();
    assert_eq!(view.data(), b"late");
    let (_publ, sent) = handle.join().unwrap();
    let latency = got.duration_since(sent);
    assert!(
        latency < Duration::from_millis(40),
        "{latency:?}: polled, not woken"
    );
}

#[test]
fn subscriber_follows_a_restarted_publisher() {
    let _g = shared();
    let t = Topic::new("restart");
    let mut p = publisher(&t, 2);
    let mut s = subscriber(&t);
    assert_eq!(p.publish(b"one"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"one");
    let gen0 = s.attach_generation();

    drop(p);
    for _ in 0..10 {
        assert!(s.take().is_none());
    }
    assert!(s.orphaned());

    let mut p = publisher(&t, 2);
    assert!(s.take().is_none());
    assert_ne!(s.attach_generation(), gen0);
    assert!(!s.orphaned());
    assert_eq!(p.publish(b"two"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"two");
}

/// A view keeps the old mapping alive by itself, so holding one must not keep the subscriber on
/// a publisher that is gone: a `message_filters` queue holds views for as long as a match is late.
#[test]
fn a_held_view_does_not_pin_a_subscriber_to_a_dead_publisher() {
    let _g = shared();
    let t = Topic::new("held_restart");
    let mut p = publisher(&t, 2);
    let mut s = subscriber(&t);
    assert_eq!(p.publish(b"one"), Published::Ok);
    let held = s.take().unwrap();

    drop(p);
    for _ in 0..10 {
        assert!(s.take().is_none());
    }
    let gen0 = s.attach_generation();
    let mut p = publisher(&t, 2);
    assert!(s.take().is_none());
    assert_ne!(s.attach_generation(), gen0, "still on the dead publisher");
    assert_eq!(p.publish(b"two"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"two");
    assert_eq!(held.data(), b"one");
}

/// At `max_borrow` no take stalls, so the restart must be noticed without one. The held views go
/// on pinning only the old mapping.
#[test]
fn views_at_max_borrow_do_not_pin_a_subscriber_to_a_dead_publisher() {
    let _g = shared();
    let t = Topic::new("full_restart");
    let mut p = publisher(&t, 4);
    let mut s = subscriber(&t);
    s.set_qos(QoS::new(2).unwrap().max_borrow(2).unwrap())
        .unwrap();
    assert_eq!(p.publish(b"one"), Published::Ok);
    let first = s.take().unwrap();
    assert_eq!(p.publish(b"two"), Published::Ok);
    let second = s.take().unwrap();

    drop(p);
    let gen0 = s.attach_generation();
    let mut p = publisher(&t, 4);
    assert!(s.take().is_none());
    assert_ne!(s.attach_generation(), gen0, "still on the dead publisher");
    assert_eq!(p.publish(b"three"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"three");
    assert_eq!((first.data(), second.data()), (&b"one"[..], &b"two"[..]));
}

/// The frames a restarted publisher sends before the next take all came after this consumer
/// joined: they are delivered, and those past `depth` are counted lost.
#[test]
fn frames_of_a_restarted_publisher_before_the_next_take_are_delivered_or_counted() {
    let _g = shared();
    for (depth, got, lost) in [
        (4, &[&b"two"[..], b"three"][..], 0),
        (1, &[&b"three"[..]], 1),
    ] {
        let t = Topic::new(&format!("restart_early{depth}"));
        let mut p = publisher(&t, 4);
        let mut s = subscriber(&t);
        s.set_qos(QoS::new(depth).unwrap()).unwrap();
        assert_eq!(p.publish(b"one"), Published::Ok);
        assert_eq!(s.take().unwrap().data(), b"one");
        drop(p);
        let mut p = publisher(&t, 4);
        assert_eq!(p.publish(b"two"), Published::Ok);
        assert_eq!(p.publish(b"three"), Published::Ok);
        let taken: Vec<Vec<u8>> = (0..20)
            .filter_map(|_| s.take())
            .map(|v| v.data().to_vec())
            .collect();
        assert_eq!(taken, got, "depth {depth}");
        assert_eq!(s.lost(), lost, "depth {depth}");
    }
}

/// `lost` covers the subscription's whole life: a re-attach adds to it.
#[test]
fn lost_keeps_counting_across_a_publisher_restart() {
    let _g = shared();
    let t = Topic::new("lost_restart");
    let mut p = publisher(&t, 4);
    let mut s = subscriber(&t);
    s.set_qos(QoS::new(1).unwrap()).unwrap();
    for b in [b"a", b"b", b"c"] {
        assert_eq!(p.publish(b), Published::Ok);
    }
    assert_eq!(s.take().unwrap().data(), b"c");
    assert_eq!(s.lost(), 2);
    drop(p);
    let mut p = publisher(&t, 4);
    assert_eq!(p.publish(b"d"), Published::Ok);
    assert_eq!(p.publish(b"e"), Published::Ok);
    let taken: Vec<Vec<u8>> = (0..20)
        .filter_map(|_| s.take())
        .map(|v| v.data().to_vec())
        .collect();
    assert_eq!(taken, [b"e"]);
    assert_eq!(s.lost(), 3);
}

/// Takes a subscriber number on a segment as another process would, through a description of
/// its own.
fn hold_number(t: &Topic, number: i64) -> std::fs::File {
    use std::os::fd::AsRawFd;
    let epoch = read_channel_stats(&t.0).epoch;
    let f = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(format!("/dev/shm{}.e{epoch}", t.0))
        .unwrap();
    // SAFETY: flock is plain integers; all-zero is a valid value.
    let mut fl: libc::flock = unsafe { std::mem::zeroed() };
    fl.l_type = libc::c_short::try_from(libc::F_WRLCK).unwrap();
    fl.l_whence = libc::c_short::try_from(libc::SEEK_SET).unwrap();
    fl.l_start = 1 + number;
    fl.l_len = 1;
    // SAFETY: fl is a valid flock that outlives the call.
    let r = unsafe { libc::fcntl(f.as_raw_fd(), libc::F_OFD_SETLK, &raw const fl) };
    assert_eq!(r, 0);
    f
}

/// A restarted publisher with no number left for this process: the take reports why, does not
/// try again on every call, and tries again once the publisher restarts.
#[test]
fn a_refused_re_attach_is_reported_until_the_publisher_restarts() {
    let _g = shared();
    let t = Topic::new("refused_restart");
    let p = publisher(&t, 2);
    let mut s = subscriber(&t);
    assert!(s.take().is_none());
    drop(p);
    let one = || Channel::create(&t.0, 64, 2, 1, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    let p = one();
    let other = hold_number(&t, 0);
    let gen0 = s.attach_generation();
    for _ in 0..3 {
        assert!(s.take().is_none());
    }
    assert!(
        matches!(s.attach_error(), Some(Error::SubscribersFull(_))),
        "{:?}",
        s.attach_error()
    );
    assert!(
        s.take_blocking(-1).is_none(),
        "waited for a publisher that refused it"
    );
    let retried = Instant::now();
    for _ in 0..20 {
        assert!(s.take().is_none());
    }
    assert!(
        retried.elapsed() < Duration::from_millis(10),
        "asked the full segment again"
    );
    drop((p, other));
    let mut p = publisher(&t, 2);
    assert!(s.take().is_none());
    assert!(s.attach_error().is_none());
    assert_ne!(s.attach_generation(), gen0);
    assert_eq!(p.publish(b"back"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"back");
}

/// One mapping per segment and process: subscriptions share it, with their own cursors and
/// borrows, and a memory policy one asks for covers them all.
#[test]
fn subscriptions_of_one_process_share_one_mapping() {
    let _g = shared();
    let topic = Topic::new("share");
    let mut publ = publisher(&topic, 4);
    let seg = format!("{}.e{}", topic.0, read_channel_stats(&topic.0).epoch);
    let mappings = || {
        std::fs::read_to_string("/proc/self/maps")
            .unwrap()
            .lines()
            .filter(|l| l.ends_with(&seg))
            .count()
    };
    let before = mappings();
    let mut first = subscriber(&topic);
    let one = mappings() - before;
    let mut second = subscriber(&topic);
    let precommit = MemoryPolicy {
        precommit: true,
        lock: false,
    };
    let third = Channel::open(&topic.0, NO_SCHEMA, precommit).unwrap();
    assert_eq!(mappings() - before, one, "mapped again");
    assert!(first.pages_committed() && third.pages_committed());

    let one_borrow = QoS::new(1).unwrap().max_borrow(1).unwrap();
    first.set_qos(one_borrow).unwrap();
    second.set_qos(one_borrow).unwrap();
    assert_eq!(publ.publish(b"one"), Published::Ok);
    let held = first.take().unwrap();
    let other = second.take().unwrap();
    assert_eq!(publ.publish(b"two"), Published::Ok);
    assert!(first.take().is_none(), "at its own max_borrow");
    drop((second, third, other));
    assert_eq!(held.data(), b"one");
    drop(held);
    assert_eq!(first.take().unwrap().data(), b"two");
}

/// A consumer further behind than the ticket index reaches (four entries for two slots) still
/// gets the oldest frame left, here one another consumer holds.
#[test]
fn a_frame_older_than_the_ticket_index_reaches_is_still_found() {
    let _g = shared();
    let t = Topic::new("index_reach");
    let mut p = publisher(&t, 2);
    let mut holder = subscriber(&t);
    let mut late = subscriber(&t);
    late.set_qos(QoS::new(100).unwrap()).unwrap();
    assert_eq!(p.publish(b"old"), Published::Ok);
    let held = holder.take().unwrap();
    for _ in 0..10 {
        assert_eq!(p.publish(b"new"), Published::Ok);
    }
    assert_eq!(late.take().unwrap().data(), b"old");
    assert_eq!(late.take().unwrap().data(), b"new");
    assert_eq!(late.lost(), 9);
    drop(held);
}

/// Any peer can write the old segment's header. The re-attach keeps to the subscriber's own value.
#[test]
fn a_rewritten_fingerprint_does_not_keep_the_subscriber_off_a_restarted_publisher() {
    use std::os::unix::fs::FileExt;

    let _g = shared();
    let t = Topic::new("fp_refetch");
    let p = publisher(&t, 2);
    let mut s = subscriber(&t);
    let epoch = read_channel_stats(&t.0).epoch;
    let header = std::fs::OpenOptions::new()
        .write(true)
        .open(format!("/dev/shm{}.e{epoch}", t.0))
        .unwrap();
    // ControlHeader::fingerprint is at offset 32.
    header.write_all_at(&0xdead_u64.to_ne_bytes(), 32).unwrap();

    drop(p);
    let gen0 = s.attach_generation();
    let mut p = publisher(&t, 2);
    for _ in 0..20 {
        if s.take().is_none() && s.attach_generation() != gen0 {
            break;
        }
    }
    assert_ne!(s.attach_generation(), gen0, "never re-attached");
    assert_eq!(p.publish(b"two"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"two");
}

const FD_CHILD_ENV: &str = "FLUX_TEST_FD_CHILD_KEY";

/// Runs only as the child of `running_short_of_fds_is_an_io_error_and_leaves_nothing`: with 0 to
/// 3 descriptors free, creates a publisher and opens the parent's topic, and prints each outcome.
#[test]
fn child_runs_short_of_fds() {
    let (Ok(key), Ok(theirs)) = (std::env::var(FD_CHILD_ENV), std::env::var(CHILD_ENV)) else {
        return;
    };
    let mut lim = libc::rlimit {
        rlim_cur: 0,
        rlim_max: 0,
    };
    // SAFETY: lim is a valid out-parameter.
    unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &raw mut lim) };
    lim.rlim_cur = 64;
    // SAFETY: lim is a valid rlimit.
    let set = unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &raw const lim) };
    assert_eq!(set, 0);
    let mut spare = Vec::new();
    for free in 0..4 {
        for side in ["create", "open"] {
            loop {
                // SAFETY: duplicates stderr; the copy is ours.
                let fd = unsafe { libc::dup(2) };
                if fd < 0 {
                    break;
                }
                spare.push(fd);
            }
            for fd in spare.drain(spare.len() - free..) {
                // SAFETY: one of the copies made above.
                unsafe { libc::close(fd) };
            }
            let r = if side == "create" {
                let name = signpost_name(&format!("{key}_{free}"), NO_SCHEMA).unwrap();
                Channel::create(
                    &name,
                    64,
                    2,
                    DEFAULT_MAX_SUBSCRIBERS,
                    NO_SCHEMA,
                    MemoryPolicy::default(),
                )
            } else {
                Channel::open(&theirs, NO_SCHEMA, MemoryPolicy::default())
            };
            let outcome = match r {
                Ok(_) => "ok".to_owned(),
                Err(Error::Io { .. }) => "io".to_owned(),
                Err(e) => format!("unexpected {e}"),
            };
            println!("\n{side} with {free} free: {outcome}");
        }
    }
}

#[test]
fn running_short_of_fds_is_an_io_error_and_leaves_nothing() {
    let _g = alone();
    let t = Topic::new("fds_open");
    let _p = publisher(&t, 2);
    let key = format!("/t{}_fds", std::process::id());
    let out = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "child_runs_short_of_fds",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(FD_CHILD_ENV, &key)
        .env(CHILD_ENV, &t.0)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&out.stdout);
    let outcomes: Vec<&str> = text.lines().filter(|l| l.contains(" free: ")).collect();
    assert_eq!(outcomes.len(), 8, "{text}");
    for l in outcomes {
        assert!(l.ends_with(": ok") || l.ends_with(": io"), "{text}");
    }
    let flat = flatten_key(&key);
    let left: Vec<String> = std::fs::read_dir("/dev/shm")
        .unwrap()
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .filter(|n| n.contains(&flat))
        .collect();
    assert!(left.is_empty(), "left behind: {left:?}");
}

#[test]
fn the_last_user_out_removes_the_signpost() {
    let _g = shared();
    let t = Topic::new("last_out");
    let p = publisher(&t, 2);
    let s = subscriber(&t);
    drop(p);
    assert!(exists(&t.0), "the subscriber still uses it");
    drop(s);
    assert!(!exists(&t.0));
}

/// Runs only as the child of `a_new_process_sweeps_an_unused_signpost`: creates a publisher.
#[test]
fn child_creates_a_publisher() {
    if std::env::var(CHILD_ENV).is_ok() {
        drop(publisher(&Topic::new("sweeper"), 2));
    }
}

/// What a crashed process leaves: a signpost no one holds a lock on. A process sweeps once, on its
/// first publisher or subscription.
#[test]
fn a_new_process_sweeps_an_unused_signpost() {
    let _g = alone();
    let t = Topic::new("sweep");
    let c = std::ffi::CString::new(t.0.clone()).unwrap();
    // SAFETY: c is a valid C string.
    let fd = unsafe { libc::shm_open(c.as_ptr(), libc::O_CREAT | libc::O_RDWR, 0o600) };
    assert!(fd >= 0);
    // SAFETY: fd was just opened here.
    unsafe { libc::close(fd) };
    assert!(exists(&t.0));
    let ok = Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "child_creates_a_publisher", "--test-threads=1"])
        .env(CHILD_ENV, &t.0)
        .stdout(Stdio::null())
        .status()
        .unwrap()
        .success();
    assert!(ok);
    assert!(!exists(&t.0));
}

/// A publisher that crashed after this process swept leaves its segment. The next publisher of
/// the channel removes it without sweeping again.
#[test]
fn a_publisher_removes_the_segment_its_crashed_predecessor_left() {
    let _g = alone();
    let t = Topic::new("predecessor");
    drop(publisher(&Topic::new("swept"), 2));
    // SAFETY: the child creates a publisher and leaves without running any destructor.
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        child_exit(|| {
            std::mem::forget(publisher(&t, 2));
            0
        });
    }
    assert_eq!(wait_child(pid), 0);
    let epoch = read_channel_stats(&t.0).epoch;
    let left = format!("{}.e{epoch}", t.0);
    assert!(exists(&left), "the crashed publisher left nothing");
    let _p = publisher(&t, 2);
    assert!(!exists(&left), "still there");
}

/// Anyone can create the names a publisher would take next. Ones it cannot open, as another
/// user's would be, do not stop it.
#[test]
fn squatted_segment_names_do_not_stop_a_publisher() {
    let _g = shared();
    let t = Topic::new("squat");
    let names: Vec<String> = (1..=64).map(|e| format!("{}.e{e}", t.0)).collect();
    for n in &names {
        let c = std::ffi::CString::new(n.clone()).unwrap();
        // SAFETY: c is a valid C string.
        let fd =
            unsafe { libc::shm_open(c.as_ptr(), libc::O_CREAT | libc::O_EXCL | libc::O_RDWR, 0) };
        assert!(fd >= 0);
        // SAFETY: fd was just opened here.
        unsafe { libc::close(fd) };
    }
    let p = Channel::create(
        &t.0,
        64,
        2,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    );
    for n in &names {
        let c = std::ffi::CString::new(n.clone()).unwrap();
        // SAFETY: c is a valid C string.
        unsafe { libc::shm_unlink(c.as_ptr()) };
    }
    let mut p = p.unwrap();
    let mut s = subscriber(&t);
    assert_eq!(p.publish(b"x"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"x");
}

/// Runs only as the child of `dead_subscriber_slot_is_reclaimed`: holds a view, says so, and
/// waits to be killed.
#[test]
fn child_holds_a_view() {
    let Ok(name) = std::env::var(CHILD_ENV) else {
        return;
    };
    let mut s = Channel::open(&name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    let v = take_both(&mut s);
    println!("held {}", v.1.ticket());
    std::thread::sleep(Duration::from_secs(60));
}

#[test]
fn dead_subscriber_slot_is_reclaimed() {
    let _g = alone();
    let t = Topic::new("crash");
    let mut p = publisher(&t, 2);
    assert_eq!(p.publish(b"a"), Published::Ok);
    assert_eq!(p.publish(b"b"), Published::Ok);

    let mut child = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "child_holds_a_view",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD_ENV, &t.0)
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut out = BufReader::new(child.stdout.take().unwrap());
    let mut line = String::new();
    // libtest prints the test name and its output on one line.
    while !line.contains("held ") {
        line.clear();
        assert_ne!(
            out.read_line(&mut line).unwrap(),
            0,
            "child exited without holding"
        );
    }

    assert_eq!(p.publish(b"c"), Published::Backpressure);
    child.kill().unwrap();
    child.wait().unwrap();
    assert_eq!(p.publish(b"c"), Published::Ok);
}

/// Runs only as the child of `a_subscriber_killed_while_parked_does_not_keep_the_wake_on`.
#[test]
fn child_parks() {
    let Ok(name) = std::env::var(CHILD_ENV) else {
        return;
    };
    let s = Channel::open(&name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    println!("parking");
    loop {
        s.wait(s.wake_seq(), -1);
    }
}

/// The first publish with a waiter sweeps, so the sweep that takes the dead count back is the
/// timed one after it, not the first ever.
#[test]
fn a_subscriber_killed_while_parked_does_not_keep_the_wake_on() {
    let _g = alone();
    let t = Topic::new("parked");
    let mut p = publisher(&t, 2);
    let waiters = || read_channel_stats(&t.0).waiters;

    let mut child = Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "child_parks", "--nocapture", "--test-threads=1"])
        .env(CHILD_ENV, &t.0)
        .stdout(Stdio::null())
        .spawn()
        .unwrap();
    while waiters() != 1 {
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(p.publish(b"a"), Published::Ok);
    let swept = Instant::now();
    child.kill().unwrap();
    child.wait().unwrap();
    assert_eq!(waiters(), 1, "the dead subscriber's count is still there");

    assert_eq!(p.publish(b"b"), Published::Ok);
    if swept.elapsed() < Duration::from_millis(900) {
        assert_eq!(waiters(), 1, "swept again within a second");
    }
    std::thread::sleep(Duration::from_secs(1).saturating_sub(swept.elapsed()));
    assert_eq!(p.publish(b"c"), Published::Ok);
    assert_eq!(waiters(), 0, "the publish that paid the wake took it back");
}

#[test]
fn forked_child_does_not_keep_the_publisher_alive() {
    let _g = alone();
    let t = Topic::new("fork");
    let p = publisher(&t, 2);
    let ready = pipe();
    let hold = pipe();
    // SAFETY: the child only calls async-signal-safe functions before _exit.
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        // SAFETY: the child's copy of hold's write end; read below sees EOF once the parent's goes.
        unsafe { libc::close(hold[1]) };
        // Past fork(), so the atfork handler has closed the lock fds.
        write_byte(ready[1]);
        read_byte(hold[0]);
        // SAFETY: ends the child without unwinding into the test harness.
        unsafe { libc::_exit(0) };
    }
    read_byte(ready[0]);
    drop(p);
    let again = Channel::create(
        &t.0,
        64,
        2,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    );
    for fd in ready.into_iter().chain(hold) {
        // SAFETY: our pipe ends; closing hold's write end releases the child.
        unsafe { libc::close(fd) };
    }
    // SAFETY: reaps our own child.
    unsafe { libc::waitpid(pid, std::ptr::null_mut(), 0) };
    assert!(again.is_ok(), "{:?}", again.err());
}

fn wait_child(pid: libc::pid_t) -> i32 {
    let mut status = 0;
    // SAFETY: reaps our own child.
    unsafe { libc::waitpid(pid, &raw mut status, 0) };
    status
}

/// Ends a forked child with `f`'s code. A panic would otherwise unwind into the copy of the test
/// harness, which ends the child with status 0.
fn child_exit(f: impl FnOnce() -> i32) -> ! {
    let code = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or(101);
    // SAFETY: ends the child without running the parent's destructors.
    unsafe { libc::_exit(code) }
}

#[test]
fn a_forked_child_dropping_an_inherited_view_leaves_the_parent_borrow() {
    let _g = alone();
    let t = Topic::new("forkview");
    let mut p = publisher(&t, 2);
    assert_eq!(p.publish(b"a"), Published::Ok);
    assert_eq!(p.publish(b"b"), Published::Ok);
    let mut s = subscriber(&t);
    let v = take_both(&mut s);
    // SAFETY: fork; the child drops its copy of the view and exits without unwinding.
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        child_exit(|| {
            drop(v);
            0
        });
    }
    assert_eq!(wait_child(pid), 0);
    assert_eq!(
        p.publish(b"c"),
        Published::Backpressure,
        "the parent still reads both slots"
    );
    assert_eq!(v.0.data(), b"a");
}

#[test]
fn an_inherited_subscription_takes_nothing_and_counts_no_reader() {
    let _g = alone();
    let t = Topic::new("forkrefuse");
    let mut p = publisher(&t, 2);
    let mut s = subscriber(&t);
    assert_eq!(p.publish(b"a"), Published::Ok);
    // SAFETY: fork; the child takes on its copy and exits without unwinding.
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        child_exit(|| {
            let none = s.take().is_none() && s.take_blocking(-1).is_none();
            i32::from(!(none && s.refused().no_reader == 2))
        });
    }
    assert_eq!(wait_child(pid), 0);
    assert_eq!(s.take().unwrap().data(), b"a");
    assert_eq!(s.refused().no_reader, 0);
}

#[test]
fn a_forked_child_subscribes_under_a_number_of_its_own() {
    let _g = alone();
    let t = Topic::new("forksub");
    let mut p = publisher(&t, 2);
    assert_eq!(p.publish(b"a"), Published::Ok);
    assert_eq!(p.publish(b"b"), Published::Ok);
    let _parent = subscriber(&t);
    // SAFETY: fork; the child takes a view on a channel of its own and exits holding it.
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        // Exits holding the borrows, as a killed process would.
        child_exit(|| {
            let mut s = subscriber(&t);
            std::mem::forget(take_both(&mut s));
            std::mem::forget(s);
            0
        });
    }
    assert_eq!(wait_child(pid), 0, "the child took both frames");
    // On the parent's number the dead child's borrows could never be taken back.
    assert_eq!(p.publish(b"c"), Published::Ok);
}

fn pipe() -> [libc::c_int; 2] {
    let mut fds = [0; 2];
    // SAFETY: fds is a valid out-array of two ints.
    assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
    fds
}

fn write_byte(fd: libc::c_int) {
    let b = 1u8;
    // SAFETY: b is a one-byte buffer.
    unsafe { libc::write(fd, (&raw const b).cast(), 1) };
}

fn read_byte(fd: libc::c_int) {
    let mut b = 0u8;
    // SAFETY: b is a one-byte buffer.
    unsafe { libc::read(fd, (&raw mut b).cast(), 1) };
}

extern "C" fn ignore(_: libc::c_int) {}

#[test]
fn a_signal_while_the_segment_is_reserved_does_not_fail_the_publisher() {
    let _g = shared();
    // SAFETY: sigaction is plain data; all zeros is a valid value.
    let mut sa: libc::sigaction = unsafe { std::mem::zeroed() };
    sa.sa_sigaction = ignore as *const () as libc::sighandler_t;
    // SAFETY: a handler that does nothing, installed without SA_RESTART as an application may.
    let installed = unsafe { libc::sigaction(libc::SIGUSR2, &raw const sa, std::ptr::null_mut()) };
    assert_eq!(installed, 0);
    // SAFETY: names the calling thread.
    let me = unsafe { libc::pthread_self() };
    let created = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let poker = {
        let created = std::sync::Arc::clone(&created);
        // A steady 1 kHz, as a profiler's timer, for at most 3 s.
        std::thread::spawn(move || {
            for _ in 0..3000 {
                if created.load(Ordering::Relaxed) {
                    break;
                }
                // SAFETY: the target thread outlives this one, which is joined below.
                unsafe { libc::pthread_kill(me, libc::SIGUSR2) };
                std::thread::sleep(Duration::from_millis(1));
            }
        })
    };
    let t = Topic::new("eintr");
    let start = Instant::now();
    let p = Channel::create(
        &t.0,
        1 << 20,
        64,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    );
    let took = start.elapsed();
    created.store(true, Ordering::Relaxed);
    poker.join().unwrap();
    assert!(p.is_ok(), "{:?}", p.err());
    assert!(
        took < Duration::from_secs(1),
        "{took:?} under steady signals"
    );
}

/// The `VmFlags` of every mapping of `file`, in address order.
fn vm_flags_of(file: &str) -> Vec<String> {
    let text = std::fs::read_to_string("/proc/self/smaps").unwrap();
    let mut out = Vec::new();
    let mut ours = false;
    for line in text.lines() {
        let head = line.split_whitespace().next().unwrap_or("");
        if head.contains('-') && !head.ends_with(':') {
            ours = line.contains(file);
        } else if let (true, Some(flags)) = (ours, line.strip_prefix("VmFlags:")) {
            out.push(flags.trim().to_owned());
        }
    }
    out
}

/// Whatever the memory policy, the control plane is locked and the payload kept out of core dumps,
/// on both ends.
#[test]
fn the_control_plane_is_locked_and_the_payload_left_out_of_dumps() {
    let _g = shared();
    let t = Topic::new("pinned");
    let _p = publisher(&t, 4);
    let file = format!("{}.e{}", &t.0[1..], read_channel_stats(&t.0).epoch);
    let alone = vm_flags_of(&file);
    let _s = subscriber(&t);
    let both = vm_flags_of(&file);
    // Each end maps the control plane and the payload apart, the subscriber's payload read-only.
    assert_eq!((alone.len(), both.len()), (2, 4), "{both:?}");
    for (i, flags) in both.iter().enumerate() {
        let flags: Vec<&str> = flags.split(' ').collect();
        let (control, payload) = (i % 2 == 0, i % 2 == 1);
        assert_eq!(flags.contains(&"lo"), control, "{both:?}");
        assert_eq!(flags.contains(&"dd"), payload, "{both:?}");
    }
    if cfg!(debug_assertions) {
        let filter = std::fs::read_to_string("/proc/self/coredump_filter").unwrap();
        assert_ne!(u32::from_str_radix(filter.trim(), 16).unwrap() & 1 << 3, 0);
    }
}

/// A re-attach leaves the old mapping to a helper thread, whose munmap frees the pages of a segment
/// no publisher holds; it must still go.
#[test]
fn a_replaced_mapping_is_let_go_after_a_reattach() {
    let _g = shared();
    let t = Topic::new("retired");
    let p = publisher(&t, 2);
    let old = format!("{}.e{}", &t.0[1..], read_channel_stats(&t.0).epoch);
    let mut s = subscriber(&t);
    assert!(s.take().is_none());
    drop(p);
    let mut p = publisher(&t, 2);
    assert_eq!(p.publish(b"x"), Published::Ok);
    assert_eq!(s.take().unwrap().data(), b"x");
    let start = Instant::now();
    while !vm_flags_of(&old).is_empty() {
        assert!(
            start.elapsed() < Duration::from_secs(2),
            "the old mapping stayed"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn subscriber_count_follows_the_subscribers_of_this_process() {
    let _g = shared();
    let t = Topic::new("count");
    let p = publisher(&t, 2);
    assert_eq!(p.subscriber_count(), 0);
    let a = subscriber(&t);
    assert_eq!(p.subscriber_count(), 1);
    // One process holds one number, however many subscriptions it has.
    let b = subscriber(&t);
    assert_eq!(p.subscriber_count(), 1);
    drop(a);
    assert_eq!(p.subscriber_count(), 1);
    drop(b);
    assert_eq!(p.subscriber_count(), 0);
}

/// Runs only as the child of `a_killed_subscriber_leaves_the_count`: subscribes, says so, and
/// waits to be killed.
#[test]
fn child_subscribes() {
    let Ok(name) = std::env::var(CHILD_ENV) else {
        return;
    };
    let _s = Channel::open(&name, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    println!("subscribed");
    std::thread::sleep(Duration::from_secs(60));
}

#[test]
fn a_killed_subscriber_leaves_the_count() {
    let _g = alone();
    let t = Topic::new("count_kill");
    let p = publisher(&t, 2);
    let mut child = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "child_subscribes",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(CHILD_ENV, &t.0)
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut out = BufReader::new(child.stdout.take().unwrap());
    let mut line = String::new();
    while !line.contains("subscribed") {
        line.clear();
        assert_ne!(
            out.read_line(&mut line).unwrap(),
            0,
            "child exited before subscribing"
        );
    }
    assert_eq!(p.subscriber_count(), 1);
    child.kill().unwrap();
    child.wait().unwrap();
    assert_eq!(p.subscriber_count(), 0);
}
