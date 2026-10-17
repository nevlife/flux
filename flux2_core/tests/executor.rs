use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread;
use std::time::{Duration, Instant};

use flux2_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, Error, Executor, MAX_DRAIN, MemoryPolicy, NO_SCHEMA,
    PartitionedExecutor, Published, PublisherWatch, QoS, Result, Source, read_channel_stats,
    signpost_name,
};

const SLOTS: u32 = 8;

fn lock<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

fn ms_since(t0: Instant) -> u128 {
    t0.elapsed().as_millis()
}

fn executor<'a>() -> Executor<'a> {
    Executor::new().unwrap()
}

const DISABLE_FUTEX_WAITV_ENV: &str = "FLUX2_DISABLE_FUTEX_WAITV";

fn forced_fallback() -> bool {
    std::env::var_os(DISABLE_FUTEX_WAITV_ENV).is_some()
}

/// Thread ids, sorted.
fn parker_threads() -> Vec<String> {
    let mut tids: Vec<String> = std::fs::read_dir("/proc/self/task")
        .unwrap()
        .map(|t| t.unwrap().path())
        .filter(|t| {
            std::fs::read_to_string(t.join("comm")).is_ok_and(|c| c.trim_end() == "flux2-parker")
        })
        .map(|t| t.file_name().unwrap().to_string_lossy().into_owned())
        .collect();
    tids.sort();
    tids
}

const ALONE_ENV: &str = "FLUX2_TEST_ALONE";

/// True in a child running `test` alone on the parker fallback, so every parker in the process is
/// that test's. Elsewhere it starts that child, once, and returns false.
fn alone_on_the_fallback(test: &str) -> bool {
    if std::env::var_os(ALONE_ENV).is_some() {
        return true;
    }
    if !forced_fallback() {
        let out = Command::new(std::env::current_exe().unwrap())
            .args([test, "--exact"])
            .env(DISABLE_FUTEX_WAITV_ENV, "1")
            .env(ALONE_ENV, "1")
            .output()
            .unwrap();
        let stdout = String::from_utf8_lossy(&out.stdout);
        assert!(
            out.status.success() && stdout.contains("1 passed"),
            "{stdout}{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
    false
}

/// Waits up to `timeout_ns` for a pass that delivers, as one turn of `spin` does.
fn spin_once(ex: &mut Executor<'_>, timeout_ns: i64) -> usize {
    let deadline = Instant::now() + Duration::from_nanos(timeout_ns.unsigned_abs());
    loop {
        let n = ex.dispatch().unwrap();
        let left = deadline.saturating_duration_since(Instant::now());
        if n > 0 || left.is_zero() {
            return n;
        }
        ex.wait_for_work(i64::try_from(left.as_nanos()).unwrap())
            .unwrap();
    }
}

/// A shm topic with its publisher.
struct Feed {
    topic: String,
    publisher: Arc<Mutex<Option<Channel>>>,
}

impl Feed {
    fn new(test: &str) -> Feed {
        static N: AtomicU32 = AtomicU32::new(0);
        let key = format!(
            "/x{}_{}_{test}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        );
        let topic = signpost_name(&key, NO_SCHEMA).unwrap();
        let p = Channel::create(
            &topic,
            64,
            SLOTS,
            DEFAULT_MAX_SUBSCRIBERS,
            NO_SCHEMA,
            MemoryPolicy::default(),
        )
        .unwrap();
        Feed {
            topic,
            publisher: Arc::new(Mutex::new(Some(p))),
        }
    }

    fn publish(&self, id: u8) {
        let mut p = lock(&self.publisher);
        assert_eq!(p.as_mut().unwrap().publish(&[id]), Published::Ok);
    }

    fn publisher(&self) -> impl Fn(u8) + Send + 'static {
        let p = Arc::clone(&self.publisher);
        move |id| {
            let _ = lock(&p).as_mut().unwrap().publish(&[id]);
        }
    }

    /// The old publisher must be gone before the new one is created: one publisher per topic.
    fn restart(&self) {
        let mut p = lock(&self.publisher);
        *p = None;
        *p = Some(
            Channel::create(
                &self.topic,
                64,
                SLOTS,
                DEFAULT_MAX_SUBSCRIBERS,
                NO_SCHEMA,
                MemoryPolicy::default(),
            )
            .unwrap(),
        );
    }

    fn gate(&self) -> i32 {
        read_channel_stats(&self.topic).waiters
    }

    fn source(&self, tag: u32) -> (TestSource, Arc<Stats>) {
        let mut s = self.lazy_source(tag);
        s.st.attachable.store(true, Ordering::Relaxed);
        assert!(s.attach().unwrap());
        let st = Arc::clone(&s.st);
        (s, st)
    }

    fn lazy_source(&self, tag: u32) -> TestSource {
        TestSource {
            topic: self.topic.clone(),
            tag,
            ch: None,
            st: Arc::new(Stats::default()),
            on_deliver: None,
        }
    }
}

#[derive(Default)]
struct Stats {
    attach_calls: AtomicU32,
    delivered: AtomicU32,
    last_id: AtomicU8,
    attachable: AtomicBool,
    fail_next: AtomicBool,
    orphan_next: AtomicBool,
    log: Mutex<Vec<u32>>,
}

impl Stats {
    fn delivered(&self) -> u32 {
        self.delivered.load(Ordering::Relaxed)
    }
    fn last_id(&self) -> u8 {
        self.last_id.load(Ordering::Relaxed)
    }
}

struct TestSource {
    topic: String,
    tag: u32,
    ch: Option<Channel>,
    st: Arc<Stats>,
    on_deliver: Option<Box<dyn FnMut() + Send>>,
}

impl TestSource {
    /// Shares the log with other sources, so it shows the order the pass visited them in.
    fn log_into(mut self, log: &Arc<Stats>) -> TestSource {
        let shared = Arc::clone(log);
        let tag = self.tag;
        let mut prev = self.on_deliver.take();
        self.on_deliver = Some(Box::new(move || {
            lock(&shared.log).push(tag);
            if let Some(f) = prev.as_mut() {
                f();
            }
        }));
        self
    }

    fn on_deliver(mut self, f: impl FnMut() + Send + 'static) -> TestSource {
        self.on_deliver = Some(Box::new(f));
        self
    }
}

impl Source for TestSource {
    fn attach(&mut self) -> Result<bool> {
        self.st.attach_calls.fetch_add(1, Ordering::Relaxed);
        if self.ch.is_some() {
            return Ok(true);
        }
        if !self.st.attachable.load(Ordering::Relaxed) {
            return Ok(false);
        }
        let Ok(mut ch) = Channel::open(&self.topic, NO_SCHEMA, MemoryPolicy::default()) else {
            return Ok(false);
        };
        // Past depth 1, so "one pass drains what arrived" differs from "one frame per pass".
        ch.set_qos(QoS::new(SLOTS).unwrap()).unwrap();
        self.ch = Some(ch);
        Ok(true)
    }

    fn deliver_one(&mut self) -> Result<bool> {
        if self.st.fail_next.swap(false, Ordering::Relaxed) {
            return Err(Error::Callback("test: deliver_one failed".into()));
        }
        if self.st.orphan_next.swap(false, Ordering::Relaxed) {
            self.ch = None;
            return Ok(false);
        }
        let Some(v) = self.ch.as_mut().and_then(Channel::take) else {
            return Ok(false);
        };
        self.st.last_id.store(v.data()[0], Ordering::Relaxed);
        self.st.delivered.fetch_add(1, Ordering::Relaxed);
        drop(v);
        if let Some(f) = self.on_deliver.as_mut() {
            f();
        }
        Ok(true)
    }

    fn channel(&self) -> Option<&Channel> {
        self.ch.as_ref()
    }
}

fn order(log: &Stats) -> Vec<u32> {
    lock(&log.log).clone()
}

#[test]
fn dispatch_drains_every_registered_source() {
    let (fa, fb) = (Feed::new("drain_a"), Feed::new("drain_b"));
    let (a, sa) = fa.source(1);
    let (b, sb) = fb.source(2);
    let mut ex = executor();
    ex.add(a, 0).unwrap();
    ex.add(b, 0).unwrap();
    assert_eq!(ex.size(), 2);
    fa.publish(1);
    fa.publish(2);
    fb.publish(3);
    assert_eq!(ex.dispatch().unwrap(), 3);
    assert_eq!((sa.delivered(), sb.delivered()), (2, 1));
    assert_eq!(ex.dispatch().unwrap(), 0);
}

fn two_sources(test: &str, pa: i32, pb: i32) -> (Feed, Feed, Executor<'static>, Arc<Stats>) {
    let (fa, fb) = (
        Feed::new(&format!("{test}_a")),
        Feed::new(&format!("{test}_b")),
    );
    let log = Arc::new(Stats::default());
    let mut ex = executor();
    ex.add(fa.source(1).0.log_into(&log), pa).unwrap();
    ex.add(fb.source(2).0.log_into(&log), pb).unwrap();
    (fa, fb, ex, log)
}

#[test]
fn equal_priorities_start_in_registration_order() {
    let (fa, fb, mut ex, log) = two_sources("regorder", 0, 0);
    fb.publish(1);
    fb.publish(2);
    fa.publish(3);
    fa.publish(4);
    assert_eq!(ex.dispatch().unwrap(), 4);
    assert_eq!(order(&log), [1, 2, 1, 2]);
}

#[test]
fn a_higher_priority_is_visited_first() {
    let (fa, fb, mut ex, log) = two_sources("prio", 0, 10);
    fa.publish(1);
    fa.publish(2);
    fb.publish(3);
    fb.publish(4);
    assert_eq!(ex.dispatch().unwrap(), 4);
    assert_eq!(order(&log), [2, 2, 1, 1]);
}

#[test]
fn a_negative_priority_goes_last() {
    let (fa, fb, mut ex, log) = two_sources("negprio", -1, 0);
    fa.publish(1);
    fb.publish(2);
    assert_eq!(ex.dispatch().unwrap(), 2);
    assert_eq!(order(&log), [2, 1]);
}

#[test]
fn equal_priorities_take_turns_frame_by_frame() {
    let feeds = [
        Feed::new("turn_a"),
        Feed::new("turn_b"),
        Feed::new("turn_c"),
    ];
    let log = Arc::new(Stats::default());
    let mut ex = executor();
    for (tag, f) in (1..).zip(&feeds) {
        ex.add(f.source(tag).0.log_into(&log), 0).unwrap();
    }
    for i in 1..=3 {
        for f in &feeds {
            f.publish(i);
        }
    }
    assert_eq!(ex.dispatch().unwrap(), 9);
    assert_eq!(order(&log), [1, 2, 3, 1, 2, 3, 1, 2, 3]);
}

#[test]
fn a_saturated_source_does_not_starve_an_equal_peer() {
    let (fa, fb) = (Feed::new("sat_a"), Feed::new("sat_b"));
    let again = fa.publisher();
    let a = fa.source(1).0.on_deliver(move || again(9));
    let (b, sb) = fb.source(2);
    let mut ex = executor();
    ex.add(a, 0).unwrap();
    ex.add(b, 0).unwrap();
    fa.publish(1);
    fb.publish(1);
    ex.dispatch().unwrap();
    assert_eq!(sb.delivered(), 1);
}

#[test]
fn a_frame_arriving_during_a_callback_goes_before_the_next_lower_one() {
    let (fh, fl) = (Feed::new("arrive_h"), Feed::new("arrive_l"));
    let log = Arc::new(Stats::default());
    let high = fh.publisher();
    let mut once = true;
    let low = fl
        .source(2)
        .0
        .on_deliver(move || {
            if std::mem::take(&mut once) {
                high(3);
            }
        })
        .log_into(&log);
    let mut ex = executor();
    ex.add(fh.source(1).0.log_into(&log), 10).unwrap();
    ex.add(low, 0).unwrap();
    fl.publish(1);
    fl.publish(2);
    assert_eq!(ex.dispatch().unwrap(), 3);
    assert_eq!(order(&log), [2, 1, 2]);
}

#[test]
fn the_pass_budget_covers_every_channel_together() {
    let (fa, fb, mut ex, log) = two_sources("budget", 10, 0);
    ex.set_pass_budget(3).unwrap();
    for i in 1..=2 {
        fa.publish(i);
        fb.publish(i);
    }
    assert_eq!(ex.dispatch().unwrap(), 3);
    assert_eq!(order(&log), [1, 1, 2]);
    assert_eq!(ex.dispatch().unwrap(), 1);
    assert_eq!(order(&log), [1, 1, 2, 2]);
}

#[test]
fn a_yield_ends_the_pass_at_the_next_callback_boundary() {
    let f = Feed::new("yield");
    let mut ex = executor();
    ex.add(f.source(1).0, 0).unwrap();
    (1..=8).for_each(|i| f.publish(i));
    let mut asked = 0;
    let n = ex
        .dispatch_yielding(|| {
            asked += 1;
            true
        })
        .unwrap();
    assert_eq!((n, asked), (1, 1));
    assert!(ex.has_more());
    assert!(ex.dispatch().unwrap() > 1);
}

#[test]
fn a_yield_that_never_fires_changes_nothing() {
    let (fa, fb, mut ex, log) = two_sources("noyield", 10, 0);
    ex.set_pass_budget(3).unwrap();
    for i in 1..=2 {
        fa.publish(i);
        fb.publish(i);
    }
    assert_eq!(ex.dispatch_yielding(|| false).unwrap(), 3);
    assert_eq!(order(&log), [1, 1, 2]);
}

#[test]
fn a_budget_cut_pass_says_there_is_more() {
    let f = Feed::new("more");
    let mut ex = executor();
    ex.set_pass_budget(2).unwrap();
    ex.add(f.source(1).0, 0).unwrap();
    (1..=3).for_each(|i| f.publish(i));
    assert_eq!(ex.dispatch().unwrap(), 2);
    assert!(ex.has_more());
    assert_eq!(ex.dispatch().unwrap(), 1);
    assert!(!ex.has_more(), "the pass ended on an empty channel");
    assert_eq!(ex.dispatch().unwrap(), 0);
    assert!(!ex.has_more());
}

#[test]
fn the_pass_budget_rejects_an_empty_pass() {
    let mut ex = executor();
    assert_eq!(ex.pass_budget(), MAX_DRAIN);
    assert!(matches!(
        ex.set_pass_budget(0),
        Err(Error::InvalidArgument(_))
    ));
    ex.set_pass_budget(1).unwrap();
    assert_eq!(ex.pass_budget(), 1);
}

#[test]
fn dispatch_retries_a_source_that_cannot_attach_yet() {
    let f = Feed::new("late");
    let s = f.lazy_source(1);
    let st = Arc::clone(&s.st);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    assert_eq!(ex.dispatch().unwrap(), 0);
    assert_eq!(ex.dispatch().unwrap(), 0);
    assert!(st.attach_calls.load(Ordering::Relaxed) >= 2);
    assert_eq!(f.gate(), 0);

    st.attachable.store(true, Ordering::Relaxed);
    assert_eq!(ex.dispatch().unwrap(), 0);
    assert_eq!(f.gate(), 1, "attached and armed");
    f.publish(7);
    assert_eq!(ex.dispatch().unwrap(), 1);
    assert_eq!(st.last_id(), 7);
}

/// Opens through a watch, which a publisher coming up wakes.
struct WatchedSource {
    watch: PublisherWatch,
    ch: Option<Channel>,
    delivered: Arc<AtomicU32>,
}

impl Source for WatchedSource {
    fn attach(&mut self) -> Result<bool> {
        match Channel::open_watched(
            &self.watch,
            NO_SCHEMA,
            QoS::default(),
            MemoryPolicy::default(),
        ) {
            Ok(ch) => self.ch = Some(ch),
            Err(Error::NotReady(_)) => return Ok(false),
            Err(e) => return Err(e),
        }
        Ok(true)
    }

    fn deliver_one(&mut self) -> Result<bool> {
        let got = self.ch.as_mut().and_then(Channel::take).is_some();
        self.delivered.fetch_add(u32::from(got), Ordering::Relaxed);
        Ok(got)
    }

    fn channel(&self) -> Option<&Channel> {
        self.ch.as_ref()
    }

    fn watch(&self) -> Option<&PublisherWatch> {
        Some(&self.watch)
    }
}

/// A source made before its publisher sleeps on the signpost, and the publisher's arrival wakes
/// it: with a tick this long, a retry once a tick would deliver nothing in time.
#[test]
fn a_publisher_coming_up_wakes_a_source_waiting_for_it() {
    let topic = signpost_name(&format!("/x{}_lateattach", std::process::id()), NO_SCHEMA).unwrap();
    let delivered = Arc::new(AtomicU32::new(0));
    let mut ex = executor();
    ex.add(
        WatchedSource {
            watch: PublisherWatch::open(&topic).unwrap(),
            ch: None,
            delivered: Arc::clone(&delivered),
        },
        0,
    )
    .unwrap();
    let handle = ex.handle();
    let spin = thread::spawn(move || ex.spin(10_000_000_000));
    while !handle.is_spinning() {
        thread::sleep(Duration::from_millis(1));
    }
    thread::sleep(Duration::from_millis(50));
    let mut p = Channel::create(
        &topic,
        64,
        SLOTS,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    )
    .unwrap();
    let t0 = Instant::now();
    while delivered.load(Ordering::Relaxed) == 0 && ms_since(t0) < 2000 {
        assert_eq!(p.publish(&[1]), Published::Ok);
        thread::sleep(Duration::from_millis(1));
    }
    let took = ms_since(t0);
    handle.stop();
    spin.join().unwrap().unwrap();
    assert!(took < 500, "first frame after {took} ms");
}

#[test]
fn a_publish_from_another_thread_ends_the_wait() {
    let f = Feed::new("xthread");
    let (s, st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    ex.dispatch().unwrap();
    let publish = f.publisher();
    let t0 = Instant::now();
    let n = thread::scope(|sc| {
        sc.spawn(|| {
            thread::sleep(Duration::from_millis(100));
            publish(0x42);
        });
        spin_once(&mut ex, 5_000_000_000)
    });
    assert_eq!((n, st.last_id()), (1, 0x42));
    assert!(ms_since(t0) < 3000, "the publish did not wake the executor");
}

#[test]
fn an_interrupt_ends_exactly_one_wait() {
    let f = Feed::new("interrupt");
    let run = AtomicBool::new(true);
    let mut ex = executor();
    ex.add(f.source(1).0, 0).unwrap();
    ex.on_pass(|| {
        run.store(false, Ordering::Relaxed);
        Ok(())
    });
    ex.handle().interrupt();
    let t0 = Instant::now();
    ex.spin_while(&run, 2_000_000_000).unwrap();
    assert!(ms_since(t0) < 1000, "interrupt did not end the wait");
    run.store(true, Ordering::Relaxed);
    let t0 = Instant::now();
    ex.spin_while(&run, 300_000_000).unwrap();
    assert!(
        ms_since(t0) >= 250,
        "the consumed interrupt ended a second wait"
    );
}

/// A stale interrupt would end the next wait at the first wake, with nothing delivered.
#[test]
fn clear_interrupt_drops_an_interrupt_no_call_consumed() {
    let f = Feed::new("clearint");
    let run = AtomicBool::new(true);
    let mut ex = executor();
    ex.add(f.source(1).0, 0).unwrap();
    ex.on_pass(|| {
        run.store(false, Ordering::Relaxed);
        Ok(())
    });
    let h = ex.handle();
    h.interrupt();
    ex.wait_for_work(1_000_000_000).unwrap();
    ex.clear_interrupt();
    let t0 = Instant::now();
    thread::scope(|sc| {
        sc.spawn(|| {
            thread::sleep(Duration::from_millis(50));
            h.wake();
        });
        ex.spin_while(&run, 300_000_000).unwrap();
    });
    assert!(ms_since(t0) >= 250, "a stale interrupt ended a later wait");
}

#[test]
fn a_wake_breaks_the_wait_and_outlives_the_executor() {
    let f = Feed::new("waker");
    let h = {
        let mut ex = executor();
        ex.add(f.source(1).0, 0).unwrap();
        let h = ex.handle();
        ex.dispatch().unwrap();
        let t0 = Instant::now();
        thread::scope(|sc| {
            sc.spawn(|| {
                thread::sleep(Duration::from_millis(100));
                h.wake();
            });
            ex.wait_for_work(5_000_000_000).unwrap();
        });
        assert!(ms_since(t0) < 3000, "wake did not break the wait");
        h
    };
    h.wake();
    assert!(!h.is_spinning());
}

#[test]
fn a_pass_budget_below_the_channel_count_starves_no_channel() {
    let feeds: Vec<Feed> = (0..20).map(|i| Feed::new(&format!("starve{i}"))).collect();
    let mut ex = executor();
    ex.set_pass_budget(16).unwrap();
    let last = feeds.last().unwrap().source(1);
    for f in &feeds[..19] {
        ex.add(f.source(0).0, 0).unwrap();
    }
    ex.add(last.0, 0).unwrap();
    ex.dispatch().unwrap();
    feeds[19].publish(0x42);
    let mut delivered = 0;
    for _ in 0..8 {
        if delivered > 0 {
            break;
        }
        delivered += ex.dispatch().unwrap();
    }
    assert_eq!((delivered, last.1.last_id()), (1, 0x42));
}

#[test]
fn every_channel_of_a_full_executor_wakes() {
    let feeds: Vec<Feed> = (0..127).map(|i| Feed::new(&format!("full{i}"))).collect();
    let mut ex = executor();
    let stats: Vec<Arc<Stats>> = feeds
        .iter()
        .map(|f| {
            let (s, st) = f.source(0);
            ex.add(s, 0).unwrap();
            st
        })
        .collect();
    ex.dispatch().unwrap();
    for i in [0usize, 63, 64, 126] {
        let id = u8::try_from(i).unwrap();
        feeds[i].publish(id);
        let t0 = Instant::now();
        assert_eq!(spin_once(&mut ex, 5_000_000_000), 1, "channel {i}");
        assert!(ms_since(t0) < 3000, "channel {i} was never woken");
        assert_eq!(stats[i].last_id(), id);
    }
}

#[test]
fn wakes_reach_both_entries_when_priority_reverses_the_visit() {
    let (fl, fh, mut ex, log) = two_sources("reverse", 0, 10);
    assert_eq!(ex.dispatch().unwrap(), 0);
    let (low, high) = (fl.publisher(), fh.publisher());
    let mut want = Vec::new();
    for round in 0..4u8 {
        for (tag, publish) in [(1u32, &low), (2u32, &high)] {
            let t0 = Instant::now();
            let n = thread::scope(|sc| {
                sc.spawn(|| {
                    thread::sleep(Duration::from_millis(30));
                    publish(round);
                });
                spin_once(&mut ex, 3_000_000_000)
            });
            want.push(tag);
            assert_eq!(n, 1, "round {round}");
            assert_eq!(order(&log), want, "round {round}");
            assert!(
                ms_since(t0) < 1500,
                "round {round}: the wake never reached this entry"
            );
        }
    }
}

#[test]
fn a_stop_before_spin_ends_it_and_is_cleared_on_the_way_out() {
    let f = Feed::new("stopfirst");
    let (s, st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    let h = ex.handle();
    h.stop();
    let t0 = Instant::now();
    ex.spin(5_000_000_000).unwrap();
    assert!(ms_since(t0) < 2000, "a stop placed before spin was lost");

    thread::scope(|sc| {
        sc.spawn(|| ex.spin(20_000_000).unwrap());
        while !h.is_spinning() {
            thread::sleep(Duration::from_millis(1));
        }
        f.publish(3);
        while st.last_id() != 3 {
            thread::sleep(Duration::from_millis(1));
        }
        h.stop();
    });
    assert!(!h.is_spinning());
}

#[test]
fn a_cleared_run_flag_ends_the_spin() {
    let f = Feed::new("runflag");
    let mut ex = executor();
    ex.add(f.source(1).0, 0).unwrap();
    let t0 = Instant::now();
    ex.spin_while(&AtomicBool::new(false), 5_000_000_000)
        .unwrap();
    assert!(ms_since(t0) < 2000);
}

#[test]
fn on_pass_runs_once_per_spin_pass() {
    let f = Feed::new("onpass");
    let passes = Arc::new(AtomicU32::new(0));
    let mut ex = executor();
    ex.add(f.source(1).0, 0).unwrap();
    let p = Arc::clone(&passes);
    ex.on_pass(move || {
        p.fetch_add(1, Ordering::Relaxed);
        Ok(())
    });
    let h = ex.handle();
    thread::scope(|sc| {
        sc.spawn(|| ex.spin(10_000_000).unwrap());
        while passes.load(Ordering::Relaxed) < 3 {
            thread::sleep(Duration::from_millis(2));
        }
        h.stop();
    });
}

#[test]
fn an_error_from_deliver_ends_the_spin_and_leaves_it_spinnable() {
    let f = Feed::new("fail");
    let (s, st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    let h = ex.handle();
    f.publish(1);
    st.fail_next.store(true, Ordering::Relaxed);
    assert!(matches!(ex.spin(20_000_000), Err(Error::Callback(_))));
    assert!(!h.is_spinning());
    thread::scope(|sc| {
        sc.spawn(|| ex.spin(10_000_000).unwrap());
        while st.last_id() != 1 {
            thread::sleep(Duration::from_millis(1));
        }
        h.stop();
    });
}

/// Hands every view to `held`, as a callback that queues frames for a worker does.
struct Holding {
    ch: Channel,
    held: Arc<Mutex<Vec<flux2_core::FrameView>>>,
}

impl Source for Holding {
    fn attach(&mut self) -> Result<bool> {
        Ok(true)
    }
    fn deliver_one(&mut self) -> Result<bool> {
        let Some(v) = self.ch.take() else {
            return Ok(false);
        };
        lock(&self.held).push(v);
        Ok(true)
    }
    fn channel(&self) -> Option<&Channel> {
        Some(&self.ch)
    }
}

/// A frame refused for `max_borrow` goes out when a view is dropped, not on the next tick.
#[test]
fn a_view_release_wakes_a_source_at_max_borrow() {
    let f = Feed::new("release");
    let mut ch = Channel::open(&f.topic, NO_SCHEMA, MemoryPolicy::default()).unwrap();
    ch.set_qos(QoS::new(8).unwrap().max_borrow(2).unwrap())
        .unwrap();
    let held = Arc::new(Mutex::new(Vec::new()));
    let mut ex = executor();
    ex.add(
        Holding {
            ch,
            held: Arc::clone(&held),
        },
        0,
    )
    .unwrap();
    let h = ex.handle();
    let count = || lock(&held).len();
    thread::scope(|sc| {
        sc.spawn(|| ex.spin(10_000_000_000).unwrap());
        for id in 1..=2 {
            f.publish(id);
            while count() < usize::from(id) {
                thread::sleep(Duration::from_millis(1));
            }
        }
        f.publish(3);
        thread::sleep(Duration::from_millis(20));
        assert_eq!(count(), 2, "taken past max_borrow");
        let released = Instant::now();
        lock(&held).clear();
        while count() == 0 && released.elapsed() < Duration::from_secs(5) {
            thread::sleep(Duration::from_millis(1));
        }
        let waited = released.elapsed();
        h.stop();
        assert!(
            waited < Duration::from_secs(1),
            "{waited:?}: waited for the tick"
        );
    });
}

struct Mismatched;

impl Source for Mismatched {
    fn attach(&mut self) -> Result<bool> {
        Err(Error::Mismatch("test: wrong fingerprint".into()))
    }
    fn deliver_one(&mut self) -> Result<bool> {
        Ok(false)
    }
    fn channel(&self) -> Option<&Channel> {
        None
    }
}

#[test]
fn one_executor_holds_at_most_127_sources() {
    let mut ex = executor();
    for _ in 0..127 {
        ex.add(Mismatched, 0).unwrap();
    }
    // An executor of a PartitionedExecutor's group is full too, so the advice cannot be to use one.
    let Err(Error::InvalidArgument(why)) = ex.add(Mismatched, 0) else {
        panic!("the 128th source was taken");
    };
    assert!(why.contains("callback groups"), "{why}");
}

/// The publisher's own channel has no subscriber number until its first take, and the executor's
/// count must leave from where it went in.
struct OwnChannel<'c>(&'c mut Channel);

impl Source for OwnChannel<'_> {
    fn attach(&mut self) -> Result<bool> {
        Ok(true)
    }
    fn deliver_one(&mut self) -> Result<bool> {
        Ok(self.0.take().is_some())
    }
    fn channel(&self) -> Option<&Channel> {
        Some(self.0)
    }
}

#[test]
fn a_number_taken_while_bound_leaves_the_gate_balanced() {
    let key = format!("/x{}_own_gate", std::process::id());
    let topic = signpost_name(&key, NO_SCHEMA).unwrap();
    let mut p = Channel::create(
        &topic,
        64,
        SLOTS,
        DEFAULT_MAX_SUBSCRIBERS,
        NO_SCHEMA,
        MemoryPolicy::default(),
    )
    .unwrap();
    assert_eq!(p.publish(&[1]), Published::Ok);
    {
        let mut ex = executor();
        ex.add(OwnChannel(&mut p), 0).unwrap();
        assert_eq!(ex.dispatch().unwrap(), 1);
        assert_eq!(read_channel_stats(&topic).waiters, 1);
    }
    assert_eq!(read_channel_stats(&topic).waiters, 0);
}

#[test]
fn an_error_from_attach_ends_the_spin() {
    let mut ex = executor();
    ex.add(Mismatched, 0).unwrap();
    assert!(matches!(ex.spin(20_000_000), Err(Error::Mismatch(_))));
}

#[test]
fn the_waiter_gate_is_balanced_across_the_executor_lifetime() {
    let f = Feed::new("gate");
    {
        let mut ex = executor();
        ex.add(f.source(1).0, 0).unwrap();
        assert_eq!(f.gate(), 0);
        ex.dispatch().unwrap();
        assert_eq!(f.gate(), 1);
        ex.dispatch().unwrap();
        ex.dispatch().unwrap();
        assert_eq!(f.gate(), 1, "repeated passes stacked waiter counts");
    }
    assert_eq!(f.gate(), 0, "the executor did not release its waiter count");
}

#[test]
fn an_orphaned_channel_restarts_the_entry_without_leaking_the_gate() {
    let f = Feed::new("orphan");
    let (s, st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    f.publish(1);
    assert_eq!(ex.dispatch().unwrap(), 1);
    assert_eq!(f.gate(), 1);

    st.orphan_next.store(true, Ordering::Relaxed);
    assert_eq!(ex.dispatch().unwrap(), 0);
    assert_eq!(
        f.gate(),
        0,
        "the waiter count stayed on the dropped channel's block"
    );
    assert_eq!(ex.dispatch().unwrap(), 0);
    assert_eq!(f.gate(), 1);
    f.publish(2);
    assert_eq!(ex.dispatch().unwrap(), 1);
    assert_eq!(st.last_id(), 2);
}

#[test]
fn delivery_survives_a_publisher_restart() {
    let f = Feed::new("restart");
    let (s, st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    f.publish(1);
    assert_eq!(ex.dispatch().unwrap(), 1);
    assert_eq!(st.last_id(), 1);

    f.restart();
    let mut recovered = false;
    for _ in 0..400 {
        f.publish(2);
        if ex.dispatch().unwrap() > 0 && st.last_id() == 2 {
            recovered = true;
            break;
        }
    }
    assert!(recovered, "delivery stopped across a publisher restart");

    // The wait must be armed on the new word too: a publish alone ends it.
    let publish = f.publisher();
    let t0 = Instant::now();
    let woke = thread::scope(|sc| {
        sc.spawn(|| {
            thread::sleep(Duration::from_millis(100));
            publish(3);
        });
        spin_once(&mut ex, 5_000_000_000)
    });
    assert!(woke > 0);
    assert!(
        ms_since(t0) < 3000,
        "the wait stayed on the replaced segment's word"
    );
}

/// Released once per pass, so no thread is created inside the window a pass aims at.
struct Shot {
    fire: AtomicBool,
    quit: AtomicBool,
    delay_ns: std::sync::atomic::AtomicU64,
}

impl Shot {
    fn new() -> Shot {
        Shot {
            fire: AtomicBool::new(false),
            quit: AtomicBool::new(false),
            delay_ns: 0.into(),
        }
    }

    fn serve(&self, action: impl Fn()) {
        loop {
            while !self.fire.load(Ordering::Acquire) {
                if self.quit.load(Ordering::Relaxed) {
                    return;
                }
            }
            let t0 = Instant::now();
            let d = Duration::from_nanos(self.delay_ns.load(Ordering::Relaxed));
            while t0.elapsed() < d {}
            action();
            self.fire.store(false, Ordering::Release);
        }
    }

    fn go(&self, delay_ns: u64) {
        self.delay_ns.store(delay_ns, Ordering::Relaxed);
        self.fire.store(true, Ordering::Release);
    }

    fn settle(&self) {
        while self.fire.load(Ordering::Acquire) {
            std::hint::spin_loop();
        }
    }
}

// The offset sweeps a nanosecond at a time across the pass's entry path. Three kinds of pass,
// because an interrupt on every pass would mask a lost frame re-arm.
#[test]
fn a_merged_wait_loses_neither_a_frame_nor_an_interrupt() {
    const PER_KIND: u64 = 150;
    const TIMEOUT_NS: i64 = 100_000_000;
    let f = Feed::new("merged");
    let (s, st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    ex.dispatch().unwrap();
    let h = ex.handle();
    let publish = f.publisher();
    let ok = AtomicU32::new(0);
    let (publisher, interrupter) = (Shot::new(), Shot::new());
    let mut stalls = [0u32; 3];
    thread::scope(|sc| {
        sc.spawn(|| {
            publisher.serve(|| {
                publish(1);
                ok.fetch_add(1, Ordering::Relaxed);
            });
        });
        sc.spawn(|| interrupter.serve(|| h.interrupt()));
        for i in 0..PER_KIND * 3 {
            let kind = usize::try_from(i % 3).unwrap();
            if kind != 1 {
                publisher.go(i);
            }
            if kind != 0 {
                interrupter.go(i + 1);
            }
            let t0 = Instant::now();
            if ex.dispatch().unwrap() == 0 {
                ex.wait_for_work(TIMEOUT_NS).unwrap();
                ex.dispatch().unwrap();
            }
            let took = t0.elapsed();
            publisher.settle();
            interrupter.settle();
            if took >= Duration::from_nanos(TIMEOUT_NS.unsigned_abs()) {
                stalls[kind] += 1;
            }
        }
        publisher.quit.store(true, Ordering::Relaxed);
        interrupter.quit.store(true, Ordering::Relaxed);
    });
    for _ in 0..200 {
        if st.delivered() >= ok.load(Ordering::Relaxed) {
            break;
        }
        spin_once(&mut ex, 5_000_000);
    }
    assert_eq!(stalls, [0, 0, 0], "frame only, control only, collision");
    assert_eq!(st.delivered(), ok.load(Ordering::Relaxed));
}

#[test]
fn partitions_run_on_their_own_threads_with_their_init() {
    let (fa, fb) = (Feed::new("part_a"), Feed::new("part_b"));
    let mut px = PartitionedExecutor::new().unwrap();
    let mut init_threads = Vec::new();
    let mut deliver_threads = Vec::new();
    for f in [&fa, &fb] {
        let seen = Arc::new(Mutex::new(None));
        let at_init = Arc::new(Mutex::new(None));
        let s = Arc::clone(&seen);
        let mut ex = executor();
        ex.add(
            f.source(1)
                .0
                .on_deliver(move || *lock(&s) = Some(thread::current().id())),
            0,
        )
        .unwrap();
        let part = px.add(ex);
        let i = Arc::clone(&at_init);
        px.on_thread_start(part, move || {
            *lock(&i) = Some(thread::current().id());
            Ok(())
        })
        .unwrap();
        init_threads.push(at_init);
        deliver_threads.push(seen);
    }
    assert_eq!(px.size(), 2);
    let h = px.handle();
    thread::scope(|sc| {
        sc.spawn(|| px.spin(20_000_000).unwrap());
        while !h.is_spinning() {
            thread::sleep(Duration::from_millis(1));
        }
        let t0 = Instant::now();
        while deliver_threads.iter().any(|d| lock(d).is_none()) && ms_since(t0) < 3000 {
            fa.publish(1);
            fb.publish(1);
            thread::sleep(Duration::from_millis(5));
        }
        h.stop();
    });
    let main = thread::current().id();
    let ids: Vec<_> = deliver_threads.iter().map(|d| lock(d).unwrap()).collect();
    assert_ne!(ids[0], ids[1]);
    assert!(!ids.contains(&main));
    for (d, i) in ids.iter().zip(&init_threads) {
        assert_eq!(
            Some(*d),
            *lock(i),
            "init ran on another thread than its partition"
        );
    }
}

#[test]
fn a_partition_error_stops_every_partition_and_is_returned() {
    let (fa, fb) = (Feed::new("perr_a"), Feed::new("perr_b"));
    let (a, sa) = fa.source(1);
    let mut px = PartitionedExecutor::new().unwrap();
    let mut ea = executor();
    ea.add(a, 0).unwrap();
    px.add(ea);
    let mut eb = executor();
    eb.add(fb.source(2).0, 0).unwrap();
    px.add(eb);
    let external_stopped = Arc::new(AtomicBool::new(false));
    let (run_flag, stop_flag) = (Arc::clone(&external_stopped), Arc::clone(&external_stopped));
    px.add_external(
        move || {
            while !run_flag.load(Ordering::Acquire) {
                thread::sleep(Duration::from_millis(1));
            }
            Ok(())
        },
        move || stop_flag.store(true, Ordering::Release),
    );
    sa.fail_next.store(true, Ordering::Relaxed);
    fa.publish(1);
    let t0 = Instant::now();
    assert!(matches!(px.spin(5_000_000_000), Err(Error::Callback(_))));
    assert!(ms_since(t0) < 3000);
    assert!(external_stopped.load(Ordering::Relaxed));

    external_stopped.store(false, Ordering::Relaxed);
    let h = px.handle();
    thread::scope(|sc| {
        sc.spawn(|| px.spin(20_000_000).unwrap());
        while !h.is_spinning() {
            thread::sleep(Duration::from_millis(1));
        }
        while sa.last_id() != 1 {
            fa.publish(1);
            thread::sleep(Duration::from_millis(5));
        }
        h.stop();
    });
}

#[test]
fn a_failed_thread_start_is_returned() {
    let f = Feed::new("initerr");
    let mut px = PartitionedExecutor::new().unwrap();
    let mut ex = executor();
    ex.add(f.source(1).0, 0).unwrap();
    let part = px.add(ex);
    px.on_thread_start(part, || Err(Error::Callback("test: init failed".into())))
        .unwrap();
    assert!(matches!(px.spin(5_000_000_000), Err(Error::Callback(_))));
    assert!(matches!(
        px.on_thread_start(9, || Ok(())),
        Err(Error::InvalidArgument(_))
    ));
}

#[test]
fn partitioned_stop_before_spin_and_interrupt_reach_children() {
    let f = Feed::new("pstop");
    let passes = Arc::new(AtomicU32::new(0));
    let mut px = PartitionedExecutor::new().unwrap();
    let mut ex = executor();
    ex.add(f.source(1).0, 0).unwrap();
    let p = Arc::clone(&passes);
    ex.on_pass(move || {
        p.fetch_add(1, Ordering::Relaxed);
        Ok(())
    });
    px.add(ex);
    let h = px.handle();
    h.stop();
    let t0 = Instant::now();
    px.spin(5_000_000_000).unwrap();
    assert!(ms_since(t0) < 2000, "a stop placed before spin was lost");

    thread::scope(|sc| {
        sc.spawn(|| px.spin(5_000_000_000).unwrap());
        while !h.is_spinning() {
            thread::sleep(Duration::from_millis(1));
        }
        thread::sleep(Duration::from_millis(50));
        let before = passes.load(Ordering::Relaxed);
        h.interrupt();
        let t0 = Instant::now();
        while passes.load(Ordering::Relaxed) == before && ms_since(t0) < 2000 {
            thread::sleep(Duration::from_millis(1));
        }
        assert!(
            passes.load(Ordering::Relaxed) > before,
            "the interrupt never reached the child"
        );
        h.stop();
    });
}

/// A publisher restart moves the source to another word. The executor re-aims the parker it made
/// on `add` instead of starting a thread, and the re-aimed parker hears frames on the new word.
#[test]
fn a_reattach_moves_the_parker_without_a_new_thread() {
    if !alone_on_the_fallback("a_reattach_moves_the_parker_without_a_new_thread") {
        return;
    }
    let f = Feed::new("reaim");
    let (s, st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    // A new thread names itself, a moment after it starts.
    let t0 = Instant::now();
    while parker_threads().is_empty() && ms_since(t0) < 2000 {
        thread::sleep(Duration::from_millis(1));
    }
    let parkers = parker_threads();
    assert_eq!(parkers.len(), 1);
    f.publish(1);
    assert_eq!(spin_once(&mut ex, 5_000_000_000), 1);

    f.restart();
    for _ in 0..400 {
        f.publish(2);
        if ex.dispatch().unwrap() > 0 && st.last_id() == 2 {
            break;
        }
    }
    assert_eq!(
        st.last_id(),
        2,
        "delivery stopped across a publisher restart"
    );
    ex.dispatch().unwrap();
    // After the pass sampled the new word, before the wait aims the parker at it.
    f.publish(3);
    let t0 = Instant::now();
    ex.wait_for_work(5_000_000_000).unwrap();
    assert!(
        ms_since(t0) < 1000,
        "the frame before the re-aim was missed"
    );
    assert_eq!((ex.dispatch().unwrap(), st.last_id()), (1, 3));

    let publish = f.publisher();
    let t0 = Instant::now();
    let n = thread::scope(|sc| {
        sc.spawn(|| {
            thread::sleep(Duration::from_millis(100));
            publish(4);
        });
        spin_once(&mut ex, 5_000_000_000)
    });
    assert_eq!((n, st.last_id()), (1, 4));
    assert!(ms_since(t0) < 1000, "the re-aimed parker missed the frame");
    assert_eq!(parker_threads(), parkers);
}

// The fallback a kernel below 5.16 takes cannot be reached here unless forced: this binary again,
// every test, on parker threads.
#[test]
fn every_test_again_on_the_parker_fallback() {
    if forced_fallback() {
        return;
    }
    let out = Command::new(std::env::current_exe().unwrap())
        .env(DISABLE_FUTEX_WAITV_ENV, "1")
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn the_wait_takes_the_path_the_environment_chose() {
    let f = Feed::new("path");
    let (s, _st) = f.source(1);
    let mut ex = executor();
    ex.add(s, 0).unwrap();
    ex.dispatch().unwrap();
    ex.wait_for_work(0).unwrap();
    assert_eq!(!parker_threads().is_empty(), forced_fallback());
}
