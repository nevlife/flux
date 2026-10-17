//! Free-running torture with kills (locktorture style). A publisher and two subscriber processes
//! loop as fast as they can while the coordinator kills and restarts them, stops one for a while,
//! stops them all at once now and then (stutter) and moves them between CPUs (shuffle). Every frame
//! checks its own ticket, shape and hash when taken and again when released, so a slot rewritten
//! while held fails in the subscriber that holds it. A keeper subscriber is never killed: at the
//! end its descriptors and flux2 mappings must be what they were before it opened the channel, and
//! after a last sweep nothing of the channel may be left in /dev/shm.
//!
//! `FLUX2_TORTURE_SECS` (default 2) sets the length, `FLUX2_TORTURE_STUTTER_MS` (500) and
//! `FLUX2_TORTURE_SHUFFLE_MS` (1000) the two periods.

mod common;

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::io::Read;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use common::{Rng, Worker, check, frame, leftovers, reply, shape_of, violation};
use flux2_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, Error, FrameView, MemoryPolicy, NO_SCHEMA, Published, QoS,
    fuzz, signpost_name,
};

const SLOT: u32 = 8192;
const SLOTS: u32 = 4;
const ROLE: &str = "FLUX2_TORTURE_ROLE";
const SIGNPOST: &str = "FLUX2_TORTURE_SIGNPOST";
const SEED: &str = "FLUX2_TORTURE_SEED";

fn env_ms(name: &str, default: u64) -> Duration {
    Duration::from_millis(std::env::var(name).map_or(default, |v| v.parse().unwrap()))
}

fn create(signpost: &str) -> Channel {
    let until = Instant::now() + Duration::from_secs(2);
    loop {
        match Channel::create(
            signpost,
            SLOT,
            SLOTS,
            DEFAULT_MAX_SUBSCRIBERS,
            NO_SCHEMA,
            MemoryPolicy::default(),
        ) {
            Ok(ch) => return ch,
            Err(Error::PublisherExists(_)) if Instant::now() < until => {
                std::thread::sleep(Duration::from_millis(1));
            }
            Err(e) => violation(&format!("create: {e}")),
        }
    }
}

/// The next frame's ticket follows the publisher's successes.
fn advance(next: &mut u64, r: Published) {
    match r {
        Published::Ok => *next += 1,
        Published::Backpressure => {}
        r => violation(&format!("publish gave {r}")),
    }
}

fn publish_role(signpost: &str, rng: &mut Rng, stop: &AtomicBool) {
    let mut restarts = 0u64;
    while !stop.load(Ordering::Relaxed) {
        let mut ch = create(signpost);
        let nonce = u64::from(std::process::id()) << 20 | restarts;
        let mut next = 1;
        while !stop.load(Ordering::Relaxed) && rng.below(5000) != 0 {
            let code = rng.below(1 << 16);
            let (dt, shape) = shape_of(code, SLOT);
            let f = frame(next, nonce, dt, &shape);
            match rng.below(100) {
                0..65 => advance(&mut next, ch.publish_typed(&f, dt, &shape)),
                65..92 => {
                    let w = if code % 2 == 1 {
                        ch.loan()
                    } else {
                        ch.loan_typed(dt, &shape)
                    };
                    let Some(mut w) = w else { continue };
                    w.data_mut().fill(0xA5);
                    match rng.below(10) {
                        0 => drop(w),
                        1 => {
                            let past = w.capacity() + 1;
                            assert_eq!(w.commit_len(past), Published::TooLarge);
                        }
                        _ => {
                            w.data_mut()[..f.len()].copy_from_slice(&f);
                            let r = if code % 2 == 1 {
                                w.commit_len(f.len())
                            } else {
                                w.commit()
                            };
                            advance(&mut next, r);
                        }
                    }
                }
                _ => std::thread::sleep(Duration::from_micros(u64::from(rng.below(200)))),
            }
        }
        restarts += 1;
    }
}

/// Descriptors and the flux2 objects mapped, for the keeper's before and after.
fn resources() -> (usize, BTreeSet<String>) {
    let fds = std::fs::read_dir("/proc/self/fd").unwrap().count();
    let mut maps = String::new();
    std::fs::File::open("/proc/self/maps")
        .unwrap()
        .read_to_string(&mut maps)
        .unwrap();
    let names = maps
        .lines()
        .filter_map(|l| l.split_once("/dev/shm/"))
        .map(|(_, n)| n.trim_end_matches(" (deleted)").to_owned())
        .collect();
    (fds, names)
}

#[derive(Default)]
struct SubStats {
    takes: u64,
    releases: u64,
    /// Releases by frames committed after the view's, capped at twice the slot count. At the slot
    /// count or more, the borrow is what kept the slot from being rewritten.
    lag: BTreeMap<u64, u64>,
}

fn open(signpost: &str, stop: &AtomicBool) -> Option<Channel> {
    while !stop.load(Ordering::Relaxed) {
        match Channel::open(signpost, NO_SCHEMA, MemoryPolicy::default()) {
            Ok(ch) => return Some(ch),
            Err(Error::NotReady(_)) => std::thread::sleep(Duration::from_micros(200)),
            Err(e) => violation(&format!("open: {e}")),
        }
    }
    None
}

fn release(v: &FrameView, st: &mut SubStats) {
    if let Err(e) = check(v) {
        violation(&format!("rewritten while held: {e}"));
    }
    *st.lag
        .entry(fuzz::lag(v).min(2 * u64::from(SLOTS)))
        .or_default() += 1;
    st.releases += 1;
}

/// One attach until a reopen or the end. The keeper never reopens.
fn attach(ch: &mut Channel, rng: &mut Rng, stop: &AtomicBool, keeper: bool, st: &mut SubStats) {
    let borrow = 1 + rng.below(3);
    let q = QoS::new(1 + rng.below(4 * SLOTS + 1))
        .unwrap()
        .max_borrow(borrow);
    ch.set_qos(q.unwrap()).unwrap();
    let mut held = VecDeque::new();
    let (mut last, mut lost) = ((u32::MAX, 0), 0);
    let mut nonces = BTreeMap::new();
    let mut next_look = Instant::now();
    while !stop.load(Ordering::Relaxed) && (keeper || rng.below(20_000) != 0) {
        let got = if rng.below(4) == 0 {
            ch.take_blocking(5_000_000)
        } else {
            ch.take()
        };
        let r = ch.refused();
        if r.no_reader + r.contended + r.bad_frame + r.fence != 0 || ch.lost() < lost {
            violation(&format!("refused {r:?}, lost {} after {lost}", ch.lost()));
        }
        lost = ch.lost();
        if let Some(v) = got {
            let nonce = check(&v).unwrap_or_else(|e| violation(&e));
            let at = (ch.attach_generation(), v.ticket());
            if at.0 == last.0 && at.1 <= last.1 {
                violation(&format!("ticket {} after {}", at.1, last.1));
            }
            if *nonces.entry(at.0).or_insert(nonce) != nonce {
                violation("two publishers' frames in one attach");
            }
            last = at;
            held.push_back(v);
            st.takes += 1;
        }
        while held.len() >= borrow as usize || (!held.is_empty() && rng.below(3) == 0) {
            let v = held
                .remove(rng.index(held.len().try_into().unwrap()))
                .unwrap();
            release(&v, st);
            if rng.below(2) == 0 {
                break;
            }
        }
        // A view keeps its segment mapped, nothing else does: the current one and the signpost.
        if keeper && Instant::now() >= next_look {
            next_look += Duration::from_millis(100);
            let mapped = resources().1.len();
            if mapped > 2 + held.len() {
                violation(&format!(
                    "{mapped} flux2 objects mapped, {} views",
                    held.len()
                ));
            }
        }
    }
    for v in held {
        release(&v, st);
    }
}

fn subscribe_role(signpost: &str, rng: &mut Rng, stop: &AtomicBool, keeper: bool) {
    let before = resources();
    let mut st = SubStats::default();
    while let Some(mut ch) = open(signpost, stop) {
        attach(&mut ch, rng, stop, keeper, &mut st);
    }
    if keeper && resources() != before {
        violation(&format!("held {:?} after, {before:?} before", resources()));
    }
    let lag: Vec<String> = st.lag.iter().map(|(l, n)| format!("{l}:{n}")).collect();
    reply(&format!("{} {} {}", st.takes, st.releases, lag.join(",")));
}

/// Runs only as a child of `kills_stops_and_shuffles_leave_frames_whole_and_nothing_behind`.
#[test]
fn torture_worker() {
    let (Ok(role), Ok(signpost), Ok(seed)) = (
        std::env::var(ROLE),
        std::env::var(SIGNPOST),
        std::env::var(SEED),
    ) else {
        return;
    };
    let stop = Arc::new(AtomicBool::new(false));
    let eof = Arc::clone(&stop);
    std::thread::spawn(move || {
        let _ = std::io::stdin().read_to_end(&mut Vec::new());
        eof.store(true, Ordering::Relaxed);
    });
    let mut rng = Rng(seed.parse().unwrap());
    match role.as_str() {
        "pub" => publish_role(&signpost, &mut rng, &stop),
        "sub" => subscribe_role(&signpost, &mut rng, &stop, false),
        "keeper" => subscribe_role(&signpost, &mut rng, &stop, true),
        _ => drop(Channel::open(&signpost, NO_SCHEMA, MemoryPolicy::default())),
    }
}

fn allowed_cpus() -> Vec<usize> {
    // SAFETY: cpu_set_t is plain data; all zeros is an empty set.
    let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    // SAFETY: set is a valid cpu_set_t of the size passed.
    unsafe { libc::sched_getaffinity(0, size_of::<libc::cpu_set_t>(), &raw mut set) };
    let mut cpus = Vec::new();
    for c in 0..libc::CPU_SETSIZE as usize {
        // SAFETY: an index below the set's capacity.
        if unsafe { libc::CPU_ISSET(c, &set) } {
            cpus.push(c);
        }
    }
    cpus
}

/// A random non-empty subset of `cpus`.
fn shuffle(pid: libc::pid_t, cpus: &[usize], rng: &mut Rng) {
    // SAFETY: as in allowed_cpus.
    let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    let first = rng.index(cpus.len().try_into().unwrap());
    for (i, &c) in cpus.iter().enumerate() {
        if i == first || rng.below(2) == 0 {
            // SAFETY: c is an index below the set's capacity.
            unsafe { libc::CPU_SET(c, &mut set) };
        }
    }
    // SAFETY: a valid set for our own child; a child that just died fails it harmlessly.
    unsafe { libc::sched_setaffinity(pid, size_of::<libc::cpu_set_t>(), &raw const set) };
}

struct Pool {
    signpost: String,
    rng: Rng,
    workers: Vec<(&'static str, Worker)>,
    violations: Vec<String>,
    kills: u64,
}

impl Pool {
    fn spawn(&mut self, role: &'static str) -> Worker {
        let seed = self.rng.below(u32::MAX).to_string();
        Worker::spawn(
            "torture_worker",
            &[(ROLE, role), (SIGNPOST, &self.signpost), (SEED, &seed)],
        )
    }

    /// Respawns any worker that ended by itself, which only a violation does.
    fn reap(&mut self) {
        for i in 0..self.workers.len() {
            if self.workers[i].1.child.try_wait().unwrap().is_some() {
                let why = self.workers[i].1.death();
                self.violations
                    .push(format!("{}: {why}", self.workers[i].0));
                self.workers[i].1 = self.spawn(self.workers[i].0);
            }
        }
    }

    fn kill(&mut self, i: usize) {
        let w = &mut self.workers[i].1;
        w.signal(libc::SIGKILL);
        let _ = w.child.wait();
        self.workers[i].1 = self.spawn(self.workers[i].0);
        self.kills += 1;
    }
}

#[test]
fn kills_stops_and_shuffles_leave_frames_whole_and_nothing_behind() {
    let secs = std::env::var("FLUX2_TORTURE_SECS").map_or(2, |s| s.parse().unwrap());
    let stutter = env_ms("FLUX2_TORTURE_STUTTER_MS", 500);
    let every = env_ms("FLUX2_TORTURE_SHUFFLE_MS", 1000);
    let key = format!("/tt{}", std::process::id());
    let mut pool = Pool {
        signpost: signpost_name(&key, NO_SCHEMA).unwrap(),
        rng: Rng(u64::from(std::process::id()) | 1 << 40),
        workers: Vec::new(),
        violations: Vec::new(),
        kills: 0,
    };
    for role in ["pub", "sub", "sub", "keeper"] {
        let w = pool.spawn(role);
        pool.workers.push((role, w));
    }
    let cpus = allowed_cpus();
    let (mut stops, mut stutters, mut shuffles) = (0, 0, 0);
    let mut stopped: Vec<(usize, Instant)> = Vec::new();
    let start = Instant::now();
    let (mut next_stutter, mut next_shuffle) = (start + stutter, start + every);
    while start.elapsed() < Duration::from_secs(secs) {
        pool.reap();
        stopped.retain(|&(i, at)| {
            if Instant::now() < at {
                return true;
            }
            pool.workers[i].1.signal(libc::SIGCONT);
            false
        });
        let i = pool.rng.index(3);
        if pool.rng.below(100) == 0 {
            stopped.retain(|&(j, _)| j != i);
            pool.kill(i);
        } else if pool.rng.below(30) == 0 && stopped.iter().all(|&(j, _)| j != i) {
            pool.workers[i].1.signal(libc::SIGSTOP);
            let ms = 1 + u64::from(pool.rng.below(20));
            stopped.push((i, Instant::now() + Duration::from_millis(ms)));
            stops += 1;
        }
        if Instant::now() >= next_stutter {
            next_stutter += stutter;
            stopped.clear();
            for (_, w) in &mut pool.workers {
                w.signal(libc::SIGSTOP);
            }
            std::thread::sleep(Duration::from_millis(10));
            for (_, w) in &mut pool.workers {
                w.signal(libc::SIGCONT);
            }
            stutters += 1;
        }
        if Instant::now() >= next_shuffle {
            next_shuffle += every;
            for (_, w) in &pool.workers {
                shuffle(w.pid(), &cpus, &mut pool.rng);
            }
            shuffles += 1;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    let (mut takes, mut releases, mut lag) = (0u64, 0u64, BTreeMap::<u64, u64>::new());
    for (role, mut w) in std::mem::take(&mut pool.workers) {
        if w.stopped {
            w.signal(libc::SIGCONT);
        }
        w.close();
        if role != "pub" {
            match w.reply() {
                Ok(r) => {
                    let mut t = r.split(' ');
                    takes += t.next().unwrap().parse::<u64>().unwrap();
                    releases += t.next().unwrap().parse::<u64>().unwrap();
                    for (l, n) in t
                        .next()
                        .unwrap_or("")
                        .split(',')
                        .filter_map(|e| e.split_once(':'))
                    {
                        *lag.entry(l.parse().unwrap()).or_default() += n.parse::<u64>().unwrap();
                    }
                }
                Err(why) => pool.violations.push(format!("{role}: {why}")),
            }
        }
        if let Err(why) = w.finish() {
            pool.violations.push(format!("{role}: {why}"));
        }
    }
    let sweep = pool.spawn("sweep");
    sweep.finish().unwrap();
    let left = leftovers(&pool.signpost);
    eprintln!(
        "{secs} s: {} kills, {stops} stops, {stutters} stutters, {shuffles} shuffles; {takes} \
         takes, {releases} releases, lag at release {lag:?}",
        pool.kills
    );
    assert!(pool.violations.is_empty(), "{:?}", pool.violations);
    assert!(left.is_empty(), "left behind: {left:?}");
    assert!(takes > 0);
}
