//! Many short-lived processes publish and subscribe on a few keys while others are killed, or
//! stopped for a while, at random points. Each process sweeps when it starts, with no lock that takes turns. No live object's name
//! may be removed or rebound, and nothing may be left once a last sweep has run.
//!
//! `FLUX_STRESS_SECS` (default 2) sets the length. Run it in a private /dev/shm to keep other
//! processes' sweeps out: `bwrap --dev-bind / / --tmpfs /dev/shm -- cargo test --test sweep_stress`.

use std::io::Read;
use std::os::unix::fs::MetadataExt;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use flux_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, Error, MemoryPolicy, NO_SCHEMA, read_channel_stats,
    signpost_name,
};

const ROLE: &str = "FLUX_STRESS_ROLE";
const KEY: &str = "FLUX_STRESS_KEY";
const LIFE_MS: &str = "FLUX_STRESS_LIFE_MS";
const KEYS: usize = 3;
const WORKERS: usize = 12;
const VIOLATION: i32 = 3;

fn ino(name: &str) -> Option<u64> {
    std::fs::metadata(format!("/dev/shm{name}"))
        .ok()
        .map(|m| m.ino())
}

fn violation(what: &str) -> ! {
    eprintln!("VIOLATION {what}");
    std::process::exit(VIOLATION);
}

/// Holds a publisher for its life, publishing, and checks its two names still bind its objects.
fn publish(signpost: &str, until: Instant) {
    let mut p = loop {
        match Channel::create(
            signpost,
            64,
            4,
            DEFAULT_MAX_SUBSCRIBERS,
            0,
            NO_SCHEMA,
            MemoryPolicy::default(),
        ) {
            Ok(p) => break p,
            Err(Error::PublisherExists(_) | Error::NotReady(_)) if Instant::now() < until => {
                std::thread::sleep(Duration::from_micros(200));
            }
            Err(Error::PublisherExists(_) | Error::NotReady(_)) => return,
            Err(e) => violation(&format!("create: {e}")),
        }
    };
    let segment = format!("{signpost}.e{}", read_channel_stats(signpost).epoch);
    let mine = (ino(signpost), ino(&segment));
    if mine.0.is_none() || mine.1.is_none() {
        violation(&format!("{segment}: a fresh publisher's names are gone"));
    }
    while Instant::now() < until {
        let _ = p.publish(b"frame");
        if (ino(signpost), ino(&segment)) != mine {
            violation(&format!(
                "{segment}: a live publisher's name was removed or rebound"
            ));
        }
        std::thread::sleep(Duration::from_micros(100));
    }
}

/// Attaches when it can, takes, and checks the signpost it holds keeps its name.
fn subscribe(signpost: &str, until: Instant) {
    while Instant::now() < until {
        let mut s = match Channel::open(signpost, NO_SCHEMA, MemoryPolicy::default()) {
            Ok(s) => s,
            Err(Error::NotReady(_)) => {
                std::thread::sleep(Duration::from_micros(200));
                continue;
            }
            Err(e) => violation(&format!("open: {e}")),
        };
        let mine = ino(signpost);
        if mine.is_none() {
            violation(&format!(
                "{signpost}: an attached subscriber's signpost is gone"
            ));
        }
        while Instant::now() < until && !s.orphaned() {
            drop(s.take());
            if ino(signpost) != mine {
                violation(&format!(
                    "{signpost}: a held signpost was removed or rebound"
                ));
            }
            std::thread::sleep(Duration::from_micros(100));
        }
    }
}

/// Runs only as a child of `concurrent_sweeps_never_remove_a_live_name`.
#[test]
fn stress_worker() {
    let (Ok(role), Ok(key), Ok(life)) = (
        std::env::var(ROLE),
        std::env::var(KEY),
        std::env::var(LIFE_MS),
    ) else {
        return;
    };
    let signpost = signpost_name(&key, NO_SCHEMA).unwrap();
    let until = Instant::now() + Duration::from_millis(life.parse().unwrap());
    match role.as_str() {
        "pub" => publish(&signpost, until),
        "sub" => subscribe(&signpost, until),
        // A process whose only job is the sweep every process runs on its first subscription.
        _ => drop(Channel::open(&signpost, NO_SCHEMA, MemoryPolicy::default())),
    }
}

fn signal(pid: u32, sig: libc::c_int) {
    // SAFETY: plain syscall on our own child.
    unsafe { libc::kill(libc::pid_t::try_from(pid).unwrap(), sig) };
}

struct Rng(u64);

impl Rng {
    fn below(&mut self, n: usize) -> usize {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        usize::try_from(self.0 % u64::try_from(n).unwrap()).unwrap()
    }

    fn ms(&mut self, from: u64, span: usize) -> u64 {
        from + u64::try_from(self.below(span)).unwrap()
    }
}

fn spawn(role: &str, key: &str, life_ms: u64) -> Child {
    Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "stress_worker", "--test-threads=1"])
        .env(ROLE, role)
        .env(KEY, key)
        .env(LIFE_MS, life_ms.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap()
}

/// What a finished child reported, if it found a violation.
fn reap(mut child: Child) -> Option<String> {
    let status = child.wait().unwrap();
    let mut err = String::new();
    let _ = child.stderr.take().unwrap().read_to_string(&mut err);
    (status.code() == Some(VIOLATION)).then_some(err)
}

#[test]
fn concurrent_sweeps_never_remove_a_live_name() {
    let secs = std::env::var("FLUX_STRESS_SECS").map_or(2, |s| s.parse().unwrap());
    let prefix = format!("st{}_", std::process::id());
    let keys: Vec<String> = (0..KEYS).map(|k| format!("/{prefix}k{k}")).collect();
    let mut rng = Rng(u64::from(std::process::id()) | 1 << 40);
    let mut running: Vec<Child> = Vec::new();
    let mut violations = Vec::new();
    let (mut spawned, mut killed, mut paused) = (0, 0, 0);
    // Stopped children and when to continue them: a stop stretches whatever window the child is in.
    let mut stopped: Vec<(u32, Instant)> = Vec::new();
    let end = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < end {
        let mut i = 0;
        while i < running.len() {
            if running[i].try_wait().unwrap().is_some() {
                violations.extend(reap(running.swap_remove(i)));
            } else {
                i += 1;
            }
        }
        while running.len() < WORKERS {
            let role = ["pub", "sub", "sub", "sweep"][rng.below(4)];
            let key = &keys[rng.below(KEYS)];
            running.push(spawn(role, key, rng.ms(10, 150)));
            spawned += 1;
        }
        stopped.retain(|&(pid, at)| {
            if Instant::now() < at {
                return true;
            }
            signal(pid, libc::SIGCONT);
            false
        });
        if rng.below(10) == 0 {
            let i = rng.below(running.len());
            let mut victim = running.swap_remove(i);
            let _ = victim.kill();
            let _ = victim.wait();
            stopped.retain(|&(pid, _)| pid != victim.id());
            killed += 1;
        }
        if rng.below(3) == 0 {
            let pid = running[rng.below(running.len())].id();
            if stopped.iter().all(|&(p, _)| p != pid) {
                signal(pid, libc::SIGSTOP);
                stopped.push((pid, Instant::now() + Duration::from_millis(rng.ms(1, 20))));
                paused += 1;
            }
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    for (pid, _) in stopped {
        signal(pid, libc::SIGCONT);
    }
    for child in running {
        violations.extend(reap(child));
    }
    // A last process sweeps what the killed ones left.
    assert!(reap(spawn("sweep", &keys[0], 0)).is_none());
    let left: Vec<String> = std::fs::read_dir("/dev/shm")
        .unwrap()
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .filter(|n| n.contains(&prefix))
        .collect();
    eprintln!("{spawned} processes, {killed} killed, {paused} stopped for a while");
    assert!(violations.is_empty(), "{violations:?}");
    assert!(left.is_empty(), "left behind: {left:?}");
}
