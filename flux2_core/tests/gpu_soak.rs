//! Kill soak on a GPU channel: a publisher writes each frame with the device and two subscribers
//! read it back from the device, while the coordinator kills and restarts them. A frame is its
//! ticket's byte over its ticket's length, checked when taken and again when released. A keeper
//! subscriber is never killed; its own device memory must come back to where it was before it
//! opened the channel, and every device endpoint a publisher served must be gone after a last
//! sweep. Returns early without a CUDA device.
//!
//! `FLUX2_GPU_SOAK_SECS` (default 2) sets the length. Set, it also checks that the device memory no
//! live process accounts for is back to the start, which graphics clients move a little.

mod common;

use std::ffi::{CStr, c_void};
use std::io::Read;
use std::os::unix::fs::MetadataExt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use common::{Rng, Worker, reply, violation};
use flux2_core::gpu::{Route, Stream, probe};
use flux2_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, Error, FrameView, MemoryPolicy, NO_SCHEMA, Published, QoS,
    read_channel_stats, signpost_name,
};

const SLOT: u32 = 8 << 20;
const SLOTS: u32 = 4;
const PAGE: u64 = 4096;
const ROLE: &str = "FLUX2_GPU_SOAK_ROLE";
const SIGNPOST: &str = "FLUX2_GPU_SOAK_SIGNPOST";
const SEED: &str = "FLUX2_GPU_SOAK_SEED";

type MemsetAsync = unsafe extern "C" fn(u64, u8, usize, *mut c_void) -> i32;
type CopyDtoH = unsafe extern "C" fn(*mut c_void, u64, usize) -> i32;

struct Cuda {
    memset: MemsetAsync,
    copy: CopyDtoH,
}

fn cuda() -> Cuda {
    // SAFETY: flux2 has loaded the driver already; this takes another reference to it.
    let lib = unsafe { libc::dlopen(c"libcuda.so.1".as_ptr(), libc::RTLD_NOW) };
    assert!(!lib.is_null());
    let sym = |name: &CStr| {
        // SAFETY: a live handle and a C string.
        let p = unsafe { libc::dlsym(lib, name.as_ptr()) };
        assert!(!p.is_null(), "{name:?}");
        p
    };
    // SAFETY: cuMemsetD8Async's C signature.
    let memset =
        unsafe { std::mem::transmute::<*mut c_void, MemsetAsync>(sym(c"cuMemsetD8Async")) };
    // SAFETY: cuMemcpyDtoH_v2's C signature.
    let copy = unsafe { std::mem::transmute::<*mut c_void, CopyDtoH>(sym(c"cuMemcpyDtoH_v2")) };
    Cuda { memset, copy }
}

fn len_of(ticket: u64) -> u64 {
    PAGE * (1 + ticket % (u64::from(SLOT) / PAGE))
}

fn byte_of(ticket: u64) -> u8 {
    (ticket.wrapping_mul(37) + 11).to_le_bytes()[0]
}

fn check(cu: &Cuda, v: &FrameView) -> Result<(), String> {
    let t = v.ticket();
    if v.meta().nbytes != len_of(t) {
        return Err(format!("ticket {t}: {} bytes", v.meta().nbytes));
    }
    let mut back = vec![0u8; usize::try_from(len_of(t)).unwrap()];
    // SAFETY: back holds the frame's bytes; the view keeps the slot.
    let rc = unsafe { (cu.copy)(back.as_mut_ptr().cast(), v.device_ptr(), back.len()) };
    match back.iter().position(|&b| b != byte_of(t)) {
        _ if rc != 0 => Err(format!("ticket {t}: cuMemcpyDtoH {rc}")),
        Some(i) => Err(format!("ticket {t}: byte {i} is {}", back[i])),
        None => Ok(()),
    }
}

fn create(signpost: &str, stream: &Stream) -> Channel {
    let until = Instant::now() + Duration::from_secs(2);
    loop {
        match Channel::create_on(
            signpost,
            SLOT,
            SLOTS,
            DEFAULT_MAX_SUBSCRIBERS,
            NO_SCHEMA,
            MemoryPolicy::default(),
            stream.clone(),
            -1,
        ) {
            Ok(ch) => return ch,
            Err(Error::PublisherExists(_)) if Instant::now() < until => {
                std::thread::sleep(Duration::from_millis(1));
            }
            Err(e) => violation(&format!("create: {e}")),
        }
    }
}

/// The device endpoint of the segment `signpost` advertises, as the publisher names it.
fn endpoint(signpost: &str) -> String {
    let epoch = read_channel_stats(signpost).epoch;
    let m = std::fs::metadata(format!("/dev/shm{signpost}.e{epoch}")).unwrap();
    format!("/dev/shm/flux2.gpu.{}.{}", m.dev(), m.ino())
}

fn publish_role(signpost: &str, rng: &mut Rng, stop: &AtomicBool, cu: &Cuda) {
    let stream = Stream::create().unwrap();
    while !stop.load(Ordering::Relaxed) {
        let mut ch = create(signpost, &stream);
        reply(&format!("ep {}", endpoint(signpost)));
        let mut next = 1;
        while !stop.load(Ordering::Relaxed) && rng.below(500) != 0 {
            let Some(w) = ch.loan() else {
                std::thread::sleep(Duration::from_micros(100));
                continue;
            };
            let n = usize::try_from(len_of(next)).unwrap();
            // SAFETY: the loan's slot holds n bytes; the commit waits for the stream.
            let rc = unsafe { (cu.memset)(w.device_ptr(), byte_of(next), n, stream.native()) };
            if rc != 0 {
                violation(&format!("cuMemsetD8Async {rc}"));
            }
            match w.commit_len(n) {
                Published::Ok => next += 1,
                r => violation(&format!("commit gave {r}")),
            }
        }
    }
}

/// Device memory per compute process in MiB, as the driver accounts it.
fn vram() -> Vec<(u32, u64)> {
    let out = std::process::Command::new("nvidia-smi")
        .args([
            "--query-compute-apps=pid,used_memory",
            "--format=csv,noheader,nounits",
        ])
        .output()
        .unwrap();
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .filter_map(|l| {
            let (p, m) = l.split_once(", ")?;
            Some((p.trim().parse().ok()?, m.trim().parse().ok()?))
        })
        .collect()
}

fn vram_of(pid: u32) -> u64 {
    vram()
        .iter()
        .filter(|&&(p, _)| p == pid)
        .map(|&(_, m)| m)
        .sum()
}

/// Used device memory no live compute process accounts for, in MiB: what a dead process's
/// allocation kept alive by an import would add. Graphics clients move it by a few MiB.
fn unaccounted() -> u64 {
    let out = std::process::Command::new("nvidia-smi")
        .args(["--query-gpu=memory.used", "--format=csv,noheader,nounits"])
        .output()
        .unwrap();
    let used: u64 = String::from_utf8_lossy(&out.stdout).trim().parse().unwrap();
    used.saturating_sub(vram().iter().map(|&(_, m)| m).sum())
}

fn subscribe_role(signpost: &str, rng: &mut Rng, stop: &AtomicBool, cu: &Cuda, keeper: bool) {
    let stream = Stream::create().unwrap();
    let before = vram_of(std::process::id());
    let (mut takes, mut most) = (0u64, before);
    while !stop.load(Ordering::Relaxed) {
        let mut ch = match Channel::open_on(
            signpost,
            NO_SCHEMA,
            MemoryPolicy::default(),
            stream.clone(),
            -1,
        ) {
            Ok(ch) => ch,
            Err(Error::NotReady(_)) => {
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            Err(e) => violation(&format!("open: {e}")),
        };
        let borrow = 1 + rng.below(2);
        let q = QoS::new(1 + rng.below(SLOTS)).unwrap().max_borrow(borrow);
        ch.set_qos(q.unwrap()).unwrap();
        let mut held: Vec<FrameView> = Vec::new();
        let mut next_look = Instant::now();
        while !stop.load(Ordering::Relaxed) && (keeper || rng.below(2000) != 0) {
            if let Some(v) = ch.take_blocking(5_000_000) {
                check(cu, &v).unwrap_or_else(|e| violation(&e));
                held.push(v);
                takes += 1;
            }
            while held.len() >= borrow as usize || (!held.is_empty() && rng.below(2) == 0) {
                let v = held.remove(rng.index(held.len().try_into().unwrap()));
                check(cu, &v).unwrap_or_else(|e| violation(&format!("rewritten while held: {e}")));
            }
            if keeper && Instant::now() >= next_look {
                next_look += Duration::from_millis(500);
                most = most.max(vram_of(std::process::id()));
            }
        }
    }
    let after = vram_of(std::process::id());
    if keeper && after > before {
        violation(&format!(
            "device memory {before} MiB before, {after} MiB after"
        ));
    }
    reply(&format!("{takes} {before} {most} {after}"));
}

/// Runs only as a child of `killed_gpu_peers_leave_frames_whole_and_memory_returned`.
#[test]
fn gpu_soak_worker() {
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
    if role == "sweep" {
        drop(Channel::open(&signpost, NO_SCHEMA, MemoryPolicy::default()));
        return;
    }
    let cu = cuda();
    match role.as_str() {
        "pub" => publish_role(&signpost, &mut rng, &stop, &cu),
        r => subscribe_role(&signpost, &mut rng, &stop, &cu, r == "keeper"),
    }
}

struct Soak {
    signpost: String,
    rng: Rng,
    endpoints: Vec<String>,
    violations: Vec<String>,
}

impl Soak {
    fn spawn(&mut self, role: &str) -> Worker {
        let seed = self.rng.below(u32::MAX).to_string();
        Worker::spawn(
            "gpu_soak_worker",
            &[(ROLE, role), (SIGNPOST, &self.signpost), (SEED, &seed)],
        )
    }

    fn collect(&mut self, w: &Worker) {
        let eps = w
            .drain()
            .into_iter()
            .filter_map(|l| Some(l.strip_prefix("ep ")?.to_owned()));
        self.endpoints.extend(eps);
    }
}

#[test]
fn killed_gpu_peers_leave_frames_whole_and_memory_returned() {
    if probe(0).route == Route::None {
        return;
    }
    let long = std::env::var("FLUX2_GPU_SOAK_SECS").ok();
    let secs: u64 = long.as_ref().map_or(2, |s| s.parse().unwrap());
    let mut soak = Soak {
        signpost: signpost_name(&format!("/gs{}", std::process::id()), NO_SCHEMA).unwrap(),
        rng: Rng(u64::from(std::process::id()) | 1 << 40),
        endpoints: Vec::new(),
        violations: Vec::new(),
    };
    let used_before = unaccounted();
    let mut workers: Vec<(&str, Worker)> = Vec::new();
    for role in ["pub", "sub", "sub", "keeper"] {
        let w = soak.spawn(role);
        workers.push((role, w));
    }
    let mut kills = 0;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(secs) {
        std::thread::sleep(Duration::from_millis(20));
        for (role, w) in &mut workers {
            soak.collect(w);
            if w.child.try_wait().unwrap().is_some() {
                let why = w.death();
                soak.violations.push(format!("{role}: {why}"));
                *w = soak.spawn(role);
            }
        }
        if soak.rng.below(5) == 0 {
            let i = soak.rng.index(3);
            workers[i].1.signal(libc::SIGKILL);
            let _ = workers[i].1.child.wait();
            soak.collect(&workers[i].1);
            workers[i].1 = soak.spawn(workers[i].0);
            kills += 1;
        }
    }
    let mut keeper = String::new();
    for (role, mut w) in workers {
        w.close();
        if role != "pub" {
            match w.reply() {
                Ok(r) if role == "keeper" => keeper = r,
                Ok(_) => {}
                Err(why) => soak.violations.push(format!("{role}: {why}")),
            }
        }
        soak.collect(&w);
        if let Err(why) = w.finish() {
            soak.violations.push(format!("{role}: {why}"));
        }
    }
    soak.spawn("sweep").finish().unwrap();
    let left: Vec<&String> = soak
        .endpoints
        .iter()
        .filter(|e| std::fs::symlink_metadata(e).is_ok())
        .collect();
    let until = Instant::now() + Duration::from_secs(5);
    let mut used_after = unaccounted();
    while used_after > used_before + 32 && Instant::now() < until {
        std::thread::sleep(Duration::from_millis(100));
        used_after = unaccounted();
    }
    eprintln!(
        "{secs} s: {kills} kills, {} endpoints; keeper takes, MiB before, most, after: {keeper}; \
         unaccounted {used_before} MiB before, {used_after} MiB after",
        soak.endpoints.len()
    );
    assert!(soak.violations.is_empty(), "{:?}", soak.violations);
    assert!(left.is_empty(), "endpoints left behind: {left:?}");
    if long.is_some() {
        assert!(
            used_after <= used_before + 32,
            "unaccounted device memory {used_before} -> {used_after} MiB"
        );
    }
}
