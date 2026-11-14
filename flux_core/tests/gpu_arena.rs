//! The GPU arena across processes: a publisher grows it chunk by chunk and a child of this test
//! binary imports the chunks and reads each run back, on the device and, for a `HOST_NUMA` arena,
//! from the CPU. A binary of its own: the child makes a CUDA context of its own, which in the
//! library's test binary would run beside the tests that time GPU work or read free memory.
//! Returns early without a CUDA device with VMM.

use std::ffi::{CStr, c_void};

use flux_core::arena::{BLOCK_BYTES, GPU_POS, HEAP_POS, TRAILER_BYTES};
use flux_core::gpu::arena::{CHUNK, GpuArena, GpuArenaImport, Place};
use flux_core::gpu::{Route, Stream, probe};
use flux_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, MemoryPolicy, Published, PublisherWatch, QoS, signpost_name,
};

const CHILD: &str = "FLUX_GPU_ARENA_CHILD";

type Memset = unsafe extern "C" fn(u64, u8, usize) -> i32;
type CopyDtoH = unsafe extern "C" fn(*mut c_void, u64, usize) -> i32;
type Attribute = unsafe extern "C" fn(*mut i32, i32, i32) -> i32;
const HOST_NUMA_VMM_SUPPORTED: i32 = 141;

struct Cuda {
    memset: Memset,
    copy: CopyDtoH,
    attribute: Attribute,
    /// Keeps a context current on this thread for the copies.
    _stream: Stream,
}

/// None without a device that has VMM and POSIX fd handles.
fn cuda() -> Option<Cuda> {
    let p = probe(0);
    if !(p.vmm && p.posix_fd_handle) {
        eprintln!("skipped: {p:?}");
        return None;
    }
    let stream = Stream::create().ok()?;
    // SAFETY: flux has loaded the driver already; this takes another reference to it.
    let lib = unsafe { libc::dlopen(c"libcuda.so.1".as_ptr(), libc::RTLD_NOW) };
    assert!(!lib.is_null());
    let sym = |name: &CStr| {
        // SAFETY: a live handle and a C string.
        let p = unsafe { libc::dlsym(lib, name.as_ptr()) };
        assert!(!p.is_null(), "{name:?}");
        p
    };
    // SAFETY: cuMemsetD8_v2's C signature.
    let memset = unsafe { std::mem::transmute::<*mut c_void, Memset>(sym(c"cuMemsetD8_v2")) };
    // SAFETY: cuMemcpyDtoH_v2's C signature.
    let copy = unsafe { std::mem::transmute::<*mut c_void, CopyDtoH>(sym(c"cuMemcpyDtoH_v2")) };
    // SAFETY: cuDeviceGetAttribute's C signature.
    let attribute =
        unsafe { std::mem::transmute::<*mut c_void, Attribute>(sym(c"cuDeviceGetAttribute")) };
    Some(Cuda {
        memset,
        copy,
        attribute,
        _stream: stream,
    })
}

/// Runs `test` alone in a child of this binary with `vars`; panics with its output on failure.
fn in_child(test: &str, vars: &[(&str, String)]) {
    let out = std::process::Command::new(std::env::current_exe().unwrap())
        .args([test, "--exact", "--nocapture"])
        .env(CHILD, "1")
        .envs(vars.iter().map(|(k, v)| (*k, v.as_str())))
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&out.stdout);
    assert!(
        out.status.success() && log.contains("1 passed"),
        "{log}{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

fn var(name: &str) -> u64 {
    std::env::var(name).unwrap().parse().unwrap()
}

/// The child's half: imports what the parent named and checks each run's byte.
fn check_runs(place: Place) {
    let cu = cuda().unwrap();
    let endpoint = std::env::var("FLUX_GPU_ARENA_ENDPOINT").unwrap();
    let size = usize::try_from(var("FLUX_GPU_ARENA_SIZE")).unwrap();
    let m = GpuArenaImport::open(&endpoint, size, place, 0).unwrap();
    // A frame first reaches one chunk, then all: the second fetch asks from the first's end.
    m.reach(1).unwrap();
    m.reach(u32::try_from(var("FLUX_GPU_ARENA_CHUNKS")).unwrap())
        .unwrap();
    for (i, run) in std::env::var("FLUX_GPU_ARENA_RUNS")
        .unwrap()
        .split(',')
        .enumerate()
    {
        let (off, len) = run.split_once(':').unwrap();
        let (off, len): (u64, usize) = (off.parse().unwrap(), len.parse().unwrap());
        let want = 0x30 + u8::try_from(i).unwrap();
        let mut got = vec![0u8; len];
        // SAFETY: got holds len bytes; off..off+len is imported.
        let rc = unsafe { (cu.copy)(got.as_mut_ptr().cast(), m.device_ptr(off), len) };
        assert_eq!(rc, 0);
        assert!(got.iter().all(|&b| b == want), "run {i} on the device");
        if matches!(place, Place::HostNuma(_)) {
            let p = m.host(off, len).expect("a HOST_NUMA arena has a CPU view");
            // SAFETY: host() checked that the range is mapped.
            let cpu = unsafe { std::slice::from_raw_parts(p, len) };
            assert!(cpu.iter().all(|&b| b == want), "run {i} from the CPU");
        }
    }
}

/// The parent's half: three runs, one across two chunk boundaries, then a child reads them.
fn publish_runs(cu: &Cuda, test: &str, place: Place) {
    let a = GpuArena::create(0, 256 << 20, Some(place)).unwrap();
    let mut runs = Vec::new();
    for (i, len) in [4096usize, 2 * CHUNK + 1000, 1 << 20]
        .into_iter()
        .enumerate()
    {
        let off = a.alloc(len, 64).unwrap();
        let fill = 0x30 + u8::try_from(i).unwrap();
        if let Some(p) = a.host_ptr(off) {
            // SAFETY: a HOST_NUMA block of len bytes this process may write.
            unsafe { std::ptr::write_bytes(p, fill, len) };
        } else {
            // SAFETY: off..off+len is device memory the arena holds.
            assert_eq!(unsafe { (cu.memset)(a.device_ptr(off), fill, len) }, 0);
        }
        runs.push(format!("{off}:{len}"));
    }
    assert!(a.chunks() >= 3);
    in_child(
        test,
        &[
            ("FLUX_GPU_ARENA_ENDPOINT", a.endpoint().to_owned()),
            ("FLUX_GPU_ARENA_SIZE", a.size().to_string()),
            ("FLUX_GPU_ARENA_CHUNKS", a.chunks().to_string()),
            ("FLUX_GPU_ARENA_RUNS", runs.join(",")),
        ],
    );
}

#[test]
fn device_chunks_reach_another_process_as_they_grow() {
    if std::env::var_os(CHILD).is_some() {
        return check_runs(Place::Device);
    }
    let Some(cu) = cuda() else { return };
    publish_runs(
        &cu,
        "device_chunks_reach_another_process_as_they_grow",
        Place::Device,
    );
}

/// x86 drivers support `HOST_NUMA` too, so this path runs here as well as on an integrated GPU,
/// where the child takes the descriptor-mapping branch instead.
#[test]
fn host_numa_chunks_reach_another_processs_cpu_and_gpu() {
    if std::env::var_os(CHILD).is_some() {
        return check_runs(Place::HostNuma(0));
    }
    let Some(cu) = cuda() else { return };
    let mut supported = 0;
    // SAFETY: a valid out-pointer; device 0 exists.
    let rc = unsafe { (cu.attribute)(&raw mut supported, HOST_NUMA_VMM_SUPPORTED, 0) };
    if rc != 0 || supported == 0 {
        eprintln!("skipped: no HOST_NUMA VMM");
        return;
    }
    publish_runs(
        &cu,
        "host_numa_chunks_reach_another_processs_cpu_and_gpu",
        Place::HostNuma(0),
    );
}

const FINGERPRINT: u64 = 0x5eed;

/// Frames of the channel test: a descriptor-like word, `GPU_POS` + the run's arena offset, then
/// the run's length, then its fill byte.
const RUNS: [(usize, u8); 2] = [(3 << 20, 0x51), (40 << 20, 0x52)];

/// The child's half: a subscriber on a stream of its own reads each frame's run on the device, and
/// a CPU subscriber reads its copy.
fn subscribe_runs(cu: &Cuda) {
    let name = std::env::var("FLUX_GPU_ARENA_CHANNEL").unwrap();
    let watch = PublisherWatch::open(&name).unwrap();
    let qos = QoS::new(4).unwrap().transient_local(4);
    let mut ch = Channel::open_watched_on(
        &watch,
        FINGERPRINT,
        qos,
        MemoryPolicy::default(),
        Stream::create().unwrap(),
        -1,
    )
    .unwrap();
    let mut cpu = Channel::open_watched(&watch, FINGERPRINT, qos, MemoryPolicy::default()).unwrap();
    for (len, fill) in RUNS {
        let c = cpu.take().expect("the CPU subscriber's frame");
        let g = c
            .gpu_heap()
            .expect("a CPU subscriber's copy of the frame's blocks");
        assert_eq!((g.device, g.count), (0, 1));
        // SAFETY: count blocks at blocks, valid while c lives.
        let b = unsafe { *g.blocks };
        let off = u64::from_le_bytes(c.data()[..8].try_into().unwrap());
        assert_eq!((b.off, b.len), (off, len as u64));
        // SAFETY: the copy holds len bytes.
        let copy = unsafe { std::slice::from_raw_parts(b.host, len) };
        assert!(
            copy.iter().all(|&x| x == fill),
            "the CPU copy of {len} bytes"
        );
        let v = ch
            .take()
            .expect("the publisher kept its frames for late subscribers");
        let word = |i: usize| u64::from_le_bytes(v.data()[i * 8..i * 8 + 8].try_into().unwrap());
        assert_eq!(word(1), len as u64);
        let g = v
            .gpu_heap()
            .expect("the trailer names a GPU arena this process imports");
        assert_ne!(g.device, 0);
        let off = word(0) - GPU_POS;
        assert!(off + len as u64 <= g.size as u64);
        let mut got = vec![0u8; len];
        // SAFETY: got holds len bytes; the run is inside what was imported.
        let rc = unsafe { (cu.copy)(got.as_mut_ptr().cast(), g.device + off, len) };
        assert_eq!(rc, 0);
        assert!(got.iter().all(|&b| b == fill), "the run of {len} bytes");
    }
}

/// A host-slot channel that declared a stream: each frame's run is in the GPU arena, and the second
/// reaches chunks the subscriber had not imported for the first.
#[test]
fn a_frame_names_its_device_run_to_a_subscriber_in_another_process() {
    let test = "a_frame_names_its_device_run_to_a_subscriber_in_another_process";
    let Some(cu) = cuda() else { return };
    if std::env::var_os(CHILD).is_some() {
        return subscribe_runs(&cu);
    }
    let name = signpost_name(&format!("/g{}_runs", std::process::id()), FINGERPRINT).unwrap();
    let stream = Stream::create().unwrap();
    let mut p = Channel::create_on(
        &name,
        4096,
        4,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        FINGERPRINT,
        MemoryPolicy::default(),
        stream.clone(),
        -1,
        Some(0),
    )
    .unwrap();
    assert!(!p.device_backed());
    for (len, fill) in RUNS {
        let mut w = p.loan().unwrap();
        let heap = w.heap().unwrap();
        assert!(heap.gpu);
        // SAFETY: w is live.
        let (at, off) = unsafe { heap.alloc_device(len, 256) }.unwrap();
        // SAFETY: the run is len bytes of device memory this frame owns.
        assert_eq!(unsafe { (cu.memset)(at, fill, len) }, 0);
        let d = w.data_mut();
        d[..8].copy_from_slice(&(GPU_POS + off).to_le_bytes());
        d[8..16].copy_from_slice(&(len as u64).to_le_bytes());
        assert_eq!(w.commit_len(16), Published::Ok);
    }
    in_child(test, &[("FLUX_GPU_ARENA_CHANNEL", name.clone())]);
}

/// The other way: a CPU publisher's frame, one field in its slot and one run in the host arena,
/// read on the device by a subscriber on a stream. The slot comes with the take's copy of the
/// frame; the run is copied by the view's first `gpu_heap`. The second frame fills its slot, so
/// its block list goes to the arena.
#[test]
fn a_gpu_subscriber_reads_a_cpu_frames_arena_run_on_the_device() {
    let Some(cu) = cuda() else { return };
    if probe(0).route != Route::DeviceHandle {
        eprintln!("skipped: an integrated GPU reads the slot in place");
        return;
    }
    let name = signpost_name(&format!("/g{}_host", std::process::id()), FINGERPRINT).unwrap();
    let mut publisher = Channel::create(
        &name,
        4096,
        4,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        FINGERPRINT,
        MemoryPolicy::default(),
    )
    .unwrap();
    let stream = Stream::create().unwrap();
    let mut sub = Channel::open_on(
        &name,
        FINGERPRINT,
        MemoryPolicy::default(),
        stream.clone(),
        -1,
    )
    .unwrap();
    let len = 3 << 20;
    for (fill, used) in [(0x61u8, 16), (0x62, 4096 - TRAILER_BYTES)] {
        let mut w = publisher.loan().unwrap();
        let heap = w.heap().unwrap();
        // SAFETY: w is live.
        let (at, off) = unsafe { heap.alloc(len, 64) }.unwrap();
        // SAFETY: alloc handed out len writable bytes.
        unsafe { std::ptr::write_bytes(at, fill, len) };
        let data = w.data_mut();
        data[..8].copy_from_slice(&(HEAP_POS + off).to_le_bytes());
        data[8..16].fill(fill);
        assert_eq!(w.commit_len(used), Published::Ok);

        let v = sub.take().unwrap();
        let list = if used == 16 { BLOCK_BYTES } else { 0 };
        assert_eq!(v.data().len(), used + list + TRAILER_BYTES);
        let g = v.gpu_heap().expect("a GPU subscriber's view of the frame");
        assert_eq!(g.frame, v.device_ptr());
        assert_eq!(g.count, 1, "the run, copied");
        // SAFETY: count blocks at blocks, valid while v lives.
        let b = unsafe { *g.blocks };
        assert_eq!((b.off, b.len), (HEAP_POS + off, len as u64));
        assert!(v.stream().wait());
        let mut got = vec![0u8; len];
        // SAFETY: got holds len bytes; the copy holds the run.
        let rc = unsafe { (cu.copy)(got.as_mut_ptr().cast(), b.device, len) };
        assert_eq!(rc, 0);
        assert!(got.iter().all(|&x| x == fill), "the run on the device");
        // SAFETY: as above, for the slot field's 8 bytes in the frame's device copy.
        let rc = unsafe { (cu.copy)(got.as_mut_ptr().cast(), g.frame + 8, 8) };
        assert_eq!(rc, 0);
        assert!(
            got[..8].iter().all(|&x| x == fill),
            "the slot field on the device"
        );
        let again = v.gpu_heap().unwrap();
        assert_eq!(
            again.blocks, g.blocks,
            "the second call reads the same copy"
        );
    }
}

/// A GPU arena block whose slot a reader still held when its publisher went is freed once the
/// reader lets go: the next allocation that finds no room frees it and tries again.
#[test]
fn a_device_block_held_past_its_publisher_is_freed_once_its_reader_lets_go() {
    let test = "a_device_block_held_past_its_publisher_is_freed_once_its_reader_lets_go";
    let Some(_cu) = cuda() else { return };
    if std::env::var_os(CHILD).is_none() {
        return in_child(test, &[("FLUX_ARENA_BYTES", (64u64 << 20).to_string())]);
    }
    let stream = Stream::create().unwrap();
    let publisher = |what: &str| {
        let name = signpost_name(&format!("/g{}_{what}", std::process::id()), FINGERPRINT);
        let name = name.unwrap();
        let p = Channel::create_on(
            &name,
            4096,
            2,
            DEFAULT_MAX_SUBSCRIBERS,
            0,
            FINGERPRINT,
            MemoryPolicy::default(),
            stream.clone(),
            -1,
            Some(0),
        )
        .unwrap();
        let mut s = Channel::open(&name, FINGERPRINT, MemoryPolicy::default()).unwrap();
        s.set_qos(QoS::new(2).unwrap().transient_local(2)).unwrap();
        (p, s)
    };
    let (mut p, mut s) = publisher("orphan");
    let mut w = p.loan().unwrap();
    // SAFETY: w is live.
    let (_, off) = unsafe { w.heap().unwrap().alloc_device(12 << 20, 256) }.unwrap();
    w.data_mut()[..8].copy_from_slice(&(GPU_POS + off).to_le_bytes());
    assert_eq!(w.commit_len(8), Published::Ok);
    let held = s.take().unwrap();
    drop(p);
    let (mut q, _t) = publisher("orphan_after");
    let w = q.loan().unwrap();
    // SAFETY: w is live.
    assert!(unsafe { w.heap().unwrap().alloc_device(56 << 20, 256) }.is_none());
    drop(held);
    // SAFETY: w is live.
    assert!(unsafe { w.heap().unwrap().alloc_device(56 << 20, 256) }.is_some());
}

/// A reader that has its publisher's chunks gets the ones the publisher adds later without asking:
/// the publisher sends each as it makes it, before any frame names it. So a publisher that stops
/// right after growing (a debugger, a CPU taken by real-time threads) costs the reader no wait,
/// and a frame past what was ever made is refused at once, frame after frame. In a child process,
/// stopped with SIGSTOP once it has grown.
#[test]
fn a_reader_does_not_wait_for_a_publisher_that_stopped_after_growing() {
    use std::io::{BufRead, Write};
    let test = "a_reader_does_not_wait_for_a_publisher_that_stopped_after_growing";
    if std::env::var_os(CHILD).is_some() {
        let Some(_cu) = cuda() else { return };
        let a = GpuArena::create(0, 256 << 20, Some(Place::Device)).unwrap();
        println!("\nENDPOINT {} {}", a.endpoint(), a.size());
        let mut line = String::new();
        std::io::stdin().read_line(&mut line).unwrap();
        let _ = a.alloc(CHUNK + 4096, 64).unwrap();
        assert_eq!(a.chunks(), 2);
        // SAFETY: stops this process until the parent kills it.
        unsafe { libc::raise(libc::SIGSTOP) };
        return;
    }
    let Some(_cu) = cuda() else { return };
    let mut child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([test, "--exact", "--nocapture"])
        .env(CHILD, "1")
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .spawn()
        .unwrap();
    let mut lines = std::io::BufReader::new(child.stdout.take().unwrap()).lines();
    let (endpoint, size) = loop {
        let line = lines.next().expect("the child's endpoint").unwrap();
        if let Some((e, s)) = line
            .strip_prefix("ENDPOINT ")
            .and_then(|r| r.split_once(' '))
        {
            break (e.to_owned(), s.parse().unwrap());
        }
    };
    let m = GpuArenaImport::open(&endpoint, size, Place::Device, 0).unwrap();
    m.reach(1).unwrap();
    child.stdin.take().unwrap().write_all(b"grow\n").unwrap();
    let stat = format!("/proc/{}/stat", child.id());
    while !std::fs::read_to_string(&stat).unwrap().contains(") T ") {
        std::thread::yield_now();
    }
    let quick = std::time::Duration::from_millis(100);
    let t = std::time::Instant::now();
    let grown = m.reach(2);
    let took = t.elapsed();
    let mut past = Vec::new();
    for _ in 0..3 {
        let t = std::time::Instant::now();
        past.push((m.reach(3).is_err(), t.elapsed()));
    }
    child.kill().unwrap();
    child.wait().unwrap();
    assert!(grown.is_ok() && took < quick, "{grown:?} after {took:?}");
    assert!(
        past.iter().all(|&(refused, took)| refused && took < quick),
        "{past:?}"
    );
}
