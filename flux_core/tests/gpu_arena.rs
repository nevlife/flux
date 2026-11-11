//! The GPU arena across processes: a publisher grows it chunk by chunk and a child of this test
//! binary imports the chunks and reads each run back, on the device and, for a `HOST_NUMA` arena,
//! from the CPU. A binary of its own: the child makes a CUDA context of its own, which in the
//! library's test binary would run beside the tests that time GPU work or read free memory.
//! Returns early without a CUDA device with VMM.

use std::ffi::{CStr, c_void};

use flux_core::gpu::arena::{CHUNK, GpuArena, GpuArenaImport, Place};
use flux_core::gpu::{Stream, probe};

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
