//! On the host GPU. Each test returns early where the host has no CUDA device. The GPU writes and
//! reads with `cuMemsetD8Async` and `cuMemcpyDtoH`, which need no compiled kernel.

use std::ffi::{c_int, c_void};
use std::os::fd::AsRawFd;
use std::ptr::NonNull;

use super::driver::{
    ATTR_READ_ONLY_HOST_REGISTER_SUPPORTED, CuResult, DevicePtr, Driver, MEM_ACCESS_READ,
    MEM_ACCESS_READ_WRITE, MEM_LOCATION_TYPE_DEVICE, MemLocation, SUCCESS,
};
use super::{DeviceAlloc, DeviceImport, HostRegistration, Route, Stream, probe};
use crate::sys;

type MemsetAsync = unsafe extern "C" fn(DevicePtr, u8, usize, *mut c_void) -> CuResult;
type CopyDtoH = unsafe extern "C" fn(*mut c_void, DevicePtr, usize) -> CuResult;
type GetAccess = unsafe extern "C" fn(*mut u64, *const MemLocation, DevicePtr) -> CuResult;

type CtxCreate = unsafe extern "C" fn(*mut *mut c_void, std::ffi::c_uint, c_int) -> CuResult;
type CtxDestroy = unsafe extern "C" fn(*mut c_void) -> CuResult;
type MemGetInfo = unsafe extern "C" fn(*mut usize, *mut usize) -> CuResult;
type HostGetFlags = unsafe extern "C" fn(*mut std::ffi::c_uint, *mut c_void) -> CuResult;
type HostFn = unsafe extern "C" fn(*mut c_void);
type LaunchHostFunc = unsafe extern "C" fn(*mut c_void, HostFn, *mut c_void) -> CuResult;

/// Holds a stream until opened, as a hung kernel would; opened on drop so a failed test does not
/// leave the stream stuck. At most 5 s: another test's device-wide synchronizing call (a VMM
/// unmap, a host unregister) waits for this stream, and the gating test may wait on that call.
struct Gate(std::sync::Arc<std::sync::atomic::AtomicBool>);

impl Gate {
    fn on(stream: &Stream) -> Gate {
        use std::sync::Arc;
        use std::sync::atomic::{AtomicBool, Ordering};
        unsafe extern "C" fn hold(p: *mut c_void) {
            // SAFETY: the Arc that `on` leaked for this call.
            let open = unsafe { Arc::from_raw(p.cast_const().cast::<AtomicBool>()) };
            let until = std::time::Instant::now() + std::time::Duration::from_secs(5);
            while !open.load(Ordering::Acquire) && std::time::Instant::now() < until {
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        }
        let open = Arc::new(AtomicBool::new(false));
        // SAFETY: cuLaunchHostFunc's C signature.
        let launch: LaunchHostFunc = unsafe { Driver::get().extra(c"cuLaunchHostFunc") }.unwrap();
        let arg = Arc::into_raw(Arc::clone(&open)).cast_mut().cast();
        // SAFETY: a live stream; hold takes back the Arc passed as its argument.
        assert_eq!(unsafe { launch(stream.native(), hold, arg) }, SUCCESS);
        Gate(open)
    }
}

impl Drop for Gate {
    fn drop(&mut self) {
        self.0.store(true, std::sync::atomic::Ordering::Release);
    }
}

fn gpu() -> Option<(MemsetAsync, CopyDtoH)> {
    let p = probe(0);
    eprintln!("{p:?}");
    if p.route == Route::None {
        return None;
    }
    let drv = Driver::get();
    // SAFETY: cuMemsetD8Async's C signature.
    let memset = unsafe { drv.extra(c"cuMemsetD8Async") }?;
    // SAFETY: cuMemcpyDtoH_v2's C signature.
    let copy = unsafe { drv.extra(c"cuMemcpyDtoH_v2") }?;
    Some((memset, copy))
}

fn fill(memset: MemsetAsync, at: DevicePtr, value: u8, n: usize, stream: &Stream) {
    // SAFETY: at..at+n is device memory the test holds.
    assert_eq!(unsafe { memset(at, value, n, stream.native()) }, SUCCESS);
}

fn read_back(copy: CopyDtoH, from: DevicePtr, n: usize) -> Vec<u8> {
    let mut out = vec![0u8; n];
    // SAFETY: out holds n bytes; from..from+n is device memory the test holds.
    assert_eq!(unsafe { copy(out.as_mut_ptr().cast(), from, n) }, SUCCESS);
    out
}

type ModuleLoadData = unsafe extern "C" fn(*mut *mut c_void, *const c_void) -> CuResult;
type ModuleGetFunction =
    unsafe extern "C" fn(*mut *mut c_void, *mut c_void, *const std::ffi::c_char) -> CuResult;
type LaunchKernel = unsafe extern "C" fn(
    *mut c_void,
    Dim,
    Dim,
    Dim,
    Dim,
    Dim,
    Dim,
    Dim,
    *mut c_void,
    *mut *mut c_void,
    *mut *mut c_void,
) -> CuResult;

type Dim = std::ffi::c_uint;

/// Queues one thread's 4-byte store to `at` on `stream`, through a kernel the driver compiles
/// from PTX: a memset into memory the device may not write is refused before it runs.
fn poke(at: DevicePtr, stream: &Stream) {
    const PTX: &std::ffi::CStr = c".version 7.0
.target sm_50
.address_size 64
.visible .entry poke(.param .u64 p) {
  .reg .b64 %rd<2>;
  .reg .b32 %r<2>;
  ld.param.u64 %rd1, [p];
  mov.u32 %r1, 255;
  st.global.u32 [%rd1], %r1;
  ret;
}
";
    let drv = Driver::get();
    // SAFETY: cuModuleLoadData's C signature.
    let load: ModuleLoadData = unsafe { drv.extra(c"cuModuleLoadData") }.unwrap();
    // SAFETY: cuModuleGetFunction's C signature.
    let get: ModuleGetFunction = unsafe { drv.extra(c"cuModuleGetFunction") }.unwrap();
    // SAFETY: cuLaunchKernel's C signature.
    let launch: LaunchKernel = unsafe { drv.extra(c"cuLaunchKernel") }.unwrap();
    let (mut module, mut f) = (std::ptr::null_mut(), std::ptr::null_mut());
    // SAFETY: a NUL-terminated PTX image; valid out-pointers.
    let rc = unsafe { load(&raw mut module, PTX.as_ptr().cast()) };
    assert_eq!(rc, SUCCESS);
    // SAFETY: a module just loaded; a NUL-terminated name.
    let rc = unsafe { get(&raw mut f, module, c"poke".as_ptr()) };
    assert_eq!(rc, SUCCESS);
    let mut arg = at;
    let mut params = [(&raw mut arg).cast::<c_void>()];
    // SAFETY: one block of one thread; params holds the kernel's one u64 parameter.
    let rc = unsafe {
        launch(
            f,
            1,
            1,
            1,
            1,
            1,
            1,
            0,
            stream.native(),
            params.as_mut_ptr(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(rc, SUCCESS);
}

#[test]
fn the_legacy_default_stream_is_refused() {
    // SAFETY: null is refused before it is used.
    assert!(unsafe { Stream::from_native(std::ptr::null_mut()) }.is_err());
}

#[test]
fn an_undeclared_stream_waits_for_nothing() {
    let s = Stream::default();
    assert!(!s.declared() && s.wait() && s.native().is_null());
}

#[test]
fn a_stream_is_waited_for_on_a_thread_without_a_context() {
    let Some((memset, _)) = gpu() else { return };
    let stream = Stream::create().unwrap();
    let alloc = DeviceAlloc::create(&format!("t{}.ctx", std::process::id()), 1 << 20, 0).unwrap();
    fill(memset, alloc.base(), 1, 1 << 20, &stream);
    let s = stream.clone();
    let waited = std::thread::spawn(move || {
        assert!(
            Driver::get()
                .core
                .as_ref()
                .unwrap()
                .current_context()
                .is_none()
        );
        s.wait()
    });
    assert!(waited.join().unwrap());
}

#[test]
fn a_registered_host_mapping_is_written_by_the_device() {
    let Some((memset, _)) = gpu() else { return };
    if probe(0).route != Route::ShmRegistered && !probe(0).host_register {
        return;
    }
    let n = 1 << 16;
    let fd = sys::shm_open(
        &format!("/flux.gputest.{}", std::process::id()),
        libc::O_CREAT | libc::O_RDWR,
    )
    .unwrap();
    sys::shm_unlink(&format!("/flux.gputest.{}", std::process::id()));
    sys::allocate(fd.as_raw_fd(), n).unwrap();
    let base: NonNull<u8> = sys::map_shared(fd.as_raw_fd(), n).unwrap();
    let stream = Stream::create().unwrap();
    {
        let reg = HostRegistration::new(base.as_ptr(), n, false, 0).unwrap();
        fill(memset, reg.device_base(), 0x5a, n, &stream);
        assert!(stream.wait());
        // SAFETY: n mapped bytes, written by the device and waited for.
        let host = unsafe { std::slice::from_raw_parts(base.as_ptr(), n) };
        assert!(host.iter().all(|&b| b == 0x5a));
    }
    // SAFETY: mapped above with n bytes; the registration is gone.
    unsafe { sys::munmap(base, n) };
}

#[test]
fn a_read_only_mapping_registers_read_only() {
    if gpu().is_none() || !probe(0).host_register {
        return;
    }
    let core = super::core().unwrap();
    let mut dev = 0;
    // SAFETY: dev is a valid out-pointer.
    assert_eq!(unsafe { (core.device_get)(&raw mut dev, 0) }, SUCCESS);
    if !core.attribute(dev, ATTR_READ_ONLY_HOST_REGISTER_SUPPORTED) {
        return;
    }
    let n = 1 << 16;
    let name = format!("/flux.gputest.ro.{}", std::process::id());
    let fd = sys::shm_open(&name, libc::O_CREAT | libc::O_RDWR).unwrap();
    sys::shm_unlink(&name);
    sys::allocate(fd.as_raw_fd(), n).unwrap();
    let base = sys::map_readonly(fd.as_raw_fd(), n).unwrap();
    assert!(HostRegistration::new(base.as_ptr(), n, false, 0).is_err());
    let reg = HostRegistration::new(base.as_ptr(), n, true, 0);
    assert!(reg.is_ok());
    drop(reg);
    // SAFETY: mapped above with n bytes; the registration is gone.
    unsafe { sys::munmap(base, n) };
}

#[test]
fn an_exported_allocation_is_imported_through_its_endpoint() {
    let Some((memset, copy)) = gpu() else { return };
    let p = probe(0);
    if !(p.vmm && p.posix_fd_handle) {
        return;
    }
    let n = 3 << 20;
    let endpoint = format!("t{}.vmm", std::process::id());
    let alloc = DeviceAlloc::create(&endpoint, n, 0).unwrap();
    let stream = Stream::create().unwrap();
    fill(memset, alloc.base(), 0x33, n, &stream);
    assert!(stream.wait());
    let import = DeviceImport::open(&endpoint, n, 0).unwrap();
    assert_ne!(import.base(), alloc.base());
    assert!(read_back(copy, import.base(), n).iter().all(|&b| b == 0x33));
    // Read-only for the importer, asked rather than tried: a faulting write would poison the
    // context for every test after it.
    // SAFETY: cuMemGetAccess's C signature.
    let get_access: GetAccess = unsafe { Driver::get().extra(c"cuMemGetAccess") }.unwrap();
    let loc = MemLocation {
        kind: MEM_LOCATION_TYPE_DEVICE,
        id: 0,
    };
    let (mut theirs, mut ours) = (0u64, 0u64);
    // SAFETY: valid pointers into a mapped range.
    let rc = unsafe { get_access(&raw mut theirs, &raw const loc, import.base()) };
    assert_eq!(rc, SUCCESS);
    // SAFETY: as above.
    let rc = unsafe { get_access(&raw mut ours, &raw const loc, alloc.base()) };
    assert_eq!(rc, SUCCESS);
    assert_eq!(
        (theirs, ours),
        (MEM_ACCESS_READ as u64, MEM_ACCESS_READ_WRITE as u64)
    );
}

#[test]
fn an_allocation_comes_apart_after_the_context_current_at_its_making_is_destroyed() {
    if gpu().is_none() || !(probe(0).vmm && probe(0).posix_fd_handle) {
        return;
    }
    let drv = Driver::get();
    // SAFETY: cuCtxCreate_v2's C signature.
    let create: CtxCreate = unsafe { drv.extra(c"cuCtxCreate_v2") }.unwrap();
    // SAFETY: cuCtxDestroy_v2's C signature.
    let destroy: CtxDestroy = unsafe { drv.extra(c"cuCtxDestroy_v2") }.unwrap();
    // SAFETY: cuMemGetInfo_v2's C signature.
    let info: MemGetInfo = unsafe { drv.extra(c"cuMemGetInfo_v2") }.unwrap();
    let free = || {
        let (mut free, mut total) = (0, 0);
        // SAFETY: valid out-pointers, on a thread with a current context.
        assert_eq!(unsafe { info(&raw mut free, &raw mut total) }, SUCCESS);
        free
    };
    let _primary = Stream::create().unwrap();
    let before = free();
    let core = super::core().unwrap();
    let mut dev = 0;
    // SAFETY: dev is a valid out-pointer.
    assert_eq!(unsafe { (core.device_get)(&raw mut dev, 0) }, SUCCESS);
    let mut ctx = std::ptr::null_mut();
    // SAFETY: ctx is a valid out-pointer; the new context is pushed on this thread.
    assert_eq!(unsafe { create(&raw mut ctx, 0, dev) }, SUCCESS);
    let n = 512 << 20;
    let endpoint = format!("t{}.ctx", std::process::id());
    let alloc = DeviceAlloc::create(&endpoint, n, 0).unwrap();
    let import = DeviceImport::open(&endpoint, n, 0).unwrap();
    // SAFETY: created above; destroying it pops it and leaves the primary context current.
    assert_eq!(unsafe { destroy(ctx) }, SUCCESS);
    drop(import);
    drop(alloc);
    let leaked = before.saturating_sub(free());
    assert!(leaked < n / 2, "{leaked} bytes still allocated");
}

#[test]
fn a_registration_comes_apart_after_the_context_current_at_its_making_is_destroyed() {
    if gpu().is_none() || !probe(0).host_register {
        return;
    }
    let drv = Driver::get();
    // SAFETY: cuCtxCreate_v2's C signature.
    let create: CtxCreate = unsafe { drv.extra(c"cuCtxCreate_v2") }.unwrap();
    // SAFETY: cuCtxDestroy_v2's C signature.
    let destroy: CtxDestroy = unsafe { drv.extra(c"cuCtxDestroy_v2") }.unwrap();
    let n = 1 << 16;
    let name = format!("/flux.gputest.ctx.{}", std::process::id());
    let fd = sys::shm_open(&name, libc::O_CREAT | libc::O_RDWR).unwrap();
    sys::shm_unlink(&name);
    sys::allocate(fd.as_raw_fd(), n).unwrap();
    let base = sys::map_shared(fd.as_raw_fd(), n).unwrap();
    let core = super::core().unwrap();
    let mut dev = 0;
    // SAFETY: dev is a valid out-pointer.
    assert_eq!(unsafe { (core.device_get)(&raw mut dev, 0) }, SUCCESS);
    let mut ctx = std::ptr::null_mut();
    // SAFETY: ctx is a valid out-pointer; the new context is pushed on this thread.
    assert_eq!(unsafe { create(&raw mut ctx, 0, dev) }, SUCCESS);
    let reg = HostRegistration::new(base.as_ptr(), n, false, 0).unwrap();
    // SAFETY: created above; destroying it pops it.
    assert_eq!(unsafe { destroy(ctx) }, SUCCESS);
    // SAFETY: cuMemHostGetFlags's C signature.
    let flags: HostGetFlags = unsafe { drv.extra(c"cuMemHostGetFlags") }.unwrap();
    let primary = core.primary_context(0).unwrap();
    let mut f = 0;
    // SAFETY: f is a valid out-pointer; base is mapped.
    let rc = core.with_context(primary, || unsafe {
        flags(&raw mut f, base.as_ptr().cast())
    });
    assert_eq!(
        rc, SUCCESS,
        "the registration went with the destroyed context"
    );
    drop(reg);
    let again = HostRegistration::new(base.as_ptr(), n, false, 0);
    assert!(again.is_ok(), "the first registration was never undone");
    drop(again);
    // SAFETY: mapped above with n bytes; the registrations are gone.
    unsafe { sys::munmap(base, n) };
}

#[test]
fn an_endpoint_nobody_serves_is_not_ready() {
    if gpu().is_none() {
        return;
    }
    let r = DeviceImport::open(&format!("t{}.nobody", std::process::id()), 1 << 20, 0);
    assert!(
        matches!(r, Err(crate::error::Error::NotReady(_))),
        "{:?}",
        r.err()
    );
}

/// Unregistrations on this thread that found their memory already unmapped.
pub(super) fn note_unregister(base: usize) {
    let mut resident = 0u8;
    // SAFETY: base is page-aligned; mincore only reports, and fails with ENOMEM if unmapped.
    let r = unsafe { libc::mincore(base as *mut c_void, 1, &raw mut resident) };
    if r != 0 {
        UNMAPPED_UNREGISTERS.set(UNMAPPED_UNREGISTERS.get() + 1);
    }
}

/// True in a child of this test binary that runs only `test` with `env` set; the parent runs that
/// child and checks that it passed.
fn child(test: &str, env: &str) -> bool {
    if std::env::var_os(env).is_some() {
        return true;
    }
    let out = std::process::Command::new(std::env::current_exe().unwrap())
        .args([test, "--exact", "--include-ignored"])
        .env(env, "1")
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&out.stdout);
    assert!(
        out.status.success() && log.contains("1 passed"),
        "{log}{}",
        String::from_utf8_lossy(&out.stderr)
    );
    false
}

mod stand_in;

thread_local! {
    pub(super) static UNMAPPED_UNREGISTERS: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
    /// The route `probe` reports on this thread, in place of the host's.
    pub(super) static ROUTE: std::cell::Cell<Option<Route>> = const { std::cell::Cell::new(None) };
    /// Every stream wait on this thread fails.
    pub(super) static FAIL_FENCES: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// The ordinal `device_for_uuid` finds on this thread, in place of the host's.
    pub(super) static UUID_DEVICE: std::cell::Cell<Option<i32>> =
        const { std::cell::Cell::new(None) };
}

mod channel {
    use std::sync::atomic::{AtomicU32, Ordering};

    use super::{FAIL_FENCES, Gate, ROUTE, UUID_DEVICE, fill, gpu, read_back};
    use crate::gpu::{Stream, probe};
    use crate::layout::{DEFAULT_MAX_SUBSCRIBERS, NO_SCHEMA};
    use crate::qos::{MemoryPolicy, QoS};
    use crate::{Channel, Error, Published, signpost_name};

    pub(super) fn topic(what: &str) -> String {
        static N: AtomicU32 = AtomicU32::new(0);
        let key = format!(
            "/g{}_{}_{what}",
            std::process::id(),
            N.fetch_add(1, Ordering::Relaxed)
        );
        signpost_name(&key, NO_SCHEMA).unwrap()
    }

    pub(super) fn publisher(name: &str, stream: Stream) -> Channel {
        let mem = MemoryPolicy::default();
        Channel::create_on(
            name,
            1 << 16,
            4,
            DEFAULT_MAX_SUBSCRIBERS,
            NO_SCHEMA,
            mem,
            stream,
            crate::trace::NO_STAMP,
        )
        .unwrap()
    }

    pub(super) fn subscriber(name: &str, stream: Stream) -> crate::Result<Channel> {
        let mut s = Channel::open_on(
            name,
            NO_SCHEMA,
            MemoryPolicy::default(),
            stream,
            crate::trace::NO_STAMP,
        )?;
        s.set_qos(QoS::new(4).unwrap().transient_local(4)).unwrap();
        Ok(s)
    }

    struct Forced(Option<super::Route>);
    impl Forced {
        fn route(r: super::Route) -> Forced {
            Forced(ROUTE.replace(Some(r)))
        }
    }
    impl Drop for Forced {
        fn drop(&mut self) {
            ROUTE.set(self.0);
        }
    }

    #[test]
    fn a_registered_payload_is_written_by_the_device_and_read_by_both_kinds_of_subscriber() {
        let Some((memset, copy)) = gpu() else { return };
        if probe(0).route != super::Route::ShmRegistered {
            return;
        }
        let name = topic("reg");
        let mut p = publisher(&name, Stream::create().unwrap());
        assert!(!p.device_backed());
        let w = p.loan().unwrap();
        assert_ne!(w.device_ptr(), 0);
        fill(memset, w.device_ptr(), 0x7e, 4096, &w.stream());
        assert_eq!(w.commit_len(4096), Published::Ok);
        assert_eq!(p.fence_wait().commit_waits, 1);

        let mut cpu = subscriber(&name, Stream::default()).unwrap();
        let v = cpu.take().unwrap();
        assert_eq!(v.device_ptr(), 0);
        assert!(v.data().iter().all(|&b| b == 0x7e));
        drop(v);

        let mut dev = subscriber(&name, Stream::create().unwrap()).unwrap();
        let v = dev.take().unwrap();
        assert_ne!(v.device_ptr(), 0);
        assert!(
            read_back(copy, v.device_ptr(), 4096)
                .iter()
                .all(|&b| b == 0x7e)
        );
        assert!(v.data().iter().all(|&b| b == 0x7e));
        drop(v);
        assert_eq!(dev.fence_wait().release_waits, 1);
    }

    /// The order a channel comes apart in: the registration goes while its memory is mapped.
    #[test]
    fn a_registration_is_undone_before_its_mapping_goes() {
        if gpu().is_none() {
            return;
        }
        let _route = Forced::route(super::Route::ShmRegistered);
        let name = topic("unregister");
        let mut p = publisher(&name, Stream::create().unwrap());
        assert!(!p.device_backed() && p.loan().unwrap().device_ptr() != 0);
        drop(p);
        assert_eq!(super::UNMAPPED_UNREGISTERS.get(), 0);
    }

    fn discrete() -> Option<Forced> {
        let p = probe(0);
        (p.vmm && p.posix_fd_handle).then(|| Forced::route(super::Route::DeviceHandle))
    }

    fn pattern() -> Vec<u8> {
        #[allow(clippy::cast_possible_truncation)]
        (0..4096u32).map(|i| (i * 7) as u8).collect()
    }

    #[test]
    fn a_device_payload_reaches_both_kinds_of_subscriber() {
        let Some((memset, copy)) = gpu() else { return };
        let Some(_route) = discrete() else { return };
        let name = topic("dev");
        let mut p = publisher(&name, Stream::create().unwrap());
        assert!(p.device_backed());
        let mut cpu = subscriber(&name, Stream::default()).unwrap();
        let mut dev = subscriber(&name, Stream::create().unwrap()).unwrap();
        let mut w = p.loan().unwrap();
        assert!(w.data_mut().is_empty());
        fill(memset, w.device_ptr(), 0x21, 4096, &w.stream());
        assert_eq!(w.commit_len(4096), Published::Ok);

        let v = dev.take().unwrap();
        assert!(v.data().is_empty() && !v.host_addressable());
        assert_eq!(v.meta().nbytes, 4096);
        assert!(
            read_back(copy, v.device_ptr(), 4096)
                .iter()
                .all(|&b| b == 0x21)
        );
        drop(v);

        let v = cpu.take().unwrap();
        assert!(v.host_addressable());
        assert_eq!((v.device_ptr(), v.stream().declared()), (0, false));
        assert_eq!(v.data().len(), 4096);
        assert!(v.data().iter().all(|&b| b == 0x21));
    }

    #[test]
    fn host_bytes_published_into_a_device_payload_reach_both_kinds_of_subscriber() {
        let Some((_, copy)) = gpu() else { return };
        let Some(_route) = discrete() else { return };
        let name = topic("dev_publish");
        let mut p = publisher(&name, Stream::create().unwrap());
        let mut cpu = subscriber(&name, Stream::default()).unwrap();
        let mut dev = subscriber(&name, Stream::create().unwrap()).unwrap();
        let bytes = pattern();
        assert_eq!(p.publish(&bytes), Published::Ok);

        let v = dev.take().unwrap();
        assert_eq!(read_back(copy, v.device_ptr(), bytes.len()), bytes);
        drop(v);
        assert_eq!(cpu.take().unwrap().data(), &bytes[..]);
    }

    #[test]
    fn a_host_payload_is_copied_for_a_device_subscriber_on_a_discrete_gpu() {
        let Some((_, copy)) = gpu() else { return };
        let Some(_route) = discrete() else { return };
        let name = topic("h2d");
        let mut p = publisher(&name, Stream::default());
        assert!(!p.device_backed());
        let mut dev = subscriber(&name, Stream::create().unwrap()).unwrap();
        let bytes = pattern();
        let other: Vec<u8> = bytes.iter().map(|b| !b).collect();
        assert_eq!(p.publish(&bytes), Published::Ok);
        assert_eq!(p.publish(&other), Published::Ok);

        let qos = QoS::new(4).unwrap().transient_local(4).max_borrow(2);
        dev.set_qos(qos.unwrap()).unwrap();
        let first = dev.take().unwrap();
        let second = dev.take().unwrap();
        assert!(first.host_addressable() && first.data() == &bytes[..]);
        assert_ne!(first.device_ptr(), second.device_ptr());
        assert!(first.stream().wait());
        assert_eq!(read_back(copy, first.device_ptr(), bytes.len()), bytes);
        assert_eq!(read_back(copy, second.device_ptr(), other.len()), other);
    }

    /// The endpoint of the publisher's segment, found by the segment file's inode.
    fn endpoint_of(signpost: &str) -> String {
        use std::os::unix::fs::MetadataExt;
        let seg = format!("{}.e", signpost.trim_start_matches('/'));
        let meta = std::fs::read_dir("/dev/shm")
            .unwrap()
            .flatten()
            .find(|e| e.file_name().to_str().is_some_and(|n| n.starts_with(&seg)))
            .unwrap()
            .metadata()
            .unwrap();
        crate::gpu::endpoint(crate::segment::SegmentId {
            dev: meta.dev(),
            ino: meta.ino(),
        })
    }

    #[test]
    fn the_endpoint_is_a_private_socket_file_that_goes_with_its_publisher() {
        use std::os::unix::fs::{FileTypeExt, PermissionsExt};
        if gpu().is_none() || !(probe(0).vmm && probe(0).posix_fd_handle) {
            return;
        }
        let _route = Forced::route(super::Route::DeviceHandle);
        let name = topic("ep");
        let p = publisher(&name, Stream::create().unwrap());
        let path = endpoint_of(&name);
        let meta = std::fs::symlink_metadata(&path).unwrap();
        assert!(meta.file_type().is_socket());
        assert_eq!(meta.permissions().mode() & 0o777, 0o600);
        drop(p);
        assert!(std::fs::symlink_metadata(&path).is_err());
    }

    #[test]
    fn a_sweep_removes_only_an_endpoint_nothing_listens_on() {
        use std::os::unix::net::UnixListener;
        let base = format!("/dev/shm/flux.gpu.{}", std::process::id());
        let (dead, live, binding) = (
            format!("{base}.1"),
            format!("{base}.2"),
            format!("{base}.3.{}", std::process::id()),
        );
        drop(UnixListener::bind(&dead).unwrap());
        let _listening = UnixListener::bind(&live).unwrap();
        drop(UnixListener::bind(&binding).unwrap());
        for path in [&dead, &live, &binding] {
            crate::gpu::remove_dead_endpoint(path.trim_start_matches("/dev/shm/"));
        }
        assert!(std::fs::symlink_metadata(&dead).is_err());
        assert!(std::fs::symlink_metadata(&live).is_ok());
        assert!(
            std::fs::symlink_metadata(&binding).is_ok(),
            "a bind of seconds ago"
        );
        let _ = std::fs::remove_file(&live);
        let _ = std::fs::remove_file(&binding);
    }

    #[test]
    fn a_device_payload_refuses_a_stream_on_another_device() {
        if gpu().is_none() || !(probe(0).vmm && probe(0).posix_fd_handle) {
            return;
        }
        let _route = Forced::route(super::Route::DeviceHandle);
        let name = topic("other");
        let _p = publisher(&name, Stream::create().unwrap());
        UUID_DEVICE.set(Some(1));
        let r = subscriber(&name, Stream::create().unwrap());
        UUID_DEVICE.set(None);
        assert!(matches!(r, Err(Error::Mismatch(_))), "{:?}", r.err());
    }

    #[test]
    fn a_host_route_does_not_import_a_device_payload() {
        if gpu().is_none() || !probe(0).vmm {
            return;
        }
        let name = topic("mix");
        let _p = {
            let _route = Forced::route(super::Route::DeviceHandle);
            publisher(&name, Stream::create().unwrap())
        };
        let _route = Forced::route(super::Route::ShmRegistered);
        let r = subscriber(&name, Stream::create().unwrap());
        assert!(matches!(r, Err(Error::Mismatch(_))), "{:?}", r.err());
    }

    #[test]
    fn a_direct_payload_is_its_own_device_address() {
        if gpu().is_none() {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("direct");
        let mut p = publisher(&name, Stream::create().unwrap());
        let mut w = p.loan().unwrap();
        assert_eq!(w.device_ptr(), w.data_mut().as_ptr() as u64);
    }

    #[test]
    fn a_failed_commit_fence_keeps_the_slot_and_is_counted() {
        if gpu().is_none() {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("fcommit");
        let mut p = publisher(&name, Stream::create().unwrap());
        let w = p.loan().unwrap();
        FAIL_FENCES.set(true);
        assert_eq!(w.commit_len(8), Published::FenceFailed);
        assert_eq!(p.publish(b"x"), Published::FenceFailed);
        FAIL_FENCES.set(false);
        assert_eq!(p.fence_failed(), 2);
        // Three slots left of four; the claimed one is never handed out again.
        for _ in 0..3 {
            assert!(p.loan().is_some_and(|w| w.commit_len(1) == Published::Ok));
        }
        let held: Vec<_> = (0..4).filter_map(|_| p.loan()).collect();
        assert_eq!(held.len(), 3, "the fenced slot came back");
    }

    #[test]
    fn failed_release_fences_leak_their_borrows_until_the_take_is_refused_for_it() {
        if gpu().is_none() {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("frelease");
        let mut p = publisher(&name, Stream::create().unwrap());
        for _ in 0..4 {
            assert_eq!(p.publish(b"frame"), Published::Ok);
        }
        let mut s = subscriber(&name, Stream::create().unwrap()).unwrap();
        s.set_qos(
            QoS::new(4)
                .unwrap()
                .transient_local(4)
                .max_borrow(2)
                .unwrap(),
        )
        .unwrap();
        FAIL_FENCES.set(true);
        drop(s.take().unwrap());
        drop(s.take().unwrap());
        FAIL_FENCES.set(false);
        assert_eq!(s.fence_failed(), 2);
        assert!(s.take().is_none());
        assert_eq!((s.refused().fence, s.refused().max_borrow), (1, 0));
    }

    /// True in a child of this test binary that runs only `test`. A gated stream stalls every
    /// device-wide synchronizing call the other tests make, and those hold driver locks the gated
    /// test then waits on, so it runs where nothing else does.
    fn alone(test: &str) -> bool {
        super::child(&format!("gpu::tests::channel::{test}"), "FLUX_GATED_TEST")
    }

    /// Slots a claim can take now, each held until all are counted.
    pub(super) fn claimable(p: &mut Channel) -> usize {
        let held: Vec<_> = (0..4).filter_map(|_| p.loan()).collect();
        held.len()
    }

    #[test]
    fn a_timed_out_release_keeps_its_slot_until_the_work_is_done() {
        if gpu().is_none() || !alone("a_timed_out_release_keeps_its_slot_until_the_work_is_done") {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("rlate");
        let mut p = publisher(&name, Stream::create().unwrap());
        for _ in 0..4 {
            assert_eq!(p.publish(b"frame"), Published::Ok);
        }
        let stream = Stream::create().unwrap();
        let mut s = subscriber(&name, stream.clone()).unwrap();
        s.set_fence_timeout(20_000_000);
        let gate = Gate::on(&stream);
        let t0 = std::time::Instant::now();
        drop(s.take().unwrap());
        assert!(t0.elapsed() < std::time::Duration::from_secs(1));
        assert_eq!(s.fence_failed(), 1);
        assert_eq!(claimable(&mut p), 3, "held while the work runs");
        drop(gate);
        assert!(stream.wait());
        drop(s.take());
        assert_eq!(claimable(&mut p), 4, "given back once it is done");
    }

    #[test]
    fn a_timed_out_commit_publishes_nothing_and_frees_its_claim_once_the_work_is_done() {
        if gpu().is_none()
            || !alone(
                "a_timed_out_commit_publishes_nothing_and_frees_its_claim_once_the_work_is_done",
            )
        {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("clate");
        let stream = Stream::create().unwrap();
        let mut p = publisher(&name, stream.clone());
        p.set_fence_timeout(20_000_000);
        let gate = Gate::on(&stream);
        assert_eq!(p.loan().unwrap().commit(), Published::FenceFailed);
        assert_eq!(claimable(&mut p), 3, "claimed while the work runs");
        drop(gate);
        assert!(stream.wait());
        let mut s = subscriber(&name, Stream::default()).unwrap();
        assert!(s.take().is_none(), "nothing was published");
        assert_eq!(claimable(&mut p), 4, "freed once it is done");
    }

    #[test]
    fn a_deferred_release_returns_at_once_and_keeps_its_slot_until_the_work_is_done() {
        let me = "a_deferred_release_returns_at_once_and_keeps_its_slot_until_the_work_is_done";
        if gpu().is_none() || !alone(me) {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("rdef");
        let mut p = publisher(&name, Stream::create().unwrap());
        for _ in 0..4 {
            assert_eq!(p.publish(b"frame"), Published::Ok);
        }
        let stream = Stream::create().unwrap();
        let mut s = subscriber(&name, stream.clone()).unwrap();
        s.set_release_mode(crate::ReleaseMode::Deferred);
        let gate = Gate::on(&stream);
        let t0 = std::time::Instant::now();
        drop(s.take().unwrap());
        assert!(
            t0.elapsed() < std::time::Duration::from_millis(100),
            "{:?}",
            t0.elapsed()
        );
        assert_eq!(s.fence_failed(), 0);
        assert_eq!(claimable(&mut p), 3, "held while the device reads it");
        drop(gate);
        assert!(stream.wait());
        drop(s.take());
        assert_eq!(claimable(&mut p), 4, "given back once the device is done");
    }

    /// Mappings of the segment behind `signpost`, from this process's maps.
    fn segment_mappings(signpost: &str) -> usize {
        let seg = format!("{}.e", signpost.trim_start_matches('/'));
        std::fs::read_to_string("/proc/self/maps")
            .unwrap()
            .lines()
            .filter(|l| l.contains(&seg))
            .count()
    }

    #[test]
    fn a_deferred_release_outliving_its_subscription_lets_the_mapping_go_once_done() {
        let me = "a_deferred_release_outliving_its_subscription_lets_the_mapping_go_once_done";
        if gpu().is_none() || !alone(me) {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("rorph");
        let mut p = publisher(&name, Stream::create().unwrap());
        assert_eq!(p.publish(b"frame"), Published::Ok);
        let stream = Stream::create().unwrap();
        let mut s = subscriber(&name, stream.clone()).unwrap();
        s.set_release_mode(crate::ReleaseMode::Deferred);
        let gate = Gate::on(&stream);
        drop(s.take().unwrap());
        drop(s);
        drop(p);
        assert!(
            segment_mappings(&name) > 0,
            "kept while the device reads it"
        );
        drop(gate);
        assert!(stream.wait());
        let t0 = std::time::Instant::now();
        while segment_mappings(&name) > 0 && t0.elapsed() < std::time::Duration::from_secs(5) {
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert_eq!(segment_mappings(&name), 0, "kept after the device was done");
    }

    struct Sub(Channel, std::sync::Arc<AtomicU32>);

    impl crate::executor::Source for Sub {
        fn attach(&mut self) -> crate::Result<bool> {
            Ok(true)
        }
        fn deliver_one(&mut self) -> crate::Result<bool> {
            let got = self.0.take().is_some();
            self.1.fetch_add(u32::from(got), Ordering::Relaxed);
            Ok(got)
        }
        fn channel(&self) -> Option<&Channel> {
            Some(&self.0)
        }
    }

    /// No frame and no tick: only the waiter's poke can end the wait.
    #[test]
    fn the_executor_wakes_when_deferred_work_is_done() {
        let me = "the_executor_wakes_when_deferred_work_is_done";
        if gpu().is_none() || !alone(me) {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("rwake");
        let mut p = publisher(&name, Stream::create().unwrap());
        for _ in 0..2 {
            assert_eq!(p.publish(b"frame"), Published::Ok);
        }
        let stream = Stream::create().unwrap();
        let mut s = subscriber(&name, stream.clone()).unwrap();
        let q = QoS::new(4)
            .unwrap()
            .transient_local(4)
            .max_borrow(1)
            .unwrap();
        s.set_qos(q).unwrap();
        s.set_release_mode(crate::ReleaseMode::Deferred);
        let got = std::sync::Arc::new(AtomicU32::new(0));
        let mut ex = crate::executor::Executor::new().unwrap();
        ex.add(Sub(s, std::sync::Arc::clone(&got)), 0).unwrap();
        let gate = Gate::on(&stream);
        ex.dispatch().unwrap();
        assert_eq!(
            got.load(Ordering::Relaxed),
            1,
            "the second waits for the first's slot"
        );
        let opener = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(100));
            drop(gate);
        });
        let t0 = std::time::Instant::now();
        ex.wait_for_work(10_000_000_000).unwrap();
        let woke = t0.elapsed();
        opener.join().unwrap();
        assert!(woke < std::time::Duration::from_secs(2), "{woke:?}");
        ex.dispatch().unwrap();
        assert_eq!(got.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn a_bounded_wait_ends_soon_after_the_work_does() {
        if gpu().is_none() || !alone("a_bounded_wait_ends_soon_after_the_work_does") {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("rsoon");
        let mut p = publisher(&name, Stream::create().unwrap());
        assert_eq!(p.publish(b"frame"), Published::Ok);
        let stream = Stream::create().unwrap();
        let mut s = subscriber(&name, stream.clone()).unwrap();
        s.set_fence_timeout(5_000_000_000);
        let gate = Gate::on(&stream);
        let v = s.take().unwrap();
        let opened = std::sync::Arc::new(std::sync::Mutex::new(None));
        let at = std::sync::Arc::clone(&opened);
        let opener = std::thread::spawn(move || {
            std::thread::sleep(std::time::Duration::from_millis(100));
            *at.lock().unwrap() = Some(std::time::Instant::now());
            drop(gate);
        });
        drop(v);
        let done = std::time::Instant::now();
        opener.join().unwrap();
        let lag = done - opened.lock().unwrap().unwrap();
        assert!(lag < std::time::Duration::from_millis(5), "{lag:?}");
        assert_eq!(s.fence_failed(), 0);
    }

    /// A real sticky error: a kernel writes through a subscriber's read-only import.
    #[test]
    #[ignore = "faults the GPU on purpose, which the kernel logs as an Xid"]
    fn a_write_through_a_read_only_import_wedges_every_gpu_channel() {
        let me = "a_write_through_a_read_only_import_wedges_every_gpu_channel";
        if gpu().is_none() || !(probe(0).vmm && probe(0).posix_fd_handle) || !alone(me) {
            return;
        }
        let _route = Forced::route(super::Route::DeviceHandle);
        let name = topic("ro");
        let mut p = publisher(&name, Stream::create().unwrap());
        assert_eq!(p.loan().unwrap().commit_len(4096), Published::Ok);
        let stream = Stream::create().unwrap();
        let mut s = subscriber(&name, stream.clone()).unwrap();
        let v = s.take().unwrap();
        super::poke(v.device_ptr(), &stream);
        drop(v);
        assert!(
            crate::gpu::wedged().is_some(),
            "an illegal address (700) here"
        );
        assert!(s.take().is_none());
        assert_eq!(s.refused().fence, 1);
        assert!(p.loan().is_none());
        assert!(Stream::create().is_err());
    }

    #[test]
    fn a_failed_release_fence_keeps_its_slot_after_the_channel_is_gone() {
        if gpu().is_none() {
            return;
        }
        let _route = Forced::route(super::Route::ShmDirect);
        let name = topic("fgone");
        let mut p = publisher(&name, Stream::create().unwrap());
        for _ in 0..4 {
            assert_eq!(p.publish(b"frame"), Published::Ok);
        }
        let mut s = subscriber(&name, Stream::create().unwrap()).unwrap();
        FAIL_FENCES.set(true);
        drop(s.take().unwrap());
        FAIL_FENCES.set(false);
        drop(s);
        let _next = subscriber(&name, Stream::create().unwrap()).unwrap();
        let held: Vec<_> = (0..4).filter_map(|_| p.loan()).collect();
        assert_eq!(held.len(), 3, "the fenced slot came back");
    }
}
