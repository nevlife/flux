//! Thin Linux wrappers: OFD locks, shm, mmap, futex, clock, and the fork rule for lock fds.

use std::cell::UnsafeCell;
use std::ffi::CString;
use std::fs::OpenOptions;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::ptr::{self, NonNull};
use std::sync::Once;
use std::sync::atomic::{AtomicU32, Ordering};

use crate::error::{Error, Result};

fn flock_at(kind: libc::c_int, byte: i64) -> libc::flock {
    // SAFETY: flock is plain integers; all-zero is a valid value.
    let mut fl: libc::flock = unsafe { std::mem::zeroed() };
    #[allow(clippy::cast_possible_truncation)]
    {
        fl.l_type = kind as libc::c_short;
        fl.l_whence = libc::SEEK_SET as libc::c_short;
    }
    fl.l_start = byte;
    fl.l_len = 1;
    fl
}

fn ofd_setlk(fd: RawFd, kind: libc::c_int, byte: i64) -> bool {
    let fl = flock_at(kind, byte);
    // SAFETY: fl is a valid flock that outlives the call.
    unsafe { libc::fcntl(fd, libc::F_OFD_SETLK, &raw const fl) == 0 }
}

/// Non-blocking. False on conflict.
pub(crate) fn lock_read(fd: RawFd, byte: i64) -> bool {
    ofd_setlk(fd, libc::F_RDLCK, byte)
}

pub(crate) fn lock_write(fd: RawFd, byte: i64) -> bool {
    ofd_setlk(fd, libc::F_WRLCK, byte)
}

pub(crate) fn unlock(fd: RawFd, byte: i64) {
    ofd_setlk(fd, libc::F_UNLCK, byte);
}

/// The lock type another description holds on `byte`, or `F_UNLCK`. Takes nothing.
pub(crate) fn lock_held(fd: RawFd, byte: i64) -> libc::c_int {
    let mut fl = flock_at(libc::F_WRLCK, byte);
    // SAFETY: fl is a valid flock the kernel fills in.
    let r = unsafe { libc::fcntl(fd, libc::F_OFD_GETLK, &raw mut fl) };
    if r == 0 {
        libc::c_int::from(fl.l_type)
    } else {
        libc::F_UNLCK
    }
}

fn cname(name: &str) -> Result<CString> {
    CString::new(name).map_err(|_| Error::InvalidArgument(format!("name has a NUL byte: {name}")))
}

pub(crate) fn shm_open(name: &str, flags: libc::c_int) -> io::Result<OwnedFd> {
    let c = cname(name).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e.to_string()))?;
    // SAFETY: c is a valid C string.
    let fd = unsafe { libc::shm_open(c.as_ptr(), flags | libc::O_CLOEXEC, 0o600) };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: shm_open returned a fresh descriptor this process owns.
    Ok(unsafe { OwnedFd::from_raw_fd(fd) })
}

pub(crate) fn shm_unlink(name: &str) {
    if let Ok(c) = CString::new(name) {
        // SAFETY: c is a valid C string.
        unsafe { libc::shm_unlink(c.as_ptr()) };
    }
}

pub(crate) struct Stat {
    pub size: usize,
    pub dev: u64,
    pub ino: u64,
    pub uid: u32,
}

impl Stat {
    /// Another user's object may change size or contents under a mapping of it.
    pub(crate) fn ours(&self) -> bool {
        self.uid == euid()
    }
}

#[cfg(not(test))]
pub(crate) fn euid() -> u32 {
    // SAFETY: no preconditions.
    unsafe { libc::geteuid() }
}

#[cfg(test)]
thread_local! {
    /// Makes this thread's objects another user's, which a test cannot otherwise create.
    pub(crate) static AS_ANOTHER_USER: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

#[cfg(test)]
pub(crate) fn euid() -> u32 {
    // SAFETY: no preconditions.
    unsafe { libc::geteuid() }.wrapping_add(u32::from(AS_ANOTHER_USER.get()))
}

pub(crate) fn fstat(fd: RawFd) -> io::Result<Stat> {
    // SAFETY: stat is plain integers; all-zero is a valid value.
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    // SAFETY: st is a valid out-parameter.
    if unsafe { libc::fstat(fd, &raw mut st) } != 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(Stat {
        size: usize::try_from(st.st_size).unwrap_or(0),
        dev: st.st_dev,
        ino: st.st_ino,
        uid: st.st_uid,
    })
}

/// Sizes the file and reserves its pages, so a full `/dev/shm` fails here with ENOSPC instead of
/// raising SIGBUS on the first store into a page that cannot be backed.
pub(crate) fn allocate(fd: RawFd, bytes: usize) -> io::Result<()> {
    // tmpfs gives up on any pending signal, a stop and continue included, and frees what that call
    // reserved. In steps, a signal costs one step; over the whole range, steady signals (a profiler,
    // a timer) would restart it forever.
    const STEP: usize = 1 << 20;
    let off = |n: usize| {
        libc::off_t::try_from(n).map_err(|_| io::Error::from(io::ErrorKind::InvalidInput))
    };
    let mut done = 0;
    while done < bytes {
        let n = (bytes - done).min(STEP);
        // SAFETY: plain syscall on a descriptor we hold.
        if unsafe { libc::fallocate(fd, 0, off(done)?, off(n)?) } == 0 {
            done += n;
            continue;
        }
        let e = io::Error::last_os_error();
        if e.raw_os_error() != Some(libc::EINTR) {
            return Err(e);
        }
    }
    Ok(())
}

fn mmap(fd: RawFd, bytes: usize, prot: libc::c_int) -> io::Result<NonNull<u8>> {
    // SAFETY: a fresh shared mapping; no existing memory is affected.
    let p = unsafe { libc::mmap(ptr::null_mut(), bytes, prot, libc::MAP_SHARED, fd, 0) };
    if p == libc::MAP_FAILED {
        return Err(io::Error::last_os_error());
    }
    NonNull::new(p.cast()).ok_or_else(|| io::Error::from(io::ErrorKind::Other))
}

/// Maps through a description of its own. A mapping keeps its file open, so mapping the lock fd
/// would let a forked child's copy of the mapping hold the lock after the parent dies.
pub(crate) fn map_shared(fd: RawFd, bytes: usize) -> io::Result<NonNull<u8>> {
    let own = OpenOptions::new()
        .read(true)
        .write(true)
        .open(format!("/proc/self/fd/{fd}"))?;
    mmap(own.as_raw_fd(), bytes, libc::PROT_READ | libc::PROT_WRITE)
}

/// Debug builds only: `panic = "abort"` leaves a core as the one record of a crash, and the default
/// `coredump_filter` (0x33) leaves out shared file mappings, the segment headers among them. Bit 3
/// puts them in; the payloads opt out with `MADV_DONTDUMP`.
#[cfg(debug_assertions)]
pub(crate) fn dump_shared_mappings() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        const FILE_SHARED: u32 = 1 << 3;
        let path = "/proc/self/coredump_filter";
        let Some(now) = std::fs::read_to_string(path)
            .ok()
            .and_then(|s| u32::from_str_radix(s.trim(), 16).ok())
        else {
            return;
        };
        if now & FILE_SHARED == 0 {
            // The kernel parses with base 0, so hex needs its 0x.
            let _ = std::fs::write(path, format!("{:#x}", now | FILE_SHARED));
        }
    });
}

pub(crate) fn map_readonly(fd: RawFd, bytes: usize) -> io::Result<NonNull<u8>> {
    mmap(fd, bytes, libc::PROT_READ)
}

/// A second description of the same file, for a lock that must not share the first one's.
pub(crate) fn reopen(fd: RawFd) -> io::Result<OwnedFd> {
    let f = OpenOptions::new()
        .read(true)
        .write(true)
        .open(format!("/proc/self/fd/{fd}"))?;
    Ok(OwnedFd::from(f))
}

/// # Safety
/// `base..base+bytes` must be a mapping this process made and no reference into it may outlive
/// the call.
pub(crate) unsafe fn munmap(base: NonNull<u8>, bytes: usize) {
    // SAFETY: guaranteed by the caller.
    unsafe { libc::munmap(base.as_ptr().cast(), bytes) };
}

pub(crate) fn monotonic_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: ts is a valid out-parameter.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &raw mut ts) };
    u64::try_from(ts.tv_sec).unwrap_or(0) * 1_000_000_000 + u64::try_from(ts.tv_nsec).unwrap_or(0)
}

/// How a futex wait ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Woke {
    /// By a wake on the word at this index of the set (0 for a single word).
    Word(usize),
    Timeout,
    /// A word was not its expected value on entry (`EAGAIN`).
    Already,
}

/// Plain (not private) futex ops: the word is in shared memory. `timeout_ns` < 0 waits forever.
/// False only on a timeout. A signal does not end a wait.
pub(crate) fn futex_wait(word: &AtomicU32, expected: u32, timeout_ns: i64) -> bool {
    futex_wait_op(word, libc::FUTEX_WAIT, expected, timeout_ns) != Woke::Timeout
}

/// A word only this process wakes, with `futex_wake_private`.
pub(crate) fn futex_wait_private(word: &AtomicU32, expected: u32, timeout_ns: i64) -> Woke {
    futex_wait_op(
        word,
        libc::FUTEX_WAIT | libc::FUTEX_PRIVATE_FLAG,
        expected,
        timeout_ns,
    )
}

fn futex_wait_op(word: &AtomicU32, op: libc::c_int, expected: u32, timeout_ns: i64) -> Woke {
    // A signal handler installed without SA_RESTART ends a FUTEX_WAIT with EINTR, and SA_RESTART
    // does not restart one with a timeout: the wait goes on for what is left.
    let start = (timeout_ns > 0).then(monotonic_ns);
    let mut left = timeout_ns;
    loop {
        let ts = libc::timespec {
            tv_sec: left / 1_000_000_000,
            tv_nsec: left % 1_000_000_000,
        };
        let pts: *const libc::timespec = if left < 0 { ptr::null() } else { &raw const ts };
        // SAFETY: word is a live u32-sized atomic; pts is null or points at ts.
        let r = unsafe {
            libc::syscall(
                libc::SYS_futex,
                word.as_ptr(),
                op,
                expected,
                pts,
                ptr::null::<u32>(),
                0,
            )
        };
        if r == 0 {
            return Woke::Word(0);
        }
        match io::Error::last_os_error().raw_os_error() {
            Some(libc::ETIMEDOUT) => return Woke::Timeout,
            Some(libc::EAGAIN) => return Woke::Already,
            Some(libc::EINTR) => {
                if let Some(start) = start {
                    let spent = i64::try_from(monotonic_ns() - start).unwrap_or(i64::MAX);
                    left = timeout_ns.saturating_sub(spent);
                    if left <= 0 {
                        return Woke::Timeout;
                    }
                }
            }
            e => panic!("futex wait: errno {e:?}"),
        }
    }
}

pub(crate) fn futex_wake(word: &AtomicU32) {
    // SAFETY: word is a live u32-sized atomic.
    unsafe {
        libc::syscall(
            libc::SYS_futex,
            word.as_ptr(),
            libc::FUTEX_WAKE,
            i32::MAX,
            ptr::null::<libc::timespec>(),
            ptr::null::<u32>(),
            0,
        )
    };
}

/// One word of a `futex_waitv` set, as the kernel's `struct futex_waitv` lays it out.
#[repr(C)]
pub(crate) struct Waitv {
    val: u64,
    uaddr: u64,
    flags: u32,
    reserved: u32,
}

const FUTEX2_SIZE_U32: u32 = 0x02;
const FUTEX2_PRIVATE: u32 = 128;
/// The kernel's limit on one set.
pub(crate) const WAITV_MAX: usize = 128;

impl Waitv {
    /// A word in shared memory, woken from another process.
    pub(crate) fn shared(word: &AtomicU32, expected: u32) -> Waitv {
        Waitv {
            val: u64::from(expected),
            uaddr: word.as_ptr() as u64,
            flags: FUTEX2_SIZE_U32,
            reserved: 0,
        }
    }

    /// A word only this process wakes.
    pub(crate) fn private(word: &AtomicU32, expected: u32) -> Waitv {
        Waitv {
            flags: FUTEX2_SIZE_U32 | FUTEX2_PRIVATE,
            ..Waitv::shared(word, expected)
        }
    }
}

/// Sleeps until a word is woken or is no longer its expected value. `timeout_ns` < 0 waits
/// forever.
pub(crate) fn futex_waitv(set: &[Waitv], timeout_ns: i64) -> io::Result<Woke> {
    let deadline = u64::try_from(timeout_ns)
        .ok()
        .map(|t| monotonic_ns().saturating_add(t));
    let ts = deadline.map(|d| libc::timespec {
        tv_sec: libc::time_t::try_from(d / 1_000_000_000).unwrap_or(libc::time_t::MAX),
        tv_nsec: libc::c_long::try_from(d % 1_000_000_000).unwrap_or(0),
    });
    let pts = ts.as_ref().map_or(ptr::null(), ptr::from_ref);
    loop {
        // SAFETY: set is a live slice of kernel-layout entries; pts is null or points at ts.
        let r = unsafe {
            libc::syscall(
                libc::SYS_futex_waitv,
                set.as_ptr(),
                set.len(),
                0,
                pts,
                libc::CLOCK_MONOTONIC,
            )
        };
        if let Ok(i) = usize::try_from(r) {
            return Ok(Woke::Word(i));
        }
        let e = io::Error::last_os_error();
        // The deadline is absolute, so a retry after EINTR waits only for what is left.
        match e.raw_os_error() {
            Some(libc::EINTR) => {}
            Some(libc::EAGAIN) => return Ok(Woke::Already),
            Some(libc::ETIMEDOUT) => return Ok(Woke::Timeout),
            _ => return Err(e),
        }
    }
}

pub(crate) fn futex_wake_private(word: &AtomicU32) {
    // SAFETY: word is a live u32-sized atomic.
    unsafe {
        libc::syscall(
            libc::SYS_futex,
            word.as_ptr(),
            libc::FUTEX_WAKE | libc::FUTEX_PRIVATE_FLAG,
            i32::MAX,
            ptr::null::<libc::timespec>(),
            ptr::null::<u32>(),
            0,
        )
    };
}

/// Spawns a helper thread with every signal blocked from its first instruction: a signal sent to
/// the process then goes to one of the caller's threads, never to a helper waiting in a syscall.
pub(crate) fn spawn_helper<T: Send + 'static>(
    builder: std::thread::Builder,
    f: impl FnOnce() -> T + Send + 'static,
) -> io::Result<std::thread::JoinHandle<T>> {
    // SAFETY: sigset_t is plain data, filled by sigfillset before use.
    let mut all: libc::sigset_t = unsafe { std::mem::zeroed() };
    // SAFETY: as above; written by pthread_sigmask.
    let mut old: libc::sigset_t = unsafe { std::mem::zeroed() };
    // SAFETY: a valid sigset_t.
    unsafe { libc::sigfillset(&raw mut all) };
    // SAFETY: valid sets; only this thread's mask changes, and it is restored below.
    unsafe { libc::pthread_sigmask(libc::SIG_SETMASK, &raw const all, &raw mut old) };
    let spawned = builder.spawn(f);
    // SAFETY: the mask saved above.
    unsafe { libc::pthread_sigmask(libc::SIG_SETMASK, &raw const old, ptr::null_mut()) };
    spawned
}

type Retired = Box<dyn Send>;

/// The thread `retire` hands things to, and the fork generation it runs in.
static REAPER: std::sync::OnceLock<Option<(std::sync::mpsc::Sender<Retired>, u32)>> =
    std::sync::OnceLock::new();

/// On a cold path, before an executor thread may `retire` anything. A housekeeping thread nobody
/// waits for, so it drops to `SCHED_OTHER` at nice 0 whatever its creator ran at: an inherited RT
/// policy would let page freeing preempt the RT threads it exists to spare.
pub(crate) fn start_reaper() {
    REAPER.get_or_init(|| {
        let (tx, rx) = std::sync::mpsc::channel::<Retired>();
        let builder = std::thread::Builder::new().name("flux-reaper".into());
        let reap = move || {
            normal_priority();
            rx.into_iter().for_each(drop);
        };
        spawn_helper(builder, reap).ok()?;
        Some((tx, fork_gen()))
    });
}

/// Leaving an RT policy is always allowed. Raising a nice above 0 back to 0 needs `RLIMIT_NICE`,
/// and without it the thread keeps the nicer value.
fn normal_priority() {
    let param = libc::sched_param { sched_priority: 0 };
    // SAFETY: no preconditions.
    let me = unsafe { libc::pthread_self() };
    // SAFETY: a valid param for the calling thread.
    unsafe { libc::pthread_setschedparam(me, libc::SCHED_OTHER, &raw const param) };
    // SAFETY: who 0 is the calling thread on Linux.
    unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, 0) };
}

/// Drops `x` on the reaper thread, or here if none runs in this process. The last mapping of a
/// segment whose publisher is gone frees all its pages on the thread that lets it go: 7 ms for
/// 128 MiB, which an executor thread must not pay on a re-attach.
pub(crate) fn retire<T: Send + 'static>(x: T) {
    if let Some(Some((tx, born))) = REAPER.get()
        && *born == fork_gen()
    {
        let _ = tx.send(Box::new(x));
        return;
    }
    drop(x);
}

pub(crate) fn getpid() -> libc::pid_t {
    // SAFETY: no preconditions.
    unsafe { libc::getpid() }
}

thread_local! {
    /// This thread's id, the owner a PI futex word records. 0 until asked.
    static TID: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
}

fn tid() -> u32 {
    TID.with(|t| {
        if t.get() == 0 {
            // SAFETY: no preconditions.
            t.set(u32::try_from(unsafe { libc::gettid() }).unwrap_or(0));
        }
        t.get()
    })
}

/// A mutex with priority inheritance: a thread that waits for it lends its priority to the holder,
/// which `std::sync::Mutex` and a default pthread mutex never do. The word is a PI futex: 0, or the
/// holder's thread id, plus `FUTEX_WAITERS` once someone sleeps in the kernel. Uncontended, lock
/// and unlock are one compare-exchange each.
pub(crate) struct PiMutex<T> {
    word: AtomicU32,
    data: UnsafeCell<T>,
}

// SAFETY: data is reached only through a guard, which holds the lock.
unsafe impl<T: Send> Sync for PiMutex<T> {}

/// Not `Send`: the unlock must come from the thread the kernel recorded as the owner.
pub(crate) struct PiGuard<'a, T>(&'a PiMutex<T>, std::marker::PhantomData<*const ()>);

impl<T> PiMutex<T> {
    pub(crate) const fn new(data: T) -> PiMutex<T> {
        PiMutex {
            word: AtomicU32::new(0),
            data: UnsafeCell::new(data),
        }
    }

    pub(crate) fn lock(&self) -> PiGuard<'_, T> {
        self.lock_raw();
        PiGuard(self, std::marker::PhantomData)
    }

    fn lock_raw(&self) {
        if self
            .word
            .compare_exchange(0, tid(), Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            self.futex_op(libc::FUTEX_LOCK_PI);
        }
    }

    /// Miri has no PI futex. Contended, it retries the same CAS, so the waiters bit is never set
    /// and the uncontended unlock always applies.
    #[cfg(miri)]
    fn futex_op(&self, op: libc::c_int) {
        if op == libc::FUTEX_LOCK_PI {
            while self
                .word
                .compare_exchange(0, tid(), Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {
                std::thread::yield_now();
            }
        }
    }

    fn unlock_raw(&self) {
        if self
            .word
            .compare_exchange(tid(), 0, Ordering::Release, Ordering::Relaxed)
            .is_err()
        {
            self.futex_op(libc::FUTEX_UNLOCK_PI);
        }
    }

    /// The kernel queues the caller by priority and boosts the holder, or hands the lock to the
    /// highest waiter.
    #[cfg(not(miri))]
    fn futex_op(&self, op: libc::c_int) {
        loop {
            // SAFETY: word is a live u32-sized atomic; the PI ops take no timeout here.
            let r = unsafe {
                libc::syscall(
                    libc::SYS_futex,
                    self.word.as_ptr(),
                    op | libc::FUTEX_PRIVATE_FLAG,
                    0,
                    ptr::null::<libc::timespec>(),
                    ptr::null::<u32>(),
                    0,
                )
            };
            match (r, io::Error::last_os_error().raw_os_error()) {
                (0, _) => return,
                // The holder is exiting: ask again.
                (_, Some(libc::EAGAIN | libc::EINTR)) => {}
                (_, e) => panic!("PI futex op {op}: errno {e:?}"),
            }
        }
    }

    /// In a forked child, for a lock `fork_prepare` took: the child's one thread has another id,
    /// and no other thread came along to wait for it.
    fn reset_in_child(&self) {
        self.word.store(0, Ordering::Relaxed);
    }

    #[cfg(test)]
    pub(crate) fn word(&self) -> u32 {
        self.word.load(Ordering::Relaxed)
    }
}

impl<T> std::ops::Deref for PiGuard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: the guard holds the lock.
        unsafe { &*self.0.data.get() }
    }
}

impl<T> std::ops::DerefMut for PiGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: the guard holds the lock.
        unsafe { &mut *self.0.data.get() }
    }
}

impl<T> Drop for PiGuard<'_, T> {
    fn drop(&mut self) {
        self.0.unlock_raw();
    }
}

// Every fd holding a liveness lock. A forked child shares each one's description and with it the
// lock, so it would keep a dead parent looking alive; the child closes them at fork.
static LIVE: PiMutex<Vec<RawFd>> = PiMutex::new(Vec::new());

// Guards process-wide registries (the subscriber leases). fork() takes it first, so a child never
// starts with it held by a thread that did not come along. Taken before LIVE, never after. A
// re-attach takes it on an executor thread.
static REGISTRY: PiMutex<()> = PiMutex::new(());
static ATFORK: Once = Once::new();
static FORK_GEN: AtomicU32 = AtomicU32::new(0);

/// Bumped in every forked child, so state made before a fork knows it is the parent's copy.
pub(crate) fn fork_gen() -> u32 {
    FORK_GEN.load(Ordering::Relaxed)
}

pub(crate) fn watch_fork() {
    ATFORK.call_once(|| {
        // SAFETY: the handlers are extern "C" fns that live for the program.
        unsafe { libc::pthread_atfork(Some(fork_prepare), Some(fork_parent), Some(fork_child)) };
    });
}

fn with_live_fds(f: impl FnOnce(&mut Vec<RawFd>)) {
    f(&mut LIVE.lock());
}

/// Runs `f` with no `fork()` in between. `f` may open and close `LiveFd`s.
pub(crate) fn fork_safe<R>(f: impl FnOnce() -> R) -> R {
    watch_fork();
    let _held = REGISTRY.lock();
    f()
}

extern "C" fn fork_prepare() {
    REGISTRY.lock_raw();
    LIVE.lock_raw();
}

extern "C" fn fork_parent() {
    LIVE.unlock_raw();
    REGISTRY.unlock_raw();
}

extern "C" fn fork_child() {
    TID.with(|t| t.set(0));
    FORK_GEN.fetch_add(1, Ordering::Relaxed);
    // SAFETY: fork_prepare took LIVE, and the child is the only thread.
    let fds = unsafe { &mut *LIVE.data.get() };
    for &fd in fds.iter() {
        // SAFETY: the child's copy of a descriptor the parent registered.
        unsafe { libc::close(fd) };
    }
    fds.clear();
    LIVE.reset_in_child();
    REGISTRY.reset_in_child();
}

/// An fd whose OFD locks mean "this process is alive". Closed only by the process that took it.
pub(crate) struct LiveFd {
    fd: RawFd,
    pid: libc::pid_t,
}

impl LiveFd {
    /// Opens under the fork lock. A lock belongs to the description, so a child forked between
    /// open and registration would share every lock taken on it later.
    pub(crate) fn open(open: impl FnOnce() -> io::Result<OwnedFd>) -> io::Result<LiveFd> {
        watch_fork();
        let mut opened = Err(io::Error::from(io::ErrorKind::Other));
        with_live_fds(|v| {
            opened = open().map(|fd| {
                let fd = std::os::fd::IntoRawFd::into_raw_fd(fd);
                v.push(fd);
                fd
            });
        });
        Ok(LiveFd {
            fd: opened?,
            pid: getpid(),
        })
    }

    pub(crate) fn raw(&self) -> RawFd {
        self.fd
    }

    pub(crate) fn owned_here(&self) -> bool {
        self.pid == getpid()
    }
}

impl Drop for LiveFd {
    fn drop(&mut self) {
        if !self.owned_here() {
            return;
        }
        // Closed under the lock, so a fork cannot slip between the two and leave the child a copy.
        with_live_fds(|v| {
            v.retain(|&f| f != self.fd);
            // SAFETY: this process opened fd and nothing else closes it.
            unsafe { libc::close(self.fd) };
        });
    }
}

#[cfg(test)]
mod tests {
    fn blocked(signal: libc::c_int) -> bool {
        // SAFETY: sigset_t is plain data, written by pthread_sigmask.
        let mut now: libc::sigset_t = unsafe { std::mem::zeroed() };
        // SAFETY: a null set only reads the mask.
        unsafe { libc::pthread_sigmask(libc::SIG_BLOCK, std::ptr::null(), &raw mut now) };
        // SAFETY: a valid set.
        unsafe { libc::sigismember(&raw const now, signal) == 1 }
    }

    #[test]
    fn a_helper_thread_takes_no_signal_and_its_spawner_keeps_its_own() {
        let before = blocked(libc::SIGINT);
        let helper = super::spawn_helper(std::thread::Builder::new(), || {
            [libc::SIGINT, libc::SIGTERM, libc::SIGCHLD, libc::SIGUSR1].map(blocked)
        })
        .unwrap();
        assert_eq!(helper.join().unwrap(), [true; 4]);
        assert_eq!(blocked(libc::SIGINT), before);
    }

    fn fifo(priority: libc::c_int) -> bool {
        let param = libc::sched_param {
            sched_priority: priority,
        };
        // SAFETY: no preconditions.
        let me = unsafe { libc::pthread_self() };
        // SAFETY: a valid param for the calling thread.
        unsafe { libc::pthread_setschedparam(me, libc::SCHED_FIFO, &raw const param) == 0 }
    }

    /// The `priority` field of a thread's stat: -1 - its effective real-time priority.
    fn effective_priority(tid: u32) -> i64 {
        let stat = std::fs::read_to_string(format!("/proc/self/task/{tid}/stat")).unwrap();
        let after_comm = &stat[stat.rfind(')').unwrap() + 2..];
        after_comm.split(' ').nth(15).unwrap().parse().unwrap()
    }

    /// Two threads on one lock. The holder's id and the kernel's waiters bit in the word show
    /// the lock is the PI protocol, so the kernel boosts the holder. None if `fifo` is refused.
    fn contend(fifo_at: Option<(i32, i32)>) -> Option<(u32, u32, i64)> {
        use std::sync::{Arc, Barrier, mpsc};
        let m = Arc::new(super::PiMutex::new(()));
        let release = Arc::new(Barrier::new(2));
        let (held, holding) = mpsc::channel();
        let holder = {
            let (m, release) = (Arc::clone(&m), Arc::clone(&release));
            std::thread::spawn(move || {
                if fifo_at.is_some_and(|(low, _)| !fifo(low)) {
                    held.send(false).unwrap();
                    return 0;
                }
                let g = m.lock();
                held.send(true).unwrap();
                release.wait();
                drop(g);
                super::tid()
            })
        };
        if !holding.recv().unwrap() {
            holder.join().unwrap();
            return None;
        }
        let waiter = {
            let m = Arc::clone(&m);
            std::thread::spawn(move || {
                if let Some((_, high)) = fifo_at {
                    assert!(fifo(high));
                }
                drop(m.lock());
            })
        };
        let start = std::time::Instant::now();
        while m.word() & libc::FUTEX_WAITERS == 0 {
            assert!(
                start.elapsed().as_secs() < 5,
                "the waiter never slept in the kernel"
            );
            std::thread::yield_now();
        }
        let word = m.word();
        let priority = effective_priority(word & libc::FUTEX_TID_MASK);
        release.wait();
        let tid = holder.join().unwrap();
        waiter.join().unwrap();
        assert_eq!(m.word(), 0);
        Some((word & libc::FUTEX_TID_MASK, tid, priority))
    }

    #[test]
    fn a_contended_lock_is_a_kernel_pi_futex_owned_by_its_holder() {
        let (owner, holder, _) = contend(None).unwrap();
        assert_eq!(owner, holder);
    }

    /// Needs `SCHED_FIFO`, which `RLIMIT_RTPRIO` or `CAP_SYS_NICE` grants.
    #[test]
    fn a_waiting_fifo_thread_lends_its_priority_to_the_holder() {
        let Some((_, _, priority)) = contend(Some((1, 3))) else {
            eprintln!("skipped: SCHED_FIFO is not permitted here");
            return;
        };
        assert_eq!(priority, -4, "the FIFO 1 holder was not boosted to FIFO 3");
    }

    /// The reaper's policy, started from a `SCHED_FIFO` thread where that is permitted and from a
    /// `SCHED_BATCH` one otherwise. Runs alone in a child: the reaper is one per process.
    #[test]
    fn the_reaper_runs_at_normal_priority_whatever_starts_it() {
        const ENV: &str = "FLUX_TEST_REAPER";
        let name = "sys::tests::the_reaper_runs_at_normal_priority_whatever_starts_it";
        if std::env::var_os(ENV).is_none() {
            let out = std::process::Command::new(std::env::current_exe().unwrap())
                .args([name, "--exact", "--test-threads=1", "--nocapture"])
                .env(ENV, "1")
                .output()
                .unwrap();
            let stdout = String::from_utf8_lossy(&out.stdout);
            assert!(
                out.status.success() && stdout.contains("1 passed"),
                "{stdout}{}",
                String::from_utf8_lossy(&out.stderr)
            );
            return;
        }
        let started_from = std::thread::spawn(|| {
            let policy = if fifo(1) {
                libc::SCHED_FIFO
            } else {
                println!("SCHED_FIFO is not permitted here; starting from SCHED_BATCH");
                let param = libc::sched_param { sched_priority: 0 };
                // SAFETY: no preconditions.
                let me = unsafe { libc::pthread_self() };
                // SAFETY: a valid param; SCHED_BATCH needs no privilege.
                let r =
                    unsafe { libc::pthread_setschedparam(me, libc::SCHED_BATCH, &raw const param) };
                assert_eq!(r, 0);
                libc::SCHED_BATCH
            };
            super::start_reaper();
            policy
        })
        .join()
        .unwrap();
        let start = std::time::Instant::now();
        let reaper = loop {
            let found = std::fs::read_dir("/proc/self/task").unwrap().find_map(|t| {
                let t = t.unwrap().path();
                let comm = std::fs::read_to_string(t.join("comm")).ok()?;
                (comm.trim_end() == "flux-reaper").then(|| t.file_name().unwrap().to_owned())
            });
            if let Some(tid) = found {
                break tid.to_string_lossy().parse::<libc::pid_t>().unwrap();
            }
            assert!(start.elapsed().as_secs() < 5, "no reaper thread");
            std::thread::yield_now();
        };
        // The reaper sets its policy as its first act.
        let start = std::time::Instant::now();
        loop {
            // SAFETY: plain syscall on a thread of this process.
            let policy = unsafe { libc::sched_getscheduler(reaper) };
            if policy == libc::SCHED_OTHER {
                break;
            }
            assert!(
                start.elapsed().as_secs() < 5,
                "started from policy {started_from}, the reaper runs at {policy}"
            );
            std::thread::yield_now();
        }
    }
}
