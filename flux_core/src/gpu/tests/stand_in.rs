//! The commit, release, bound, deferral, sticky error, fork and context rules on the stand-in
//! driver (`gpu::fake`), where each test decides when the work on a stream finishes or fails. They
//! need no GPU: each runs in a child process whose driver is the stand-in.

use std::os::fd::AsRawFd;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use super::channel::{claimable, publisher, subscriber, topic};
use crate::gpu::{HostRegistration, Stream, fake, ledger, wedged};
use crate::layout::{DEFAULT_MAX_SUBSCRIBERS, NO_SCHEMA};
use crate::qos::{MemoryPolicy, QoS};
use crate::{Channel, Error, Published, ReleaseMode, sys, trace};

/// Not one of the sticky errors: one launch failed and the context is fine.
const LAUNCH_OUT_OF_RESOURCES: i32 = 701;
const ILLEGAL_ADDRESS: i32 = 700;

fn stand_in(test: &str) -> bool {
    super::child(&format!("gpu::tests::stand_in::{test}"), fake::ENV)
}

/// Publishes `n` frames on a fresh topic, for a subscriber on `stream`.
fn published(n: usize, stream: &Stream) -> (Channel, Channel) {
    let name = topic("si");
    let mut p = publisher(&name, Stream::create().unwrap());
    for _ in 0..n {
        assert_eq!(p.publish(b"frame"), Published::Ok);
    }
    let s = subscriber(&name, stream.clone()).unwrap();
    (p, s)
}

fn returns_within<T: Send + 'static>(
    f: impl FnOnce() -> T + Send + 'static,
    finish: impl FnOnce(),
) -> T {
    let done = Arc::new(AtomicBool::new(false));
    let d = Arc::clone(&done);
    let t = std::thread::spawn(move || {
        let r = f();
        d.store(true, Ordering::Release);
        r
    });
    std::thread::sleep(Duration::from_millis(50));
    assert!(
        !done.load(Ordering::Acquire),
        "returned before the work was done"
    );
    finish();
    t.join().unwrap()
}

#[test]
fn a_commit_publishes_once_the_work_on_its_stream_is_done() {
    if !stand_in("a_commit_publishes_once_the_work_on_its_stream_is_done") {
        return;
    }
    let stream = Stream::create().unwrap();
    let name = topic("commit");
    let mut p = publisher(&name, stream.clone());
    let mut s = subscriber(&name, Stream::default()).unwrap();
    let w = p.loan().unwrap();
    fake::submit(&stream);
    let r = returns_within(
        move || w.commit_len(1),
        || {
            assert!(s.take().is_none(), "published before the work was done");
            fake::finish(&stream);
        },
    );
    assert_eq!(r, Published::Ok);
}

#[test]
fn a_release_gives_its_slot_back_once_the_work_on_its_stream_is_done() {
    if !stand_in("a_release_gives_its_slot_back_once_the_work_on_its_stream_is_done") {
        return;
    }
    let stream = Stream::create().unwrap();
    let (mut p, mut s) = published(4, &stream);
    let v = s.take().unwrap();
    fake::submit(&stream);
    let st = stream.clone();
    returns_within(move || drop(v), || fake::finish(&st));
    assert_eq!(claimable(&mut p), 4);
    assert_eq!(s.fence_failed(), 0);
    drop(s);
    assert_eq!(ledger::leaks(), 0);
}

#[test]
fn a_timed_out_commit_holds_its_claim_until_the_work_is_done() {
    if !stand_in("a_timed_out_commit_holds_its_claim_until_the_work_is_done") {
        return;
    }
    let stream = Stream::create().unwrap();
    let name = topic("ctimeout");
    let mut p = publisher(&name, stream.clone());
    p.set_fence_timeout(10_000_000);
    fake::submit(&stream);
    assert_eq!(p.loan().unwrap().commit(), Published::FenceFailed);
    assert_eq!(p.fence_failed(), 1);
    assert_eq!(claimable(&mut p), 3, "claimed while the work runs");
    fake::finish(&stream);
    assert_eq!(claimable(&mut p), 4, "freed once it is done");
    let mut s = subscriber(&name, Stream::default()).unwrap();
    assert!(s.take().is_none(), "nothing was published");
}

#[test]
fn a_deferred_release_holds_its_borrow_until_the_work_is_done() {
    if !stand_in("a_deferred_release_holds_its_borrow_until_the_work_is_done") {
        return;
    }
    let stream = Stream::create().unwrap();
    let (mut p, mut s) = published(4, &stream);
    s.set_qos(
        QoS::new(4)
            .unwrap()
            .transient_local(4)
            .max_borrow(2)
            .unwrap(),
    )
    .unwrap();
    s.set_release_mode(ReleaseMode::Deferred);
    let v = s.take().unwrap();
    fake::submit(&stream);
    drop(v);
    assert!(s.can_borrow());
    assert_eq!(claimable(&mut p), 3, "held while the device reads it");
    fake::finish(&stream);
    assert!(s.can_borrow());
    assert_eq!(claimable(&mut p), 4, "given back once the device is done");
    assert_eq!(s.fence_failed(), 0);
}

#[test]
fn work_that_never_finishes_keeps_its_slot_and_its_caller_waits_only_the_bound() {
    if !stand_in("work_that_never_finishes_keeps_its_slot_and_its_caller_waits_only_the_bound") {
        return;
    }
    let stream = Stream::create().unwrap();
    let (mut p, mut s) = published(4, &stream);
    s.set_qos(
        QoS::new(4)
            .unwrap()
            .transient_local(4)
            .max_borrow(2)
            .unwrap(),
    )
    .unwrap();
    s.set_fence_timeout(20_000_000);
    let v = s.take().unwrap();
    fake::submit(&stream);
    let t0 = Instant::now();
    drop(v);
    let waited = t0.elapsed();
    assert!(waited >= Duration::from_millis(20) && waited < Duration::from_secs(1));
    assert_eq!(s.fence_failed(), 1);
    assert!(s.can_borrow());
    assert_eq!(claimable(&mut p), 3);
}

#[test]
fn a_failed_launch_fails_its_fence_and_wedges_nothing() {
    if !stand_in("a_failed_launch_fails_its_fence_and_wedges_nothing") {
        return;
    }
    let stream = Stream::create().unwrap();
    let name = topic("launch");
    let mut p = publisher(&name, stream.clone());
    let w = p.loan().unwrap();
    fake::submit(&stream);
    fake::fail(&stream, LAUNCH_OUT_OF_RESOURCES);
    assert_eq!(w.commit_len(1), Published::FenceFailed);
    assert_eq!(wedged(), None);
    let mut other = publisher(&topic("launch2"), Stream::create().unwrap());
    assert_eq!(other.publish(b"x"), Published::Ok);
}

#[test]
fn a_sticky_error_wedges_every_gpu_channel_at_once() {
    if !stand_in("a_sticky_error_wedges_every_gpu_channel_at_once") {
        return;
    }
    let stream = Stream::create().unwrap();
    let (mut p, mut s) = published(4, &stream);
    let q = QoS::new(4)
        .unwrap()
        .transient_local(4)
        .max_borrow(2)
        .unwrap();
    s.set_qos(q).unwrap();
    let held = p.loan().unwrap();
    let mut host = Channel::new(64, 4).unwrap();
    let v = s.take().unwrap();
    fake::submit(&stream);
    trace::drain();
    fake::wedge(ILLEGAL_ADDRESS);
    drop(v);
    assert_eq!(wedged(), Some(ILLEGAL_ADDRESS));
    let ev = trace::drain();
    assert_eq!(
        ev.iter()
            .filter(|(n, a)| *n == "gpu_wedged" && a[..] == [700])
            .count(),
        1,
        "{ev:?}"
    );

    for _ in 0..3 {
        assert!(s.take().is_none());
    }
    assert!(s.take_blocking(-1).is_none());
    assert_eq!((s.refused().fence, s.refused().max_borrow), (4, 0));
    assert_eq!(p.publish(b"x"), Published::GpuWedged);
    assert!(p.loan().is_none());
    assert_eq!(held.commit(), Published::GpuWedged);
    assert!(unsupported(Stream::create()).contains("700"));
    let late = Channel::create_on(
        &topic("late"),
        64,
        4,
        DEFAULT_MAX_SUBSCRIBERS,
        0,
        NO_SCHEMA,
        MemoryPolicy::default(),
        stream.clone(),
        trace::NO_STAMP,
        false,
    );
    assert!(unsupported(late).contains("700"));
    let ev = trace::drain();
    for (name, reason) in [
        ("refused", trace::REFUSED_GPU_WEDGED),
        ("claim", trace::CLAIM_GPU_WEDGED),
        ("loan_end", trace::LOAN_GPU_WEDGED),
    ] {
        assert!(
            ev.iter()
                .any(|(n, a)| *n == name && a.contains(&trace::Word::word(reason))),
            "no {name} for the wedge: {ev:?}"
        );
    }

    assert_eq!(host.publish(b"host"), Published::Ok);
    assert!(host.take().is_some());
}

fn unsupported<T>(r: crate::Result<T>) -> String {
    match r {
        Err(Error::Unsupported(m)) => m,
        Err(e) => panic!("{e}"),
        Ok(_) => panic!("made after the GPU was wedged"),
    }
}

#[test]
fn a_forked_child_makes_no_driver_call() {
    if !stand_in("a_forked_child_makes_no_driver_call") {
        return;
    }
    let stream = Stream::create().unwrap();
    let (p, mut s) = published(2, &stream);
    let v = s.take().unwrap();
    let before = fake::calls();
    // SAFETY: the child only drops values and exits; no other thread holds the stand-in's lock.
    let pid = unsafe { libc::fork() };
    if pid == 0 {
        let refused = matches!(Stream::create(), Err(Error::Unsupported(m)) if m.contains("fork"));
        drop(v);
        drop(s);
        drop(p);
        drop(stream);
        let quiet = fake::calls() == before;
        // SAFETY: ends the child without running the parent's exit handlers.
        unsafe { libc::_exit(i32::from(!(refused && quiet))) };
    }
    let mut status = 0;
    // SAFETY: waits for the child forked above.
    assert_eq!(unsafe { libc::waitpid(pid, &raw mut status, 0) }, pid);
    assert!(libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0);
}

#[test]
fn a_registration_outlives_the_context_current_at_its_making() {
    if !stand_in("a_registration_outlives_the_context_current_at_its_making") {
        return;
    }
    let n = 1 << 16;
    let name = format!("/flux.gputest.si.{}", std::process::id());
    let fd = sys::shm_open(&name, libc::O_CREAT | libc::O_RDWR).unwrap();
    sys::shm_unlink(&name);
    sys::allocate(fd.as_raw_fd(), n).unwrap();
    let base = sys::map_shared(fd.as_raw_fd(), n).unwrap();
    let ctx = fake::ctx_create();
    let reg = HostRegistration::new(base.as_ptr(), n, false, 0).unwrap();
    fake::ctx_destroy(ctx);
    assert_eq!(fake::live().2, 1, "the registration went with the context");
    drop(reg);
    assert_eq!(fake::misuse(), Vec::<String>::new());
    assert_eq!(fake::live().2, 0);
    // SAFETY: mapped above with n bytes; the registration is gone.
    unsafe { sys::munmap(base, n) };
}
