//! A steady stream of signals whose handlers have no `SA_RESTART`, as an application's handler or a
//! profiler's timer sends them. A blocking call that gives up on EINTR fails or ends early here.
//! The interval timer is process-wide, so each test runs alone in a child of the test binary.

use std::process::Command;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

const STORM_ENV: &str = "FLUX2_TEST_STORM";
const PERIOD_US: i64 = 200;

static CAUGHT: AtomicU32 = AtomicU32::new(0);

extern "C" fn caught(_: libc::c_int) {
    CAUGHT.fetch_add(1, Ordering::Relaxed);
}

/// True in a child running `test` alone under the storm, which then aims at the calling thread.
/// Elsewhere it runs that child on both executor waits and returns false.
pub(crate) fn in_storm(test: &str) -> bool {
    if std::env::var_os(STORM_ENV).is_some() {
        start();
        return true;
    }
    // Once on each executor wait: the merged one, then the parker threads.
    for parked in [false, true] {
        let mut child = Command::new(std::env::current_exe().unwrap());
        child
            .args([test, "--exact", "--test-threads=1"])
            .env(STORM_ENV, "1");
        if parked {
            child.env("FLUX2_DISABLE_FUTEX_WAITV", "1");
        }
        let out = child.output().unwrap();
        let stdout = String::from_utf8_lossy(&out.stdout);
        assert!(
            out.status.success() && stdout.contains("1 passed"),
            "parked {parked}: {stdout}{}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
    false
}

/// How many signals the handlers ran for so far, to show the storm reached the code under test.
pub(crate) fn caught_so_far() -> u32 {
    CAUGHT.load(Ordering::Relaxed)
}

fn start() {
    for sig in [libc::SIGUSR1, libc::SIGALRM] {
        // SAFETY: sigaction is plain data; all zeros is a valid value.
        let mut sa: libc::sigaction = unsafe { std::mem::zeroed() };
        sa.sa_sigaction = caught as *const () as libc::sighandler_t;
        // SAFETY: an async-signal-safe handler, installed without SA_RESTART.
        let r = unsafe { libc::sigaction(sig, &raw const sa, std::ptr::null_mut()) };
        assert_eq!(r, 0);
    }
    let every = libc::timeval {
        tv_sec: 0,
        tv_usec: PERIOD_US,
    };
    let timer = libc::itimerval {
        it_interval: every,
        it_value: every,
    };
    // SAFETY: a valid itimerval; SIGALRM goes to any thread that does not block it.
    let r = unsafe { libc::setitimer(libc::ITIMER_REAL, &raw const timer, std::ptr::null_mut()) };
    assert_eq!(r, 0);
    // SAFETY: names the calling thread, which runs the test until the child exits.
    let target = unsafe { libc::pthread_self() };
    let period = Duration::from_micros(PERIOD_US.unsigned_abs());
    std::thread::spawn(move || {
        loop {
            // SAFETY: the target outlives this thread, which ends with the process.
            unsafe { libc::pthread_kill(target, libc::SIGUSR1) };
            std::thread::sleep(period);
        }
    });
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::{caught_so_far, in_storm};
    use crate::channel::{Channel, PublisherWatch};
    use crate::discovery::signpost_name;
    use crate::error::Result;
    use crate::executor::{Executor, Source};
    use crate::layout::{DEFAULT_MAX_SUBSCRIBERS, NO_SCHEMA};
    use crate::qos::MemoryPolicy;

    const WAIT: Duration = Duration::from_millis(100);
    const WAIT_NS: i64 = 100_000_000;

    struct Sub(Channel);

    impl Source for Sub {
        fn attach(&mut self) -> Result<bool> {
            Ok(true)
        }
        fn deliver_one(&mut self) -> Result<bool> {
            Ok(self.0.take().is_some())
        }
        fn channel(&self) -> Option<&Channel> {
            Some(&self.0)
        }
    }

    fn lasts_its_timeout(what: &str, wait: impl FnOnce()) {
        let before = caught_so_far();
        let t0 = Instant::now();
        wait();
        let took = t0.elapsed();
        assert!(took >= WAIT, "{what} ended after {took:?}");
        assert!(caught_so_far() > before + 10, "{what}: no storm");
    }

    #[test]
    fn signals_neither_fail_a_create_nor_end_a_wait_early() {
        if !in_storm("storm::tests::signals_neither_fail_a_create_nor_end_a_wait_early") {
            return;
        }
        let name = signpost_name(&format!("/s{}_storm", std::process::id()), NO_SCHEMA).unwrap();
        let mem = MemoryPolicy::default();
        let before = caught_so_far();
        let mut publisher =
            Channel::create(&name, 1 << 20, 64, DEFAULT_MAX_SUBSCRIBERS, NO_SCHEMA, mem).unwrap();
        assert!(caught_so_far() > before, "the create saw no signal");
        let sub = Channel::open(&name, NO_SCHEMA, mem).unwrap();
        lasts_its_timeout("Channel::wait", || {
            assert!(!sub.wait(sub.wake_seq(), WAIT_NS));
        });
        let nobody = signpost_name(&format!("/s{}_storm_none", std::process::id()), NO_SCHEMA);
        let watch = PublisherWatch::open(&nobody.unwrap()).unwrap();
        watch.arm();
        lasts_its_timeout("PublisherWatch::wait", || assert!(!watch.wait(WAIT_NS)));
        let mut ex = Executor::new().unwrap();
        ex.add(Sub(sub), 0).unwrap();
        ex.dispatch().unwrap();
        lasts_its_timeout("Executor::wait_for_work", || {
            ex.wait_for_work(WAIT_NS).unwrap();
        });
        assert_eq!(publisher.publish(b"x"), crate::Published::Ok);
        let t0 = Instant::now();
        ex.wait_for_work(-1).unwrap();
        assert!(t0.elapsed() < Duration::from_secs(1));
        assert_eq!(ex.dispatch().unwrap(), 1);
    }
}
