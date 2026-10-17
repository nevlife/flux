//! The `flux` binary against real channels. Every channel here has a key unique to this run, and
//! the checks look for it by name, since the host may list other channels too.

use std::io::{BufRead, BufReader};
use std::process::{Command, Output, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::thread;
use std::time::Duration;

use flux_core::{
    Announcement, Channel, DEFAULT_MAX_SUBSCRIBERS, ManifestEntry, MemoryPolicy, Published,
    flatten_key, signpost_name,
};

const FP: u64 = 0xC11;

fn key(name: &str) -> String {
    static N: AtomicU32 = AtomicU32::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    format!("/cli{}_{n}/{name}", std::process::id())
}

struct Unlink(String);

impl Drop for Unlink {
    fn drop(&mut self) {
        let c = std::ffi::CString::new(self.0.clone()).unwrap();
        // SAFETY: c is a valid C string.
        unsafe { libc::shm_unlink(c.as_ptr()) };
    }
}

fn publisher(key: &str) -> (Channel, Unlink) {
    let name = signpost_name(key, FP).unwrap();
    let ch = Channel::create(
        &name,
        64,
        4,
        DEFAULT_MAX_SUBSCRIBERS,
        FP,
        MemoryPolicy::default(),
    )
    .unwrap();
    (ch, Unlink(name))
}

fn announce(key: &str, label: &str, publisher: bool) -> Announcement {
    Announcement::new(&ManifestEntry {
        signpost: signpost_name(key, FP).unwrap(),
        key: key.into(),
        label: label.into(),
        publisher,
    })
    .unwrap()
}

fn flux(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_flux"))
        .args(args)
        .output()
        .unwrap()
}

fn text(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

fn row(listing: &str, prefix: &str) -> String {
    listing
        .lines()
        .find(|l| l.starts_with(prefix))
        .unwrap_or_default()
        .to_owned()
}

#[test]
fn list_shows_announced_and_unannounced_channels() {
    let (cam_key, lidar_key) = (key("cam"), key("lidar"));
    let (cam_pub, _u1) = publisher(&cam_key);
    let _a = announce(&cam_key, "/driver", true);
    let _s = announce(&cam_key, "/viewer", false);
    let (lidar, _u2) = publisher(&lidar_key);
    let flat = flatten_key(&lidar_key);

    let out = flux(&["topic", "list"]);
    assert!(out.status.success());
    let t = text(&out);
    assert!(row(&t, "CHANNEL").ends_with("SEGMENT  ENDPOINTS"), "{t}");
    let cam = row(&t, &format!("{cam_key} "));
    assert!(cam.ends_with("up       1 pub, 1 sub"), "{t}");
    let lidar_row = row(&t, &format!("{flat} (from name) "));
    assert!(lidar_row.ends_with("up       -"), "{t}");
    drop(lidar);
    drop(cam_pub);
    let t = text(&flux(&["topic", "list"]));
    assert!(row(&t, &format!("{flat} ")).is_empty(), "{t}");
    assert!(row(&t, &format!("{cam_key} ")).contains("down"), "{t}");
}

#[test]
fn info_shows_the_segment_and_the_endpoints() {
    let k = key("cam");
    let (mut cam, _u) = publisher(&k);
    let _a = announce(&k, "/driver", true);
    for _ in 0..3 {
        assert_eq!(cam.publish(b"x"), Published::Ok);
    }
    let out = flux(&["topic", "info", &k]);
    let err = String::from_utf8_lossy(&out.stderr).into_owned();
    assert!(out.status.success(), "{err}");
    let t = text(&out);
    assert!(t.contains(&format!("channel      {k}\n")), "{t}");
    assert!(t.contains("fingerprint  0x0000000000000c11"), "{t}");
    assert!(t.contains("slots        4 x 64 B"), "{t}");
    assert!(t.contains("storage      host"), "{t}");
    assert!(t.contains("published    3 frames on this segment"), "{t}");
    // Fields, not columns: the PID column is as wide as the widest PID.
    let pid = std::process::id().to_string();
    let me = ["pub", pid.as_str(), "/driver"];
    assert!(
        t.lines()
            .any(|l| l.split_whitespace().eq(me.iter().copied())),
        "{t}"
    );
}

/// A label comes from another process; ESC must not reach the terminal.
#[test]
fn info_escapes_control_characters() {
    let k = key("cam");
    let (_cam, _u) = publisher(&k);
    let _a = announce(&k, "/driver\u{1b}[31m", true);
    let t = text(&flux(&["topic", "info", &k]));
    assert!(!t.contains('\u{1b}'), "{t:?}");
    assert!(t.contains("/driver\\u{1b}[31m"), "{t}");
}

#[test]
fn a_window_too_long_or_too_short_is_refused() {
    for w in ["1e300", "1e-300"] {
        assert_eq!(
            flux(&["topic", "hz", "/x", "--window", w]).status.code(),
            Some(2)
        );
    }
}

#[test]
fn a_missing_channel_or_a_bad_invocation_fails() {
    let out = flux(&["topic", "info", &key("none")]);
    assert_eq!(out.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&out.stderr).contains("no channel named"));
    assert_eq!(flux(&["topic", "echo"]).status.code(), Some(2));
    let with_domain = flux(&["topic", "list", "--domain", "0"]);
    assert_eq!(with_domain.status.code(), Some(2));
    assert!(flux(&["--help"]).status.success());
}

#[test]
fn hz_needs_a_publisher() {
    let k = key("cam");
    let (cam, _u) = publisher(&k);
    let _s = announce(&k, "/viewer", false);
    drop(cam);
    let out = flux(&["topic", "hz", &k]);
    assert_eq!(out.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&out.stderr).contains("nothing is publishing"));
}

/// No subscriber exists: the rate is what the publisher committed.
#[test]
fn hz_counts_published_frames_with_no_subscriber() {
    let k = key("cam");
    let (mut cam, _u) = publisher(&k);
    let stop = AtomicBool::new(false);
    // Read everything before asserting, so a failure still stops the publisher thread.
    let lines: Vec<String> = thread::scope(|s| {
        s.spawn(|| {
            while !stop.load(Ordering::Relaxed) {
                let _ = cam.publish(b"x");
                thread::sleep(Duration::from_millis(5));
            }
        });
        let mut child = Command::new(env!("CARGO_BIN_EXE_flux"))
            .args(["topic", "hz", &k, "--window", "0.3"])
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        let out = child.stdout.take().unwrap();
        let lines = BufReader::new(out)
            .lines()
            .take(4)
            .map_while(Result::ok)
            .collect();
        let _ = child.kill();
        let _ = child.wait();
        stop.store(true, Ordering::Relaxed);
        lines
    });
    assert_eq!(lines.len(), 4, "{lines:?}");
    assert!(
        lines[0].ends_with("(from name) every 0.3s (Ctrl-C to stop)"),
        "{lines:?}"
    );
    let rates: Vec<f64> = lines[1..]
        .iter()
        .filter_map(|l| l.split_whitespace().next()?.parse().ok())
        .collect();
    // About 200 Hz less sleep overshoot; far from 0 and from the 1/window a reader would see.
    assert!(
        rates.len() == 3 && rates.iter().all(|&r| (50.0..400.0).contains(&r)),
        "{lines:?}"
    );
}
