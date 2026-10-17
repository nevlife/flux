//! A build with one codegen unit and fat LTO sees a trace function whole and would leave the call
//! site's argument registers unset if the function did not read them. This builds `flux2_ffi` that
//! way and runs `tests/c/trace_args.c` under `tests/c/trace_recorder.c`, against the shared library
//! (recorder by `LD_PRELOAD`) and the static one (its trace symbols made weak, the recorder linked
//! in), then checks the values each event carried.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

use flux2_core::trace;

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn tmp() -> PathBuf {
    PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("trace_args")
}

fn sh(cmd: &mut Command) {
    let out = cmd.output().unwrap();
    assert!(
        out.status.success(),
        "{cmd:?}: {}",
        String::from_utf8_lossy(&out.stderr)
    );
}

fn build_lto() -> PathBuf {
    let target = tmp().join("cargo");
    sh(Command::new(env!("CARGO"))
        .args(["build", "--release", "--locked", "-p", "flux2_ffi"])
        .current_dir(dir().join(".."))
        .env("CARGO_TARGET_DIR", &target)
        .env("CARGO_PROFILE_RELEASE_CODEGEN_UNITS", "1")
        .env("CARGO_PROFILE_RELEASE_LTO", "fat"));
    let lib = target.join("release");
    let soname = lib.join(format!("libflux2_ffi.so.{}", flux2_ffi::FLUX2_ABI_VERSION));
    let _ = std::fs::remove_file(&soname);
    std::os::unix::fs::symlink("libflux2_ffi.so", &soname).unwrap();
    lib
}

fn cc(args: &[&dyn AsRef<std::ffi::OsStr>], out: &Path) {
    let mut c = Command::new("cc");
    c.args([
        "-std=c11",
        "-D_GNU_SOURCE",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
    ])
    .arg(dir().join("include"));
    for a in args {
        c.arg(a);
    }
    sh(c.args(["-lpthread", "-ldl", "-lm", "-o"]).arg(out));
}

fn run(exe: &Path, preload: Option<&Path>) -> Vec<(String, Vec<String>)> {
    let log = exe.with_extension("log");
    let mut c = Command::new(exe);
    c.env("FLUX2_TRACE_LOG", &log);
    if let Some(p) = preload {
        c.env("LD_PRELOAD", p);
    }
    sh(&mut c);
    std::fs::read_to_string(&log)
        .unwrap()
        .lines()
        .map(|l| {
            let mut f = l.split(' ').map(str::to_owned);
            (f.next().unwrap(), f.collect())
        })
        .collect()
}

fn num(s: &str) -> i128 {
    s.parse().unwrap_or_else(|_| panic!("{s} is not a number"))
}

fn stamp_of(i: u64) -> u64 {
    i | ((i * 1000) << 32)
}

fn check(ev: &[(String, Vec<String>)], how: &str) {
    let of = |name: &str| -> Vec<&Vec<String>> {
        ev.iter()
            .filter(|(n, _)| n == name)
            .map(|(_, a)| a)
            .collect()
    };
    let names: BTreeSet<&str> = ev.iter().map(|(n, _)| n.as_str()).collect();
    for name in [
        "channel_init",
        "claim",
        "loan",
        "loan_end",
        "commit",
        "take",
        "release",
        "wait",
        "wake",
        "callback_start",
        "callback_end",
        "endpoint_init",
        "py_gil",
        "py_ready",
        "record_drop",
        "gpu_wedged",
        "contract_2",
    ] {
        assert!(names.contains(name), "{how}: no {name}");
    }
    let init = of("channel_init");
    assert_eq!(init.len(), 2, "{how}");
    let (publisher, subscriber) = (&init[0][0], &init[1][0]);
    for (i, a) in init.iter().enumerate() {
        assert!(a[3].starts_with("s:/flux2."), "{how}: {a:?}");
        assert_eq!(a[4..], [["1", "0"][i], "4", "64", "4"], "{how}");
    }
    for (i, c) in of("commit").iter().enumerate() {
        let ticket = i as u64 + 1;
        assert_eq!(&c[0], publisher, "{how}");
        assert_eq!(num(&c[1]), ticket.into(), "{how}");
        assert_eq!(num(&c[3]), 16, "{how}");
        assert_eq!(num(&c[5]), stamp_of(ticket).into(), "{how}");
    }
    let takes = of("take");
    assert_eq!(takes.len(), 6, "{how}");
    for (i, t) in takes.iter().enumerate() {
        let ticket = i as u64 + 1;
        assert_eq!(&t[0], subscriber, "{how}");
        assert_eq!(num(&t[1]), ticket.into(), "{how}");
        assert_eq!(num(&t[4]), stamp_of(ticket).into(), "{how}");
    }
    for r in of("release") {
        assert_eq!((&r[0], r[2].as_str()), (subscriber, "0"), "{how}");
    }
    for r in of("py_ready") {
        assert_eq!(&r[0], subscriber, "{how}");
    }
    let ranges = [
        ("claim", 2, trace::CLAIM_FENCE_FAILED),
        ("loan_end", 2, trace::LOAN_NOT_PUBLISHER),
        ("wake", 1, trace::WAKE_SIGNAL),
        ("record_drop", 3, trace::RECORD_WRITE_FAILED),
    ];
    for (name, at, max) in ranges {
        for a in of(name) {
            assert!((0..=i128::from(max)).contains(&num(&a[at])), "{how}: {a:?}");
        }
    }
    for c in of("claim") {
        assert_eq!(&c[0], publisher, "{how}");
    }
    let endpoint = &of("endpoint_init")[0];
    assert_eq!(endpoint[1..3], ["s:/ns/node", "s:/key"], "{how}");
    assert_eq!(&of("callback_start")[0][2], &endpoint[0], "{how}");
    assert_eq!(&of("callback_end")[0][3], subscriber, "{how}");
    assert_eq!(
        of("record_drop")[0][..3],
        [subscriber.clone(), "0".into(), "3".into()]
    );
    assert_eq!(of("gpu_wedged")[0][..], ["700"], "{how}");
}

#[test]
fn every_event_carries_its_arguments_under_fat_lto() {
    std::fs::create_dir_all(tmp()).unwrap();
    let lib = build_lto();
    let recorder = tmp().join("recorder.so");
    cc(
        &[
            &"-shared",
            &"-fPIC",
            &dir().join("tests/c/trace_recorder.c"),
        ],
        &recorder,
    );
    let program = dir().join("tests/c/trace_args.c");

    let shared = tmp().join("shared");
    let rpath = format!("-Wl,-rpath,{}", lib.display());
    cc(&[&program, &"-L", &lib, &"-lflux2_ffi", &rpath], &shared);
    check(&run(&shared, Some(&recorder)), "cdylib");

    let weak = tmp().join("libflux2_ffi_weak.a");
    sh(Command::new("objcopy")
        .args(["--wildcard", "-W", "flux2_trace_*"])
        .arg(lib.join("libflux2_ffi.a"))
        .arg(&weak));
    let fixed = tmp().join("static");
    cc(
        &[&program, &dir().join("tests/c/trace_recorder.c"), &weak],
        &fixed,
    );
    check(&run(&fixed, None), "staticlib");
}
