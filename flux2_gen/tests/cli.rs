//! The `flux2_gen` program: arguments, paths and the registry it reads.

use std::ffi::OsStr;
use std::fs;
use std::os::unix::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

fn scratch(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("flux2_gen_cli_{tag}_{}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn msg(dir: &Path, name: &str, text: &str) -> PathBuf {
    fs::create_dir_all(dir).unwrap();
    let path = dir.join(format!("{name}.msg"));
    fs::write(&path, text).unwrap();
    path
}

fn run_gen(args: &[&OsStr], ament: &OsStr) -> Output {
    Command::new(env!("CARGO_BIN_EXE_flux2_gen"))
        .args(args)
        .env("AMENT_PREFIX_PATH", ament)
        .output()
        .unwrap()
}

fn stderr(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).into_owned()
}

#[test]
fn help_is_not_an_error() {
    let o = run_gen(&["--help".as_ref()], "".as_ref());
    assert!(o.status.success());
    assert!(String::from_utf8_lossy(&o.stdout).contains("usage"));
}

#[test]
fn a_path_that_is_not_utf8_is_still_a_path() {
    // Linux paths are bytes; reading the arguments as UTF-8 panicked.
    let root = scratch("latin1");
    let dir = root
        .join(OsStr::from_bytes(b"lat\xe9"))
        .join("tpkg")
        .join("msg");
    let m = msg(&dir, "Main", "int32 x\n");
    let (out, dep) = (root.join("out"), dir.join("main.d"));
    let o = run_gen(
        &[
            "--pkg".as_ref(),
            "tpkg".as_ref(),
            "--msg".as_ref(),
            m.as_os_str(),
            "--out-cpp".as_ref(),
            out.as_os_str(),
            "--depfile".as_ref(),
            dep.as_os_str(),
        ],
        "".as_ref(),
    );
    assert!(o.status.success(), "{}", stderr(&o));
    assert!(out.join("tpkg/flux2/main.hpp").is_file());
    let deps = fs::read(&dep).unwrap();
    assert!(
        deps.windows(4).any(|w| w == b"lat\xe9"),
        "depfile keeps the bytes"
    );
    fs::remove_dir_all(root).unwrap();
}

#[test]
fn a_bad_msg_elsewhere_does_not_stop_generation() {
    // One file the parser refuses, on AMENT_PREFIX_PATH or beside the message, failed every
    // generation; rosidl may well take it.
    let root = scratch("poison");
    msg(
        &root.join("prefix/share/other/msg"),
        "Other",
        "float64[bad] x\n",
    );
    let dir = root.join("tpkg/msg");
    let good = msg(&dir, "Good", "float64 v\n");
    let bad = msg(&dir, "Bad", "int32 x\nint32[bad] y\n");
    let out = root.join("out");
    let ament = root.join("prefix");
    let run = |m: &Path| {
        run_gen(
            &[
                "--pkg".as_ref(),
                "tpkg".as_ref(),
                "--msg".as_ref(),
                m.as_os_str(),
                "--out-py".as_ref(),
                out.as_os_str(),
            ],
            ament.as_os_str(),
        )
    };
    let o = run(&good);
    assert!(o.status.success(), "{}", stderr(&o));
    let o = run(&bad);
    assert!(!o.status.success());
    assert!(stderr(&o).contains("Bad.msg:2"), "{}", stderr(&o));
    fs::remove_dir_all(root).unwrap();
}

#[test]
fn list_names_every_file_a_run_writes() {
    // CMake declares these as outputs, so the naming rules live here alone. A message named
    // like a Python keyword gets a module it can be imported from.
    let args = |ros: bool| {
        let mut a = vec![
            "--pkg",
            "p",
            "--msg",
            "x/Global.msg",
            "--out-cpp",
            "c",
            "--out-py",
            "y",
        ];
        if !ros {
            a.push("--no-ros");
        }
        a.push("--list");
        let a: Vec<&OsStr> = a.iter().map(OsStr::new).collect();
        String::from_utf8(run_gen(&a, "".as_ref()).stdout).unwrap()
    };
    assert_eq!(
        args(true),
        concat!(
            "c/p/flux2/global.hpp\nc/p/flux2/global_ros.hpp\n",
            "y/p_flux2/global_.py\ny/p_flux2/global_ros.py\n"
        )
    );
    assert_eq!(args(false), "c/p/flux2/global.hpp\ny/p_flux2/global_.py\n");
}
