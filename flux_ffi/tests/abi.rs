//! The C ABI tests are C programs in tests/c, built against the committed header and the static
//! library, so they see exactly what a C or C++ caller sees.

use std::path::{Path, PathBuf};
use std::process::Command;

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn staticlib() -> PathBuf {
    let lib = std::env::current_exe()
        .unwrap()
        .parent()
        .unwrap()
        .join("libflux_ffi.a");
    assert!(lib.exists(), "no {}", lib.display());
    lib
}

fn build(src: &str) -> PathBuf {
    let exe = std::env::temp_dir().join(format!("flux_c_{src}_{}", std::process::id()));
    let ok = Command::new("cc")
        .args([
            "-std=c11",
            "-D_GNU_SOURCE",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
        ])
        .arg(dir().join("include"))
        .arg(dir().join(format!("tests/c/{src}.c")))
        .arg(staticlib())
        .args(["-lpthread", "-ldl", "-lm", "-o"])
        .arg(&exe)
        .status()
        .unwrap()
        .success();
    assert!(ok, "{src}.c failed to build");
    exe
}

fn run(src: &str) {
    run_in(src, &[&[]]);
}

/// One build, run once in each environment.
fn run_in(src: &str, envs: &[&[(&str, &str)]]) {
    let exe = build(src);
    let outs: Vec<_> = envs
        .iter()
        .map(|env| {
            Command::new(&exe)
                .envs(env.iter().copied())
                .output()
                .unwrap()
        })
        .collect();
    let _ = std::fs::remove_file(&exe);
    for (env, out) in envs.iter().zip(outs) {
        assert!(
            out.status.success(),
            "{src} {env:?}: {}",
            String::from_utf8_lossy(&out.stderr)
        );
    }
}

#[test]
fn channel() {
    run("channel");
}

#[test]
fn executor() {
    run_in("executor", &[&[], &[("FLUX_DISABLE_FUTEX_WAITV", "1")]]);
}

#[test]
fn manifest() {
    run("manifest");
}

/// A library of another ABI must not be loaded in place of this one: the dynamic linker looks for
/// the soname a binary was linked with.
#[test]
fn the_soname_carries_the_abi_version() {
    let so = staticlib().with_extension("so");
    let out = Command::new("readelf").arg("-d").arg(&so).output().unwrap();
    let dynamic = String::from_utf8_lossy(&out.stdout);
    let want = format!("[libflux_ffi.so.{}]", flux_ffi::FLUX_ABI_VERSION);
    assert!(dynamic.contains(&want), "{dynamic}");
}

fn build_cpp(src: &str, include: &Path, tag: &str) -> PathBuf {
    let exe = std::env::temp_dir().join(format!("flux_cpp_{src}_{tag}_{}", std::process::id()));
    let ok = Command::new("c++")
        .args(["-std=c++17", "-Wall", "-Wextra", "-Werror", "-I"])
        .arg(include)
        .arg("-I")
        .arg(dir().join("../flux_cpp/include"))
        .arg(dir().join(format!("tests/c/{src}.cpp")))
        .arg(staticlib())
        .args(["-lpthread", "-ldl", "-lm", "-o"])
        .arg(&exe)
        .status()
        .unwrap()
        .success();
    assert!(ok, "{src}.cpp failed to build");
    exe
}

/// A program built against an older or newer header, with the same soname, would write past the
/// storage its header sized. The C++ wrapper refuses before the first call that could.
#[test]
fn the_cpp_wrapper_refuses_a_library_of_another_abi() {
    let other = std::env::temp_dir().join(format!("flux_abi_other_{}", std::process::id()));
    std::fs::create_dir_all(&other).unwrap();
    let header = std::fs::read_to_string(dir().join("include/flux.h")).unwrap();
    let define = format!("#define FLUX_ABI_VERSION {}\n", flux_ffi::FLUX_ABI_VERSION);
    assert!(header.contains(&define));
    let skewed = header.replace(
        &define,
        &format!(
            "#define FLUX_ABI_VERSION {}\n",
            flux_ffi::FLUX_ABI_VERSION + 1
        ),
    );
    std::fs::write(other.join("flux.h"), skewed).unwrap();
    for (include, arg) in [(dir().join("include"), "same"), (other.clone(), "other")] {
        let exe = build_cpp("abi_check", &include, arg);
        let out = Command::new(&exe).arg(arg).output().unwrap();
        let _ = std::fs::remove_file(&exe);
        assert!(
            out.status.success(),
            "{arg}: {:?} {}",
            out.status,
            String::from_utf8_lossy(&out.stderr)
        );
    }
    let _ = std::fs::remove_dir_all(&other);
}

#[test]
fn the_header_is_valid_cpp() {
    let header: &Path = &dir().join("include/flux.h");
    let ok = Command::new("c++")
        .args([
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-fsyntax-only",
            "-x",
            "c++",
        ])
        .arg(header)
        .status()
        .unwrap()
        .success();
    assert!(ok, "flux.h is not valid C++");
}

/// cbindgen skips what it cannot see, such as functions a macro writes; nothing else would notice
/// an exported symbol missing from the header.
fn exported() -> String {
    let so = staticlib().with_extension("so");
    let out = Command::new("nm")
        .args(["-D", "--defined-only"])
        .arg(&so)
        .output()
        .unwrap();
    assert!(out.status.success(), "nm failed on {}", so.display());
    String::from_utf8_lossy(&out.stdout).into_owned()
}

#[test]
fn every_exported_symbol_is_declared() {
    let header = std::fs::read_to_string(dir().join("include/flux.h")).unwrap();
    let trace = std::fs::read_to_string(dir().join("include/flux/trace.h")).unwrap();
    let missing: Vec<String> = exported()
        .lines()
        .filter_map(|l| l.split_whitespace().nth(2))
        .filter(|s| s.starts_with("flux_"))
        .filter(|s| {
            let h = if s.starts_with("flux_trace_") {
                &trace
            } else {
                &header
            };
            !h.contains(&format!("{s}("))
        })
        .map(str::to_owned)
        .collect();
    assert!(
        missing.is_empty(),
        "exported but not in flux.h: {missing:?}"
    );
}

const TRACE_EVENTS: [&str; 18] = [
    "channel_init",
    "claim",
    "loan",
    "loan_end",
    "commit",
    "take",
    "release",
    "refused",
    "gpu_wedged",
    "wait",
    "wake",
    "callback_start",
    "callback_end",
    "endpoint_init",
    "py_gil",
    "py_ready",
    "record_drop",
    "contract_2",
];

/// armo attaches uprobes to these by name. Two events at one address would make a probe on one
/// fire for both.
#[test]
fn trace_events_are_exported_at_distinct_addresses() {
    let nm = exported();
    let events: Vec<(&str, &str)> = nm
        .lines()
        .filter_map(|l| {
            let f: Vec<&str> = l.split_whitespace().collect();
            (f.len() == 3 && f[2].starts_with("flux_trace_")).then(|| (f[0], f[2]))
        })
        .collect();
    assert_eq!(events.len(), TRACE_EVENTS.len(), "{events:?}");
    for name in TRACE_EVENTS {
        assert!(
            events
                .iter()
                .any(|(_, s)| *s == format!("flux_trace_{name}")),
            "flux_trace_{name} is not exported"
        );
    }
    let mut addrs: Vec<&str> = events.iter().map(|(a, _)| *a).collect();
    addrs.sort_unstable();
    addrs.dedup();
    assert_eq!(
        addrs.len(),
        events.len(),
        "trace events share an address: {events:?}"
    );
}

/// The values are the contract a collector decodes with, so the header must say what the core
/// emits.
#[test]
fn trace_constants_match_the_core() {
    use flux_core::trace as t;
    let header = std::fs::read_to_string(dir().join("include/flux/trace.h")).unwrap();
    let defined: Vec<(&str, &str)> = header
        .lines()
        .filter_map(|l| l.strip_prefix("#define FLUX_TRACE_"))
        .filter_map(|l| l.split_once(' '))
        .collect();
    let core = [
        ("CLAIM_OK", t::CLAIM_OK),
        ("CLAIM_RECLAIMED", t::CLAIM_RECLAIMED),
        ("CLAIM_BACKPRESSURE", t::CLAIM_BACKPRESSURE),
        ("CLAIM_NOT_PUBLISHER", t::CLAIM_NOT_PUBLISHER),
        ("CLAIM_WRONG_DEVICE", t::CLAIM_WRONG_DEVICE),
        ("CLAIM_TOO_LARGE", t::CLAIM_TOO_LARGE),
        ("CLAIM_FENCE_FAILED", t::CLAIM_FENCE_FAILED),
        ("CLAIM_GPU_WEDGED", t::CLAIM_GPU_WEDGED),
        ("LOAN_COMMITTED", t::LOAN_COMMITTED),
        ("LOAN_ABORTED", t::LOAN_ABORTED),
        ("LOAN_TOO_LARGE", t::LOAN_TOO_LARGE),
        ("LOAN_FENCE_FAILED", t::LOAN_FENCE_FAILED),
        ("LOAN_NOT_PUBLISHER", t::LOAN_NOT_PUBLISHER),
        ("LOAN_GPU_WEDGED", t::LOAN_GPU_WEDGED),
        ("WAKE_TIMEOUT", t::WAKE_TIMEOUT),
        ("WAKE_SOURCE", t::WAKE_SOURCE),
        ("WAKE_CONTROL", t::WAKE_CONTROL),
        ("WAKE_ALREADY", t::WAKE_ALREADY),
        ("WAKE_SIGNAL", t::WAKE_SIGNAL),
        ("REFUSED_MAX_BORROW", t::REFUSED_MAX_BORROW),
        ("REFUSED_NO_READER", t::REFUSED_NO_READER),
        ("REFUSED_CONTENDED", t::REFUSED_CONTENDED),
        ("REFUSED_BAD_FRAME", t::REFUSED_BAD_FRAME),
        ("REFUSED_FENCE", t::REFUSED_FENCE),
        ("REFUSED_GPU_WEDGED", t::REFUSED_GPU_WEDGED),
        ("RECORD_CACHE_FULL", t::RECORD_CACHE_FULL),
        ("RECORD_WRITE_FAILED", t::RECORD_WRITE_FAILED),
        ("RECORD_OVERRUN", t::RECORD_OVERRUN),
        ("ROLE_PUBLISHER", t::ROLE_PUBLISHER),
        ("ROLE_SUBSCRIPTION", t::ROLE_SUBSCRIPTION),
        ("LANG_CPP", t::LANG_CPP),
        ("LANG_PYTHON", t::LANG_PYTHON),
    ];
    let numeric: Vec<(&str, i32)> = defined
        .iter()
        .filter(|(n, _)| *n != "NONE")
        .map(|(n, v)| (*n, v.parse().unwrap()))
        .collect();
    assert_eq!(numeric, core);
    assert!(defined.contains(&("NONE", "UINT32_MAX")));
    assert_eq!(t::NONE, u32::MAX);
}

#[test]
fn the_trace_header_is_valid_c_and_cpp() {
    for (compiler, lang, std) in [("cc", "c", "-std=c11"), ("c++", "c++", "-std=c++17")] {
        let ok = Command::new(compiler)
            .args([
                std,
                "-Wall",
                "-Wextra",
                "-Werror",
                "-fsyntax-only",
                "-x",
                lang,
            ])
            .arg(dir().join("include/flux/trace.h"))
            .status()
            .unwrap()
            .success();
        assert!(ok, "flux/trace.h is not valid {lang}");
    }
}
