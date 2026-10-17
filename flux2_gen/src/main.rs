//! `flux2_gen` command line: turn .msg files into C++ and Python flux2 adapters.
//!
//! ```text
//! flux2_gen --pkg my_pkg --msg msg/Cloud.msg --out-cpp include --out-py python
//! ```
//!
//! Nested types resolve against the message's own directory first, then against every ROS
//! package on `AMENT_PREFIX_PATH`: a schema's fingerprint depends on what its nested types flatten
//! to, so an unresolvable reference is an error rather than a guess.

use std::collections::BTreeSet;
use std::ffi::OsString;
use std::io::Write as _;
use std::os::unix::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use flux2_gen::ament::{ament_registry, sources};
use flux2_gen::flatten::flatten_message;
use flux2_gen::layout::{Layout, build_layout};
use flux2_gen::parse::{Registry, load_dir};
use flux2_gen::text::{py_module, snake};
use flux2_gen::{emit_cpp, emit_py, emit_ros};

const USAGE: &str = "usage: flux2_gen --pkg PKG --msg MSG [--msg MSG ...] [--out-cpp DIR] \
                     [--out-py DIR] [--no-ros] [--depfile FILE] [--list]

  --no-ros   skip the ROS bridge (<name>_ros), which needs the message's rosidl types
  --list     print the files a run would write and exit, without reading any .msg: per
             message the C++ header, its bridge, the Python module, its bridge";

struct Args {
    pkg: String,
    msgs: Vec<PathBuf>,
    out_cpp: Option<PathBuf>,
    out_py: Option<PathBuf>,
    depfile: Option<PathBuf>,
    ros: bool,
    list: bool,
}

/// `None` for --help. Paths are taken as bytes: a Linux path need not be UTF-8.
fn parse_args() -> Result<Option<Args>, String> {
    let mut a = Args {
        pkg: String::new(),
        msgs: Vec::new(),
        out_cpp: None,
        out_py: None,
        depfile: None,
        ros: true,
        list: false,
    };
    let mut it = std::env::args_os().skip(1);
    while let Some(flag) = it.next() {
        let flag = flag.to_string_lossy().into_owned();
        match flag.as_str() {
            "--help" | "-h" => return Ok(None),
            "--no-ros" => a.ros = false,
            "--list" => a.list = true,
            "--pkg" | "--msg" | "--out-cpp" | "--out-py" | "--depfile" => {
                let value: OsString = it
                    .next()
                    .ok_or_else(|| format!("argument {flag}: expected one argument"))?;
                match flag.as_str() {
                    "--pkg" => {
                        a.pkg = value
                            .into_string()
                            .map_err(|_| "argument --pkg: not UTF-8".to_owned())?;
                    }
                    "--msg" => a.msgs.push(value.into()),
                    "--out-cpp" => a.out_cpp = Some(value.into()),
                    "--out-py" => a.out_py = Some(value.into()),
                    _ => a.depfile = Some(value.into()),
                }
            }
            _ => return Err(format!("unrecognized argument: {flag}")),
        }
    }
    if a.pkg.is_empty() || a.msgs.is_empty() {
        return Err("the following arguments are required: --pkg, --msg".into());
    }
    if a.out_cpp.is_none() && a.out_py.is_none() {
        return Err("nothing to do: pass --out-cpp and/or --out-py".into());
    }
    if a.depfile.is_some() && a.msgs.len() != 1 {
        return Err("--depfile describes one message; pass a single --msg".into());
    }
    Ok(Some(a))
}

fn stem(msg_path: &Path) -> Result<&str, String> {
    let file = msg_path.file_name().unwrap_or_default();
    let file = file.to_str().ok_or_else(|| {
        format!(
            "flux2_gen: {}: message file name is not UTF-8",
            msg_path.display()
        )
    })?;
    Ok(file.strip_suffix(".msg").unwrap_or(file))
}

#[derive(Clone, Copy)]
enum Emit {
    Cpp,
    CppRos,
    Py,
    PyRos,
}

/// The files one message generates, in the order --list prints them.
fn outputs(a: &Args, stem: &str) -> Vec<(Emit, PathBuf)> {
    let pkg = &a.pkg;
    let mut out = Vec::new();
    if let Some(root) = &a.out_cpp {
        let (dir, name) = (root.join(pkg).join("flux2"), snake(stem));
        out.push((Emit::Cpp, dir.join(format!("{name}.hpp"))));
        if a.ros {
            out.push((Emit::CppRos, dir.join(format!("{name}_ros.hpp"))));
        }
    }
    if let Some(root) = &a.out_py {
        let dir = root.join(format!("{pkg}_flux2"));
        out.push((Emit::Py, dir.join(format!("{}.py", py_module(stem)))));
        if a.ros {
            out.push((Emit::PyRos, dir.join(format!("{}_ros.py", snake(stem)))));
        }
    }
    out
}

fn write(path: &Path, text: &[u8]) -> Result<(), String> {
    let fail = |e: std::io::Error| format!("flux2_gen: {}: {e}", path.display());
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(fail)?;
    }
    std::fs::write(path, text).map_err(fail)
}

fn print_path(path: &Path) {
    let mut out = std::io::stdout().lock();
    // A closed stdout loses only the listing, never a generated file.
    let _ = out
        .write_all(path.as_os_str().as_bytes())
        .and_then(|()| out.write_all(b"\n"));
}

fn layout_of(a: &Args, stem: &str, msg_path: &Path, reg: &Registry) -> Result<Layout, String> {
    let pkg = &a.pkg;
    let msg = match reg.entry(&format!("{pkg}/{stem}")) {
        Some(Ok(msg)) => msg,
        Some(Err(e)) => return Err(e.to_string()),
        None => {
            return Err(format!(
                "flux2_gen: {pkg}/{stem} not found (looked beside {})",
                msg_path.display()
            ));
        }
    };
    let flat = flatten_message(msg, reg).map_err(|e| e.to_string())?;
    if let Some(why) = &flat.rejected {
        return Err(format!(
            "flux2_gen: {pkg}/{stem} cannot ride the flat wire -- {why}"
        ));
    }
    build_layout(&flat).map_err(|e| format!("flux2_gen: {pkg}/{stem}: {e}"))
}

/// Generates the adapters of one .msg and returns the paths written.
fn generate(a: &Args, msg_path: &Path, reg: &Registry) -> Result<Vec<PathBuf>, String> {
    let stem = stem(msg_path)?;
    let layout = layout_of(a, stem, msg_path, reg)?;
    let source = format!("{}/msg/{stem}.msg", a.pkg);
    let mut written = Vec::new();
    for (emit, path) in outputs(a, stem) {
        let text = match emit {
            Emit::Cpp => emit_cpp::emit(&layout, &source),
            Emit::CppRos => emit_ros::emit_cpp(&layout, &source),
            Emit::Py => emit_py::emit(&layout, &source),
            Emit::PyRos => emit_ros::emit_py(&layout, &source),
        };
        write(&path, text.as_bytes())?;
        written.push(path);
    }
    if let Some(depfile) = &a.depfile {
        let msg = reg
            .get(&format!("{}/{stem}", a.pkg))
            .expect("the layout came from this message");
        let deps: BTreeSet<OsString> = sources(msg, reg)
            .into_iter()
            .map(PathBuf::into_os_string)
            .collect();
        let mut text = Vec::new();
        for out in &written {
            text.extend_from_slice(out.as_os_str().as_bytes());
            text.push(b':');
            for dep in &deps {
                text.push(b' ');
                text.extend_from_slice(dep.as_bytes());
            }
            text.push(b'\n');
        }
        write(depfile, &text)?;
    }
    Ok(written)
}

fn run(a: &Args) -> Result<(), String> {
    if a.list {
        for msg in &a.msgs {
            for (_, path) in outputs(a, stem(msg)?) {
                print_path(&path);
            }
        }
        return Ok(());
    }
    // The named .msg files load first so that they win: generating for a package that is
    // already installed (its own rebuild, or an adapter for a message package like sensor_msgs)
    // would otherwise read the installed copy.
    let mut reg = Registry::new();
    let dirs: BTreeSet<PathBuf> = a
        .msgs
        .iter()
        .map(|m| {
            std::path::absolute(m)
                .ok()
                .and_then(|p| p.parent().map(Path::to_path_buf))
                .unwrap_or_default()
        })
        .collect();
    for d in &dirs {
        load_dir(d, &a.pkg, &mut reg, false).map_err(|e| e.to_string())?;
    }
    ament_registry(&mut reg).map_err(|e| e.to_string())?;
    for msg in &a.msgs {
        for path in generate(a, msg, &reg)? {
            print_path(&path);
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(Some(a)) => a,
        Ok(None) => {
            println!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Err(e) => {
            eprintln!("{USAGE}\nflux2_gen: error: {e}");
            return ExitCode::from(2);
        }
    };
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}
