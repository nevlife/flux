// Each test binary compiles this module and uses only part of it.
#![allow(dead_code)]

use std::fs;
use std::path::{Path, PathBuf};

use flux2_gen::model::{ArrayKind, Field, Message};
use flux2_gen::parse::{Registry, load_dir};

pub fn msg_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("test")
        .join("msg")
}

pub fn test_registry() -> Registry {
    let mut reg = Registry::new();
    load_dir(&msg_dir(), "test", &mut reg, false).unwrap();
    reg
}

pub fn field(name: &str, type_name: &str, kind: ArrayKind, size: usize) -> Field {
    Field {
        name: name.to_owned(),
        type_name: type_name.to_owned(),
        kind,
        size,
        optional: false,
    }
}

pub fn message(name: &str, fields: Vec<Field>, package: Option<&str>) -> Message {
    Message {
        name: name.to_owned(),
        fields,
        package: package.map(str::to_owned),
        ..Message::default()
    }
}

fn sorted_entries(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = fs::read_dir(dir)
        .map(|rd| rd.filter_map(|e| Some(e.ok()?.path())).collect())
        .unwrap_or_default();
    out.sort();
    out
}

/// `/opt/ros/*/share/*/msg`, sorted.
pub fn ros_msg_dirs() -> Vec<PathBuf> {
    let mut out = Vec::new();
    for distro in sorted_entries(Path::new("/opt/ros")) {
        for pkg in sorted_entries(&distro.join("share")) {
            let dir = pkg.join("msg");
            if dir.is_dir() {
                out.push(dir);
            }
        }
    }
    out
}

/// `/opt/ros/*/share/*/msg/*.msg`, sorted.
pub fn ros_msg_files() -> Vec<PathBuf> {
    ros_msg_dirs()
        .iter()
        .flat_map(|d| sorted_entries(d))
        .filter(|p| p.extension().is_some_and(|e| e == "msg"))
        .collect()
}

pub fn ros_corpus() -> Registry {
    let mut reg = Registry::new();
    for dir in ros_msg_dirs() {
        let pkg = dir.parent().and_then(Path::file_name).unwrap();
        load_dir(&dir, pkg.to_str().unwrap(), &mut reg, false).unwrap();
    }
    reg
}
