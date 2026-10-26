//! Installed .msg files: the registry nested references resolve against, and the sources a
//! schema is read from.

use std::collections::HashSet;
use std::path::PathBuf;
use std::rc::Rc;

use crate::dtypes::ros_primitive;
use crate::flatten::flatten_message;
use crate::layout::{Layout, build_layout};
use crate::model::Message;
use crate::parse::{ParseError, Registry, load_dir, resolve};

/// Every installed .msg on `AMENT_PREFIX_PATH`, keyed pkg/Msg.
///
/// Merges without displacing: a message already in `reg` was named by the caller, and an
/// installed copy of it (the same package rebuilt, or an adapter generated for a package that
/// ships its own .msg) must not be the one the layout is read from.
pub fn ament_registry(reg: &mut Registry) -> Result<(), ParseError> {
    for (pkg, dir) in msg_dirs()? {
        load_dir(&dir, &pkg, reg, true)?;
    }
    Ok(())
}

/// `(package, share/<package>/msg)` in `AMENT_PREFIX_PATH` order, so an overlay comes first.
fn msg_dirs() -> Result<Vec<(String, PathBuf)>, ParseError> {
    let mut out = Vec::new();
    let prefixes = std::env::var_os("AMENT_PREFIX_PATH").unwrap_or_default();
    for prefix in std::env::split_paths(&prefixes) {
        let share = prefix.join("share");
        if prefix.as_os_str().is_empty() || !share.is_dir() {
            continue;
        }
        let mut pkgs: Vec<String> = std::fs::read_dir(&share)
            .map_err(|e| ParseError::new(format!("{}: {e}", share.display())))?
            .filter_map(|e| e.ok()?.file_name().into_string().ok())
            .collect();
        pkgs.sort();
        for pkg in pkgs {
            let dir = share.join(&pkg).join("msg");
            if dir.is_dir() {
                out.push((pkg, dir));
            }
        }
    }
    Ok(out)
}

pub fn installed_layouts() -> Result<Vec<Layout>, ParseError> {
    let mut reg = Registry::new();
    ament_registry(&mut reg)?;
    Ok(layouts(&reg))
}

/// Every message in `reg` the flat wire can carry; one that fails to parse, or reads one, is out.
pub fn layouts(reg: &Registry) -> Vec<Layout> {
    let mut out = Vec::new();
    for (key, m) in reg.messages() {
        if !key.contains('/') {
            continue;
        }
        if let Ok(Ok(layout)) = flatten_message(m, reg).map(|f| build_layout(&f)) {
            out.push(layout);
        }
    }
    out.sort_by_key(Layout::type_name);
    out
}

/// Every .msg a schema is read from, itself included.
///
/// A build has to depend on all of them: editing a nested type changes the carrier's layout and
/// its fingerprint, and a stale adapter is not a build error but a segment name that no longer
/// matches its peer.
pub fn sources(msg: &Rc<Message>, reg: &Registry) -> Vec<PathBuf> {
    fn rec(msg: &Rc<Message>, reg: &Registry, seen: &mut HashSet<*const Message>) -> Vec<PathBuf> {
        if !seen.insert(Rc::as_ptr(msg)) {
            return Vec::new();
        }
        let mut out: Vec<PathBuf> = msg.path.iter().cloned().collect();
        for fld in &msg.fields {
            let t = fld.type_name.as_str();
            if ros_primitive(t).is_some() || t == "string" || t == "wstring" {
                continue;
            }
            if let Ok(Some(nested)) = resolve(reg, t, msg.package.as_deref()) {
                out.extend(rec(&nested, reg, seen));
            }
        }
        out
    }
    rec(msg, reg, &mut HashSet::new())
}
