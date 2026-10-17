use std::collections::HashMap;
use std::ffi::c_char;

use flux2_core::Error;
use flux2_gen::ament::installed_layouts;
use flux2_gen::cdr::{cdr_size, write_cdr};
use flux2_gen::layout::Layout;

use crate::status::{flux2_status, status};

pub struct flux2_schema {
    layout: Layout,
    /// NUL-terminated for C. Made of file names, so it holds no other NUL.
    ros_type: String,
}

pub struct flux2_schemas {
    by_fingerprint: HashMap<u64, flux2_schema>,
}

/// Every .msg on `AMENT_PREFIX_PATH` that parses. `FLUX2_UNSUPPORTED` if a directory is unreadable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_schemas_load(out: *mut *mut flux2_schemas) -> flux2_status {
    let layouts = match installed_layouts() {
        Ok(l) => l,
        Err(e) => return status(Err(Error::Unsupported(e.to_string()))),
    };
    let by_fingerprint = layouts
        .into_iter()
        .map(|layout| {
            let ros_type = match &layout.package {
                Some(p) => format!("{p}/msg/{}\0", layout.name),
                None => format!("{}\0", layout.name),
            };
            let schema = flux2_schema { layout, ros_type };
            (schema.layout.fingerprint, schema)
        })
        .collect();
    let s = Box::into_raw(Box::new(flux2_schemas { by_fingerprint }));
    // SAFETY: out is valid for a write.
    unsafe { out.write(s) };
    status(Ok(()))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_schemas_free(s: *mut flux2_schemas) {
    if !s.is_null() {
        // SAFETY: s came from flux2_schemas_load and is freed once.
        drop(unsafe { Box::from_raw(s) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_schemas_size(s: *const flux2_schemas) -> usize {
    // SAFETY: s is a live handle.
    unsafe { &*s }.by_fingerprint.len()
}

/// NULL when no installed type has this fingerprint.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_schemas_find(
    s: *const flux2_schemas,
    fingerprint: u64,
) -> *const flux2_schema {
    // SAFETY: s is a live handle.
    unsafe { &*s }
        .by_fingerprint
        .get(&fingerprint)
        .map_or(std::ptr::null(), std::ptr::from_ref)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_schema_ros_type(s: *const flux2_schema) -> *const c_char {
    // SAFETY: s is a live schema.
    unsafe { &*s }.ros_type.as_ptr().cast()
}

unsafe fn frame<'a>(data: *const u8, len: usize) -> &'a [u8] {
    if len == 0 {
        return &[];
    }
    // SAFETY: the caller's frame is readable for len bytes while the call runs.
    unsafe { std::slice::from_raw_parts(data, len) }
}

fn unreadable(e: &flux2_gen::cdr::Unreadable) -> flux2_status {
    status(Err(Error::Mismatch(e.0.clone())))
}

/// The frame as the ROS message's CDR. `FLUX2_MISMATCH` for a frame that does not match.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_schema_cdr_size(
    s: *const flux2_schema,
    data: *const u8,
    len: usize,
    out: *mut usize,
) -> flux2_status {
    // SAFETY: s is a live schema; data holds len bytes.
    match cdr_size(&unsafe { &*s }.layout, unsafe { frame(data, len) }) {
        Ok(n) => {
            // SAFETY: out is valid for a write.
            unsafe { out.write(n) };
            status(Ok(()))
        }
        Err(e) => unreadable(&e),
    }
}

/// `*written` is the CDR size even past `cap`, which is `FLUX2_INVALID_ARGUMENT` (frames change).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux2_schema_write_cdr(
    s: *const flux2_schema,
    data: *const u8,
    len: usize,
    out: *mut u8,
    cap: usize,
    written: *mut usize,
) -> flux2_status {
    let buf: &mut [u8] = if cap == 0 {
        &mut []
    } else {
        // SAFETY: out is writable for cap bytes and aliases nothing else during the call.
        unsafe { std::slice::from_raw_parts_mut(out, cap) }
    };
    // SAFETY: s is a live schema; data holds len bytes.
    match write_cdr(&unsafe { &*s }.layout, unsafe { frame(data, len) }, buf) {
        Ok(n) => {
            // SAFETY: written is valid for a write.
            unsafe { written.write(n) };
            status(if n > cap {
                Err(Error::InvalidArgument(format!(
                    "buffer of {cap} bytes is too small for a CDR message of {n}"
                )))
            } else {
                Ok(())
            })
        }
        Err(e) => unreadable(&e),
    }
}
