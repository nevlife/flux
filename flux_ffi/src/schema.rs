use std::collections::HashMap;
use std::ffi::c_char;

use flux_core::Error;
use flux_gen::ament::installed_layouts;
use flux_gen::cdr::{cdr_size, write_cdr};
use flux_gen::inline::{gpu_fingerprint, has_gpu_data, inline_arena, with_data};
use flux_gen::layout::Layout;

use crate::arena::flux_gpu_heap;
use crate::status::{flux_status, status};

pub struct flux_schema {
    layout: Layout,
    /// NUL-terminated for C. Made of file names, so it holds no other NUL.
    ros_type: String,
}

pub struct flux_schemas {
    by_fingerprint: HashMap<u64, flux_schema>,
    /// A GPU memory channel's fingerprint to its type's (`flux_gpu_fingerprint`).
    by_gpu: HashMap<u64, u64>,
}

/// Every .msg on `AMENT_PREFIX_PATH` that parses. `FLUX_UNSUPPORTED` if a directory is unreadable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schemas_load(out: *mut *mut flux_schemas) -> flux_status {
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
            let schema = flux_schema { layout, ros_type };
            (schema.layout.fingerprint, schema)
        })
        .collect::<HashMap<_, _>>();
    let by_gpu = by_fingerprint
        .iter()
        .filter(|(_, s)| has_gpu_data(&s.layout))
        .map(|(&fp, _)| (gpu_fingerprint(fp), fp))
        .collect();
    let s = Box::into_raw(Box::new(flux_schemas {
        by_fingerprint,
        by_gpu,
    }));
    // SAFETY: out is valid for a write.
    unsafe { out.write(s) };
    status(Ok(()))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schemas_free(s: *mut flux_schemas) {
    if !s.is_null() {
        // SAFETY: s came from flux_schemas_load and is freed once.
        drop(unsafe { Box::from_raw(s) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schemas_size(s: *const flux_schemas) -> usize {
    // SAFETY: s is a live handle.
    unsafe { &*s }.by_fingerprint.len()
}

/// NULL when no installed type has this fingerprint.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schemas_find(
    s: *const flux_schemas,
    fingerprint: u64,
) -> *const flux_schema {
    // SAFETY: s is a live handle.
    unsafe { &*s }
        .by_fingerprint
        .get(&fingerprint)
        .map_or(std::ptr::null(), std::ptr::from_ref)
}

// The type a GPU memory channel of this fingerprint carries, as flux_schemas_find.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schemas_find_gpu(
    s: *const flux_schemas,
    fingerprint: u64,
) -> *const flux_schema {
    // SAFETY: s is a live handle.
    let s = unsafe { &*s };
    s.by_gpu
        .get(&fingerprint)
        .and_then(|fp| s.by_fingerprint.get(fp))
        .map_or(std::ptr::null(), std::ptr::from_ref)
}

/// The fingerprint of a GPU memory channel of the type `base`: payload `data`, aux the rest.
#[unsafe(no_mangle)]
pub extern "C" fn flux_gpu_fingerprint(base: u64) -> u64 {
    gpu_fingerprint(base)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schema_ros_type(s: *const flux_schema) -> *const c_char {
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

fn unreadable(e: &flux_gen::cdr::Unreadable) -> flux_status {
    status(Err(Error::Mismatch(e.0.clone())))
}

/// The frame as the ROS message's CDR. `FLUX_MISMATCH` for a frame that does not match.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schema_cdr_size(
    s: *const flux_schema,
    data: *const u8,
    len: usize,
    out: *mut usize,
) -> flux_status {
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

/// `*written` is the CDR size even past `cap`, which is `FLUX_INVALID_ARGUMENT` (frames change).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schema_write_cdr(
    s: *const flux_schema,
    data: *const u8,
    len: usize,
    out: *mut u8,
    cap: usize,
    written: *mut usize,
) -> flux_status {
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

/// The frame with its arena fields (`heap`; `gpu` host copies or NULL) copied in, as `write_cdr`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schema_inline_arena(
    s: *const flux_schema,
    data: *const u8,
    len: usize,
    heap: *const u8,
    heap_len: usize,
    gpu: *const flux_gpu_heap,
    out: *mut u8,
    cap: usize,
    written: *mut usize,
) -> flux_status {
    // SAFETY: data holds len bytes while the call runs.
    let f = unsafe { frame(data, len) };
    // SAFETY: heap holds heap_len bytes while the call runs.
    let h = unsafe { frame(heap, heap_len) };
    let Some(body) = flux_core::arena::body(f) else {
        return status(Err(Error::Mismatch(
            "flux: a frame shorter than its trailer".into(),
        )));
    };
    // SAFETY: gpu is NULL or a flux_gpu_heap whose memory is valid while the call runs.
    let copies = unsafe { gpu_copies(gpu) };
    // SAFETY: s is a live schema.
    let v = match inline_arena(&unsafe { &*s }.layout, body, h, &copies) {
        Ok(v) => v,
        Err(e) => return unreadable(&e),
    };
    // SAFETY: written is valid for a write.
    unsafe { written.write(v.len()) };
    if v.len() > cap {
        return status(Err(Error::InvalidArgument(format!(
            "buffer of {cap} bytes is too small for a frame of {}",
            v.len()
        ))));
    }
    // SAFETY: out is writable for cap >= v.len() bytes and aliases nothing else.
    unsafe { std::ptr::copy_nonoverlapping(v.as_ptr(), out, v.len()) };
    status(Ok(()))
}

/// The GPU arena bytes the CPU reaches through `gpu`: the whole arena where it is host memory, else
/// the copies of the frame's blocks; each at its position.
///
/// # Safety
/// `gpu` is NULL or valid, with `host` and each block's `host` readable for their lengths.
unsafe fn gpu_copies<'a>(gpu: *const flux_gpu_heap) -> Vec<(usize, &'a [u8])> {
    // SAFETY: the caller's contract.
    let Some(g) = (unsafe { gpu.as_ref() }) else {
        return Vec::new();
    };
    let mut v = Vec::new();
    if !g.host.is_null() && g.size != 0 {
        #[allow(clippy::cast_possible_truncation)]
        let pos = flux_core::arena::GPU_POS as usize;
        // SAFETY: host holds size bytes.
        v.push((pos, unsafe { std::slice::from_raw_parts(g.host, g.size) }));
    }
    if !g.blocks.is_null() {
        // SAFETY: blocks holds count entries.
        for b in unsafe { std::slice::from_raw_parts(g.blocks, g.count) } {
            if b.host.is_null() {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)]
            let (pos, len) = (b.off as usize, b.len as usize);
            // SAFETY: a block's host copy holds len bytes.
            v.push((pos, unsafe { std::slice::from_raw_parts(b.host, len) }));
        }
    }
    v
}

/// A GPU memory channel's `aux` and `data` as one frame of this type, sized as `write_cdr`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn flux_schema_with_data(
    s: *const flux_schema,
    aux: *const u8,
    aux_len: usize,
    data: *const u8,
    data_len: usize,
    out: *mut u8,
    cap: usize,
    written: *mut usize,
) -> flux_status {
    // SAFETY: aux holds aux_len bytes while the call runs.
    let a = unsafe { frame(aux, aux_len) };
    // SAFETY: data holds data_len bytes while the call runs.
    let d = unsafe { frame(data, data_len) };
    // SAFETY: s is a live schema.
    let v = match with_data(&unsafe { &*s }.layout, a, d) {
        Ok(v) => v,
        Err(e) => return unreadable(&e),
    };
    // SAFETY: written is valid for a write.
    unsafe { written.write(v.len()) };
    if v.len() > cap {
        return status(Err(Error::InvalidArgument(format!(
            "buffer of {cap} bytes is too small for a frame of {}",
            v.len()
        ))));
    }
    // SAFETY: out is writable for cap >= v.len() bytes and aliases nothing else.
    unsafe { std::ptr::copy_nonoverlapping(v.as_ptr(), out, v.len()) };
    status(Ok(()))
}
