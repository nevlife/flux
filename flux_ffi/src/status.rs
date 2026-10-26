use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char};

use flux_core::{Error, Result};

#[repr(i32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum flux_status {
    FLUX_OK = 0,
    FLUX_INVALID_ARGUMENT = 1,
    FLUX_MISMATCH = 2,
    /// Not there yet (no publisher). Retrying may help.
    FLUX_NOT_READY = 3,
    FLUX_PUBLISHER_EXISTS = 4,
    FLUX_SUBSCRIBERS_FULL = 5,
    FLUX_UNSUPPORTED = 6,
    /// A callback returned failure; the caller kept its own error.
    FLUX_CALLBACK = 7,
    FLUX_IO = 8,
    FLUX_IN_USE = 9,
}

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::default());
}

pub(crate) fn status(r: Result<()>) -> flux_status {
    r.err().map_or(flux_status::FLUX_OK, |e| status_of(&e))
}

/// Also sets this thread's `flux_last_error`.
pub(crate) fn status_of(e: &Error) -> flux_status {
    let code = match e {
        Error::InvalidArgument(_) => flux_status::FLUX_INVALID_ARGUMENT,
        Error::Mismatch(_) => flux_status::FLUX_MISMATCH,
        Error::NotReady(_) => flux_status::FLUX_NOT_READY,
        Error::PublisherExists(_) => flux_status::FLUX_PUBLISHER_EXISTS,
        Error::SubscribersFull(_) => flux_status::FLUX_SUBSCRIBERS_FULL,
        Error::Unsupported(_) => flux_status::FLUX_UNSUPPORTED,
        Error::Callback(_) => flux_status::FLUX_CALLBACK,
        Error::Io { .. } => flux_status::FLUX_IO,
    };
    set_last_error(CString::new(e.to_string().replace('\0', " ")).unwrap_or_default());
    code
}

pub(crate) fn in_use() -> flux_status {
    set_last_error(
        c"another thread is in a call on this channel; give each thread its own".to_owned(),
    );
    flux_status::FLUX_IN_USE
}

// A call from a thread-exit destructor can come after this thread's storage is torn down.
fn set_last_error(msg: CString) {
    let _ = LAST_ERROR.try_with(|l| *l.borrow_mut() = msg);
}

/// Why this thread's last failed call failed. Valid until its next failed call.
#[unsafe(no_mangle)]
pub extern "C" fn flux_last_error() -> *const c_char {
    LAST_ERROR
        .try_with(|l| l.borrow().as_ptr())
        .unwrap_or(c"flux: no error text during thread exit".as_ptr())
}

pub(crate) unsafe fn text<'a>(p: *const c_char) -> Result<&'a str> {
    // SAFETY: the caller passes a NUL-terminated string that outlives 'a.
    unsafe { CStr::from_ptr(p) }
        .to_str()
        .map_err(|_| Error::InvalidArgument("a string argument is not UTF-8".into()))
}

/// Copies `s` and a NUL into `out`. `*len` is set to `s.len()` either way, so a caller with too
/// small a buffer learns the size it needs.
pub(crate) unsafe fn copy_out(
    s: &str,
    out: *mut c_char,
    cap: usize,
    len: *mut usize,
) -> Result<()> {
    // SAFETY: len is valid for a write.
    unsafe { len.write(s.len()) };
    if s.len() >= cap {
        return Err(Error::InvalidArgument(format!(
            "buffer of {cap} bytes is too small for {} and a NUL",
            s.len()
        )));
    }
    let out = out.cast::<u8>();
    // SAFETY: out holds cap > s.len() bytes.
    unsafe { out.copy_from_nonoverlapping(s.as_ptr(), s.len()) };
    // SAFETY: as above, s.len() < cap.
    let nul = unsafe { out.add(s.len()) };
    // SAFETY: as above.
    unsafe { nul.write(0) };
    Ok(())
}
