//! Descriptors over `AF_UNIX` socket files in /dev/shm: an accept thread that hands one
//! descriptor (a device-handle payload) or a growing list of them (GPU arena chunks) to each peer
//! that runs as this user, the receiving side, and the sweep of endpoints nobody serves.

use std::ffi::c_int;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::sync::Arc;
use std::thread::JoinHandle;

use crate::error::{Error, Result};
use crate::sys::{self, PiMutex};

const RECV_TIMEOUT_NS: u64 = 1_000_000_000;

pub(crate) const ENDPOINT_PREFIX: &str = "flux.gpu.";
const STALE_BIND: std::time::Duration = std::time::Duration::from_secs(10);

/// The address and the length `bind` and `connect` take for a socket file.
fn path_addr(path: &str) -> Result<(libc::sockaddr_un, libc::socklen_t)> {
    // SAFETY: sockaddr_un is plain data; all zeros is a valid value.
    let mut addr: libc::sockaddr_un = unsafe { std::mem::zeroed() };
    if path.len() + 1 > addr.sun_path.len() {
        return Err(Error::InvalidArgument(format!(
            "device endpoint '{path}' does not fit an AF_UNIX address"
        )));
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let family = libc::AF_UNIX as libc::sa_family_t;
    addr.sun_family = family;
    for (dst, &b) in addr.sun_path.iter_mut().zip(path.as_bytes()) {
        *dst = libc::c_char::from_ne_bytes([b]);
    }
    let len = std::mem::offset_of!(libc::sockaddr_un, sun_path) + path.len() + 1;
    #[allow(clippy::cast_possible_truncation)]
    Ok((addr, len as libc::socklen_t))
}

fn connect(sock: RawFd, path: &str) -> Result<io::Result<()>> {
    let (addr, len) = path_addr(path)?;
    // SAFETY: addr and len describe a sockaddr_un.
    let r = unsafe { libc::connect(sock, (&raw const addr).cast(), len) };
    Ok(if r == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error())
    })
}

fn socket(flags: c_int) -> io::Result<OwnedFd> {
    // SAFETY: plain syscall.
    let fd = unsafe {
        libc::socket(
            libc::AF_UNIX,
            libc::SOCK_STREAM | libc::SOCK_CLOEXEC | flags,
            0,
        )
    };
    if fd < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: a new descriptor this process owns.
    Ok(unsafe { OwnedFd::from_raw_fd(fd) })
}

/// The peer runs as this user. The file's mode says the same, unless another user made it.
fn peer_is_us(conn: RawFd) -> bool {
    // SAFETY: ucred is plain data; all zeros is a valid value.
    let mut cred: libc::ucred = unsafe { std::mem::zeroed() };
    #[allow(clippy::cast_possible_truncation)]
    let mut len = size_of::<libc::ucred>() as libc::socklen_t;
    // SAFETY: cred and len describe a writable ucred.
    let r = unsafe {
        libc::getsockopt(
            conn,
            libc::SOL_SOCKET,
            libc::SO_PEERCRED,
            (&raw mut cred).cast(),
            &raw mut len,
        )
    };
    r == 0 && cred.uid == sys::euid()
}

#[allow(clippy::cast_possible_truncation)]
const INT_BYTES: u32 = size_of::<c_int>() as u32;

/// The descriptor rides in the control message, `bytes` in the data: at least one, since a message
/// with no data carries no control data either.
fn send_fd_with(conn: RawFd, fd: RawFd, bytes: &[u8]) -> bool {
    let mut iov = libc::iovec {
        iov_base: bytes.as_ptr().cast_mut().cast(),
        iov_len: bytes.len(),
    };
    // SAFETY: a constant computation.
    let space = unsafe { libc::CMSG_SPACE(INT_BYTES) } as usize;
    let mut control = vec![0u8; space];
    // SAFETY: msghdr is plain data; all zeros is a valid value.
    let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
    msg.msg_iov = &raw mut iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.as_mut_ptr().cast();
    msg.msg_controllen = space;
    // SAFETY: msg_control holds one CMSG_SPACE(int).
    let cmsg = unsafe { libc::CMSG_FIRSTHDR(&raw const msg) };
    // SAFETY: a constant computation.
    let len = unsafe { libc::CMSG_LEN(INT_BYTES) } as usize;
    // SAFETY: cmsg points into control, which has room for the header and one int.
    let header = unsafe { &mut *cmsg };
    header.cmsg_level = libc::SOL_SOCKET;
    header.cmsg_type = libc::SCM_RIGHTS;
    header.cmsg_len = len;
    // SAFETY: the int after the header, inside control.
    let data = unsafe { libc::CMSG_DATA(cmsg) };
    // SAFETY: data has room for one int, at any alignment.
    unsafe { data.cast::<c_int>().write_unaligned(fd) };
    // SAFETY: msg describes buffers that live through the call; sendmsg only reads iov_base.
    let sent = unsafe { libc::sendmsg(conn, &raw const msg, libc::MSG_NOSIGNAL) };
    usize::try_from(sent).is_ok_and(|n| n == bytes.len())
}

/// Exactly one descriptor, in a message whose data fills `bytes` exactly, or why not: none came,
/// or the kernel dropped it, as it does at this process's open file limit.
fn recv_fd_with(conn: RawFd, bytes: &mut [u8]) -> std::result::Result<OwnedFd, &'static str> {
    let want = bytes.len();
    let mut iov = libc::iovec {
        iov_base: bytes.as_mut_ptr().cast(),
        iov_len: want,
    };
    // SAFETY: a constant computation.
    let space = unsafe { libc::CMSG_SPACE(INT_BYTES) } as usize;
    let mut control = vec![0u8; space];
    // SAFETY: msghdr is plain data; all zeros is a valid value.
    let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
    msg.msg_iov = &raw mut iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.as_mut_ptr().cast();
    msg.msg_controllen = space;
    // SAFETY: msg describes buffers that live through the call.
    let got = unsafe { libc::recvmsg(conn, &raw mut msg, libc::MSG_CMSG_CLOEXEC) };
    if usize::try_from(got).ok() != Some(want) {
        return Err("sent no descriptor");
    }
    // Before the descriptor: a kernel that installed none writes no control message at all.
    let fd = take_fd(&msg);
    if msg.msg_flags & libc::MSG_CTRUNC != 0 {
        return Err("sent a descriptor this process could not take: it is at its open file limit");
    }
    fd.ok_or("sent no descriptor")
}

fn take_fd(msg: &libc::msghdr) -> Option<OwnedFd> {
    // SAFETY: msg was filled by recvmsg.
    let cmsg = unsafe { libc::CMSG_FIRSTHDR(msg) };
    if cmsg.is_null() {
        return None;
    }
    // SAFETY: cmsg points into control, written by recvmsg.
    let header = unsafe { &*cmsg };
    let (level, kind, len) = (header.cmsg_level, header.cmsg_type, header.cmsg_len);
    // SAFETY: a constant computation.
    let want = unsafe { libc::CMSG_LEN(INT_BYTES) } as usize;
    if level != libc::SOL_SOCKET || kind != libc::SCM_RIGHTS || len != want {
        return None;
    }
    // SAFETY: the int after the header, inside control.
    let data = unsafe { libc::CMSG_DATA(cmsg) };
    // SAFETY: the kernel wrote one int there, at any alignment.
    let fd = unsafe { data.cast::<c_int>().read_unaligned() };
    // SAFETY: SCM_RIGHTS installed a new descriptor in this process.
    (fd >= 0).then(|| unsafe { OwnedFd::from_raw_fd(fd) })
}

/// A connection to `endpoint` served by this user. `NotReady` when nobody answers: the publisher
/// went away after advertising its segment, or is stopped. Non-blocking, since a stopped
/// publisher's backlog fills and a blocking connect then never returns.
fn connected(endpoint: &str) -> Result<OwnedFd> {
    let sock = socket(libc::SOCK_NONBLOCK).map_err(|e| Error::io("socket", endpoint, e))?;
    if let Err(e) = connect(sock.as_raw_fd(), endpoint)? {
        return Err(match e.raw_os_error() {
            Some(libc::ECONNREFUSED | libc::ENOENT) => {
                Error::NotReady(format!("device endpoint '{endpoint}' is not served"))
            }
            Some(libc::EAGAIN) => Error::NotReady(format!(
                "device endpoint '{endpoint}' accepts no connection"
            )),
            _ => Error::io("connect", endpoint, e),
        });
    }
    if !peer_is_us(sock.as_raw_fd()) {
        return Err(Error::Mismatch(format!(
            "device endpoint '{endpoint}' is served by another user"
        )));
    }
    Ok(sock)
}

/// Up to `RECV_TIMEOUT_NS`. `SA_RESTART` never restarts `poll`, so after a signal it is called again
/// for what is left. Any other failure leaves `recv_fd_with` to find nothing.
fn wait_readable(sock: RawFd) {
    wait_readable_for(sock, RECV_TIMEOUT_NS);
}

fn wait_readable_for(sock: RawFd, timeout_ns: u64) {
    let deadline = sys::monotonic_ns() + timeout_ns;
    loop {
        let left = deadline
            .saturating_sub(sys::monotonic_ns())
            .div_ceil(1_000_000);
        let mut ready = libc::pollfd {
            fd: sock,
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: one valid pollfd.
        let r = unsafe { libc::poll(&raw mut ready, 1, c_int::try_from(left).unwrap_or(0)) };
        if r >= 0 || left == 0 || io::Error::last_os_error().raw_os_error() != Some(libc::EINTR) {
            return;
        }
    }
}

/// Waits on the listening socket and an eventfd together, so stopping is a write rather than a
/// timeout. A forked child's copy has no thread to join and only closes its descriptors.
pub(crate) struct FdServer {
    _listen: OwnedFd,
    stop: OwnedFd,
    thread: Option<JoinHandle<()>>,
    born: u32,
    path: String,
    ino: u64,
}

/// What an `FdServer` hands out: a growing list of chunks of `chunk` bytes (a GPU arena's, a device
/// payload's), from the index each peer asks for. A peer checks the size against its own.
#[derive(Clone)]
pub(crate) struct Share {
    pub(crate) list: Arc<PiMutex<Vec<OwnedFd>>>,
    pub(crate) chunk: usize,
}

/// The bytes before a chunk list: how many chunks there are and their size in MiB, little endian.
const CHUNKS_HEADER: usize = 8;
/// How long the server waits for a peer's request: a stuck peer holds the one serving thread.
const REQUEST_WAIT_NS: u64 = 100_000_000;

impl FdServer {
    pub(crate) fn start(endpoint: &str, share: Share) -> Result<FdServer> {
        let listen = socket(0).map_err(|e| Error::io("socket", endpoint, e))?;
        let ino = bind_listening(listen.as_raw_fd(), endpoint)?;
        // SAFETY: plain syscall.
        let stop = unsafe { libc::eventfd(0, libc::EFD_CLOEXEC) };
        if stop < 0 {
            return Err(Error::io("eventfd", endpoint, io::Error::last_os_error()));
        }
        // SAFETY: a new descriptor this process owns.
        let stop = unsafe { OwnedFd::from_raw_fd(stop) };
        let (l, s) = (listen.as_raw_fd(), stop.as_raw_fd());
        // Both descriptors and `share` outlive the thread: Drop joins it before they close.
        let thread = sys::spawn_helper(
            std::thread::Builder::new().name("flux-gpu-fd".into()),
            move || {
                serve(l, s, &share);
            },
        )
        .map_err(|e| Error::io("spawn", endpoint, e))?;
        Ok(FdServer {
            _listen: listen,
            stop,
            thread: Some(thread),
            born: sys::fork_gen(),
            path: endpoint.to_owned(),
            ino,
        })
    }
}

fn serve(listen: RawFd, stop: RawFd, share: &Share) {
    loop {
        let mut fds = [
            libc::pollfd {
                fd: listen,
                events: libc::POLLIN,
                revents: 0,
            },
            libc::pollfd {
                fd: stop,
                events: libc::POLLIN,
                revents: 0,
            },
        ];
        // Fails only with EINTR or ENOMEM, both passing; returning would leave the socket
        // listening with nobody to answer.
        // SAFETY: two valid pollfds.
        if unsafe { libc::poll(fds.as_mut_ptr(), 2, -1) } < 0 {
            continue;
        }
        if fds[1].revents != 0 {
            return;
        }
        if fds[0].revents & libc::POLLIN == 0 {
            continue;
        }
        // SAFETY: a listening socket; no peer address wanted.
        let conn = unsafe {
            libc::accept4(
                listen,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                libc::SOCK_CLOEXEC,
            )
        };
        if conn < 0 {
            continue;
        }
        // SAFETY: a new descriptor this thread owns.
        let conn = unsafe { OwnedFd::from_raw_fd(conn) };
        if !peer_is_us(conn.as_raw_fd()) {
            continue;
        }
        send_chunks(conn.as_raw_fd(), &share.list, share.chunk);
    }
}

/// Reads the index the peer asks from, then sends the header and each chunk from there with its
/// index. A peer that asks nothing in time gets nothing.
fn send_chunks(conn: RawFd, list: &PiMutex<Vec<OwnedFd>>, chunk: usize) {
    let mut ask = [0u8; 4];
    wait_readable_for(conn, REQUEST_WAIT_NS);
    // SAFETY: ask is 4 writable bytes.
    let got = unsafe { libc::recv(conn, ask.as_mut_ptr().cast(), 4, libc::MSG_DONTWAIT) };
    if got != 4 {
        return;
    }
    let first = u32::from_le_bytes(ask) as usize;
    let list = list.lock();
    #[allow(clippy::cast_possible_truncation)]
    let (count, mib) = (list.len() as u32, (chunk >> 20) as u32);
    let mut header = [0u8; CHUNKS_HEADER];
    header[..4].copy_from_slice(&count.to_le_bytes());
    header[4..].copy_from_slice(&mib.to_le_bytes());
    // SAFETY: header is CHUNKS_HEADER readable bytes.
    let sent = unsafe {
        libc::send(
            conn,
            header.as_ptr().cast(),
            CHUNKS_HEADER,
            libc::MSG_NOSIGNAL,
        )
    };
    if usize::try_from(sent).ok() != Some(CHUNKS_HEADER) {
        return;
    }
    for (i, fd) in list.iter().enumerate().skip(first) {
        #[allow(clippy::cast_possible_truncation)]
        if !send_fd_with(conn, fd.as_raw_fd(), &(i as u32).to_le_bytes()) {
            return;
        }
    }
}

/// The chunks `endpoint` serves (a GPU arena or a device payload), `chunk` bytes each, from index
/// `first`, in order. `NotReady` as `receive` is.
pub(crate) fn fetch_chunks(endpoint: &str, first: u32, chunk: usize) -> Result<Vec<OwnedFd>> {
    let sock = connected(endpoint)?;
    let conn = sock.as_raw_fd();
    let ask = first.to_le_bytes();
    // SAFETY: ask is 4 readable bytes.
    if unsafe { libc::send(conn, ask.as_ptr().cast(), 4, libc::MSG_NOSIGNAL) } != 4 {
        return Err(Error::NotReady(format!(
            "device endpoint '{endpoint}' took no request"
        )));
    }
    let mut header = [0u8; CHUNKS_HEADER];
    wait_readable(conn);
    // SAFETY: header is CHUNKS_HEADER writable bytes.
    let got = unsafe { libc::recv(conn, header.as_mut_ptr().cast(), CHUNKS_HEADER, 0) };
    if usize::try_from(got).ok() != Some(CHUNKS_HEADER) {
        return Err(Error::NotReady(format!(
            "device endpoint '{endpoint}' sent no chunk list"
        )));
    }
    let (count, mib) = (
        u32::from_le_bytes([header[0], header[1], header[2], header[3]]),
        u32::from_le_bytes([header[4], header[5], header[6], header[7]]),
    );
    if mib as usize != chunk >> 20 {
        return Err(Error::Mismatch(format!(
            "device endpoint '{endpoint}' serves {mib} MiB chunks, this end expects {} MiB",
            chunk >> 20
        )));
    }
    let mut out = Vec::new();
    for want in first..count {
        let mut index = [0u8; 4];
        wait_readable(conn);
        let fd = recv_fd_with(conn, &mut index)
            .map_err(|why| Error::NotReady(format!("device endpoint '{endpoint}' {why}")))?;
        if u32::from_le_bytes(index) != want {
            return Err(Error::NotReady(format!(
                "device endpoint '{endpoint}' sent chunk {} for {want}",
                u32::from_le_bytes(index)
            )));
        }
        out.push(fd);
    }
    Ok(out)
}

impl Drop for FdServer {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let one = 1u64;
        // SAFETY: an eventfd takes an 8-byte count.
        unsafe { libc::write(self.stop.as_raw_fd(), (&raw const one).cast(), 8) };
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        if file_ino(&self.path) == Some(self.ino) {
            unlink(&self.path);
        }
    }
}

/// Binds under a name of this process's, listens, then renames into place, so the endpoint names
/// only a socket that answers; the rename also replaces one a dead publisher of a reused inode
/// left. Returns the file's inode, by which Drop knows the file is still its own.
fn bind_listening(listen: RawFd, endpoint: &str) -> Result<u64> {
    let tmp = format!("{endpoint}.{}", sys::getpid());
    let (addr, len) = path_addr(&tmp)?;
    unlink(&tmp);
    // SAFETY: addr and len describe a sockaddr_un.
    if unsafe { libc::bind(listen, (&raw const addr).cast(), len) } != 0 {
        return Err(Error::io("bind", endpoint, io::Error::last_os_error()));
    }
    let placed = (|| {
        let c = cpath(&tmp)?;
        // SAFETY: a valid C string; the socket file just bound.
        if unsafe { libc::chmod(c.as_ptr(), 0o600) } != 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: a bound socket.
        if unsafe { libc::listen(listen, 16) } != 0 {
            return Err(io::Error::last_os_error());
        }
        std::fs::rename(&tmp, endpoint)
    })();
    if let Err(e) = placed {
        unlink(&tmp);
        return Err(Error::io("listen", endpoint, e));
    }
    file_ino(endpoint).ok_or_else(|| Error::io("stat", endpoint, io::Error::last_os_error()))
}

fn cpath(path: &str) -> io::Result<std::ffi::CString> {
    std::ffi::CString::new(path).map_err(|_| io::Error::from(io::ErrorKind::InvalidInput))
}

fn file_ino(path: &str) -> Option<u64> {
    use std::os::unix::fs::MetadataExt;
    std::fs::symlink_metadata(path).ok().map(|m| m.ino())
}

fn unlink(path: &str) {
    let _ = std::fs::remove_file(path);
}

/// For the sweep: removes `name` in /dev/shm if it is this user's endpoint and dead. A renamed
/// endpoint is dead when nothing listens on it. A name still carrying its binder's pid is renamed
/// microseconds after the bind, so one that old is left over; the pid itself may be another pid
/// namespace's.
pub(crate) fn remove_dead_endpoint(name: &str) {
    use std::os::unix::fs::{FileTypeExt, MetadataExt};
    let Some(rest) = name.strip_prefix(ENDPOINT_PREFIX) else {
        return;
    };
    let path = format!("/dev/shm/{name}");
    let Ok(meta) = std::fs::symlink_metadata(&path) else {
        return;
    };
    if !meta.file_type().is_socket() || meta.uid() != sys::euid() {
        return;
    }
    let dead = if rest.split('.').nth(2).is_some() {
        meta.modified()
            .ok()
            .and_then(|t| t.elapsed().ok())
            .is_some_and(|age| age > STALE_BIND)
    } else {
        socket(libc::SOCK_NONBLOCK).is_ok_and(|s| {
            let r = connect(s.as_raw_fd(), &path);
            matches!(r, Ok(Err(e)) if e.raw_os_error() == Some(libc::ECONNREFUSED))
        })
    };
    if dead && file_ino(&path) == Some(meta.ino()) {
        unlink(&path);
    }
}

#[cfg(test)]
mod tests {
    use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
    use std::time::Duration;

    use std::sync::Arc;

    use super::{
        bind_listening, fetch_chunks, recv_fd_with, send_chunks, send_fd_with, socket, unlink,
    };
    use crate::storm::{caught_so_far, in_storm};
    use crate::sys::PiMutex;

    /// A publisher slow to answer leaves the subscriber in `poll` while signals arrive.
    #[test]
    fn a_descriptor_crosses_a_signal_storm() {
        if !in_storm("gpu::fd::tests::a_descriptor_crosses_a_signal_storm") {
            return;
        }
        let endpoint = format!("/dev/shm/flux.gpu.test.{}", std::process::id());
        let listen = socket(0).unwrap();
        bind_listening(listen.as_raw_fd(), &endpoint).unwrap();
        let share = OwnedFd::from(std::fs::File::open("/proc/self/exe").unwrap());
        let list = Arc::new(PiMutex::new(vec![share]));
        let l = listen.as_raw_fd();
        let server = crate::sys::spawn_helper(std::thread::Builder::new(), move || {
            // SAFETY: a listening socket; no peer address wanted.
            let conn = unsafe {
                libc::accept4(
                    l,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                    libc::SOCK_CLOEXEC,
                )
            };
            // SAFETY: a new descriptor this thread owns, or -1, which the send refuses.
            let conn = unsafe { OwnedFd::from_raw_fd(conn) };
            std::thread::sleep(Duration::from_millis(50));
            send_chunks(conn.as_raw_fd(), &list, 2 << 20);
        })
        .unwrap();
        let before = caught_so_far();
        let got = fetch_chunks(&endpoint, 0, 2 << 20);
        server.join().unwrap();
        unlink(&endpoint);
        assert_eq!(got.map(|v| v.len()).ok(), Some(1));
        assert!(caught_so_far() > before + 10, "no storm");
    }

    /// At its open file limit a receiver gets the message without the descriptor. The error says
    /// so, rather than that none was sent.
    #[test]
    fn a_descriptor_dropped_at_the_open_file_limit_is_named() {
        let mut pair = [0; 2];
        // SAFETY: pair has room for two descriptors.
        let rc = unsafe {
            libc::socketpair(
                libc::AF_UNIX,
                libc::SOCK_STREAM | libc::SOCK_CLOEXEC,
                0,
                pair.as_mut_ptr(),
            )
        };
        assert_eq!(rc, 0);
        let shared = std::fs::File::open("/dev/null").unwrap();
        assert!(send_fd_with(pair[0], shared.as_raw_fd(), &[0; 4]));
        // SAFETY: the child sets its own limit, receives and exits without unwinding.
        let pid = unsafe { crate::sys::test_fork() };
        if pid == 0 {
            // SAFETY: dup returns the lowest free number, which then becomes the limit.
            let lowest = unsafe { libc::dup(0) };
            // SAFETY: closes the probe just made.
            unsafe { libc::close(lowest) };
            let mut limit = libc::rlimit {
                rlim_cur: 0,
                rlim_max: 0,
            };
            // SAFETY: a valid out-pointer.
            unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &raw mut limit) };
            limit.rlim_cur = libc::rlim_t::try_from(lowest).unwrap();
            // SAFETY: a valid rlimit, lowering this process's own.
            let rc = unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &raw const limit) };
            assert_eq!(rc, 0);
            let named = recv_fd_with(pair[1], &mut [0; 4])
                .is_err_and(|why| why.contains("open file limit"));
            // SAFETY: ends the child without running the parent's destructors.
            unsafe { libc::_exit(i32::from(!named)) };
        }
        let mut status = 0;
        // SAFETY: reaps our own child.
        unsafe { libc::waitpid(pid, &raw mut status, 0) };
        for fd in pair {
            // SAFETY: our socket ends.
            unsafe { libc::close(fd) };
        }
        assert!(
            libc::WIFEXITED(status) && libc::WEXITSTATUS(status) == 0,
            "{status:#x}"
        );
    }
}
