//! Descriptors over `AF_UNIX` socket files in /dev/shm: a growing list of chunks (a device
//! payload's, a GPU arena's) handed to each peer that runs as this user, then pushed to it as they
//! are made; the receiving side; and the sweep of endpoints nobody serves.

use std::ffi::c_int;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};
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

/// The most descriptors one message carries: the kernel's `SCM_MAX_FD`.
const MAX_BATCH: usize = 253;

/// Control space for `MAX_BATCH` descriptors, aligned for a `cmsghdr`.
#[repr(C, align(8))]
struct Control([u8; 1040]);

/// Every message's data: the chunk size in MiB and the index of its first descriptor, little
/// endian. A message carries data, since one with none carries no control data either.
const HEAD: usize = 8;

fn head(chunk: usize, first: u32) -> [u8; HEAD] {
    #[allow(clippy::cast_possible_truncation)]
    let mib = (chunk >> 20) as u32;
    let mut h = [0u8; HEAD];
    h[..4].copy_from_slice(&mib.to_le_bytes());
    h[4..].copy_from_slice(&first.to_le_bytes());
    h
}

/// Sends `head` with `fds` (at most `MAX_BATCH`) without waiting: false when the peer is gone or
/// its socket is full.
fn send_batch(conn: RawFd, head: &[u8; HEAD], fds: &[RawFd]) -> bool {
    let mut iov = libc::iovec {
        iov_base: head.as_ptr().cast_mut().cast(),
        iov_len: HEAD,
    };
    let mut control = Control([0; 1040]);
    // SAFETY: msghdr is plain data; all zeros is a valid value.
    let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
    msg.msg_iov = &raw mut iov;
    msg.msg_iovlen = 1;
    if !fds.is_empty() {
        #[allow(clippy::cast_possible_truncation)]
        let bytes = fds.len().min(MAX_BATCH) as u32 * INT_BYTES;
        // SAFETY: a constant computation.
        let space = unsafe { libc::CMSG_SPACE(bytes) } as usize;
        assert!(space <= size_of::<Control>());
        msg.msg_control = control.0.as_mut_ptr().cast();
        msg.msg_controllen = space;
        // SAFETY: msg_control holds CMSG_SPACE(bytes).
        let cmsg = unsafe { libc::CMSG_FIRSTHDR(&raw const msg) };
        // SAFETY: cmsg points into control, which has room for the header and the ints.
        let header = unsafe { &mut *cmsg };
        header.cmsg_level = libc::SOL_SOCKET;
        header.cmsg_type = libc::SCM_RIGHTS;
        // SAFETY: a constant computation.
        header.cmsg_len = unsafe { libc::CMSG_LEN(bytes) } as usize;
        // SAFETY: the ints after the header, inside control.
        let data = unsafe { libc::CMSG_DATA(cmsg) };
        for (i, &fd) in fds.iter().take(MAX_BATCH).enumerate() {
            // SAFETY: the i-th int, inside control, which has room for MAX_BATCH.
            let at = unsafe { data.add(i * INT_BYTES as usize) };
            // SAFETY: room for an int there, at any alignment.
            unsafe { at.cast::<c_int>().write_unaligned(fd) };
        }
    }
    // SAFETY: msg describes buffers that live through the call; sendmsg only reads iov_base.
    let sent = unsafe {
        libc::sendmsg(
            conn,
            &raw const msg,
            libc::MSG_NOSIGNAL | libc::MSG_DONTWAIT,
        )
    };
    usize::try_from(sent).is_ok_and(|n| n == HEAD)
}

enum Got {
    Batch([u8; HEAD], Vec<OwnedFd>),
    /// Nothing there yet.
    Empty,
    /// The peer went or let this end go.
    Closed,
    Bad(&'static str),
}

/// One message, without waiting.
fn recv_batch(conn: RawFd) -> Got {
    let mut head = [0u8; HEAD];
    let mut iov = libc::iovec {
        iov_base: head.as_mut_ptr().cast(),
        iov_len: HEAD,
    };
    let mut control = Control([0; 1040]);
    // SAFETY: msghdr is plain data; all zeros is a valid value.
    let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
    msg.msg_iov = &raw mut iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.0.as_mut_ptr().cast();
    msg.msg_controllen = size_of::<Control>();
    // SAFETY: msg describes buffers that live through the call.
    let got = unsafe {
        libc::recvmsg(
            conn,
            &raw mut msg,
            libc::MSG_CMSG_CLOEXEC | libc::MSG_DONTWAIT,
        )
    };
    if got < 0 {
        let e = io::Error::last_os_error().raw_os_error();
        return if matches!(e, Some(libc::EAGAIN | libc::EINTR)) {
            Got::Empty
        } else {
            Got::Closed
        };
    }
    if got == 0 {
        return Got::Closed;
    }
    // Before any check: what came is this process's to close either way.
    let fds = take_fds(&msg);
    if msg.msg_flags & libc::MSG_CTRUNC != 0 {
        return Got::Bad(
            "sent descriptors this process could not take: it is at its open file limit",
        );
    }
    if usize::try_from(got).ok() != Some(HEAD) {
        return Got::Bad("sent a short message");
    }
    Got::Batch(head, fds)
}

fn take_fds(msg: &libc::msghdr) -> Vec<OwnedFd> {
    let mut out = Vec::new();
    // SAFETY: msg was filled by recvmsg.
    let mut cmsg = unsafe { libc::CMSG_FIRSTHDR(msg) };
    while !cmsg.is_null() {
        // SAFETY: cmsg points into control, written by recvmsg.
        let header = unsafe { &*cmsg };
        if header.cmsg_level == libc::SOL_SOCKET && header.cmsg_type == libc::SCM_RIGHTS {
            // SAFETY: a constant computation.
            let empty = unsafe { libc::CMSG_LEN(0) } as usize;
            let n = header.cmsg_len.saturating_sub(empty) / INT_BYTES as usize;
            // SAFETY: the ints after the header, inside control.
            let data = unsafe { libc::CMSG_DATA(cmsg) };
            for i in 0..n {
                // SAFETY: the i-th of the n ints the kernel wrote, inside control.
                let at = unsafe { data.add(i * INT_BYTES as usize) };
                // SAFETY: an int there, at any alignment.
                let fd = unsafe { at.cast::<c_int>().read_unaligned() };
                if fd >= 0 {
                    // SAFETY: SCM_RIGHTS installed a new descriptor in this process.
                    out.push(unsafe { OwnedFd::from_raw_fd(fd) });
                }
            }
        }
        // SAFETY: msg and cmsg are recvmsg's.
        cmsg = unsafe { libc::CMSG_NXTHDR(msg, cmsg) };
    }
    out
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

/// Up to `timeout_ns`. `SA_RESTART` never restarts `poll`, so after a signal it is called again for
/// what is left. Any other failure leaves the receive that follows to find nothing.
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
/// payload's). A peer asks once, from the index it has, and keeps the connection: each chunk made
/// after that is sent to it when it is made, before any frame can name it, so a reader never waits
/// for a publisher that is stopped or starved of CPU. Sends never wait either; a peer that cannot
/// take one now is let go, and connects again for the rest.
pub(crate) struct Share {
    chunk: usize,
    state: PiMutex<Chunks>,
    /// Chunks made, each sent to every peer before it is counted. Read by commits without the lock.
    count: AtomicU32,
}

struct Chunks {
    fds: Vec<OwnedFd>,
    peers: Vec<OwnedFd>,
}

/// How long the server waits for a peer's request: a stuck peer holds the one serving thread.
const REQUEST_WAIT_NS: u64 = 100_000_000;

impl Share {
    pub(crate) fn new(chunk: usize) -> Arc<Share> {
        Arc::new(Share {
            chunk,
            state: PiMutex::new(Chunks {
                fds: Vec::new(),
                peers: Vec::new(),
            }),
            count: AtomicU32::new(0),
        })
    }

    pub(crate) fn count(&self) -> u32 {
        self.count.load(Ordering::Acquire) // order: chunk_count
    }

    /// Adds a chunk and sends it to every peer.
    pub(crate) fn push(&self, fd: OwnedFd) {
        let mut s = self.state.lock();
        #[allow(clippy::cast_possible_truncation)]
        let i = s.fds.len() as u32;
        let h = head(self.chunk, i);
        let raw = fd.as_raw_fd();
        s.peers
            .retain(|p| send_batch(p.as_raw_fd(), &h, &[raw]) || let_go(p.as_raw_fd()));
        s.fds.push(fd);
        #[allow(clippy::cast_possible_truncation)]
        let n = s.fds.len() as u32;
        self.count.store(n, Ordering::Release); // order: chunk_count
    }

    /// Sends `conn` the chunks from `first` on, in as few messages as the kernel allows, and keeps
    /// it for the ones to come. Peers that hung up go first.
    fn admit(&self, conn: OwnedFd, first: u32) {
        let mut s = self.state.lock();
        s.peers.retain(|p| !hung_up(p.as_raw_fd()));
        let from = (first as usize).min(s.fds.len());
        let rest: Vec<RawFd> = s.fds.iter().skip(from).map(AsRawFd::as_raw_fd).collect();
        let sent = if rest.is_empty() {
            #[allow(clippy::cast_possible_truncation)]
            send_batch(conn.as_raw_fd(), &head(self.chunk, from as u32), &[])
        } else {
            rest.chunks(MAX_BATCH).enumerate().all(|(k, b)| {
                #[allow(clippy::cast_possible_truncation)]
                let at = (from + k * MAX_BATCH) as u32;
                send_batch(conn.as_raw_fd(), &head(self.chunk, at), b)
            })
        };
        if sent {
            s.peers.push(conn);
        } else {
            let_go(conn.as_raw_fd());
        }
    }
}

/// Ends the connection itself, not only this descriptor of it: a child forked meanwhile holds a
/// copy, and the peer would otherwise never see it end and never come back. False, for `retain`.
fn let_go(conn: RawFd) -> bool {
    // SAFETY: a socket this process owns.
    unsafe { libc::shutdown(conn, libc::SHUT_RDWR) };
    false
}

fn hung_up(conn: RawFd) -> bool {
    let mut p = libc::pollfd {
        fd: conn,
        events: 0,
        revents: 0,
    };
    // SAFETY: one valid pollfd; no wait.
    let r = unsafe { libc::poll(&raw mut p, 1, 0) };
    r != 0 && p.revents & (libc::POLLHUP | libc::POLLERR) != 0
}

/// A subscriber's end of an endpoint: one connection, kept, on which the publisher sends each chunk
/// as it makes it. Only a connection's first use waits, for the chunks made before it was served.
pub(crate) struct Mailbox {
    endpoint: String,
    chunk: usize,
    conn: Option<OwnedFd>,
    waited: bool,
}

impl Mailbox {
    pub(crate) fn new(endpoint: &str, chunk: usize) -> Mailbox {
        Mailbox {
            endpoint: endpoint.to_owned(),
            chunk,
            conn: None,
            waited: false,
        }
    }

    /// The chunks from index `have` on that have come; a first use waits up to `RECV_TIMEOUT_NS`
    /// for `want` of them in all. `NotReady` when nobody serves the endpoint; fewer than `want`
    /// otherwise, and the caller refuses the frame.
    pub(crate) fn collect(&mut self, have: u32, want: u32) -> Result<Vec<OwnedFd>> {
        let conn = if let Some(c) = &self.conn {
            c.as_raw_fd()
        } else {
            let sock = connected(&self.endpoint)?;
            let ask = have.to_le_bytes();
            // SAFETY: ask is 4 readable bytes.
            let sent =
                unsafe { libc::send(sock.as_raw_fd(), ask.as_ptr().cast(), 4, libc::MSG_NOSIGNAL) };
            if sent != 4 {
                return Err(Error::NotReady(format!(
                    "device endpoint '{}' took no request",
                    self.endpoint
                )));
            }
            self.waited = false;
            self.conn.insert(sock).as_raw_fd()
        };
        let deadline = if self.waited {
            0
        } else {
            sys::monotonic_ns() + RECV_TIMEOUT_NS
        };
        self.waited = true;
        let mut out = Vec::new();
        loop {
            #[allow(clippy::cast_possible_truncation)]
            let at = have + out.len() as u32;
            match recv_batch(conn) {
                Got::Batch(h, fds) => {
                    let mib = u32::from_le_bytes([h[0], h[1], h[2], h[3]]);
                    let first = u32::from_le_bytes([h[4], h[5], h[6], h[7]]);
                    if mib as usize != self.chunk >> 20 {
                        self.conn = None;
                        return Err(Error::Mismatch(format!(
                            "device endpoint '{}' serves {mib} MiB chunks, this end expects {} MiB",
                            self.endpoint,
                            self.chunk >> 20
                        )));
                    }
                    if first != at {
                        self.conn = None;
                        return Err(Error::NotReady(format!(
                            "device endpoint '{}' sent chunk {first} for {at}",
                            self.endpoint
                        )));
                    }
                    out.extend(fds);
                }
                Got::Empty => {
                    if at >= want || sys::monotonic_ns() >= deadline {
                        return Ok(out);
                    }
                    wait_readable_for(conn, deadline.saturating_sub(sys::monotonic_ns()));
                }
                Got::Closed => {
                    self.conn = None;
                    return Ok(out);
                }
                Got::Bad(why) => {
                    self.conn = None;
                    return Err(Error::NotReady(format!(
                        "device endpoint '{}' {why}",
                        self.endpoint
                    )));
                }
            }
        }
    }
}

impl FdServer {
    pub(crate) fn start(endpoint: &str, share: Arc<Share>) -> Result<FdServer> {
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
        if let Some(first) = request(conn.as_raw_fd()) {
            share.admit(conn, first);
        }
    }
}

/// The index the peer asks from. A peer that asks nothing in time gets nothing.
fn request(conn: RawFd) -> Option<u32> {
    let mut ask = [0u8; 4];
    wait_readable_for(conn, REQUEST_WAIT_NS);
    // SAFETY: ask is 4 writable bytes.
    let got = unsafe { libc::recv(conn, ask.as_mut_ptr().cast(), 4, libc::MSG_DONTWAIT) };
    (got == 4).then(|| u32::from_le_bytes(ask))
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
    use std::sync::Arc;
    use std::time::Duration;

    use super::{
        FdServer, Got, Mailbox, Share, bind_listening, recv_batch, request, send_batch, socket,
        unlink,
    };
    use crate::error::Error;
    use crate::storm::{caught_so_far, in_storm};

    const CHUNK: usize = 2 << 20;

    fn null() -> OwnedFd {
        OwnedFd::from(std::fs::File::open("/dev/null").unwrap())
    }

    fn endpoint(name: &str) -> String {
        format!("/dev/shm/flux.gpu.test-{name}-{}", std::process::id())
    }

    /// A publisher slow to answer leaves the subscriber in `poll` while signals arrive.
    #[test]
    fn a_descriptor_crosses_a_signal_storm() {
        if !in_storm("gpu::fd::tests::a_descriptor_crosses_a_signal_storm") {
            return;
        }
        let endpoint = endpoint("storm");
        let listen = socket(0).unwrap();
        bind_listening(listen.as_raw_fd(), &endpoint).unwrap();
        let share = Share::new(CHUNK);
        share.push(null());
        let l = listen.as_raw_fd();
        let s = Arc::clone(&share);
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
            // SAFETY: a new descriptor this thread owns, or -1, which the request refuses.
            let conn = unsafe { OwnedFd::from_raw_fd(conn) };
            std::thread::sleep(Duration::from_millis(50));
            let first = request(conn.as_raw_fd()).unwrap();
            s.admit(conn, first);
        })
        .unwrap();
        let before = caught_so_far();
        let got = Mailbox::new(&endpoint, CHUNK).collect(0, 1);
        server.join().unwrap();
        unlink(&endpoint);
        assert_eq!(got.map(|v| v.len()).ok(), Some(1));
        assert!(caught_so_far() > before + 10, "no storm");
    }

    /// A chunk made after a peer was served reaches it with no server: the publisher sends it.
    #[test]
    fn a_chunk_made_later_comes_without_asking() {
        let endpoint = endpoint("later");
        let share = Share::new(CHUNK);
        share.push(null());
        let server = FdServer::start(&endpoint, Arc::clone(&share)).unwrap();
        let mut mail = Mailbox::new(&endpoint, CHUNK);
        assert_eq!(mail.collect(0, 1).unwrap().len(), 1);
        drop(server);
        share.push(null());
        share.push(null());
        let t = std::time::Instant::now();
        assert_eq!(mail.collect(1, 3).unwrap().len(), 2);
        assert!(
            t.elapsed() < Duration::from_millis(100),
            "{:?}",
            t.elapsed()
        );
        assert!(
            mail.collect(3, 4).unwrap().is_empty(),
            "nothing more was made"
        );
    }

    /// More chunks than a socket holds: the peer is let go when one does not fit, and connects
    /// again for the rest, which come in few messages. A child forked meanwhile holds a copy of
    /// the publisher's end, which must not keep the connection open.
    #[test]
    fn a_peer_that_falls_behind_comes_back_for_the_rest() {
        let endpoint = endpoint("behind");
        let share = Share::new(CHUNK);
        share.push(null());
        let _server = FdServer::start(&endpoint, Arc::clone(&share)).unwrap();
        let mut mail = Mailbox::new(&endpoint, CHUNK);
        assert_eq!(mail.collect(0, 1).unwrap().len(), 1);
        // SAFETY: the child only sleeps and exits without unwinding.
        let pid = unsafe { crate::sys::test_fork() };
        if pid == 0 {
            std::thread::sleep(Duration::from_secs(3));
            // SAFETY: ends the child without running the parent's destructors.
            unsafe { libc::_exit(0) };
        }
        for _ in 0..600 {
            share.push(null());
        }
        let mut have = 1u32;
        for _ in 0..4 {
            #[allow(clippy::cast_possible_truncation)]
            let got = mail.collect(have, 601).unwrap().len() as u32;
            have += got;
        }
        // SAFETY: our own child.
        unsafe { libc::kill(pid, libc::SIGKILL) };
        // SAFETY: reaps our own child.
        unsafe { libc::waitpid(pid, std::ptr::null_mut(), 0) };
        assert_eq!(have, 601);
    }

    #[test]
    fn a_chunk_size_of_another_build_is_refused() {
        let endpoint = endpoint("size");
        let share = Share::new(CHUNK);
        let _server = FdServer::start(&endpoint, share).unwrap();
        let got = Mailbox::new(&endpoint, 2 * CHUNK).collect(0, 1);
        assert!(matches!(got, Err(Error::Mismatch(_))), "{got:?}");
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
        assert!(send_batch(pair[0], &[0; 8], &[shared.as_raw_fd()]));
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
            let named =
                matches!(recv_batch(pair[1]), Got::Bad(why) if why.contains("open file limit"));
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
