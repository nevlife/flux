//! The device-handle route: one VMM allocation for the whole payload area, exported once as a
//! POSIX fd and handed to each subscriber over an `AF_UNIX` socket file beside the segment, one
//! accept thread per channel. Both ends also check that the peer is this user before the
//! descriptor moves.

use std::ffi::{c_int, c_void};
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::thread::JoinHandle;

use super::driver::{
    AllocFlags, AllocHandle, Core, DevicePtr, Driver, MEM_ACCESS_READ, MEM_ACCESS_READ_WRITE,
    MEM_ALLOCATION_TYPE_PINNED, MEM_GRANULARITY_MINIMUM, MEM_HANDLE_TYPE_POSIX_FD,
    MEM_LOCATION_TYPE_DEVICE, MemAccessDesc, MemAllocationProp, MemLocation, SUCCESS, Vmm,
};
use super::ledger::{self, Held};
use crate::error::{Error, Result};
use crate::segment::SegmentId;
use crate::sys;

const RECV_TIMEOUT_NS: u64 = 1_000_000_000;

const ENDPOINT_PREFIX: &str = "flux2.gpu.";
const STALE_BIND: std::time::Duration = std::time::Duration::from_secs(10);

/// In /dev/shm, so whoever sees the segment sees its endpoint: an abstract name would follow the
/// network namespace instead. Derived from the segment object, so a restarted publisher's new
/// segment has a new endpoint.
pub(crate) fn endpoint(id: SegmentId) -> String {
    format!("/dev/shm/{ENDPOINT_PREFIX}{}.{}", id.dev, id.ino)
}

fn vmm() -> Result<(&'static Core, &'static Vmm)> {
    let drv = Driver::get();
    let core = drv
        .usable_core()
        .map_err(|why| Error::Unsupported(why.to_owned()))?;
    let vmm = drv
        .vmm
        .as_ref()
        .map_err(|why| Error::Unsupported((*why).to_owned()))?;
    Ok((core, vmm))
}

fn device_handle(core: &Core, ordinal: i32) -> Option<c_int> {
    let mut h = 0;
    // SAFETY: h is a valid out-pointer.
    (unsafe { (core.device_get)(&raw mut h, ordinal) } == SUCCESS).then_some(h)
}

pub(crate) fn device_uuid(ordinal: i32) -> Option<[u8; 16]> {
    let core = Driver::get().core.as_ref().ok()?;
    let h = device_handle(core, ordinal)?;
    let mut uuid = [0u8; 16];
    // SAFETY: uuid is 16 writable bytes, a CUuuid.
    (unsafe { (core.device_uuid)(&raw mut uuid, h) } == SUCCESS).then_some(uuid)
}

/// This process's ordinal for the device the publisher allocated on: ordinals differ between
/// processes that see different `CUDA_VISIBLE_DEVICES`, and importing on the wrong device fails or
/// reads another GPU's memory.
pub(crate) fn device_for_uuid(uuid: &[u8; 16]) -> Option<i32> {
    #[cfg(test)]
    if let Some(d) = super::tests::UUID_DEVICE.get() {
        return Some(d);
    }
    let core = Driver::get().core.as_ref().ok()?;
    let mut count = 0;
    // SAFETY: count is a valid out-pointer.
    if unsafe { (core.device_count)(&raw mut count) } != SUCCESS {
        return None;
    }
    (0..count).find(|&d| device_uuid(d).as_ref() == Some(uuid))
}

fn pinned_on(device: c_int) -> MemAllocationProp {
    MemAllocationProp {
        kind: MEM_ALLOCATION_TYPE_PINNED,
        requested_handle_types: MEM_HANDLE_TYPE_POSIX_FD,
        location: MemLocation {
            kind: MEM_LOCATION_TYPE_DEVICE,
            id: device,
        },
        win32_handle_metadata: std::ptr::null_mut(),
        alloc_flags: AllocFlags::default(),
    }
}

/// The minimum granularity binds the allocation's size only, not slot offsets: `PAYLOAD_ALIGN`
/// and the layout arithmetic stay as they are.
fn rounded(vmm: &Vmm, device: c_int, bytes: usize) -> Result<usize> {
    let prop = pinned_on(device);
    let mut g = 0;
    // SAFETY: valid pointers.
    if unsafe { (vmm.granularity)(&raw mut g, &raw const prop, MEM_GRANULARITY_MINIMUM) } != SUCCESS
        || g == 0
    {
        return Err(Error::Unsupported(
            "cuMemGetAllocationGranularity failed".into(),
        ));
    }
    Ok(bytes.div_ceil(g) * g)
}

/// One allocation, mapped. Comes apart in the order it was built: unmap, free the range, release.
struct Region {
    addr: DevicePtr,
    alloc: AllocHandle,
    size: usize,
    ctx: usize,
    born: u32,
}

impl Region {
    fn new(ctx: *mut c_void, size: usize) -> Region {
        Region {
            addr: 0,
            alloc: 0,
            size,
            ctx: ctx as usize,
            born: sys::fork_gen(),
        }
    }

    /// Reserves a range for `alloc` and lets `device` reach it with `access`.
    fn map(&mut self, vmm: &Vmm, device: c_int, access: c_int) -> Result<()> {
        // SAFETY: valid out-pointer; no fixed address, default alignment.
        if unsafe { (vmm.address_reserve)(&raw mut self.addr, self.size, 0, 0, 0) } != SUCCESS {
            self.addr = 0;
            return Err(Error::Unsupported("cuMemAddressReserve failed".into()));
        }
        // SAFETY: the range was reserved above at the allocation's size.
        if unsafe { (vmm.map)(self.addr, self.size, 0, self.alloc, 0) } != SUCCESS {
            // SAFETY: reserved above, nothing mapped.
            unsafe { (vmm.address_free)(self.addr, self.size) };
            self.addr = 0;
            return Err(Error::Unsupported("cuMemMap failed".into()));
        }
        let desc = MemAccessDesc {
            location: MemLocation {
                kind: MEM_LOCATION_TYPE_DEVICE,
                id: device,
            },
            flags: access,
        };
        // SAFETY: a mapped range and one valid descriptor.
        if unsafe { (vmm.set_access)(self.addr, self.size, &raw const desc, 1) } != SUCCESS {
            return Err(Error::Unsupported("cuMemSetAccess failed".into()));
        }
        Ok(())
    }
}

impl Drop for Region {
    fn drop(&mut self) {
        if sys::fork_gen() != self.born {
            return;
        }
        let Ok((core, vmm)) = vmm() else {
            return;
        };
        core.with_context(self.ctx as *mut c_void, || {
            // Each object this region made, released once, in reverse order.
            if self.addr != 0 {
                // SAFETY: mapped by map() at this size.
                unsafe { (vmm.unmap)(self.addr, self.size) };
                // SAFETY: reserved by map() at this size, now unmapped.
                unsafe { (vmm.address_free)(self.addr, self.size) };
            }
            if self.alloc != 0 {
                // SAFETY: created or imported by this region.
                unsafe { (vmm.release)(self.alloc) };
            }
        });
    }
}

/// The publisher's allocation. The server is dropped first, so the descriptor stops being handed
/// out before it is closed.
pub(crate) struct DeviceAlloc {
    _server: FdServer,
    fd: OwnedFd,
    region: Region,
}

impl DeviceAlloc {
    /// Serves `endpoint` before returning, so a segment advertised after this has an answer.
    pub(crate) fn create(endpoint: &str, bytes: usize, ordinal: i32) -> Result<DeviceAlloc> {
        let (core, vmm) = vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        sys::watch_fork();
        let mut region = Region::new(ctx, rounded(vmm, device, bytes)?);
        let prop = pinned_on(device);
        // SAFETY: valid pointers; size is a multiple of the granularity.
        if unsafe { (vmm.create)(&raw mut region.alloc, region.size, &raw const prop, 0) }
            != SUCCESS
        {
            region.alloc = 0;
            return Err(Error::Unsupported("cuMemCreate failed".into()));
        }
        region.map(vmm, device, MEM_ACCESS_READ_WRITE)?;
        let mut raw: c_int = -1;
        // SAFETY: a POSIX fd handle is written as an int.
        if unsafe {
            (vmm.export)(
                (&raw mut raw).cast(),
                region.alloc,
                MEM_HANDLE_TYPE_POSIX_FD,
                0,
            )
        } != SUCCESS
        {
            return Err(Error::Unsupported(
                "cuMemExportToShareableHandle failed".into(),
            ));
        }
        // SAFETY: the export handed this process a new descriptor.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        // The driver sets it today without saying so; an exec'd child holding it keeps the
        // allocation alive and writable.
        // SAFETY: fcntl on a descriptor this process owns.
        unsafe { libc::fcntl(fd.as_raw_fd(), libc::F_SETFD, libc::FD_CLOEXEC) };
        let server = FdServer::start(endpoint, fd.as_raw_fd())?;
        ledger::made(Held::Export(region.addr), region.size);
        ledger::made(Held::Fd(fd.as_raw_fd()), 0);
        Ok(DeviceAlloc {
            _server: server,
            fd,
            region,
        })
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.region.addr
    }
}

impl Drop for DeviceAlloc {
    fn drop(&mut self) {
        ledger::undone(Held::Fd(self.fd.as_raw_fd()), 0);
        ledger::undone(Held::Export(self.region.addr), self.region.size);
    }
}

/// A subscriber's mapping of the publisher's allocation, readable only: a consuming kernel cannot
/// write the slot.
pub(crate) struct DeviceImport {
    region: Region,
}

impl DeviceImport {
    pub(crate) fn open(endpoint: &str, bytes: usize, ordinal: i32) -> Result<DeviceImport> {
        let (core, vmm) = vmm()?;
        let ctx = core
            .primary_context(ordinal)
            .ok_or_else(|| Error::Unsupported("the primary context was refused".into()))?;
        let device = device_handle(core, ordinal)
            .ok_or_else(|| Error::Unsupported("cuDeviceGet failed".into()))?;
        let fd = receive(endpoint)?;
        sys::watch_fork();
        let mut region = Region::new(ctx, rounded(vmm, device, bytes)?);
        // SAFETY: the handle is the descriptor, passed as a pointer-sized integer. The import
        // takes its own reference, so the descriptor is closed by this process either way.
        let rc = unsafe {
            (vmm.import)(
                &raw mut region.alloc,
                fd.as_raw_fd() as isize as *mut c_void,
                MEM_HANDLE_TYPE_POSIX_FD,
            )
        };
        drop(fd);
        if rc != SUCCESS {
            region.alloc = 0;
            return Err(Error::Unsupported(
                "cuMemImportFromShareableHandle failed".into(),
            ));
        }
        region.map(vmm, device, MEM_ACCESS_READ)?;
        ledger::made(Held::Import(region.addr), region.size);
        Ok(DeviceImport { region })
    }

    pub(crate) fn base(&self) -> DevicePtr {
        self.region.addr
    }
}

impl Drop for DeviceImport {
    fn drop(&mut self) {
        ledger::undone(Held::Import(self.region.addr), self.region.size);
    }
}

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

/// The descriptor rides in the control message; one data byte, since a message with no data
/// carries no control data either.
fn send_fd(conn: RawFd, fd: RawFd) -> bool {
    let mut byte = 0u8;
    let mut iov = libc::iovec {
        iov_base: (&raw mut byte).cast(),
        iov_len: 1,
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
    // SAFETY: msg describes buffers that live through the call.
    unsafe { libc::sendmsg(conn, &raw const msg, libc::MSG_NOSIGNAL) == 1 }
}

/// Exactly one descriptor, or why not: none came, or the kernel dropped it, as it does at this
/// process's open file limit.
fn recv_fd(conn: RawFd) -> std::result::Result<OwnedFd, &'static str> {
    let mut byte = 0u8;
    let mut iov = libc::iovec {
        iov_base: (&raw mut byte).cast(),
        iov_len: 1,
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
    if unsafe { libc::recvmsg(conn, &raw mut msg, libc::MSG_CMSG_CLOEXEC) } != 1 {
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

/// `NotReady` when nobody answers: the publisher went away after advertising its segment, or is
/// stopped. Non-blocking, since a stopped publisher's backlog fills and a blocking connect then
/// never returns.
fn receive(endpoint: &str) -> Result<OwnedFd> {
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
    wait_readable(sock.as_raw_fd());
    recv_fd(sock.as_raw_fd())
        .map_err(|why| Error::NotReady(format!("device endpoint '{endpoint}' {why}")))
}

/// Up to `RECV_TIMEOUT_NS`. `SA_RESTART` never restarts `poll`, so after a signal it is called again
/// for what is left. Any other failure leaves `recv_fd` to find nothing.
fn wait_readable(sock: RawFd) {
    let deadline = sys::monotonic_ns() + RECV_TIMEOUT_NS;
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
struct FdServer {
    _listen: OwnedFd,
    stop: OwnedFd,
    thread: Option<JoinHandle<()>>,
    born: u32,
    path: String,
    ino: u64,
}

impl FdServer {
    fn start(endpoint: &str, share: RawFd) -> Result<FdServer> {
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
            std::thread::Builder::new().name("flux2-gpu-fd".into()),
            move || {
                serve(l, s, share);
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

fn serve(listen: RawFd, stop: RawFd, share: RawFd) {
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
        if peer_is_us(conn.as_raw_fd()) {
            send_fd(conn.as_raw_fd(), share);
        }
    }
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

    use super::{bind_listening, receive, recv_fd, send_fd, socket, unlink};
    use crate::storm::{caught_so_far, in_storm};

    /// A publisher slow to answer leaves the subscriber in `poll` while signals arrive.
    #[test]
    fn a_descriptor_crosses_a_signal_storm() {
        if !in_storm("gpu::vmm::tests::a_descriptor_crosses_a_signal_storm") {
            return;
        }
        let endpoint = format!("/dev/shm/flux2.gpu.test.{}", std::process::id());
        let listen = socket(0).unwrap();
        bind_listening(listen.as_raw_fd(), &endpoint).unwrap();
        let share = std::fs::File::open("/proc/self/exe").unwrap();
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
            // SAFETY: a new descriptor this thread owns, or -1, which send_fd refuses.
            let conn = unsafe { OwnedFd::from_raw_fd(conn) };
            std::thread::sleep(Duration::from_millis(50));
            send_fd(conn.as_raw_fd(), share.as_raw_fd());
        })
        .unwrap();
        let before = caught_so_far();
        let got = receive(&endpoint);
        server.join().unwrap();
        unlink(&endpoint);
        assert!(got.is_ok(), "{:?}", got.err());
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
        assert!(send_fd(pair[0], shared.as_raw_fd()));
        // SAFETY: the child sets its own limit, receives and exits without unwinding.
        let pid = unsafe { libc::fork() };
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
            let named = recv_fd(pair[1]).is_err_and(|why| why.contains("open file limit"));
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
