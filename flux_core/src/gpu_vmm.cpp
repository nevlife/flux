#include "flux/gpu_vmm.hpp"

#include "cuda_driver.hpp"
#include "flux/segment.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#if defined(__linux__)
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace flux::gpu
{

namespace
{

#if defined(__linux__)

using detail::AllocHandle;
using detail::DevicePtr;
using detail::driver;
using detail::Driver;
using detail::ensure_context;
using detail::kSuccess;
using detail::MemAccessDesc;
using detail::MemAllocationProp;

[[noreturn]] void fail(const char * step)
{
  throw std::runtime_error(std::string("flux: ") + step);
}

[[noreturn]] void fail_errno(const char * step)
{
  throw std::runtime_error(std::string("flux: ") + step + ": " + std::strerror(errno));
}

std::size_t align_up(std::size_t n, std::size_t a) noexcept
{
  return a == 0 ? n : (n + a - 1) / a * a;
}

MemAllocationProp pinned_on(int device) noexcept
{
  MemAllocationProp prop{};
  prop.type = detail::kMemAllocationTypePinned;
  prop.requested_handle_types = detail::kMemHandleTypePosixFd;
  prop.location.type = detail::kMemLocationTypeDevice;
  prop.location.id = device;
  return prop;
}

// The driver's own handle for `device`. cuDeviceGet maps an ordinal to it, and every VMM call
// below takes the handle rather than the ordinal.
int device_handle(const Driver & drv, int device)
{
  int handle = 0;
  if (drv.device_get(&handle, device) != kSuccess) fail("cuDeviceGet failed");
  return handle;
}

// One VMM allocation, mapped and accessible. Owns every driver object it made, in the order they
// have to come apart: unmap the address range, free the reservation, release the allocation.
// Shared by both sides -- a publisher's created allocation and a subscriber's imported one differ
// only in where the handle came from.
struct Region
{
  Region() = default;
  ~Region()
  {
    const Driver & drv = driver();
    if (!drv.vmm_ok()) return;
    if (addr != 0 && size != 0) {
      drv.mem_unmap(addr, size);
      drv.mem_address_free(addr, size);
    }
    if (alloc != 0) drv.mem_release(alloc);
  }
  Region(const Region &) = delete;
  Region & operator=(const Region &) = delete;

  DevicePtr addr = 0;
  AllocHandle alloc = 0;
  std::size_t size = 0;
  int device = -1;
};

// Reserve a virtual range for `alloc`, back it, and let `device` read and write it. Runs
// identically for a created and an imported handle: mapping is not what distinguishes the sides.
void map_region(Region & r, const Driver & drv, int handle)
{
  if (drv.mem_address_reserve(&r.addr, r.size, 0, 0, 0) != kSuccess) {
    r.addr = 0;
    fail("cuMemAddressReserve failed");
  }
  if (drv.mem_map(r.addr, r.size, 0, r.alloc, 0) != kSuccess) fail("cuMemMap failed");

  MemAccessDesc desc{};
  desc.location.type = detail::kMemLocationTypeDevice;
  desc.location.id = handle;
  desc.flags = detail::kMemAccessReadWrite;
  if (drv.mem_set_access(r.addr, r.size, &desc, 1) != kSuccess) fail("cuMemSetAccess failed");
}

// Fill `addr` for an abstract-namespace name and return the length sendto/bind/connect want.
// Abstract addresses are not NUL-terminated strings: their length is what delimits them, so the
// caller must pass the returned value rather than sizeof(addr).
socklen_t abstract_addr(sockaddr_un & addr, const std::string & name)
{
  if (name.size() + 1 > sizeof(addr.sun_path)) {
    fail("device endpoint name does not fit an AF_UNIX address");
  }
  addr.sun_family = AF_UNIX;
  addr.sun_path[0] = '\0';  // leading NUL is what makes it abstract
  std::memcpy(addr.sun_path + 1, name.data(), name.size());
  return static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + name.size());
}

// Hand `fd` to the peer on `conn`. The descriptor rides in the control message; the one payload
// byte exists because a message with no data carries no control data either.
bool send_fd(int conn, int fd) noexcept
{
  char byte = 0;
  iovec iov{};
  iov.iov_base = &byte;
  iov.iov_len = 1;
  alignas(cmsghdr) char control[CMSG_SPACE(sizeof(int))] = {};
  msghdr msg{};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = control;
  msg.msg_controllen = sizeof(control);
  cmsghdr * cmsg = CMSG_FIRSTHDR(&msg);
  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SCM_RIGHTS;
  cmsg->cmsg_len = CMSG_LEN(sizeof(int));
  std::memcpy(CMSG_DATA(cmsg), &fd, sizeof(fd));
  return ::sendmsg(conn, &msg, MSG_NOSIGNAL) == 1;
}

// The mirror of send_fd. Returns -1 when the peer sent a message without a descriptor, which is
// what a truncated or spoofed exchange looks like from here.
int recv_fd(int conn) noexcept
{
  char byte = 0;
  iovec iov{};
  iov.iov_base = &byte;
  iov.iov_len = 1;
  alignas(cmsghdr) char control[CMSG_SPACE(sizeof(int))] = {};
  msghdr msg{};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = control;
  msg.msg_controllen = sizeof(control);
  if (::recvmsg(conn, &msg, 0) != 1) return -1;
  cmsghdr * cmsg = CMSG_FIRSTHDR(&msg);
  if (
    cmsg == nullptr || cmsg->cmsg_level != SOL_SOCKET || cmsg->cmsg_type != SCM_RIGHTS ||
    cmsg->cmsg_len != CMSG_LEN(sizeof(int))) {
    return -1;
  }
  int fd = -1;
  std::memcpy(&fd, CMSG_DATA(cmsg), sizeof(fd));
  return fd;
}

// The publisher's accept loop. One thread per channel, not per slot: the allocation is
// one region, so there is one descriptor to hand out however many slots it holds.
//
// It waits on the listening socket and a stop pipe together, so shutting down is a write to the
// pipe rather than a timeout to sit through. Closing the socket alone is not enough -- accept()
// on a closed descriptor is a race, not a wakeup.
class FdServer
{
public:
  FdServer() = default;
  ~FdServer() { stop(); }
  FdServer(const FdServer &) = delete;
  FdServer & operator=(const FdServer &) = delete;

  void start(const std::string & endpoint, int fd)
  {
    int sock = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (sock < 0) fail_errno("socket(AF_UNIX) for the device endpoint");
    sockaddr_un addr{};
    const socklen_t len = abstract_addr(addr, endpoint);
    if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), len) < 0) {
      const int e = errno;
      ::close(sock);
      errno = e;
      fail_errno("bind of the device endpoint");
    }
    if (::listen(sock, 16) < 0) {
      const int e = errno;
      ::close(sock);
      errno = e;
      fail_errno("listen on the device endpoint");
    }
    int pipefd[2];
    if (::pipe(pipefd) < 0) {
      const int e = errno;
      ::close(sock);
      errno = e;
      fail_errno("pipe for the device endpoint server");
    }
    listen_fd_ = sock;
    stop_r_ = pipefd[0];
    stop_w_ = pipefd[1];
    share_fd_ = fd;
    thread_ = std::thread([this] { run(); });
  }

  void stop() noexcept
  {
    if (thread_.joinable()) {
      const char byte = 0;
      [[maybe_unused]] ssize_t n = ::write(stop_w_, &byte, 1);
      thread_.join();
    }
    for (int * fd : {&stop_w_, &stop_r_, &listen_fd_}) {
      if (*fd >= 0) {
        ::close(*fd);
        *fd = -1;
      }
    }
  }

private:
  void run() noexcept
  {
    for (;;) {
      pollfd fds[2]{};
      fds[0].fd = listen_fd_;
      fds[0].events = POLLIN;
      fds[1].fd = stop_r_;
      fds[1].events = POLLIN;
      if (::poll(fds, 2, -1) < 0) {
        if (errno == EINTR) continue;
        return;
      }
      if (fds[1].revents != 0) return;  // asked to stop
      if ((fds[0].revents & POLLIN) == 0) continue;
      const int conn = ::accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC);
      if (conn < 0) continue;  // the subscriber gave up between poll and accept
      send_fd(conn, share_fd_);
      ::close(conn);
    }
  }

  int listen_fd_ = -1;
  int stop_r_ = -1;
  int stop_w_ = -1;
  int share_fd_ = -1;
  std::thread thread_;
};

#endif  // __linux__

}  // namespace

#if defined(__linux__)

struct DeviceAlloc::State
{
  ~State()
  {
    server.stop();  // stop handing the descriptor out before closing it
    if (fd >= 0) ::close(fd);
  }
  Region region;
  int fd = -1;
  FdServer server;
};

struct DeviceImport::State
{
  Region region;
};

std::string device_endpoint(const SegmentId & id)
{
  return "flux.gpu." + std::to_string(id.dev) + "." + std::to_string(id.ino);
}

std::size_t device_granularity(int device) noexcept
{
  const Driver & drv = driver();
  if (!drv.vmm_ok()) return 0;
  if (!ensure_context(drv, device)) return 0;
  int handle = 0;
  if (drv.device_get(&handle, device) != kSuccess) return 0;
  const MemAllocationProp prop = pinned_on(handle);
  std::size_t granularity = 0;
  if (drv.mem_granularity(&granularity, &prop, detail::kMemGranularityMinimum) != kSuccess) {
    return 0;
  }
  return granularity;
}

DeviceAlloc DeviceAlloc::create(const std::string & endpoint, std::size_t bytes, int device)
{
  const Driver & drv = driver();
  if (!drv.vmm_ok()) fail(drv.vmm_error != nullptr ? drv.vmm_error : drv.error);
  if (bytes == 0) fail("a device region of zero bytes was requested");
  if (!ensure_context(drv, device)) fail("no CUDA context and the primary context was refused");

  const int handle = device_handle(drv, device);
  const MemAllocationProp prop = pinned_on(handle);
  std::size_t granularity = 0;
  if (drv.mem_granularity(&granularity, &prop, detail::kMemGranularityMinimum) != kSuccess) {
    fail("cuMemGetAllocationGranularity failed");
  }

  auto state = std::make_shared<State>();
  state->region.device = device;
  state->region.size = align_up(bytes, granularity);
  if (drv.mem_create(&state->region.alloc, state->region.size, &prop, 0) != kSuccess) {
    state->region.alloc = 0;
    fail("cuMemCreate failed");
  }
  map_region(state->region, drv, handle);

  int shareable = -1;
  if (
    drv.mem_export(&shareable, state->region.alloc, detail::kMemHandleTypePosixFd, 0) != kSuccess) {
    fail("cuMemExportToShareableHandle failed");
  }
  state->fd = shareable;
  state->server.start(endpoint, shareable);  // throws before the fd is handed out, never after

  DeviceAlloc out;
  out.state_ = std::move(state);
  return out;
}

DeviceImport DeviceImport::open(const std::string & endpoint, std::size_t bytes, int device)
{
  const Driver & drv = driver();
  if (!drv.vmm_ok()) fail(drv.vmm_error != nullptr ? drv.vmm_error : drv.error);
  if (bytes == 0) fail("a device region of zero bytes was requested");
  if (!ensure_context(drv, device)) fail("no CUDA context and the primary context was refused");

  int sock = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (sock < 0) fail_errno("socket(AF_UNIX) for the device endpoint");
  sockaddr_un addr{};
  const socklen_t len = abstract_addr(addr, endpoint);
  if (::connect(sock, reinterpret_cast<sockaddr *>(&addr), len) < 0) {
    const int e = errno;
    ::close(sock);
    errno = e;
    // ECONNREFUSED here is the publisher not being up yet, which is the same transient the host
    // path reports as an absent segment. The caller retries rather than giving up.
    fail_errno("connect to the publisher's device endpoint");
  }
  const int shareable = recv_fd(sock);
  ::close(sock);
  if (shareable < 0) fail("the publisher's device endpoint sent no descriptor");

  const int handle = device_handle(drv, device);
  auto state = std::make_shared<State>();
  state->region.device = device;
  const int rc = drv.mem_import(
    &state->region.alloc, reinterpret_cast<void *>(static_cast<std::intptr_t>(shareable)),
    detail::kMemHandleTypePosixFd);
  // The import holds its own reference to the allocation, so this process's copy of the
  // descriptor has done its job either way.
  ::close(shareable);
  if (rc != kSuccess) {
    state->region.alloc = 0;
    fail("cuMemImportFromShareableHandle failed");
  }

  const MemAllocationProp prop = pinned_on(handle);
  std::size_t granularity = 0;
  if (drv.mem_granularity(&granularity, &prop, detail::kMemGranularityMinimum) != kSuccess) {
    fail("cuMemGetAllocationGranularity failed");
  }
  state->region.size = align_up(bytes, granularity);
  map_region(state->region, drv, handle);

  DeviceImport out;
  out.state_ = std::move(state);
  return out;
}

#else  // !__linux__

struct DeviceAlloc::State
{
};
struct DeviceImport::State
{
};

std::string device_endpoint(const SegmentId &)
{
  return {};
}

std::size_t device_granularity(int) noexcept
{
  return 0;
}

DeviceAlloc DeviceAlloc::create(const std::string &, std::size_t, int)
{
  throw std::runtime_error("flux: the GPU handle path is Linux-only");
}

DeviceImport DeviceImport::open(const std::string &, std::size_t, int)
{
  throw std::runtime_error("flux: the GPU handle path is Linux-only");
}

#endif  // __linux__

DeviceAlloc::~DeviceAlloc() = default;
DeviceAlloc::DeviceAlloc(DeviceAlloc &&) noexcept = default;
DeviceAlloc & DeviceAlloc::operator=(DeviceAlloc &&) noexcept = default;

DeviceImport::~DeviceImport() = default;
DeviceImport::DeviceImport(DeviceImport &&) noexcept = default;
DeviceImport & DeviceImport::operator=(DeviceImport &&) noexcept = default;

#if defined(__linux__)

void * DeviceAlloc::base() const noexcept
{
  return state_ ? reinterpret_cast<void *>(state_->region.addr) : nullptr;
}
std::size_t DeviceAlloc::bytes() const noexcept
{
  return state_ ? state_->region.size : 0;
}
int DeviceAlloc::device() const noexcept
{
  return state_ ? state_->region.device : -1;
}

void * DeviceImport::base() const noexcept
{
  return state_ ? reinterpret_cast<void *>(state_->region.addr) : nullptr;
}
std::size_t DeviceImport::bytes() const noexcept
{
  return state_ ? state_->region.size : 0;
}
int DeviceImport::device() const noexcept
{
  return state_ ? state_->region.device : -1;
}

#else

void * DeviceAlloc::base() const noexcept
{
  return nullptr;
}
std::size_t DeviceAlloc::bytes() const noexcept
{
  return 0;
}
int DeviceAlloc::device() const noexcept
{
  return -1;
}

void * DeviceImport::base() const noexcept
{
  return nullptr;
}
std::size_t DeviceImport::bytes() const noexcept
{
  return 0;
}
int DeviceImport::device() const noexcept
{
  return -1;
}

#endif  // __linux__

}  // namespace flux::gpu
