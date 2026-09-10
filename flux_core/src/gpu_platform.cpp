#include "flux/gpu_platform.hpp"

#include "cuda_driver.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>

namespace flux::gpu
{

namespace
{

Platform none(const char * why)
{
  Platform p;
  p.reason = why;
  return p;
}

#if defined(__linux__)

using detail::DevicePtr;
using detail::driver;
using detail::Driver;
using detail::ensure_context;
using detail::kAttrHostRegisterSupported;
using detail::kAttrIntegrated;
using detail::kAttrIpcEventSupported;
using detail::kAttrPageableMemoryAccess;
using detail::kAttrPosixFdHandleSupported;
using detail::kAttrUnifiedAddressing;
using detail::kAttrVmmSupported;
using detail::kMemHostRegisterDeviceMap;
using detail::kMemHostRegisterReadOnly;
using detail::kSuccess;

// `integrated` is asked first on purpose: host registration also
// succeeds on a discrete GPU, but the device pointer it returns crosses PCIe, so the capability
// says yes where the answer should be no. DeviceHandle additionally requires the export path to
// exist -- a route whose handle cannot be made is not a route.
Route select(const Platform & p)
{
  if (p.integrated && p.pageable_access) return Route::ShmDirect;
  if (p.integrated && p.host_register) return Route::ShmRegistered;
  if (p.vmm && p.posix_fd_handle) return Route::DeviceHandle;
  return Route::None;
}

#endif  // __linux__

}  // namespace

Platform probe(int device)
{
#if defined(__linux__)
  const Driver & drv = driver();
  if (!drv.ok()) return none(drv.error);

  int count = 0;
  if (drv.device_count(&count) != kSuccess) return none("cuDeviceGetCount failed");
  if (count <= 0) return none("no CUDA device");
  if (device < 0 || device >= count) return none("device index out of range");

  int handle = 0;
  if (drv.device_get(&handle, device) != kSuccess) return none("cuDeviceGet failed");

  Platform p;
  p.device = device;
  p.integrated = drv.attribute(handle, kAttrIntegrated);
  p.unified_addressing = drv.attribute(handle, kAttrUnifiedAddressing);
  p.pageable_access = drv.attribute(handle, kAttrPageableMemoryAccess);
  p.host_register = drv.attribute(handle, kAttrHostRegisterSupported);
  p.vmm = drv.attribute(handle, kAttrVmmSupported);
  p.posix_fd_handle = drv.attribute(handle, kAttrPosixFdHandleSupported);
  p.ipc_event = drv.attribute(handle, kAttrIpcEventSupported);

  p.route = select(p);
  if (p.route == Route::None) {
    p.reason = "device shares no memory route: not integrated and no fd export";
  }
  return p;
#else
  (void)device;
  return none("not Linux");
#endif
}

HostRegistration HostRegistration::create(void * base, std::size_t bytes, bool read_only)
{
#if defined(__linux__)
  const Driver & drv = driver();
  if (!drv.ok()) throw std::invalid_argument(std::string("flux: ") + drv.error);
  if (!drv.host_reg_ok()) throw std::invalid_argument(std::string("flux: ") + drv.host_reg_error);
  if (base == nullptr || bytes == 0) throw std::invalid_argument("flux: nothing to register");
  if (!ensure_context(drv)) {
    throw std::invalid_argument(
      "flux: no CUDA context and the device's primary context was refused");
  }
  unsigned flags = kMemHostRegisterDeviceMap;
  if (read_only) flags |= kMemHostRegisterReadOnly;
  if (drv.mem_host_register(base, bytes, flags) != kSuccess) {
    throw std::invalid_argument("flux: cuMemHostRegister refused this payload mapping");
  }
  DevicePtr device = 0;
  if (drv.mem_host_get_device_pointer(&device, base, 0) != kSuccess) {
    drv.mem_host_unregister(base);
    throw std::invalid_argument("flux: the registered mapping resolved to no device address");
  }
  HostRegistration reg;
  // The deleter carries the entry point rather than reaching for driver() again: this runs at
  // teardown, where a static's lifetime is not something to depend on.
  auto * unregister = drv.mem_host_unregister;
  reg.owner_ = std::shared_ptr<void>(base, [unregister](void * p) { unregister(p); });
  reg.device_base_ = reinterpret_cast<void *>(static_cast<std::uintptr_t>(device));
  return reg;
#else
  (void)base;
  (void)bytes;
  (void)read_only;
  throw std::runtime_error("flux: host registration is Linux-only");
#endif
}

Stream Stream::create()
{
#if defined(__linux__)
  const Driver & drv = driver();
  if (!drv.ok()) throw std::runtime_error(std::string("flux: ") + drv.error);
  if (!ensure_context(drv)) {
    throw std::runtime_error("flux: no CUDA context and the device's primary context was refused");
  }
  void * handle = nullptr;
  // CU_STREAM_DEFAULT: a caller that also submits to CUDA's default stream keeps the implicit
  // ordering it would have had, which is the forgiving direction for a stream flux hands out.
  if (drv.stream_create(&handle, 0u) != kSuccess) {
    throw std::runtime_error("flux: cuStreamCreate failed");
  }
  Stream s(handle);
  auto * destroy = drv.stream_destroy;
  s.owner_ = std::shared_ptr<void>(handle, [destroy](void * h) { destroy(h); });
  return s;
#else
  throw std::runtime_error("flux: GPU streams are Linux-only");
#endif
}

void require_route()
{
  const Platform p = probe();
  switch (p.route) {
    case Route::ShmDirect:
      return;  // the slot mapping is the GPU buffer; data() and device_ptr() agree
    case Route::DeviceHandle:
      return;  // the payload is a device allocation shared by fd
    case Route::ShmRegistered:
      return;  // the slot mapping becomes the GPU buffer once registered (gpu::HostRegistration)
    case Route::None:
      break;
  }
  throw std::invalid_argument("flux: no GPU route on this host: " + p.reason);
}

void require_fenceable(const Stream & stream)
{
  if (stream.declared() && stream.native() == nullptr) {
    throw std::invalid_argument(
      "flux: CUDA's legacy default stream cannot be declared on a channel: it resolves against "
      "the calling thread's current context, so a fence taken on a thread that has none fails "
      "and costs a slot. Pass gpu::Stream::create() or your own stream handle "
      "");
  }
}

Stream stream_for(Device device)
{
  if (device == Device::Cpu) return {};
  require_route();
  return Stream::create();
}

bool Stream::wait() const noexcept
{
  if (!declared_) return true;
#if defined(__linux__)
  const Driver & drv = driver();
  if (!drv.ok()) return false;
  return drv.stream_synchronize(native_) == kSuccess;
#else
  return false;
#endif
}

const char * to_string(Route route)
{
  switch (route) {
    case Route::None:
      return "None";
    case Route::ShmDirect:
      return "ShmDirect";
    case Route::ShmRegistered:
      return "ShmRegistered";
    case Route::DeviceHandle:
      return "DeviceHandle";
  }
  return "None";
}

}  // namespace flux::gpu
