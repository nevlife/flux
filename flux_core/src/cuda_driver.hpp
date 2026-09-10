#ifndef FLUX_SRC_CUDA_DRIVER_HPP
#define FLUX_SRC_CUDA_DRIVER_HPP

#include <cstddef>
#include <cstdint>

#if defined(__linux__)
#include <dlfcn.h>
#endif

// The driver API's ABI, spelled out so flux_core needs no CUDA headers and links no CUDA library.
// These are the values and struct shapes in cuda.h; both are ABI and neither moves between CUDA
// versions. Private to flux_core: gpu_platform.cpp judges the host with it, gpu_vmm.cpp allocates
// with it, and nothing outside those two sees it.

namespace flux::gpu::detail
{

#if defined(__linux__)

constexpr int kSuccess = 0;

constexpr int kAttrIntegrated = 18;
constexpr int kAttrUnifiedAddressing = 41;
constexpr int kAttrPageableMemoryAccess = 88;
constexpr int kAttrHostRegisterSupported = 99;
constexpr int kAttrVmmSupported = 102;
constexpr int kAttrPosixFdHandleSupported = 103;
constexpr int kAttrIpcEventSupported = 125;

constexpr int kMemAllocationTypePinned = 0x1;
constexpr int kMemHandleTypePosixFd = 0x1;
constexpr int kMemLocationTypeDevice = 0x1;
constexpr int kMemAccessReadWrite = 0x3;
constexpr int kMemGranularityMinimum = 0x0;

// cuMemHostRegister flags. DEVICEMAP asks for the device address the mapping answers to;
// READ_ONLY is mandatory for a PROT_READ mapping and refused-with-INVALID_VALUE without it,
// which is exactly what a subscriber's payload is after Segment::protect_payload().
constexpr unsigned kMemHostRegisterDeviceMap = 0x02u;
constexpr unsigned kMemHostRegisterReadOnly = 0x08u;

using DevicePtr = unsigned long long;
using AllocHandle = unsigned long long;

struct MemLocation
{
  int type;
  int id;
};

struct MemAllocationProp
{
  int type;
  int requested_handle_types;
  MemLocation location;
  void * win32_handle_metadata;
  struct
  {
    unsigned char compression_type;
    unsigned char gpu_direct_rdma_capable;
    unsigned short usage;
    unsigned char reserved[4];
  } alloc_flags;
};

struct MemAccessDesc
{
  MemLocation location;
  int flags;
};

// The driver structs are passed by address into libcuda, so their shape is a contract with a
// library this build never sees a header for. Pin it here the same way segment_layout.hpp pins
// the cross-process one.
static_assert(sizeof(MemLocation) == 8);
static_assert(sizeof(MemAllocationProp) == 32);
static_assert(alignof(MemAllocationProp) == 8);
static_assert(offsetof(MemAllocationProp, location) == 8);
static_assert(offsetof(MemAllocationProp, win32_handle_metadata) == 16);
static_assert(offsetof(MemAllocationProp, alloc_flags) == 24);
static_assert(sizeof(MemAccessDesc) == 12);

struct Driver
{
  const char * error = "not loaded";
  int (*device_count)(int *) = nullptr;
  int (*device_get)(int *, int) = nullptr;
  int (*device_attribute)(int *, int, int) = nullptr;
  int (*stream_synchronize)(void *) = nullptr;
  int (*stream_create)(void **, unsigned) = nullptr;
  int (*stream_destroy)(void *) = nullptr;
  int (*ctx_get_current)(void **) = nullptr;
  int (*ctx_set_current)(void *) = nullptr;
  int (*primary_ctx_retain)(void **, int) = nullptr;

  // Virtual memory management. Kept apart from the entry points above: a driver too old
  // to have these still serves probe() and the iGPU route, and failing the whole load over them
  // would strand a host that works.
  const char * vmm_error = "not loaded";
  int (*mem_granularity)(std::size_t *, const MemAllocationProp *, int) = nullptr;
  int (*mem_create)(AllocHandle *, std::size_t, const MemAllocationProp *, unsigned long long) =
    nullptr;
  int (*mem_address_reserve)(DevicePtr *, std::size_t, std::size_t, DevicePtr, unsigned long long) =
    nullptr;
  int (*mem_map)(DevicePtr, std::size_t, std::size_t, AllocHandle, unsigned long long) = nullptr;
  int (*mem_set_access)(DevicePtr, std::size_t, const MemAccessDesc *, std::size_t) = nullptr;
  int (*mem_export)(void *, AllocHandle, int, unsigned long long) = nullptr;
  int (*mem_import)(AllocHandle *, void *, int) = nullptr;
  int (*mem_unmap)(DevicePtr, std::size_t) = nullptr;
  int (*mem_address_free)(DevicePtr, std::size_t) = nullptr;
  int (*mem_release)(AllocHandle) = nullptr;

  // Host registration (Route::ShmRegistered). Kept apart for the same reason as the VMM group:
  // a driver without these still serves probe(), the ShmDirect route and the dGPU one, and
  // failing the whole load over them would strand a host that works.
  const char * host_reg_error = "not loaded";
  int (*mem_host_register)(void *, std::size_t, unsigned) = nullptr;
  int (*mem_host_unregister)(void *) = nullptr;
  int (*mem_host_get_device_pointer)(DevicePtr *, void *, unsigned) = nullptr;

  bool ok() const { return error == nullptr; }
  bool vmm_ok() const { return error == nullptr && vmm_error == nullptr; }
  bool host_reg_ok() const { return error == nullptr && host_reg_error == nullptr; }

  // A query the driver refuses reads as "not supported". Failing the whole probe over one
  // attribute a future driver may drop would strand a host that works.
  bool attribute(int handle, int attr) const
  {
    int value = 0;
    return device_attribute(&value, attr, handle) == kSuccess && value != 0;
  }
};

// cuInit runs once per process and costs hundreds of milliseconds, and the handle is never
// dlclose()d: unloading a driver this process has already initialized would pull it out from
// under any CUDA the application loads later.
inline const Driver & driver()
{
  static const Driver d = [] {
    Driver drv;
    void * lib = ::dlopen("libcuda.so.1", RTLD_LAZY | RTLD_LOCAL);
    if (lib == nullptr) {
      drv.error = "libcuda.so.1 not loadable";
      drv.vmm_error = drv.error;
      return drv;
    }
    const auto sym = [lib](const char * name) { return ::dlsym(lib, name); };
    auto init = reinterpret_cast<int (*)(unsigned)>(sym("cuInit"));
    drv.device_count = reinterpret_cast<decltype(drv.device_count)>(sym("cuDeviceGetCount"));
    drv.device_get = reinterpret_cast<decltype(drv.device_get)>(sym("cuDeviceGet"));
    drv.device_attribute =
      reinterpret_cast<decltype(drv.device_attribute)>(sym("cuDeviceGetAttribute"));
    drv.stream_synchronize =
      reinterpret_cast<decltype(drv.stream_synchronize)>(sym("cuStreamSynchronize"));
    drv.stream_create = reinterpret_cast<decltype(drv.stream_create)>(sym("cuStreamCreate"));
    drv.stream_destroy = reinterpret_cast<decltype(drv.stream_destroy)>(sym("cuStreamDestroy"));
    drv.ctx_get_current = reinterpret_cast<decltype(drv.ctx_get_current)>(sym("cuCtxGetCurrent"));
    drv.ctx_set_current = reinterpret_cast<decltype(drv.ctx_set_current)>(sym("cuCtxSetCurrent"));
    drv.primary_ctx_retain =
      reinterpret_cast<decltype(drv.primary_ctx_retain)>(sym("cuDevicePrimaryCtxRetain"));
    if (
      !init || !drv.device_count || !drv.device_get || !drv.device_attribute ||
      !drv.stream_synchronize || !drv.stream_create || !drv.stream_destroy ||
      !drv.ctx_get_current || !drv.ctx_set_current || !drv.primary_ctx_retain) {
      drv.error = "libcuda.so.1 is missing an entry point flux needs";
      drv.vmm_error = drv.error;
      return drv;
    }
    if (init(0) != kSuccess) {
      drv.error = "cuInit failed";
      drv.vmm_error = drv.error;
      return drv;
    }
    drv.error = nullptr;

    // libcuda exports both the _v2 ABI and the original names. The CUDA headers define the
    // unsuffixed spelling to the _v2 entry point, so that is the one to bind -- v1
    // cuMemHostGetDevicePointer returns a 32-bit pointer, and binding it by the short name on a
    // driver that has both would truncate every device address. The fallback is only for a
    // driver too old to carry _v2.
    const auto sym_v2 = [&sym](const char * v2, const char * v1) {
      void * p = sym(v2);
      return p != nullptr ? p : sym(v1);
    };
    drv.mem_host_register = reinterpret_cast<decltype(drv.mem_host_register)>(
      sym_v2("cuMemHostRegister_v2", "cuMemHostRegister"));
    drv.mem_host_unregister =
      reinterpret_cast<decltype(drv.mem_host_unregister)>(sym("cuMemHostUnregister"));
    drv.mem_host_get_device_pointer = reinterpret_cast<decltype(drv.mem_host_get_device_pointer)>(
      sym_v2("cuMemHostGetDevicePointer_v2", "cuMemHostGetDevicePointer"));
    drv.host_reg_error =
      (drv.mem_host_register && drv.mem_host_unregister && drv.mem_host_get_device_pointer)
        ? nullptr
        : "libcuda.so.1 has no host registration entry points";

    drv.mem_granularity =
      reinterpret_cast<decltype(drv.mem_granularity)>(sym("cuMemGetAllocationGranularity"));
    drv.mem_create = reinterpret_cast<decltype(drv.mem_create)>(sym("cuMemCreate"));
    drv.mem_address_reserve =
      reinterpret_cast<decltype(drv.mem_address_reserve)>(sym("cuMemAddressReserve"));
    drv.mem_map = reinterpret_cast<decltype(drv.mem_map)>(sym("cuMemMap"));
    drv.mem_set_access = reinterpret_cast<decltype(drv.mem_set_access)>(sym("cuMemSetAccess"));
    drv.mem_export =
      reinterpret_cast<decltype(drv.mem_export)>(sym("cuMemExportToShareableHandle"));
    drv.mem_import =
      reinterpret_cast<decltype(drv.mem_import)>(sym("cuMemImportFromShareableHandle"));
    drv.mem_unmap = reinterpret_cast<decltype(drv.mem_unmap)>(sym("cuMemUnmap"));
    drv.mem_address_free =
      reinterpret_cast<decltype(drv.mem_address_free)>(sym("cuMemAddressFree"));
    drv.mem_release = reinterpret_cast<decltype(drv.mem_release)>(sym("cuMemRelease"));
    if (
      !drv.mem_granularity || !drv.mem_create || !drv.mem_address_reserve || !drv.mem_map ||
      !drv.mem_set_access || !drv.mem_export || !drv.mem_import || !drv.mem_unmap ||
      !drv.mem_address_free || !drv.mem_release) {
      drv.vmm_error = "libcuda.so.1 has no virtual memory management (needs CUDA 10.2+)";
      return drv;
    }
    drv.vmm_error = nullptr;
    return drv;
  }();
  return d;
}

// A stream and every VMM call need a current context, and a caller that has not touched CUDA yet
// has none. For a node that builds its publishers before it runs any kernel that is the normal
// order, so the device's primary context is retained rather than the request refused. This is
// what the CUDA runtime does on its own first use, which is why a caller that later uses the
// runtime API lands on this same context. Never released: it lives as long as the process,
// exactly as the runtime's does.
inline bool ensure_context(const Driver & drv, int device = 0)
{
  void * ctx = nullptr;
  if (drv.ctx_get_current(&ctx) == kSuccess && ctx != nullptr) return true;
  int dev = 0;
  if (drv.device_get(&dev, device) != kSuccess) return false;
  void * primary = nullptr;
  if (drv.primary_ctx_retain(&primary, dev) != kSuccess) return false;
  return drv.ctx_set_current(primary) == kSuccess;
}

#endif  // __linux__

}  // namespace flux::gpu::detail

#endif  // FLUX_SRC_CUDA_DRIVER_HPP
