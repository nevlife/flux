#ifndef FLUX_GPU_VMM_HPP
#define FLUX_GPU_VMM_HPP

#include <cstddef>
#include <memory>
#include <string>

// The dGPU payload region and the fd handshake that shares it.
//
// One VMM allocation covers a channel's whole payload region, so slot `s` sits at
// `base() + s * payload_stride` exactly as it does on the host path. Nothing here knows about
// slots, frames or seqlocks -- this is mechanics, the layer Segment occupies for host memory.
//
// flux_core links no CUDA. Every driver call is resolved at runtime, so a build and a run on a
// host with no driver both succeed and report the absence.

namespace flux
{
// Declared, not included: segment.hpp holds the other end of this pair (a Segment owns the region
// a DeviceAlloc made), and including it here would close the cycle.
struct SegmentId;
}  // namespace flux

namespace flux::gpu
{

// Abstract-namespace socket the publisher serves this segment's exported fd on. Derived from the
// segment object's identity rather than its name: a name can outlive the object it named, and
// (dev, ino) cannot, so a rotation cannot leave a subscriber talking to the previous segment's
// server. Abstract rather than a filesystem path so a crash leaves nothing to unlink.
std::string device_endpoint(const SegmentId & id);

// Publisher side: the device allocation, its exported fd, and the thread serving that fd.
// Move-only; the destructor stops the server, closes the fd, and unmaps.
class DeviceAlloc
{
public:
  DeviceAlloc() = default;
  ~DeviceAlloc();
  DeviceAlloc(DeviceAlloc && other) noexcept;
  DeviceAlloc & operator=(DeviceAlloc && other) noexcept;
  DeviceAlloc(const DeviceAlloc &) = delete;
  DeviceAlloc & operator=(const DeviceAlloc &) = delete;

  // Allocate `bytes` on `device`, export it as a POSIX fd, and serve that fd on `endpoint`.
  // Throws std::runtime_error naming the step that failed -- a dGPU channel whose payload could
  // not be allocated has nothing to degrade to, so it must not be constructed half-made.
  static DeviceAlloc create(const std::string & endpoint, std::size_t bytes, int device);

  bool valid() const noexcept { return state_ != nullptr; }
  void * base() const noexcept;        // device address of the whole region
  std::size_t bytes() const noexcept;  // rounded up to the allocation granularity
  int device() const noexcept;

  // Type-erased owner of the mapping, for a holder that must keep the region alive without
  // depending on this header. Segment takes one of these so it need not know which side made it.
  std::shared_ptr<void> keepalive() const noexcept { return state_; }

private:
  struct State;
  std::shared_ptr<State> state_;
};

// Subscriber side: the fd this process received, imported and mapped. Move-only.
//
// A copy of the mapping, not of the memory: two processes that import the same fd see the same
// device bytes, which is what makes the borrow protocol mean the same thing on both sides.
class DeviceImport
{
public:
  DeviceImport() = default;
  ~DeviceImport();
  DeviceImport(DeviceImport && other) noexcept;
  DeviceImport & operator=(DeviceImport && other) noexcept;
  DeviceImport(const DeviceImport &) = delete;
  DeviceImport & operator=(const DeviceImport &) = delete;

  // Connect to `endpoint`, receive the fd, import and map `bytes` on `device`. Throws
  // std::runtime_error. A publisher that has not opened its socket yet fails here the same way an
  // absent segment does: transient, and the caller retries.
  static DeviceImport open(const std::string & endpoint, std::size_t bytes, int device);

  bool valid() const noexcept { return state_ != nullptr; }
  void * base() const noexcept;
  std::size_t bytes() const noexcept;
  int device() const noexcept;

  // Type-erased owner of the mapping, for a holder that must keep the region alive without
  // depending on this header. Segment takes one of these so it need not know which side made it.
  std::shared_ptr<void> keepalive() const noexcept { return state_; }

private:
  struct State;
  std::shared_ptr<State> state_;
};

// Allocation granularity for `device`, which every dGPU region size is rounded up to. 0 when the
// driver cannot answer. Exposed so a caller can size a region without allocating one.
std::size_t device_granularity(int device) noexcept;

}  // namespace flux::gpu

#endif  // FLUX_GPU_VMM_HPP
