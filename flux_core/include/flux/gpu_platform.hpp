#ifndef FLUX_GPU_PLATFORM_HPP
#define FLUX_GPU_PLATFORM_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

// Which GPU payload route this host can take. flux_core does not link
// CUDA: the driver is opened at runtime, so a build on a host with no CUDA still runs and
// reports Route::None. Nothing here allocates, maps, or publishes -- probe() answers one
// question and every later GPU decision reads that answer instead of asking the driver again.

namespace flux
{

// What a channel's payloads are produced and consumed on. Cpu is the default and means the
// channel never mentions CUDA. Cuda is spelled for the API it uses, not for "GPU" in general --
// StorageKind already reserves a vendor-neutral path, and one name covering both would blur the
// day that lands.
enum class Device : std::uint8_t {
  Cpu = 0,
  Cuda,
};

}  // namespace flux

namespace flux::gpu
{

enum class Route : std::uint8_t {
  None = 0,       // no usable route; Platform::reason says why
  ShmDirect,      // integrated + pageable: a slot mapping is already device-addressable
  ShmRegistered,  // integrated: a slot mapping becomes device-addressable once registered
  DeviceHandle,   // separate GPU allocation exported as a POSIX fd
};

// The attributes behind the verdict travel with it. A route is a judgement; the attributes are
// facts, and a caller that disagrees with the judgement needs the facts to say so.
struct Platform
{
  Route route = Route::None;
  int device = -1;
  bool integrated = false;
  bool unified_addressing = false;
  bool pageable_access = false;
  bool host_register = false;
  bool vmm = false;
  bool posix_fd_handle = false;
  bool ipc_event = false;
  std::string reason;  // non-empty exactly when route == None

  // True when a host shm mapping is the GPU buffer, so the engine keeps StorageKind::HostInline
  // and the slot stays a byte range.
  bool host_memory_is_device_memory() const
  {
    return route == Route::ShmDirect || route == Route::ShmRegistered;
  }
};

// The GPU work ordering flux has to wait on at the two seams where a CPU statement outruns the
// GPU: commit says a frame is complete, releasing a view says a slot is
// no longer read. Both must wait for the stream first.
//
// Opaque on purpose. flux_core holds the handle and one operation on it, so declaring a stream
// costs no CUDA dependency; the entry point comes from the driver probe() already opened.
class Stream
{
public:
  // Declares nothing: a channel left with this fences nothing and behaves exactly as a channel
  // that never mentions GPU.
  Stream() = default;

  // Declares `native` (CUstream / cudaStream_t). A null handle is CUDA's legacy default stream,
  // which is a real stream -- so construction is what declares, not the value. Nothing else can
  // tell "the default stream" apart from "no stream".
  //
  // Declaring it is not the same as a channel accepting it: the legacy default stream resolves
  // against the calling thread's current context, so require_fenceable() refuses it.
  explicit Stream(void * native) noexcept : native_(native), declared_(true) {}

  // A stream flux creates and owns. Ownership travels with the value, so a view that outlives
  // the channel it came from still has a live stream to wait on. Throws std::runtime_error when
  // the driver refuses, most often because the calling thread has no current CUDA context.
  //
  // Creating one rather than reusing CUDA's default stream is deliberate: which stream handle 0
  // names depends on how the caller's translation units were compiled, so a fence on it could
  // wait on a different stream than the work went to -- the silent failure this design exists to
  // prevent. A caller that has its own stream passes it instead.
  static Stream create();

  bool declared() const noexcept { return declared_; }
  void * native() const noexcept { return native_; }

  // Block until work submitted to this stream before now has finished. An undeclared stream has
  // nothing to wait for and succeeds without touching CUDA.
  //
  // False means the wait did not happen -- no driver, no context, a stream the driver rejects.
  // The caller must read that as "still running". A fence that could not be taken must never
  // reach a caller as a fence that passed, because the only thing done next is to let the
  // publisher overwrite the bytes.
  bool wait() const noexcept;

private:
  void * native_ = nullptr;
  bool declared_ = false;
  std::shared_ptr<void> owner_;  // non-null only for create(); destroys the stream
};

// A host mapping a kernel can reach because the driver was told about it (Route::ShmRegistered).
// Empty on every other route: ShmDirect needs no registration because
// the host address already is the device address, and DeviceHandle has no host payload to
// register. Copies share one registration, which is released with the last of them -- a view
// handed out before the channel went away must still be able to name its device address.
class HostRegistration
{
public:
  HostRegistration() = default;

  // Register [base, base + bytes) and resolve the device address it answers to. `read_only` must
  // be set for a PROT_READ mapping -- a subscriber's payload is one after
  // Segment::protect_payload(), and the driver refuses it with INVALID_VALUE unless told.
  //
  // Throws std::invalid_argument when the driver refuses. A caller that declared a stream must
  // not carry on with a device pointer it never got: that is the silent GPU-less run this
  // library refuses everywhere else.
  static HostRegistration create(void * base, std::size_t bytes, bool read_only);

  // The device address `base` answers to. Not equal to `base` in general, which is the whole
  // reason this type exists rather than the host pointer being handed to kernels directly.
  void * device_base() const noexcept { return device_base_; }
  explicit operator bool() const noexcept { return device_base_ != nullptr; }

private:
  std::shared_ptr<void> owner_;  // unregisters on the last release
  void * device_base_ = nullptr;
};

// Ask the driver about `device`, once. Loads libcuda.so.1 and calls cuInit, so a caller that
// must not initialize CUDA on this thread must not call this, and a process that forks after
// calling it must not expect CUDA to work in the child.
//
// Never throws. An absent driver, an absent device and a device with no usable route all come
// back as Route::None with a reason: a host without a GPU is not an error for flux, and a
// probe that did not run must not be mistaken for one that found nothing.
Platform probe(int device = 0);

const char * to_string(Route route);

// Throws std::invalid_argument unless this host can serve a GPU declaration. Every route flux
// has not implemented is refused by name rather than quietly downgraded to the host path: code
// that believes it runs on the GPU must not run off it without saying so.
void require_route();

// Throws std::invalid_argument unless `stream` is a declaration flux can fence on any thread.
// require_route() judges the host; this judges the declaration, and the two fail for different
// reasons.
//
// Only CUDA's legacy default stream (a null handle) is refused. It names the current context's
// default stream, and a CUDA context is current per thread -- so a fence taken on a thread that
// never touched CUDA fails, while the same declaration fences fine on the thread that did. Which
// threads release a view is not something a channel knows, so the declaration is refused at the
// point it is made rather than left to fail once per frame on whichever thread got the view.
void require_fenceable(const Stream & stream);

// Resolve a Device to the stream a channel will fence on. Cpu yields an undeclared stream and
// never touches CUDA. Cuda checks the route first, so an unusable host is reported as such
// rather than as a missing context.
Stream stream_for(Device device);

}  // namespace flux::gpu

#endif  // FLUX_GPU_PLATFORM_HPP
