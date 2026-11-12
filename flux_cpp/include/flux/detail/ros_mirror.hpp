#ifndef FLUX_DETAIL_ROS_MIRROR_HPP
#define FLUX_DETAIL_ROS_MIRROR_HPP

#include "flux/detail/subscriber.hpp"
#include "flux/schemas.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace flux::detail
{

// The node-free half of a publisher's ROS 2 mirror, shared by the C++ Publisher and the Python
// binding: reads the publisher's own channel and writes each frame as the CDR of its ROS 2
// message. Attached only between detach() calls, so a mirror nobody watches holds no reader.
class RosMirror
{
public:
  // A GPU memory channel's fingerprint (gpu_fingerprint) mirrors as its type: aux and payload
  // become one message. Throws Error (FLUX_INVALID_ARGUMENT) for a fingerprint that names no
  // installed message type that way or the plain one, or a device-backed publisher of the plain
  // one.
  RosMirror(const Channel & publisher, std::string signpost, std::uint64_t fingerprint)
  : signpost_(std::move(signpost)),
    fingerprint_(fingerprint),
    gpu_(schemas_.contains_gpu(fingerprint)),
    type_(find(schemas_, fingerprint, gpu_, publisher.device_backed()))
  {
  }

  [[nodiscard]] auto ros_type() const -> std::string { return type_.ros_type(); }

  void detach() { sub_.reset(); }

  // Waits up to `timeout_ns`, or until stop(), for a frame and writes its CDR into out(n), which
  // returns n bytes to write to. The size written, or 0 for no frame or one that does not match
  // its schema.
  template <class Stop, class Out>
  auto next(std::int64_t timeout_ns, Stop stop, Out out) -> std::size_t
  {
    if (!sub_) {
      sub_.emplace(signpost_, fingerprint_, QoS(1).max_borrow(1), MemoryPolicy{});
    }
    FrameView frame = sub_->take_blocking(timeout_ns, stop);
    if (!frame) {
      return 0;
    }
    // Sizing and writing must read the same bytes, and a peer can still write a committed slot.
    // A frame with fields in its publisher's arena is copied with them, while it is held.
    try {
      if (gpu_) {
        type_.with_data(frame, bytes_);
      } else if (!in_arena(frame)) {
        const auto * data = static_cast<const std::uint8_t *>(frame.data());
        bytes_.assign(data, data + frame.size());
      } else {
        type_.inline_arena(frame, bytes_);
      }
      frame.release();
      const std::size_t n = type_.cdr_size(bytes_.data(), bytes_.size());
      return type_.write_cdr(bytes_.data(), bytes_.size(), out(n), n);
    } catch (const Error &) {
      return 0;
    }
  }

private:
  static auto find(const Schemas & s, std::uint64_t fingerprint, bool gpu, bool device_backed)
    -> Schemas::Type
  {
    if (gpu) {
      return s.at_gpu(fingerprint);
    }
    if (device_backed) {
      throw Error(FLUX_INVALID_ARGUMENT, "flux: a device-backed publisher has no ROS 2 mirror");
    }
    if (!s.contains(fingerprint)) {
      throw Error(FLUX_INVALID_ARGUMENT, "flux: no installed message type has this fingerprint");
    }
    return s.at(fingerprint);
  }

  std::string signpost_;
  std::uint64_t fingerprint_;
  Schemas schemas_;
  bool gpu_;
  Schemas::Type type_;
  std::optional<Subscriber> sub_;
  std::vector<std::uint8_t> bytes_;
};

}  // namespace flux::detail

#endif  // FLUX_DETAIL_ROS_MIRROR_HPP
