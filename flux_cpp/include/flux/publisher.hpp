#ifndef FLUX_PUBLISHER_HPP
#define FLUX_PUBLISHER_HPP

#include "flux/channel.hpp"
#include "flux/detail/manifest.hpp"
#include "flux/node_ref.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <string>

namespace flux
{

// One per topic on the host: Error with FLUX_PUBLISHER_EXISTS while another is live. One call at
// a time: a publish or loan that meets another thread's call gives Published::InUse or an empty
// WriteSlot.
class Publisher
{
public:
  static constexpr std::uint32_t kDefaultSlotSize = FLUX_DEFAULT_SLOT_SIZE;
  static constexpr std::uint32_t kDefaultSlotCount = FLUX_DEFAULT_SLOT_COUNT;
  static constexpr std::uint32_t kDefaultMaxSubscribers = FLUX_DEFAULT_MAX_SUBSCRIBERS;

  // `max_subscribers` counts subscribing processes (up to FLUX_MAX_SUBSCRIBERS), not
  // subscriptions. Each one adds a counter that every publish reads.
  Publisher(
    const NodeRef & node, const std::string & topic, std::uint64_t fingerprint = kNoSchema,
    std::uint32_t slot_size = kDefaultSlotSize, std::uint32_t slot_count = kDefaultSlotCount,
    MemoryPolicy mem = {}, std::uint32_t max_subscribers = kDefaultMaxSubscribers);
  // Frames produced on the GPU. Device::Cuda declares a stream flux creates; Error where the
  // host has no GPU route.
  Publisher(
    const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
    std::uint32_t slot_size, std::uint32_t slot_count, Device device, MemoryPolicy mem = {},
    std::uint32_t max_subscribers = kDefaultMaxSubscribers);
  // The caller's stream; the commit waits for everything queued on it.
  Publisher(
    const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
    std::uint32_t slot_size, std::uint32_t slot_count, const gpu::Stream & stream,
    MemoryPolicy mem = {}, std::uint32_t max_subscribers = kDefaultMaxSubscribers);
  Publisher(const Publisher &) = delete;
  auto operator=(const Publisher &) -> Publisher & = delete;
  Publisher(Publisher &&) noexcept;
  auto operator=(Publisher &&) noexcept -> Publisher &;
  ~Publisher();

  // Also publishes each frame as its ROS 2 message on this topic while a ROS 2 subscriber wants it.
  void mirror_to_ros();

  // Backpressure while every slot is borrowed: the frame is dropped.
  auto publish(const void * data, std::size_t nbytes) noexcept -> Published
  {
    return ch_.publish(data, nbytes);
  }
  auto publish(
    const void * data, DType dtype, const std::uint64_t * shape,
    std::size_t ndim) noexcept -> Published
  {
    return ch_.publish(data, dtype, shape, ndim);
  }
  auto publish(const void * data, DType dtype, std::initializer_list<std::uint64_t> shape) noexcept
    -> Published
  {
    return ch_.publish(data, dtype, shape);
  }
  // The slot is out of the ring until the loan is committed or destroyed.
  auto loan() noexcept -> WriteSlot { return ch_.loan(); }
  auto loan(DType dtype, const std::uint64_t * shape, std::size_t ndim) noexcept -> WriteSlot
  {
    return ch_.loan(dtype, shape, ndim);
  }
  auto loan(DType dtype, std::initializer_list<std::uint64_t> shape) noexcept -> WriteSlot
  {
    return ch_.loan(dtype, shape);
  }

  [[nodiscard]] auto dropped() const noexcept -> std::uint64_t { return ch_.dropped(); }
  // Commits that could not wait for the stream, each holding its slot; see set_fence_timeout.
  [[nodiscard]] auto fence_failed() const noexcept -> std::uint64_t { return ch_.fence_failed(); }
  [[nodiscard]] auto fence_wait() const noexcept -> gpu::FenceWait { return ch_.fence_wait(); }
  // Zero, the default, is no bound. A commit that runs out is FenceFailed until its work is done.
  void set_fence_timeout(std::chrono::nanoseconds t) const noexcept { ch_.set_fence_timeout(t); }
  // The payload is device memory: publish() copies host bytes to it and a loan's data() is null.
  [[nodiscard]] auto device_backed() const noexcept -> bool { return ch_.device_backed(); }
  [[nodiscard]] auto slot_size() const noexcept -> std::uint32_t { return ch_.slot_size(); }
  [[nodiscard]] auto topic_name() const noexcept -> const std::string & { return topic_; }
  [[nodiscard]] auto signpost_name() const noexcept -> const std::string & { return signpost_; }

private:
  class RosMirror;

  NodeRef node_;
  std::uint64_t fingerprint_;
  std::string topic_;
  std::string signpost_;
  detail::Channel ch_;
  detail::Announcement announced_;
  // Last, so its thread stops before the channel goes.
  std::unique_ptr<RosMirror> mirror_;
};

}  // namespace flux

#endif  // FLUX_PUBLISHER_HPP
