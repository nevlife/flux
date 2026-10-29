#ifndef FLUX_PUBLISHER_HPP
#define FLUX_PUBLISHER_HPP

#include "flux/channel.hpp"
#include "flux/detail/manifest.hpp"
#include "flux/node_ref.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>

namespace flux
{

// One per topic on the host: Error with FLUX_PUBLISHER_EXISTS while another is live. One call at
// a time: a publish or loan that meets another thread's call gives Published::InUse or an empty
// WriteSlot.
class Publisher
{
public:
  static constexpr std::uint32_t kDefaultSlotSize = 8U << 20U;
  static constexpr std::uint32_t kDefaultSlotCount = 16;
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

  // Backpressure while every slot is borrowed: the frame is dropped.
  Published publish(const void * data, std::size_t nbytes) noexcept
  {
    return ch_.publish(data, nbytes);
  }
  Published publish(
    const void * data, DType dtype, const std::uint64_t * shape, std::size_t ndim) noexcept
  {
    return ch_.publish(data, dtype, shape, ndim);
  }
  Published publish(
    const void * data, DType dtype, std::initializer_list<std::uint64_t> shape) noexcept
  {
    return ch_.publish(data, dtype, shape);
  }
  // The slot is out of the ring until the loan is committed or destroyed.
  WriteSlot loan() noexcept { return ch_.loan(); }
  WriteSlot loan(DType dtype, const std::uint64_t * shape, std::size_t ndim) noexcept
  {
    return ch_.loan(dtype, shape, ndim);
  }
  WriteSlot loan(DType dtype, std::initializer_list<std::uint64_t> shape) noexcept
  {
    return ch_.loan(dtype, shape);
  }

  std::uint64_t dropped() const noexcept { return ch_.dropped(); }
  // Commits that could not wait for the stream, each holding its slot; see set_fence_timeout.
  std::uint64_t fence_failed() const noexcept { return ch_.fence_failed(); }
  gpu::FenceWait fence_wait() const noexcept { return ch_.fence_wait(); }
  // Zero, the default, is no bound. A commit that runs out is FenceFailed until its work is done.
  void set_fence_timeout(std::chrono::nanoseconds t) const noexcept { ch_.set_fence_timeout(t); }
  // The payload is device memory: publish() copies host bytes to it and a loan's data() is null.
  bool device_backed() const noexcept { return ch_.device_backed(); }
  std::uint32_t slot_size() const noexcept { return ch_.slot_size(); }
  const std::string & topic_name() const noexcept { return topic_; }
  const std::string & signpost_name() const noexcept { return signpost_; }

private:
  std::string topic_;
  std::string signpost_;
  detail::Channel ch_;
  detail::Announcement announced_;
};

}  // namespace flux

#endif  // FLUX_PUBLISHER_HPP
