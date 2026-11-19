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

// A declared `stream` takes precedence over `device`. A size or count of 0 is flux's default; the
// maxima bound an Alloc::Slot or device-payload publisher.
struct PublisherOptions
{
  Alloc alloc = Alloc::Arena;
  std::uint32_t slot_size = FLUX_DEFAULT_SLOT_SIZE;
  std::uint32_t slot_count = FLUX_DEFAULT_SLOT_COUNT;
  std::uint32_t max_slot_size = 0;
  std::uint32_t max_slot_count = 0;
  Device device = Device::Cpu;
  gpu::Stream stream;
  MemoryPolicy memory = {};
  // Subscribing processes (up to FLUX_MAX_SUBSCRIBERS), not subscriptions. Each one adds a
  // counter that every publish reads.
  std::uint32_t max_subscribers = FLUX_DEFAULT_MAX_SUBSCRIBERS;
  // Host bytes beside each slot's payload (up to FLUX_MAX_AUX_SIZE), which a loan's aux() writes.
  std::uint32_t aux_size = 0;
  // Alloc::Slot and device-payload publishers: slots get memory once a subscriber attaches, and
  // publish fails until then. For topics nobody may subscribe to.
  bool alloc_on_subscriber = false;
  ImageSlots image_slots = ImageSlots::Auto;
  ImageEncoding image_encoding = ImageEncoding::None;
  std::uint32_t image_width = 0;
  std::uint32_t image_height = 0;
};

// One per topic on the host: Error with Status::PublisherExists while another is live. A publish
// or loan that meets another thread's call gives Published::InUse or an empty Loan.
class Publisher
{
public:
  Publisher(
    const NodeRef & node, const std::string & topic, const PublisherOptions & options,
    std::uint64_t fingerprint = kNoSchema);
  Publisher(const Publisher &) = delete;
  auto operator=(const Publisher &) -> Publisher & = delete;
  // Not movable: trace events and the mirror name it by its address.
  Publisher(Publisher &&) = delete;
  auto operator=(Publisher &&) -> Publisher & = delete;
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
  auto loan() noexcept -> Loan { return ch_.loan(); }
  auto loan(const std::uint64_t * shape, std::size_t ndim, DType dtype) noexcept -> Loan
  {
    return ch_.loan(shape, ndim, dtype);
  }
  auto loan(std::initializer_list<std::uint64_t> shape, DType dtype) noexcept -> Loan
  {
    return ch_.loan(shape, dtype);
  }

  [[nodiscard]] auto dropped() const noexcept -> std::uint64_t { return ch_.dropped(); }
  // Live subscribing processes. One system call per subscriber number ever taken.
  [[nodiscard]] auto subscriber_count() const noexcept -> std::uint32_t
  {
    return ch_.subscriber_count();
  }
  // Commits that could not wait for the stream, each holding its slot; see set_fence_timeout.
  [[nodiscard]] auto fence_failed() const noexcept -> std::uint64_t { return ch_.fence_failed(); }
  [[nodiscard]] auto fence_wait() const noexcept -> gpu::FenceWait { return ch_.fence_wait(); }
  // Zero, the default, is no bound. A commit that runs out is FenceFailed until its work is done.
  void set_fence_timeout(std::chrono::nanoseconds t) const noexcept { ch_.set_fence_timeout(t); }
  // The payload is device memory: publish() copies host bytes to it and a loan's data() is null.
  [[nodiscard]] auto device_backed() const noexcept -> bool { return ch_.device_backed(); }
  [[nodiscard]] auto slot_size() const noexcept -> std::uint32_t { return ch_.slot_size(); }
  [[nodiscard]] auto aux_size() const noexcept -> std::uint32_t { return ch_.aux_size(); }
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

// As rclcpp::create_publisher, with flux's own arguments after rclcpp's.
inline auto create_publisher(
  const NodeRef & node, const std::string & topic,
  const PublisherOptions & options = PublisherOptions(),
  std::uint64_t fingerprint = kNoSchema) -> std::shared_ptr<Publisher>
{
  return std::make_shared<Publisher>(node, topic, options, fingerprint);
}

}  // namespace flux

#endif  // FLUX_PUBLISHER_HPP
