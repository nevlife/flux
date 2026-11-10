#include "flux/publisher.hpp"

#include "flux/detail/ros_mirror.hpp"
#include "ros_boundary.hpp"

#include <rclcpp/create_generic_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/serialized_message.hpp>

#include <atomic>
#include <chrono>
#include <exception>
#include <thread>

namespace flux
{

// A thread rather than an executor source: the mirror must not take a callback group's time,
// and take_blocking parks on the futex, so a mirror with a ROS 2 subscriber but no frames costs
// no CPU.
class Publisher::RosMirror
{
public:
  RosMirror(
    const NodeRef & node, const detail::Channel & ch, const std::string & topic,
    const std::string & signpost, std::uint64_t fingerprint)
  : topic_(topic),
    core_(ch, signpost, fingerprint),
    pub_(rclcpp::create_generic_publisher(
      node.topics(), topic, core_.ros_type(), rclcpp::QoS(1).best_effort().durability_volatile())),
    logger_(node.logging()->get_logger()),
    thread_([this] { run(); })
  {
  }
  RosMirror(const RosMirror &) = delete;
  auto operator=(const RosMirror &) -> RosMirror & = delete;
  RosMirror(RosMirror &&) = delete;
  auto operator=(RosMirror &&) -> RosMirror & = delete;
  ~RosMirror()
  {
    stop_ = true;
    thread_.join();
  }

private:
  static constexpr std::chrono::milliseconds kPoll{100};

  void run()
  {
    try {
      mirror();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "%s: ROS 2 mirror stopped: %s", topic_.c_str(), e.what());
    }
  }

  void mirror()
  {
    rclcpp::SerializedMessage msg;
    auto & raw = msg.get_rcl_serialized_message();
    const auto stopped = [this] { return stop_.load(); };
    while (!stopped()) {
      if (pub_->get_subscription_count() == 0) {
        core_.detach();
        std::this_thread::sleep_for(kPoll);
        continue;
      }
      const std::size_t n =
        core_.next(std::chrono::nanoseconds(kPoll).count(), stopped, [&](std::size_t size) {
          msg.reserve(size);
          return raw.buffer;
        });
      if (n == 0) {
        continue;
      }
      raw.buffer_length = n;
      pub_->publish(msg);
    }
  }

  std::string topic_;
  detail::RosMirror core_;
  std::shared_ptr<rclcpp::GenericPublisher> pub_;
  rclcpp::Logger logger_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, MemoryPolicy mem,
  std::uint32_t max_subscribers, std::uint32_t aux_size)
: Publisher(
    node, topic, fingerprint, slot_size, slot_count, gpu::Stream{}, mem, max_subscribers, aux_size)
{
}

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, Device device, MemoryPolicy mem,
  std::uint32_t max_subscribers, std::uint32_t aux_size)
: Publisher(
    node, topic, fingerprint, slot_size, slot_count, gpu::stream_for(device), mem, max_subscribers,
    aux_size)
{
}

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, const gpu::Stream & stream, MemoryPolicy mem,
  std::uint32_t max_subscribers, std::uint32_t aux_size)
: node_(node),
  fingerprint_(fingerprint),
  topic_(detail::resolve(node, topic)),
  signpost_(detail::signpost_name(topic_, fingerprint)),
  ch_(detail::Channel::create(
    signpost_, slot_size, slot_count, fingerprint, mem, max_subscribers, stream, aux_size)),
  announced_(detail::announce(node, signpost_, topic_, true))
{
  detail::trace_endpoint(this, node, topic_, signpost_, fingerprint, true);
}

Publisher::Publisher(Publisher &&) noexcept = default;
auto Publisher::operator=(Publisher &&) noexcept -> Publisher & = default;
Publisher::~Publisher() = default;

void Publisher::mirror_to_ros()
{
  if (!mirror_) {
    mirror_ = std::make_unique<RosMirror>(node_, ch_, topic_, signpost_, fingerprint_);
  }
}

}  // namespace flux
