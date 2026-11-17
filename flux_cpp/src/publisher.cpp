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

namespace
{

auto options_of(
  std::uint32_t slot_size, std::uint32_t slot_count, const gpu::Stream & stream, MemoryPolicy mem,
  std::uint32_t max_subscribers, std::uint32_t aux_size) -> PublisherOptions
{
  PublisherOptions o;
  o.slot_size = slot_size;
  o.slot_count = slot_count;
  o.stream = stream;
  o.memory = mem;
  o.max_subscribers = max_subscribers;
  o.aux_size = aux_size;
  return o;
}

auto config_of(const PublisherOptions & o) -> flux_publisher_config
{
  flux_publisher_config c = flux_publisher_config_default();
  c.alloc = static_cast<std::int32_t>(o.alloc);
  c.slot_size = o.slot_size;
  c.slot_count = o.slot_count;
  c.max_slot_size = o.max_slot_size;
  c.max_slot_count = o.max_slot_count;
  c.max_subscribers = o.max_subscribers;
  c.aux_size = o.aux_size;
  c.alloc_on_subscriber = o.alloc_on_subscriber;
  return c;
}

}  // namespace

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, const PublisherOptions & options,
  std::uint64_t fingerprint)
: node_(node),
  fingerprint_(fingerprint),
  topic_(detail::resolve(node, topic)),
  signpost_(detail::signpost_name(topic_, fingerprint)),
  ch_(detail::Channel::create_config(
    signpost_, config_of(options), fingerprint, options.memory,
    options.stream.declared() ? options.stream : gpu::stream_for(options.device))),
  announced_(detail::announce(node, signpost_, topic_, true))
{
  detail::trace_endpoint(this, node, topic_, signpost_, fingerprint, true);
}

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, MemoryPolicy mem,
  std::uint32_t max_subscribers, std::uint32_t aux_size)
: Publisher(
    node, topic, options_of(slot_size, slot_count, {}, mem, max_subscribers, aux_size), fingerprint)
{
}

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, Device device, MemoryPolicy mem,
  std::uint32_t max_subscribers, std::uint32_t aux_size)
: Publisher(
    node, topic,
    options_of(slot_size, slot_count, gpu::stream_for(device), mem, max_subscribers, aux_size),
    fingerprint)
{
}

Publisher::Publisher(
  const NodeRef & node, const std::string & topic, std::uint64_t fingerprint,
  std::uint32_t slot_size, std::uint32_t slot_count, const gpu::Stream & stream, MemoryPolicy mem,
  std::uint32_t max_subscribers, std::uint32_t aux_size)
: Publisher(
    node, topic, options_of(slot_size, slot_count, stream, mem, max_subscribers, aux_size),
    fingerprint)
{
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
