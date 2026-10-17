// The relay: one ROS 2 node that republishes flux channels on demand.
//
// Every poll the bridge lists the live channels on the host and, for each one whose fingerprint
// names an installed message type, asks the ROS graph whether anyone other than itself
// subscribes to a topic of that name. A relay starts when that becomes true and stops when it
// stops being true, so a channel nobody watches costs nothing. The frame path is flux slot ->
// CDR written by flux -> rmw, which is the cost of using stock tools on a flux channel.

#include "flux/schemas.hpp"
#include "flux/subscription.hpp"
#include "flux/topics.hpp"
#include "flux_bridge/tools.hpp"

#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{

constexpr std::int64_t kWaitNs = 200'000'000;
constexpr const char * kNodePrefix = "flux_bridge_";
constexpr const char * kUsage =
  "usage: ros2 run flux_bridge bridge [-h] [--poll SEC]\n"
  "\n"
  "Republish flux channels as DDS topics while a ROS 2 subscriber wants them.\n";

// Global: a signal handler reaches nothing else.
std::atomic<bool> g_stop{false};  // NOLINT(*-avoid-non-const-global-variables)

// One channel relayed to one ROS 2 publisher on its own thread. A thread rather than an executor
// because relays come and go while the bridge runs; take_blocking parks on the futex, so an idle
// relay costs no CPU.
class Relay
{
public:
  Relay(rclcpp::Node & node, const flux::Topic & topic, flux::Schemas::Type type)
  : fingerprint_(topic.fingerprint),
    signpost_(topic.signpost),
    type_(type),
    sub_(node, topic.key, topic.fingerprint, flux::QoS(1).max_borrow(1)),
    pub_(node.create_generic_publisher(
      topic.key, type.ros_type(), rclcpp::QoS(1).best_effort().durability_volatile())),
    thread_([this] { run(); })
  {
  }
  Relay(const Relay &) = delete;
  Relay & operator=(const Relay &) = delete;
  Relay(Relay &&) = delete;
  Relay & operator=(Relay &&) = delete;
  ~Relay() { stop(); }

  // Joins, so the counts read after it are final.
  void stop()
  {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
  }

  std::uint64_t fingerprint() const noexcept { return fingerprint_; }
  const std::string & signpost() const noexcept { return signpost_; }
  // What ended the relay's thread early, or null.
  const std::string * failure() const noexcept
  {
    return failed_.load(std::memory_order_acquire) ? &failure_ : nullptr;
  }
  std::uint64_t published() const noexcept { return published_; }
  std::uint64_t unreadable() const noexcept { return unreadable_; }

private:
  void run()
  {
    try {
      relay();
    } catch (const std::exception & e) {
      failure_ = e.what();
      failed_.store(true, std::memory_order_release);
    }
  }

  void relay()
  {
    rclcpp::SerializedMessage msg;
    std::vector<std::uint8_t> bytes;
    while (!stop_) {
      flux::FrameView frame = sub_.take_blocking(kWaitNs);
      if (!frame) continue;
      // Sizing and writing must read the same bytes, and a peer can still write a committed slot.
      const auto * data = static_cast<const std::uint8_t *>(frame.data());
      bytes.assign(data, data + frame.size());
      frame.release();
      try {
        const std::size_t n = type_.cdr_size(bytes.data(), bytes.size());
        msg.reserve(n);
        auto & raw = msg.get_rcl_serialized_message();
        raw.buffer_length =
          type_.write_cdr(bytes.data(), bytes.size(), raw.buffer, raw.buffer_capacity);
      } catch (const flux::Error &) {
        // A frame that disagrees with its schema is a wrong message, not the end of the channel.
        ++unreadable_;
        continue;
      }
      pub_->publish(msg);
      ++published_;
    }
  }

  std::uint64_t fingerprint_;
  std::string signpost_;
  flux::Schemas::Type type_;
  flux::Subscription sub_;
  std::shared_ptr<rclcpp::GenericPublisher> pub_;
  std::atomic<bool> stop_{false};
  std::atomic<std::uint64_t> published_{0};
  std::atomic<std::uint64_t> unreadable_{0};
  std::string failure_;
  std::atomic<bool> failed_{false};
  std::thread thread_;
};

bool external_subscribers(const rclcpp::Node & node, const std::string & topic)
{
  for (const auto & info : node.get_subscriptions_info_by_topic(topic)) {
    if (info.node_name() != node.get_name() || info.node_namespace() != node.get_namespace())
      return true;
  }
  return false;
}

class Bridge
{
public:
  explicit Bridge(rclcpp::Node & node) : node_(node) {}

  void tick()
  {
    std::map<std::string, flux::Topic> bridgeable;
    std::set<std::string> seen;
    for (flux::Topic & t : flux::enumerate_topics()) {
      if (!t.key_exact) continue;
      seen.insert(t.signpost);
      const bool valid = flux_bridge::is_ros_topic_name(t.key);
      if (!valid || !schemas_.contains(t.fingerprint)) {
        // Reported once while the signpost lasts, and its segment is not looked at again.
        if (!reported_.count(t.signpost) && flux::read_channel_stats(t.signpost).live) {
          reported_.insert(t.signpost);
          report_unrelayable(t, valid);
        }
        continue;
      }
      const flux::ChannelStats stats = flux::read_channel_stats(t.signpost);
      if (!stats.live) continue;
      if (const auto out = left_out_.find(t.signpost); out != left_out_.end()) {
        if (out->second == stats.epoch) continue;
        left_out_.erase(out);  // a new publisher: tried again
      }
      bridgeable.emplace(t.key, std::move(t));
    }
    for (auto it = reported_.begin(); it != reported_.end();) {
      it = seen.count(*it) ? std::next(it) : reported_.erase(it);
    }
    for (auto it = left_out_.begin(); it != left_out_.end();) {
      it = seen.count(it->first) ? std::next(it) : left_out_.erase(it);
    }
    for (auto it = relays_.begin(); it != relays_.end();) {
      if (const std::string * why = it->second->failure()) {
        left_out(it->first, it->second->signpost(), *why);
        it = relays_.erase(it);
        continue;
      }
      const auto now = bridgeable.find(it->first);
      // A publisher that came back under another type within one poll is a new channel.
      if (
        now != bridgeable.end() && now->second.fingerprint == it->second->fingerprint() &&
        external_subscribers(node_, it->first)) {
        ++it;
        continue;
      }
      it->second->stop();
      RCLCPP_INFO(
        node_.get_logger(), "%s: stopped after %llu frames, %llu unreadable", it->first.c_str(),
        static_cast<unsigned long long>(it->second->published()),    // NOLINT(runtime/int)
        static_cast<unsigned long long>(it->second->unreadable()));  // NOLINT(runtime/int)
      it = relays_.erase(it);
    }
    for (const auto & [key, topic] : bridgeable) {
      if (
        relays_.count(key) || left_out_.count(topic.signpost) || !external_subscribers(node_, key))
        continue;
      const flux::Schemas::Type type = schemas_.at(topic.fingerprint);
      try {
        relays_.emplace(key, std::make_unique<Relay>(node_, topic, type));
      } catch (const std::exception & e) {
        left_out(key, topic.signpost, e.what());
        continue;
      }
      RCLCPP_INFO(node_.get_logger(), "%s: relaying as %s", key.c_str(), type.ros_type().c_str());
    }
  }

  std::size_t types() const noexcept { return schemas_.size(); }

private:
  // Tried again once its publisher restarts; the other channels keep relaying.
  void left_out(const std::string & key, const std::string & signpost, const std::string & why)
  {
    left_out_[signpost] = flux::read_channel_stats(signpost).epoch;
    RCLCPP_ERROR(
      node_.get_logger(), "%s: not relayed until its publisher restarts: %s", key.c_str(),
      why.c_str());
  }

  void report_unrelayable(const flux::Topic & t, bool valid_name) const
  {
    if (!valid_name) {
      RCLCPP_WARN(node_.get_logger(), "a channel whose key is not a ROS topic name is not relayed");
      return;
    }
    RCLCPP_INFO(
      node_.get_logger(), "%s: no installed message type has fingerprint 0x%016llx", t.key.c_str(),
      static_cast<unsigned long long>(t.fingerprint));  // NOLINT(runtime/int)
  }

  rclcpp::Node & node_;
  flux::Schemas schemas_;
  std::map<std::string, std::unique_ptr<Relay>> relays_;
  // Signposts of live channels already reported as not relayable.
  std::set<std::string> reported_;
  // Signposts of channels that refused a relay, with the publisher epoch that refused.
  std::map<std::string, std::uint32_t> left_out_;
};

// Empty for -h: the usage goes to stdout and the bridge does not start.
std::optional<std::chrono::duration<double>> parse(const std::vector<std::string> & args)
{
  constexpr std::string_view kPollEq = "--poll=";
  std::chrono::duration<double> poll{1.0};
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "-h" || a == "--help") return std::nullopt;
    std::string text;
    if (a.compare(0, kPollEq.size(), kPollEq) == 0) {
      text = a.substr(kPollEq.size());
    } else if (a == "--poll") {
      if (i + 1 >= args.size())
        throw std::invalid_argument("argument --poll: expected one argument");
      text = args[++i];
    } else {
      throw std::invalid_argument("unknown argument " + a);
    }
    poll = std::chrono::duration<double>(flux_bridge::positive_number("--poll", text));
  }
  return poll;
}

}  // namespace

int main(int argc, char ** argv)
{
  // Signals are taken here, not by rclcpp, so relays stop before the context they publish on.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { g_stop = true; });
  std::signal(SIGTERM, [](int) { g_stop = true; });
  std::optional<std::chrono::duration<double>> poll;
  try {
    poll = parse(rclcpp::remove_ros_arguments(argc, argv));
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n" << kUsage;
    rclcpp::shutdown();
    return 2;
  }
  if (!poll) {
    std::cout << kUsage;
    rclcpp::shutdown();
    return 0;
  }

  int rc = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>(kNodePrefix + std::to_string(getpid()));
    Bridge bridge(*node);
    RCLCPP_INFO(node->get_logger(), "%zu installed message type(s) can be relayed", bridge.types());
    while (!g_stop) {
      bridge.tick();
      for (auto waited = std::chrono::duration<double>(0); waited < *poll && !g_stop;
           waited += std::chrono::milliseconds(50)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
