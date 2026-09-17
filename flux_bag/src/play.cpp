#include "flux/ros/publisher.hpp"
#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/raw_meta.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/info.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/play_options.hpp>
#include <rosbag2_transport/player.hpp>

#include <rclcpp/version.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr const char * kFrameType = "flux_msgs/msg/FluxFrame";

struct Options
{
  rosbag2_storage::StorageOptions storage;
  rosbag2_transport::PlayOptions play;
};

const char * kUsage =
  "usage: flux_bag play BAG [-r RATE]\n"
  "\n"
  "Plays a bag. flux_msgs/FluxFrame topics go straight from the file back into the flux\n"
  "channel they were recorded from; every other topic is played by the stock player.\n";

Options parse(const std::vector<std::string> & args)
{
  Options o;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    auto value = [&]() -> const std::string & {
      if (i + 1 >= args.size()) throw std::invalid_argument(a + " needs a value");
      return args[++i];
    };
    if (a == "-r" || a == "--rate") {
      o.play.rate = std::stof(value());
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
    } else if (o.storage.uri.empty()) {
      o.storage.uri = a;
    } else {
      throw std::invalid_argument("one bag only");
    }
  }
  if (o.storage.uri.empty()) throw std::invalid_argument("BAG is required");
  if (!(o.play.rate > 0)) throw std::invalid_argument("rate must be positive");
  return o;
}

std::int64_t bag_time(const rosbag2_storage::SerializedBagMessage & m)
{
#if RCLCPP_VERSION_MAJOR >= 28
  return m.recv_timestamp;
#else
  return m.time_stamp;
#endif
}

// The FluxFrame topics of one bag, read from the file and published on flux at the bag's own
// pace. One publisher per bag topic, created from the first frame's meta so the channel has the
// geometry the recorded one had.
class FluxPlayer
{
public:
  FluxPlayer(
    rclcpp::Node & node, const rosbag2_storage::StorageOptions & storage,
    std::vector<std::string> topics, float rate, std::int64_t start_ns)
  : node_(node), rate_(rate), start_ns_(start_ns)
  {
    reader_.open(storage);
    rosbag2_storage::StorageFilter filter;
    filter.topics = std::move(topics);
    reader_.set_filter(filter);
  }

  ~FluxPlayer() { stop(); }

  void start(std::chrono::steady_clock::time_point t0)
  {
    t0_ = t0;
    thread_ = std::thread([this] { run(); });
  }

  void stop()
  {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
  }

  bool finished() const noexcept { return finished_; }

private:
  void run()
  {
    while (!stop_ && reader_.has_next()) {
      const auto msg = reader_.read_next();
      const auto due = t0_ + std::chrono::nanoseconds(
                               static_cast<std::int64_t>((bag_time(*msg) - start_ns_) / rate_));
      while (!stop_ && std::chrono::steady_clock::now() < due) {
        std::this_thread::sleep_until(
          std::min(due, std::chrono::steady_clock::now() + std::chrono::milliseconds(100)));
      }
      if (stop_) break;
      publish(*msg);
    }
    finished_ = true;
  }

  void publish(const rosbag2_storage::SerializedBagMessage & msg)
  {
    flux_bag::FluxFrameFields f;
    flux_bag::RawMeta meta;
    const rcutils_uint8_array_t & raw = *msg.serialized_data;
    if (
      !flux_bag::FluxFrameCdr::read(raw.buffer, raw.buffer_length, f) || f.codec != "raw" ||
      !meta.read(f.meta, f.meta_size)) {
      RCLCPP_ERROR_ONCE(
        node_.get_logger(), "%s: not a raw FluxFrame, skipped", msg.topic_name.c_str());
      return;
    }
    auto it = pubs_.find(msg.topic_name);
    if (it == pubs_.end()) {
      it = pubs_
             .emplace(
               msg.topic_name,
               std::make_unique<flux::ros::Publisher>(
                 node_, std::string(f.flux_topic), f.fingerprint, meta.slot_size, meta.slot_count))
             .first;
      RCLCPP_INFO(
        node_.get_logger(), "%s -> flux channel %s", msg.topic_name.c_str(),
        std::string(f.flux_topic).c_str());
    }
    flux::WriteSlot w = it->second->loan(meta.dtype, meta.shape, meta.ndim);
    if (!w || w.capacity() < f.data_size) return;
    std::memcpy(w.data(), f.data, f.data_size);
    if (const flux::Published p = w.commit(); flux::faulted(p)) {
      RCLCPP_ERROR_ONCE(node_.get_logger(), "publish refused: %s", flux::to_string(p));
    }
  }

  rclcpp::Node & node_;
  float rate_;
  std::int64_t start_ns_;
  std::chrono::steady_clock::time_point t0_;
  rosbag2_cpp::Reader reader_;
  std::map<std::string, std::unique_ptr<flux::ros::Publisher>> pubs_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> finished_{false};
  std::thread thread_;
};

std::atomic<bool> g_stop{false};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { g_stop = true; });
  std::signal(SIGTERM, [](int) { g_stop = true; });
  Options opt;
  try {
    opt = parse(rclcpp::remove_ros_arguments(argc, argv));
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n%s", e.what(), kUsage);
    return 2;
  }

  const rosbag2_storage::BagMetadata metadata =
    rosbag2_cpp::Info().read_metadata(opt.storage.uri, opt.storage.storage_id);
  std::vector<std::string> flux_topics;
  for (const auto & t : metadata.topics_with_message_count) {
    if (t.topic_metadata.type == kFrameType) flux_topics.push_back(t.topic_metadata.name);
  }
  opt.play.exclude_topics_to_filter = flux_topics;
  opt.play.disable_keyboard_controls = true;

  auto player = std::make_shared<rosbag2_transport::Player>(opt.storage, opt.play, "flux_bag_play");
  FluxPlayer flux_player(
    *player, opt.storage, flux_topics, opt.play.rate,
    metadata.starting_time.time_since_epoch().count());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(player);
  std::thread spin([&exec] { exec.spin(); });

  player->play();
  player->wait_for_playback_to_start();
  flux_player.start(std::chrono::steady_clock::now());
  while (!g_stop && !(flux_player.finished() &&
                      player->wait_for_playback_to_finish(std::chrono::milliseconds(100)))) {
  }
  flux_player.stop();
  player->stop();
  exec.cancel();
  spin.join();
  rclcpp::shutdown();
  return 0;
}
