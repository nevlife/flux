#include "flux/discovery.hpp"
#include "flux/ros/subscription.hpp"
#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/raw_meta.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_transport/topic_filter.hpp>

#include <rclcpp/version.h>
#include <rcutils/types/uint8_array.h>
#include <rmw/rmw.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr const char * kFrameType = "flux_msgs/msg/FluxFrame";
constexpr const char * kBagTopicSuffix = "/flux";
constexpr std::int64_t kTakeWaitNs = 100'000'000;

struct Options
{
  rosbag2_storage::StorageOptions storage;
  rosbag2_transport::RecordOptions record;
  std::chrono::milliseconds poll{1000};
};

const char * kUsage =
  "usage: flux_bag record [-a] [TOPIC ...] [-e REGEX] [-x REGEX] [--exclude-topics TOPIC ...]\n"
  "                       [-o OUT] [--poll SEC]\n"
  "\n"
  "Records the selected flux channels and ROS topics into one bag. A flux channel NAME is\n"
  "stored as flux_msgs/FluxFrame on NAME/flux; ROS topics are stored as they are.\n";

std::string default_uri()
{
  const std::time_t now = std::time(nullptr);
  char buf[64];
  std::strftime(buf, sizeof(buf), "rosbag2_%Y_%m_%d-%H_%M_%S", std::localtime(&now));
  return buf;
}

Options parse(const std::vector<std::string> & args)
{
  Options o;
  std::vector<std::string> * list = &o.record.topics;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    auto value = [&]() -> const std::string & {
      if (i + 1 >= args.size()) throw std::invalid_argument(a + " needs a value");
      return args[++i];
    };
    if (a == "-a" || a == "--all") {
      o.record.all_topics = true;
    } else if (a == "-e" || a == "--regex") {
      o.record.regex = value();
    } else if (a == "-x" || a == "--exclude-regex") {
      o.record.exclude_regex = value();
    } else if (a == "--exclude-topics") {
      list = &o.record.exclude_topics;
    } else if (a == "-o" || a == "--output") {
      o.storage.uri = value();
    } else if (a == "--poll") {
      o.poll = std::chrono::milliseconds(static_cast<int>(std::stod(value()) * 1000));
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
    } else {
      list->push_back(a);
    }
  }
  if (o.storage.uri.empty()) o.storage.uri = default_uri();
  o.record.rmw_serialization_format = rmw_get_serialization_format();
  return o;
}

// One flux channel into the bag: a thread that takes each frame, serializes it as a FluxFrame
// around the slot bytes, releases the borrow, and hands the buffer to the shared writer.
class ChannelRecorder
{
public:
  ChannelRecorder(
    rclcpp::Node & node, const flux::Topic & topic, std::shared_ptr<rosbag2_cpp::Writer> writer)
  : node_(node),
    writer_(std::move(writer)),
    flux_topic_(topic.key),
    fingerprint_(topic.fingerprint),
    sub_(node, topic.key, topic.fingerprint),
    msg_(std::make_shared<rosbag2_storage::SerializedBagMessage>())
  {
    const flux::ChannelStats stats = flux::read_channel_stats(topic.signpost);
    meta_.slot_size = stats.slot_size;
    meta_.slot_count = stats.slot_count;
    msg_->topic_name = topic.key + kBagTopicSuffix;
    msg_->serialized_data = rosbag2_storage::make_empty_serialized_message(0);

    rosbag2_storage::TopicMetadata meta;
    meta.name = msg_->topic_name;
    meta.type = kFrameType;
    meta.serialization_format = "cdr";
    writer_->create_topic(meta);
    thread_ = std::thread([this] { run(); });
  }

  ~ChannelRecorder()
  {
    stop_ = true;
    thread_.join();
    RCLCPP_INFO(
      node_.get_logger(), "%s: %lu frames written, %lu lost", flux_topic_.c_str(),
      static_cast<unsigned long>(written_), static_cast<unsigned long>(sub_.lost()));
  }

private:
  void run()
  {
    while (!stop_) {
      flux::FrameView v = sub_.take_blocking(kTakeWaitNs);
      if (!v) continue;
      const rclcpp::Time stamp = node_.now();
      const flux::FrameMeta & m = v.meta();
      meta_.dtype = m.dtype;
      meta_.ndim = m.ndim;
      std::memcpy(meta_.shape, m.shape, sizeof(meta_.shape));
      std::uint8_t meta_bytes[flux_bag::RawMeta::kMaxSize];

      flux_bag::FluxFrameFields f;
      f.sec = static_cast<std::int32_t>(stamp.nanoseconds() / 1'000'000'000);
      f.nanosec = static_cast<std::uint32_t>(stamp.nanoseconds() % 1'000'000'000);
      f.flux_topic = flux_topic_;
      f.fingerprint = fingerprint_;
      f.codec = "raw";
      f.meta = meta_bytes;
      f.meta_size = meta_.write(meta_bytes);
      f.data = v.data();
      f.data_size = v.size();

      rcutils_uint8_array_t & buf = *msg_->serialized_data;
      const std::size_t n = flux_bag::FluxFrameCdr::size(f);
      if (buf.buffer_capacity < n) {
        if (rcutils_uint8_array_resize(&buf, n) != RCUTILS_RET_OK) {
          throw std::bad_alloc();
        }
      }
      buf.buffer_length = flux_bag::FluxFrameCdr::write(f, buf.buffer);
      v.release();

#if RCLCPP_VERSION_MAJOR >= 28
      msg_->recv_timestamp = stamp.nanoseconds();
      msg_->send_timestamp = stamp.nanoseconds();
#else
      msg_->time_stamp = stamp.nanoseconds();
#endif
      writer_->write(msg_);
      ++written_;
    }
  }

  rclcpp::Node & node_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  std::string flux_topic_;
  std::uint64_t fingerprint_;
  flux::ros::Subscription sub_;
  flux_bag::RawMeta meta_;
  // Reused across frames: the writer copies out of it before write() returns because the
  // recorder runs without the write cache (StorageOptions::max_cache_size == 0).
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg_;
  std::uint64_t written_ = 0;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

std::atomic<bool> g_stop{false};

bool live_publisher(const flux::Topic & t)
{
  for (const auto & ep : t.endpoints) {
    if (ep.publisher) return true;
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  // Signals are taken here, not by rclcpp, so the writer is closed before the context is.
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

  auto writer = std::make_shared<rosbag2_cpp::Writer>();
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    writer, opt.storage, opt.record, "flux_bag_record");
  recorder->record();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(recorder);
  std::thread spin([&exec] { exec.spin(); });

  rosbag2_transport::TopicFilter filter(opt.record, nullptr, /*allow_unknown_types=*/true);
  std::map<std::string, std::unique_ptr<ChannelRecorder>> channels;
  const std::string & domain = flux::process_domain();
  while (!g_stop) {
    std::map<std::string, std::vector<std::string>> candidates;
    std::map<std::string, const flux::Topic *> live;
    const std::vector<flux::Topic> topics = flux::enumerate_topics();
    for (const flux::Topic & t : topics) {
      if (t.domain != domain || !t.key_exact || !live_publisher(t)) continue;
      if (channels.count(t.key)) continue;
      candidates[t.key] = {kFrameType};
      live[t.key] = &t;
    }
    for (const auto & [key, type] : filter.filter_topics(candidates)) {
      RCLCPP_INFO(recorder->get_logger(), "recording flux channel %s", key.c_str());
      channels.emplace(key, std::make_unique<ChannelRecorder>(*recorder, *live[key], writer));
    }
    for (auto waited = std::chrono::milliseconds(0); waited < opt.poll && !g_stop;
         waited += std::chrono::milliseconds(100)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  channels.clear();
  recorder->stop();
  exec.cancel();
  spin.join();
  rclcpp::shutdown();
  return 0;
}
