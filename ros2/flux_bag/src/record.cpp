#include "flux/schemas.hpp"
#include "flux/subscription.hpp"
#include "flux/topics.hpp"
#include "flux/trace.h"
#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/frame_encoder.hpp"
#include "flux_bag/spin_thread.hpp"
#include "flux_bridge/tools.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_compression/compression_options.hpp>
#include <rosbag2_compression/sequential_compression_writer.hpp>
#include <rosbag2_cpp/cache/message_cache.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_transport/topic_filter.hpp>

#include <rcutils/types/uint8_array.h>
#include <rmw/rmw.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
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
  std::string node_name;
  std::chrono::duration<double> poll{0.1};
};

constexpr const char * kUsage =
  "usage: ros2 bag record_flux [options], see ros2 bag record_flux -h\n"
  "\n"
  "This program takes the options ros2 bag record_flux builds, as a YAML file: the\n"
  "storage and record options of rosbag2's YAML converters, node_name and poll (seconds).\n";

// The verb checked every value, as ros2 bag record does.
auto parse(const std::vector<std::string> & args) -> Options
{
  if (args.size() != 2) {
    throw std::invalid_argument("expects the options file as its one argument");
  }
  const YAML::Node n = YAML::LoadFile(args[1]);
  Options o;
  o.storage = n["storage"].as<rosbag2_storage::StorageOptions>();
  YAML::convert<rosbag2_transport::RecordOptions>::decode(n["record"], o.record);
  o.node_name = n["node_name"].as<std::string>();
  o.poll = std::chrono::duration<double>(n["poll"].as<double>());
  return o;
}

// `mono_ns` (CLOCK_MONOTONIC) as a time on the clock `now` came from, `now` having been read
// when CLOCK_MONOTONIC was `mono_now_ns`.
auto on_clock(std::uint64_t mono_ns, std::int64_t mono_now_ns, const rclcpp::Time & now)
  -> std::int64_t
{
  return now.nanoseconds() - (mono_now_ns - static_cast<std::int64_t>(mono_ns));
}

auto monotonic_ns() -> std::int64_t
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1'000'000'000LL + ts.tv_nsec;
}

// Message buffers their last owner gives back instead of freeing: the writer's cache, its snapshot
// buffer and its compression queue keep a written message for a while, and the buffer is reused
// once they let it go. A fresh frame-sized buffer costs a page fault per page, slower than the
// frames come.
class BufferPool : public std::enable_shared_from_this<BufferPool>
{
public:
  auto get(std::size_t n) -> std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  {
    std::unique_ptr<rosbag2_storage::SerializedBagMessage> msg;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (!free_.empty()) {
        msg = std::move(free_.back());
        free_.pop_back();
      }
    }
    if (!msg) {
      msg = std::make_unique<rosbag2_storage::SerializedBagMessage>();
      msg->serialized_data = rosbag2_storage::make_empty_serialized_message(n);
    } else if (msg->serialized_data->buffer_capacity < n) {
      if (rcutils_uint8_array_resize(msg->serialized_data.get(), n) != RCUTILS_RET_OK) {
        throw std::bad_alloc();
      }
    }
    // The pool lives as long as a message from it: the writer may hold one past its channel.
    return {msg.release(), [pool = shared_from_this()](auto * m) { pool->put(m); }};
  }

private:
  void put(rosbag2_storage::SerializedBagMessage * m) noexcept
  {
    std::unique_ptr<rosbag2_storage::SerializedBagMessage> owned(m);
    const std::lock_guard<std::mutex> lock(mutex_);
    try {
      free_.push_back(std::move(owned));
    } catch (const std::bad_alloc &) {
      // Freed instead.
    }
  }

  std::mutex mutex_;
  // No more than were in use at once: in the writer and being filled.
  std::vector<std::unique_ptr<rosbag2_storage::SerializedBagMessage>> free_;
};

// rosbag2's cache drops a message it has no room for and counts it per topic, ROS message or flux
// frame, but Jazzy logs the counts only when a bag file closes (later rosbag2 has a MESSAGES_LOST
// writer event). These log them while recording, from rosbag2's own counts.
constexpr auto kLossReportEvery = std::chrono::seconds(5);
using LostPerTopic = std::unordered_map<std::string, std::uint32_t>;

// rosbag2's count, a protected member of MessageCache.
struct LostIn : rosbag2_cpp::cache::MessageCache
{
  static auto of(const rosbag2_cpp::cache::MessageCache & cache) -> const LostPerTopic &
  {
    return cache.*(&LostIn::messages_dropped_per_topic_);
  }
};

// Copies of the counts from the writing threads, logged by the main thread: the writing threads
// hold the writer's lock.
class CacheLosses
{
public:
  // Assigned over the last copy, which reuses its entries: only a topic's first loss allocates.
  void post(const LostPerTopic & lost)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    posted_ = lost;
    fresh_ = true;
  }

  // What was lost since the last line, or empty.
  auto line() -> std::string
  {
    LostPerTopic now;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (!fresh_) {
        return {};
      }
      fresh_ = false;
      now = posted_;
    }
    std::string out;
    for (const auto & [topic, n] : std::map<std::string, std::uint32_t>(now.begin(), now.end())) {
      const auto it = logged_.find(topic);
      if (const std::uint32_t since = n - (it == logged_.end() ? 0 : it->second); since > 0) {
        out += (out.empty() ? "" : ", ") + std::to_string(since) + " on " + topic;
      }
    }
    logged_ = std::move(now);
    return out;
  }

private:
  std::mutex mutex_;
  LostPerTopic posted_;
  bool fresh_ = false;
  LostPerTopic logged_;
};

// rosbag2's cache, posting its counts when a topic first loses a message and then at most every
// kLossReportEvery. Only the writer pushes through it; rosbag2's consumer thread keeps the cache
// itself.
class ReportingCache : public rosbag2_cpp::cache::MessageCacheInterface
{
public:
  ReportingCache(
    std::shared_ptr<rosbag2_cpp::cache::MessageCache> cache, std::shared_ptr<CacheLosses> losses)
  : cache_(std::move(cache)), losses_(std::move(losses))
  {
  }

  // With the writer's lock held, as rosbag2 counts.
  void push(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override
  {
    cache_->push(std::move(msg));
    const LostPerTopic & lost = LostIn::of(*cache_);
    if (lost.empty()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (lost.size() == topics_ && now - posted_ < kLossReportEvery) {
      return;
    }
    topics_ = lost.size();
    posted_ = now;
    losses_->post(lost);
  }

  auto get_consumer_buffer() -> std::shared_ptr<rosbag2_cpp::cache::CacheBufferInterface> override
  {
    return cache_->get_consumer_buffer();
  }
  void release_consumer_buffer() override { cache_->release_consumer_buffer(); }
  void wait_for_data() override { cache_->wait_for_data(); }
  void swap_buffers() override { cache_->swap_buffers(); }
  void begin_flushing() override { cache_->begin_flushing(); }
  void done_flushing() override { cache_->done_flushing(); }
  void log_dropped() override { cache_->log_dropped(); }
  void notify_data_ready() override { cache_->notify_data_ready(); }

private:
  std::shared_ptr<rosbag2_cpp::cache::MessageCache> cache_;
  std::shared_ptr<CacheLosses> losses_;
  std::size_t topics_ = 0;
  std::chrono::steady_clock::time_point posted_;
};

// A rosbag2 writer whose cache reports its losses. Snapshot mode's circular cache overwrites
// instead of losing and is left as it is.
template <class Base>
class ReportingWriter : public Base
{
public:
  template <class... Args>
  explicit ReportingWriter(std::shared_ptr<CacheLosses> losses, Args &&... args)
  : Base(std::forward<Args>(args)...), losses_(std::move(losses))
  {
  }

  void open(
    const rosbag2_storage::StorageOptions & storage,
    const rosbag2_cpp::ConverterOptions & converter) override
  {
    Base::open(storage, converter);
    if (
      auto cache =
        std::dynamic_pointer_cast<rosbag2_cpp::cache::MessageCache>(this->message_cache_)) {
      this->message_cache_ = std::make_shared<ReportingCache>(std::move(cache), losses_);
    }
  }

private:
  std::shared_ptr<CacheLosses> losses_;
};

// rosbag2_transport::ReaderWriterFactory::make_writer (Jazzy), with the reporting writers.
auto make_writer(
  const rosbag2_transport::RecordOptions & record,
  const std::shared_ptr<CacheLosses> & losses) -> std::shared_ptr<rosbag2_cpp::Writer>
{
  std::unique_ptr<rosbag2_cpp::writer_interfaces::BaseWriterInterface> impl;
  if (!record.compression_format.empty()) {
    rosbag2_compression::CompressionOptions compression{
      record.compression_format,
      rosbag2_compression::compression_mode_from_string(record.compression_mode),
      record.compression_queue_size, record.compression_threads,
      record.compression_threads_priority};
    if (compression.compression_threads < 1) {
      compression.compression_threads = std::thread::hardware_concurrency();
    }
    impl = std::make_unique<ReportingWriter<rosbag2_compression::SequentialCompressionWriter>>(
      losses, compression);
  } else {
    impl = std::make_unique<ReportingWriter<rosbag2_cpp::writers::SequentialWriter>>(losses);
  }
  return std::make_shared<rosbag2_cpp::Writer>(std::move(impl));
}

// One flux channel into the bag: a thread that takes each frame, serializes it as a FluxFrame
// around the slot bytes into a buffer of its own, releases the borrow, and writes the buffer, as
// rosbag2 writes a ROS message from its subscription. rosbag2's cache is the only buffer: while
// the writer blocks, frames wait in their slots.
class ChannelRecorder
{
public:
  ChannelRecorder(
    rosbag2_transport::Recorder & recorder, const flux::Topic & topic, rosbag2_cpp::Writer & writer)
  : recorder_(recorder),
    writer_(writer),
    flux_topic_(topic.key),
    bag_topic_(topic.key + kBagTopicSuffix),
    fingerprint_(topic.fingerprint),
    sub_(recorder, topic.key, topic.fingerprint),
    encoder_(topic.key, topic.fingerprint)
  {
    rosbag2_storage::TopicMetadata meta;
    meta.name = bag_topic_;
    meta.type = kFrameType;
    meta.serialization_format = "cdr";
    writer_.create_topic(meta);
    thread_ = std::thread([this] { run(); });
  }

  ChannelRecorder(const ChannelRecorder &) = delete;
  auto operator=(const ChannelRecorder &) -> ChannelRecorder & = delete;
  ChannelRecorder(ChannelRecorder &&) = delete;
  auto operator=(ChannelRecorder &&) -> ChannelRecorder & = delete;
  ~ChannelRecorder()
  {
    stop_ = true;
    thread_.join();
    // What rosbag2's cache drops of the frames handed to it is in rosbag2's own log.
    RCLCPP_INFO(
      recorder_.get_logger(),
      "%s: %lu frames handed to rosbag2, %lu lost in the channel before they were taken, %lu left "
      "out with fields in the arena",
      flux_topic_.c_str(), static_cast<unsigned long>(handed_),
      static_cast<unsigned long>(sub_.lost()), static_cast<unsigned long>(arena_left_));
  }

  // What ended the thread early, rethrown on the main thread so the bag is still closed.
  auto error() const -> std::exception_ptr
  {
    const std::lock_guard<std::mutex> lock(error_mutex_);
    return error_;
  }

  // Why the channel refused this recorder later on (a re-attach), or empty: only this channel
  // stops, the bag goes on.
  auto key() const noexcept -> const std::string & { return flux_topic_; }

  auto left_out() const -> std::string
  {
    const std::lock_guard<std::mutex> lock(error_mutex_);
    return left_out_;
  }

private:
  void run()
  {
    try {
      while (!stop_) {
        record_frame(sub_.take_blocking(kTakeWaitNs));
      }
      // What the channel holds when recording stops goes in too. At most a depth of it: older
      // frames are not taken anyway, and a publisher that keeps going would never let this end.
      for (std::uint32_t i = 0; i < sub_.qos().depth(); ++i) {
        flux::FrameView v = sub_.take();
        if (!v) {
          break;
        }
        record_frame(std::move(v));
      }
    } catch (const flux::Error & e) {
      const std::lock_guard<std::mutex> lock(error_mutex_);
      left_out_ = e.what();
    } catch (...) {
      const std::lock_guard<std::mutex> lock(error_mutex_);
      error_ = std::current_exception();
    }
  }

  void record_frame(flux::FrameView v)
  {
    if (!v) {
      return;
    }
    const void * channel = v.trace_channel();
    const std::uint64_t ticket = v.ticket();
    // The recorder fell behind its depth and the channel overwrote these before the take.
    if (const std::uint64_t lost = sub_.lost(); lost != reported_lost_) {
      flux_trace_record_drop(
        channel, 0, static_cast<std::uint32_t>(lost - reported_lost_), FLUX_TRACE_RECORD_OVERRUN);
      reported_lost_ = lost;
    }
    // As rosbag2 does with a message that arrives while paused: taken, not written.
    if (recorder_.is_paused()) {
      return;
    }
    const rclcpp::Time stamp = recorder_.now();
    // Simulated time before its first /clock: rosbag2 writes no message then either.
    if (stamp.nanoseconds() == 0) {
      return;
    }
    const std::int64_t sent = on_clock(v.publish_ns(), monotonic_ns(), stamp);
    flux_bag::FluxFrameFields f;
    if (const char * why = encoder_.encode(v, sub_, sent, f)) {
      if (arena_left_++ == 0) {
        RCLCPP_WARN(
          recorder_.get_logger(),
          "%s: a frame with fields in its publisher's arena is not recorded: %s",
          flux_topic_.c_str(), why);
      }
      return;
    }

    auto msg = pool_->get(flux_bag::FluxFrameCdr::size(f));
    msg->topic_name = bag_topic_;
    rcutils_uint8_array_t & buf = *msg->serialized_data;
    buf.buffer_length = flux_bag::FluxFrameCdr::write(f, buf.buffer);
    v.release();

    // As rosbag2 records a DDS message: received now, sent when its publisher committed it.
    msg->recv_timestamp = stamp.nanoseconds();
    msg->send_timestamp = sent;
    try {
      writer_.write(std::move(msg));
    } catch (...) {
      flux_trace_record_drop(channel, ticket, 1, FLUX_TRACE_RECORD_WRITE_FAILED);
      throw;
    }
    ++handed_;
  }

  rosbag2_transport::Recorder & recorder_;
  rosbag2_cpp::Writer & writer_;
  std::string flux_topic_;
  std::uint64_t arena_left_ = 0;
  std::string bag_topic_;
  std::uint64_t fingerprint_;
  flux::Subscription sub_;
  flux_bag::FrameEncoder encoder_;
  std::shared_ptr<BufferPool> pool_ = std::make_shared<BufferPool>();
  std::uint64_t handed_ = 0;
  std::uint64_t reported_lost_ = 0;
  std::atomic<bool> stop_{false};
  mutable std::mutex error_mutex_;
  std::exception_ptr error_;
  std::string left_out_;
  std::thread thread_;
};

// Global: a signal handler reaches nothing else.
std::atomic<bool> g_stop{false};  // NOLINT(*-avoid-non-const-global-variables)

// A channel down this long is let go. A restarted publisher is usually back sooner, and its
// recorder re-attaches to it without waiting for the next listing.
constexpr auto kKeepDown = std::chrono::seconds(10);
constexpr auto kStopCheck = std::chrono::milliseconds(100);

struct Recorded
{
  std::unique_ptr<ChannelRecorder> recorder;
  // Set by the first listing that found the channel down, cleared by one that finds it up.
  std::optional<std::chrono::steady_clock::time_point> down_since;
};

void run(Options & opt)
{
  // As ros2 bag record (rosbag2_py's Recorder) makes it: the rmw's format unless one was asked
  // for, and a compressing writer for a compression mode.
  if (opt.record.rmw_serialization_format.empty()) {
    opt.record.rmw_serialization_format = rmw_get_serialization_format();
  }
  const auto losses = std::make_shared<CacheLosses>();
  std::shared_ptr<rosbag2_cpp::Writer> writer = make_writer(opt.record, losses);
  auto recorder =
    std::make_shared<rosbag2_transport::Recorder>(writer, opt.storage, opt.record, opt.node_name);
  recorder->record();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(recorder);
  const flux_bag::SpinThread spin(exec);

  // Without a graph the filter leaves out what only ROS topics have: unpublished and leaf topics.
  rosbag2_transport::TopicFilter filter(opt.record, nullptr, /*allow_unknown_types=*/true);
  // --topic-types and --exclude-topic-types see a flux channel as the ROS type its fingerprint
  // names, when that is installed.
  std::optional<flux::Schemas> schemas;
  if (!opt.record.topic_types.empty() || !opt.record.exclude_topic_types.empty()) {
    schemas.emplace();
  }
  const auto type_of = [&](std::uint64_t fingerprint) -> std::string {
    return schemas && schemas->contains(fingerprint) ? schemas->at(fingerprint).ros_type()
                                                     : kFrameType;
  };
  // By signpost: one name can carry channels of two fingerprints, each recorded on its own.
  std::map<std::string, Recorded> channels;
  std::set<std::string> bad_keys;
  // Signposts of channels that refused a recorder, with the publisher epoch that refused: tried
  // again once the publisher restarts.
  std::map<std::string, std::uint32_t> left_out;
  const auto leave_out =
    [&](const std::string & key, const std::string & signpost, const std::string & why) {
      left_out[signpost] = flux::read_channel_stats(signpost).epoch;
      RCLCPP_ERROR(
        recorder->get_logger(), "flux channel %s not recorded until its publisher restarts: %s",
        key.c_str(), why.c_str());
    };
  std::exception_ptr error;
  bool listed_once = false;
  while (!g_stop && !error) {
    const auto listed = std::chrono::steady_clock::now();
    // --no-discovery: the channels up at the start only, as rosbag2 does with its topics.
    if (!listed_once || !opt.record.is_discovery_disabled) {
      for (auto & [signpost, channel] : channels) {
        if (!channel.down_since) {
          channel.down_since = listed;
        }
      }
      std::map<std::string, std::vector<std::string>> candidates;
      std::map<std::string, std::vector<const flux::Topic *>> live;
      std::set<std::string> bad;
      std::set<std::string> seen;
      const std::vector<flux::Topic> topics = flux::enumerate_topics();
      for (const flux::Topic & t : topics) {
        seen.insert(t.signpost);
        if (!t.key_exact) {
          continue;
        }
        const flux::ChannelStats stats = flux::read_channel_stats(t.signpost);
        if (!stats.live) {
          continue;
        }
        if (const auto out = left_out.find(t.signpost); out != left_out.end()) {
          if (out->second == stats.epoch) {
            continue;
          }
          left_out.erase(out);
        }
        if (!flux_bridge::is_ros_topic_name(t.key)) {
          if (!bad_keys.count(t.signpost)) {
            RCLCPP_WARN(
              recorder->get_logger(),
              "a channel whose key is not a ROS topic name is not recorded");
          }
          bad.insert(t.signpost);
          continue;
        }
        if (const auto it = channels.find(t.signpost); it != channels.end()) {
          it->second.down_since.reset();
          continue;
        }
        candidates[t.key] = {type_of(t.fingerprint)};
        live[t.key].push_back(&t);
      }
      bad_keys = std::move(bad);
      for (auto it = left_out.begin(); it != left_out.end();) {
        it = seen.count(it->first) ? std::next(it) : left_out.erase(it);
      }
      // Its thread, subscription and hold on the signpost go with it.
      for (auto it = channels.begin(); it != channels.end();) {
        const auto & down = it->second.down_since;
        const bool gone = down && listed - *down > kKeepDown;
        const std::string why = it->second.recorder->left_out();
        if (!why.empty()) {
          leave_out(it->second.recorder->key(), it->first, why);
        }
        it = gone || !why.empty() ? channels.erase(it) : std::next(it);
      }
      for (const auto & [key, type] : filter.filter_topics(candidates)) {
        for (const flux::Topic * t : live[key]) {
          std::unique_ptr<ChannelRecorder> r;
          try {
            r = std::make_unique<ChannelRecorder>(*recorder, *t, *writer);
          } catch (const flux::Error & e) {
            leave_out(key, t->signpost, e.what());
            continue;
          }
          RCLCPP_INFO(recorder->get_logger(), "recording flux channel %s", key.c_str());
          channels.emplace(t->signpost, Recorded{std::move(r), std::nullopt});
        }
      }
    }
    listed_once = true;
    while (!g_stop) {
      if (const std::string lost = losses->line(); !lost.empty()) {
        RCLCPP_WARN(
          recorder->get_logger(), "rosbag2's cache is full: lost %s since the last report",
          lost.c_str());
      }
      const std::chrono::duration<double> left =
        opt.poll - (std::chrono::steady_clock::now() - listed);
      if (left <= left.zero()) {
        break;
      }
      std::this_thread::sleep_for(std::min<std::chrono::duration<double>>(left, kStopCheck));
    }
    for (const auto & [signpost, channel] : channels) {
      if (!error) {
        error = channel.recorder->error();
      }
    }
  }

  channels.clear();
  recorder->stop();
  if (error) {
    std::rethrow_exception(error);
  }
}

}  // namespace

auto main(int argc, char ** argv) -> int
{
  // Signals are taken here, not by rclcpp, so the writer is closed before the context is.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { g_stop = true; });
  std::signal(SIGTERM, [](int) { g_stop = true; });
  Options opt;
  try {
    opt = parse(rclcpp::remove_ros_arguments(argc, argv));
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n" << kUsage;
    return 2;
  }

  int rc = 0;
  try {
    run(opt);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
