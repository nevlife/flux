#include "flux/subscription.hpp"
#include "flux/topics.hpp"
#include "flux/trace.h"
#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/jpeg_frame.hpp"
#include "flux_bag/raw_meta.hpp"
#include "flux_bag/spin_thread.hpp"
#include "flux_bridge/tools.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <rosbag2_transport/topic_filter.hpp>

#include "sensor_msgs/flux/image.hpp"

#include <rcutils/types/uint8_array.h>
#include <rmw/rmw.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
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
  std::chrono::duration<double> poll{0.1};
  std::uint64_t cache_bytes = 1ULL << 30;
  // 0 records images as they are.
  int jpeg_quality = 0;
};

constexpr const char * kUsage =
  "usage: ros2 bag record_flux [-a] [TOPIC ...] [-e REGEX] [-x REGEX]\n"
  "                             [--exclude-topics TOPIC ...] [-o OUT] [--poll SEC]\n"
  "                             [--max-cache-size BYTES] [-s STORAGE]\n"
  "                             [--storage-preset-profile PROFILE]\n"
  "                             [--image-codec raw|jpeg] [--jpeg-quality Q]\n"
  "\n"
  "Records the selected flux channels and ROS topics into one bag. A flux channel NAME\n"
  "is stored as flux_msgs/FluxFrame on NAME/flux; ROS topics are stored as they are.\n"
  "flux frames wait for the bag in memory, up to BYTES (default 1 GiB); a frame that does\n"
  "not fit is dropped and reported, the publisher never waits.\n"
  "-s and --storage-preset-profile go to rosbag2 as in ros2 bag record (zstd_fast\n"
  "compresses an mcap bag). --image-codec jpeg stores the pixels of sensor_msgs/Image\n"
  "frames in rgb8, bgr8 or mono8 as JPEG of quality Q (1-100, default 90); other frames\n"
  "are stored as they are.\n";

auto default_uri() -> std::string
{
  const std::time_t now = std::time(nullptr);
  std::array<char, 64> buf{};
  std::strftime(buf.data(), buf.size(), "rosbag2_%Y_%m_%d-%H_%M_%S", std::localtime(&now));
  return buf.data();
}

auto parse(const std::vector<std::string> & args) -> Options
{
  Options o;
  std::vector<std::string> * list = &o.record.topics;
  std::string image_codec = "raw";
  int quality = 90;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    auto value = [&]() -> const std::string & {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument(a + " needs a value");
      }
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
    } else if (a == "-s" || a == "--storage") {
      o.storage.storage_id = value();
    } else if (a == "--storage-preset-profile") {
      o.storage.storage_preset_profile = value();
    } else if (a == "--image-codec") {
      const std::string & v = value();
      if (v != "raw" && v != "jpeg") {
        throw std::invalid_argument("--image-codec takes raw or jpeg");
      }
      image_codec = v;
    } else if (a == "--jpeg-quality") {
      const std::string & v = value();
      if (
        v.empty() || v.size() > 3 || v.find_first_not_of("0123456789") != std::string::npos ||
        std::stoi(v) < 1 || std::stoi(v) > 100) {
        throw std::invalid_argument("--jpeg-quality takes a number from 1 to 100");
      }
      quality = std::stoi(v);
    } else if (a == "--poll") {
      o.poll = std::chrono::duration<double>(flux_bridge::positive_number("--poll", value()));
    } else if (a == "--max-cache-size") {
      const std::string & v = value();
      if (v.empty() || v.find_first_not_of("0123456789") != std::string::npos) {
        throw std::invalid_argument("--max-cache-size takes a positive number of bytes");
      }
      o.cache_bytes = std::stoull(v);
      if (o.cache_bytes == 0) {
        throw std::invalid_argument("--max-cache-size takes a positive number of bytes");
      }
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
    } else {
      list->push_back(a);
    }
  }
  if (o.storage.uri.empty()) {
    o.storage.uri = default_uri();
  }
  if (image_codec == "jpeg") {
    o.jpeg_quality = quality;
  }
  o.record.rmw_serialization_format = rmw_get_serialization_format();
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

// Frames between the channel threads, which copy each frame out and release its slot at once, and
// the one thread that writes the bag: a burst waits here instead of being overwritten in the
// channel.
class WriteQueue
{
public:
  WriteQueue(std::shared_ptr<rosbag2_cpp::Writer> writer, std::uint64_t budget)
  : writer_(std::move(writer)), budget_(budget), thread_([this] { run(); })
  {
  }
  WriteQueue(const WriteQueue &) = delete;
  auto operator=(const WriteQueue &) -> WriteQueue & = delete;
  WriteQueue(WriteQueue &&) = delete;
  auto operator=(WriteQueue &&) -> WriteQueue & = delete;
  // Writes what is queued, then ends.
  ~WriteQueue()
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    ready_.notify_one();
    thread_.join();
  }

  // A message for `n` bytes: one the writer is done with, so its pages are already mapped (a
  // fresh frame-sized buffer costs a page fault per page, slower than the frames come).
  auto message(std::size_t n) -> std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  {
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (!free_.empty()) {
        msg = std::move(free_.back());
        free_.pop_back();
      }
    }
    if (!msg) {
      msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      msg->serialized_data = rosbag2_storage::make_empty_serialized_message(n);
    } else if (msg->serialized_data->buffer_capacity < n) {
      if (rcutils_uint8_array_resize(msg->serialized_data.get(), n) != RCUTILS_RET_OK) {
        throw std::bad_alloc();
      }
    }
    return msg;
  }

  // False when the frame does not fit in the budget.
  auto push(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg, const void * channel,
    std::uint64_t ticket) -> bool
  {
    const std::size_t n = msg->serialized_data->buffer_length;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (bytes_ + n > budget_) {
        free_.push_back(std::move(msg));
        return false;
      }
      bytes_ += n;
      queue_.push_back({std::move(msg), channel, ticket});
    }
    ready_.notify_one();
    return true;
  }

  // The first write that failed; the frames after it are reported as they fail too.
  auto error() const -> std::exception_ptr
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

private:
  struct Item
  {
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
    const void * channel;
    std::uint64_t ticket;
  };

  void run()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      ready_.wait(lock, [this] { return stop_ || !queue_.empty(); });
      if (queue_.empty()) {
        return;
      }
      Item item = std::move(queue_.front());
      queue_.pop_front();
      lock.unlock();
      const std::size_t n = item.msg->serialized_data->buffer_length;
      std::exception_ptr failed;
      try {
        writer_->write(item.msg);
      } catch (...) {
        failed = std::current_exception();
        flux_trace_record_drop(item.channel, item.ticket, 1, FLUX_TRACE_RECORD_WRITE_FAILED);
      }
      lock.lock();
      bytes_ -= n;
      free_.push_back(std::move(item.msg));
      if (failed && !error_) {
        error_ = failed;
      }
    }
  }

  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  const std::uint64_t budget_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Item> queue_;
  // As many as were queued at once, so no more than the budget.
  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> free_;
  std::uint64_t bytes_ = 0;
  bool stop_ = false;
  std::exception_ptr error_;
  std::thread thread_;
};

// One flux channel into the bag: a thread that takes each frame, serializes it as a FluxFrame
// around the slot bytes into a buffer of its own, releases the borrow, and queues the buffer.
class ChannelRecorder
{
public:
  ChannelRecorder(
    rosbag2_transport::Recorder & recorder, const flux::Topic & topic, rosbag2_cpp::Writer & writer,
    WriteQueue & queue, int jpeg_quality)
  : recorder_(recorder),
    queue_(queue),
    flux_topic_(topic.key),
    bag_topic_(topic.key + kBagTopicSuffix),
    fingerprint_(topic.fingerprint),
    sub_(recorder, topic.key, topic.fingerprint),
    meta_bytes_(flux_bag::JpegMeta::kMaxSize + flux_bag::AuxTail::kMaxSize)
  {
    if (jpeg_quality > 0 && fingerprint_ == sensor_msgs::flux_msg::Image::kFingerprint) {
      jpeg_ = std::make_unique<flux_bag::JpegEncoder>(jpeg_quality);
    }
    rosbag2_storage::TopicMetadata meta;
    meta.name = bag_topic_;
    meta.type = kFrameType;
    meta.serialization_format = "cdr";
    writer.create_topic(meta);
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
    RCLCPP_INFO(
      recorder_.get_logger(), "%s: %lu frames queued, %lu dropped with the cache full, %lu lost",
      flux_topic_.c_str(), static_cast<unsigned long>(queued_),
      static_cast<unsigned long>(dropped_), static_cast<unsigned long>(sub_.lost()));
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
        record_one();
      }
    } catch (const flux::Error & e) {
      const std::lock_guard<std::mutex> lock(error_mutex_);
      left_out_ = e.what();
    } catch (...) {
      const std::lock_guard<std::mutex> lock(error_mutex_);
      error_ = std::current_exception();
    }
  }

  void record_one()
  {
    flux::FrameView v = sub_.take_blocking(kTakeWaitNs);
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
    const std::int64_t sent = on_clock(v.publish_ns(), monotonic_ns(), stamp);
    // A publisher restarted with other slots is a new attach: replay recreates it the same way.
    if (sub_.attach_generation() != generation_) {
      generation_ = sub_.attach_generation();
      meta_.slot_size = sub_.channel()->slot_size();
      meta_.slot_count = sub_.channel()->slot_count();
      aux_size_ = sub_.channel()->aux_size();
    }
    const flux::FrameMeta & m = v.meta();
    meta_.dtype = static_cast<flux::DType>(m.dtype);
    meta_.ndim = m.ndim;
    std::memcpy(meta_.shape.data(), m.shape, sizeof(meta_.shape));

    flux_bag::FluxFrameFields f;
    f.sec = static_cast<std::int32_t>(sent / 1'000'000'000);
    f.nanosec = static_cast<std::uint32_t>(sent % 1'000'000'000);
    f.flux_topic = flux_topic_;
    f.fingerprint = fingerprint_;
    f.codec = "raw";
    f.meta = meta_bytes_.data();
    f.meta_size = meta_.write(meta_bytes_.data());
    f.data = v.data();
    f.data_size = v.size();
    if (jpeg_) {
      compress(v, f, meta_bytes_.data());
    }
    if (aux_size_ != 0) {
      const auto aux = v.aux();
      flux_bag::AuxTail tail;
      tail.aux_size = aux_size_;
      tail.aux = aux.data();
      tail.aux_len = static_cast<std::uint32_t>(aux.size());
      f.meta_size += tail.write(meta_bytes_.data() + f.meta_size);
    }

    auto msg = queue_.message(flux_bag::FluxFrameCdr::size(f));
    msg->topic_name = bag_topic_;
    rcutils_uint8_array_t & buf = *msg->serialized_data;
    buf.buffer_length = flux_bag::FluxFrameCdr::write(f, buf.buffer);
    v.release();

    // As rosbag2 records a DDS message: received now, sent when its publisher committed it.
    msg->recv_timestamp = stamp.nanoseconds();
    msg->send_timestamp = sent;
    if (queue_.push(std::move(msg), channel, ticket)) {
      ++queued_;
    } else {
      flux_trace_record_drop(channel, ticket, 1, FLUX_TRACE_RECORD_CACHE_FULL);
      ++dropped_;
    }
  }

  // An Image frame JPEG holds becomes codec "jpeg" in `f`, its data in packed_; any other frame
  // stays raw.
  void compress(const flux::FrameView & v, flux_bag::FluxFrameFields & f, std::uint8_t * meta)
  {
    const sensor_msgs::flux_msg::Image::View img(v);
    const std::uint32_t width = img.width();
    const std::uint32_t height = img.height();
    const std::uint32_t step = img.step();
    const J_COLOR_SPACE space = flux_bag::jpeg_space(img.encoding());
    const auto pixels = img.data();
    if (
      !img.ok__() || space == JCS_UNKNOWN ||
      std::uint64_t{width} * flux_bag::jpeg_channels(space) > step ||
      pixels.size() != std::uint64_t{step} * height) {
      return;
    }
    const std::size_t n = jpeg_->encode(pixels.data(), width, height, step, space);
    if (n == 0) {
      return;
    }
    const auto * frame = static_cast<const std::uint8_t *>(v.data());
    flux_bag::JpegMeta jm;
    jm.raw = meta_;
    jm.at = static_cast<std::uint64_t>(pixels.data() - frame);
    jm.len = pixels.size();
    const std::size_t rest = v.size() - jm.at - jm.len;
    packed_.resize(jm.at + rest + n);
    std::memcpy(packed_.data(), frame, jm.at);
    std::memcpy(packed_.data() + jm.at, frame + jm.at + jm.len, rest);
    std::memcpy(packed_.data() + jm.at + rest, jpeg_->data(), n);
    f.codec = "jpeg";
    f.meta_size = jm.write(meta);
    f.data = packed_.data();
    f.data_size = packed_.size();
  }

  rosbag2_transport::Recorder & recorder_;
  WriteQueue & queue_;
  std::string flux_topic_;
  std::string bag_topic_;
  std::uint64_t fingerprint_;
  flux::Subscription sub_;
  std::uint32_t generation_ = 0;
  flux_bag::RawMeta meta_;
  std::uint32_t aux_size_ = 0;
  // The codec's meta, then the aux tail of a channel made with an aux_size.
  std::vector<std::uint8_t> meta_bytes_;
  std::unique_ptr<flux_bag::JpegEncoder> jpeg_;
  std::vector<std::uint8_t> packed_;
  std::uint64_t queued_ = 0;
  std::uint64_t dropped_ = 0;
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
  auto writer = std::make_shared<rosbag2_cpp::Writer>();
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    writer, opt.storage, opt.record, "flux_bag_record");
  recorder->record();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(recorder);
  const flux_bag::SpinThread spin(exec);

  rosbag2_transport::TopicFilter filter(opt.record, nullptr, /*allow_unknown_types=*/true);
  auto queue = std::make_unique<WriteQueue>(writer, opt.cache_bytes);
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
  while (!g_stop && !error) {
    const auto listed = std::chrono::steady_clock::now();
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
            recorder->get_logger(), "a channel whose key is not a ROS topic name is not recorded");
        }
        bad.insert(t.signpost);
        continue;
      }
      if (const auto it = channels.find(t.signpost); it != channels.end()) {
        it->second.down_since.reset();
        continue;
      }
      candidates[t.key] = {kFrameType};
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
          r = std::make_unique<ChannelRecorder>(*recorder, *t, *writer, *queue, opt.jpeg_quality);
        } catch (const flux::Error & e) {
          leave_out(key, t->signpost, e.what());
          continue;
        }
        RCLCPP_INFO(recorder->get_logger(), "recording flux channel %s", key.c_str());
        channels.emplace(t->signpost, Recorded{std::move(r), std::nullopt});
      }
    }
    while (!g_stop) {
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
    if (!error) {
      error = queue->error();
    }
  }

  channels.clear();
  queue.reset();
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
