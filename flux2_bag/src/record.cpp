#include "flux2/subscription.hpp"
#include "flux2/topics.hpp"
#include "flux2/trace.h"
#include "flux2_bag/bag_time.hpp"
#include "flux2_bag/flux_frame_cdr.hpp"
#include "flux2_bag/raw_meta.hpp"
#include "flux2_bag/spin_thread.hpp"
#include "flux2_bridge/tools.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>
#if FLUX2_ROSBAG2_MINOR >= 26
#include <rosbag2_transport/topic_filter.hpp>
#else
#include <regex>
#endif

#include <poll.h>
#include <rcutils/types/uint8_array.h>
#include <rmw/rmw.h>
#include <sys/inotify.h>
#include <unistd.h>

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

constexpr const char * kFrameType = "flux2_msgs/msg/FluxFrame";
constexpr const char * kBagTopicSuffix = "/flux2";
constexpr std::int64_t kTakeWaitNs = 100'000'000;

struct Options
{
  rosbag2_storage::StorageOptions storage;
  rosbag2_transport::RecordOptions record;
  std::chrono::duration<double> poll{1.0};
  std::uint64_t cache_bytes = 1ULL << 30;
};

constexpr const char * kUsage =
  "usage: ros2 bag record_flux2 [-a] [TOPIC ...] [-e REGEX] [-x REGEX]\n"
  "                             [--exclude-topics TOPIC ...] [-o OUT] [--poll SEC]\n"
  "                             [--max-cache-size BYTES]\n"
  "\n"
  "Records the selected flux2 channels and ROS topics into one bag. A flux2 channel NAME\n"
  "is stored as flux2_msgs/FluxFrame on NAME/flux2; ROS topics are stored as they are.\n"
  "flux2 frames wait for the bag in memory, up to BYTES (default 1 GiB); a frame that does\n"
  "not fit is dropped and reported, the publisher never waits.\n";

std::string default_uri()
{
  const std::time_t now = std::time(nullptr);
  std::array<char, 64> buf{};
  std::strftime(buf.data(), buf.size(), "rosbag2_%Y_%m_%d-%H_%M_%S", std::localtime(&now));
  return buf.data();
}

// Lyrical deprecates rmw_serialization_format for output_serialization_format; Jazzy has only
// the former.
template <class RecordOptions>
auto set_output_format(RecordOptions & o, int) -> decltype(o.output_serialization_format, void())
{
  o.output_serialization_format = rmw_get_serialization_format();
}
template <class RecordOptions>
void set_output_format(RecordOptions & o, long)  // NOLINT(runtime/int)
{
  o.rmw_serialization_format = rmw_get_serialization_format();
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
#if FLUX2_ROSBAG2_MINOR >= 26
    if (a == "-a" || a == "--all") {
      o.record.all_topics = true;
    } else if (a == "-e" || a == "--regex") {
      o.record.regex = value();
    } else if (a == "-x" || a == "--exclude-regex") {
      o.record.exclude_regex = value();
    } else if (a == "--exclude-topics") {
      list = &o.record.exclude_topics;
    } else if
#else
    if (a == "-a" || a == "--all") {
      o.record.all = true;
    } else if (a == "-e" || a == "--regex") {
      o.record.regex = value();
    } else if (a == "-x" || a == "--exclude-regex") {
      o.record.exclude = value();
    } else if (a == "--exclude-topics") {
      throw std::invalid_argument("--exclude-topics needs rosbag2 0.26 (Jazzy) or newer");
    } else if
#endif
      (a == "-o" || a == "--output") {
      o.storage.uri = value();
    } else if (a == "--poll") {
      o.poll = std::chrono::duration<double>(flux2_bridge::positive_number("--poll", value()));
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
  if (o.storage.uri.empty()) o.storage.uri = default_uri();
  set_output_format(o.record, 0);
  return o;
}

// `mono_ns` (CLOCK_MONOTONIC) as a time on the clock `now` came from, `now` having been read
// when CLOCK_MONOTONIC was `mono_now_ns`.
std::int64_t on_clock(std::uint64_t mono_ns, std::int64_t mono_now_ns, const rclcpp::Time & now)
{
  return now.nanoseconds() - (mono_now_ns - static_cast<std::int64_t>(mono_ns));
}

std::int64_t monotonic_ns()
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
  WriteQueue & operator=(const WriteQueue &) = delete;
  WriteQueue(WriteQueue &&) = delete;
  WriteQueue & operator=(WriteQueue &&) = delete;
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
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message(std::size_t n)
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
  bool push(
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg, const void * channel,
    std::uint64_t ticket)
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
  std::exception_ptr error() const
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
      if (queue_.empty()) return;
      Item item = std::move(queue_.front());
      queue_.pop_front();
      lock.unlock();
      const std::size_t n = item.msg->serialized_data->buffer_length;
      std::exception_ptr failed;
      try {
        writer_->write(item.msg);
      } catch (...) {
        failed = std::current_exception();
        flux2_trace_record_drop(item.channel, item.ticket, 1, FLUX2_TRACE_RECORD_WRITE_FAILED);
      }
      lock.lock();
      bytes_ -= n;
      free_.push_back(std::move(item.msg));
      if (failed && !error_) error_ = failed;
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

// One flux2 channel into the bag: a thread that takes each frame, serializes it as a FluxFrame
// around the slot bytes into a buffer of its own, releases the borrow, and queues the buffer.
class ChannelRecorder
{
public:
  ChannelRecorder(
    rosbag2_transport::Recorder & recorder, const flux2::Topic & topic,
    rosbag2_cpp::Writer & writer, WriteQueue & queue)
  : recorder_(recorder),
    queue_(queue),
    flux_topic_(topic.key),
    bag_topic_(topic.key + kBagTopicSuffix),
    fingerprint_(topic.fingerprint),
    sub_(recorder, topic.key, topic.fingerprint)
  {
    rosbag2_storage::TopicMetadata meta;
    meta.name = bag_topic_;
    meta.type = kFrameType;
    meta.serialization_format = "cdr";
    writer.create_topic(meta);
    thread_ = std::thread([this] { run(); });
  }

  ChannelRecorder(const ChannelRecorder &) = delete;
  ChannelRecorder & operator=(const ChannelRecorder &) = delete;
  ChannelRecorder(ChannelRecorder &&) = delete;
  ChannelRecorder & operator=(ChannelRecorder &&) = delete;
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
  std::exception_ptr error() const
  {
    const std::lock_guard<std::mutex> lock(error_mutex_);
    return error_;
  }

  // Why the channel refused this recorder later on (a re-attach), or empty: only this channel
  // stops, the bag goes on.
  const std::string & key() const noexcept { return flux_topic_; }

  std::string left_out() const
  {
    const std::lock_guard<std::mutex> lock(error_mutex_);
    return left_out_;
  }

private:
  void run()
  {
    try {
      while (!stop_) record_one();
    } catch (const flux2::Error & e) {
      const std::lock_guard<std::mutex> lock(error_mutex_);
      left_out_ = e.what();
    } catch (...) {
      const std::lock_guard<std::mutex> lock(error_mutex_);
      error_ = std::current_exception();
    }
  }

  void record_one()
  {
    flux2::FrameView v = sub_.take_blocking(kTakeWaitNs);
    if (!v) return;
    const void * channel = v.trace_channel();
    const std::uint64_t ticket = v.ticket();
    // The recorder fell behind its depth and the channel overwrote these before the take.
    if (const std::uint64_t lost = sub_.lost(); lost != reported_lost_) {
      flux2_trace_record_drop(
        channel, 0, static_cast<std::uint32_t>(lost - reported_lost_), FLUX2_TRACE_RECORD_OVERRUN);
      reported_lost_ = lost;
    }
    // As rosbag2 does with a message that arrives while paused: taken, not written.
    if (recorder_.is_paused()) return;
    const rclcpp::Time stamp = recorder_.now();
    const std::int64_t sent = on_clock(v.publish_ns(), monotonic_ns(), stamp);
    // A publisher restarted with other slots is a new attach: replay recreates it the same way.
    if (sub_.attach_generation() != generation_) {
      generation_ = sub_.attach_generation();
      meta_.slot_size = sub_.channel()->slot_size();
      meta_.slot_count = sub_.channel()->slot_count();
    }
    const flux2::FrameMeta & m = v.meta();
    meta_.dtype = static_cast<flux2::DType>(m.dtype);
    meta_.ndim = m.ndim;
    std::memcpy(meta_.shape.data(), m.shape, sizeof(meta_.shape));
    std::array<std::uint8_t, flux2_bag::RawMeta::kMaxSize> meta_bytes{};

    flux2_bag::FluxFrameFields f;
    f.sec = static_cast<std::int32_t>(sent / 1'000'000'000);
    f.nanosec = static_cast<std::uint32_t>(sent % 1'000'000'000);
    f.flux_topic = flux_topic_;
    f.fingerprint = fingerprint_;
    f.codec = "raw";
    f.meta = meta_bytes.data();
    f.meta_size = meta_.write(meta_bytes.data());
    f.data = v.data();
    f.data_size = v.size();

    auto msg = queue_.message(flux2_bag::FluxFrameCdr::size(f));
    msg->topic_name = bag_topic_;
    rcutils_uint8_array_t & buf = *msg->serialized_data;
    buf.buffer_length = flux2_bag::FluxFrameCdr::write(f, buf.buffer);
    v.release();

    // As rosbag2 records a DDS message: received now, sent when its publisher committed it.
    flux2_bag::set_times(*msg, stamp.nanoseconds(), sent);
    if (queue_.push(std::move(msg), channel, ticket)) {
      ++queued_;
    } else {
      flux2_trace_record_drop(channel, ticket, 1, FLUX2_TRACE_RECORD_CACHE_FULL);
      ++dropped_;
    }
  }

  rosbag2_transport::Recorder & recorder_;
  WriteQueue & queue_;
  std::string flux_topic_;
  std::string bag_topic_;
  std::uint64_t fingerprint_;
  flux2::Subscription sub_;
  std::uint32_t generation_ = 0;
  flux2_bag::RawMeta meta_;
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
// A publisher's last change to /dev/shm comes before it stores the epoch and key that make its
// channel recordable, so the channels are listed this often until changes stop for kSettle.
constexpr auto kRecheck = std::chrono::milliseconds(10);
constexpr auto kSettle = std::chrono::milliseconds(500);
constexpr auto kStopCheck = std::chrono::milliseconds(100);

struct Recorded
{
  std::unique_ptr<ChannelRecorder> recorder;
  // Set by the first listing that found the channel down, cleared by one that finds it up.
  std::optional<std::chrono::steady_clock::time_point> down_since;
};

// Flux2 signposts and segments created, sized or removed in /dev/shm.
class ShmWatch
{
public:
  ShmWatch() : fd_(inotify_init1(IN_NONBLOCK | IN_CLOEXEC))
  {
    if (fd_ < 0) throw std::system_error(errno, std::generic_category(), "inotify_init1");
    if (inotify_add_watch(fd_, "/dev/shm", IN_CREATE | IN_MOVED_TO | IN_MODIFY | IN_DELETE) < 0) {
      const int e = errno;
      ::close(fd_);
      throw std::system_error(e, std::generic_category(), "inotify_add_watch /dev/shm");
    }
  }
  ShmWatch(const ShmWatch &) = delete;
  ShmWatch & operator=(const ShmWatch &) = delete;
  ShmWatch(ShmWatch &&) = delete;
  ShmWatch & operator=(ShmWatch &&) = delete;
  ~ShmWatch() { ::close(fd_); }

  // Waits up to `timeout` for a change; reads every queued event, flux2 or not.
  bool changed(std::chrono::milliseconds timeout) const
  {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, static_cast<int>(timeout.count())) <= 0) return false;
    alignas(inotify_event) std::array<char, 4096> buf{};
    bool flux2 = false;
    for (ssize_t n = 0; (n = ::read(fd_, buf.data(), buf.size())) > 0;) {
      for (ssize_t at = 0; at < n;) {
        // NOLINTNEXTLINE(*-reinterpret-cast)
        const auto * e = reinterpret_cast<const inotify_event *>(buf.data() + at);
        flux2 = flux2 || (e->len > 0 && std::strncmp(e->name, "flux2.", 6) == 0);
        at += static_cast<ssize_t>(sizeof(inotify_event) + e->len);
      }
    }
    return flux2;
  }

private:
  int fd_;
};

#if FLUX2_ROSBAG2_MINOR < 26
// rosbag2 0.15's TopicFilter, which it does not install.
bool selected(const rosbag2_transport::RecordOptions & o, const std::string & key)
{
  if (!o.topics.empty() && std::find(o.topics.begin(), o.topics.end(), key) == o.topics.end()) {
    return false;
  }
  if (!o.exclude.empty() && std::regex_search(key, std::regex(o.exclude))) return false;
  if (!o.all && !o.regex.empty() && !std::regex_search(key, std::regex(o.regex))) return false;
  return o.include_hidden_topics || key.find("/_") == std::string::npos;
}
#endif

void run(Options & opt)
{
  auto writer = std::make_shared<rosbag2_cpp::Writer>();
  auto recorder = std::make_shared<rosbag2_transport::Recorder>(
    writer, opt.storage, opt.record, "flux2_bag_record");
  recorder->record();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(recorder);
  const flux2_bag::SpinThread spin(exec);

#if FLUX2_ROSBAG2_MINOR >= 26
  rosbag2_transport::TopicFilter filter(opt.record, nullptr, /*allow_unknown_types=*/true);
#endif
  auto queue = std::make_unique<WriteQueue>(writer, opt.cache_bytes);
  // By signpost: one name can carry channels of two fingerprints, each recorded on its own.
  std::map<std::string, Recorded> channels;
  std::set<std::string> bad_keys;
  // Signposts of channels that refused a recorder, with the publisher epoch that refused: tried
  // again once the publisher restarts.
  std::map<std::string, std::uint32_t> left_out;
  const auto leave_out =
    [&](const std::string & key, const std::string & signpost, const std::string & why) {
      left_out[signpost] = flux2::read_channel_stats(signpost).epoch;
      RCLCPP_ERROR(
        recorder->get_logger(), "flux2 channel %s not recorded until its publisher restarts: %s",
        key.c_str(), why.c_str());
    };
  const ShmWatch watch;
  std::chrono::steady_clock::time_point settle_until;
  std::exception_ptr error;
  while (!g_stop && !error) {
    const auto listed = std::chrono::steady_clock::now();
    for (auto & [signpost, channel] : channels) {
      if (!channel.down_since) channel.down_since = listed;
    }
    std::map<std::string, std::vector<std::string>> candidates;
    std::map<std::string, std::vector<const flux2::Topic *>> live;
    std::set<std::string> bad;
    std::set<std::string> seen;
    const std::vector<flux2::Topic> topics = flux2::enumerate_topics();
    for (const flux2::Topic & t : topics) {
      seen.insert(t.signpost);
      if (!t.key_exact) continue;
      const flux2::ChannelStats stats = flux2::read_channel_stats(t.signpost);
      if (!stats.live) continue;
      if (const auto out = left_out.find(t.signpost); out != left_out.end()) {
        if (out->second == stats.epoch) continue;
        left_out.erase(out);
      }
      if (!flux2_bridge::is_ros_topic_name(t.key)) {
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
      if (!why.empty()) leave_out(it->second.recorder->key(), it->first, why);
      it = gone || !why.empty() ? channels.erase(it) : std::next(it);
    }
#if FLUX2_ROSBAG2_MINOR >= 26
    for (const auto & [key, type] : filter.filter_topics(candidates)) {
#else
    for (const auto & [key, type] : candidates) {
      if (!selected(opt.record, key)) continue;
#endif
      for (const flux2::Topic * t : live[key]) {
        std::unique_ptr<ChannelRecorder> r;
        try {
          r = std::make_unique<ChannelRecorder>(*recorder, *t, *writer, *queue);
        } catch (const flux2::Error & e) {
          leave_out(key, t->signpost, e.what());
          continue;
        }
        RCLCPP_INFO(recorder->get_logger(), "recording flux2 channel %s", key.c_str());
        channels.emplace(t->signpost, Recorded{std::move(r), std::nullopt});
      }
    }
    const std::chrono::duration<double> period =
      listed < settle_until ? std::chrono::duration<double>(kRecheck) : opt.poll;
    while (!g_stop) {
      const std::chrono::duration<double> left =
        period - (std::chrono::steady_clock::now() - listed);
      if (left <= left.zero()) break;
      const auto wait = std::chrono::ceil<std::chrono::milliseconds>(
        std::min<std::chrono::duration<double>>(left, kStopCheck));
      if (watch.changed(wait)) {
        settle_until = std::chrono::steady_clock::now() + kSettle;
        break;
      }
    }
    for (const auto & [signpost, channel] : channels) {
      if (!error) error = channel.recorder->error();
    }
    if (!error) error = queue->error();
  }

  channels.clear();
  queue.reset();
  recorder->stop();
  if (error) std::rethrow_exception(error);
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
