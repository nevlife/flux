#include "flux2/publisher.hpp"
#include "flux2_bag/bag_time.hpp"
#include "flux2_bag/flux_frame_cdr.hpp"
#include "flux2_bag/raw_meta.hpp"
#include "flux2_bag/spin_thread.hpp"
#include "flux2_bridge/tools.hpp"

#include <keyboard_handler/keyboard_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/clocks/time_controller_clock.hpp>
#include <rosbag2_cpp/info.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/play_options.hpp>
#include <rosbag2_transport/player.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace
{

constexpr const char * kFrameType = "flux2_msgs/msg/FluxFrame";

struct Options
{
  rosbag2_storage::StorageOptions storage;
  rosbag2_transport::PlayOptions play;
};

constexpr const char * kUsage =
  "usage: ros2 bag play_flux2 BAG [-r RATE]\n"
  "\n"
  "Plays a bag. flux2_msgs/FluxFrame topics go straight from the file back into the flux2\n"
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
      o.play.rate = static_cast<float>(flux2_bridge::positive_number("-r", value()));
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
    } else if (o.storage.uri.empty()) {
      o.storage.uri = a;
    } else {
      throw std::invalid_argument("one bag only");
    }
  }
  if (o.storage.uri.empty()) throw std::invalid_argument("BAG is required");
  if (!(o.play.rate > 0)) throw std::invalid_argument("argument -r: Value must be positive");
  return o;
}

bool decode(
  const rosbag2_storage::SerializedBagMessage & msg, flux2_bag::FluxFrameFields & f,
  flux2_bag::RawMeta & meta)
{
  const rcutils_uint8_array_t & raw = *msg.serialized_data;
  return flux2_bag::FluxFrameCdr::read(raw.buffer, raw.buffer_length, f) && f.codec == "raw" &&
         meta.read(f.meta, f.meta_size);
}

// A waiting subscriber looks for its channel once per tick.
constexpr auto kAttachWait = std::chrono::nanoseconds(2 * FLUX2_DEFAULT_TICK_NS);

// In bytes, not messages: a frame runs to several MiB.
constexpr std::size_t kReadAheadBytes = std::size_t{256} << 20;

using MessagePtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;

// The FluxFrame topics of one bag, published on flux2 at the bag's own pace. A loader thread reads
// the frames ahead of a publish thread, which sends each one when its own clock reaches the
// frame's receive time. A publisher per channel and geometry the frames carry: a channel recorded
// across a restart with other slots is restarted the same way here.
class FluxPlayer
{
public:
  FluxPlayer(
    rclcpp::Node & node, const rosbag2_storage::StorageOptions & storage,
    const std::vector<std::string> & topics, rcutils_time_point_value_t start, double rate)
  : node_(node), clock_(start, std::chrono::steady_clock::now, std::chrono::milliseconds{100}, true)
  {
    clock_.set_rate(rate);
    // Up before playback, as the stock player creates its publishers, so a subscriber already
    // waiting is attached when the first frame is due.
    for (const std::string & topic : topics) {
      rosbag2_cpp::Reader first;
      first.open(storage);
      rosbag2_storage::StorageFilter only;
      only.topics = {topic};
      first.set_filter(only);
      if (!first.has_next()) continue;
      const auto msg = first.read_next();
      flux2_bag::FluxFrameFields f;
      flux2_bag::RawMeta meta;
      if (decode(*msg, f, meta)) publisher_for(f, meta);
    }
    if (!pubs_.empty()) std::this_thread::sleep_for(kAttachWait);
    reader_.open(storage);
    rosbag2_storage::StorageFilter filter;
    filter.topics = topics;
    reader_.set_filter(filter);
    loader_ = std::thread([this] { load(); });
  }

  FluxPlayer(const FluxPlayer &) = delete;
  FluxPlayer & operator=(const FluxPlayer &) = delete;
  FluxPlayer(FluxPlayer &&) = delete;
  FluxPlayer & operator=(FluxPlayer &&) = delete;
  ~FluxPlayer() { stop(); }

  // The clock starts paused at the bag's start; resume() sets it going.
  void start()
  {
    sender_ = std::thread([this] { play(); });
  }

  void stop()
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    wake_.notify_all();
    room_.notify_all();
    if (loader_.joinable()) loader_.join();
    if (sender_.joinable()) sender_.join();
  }

  bool finished() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return finished_;
  }

  // What ended a thread early, rethrown on the main thread.
  std::exception_ptr error() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

  void pause()
  {
    control([this] { clock_.pause(); });
  }
  void resume()
  {
    control([this] { clock_.resume(); });
  }
  void set_rate(double rate)
  {
    control([this, rate] { clock_.set_rate(rate); });
  }
  void jump(rcutils_time_point_value_t t)
  {
    control([this, t] { clock_.jump(t); });
  }

  // While paused, sends the next frame not yet due and moves the clock to it. False when there is
  // none.
  bool play_next()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!clock_.is_paused() || finished_) return false;
    const std::uint64_t before = steps_;
    step_ = true;
    wake_.notify_all();
    wake_.wait(lock, [&] { return steps_ != before || finished_ || stop_; });
    return steps_ != before;
  }

private:
  using Key = std::tuple<std::string, std::uint64_t>;
  struct Channel
  {
    std::uint32_t slot_size = 0;
    std::uint32_t slot_count = 0;
    std::unique_ptr<flux2::Publisher> pub;
    // A publisher it could not get: as rosbag2's player does, the channel is skipped from then on.
    bool ignored = false;
  };

  // Under mutex_, so the publish thread, which reads the clock under it, never misses a change.
  template <typename Change>
  void control(Change change)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    change();
    wake_.notify_all();
  }

  void fail(std::exception_ptr e)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!error_) error_ = std::move(e);
    wake_.notify_all();
  }

  void load()
  {
    try {
      while (reader_.has_next()) {
        MessagePtr msg = reader_.read_next();
        std::unique_lock<std::mutex> lock(mutex_);
        room_.wait(lock, [this] { return stop_ || queued_bytes_ < kReadAheadBytes; });
        if (stop_) return;
        queued_bytes_ += msg->serialized_data->buffer_length;
        queue_.push_back(std::move(msg));
        wake_.notify_all();
      }
    } catch (...) {
      fail(std::current_exception());
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    loaded_ = true;
    wake_.notify_all();
  }

  void play()
  {
    try {
      bool stepped = false;
      while (const MessagePtr msg = next_due(stepped)) {
        publish(*msg);
        if (stepped) {
          const std::lock_guard<std::mutex> lock(mutex_);
          ++steps_;
          wake_.notify_all();
        }
      }
    } catch (...) {
      fail(std::current_exception());
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    finished_ = true;
    wake_.notify_all();
  }

  // The next frame once the clock reaches it or play_next asks for it; null at the end of the
  // bag, on stop and after an error.
  MessagePtr next_due(bool & stepped)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      if (stop_ || error_ || (queue_.empty() && loaded_)) return nullptr;
      if (queue_.empty()) {
        wake_.wait(lock);
        continue;
      }
      const rcutils_time_point_value_t due = flux2_bag::received(*queue_.front());
      const rcutils_time_point_value_t now = clock_.now();
      stepped = step_ && now < due;
      if (stepped) {
        step_ = false;
        clock_.jump(due);
      }
      if (stepped || now >= due) {
        MessagePtr msg = std::move(queue_.front());
        queue_.pop_front();
        queued_bytes_ -= msg->serialized_data->buffer_length;
        room_.notify_one();
        return msg;
      }
      if (clock_.is_paused()) {
        wake_.wait(lock);
      } else {
        // In seconds: at a tiny rate the wait is past what int64 nanoseconds hold.
        const double left = static_cast<double>(due - now) / 1e9 / clock_.get_rate();
        wake_.wait_for(lock, std::chrono::duration<double>(std::min(left, 1.0)));
      }
    }
  }

  flux2::Publisher * publisher_for(
    const flux2_bag::FluxFrameFields & f, const flux2_bag::RawMeta & meta)
  {
    Channel & c = pubs_[Key(f.flux_topic, f.fingerprint)];
    if (c.ignored) return nullptr;
    if (c.pub && c.slot_size == meta.slot_size && c.slot_count == meta.slot_count)
      return c.pub.get();
    c.pub.reset();  // one publisher per channel: the old one goes first
    try {
      c.pub = std::make_unique<flux2::Publisher>(
        node_, std::string(f.flux_topic), f.fingerprint, meta.slot_size, meta.slot_count);
    } catch (const flux2::Error & e) {
      c.ignored = true;
      RCLCPP_ERROR(
        node_.get_logger(), "Ignoring a flux2 channel '%s', reason: %s.",
        std::string(f.flux_topic).c_str(), e.what());
      return nullptr;
    }
    c.slot_size = meta.slot_size;
    c.slot_count = meta.slot_count;
    RCLCPP_INFO(
      node_.get_logger(), "flux2 channel %s: %u slots of %u bytes",
      std::string(f.flux_topic).c_str(), meta.slot_count, meta.slot_size);
    return c.pub.get();
  }

  void publish(const rosbag2_storage::SerializedBagMessage & msg)
  {
    flux2_bag::FluxFrameFields f;
    flux2_bag::RawMeta meta;
    if (!decode(msg, f, meta)) {
      RCLCPP_ERROR_ONCE(
        node_.get_logger(), "%s: not a raw FluxFrame, skipped", msg.topic_name.c_str());
      return;
    }
    if (meta.nbytes() != f.data_size) {
      RCLCPP_ERROR_ONCE(
        node_.get_logger(), "%s: a frame's data does not match its dtype and shape, skipped",
        msg.topic_name.c_str());
      return;
    }
    flux2::Publisher * pub = publisher_for(f, meta);
    if (pub == nullptr) return;
    flux2::WriteSlot w = pub->loan(meta.dtype, meta.shape.data(), meta.ndim);
    if (!w) return;
    std::memcpy(w.data(), f.data, f.data_size);
    if (const flux2::Published p = w.commit(); flux2::faulted(p)) {
      RCLCPP_ERROR_ONCE(node_.get_logger(), "publish refused: %s", flux2::to_string(p));
    }
  }

  rclcpp::Node & node_;
  rosbag2_cpp::TimeControllerClock clock_;
  rosbag2_cpp::Reader reader_;
  std::map<Key, Channel> pubs_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable room_;
  std::deque<MessagePtr> queue_;
  std::size_t queued_bytes_ = 0;
  bool loaded_ = false;
  bool stop_ = false;
  bool step_ = false;
  std::uint64_t steps_ = 0;
  bool finished_ = false;
  std::exception_ptr error_;
  std::thread loader_;
  std::thread sender_;
};

#if FLUX2_ROSBAG2_MINOR < 26
// Where rosbag2 0.15's player starts without a start offset; it keeps its own private.
rcutils_time_point_value_t bag_start(const rosbag2_storage::StorageOptions & s)
{
  const rosbag2_storage::BagMetadata m = rosbag2_cpp::Info().read_metadata(s.uri, s.storage_id);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(m.starting_time.time_since_epoch())
    .count();
}
#endif

// The stock player for the ROS topics, its pause, resume and rate also driving the flux2 clock:
// its services and keys call these through the base class. seek is not virtual, so it moves the
// ROS topics only. play_next and burst step the ROS topics, the flux2 channels following to the
// last message played; once no ROS message is left they step the flux2 frames. rosbag2 0.15 tells
// nothing of the message played, so there the flux2 channels stay put while ROS topics step.
class BagPlayer : public rosbag2_transport::Player
{
public:
  BagPlayer(const Options & o, const std::vector<std::string> & flux_topics)
  : rosbag2_transport::Player(o.storage, o.play, "flux2_bag_play"),
    options_(o.play),
#if FLUX2_ROSBAG2_MINOR >= 26
    flux_(*this, o.storage, flux_topics, get_starting_time(), o.play.rate)
  {
    add_on_play_message_post_callback(
      [this](const MessagePtr & m) { played_ = flux2_bag::received(*m); });
  }
#else
    flux_(*this, o.storage, flux_topics, bag_start(o.storage), o.play.rate)
  {
  }

  // rosbag2 0.15's play() returns at the end of the bag, so it runs on a thread of its own.
  void play()
  {
    playing_ = std::async(std::launch::async, [this] { rosbag2_transport::Player::play(); });
  }

  // Started paused, play() keeps the queue ready until resumed.
  void wait_for_playback_to_start()
  {
    std::unique_lock<std::mutex> lock(ready_to_play_from_queue_mutex_);
    ready_to_play_from_queue_cv_.wait(lock, [this] { return is_ready_to_play_from_queue_; });
  }

  bool wait_for_playback_to_finish(std::chrono::milliseconds timeout)
  {
    return playing_.wait_for(timeout) == std::future_status::ready;
  }

  // rosbag2 0.15's play loop ends only with the context.
  void stop()
  {
    rclcpp::shutdown();
    playing_.wait();
  }
#endif

  // Both clocks set going from the bag's start at once. The keys come last: rosbag2's own would
  // call into this object before it is built.
  void start()
  {
    resume();
    flux_.start();
    const rosbag2_transport::PlayOptions & o = options_;
    keyboard_.add_key_press_callback(
      [this](auto, auto) { toggle_paused(); }, o.pause_resume_toggle_key);
    keyboard_.add_key_press_callback([this](auto, auto) { play_next(); }, o.play_next_key);
    keyboard_.add_key_press_callback(
      [this](auto, auto) { set_rate(get_rate() + 0.1); }, o.increase_rate_key);
    keyboard_.add_key_press_callback(
      [this](auto, auto) { set_rate(get_rate() - 0.1); }, o.decrease_rate_key);
  }

  FluxPlayer & flux() { return flux_; }

  void pause() override
  {
    rosbag2_transport::Player::pause();
    flux_.pause();
  }

  void resume() override
  {
    rosbag2_transport::Player::resume();
    flux_.resume();
  }

  bool set_rate(double rate) override
  {
    if (!rosbag2_transport::Player::set_rate(rate)) return false;
    flux_.set_rate(rate);
    return true;
  }

  bool play_next() override
  {
    if (!rosbag2_transport::Player::play_next()) return flux_.play_next();
#if FLUX2_ROSBAG2_MINOR >= 26
    flux_.jump(played_);
#endif
    return true;
  }

  std::size_t burst(const std::size_t num_messages) override
  {
    std::size_t played = rosbag2_transport::Player::burst(num_messages);
#if FLUX2_ROSBAG2_MINOR >= 26
    if (played > 0) flux_.jump(played_);
#endif
    while ((num_messages == 0 || played < num_messages) && flux_.play_next()) ++played;
    return played;
  }

private:
  rosbag2_transport::PlayOptions options_;
  FluxPlayer flux_;
  KeyboardHandler keyboard_;
#if FLUX2_ROSBAG2_MINOR >= 26
  std::atomic<rcutils_time_point_value_t> played_{0};
#else
  std::future<void> playing_;
#endif
};

// Global: a signal handler reaches nothing else.
std::atomic<bool> g_stop{false};  // NOLINT(*-avoid-non-const-global-variables)

void run(Options & opt)
{
  const rosbag2_storage::BagMetadata metadata =
    rosbag2_cpp::Info().read_metadata(opt.storage.uri, opt.storage.storage_id);
  std::vector<std::string> flux_topics;
  for (const auto & t : metadata.topics_with_message_count) {
    if (t.topic_metadata.type == kFrameType) flux_topics.push_back(t.topic_metadata.name);
  }
#if FLUX2_ROSBAG2_MINOR >= 26
  opt.play.exclude_topics_to_filter = flux_topics;
#else
  // rosbag2 0.15 filters by inclusion only: the ROS topics, or with none a name no topic has.
  for (const auto & t : metadata.topics_with_message_count) {
    if (t.topic_metadata.type != kFrameType) {
      opt.play.topics_to_filter.push_back(t.topic_metadata.name);
    }
  }
  if (opt.play.topics_to_filter.empty()) opt.play.topics_to_filter = {""};
#endif
  opt.play.disable_keyboard_controls = true;  // BagPlayer::start adds them
  opt.play.start_paused = true;               // until BagPlayer::start resumes both clocks

  auto player = std::make_shared<BagPlayer>(opt, flux_topics);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(player);
  const flux2_bag::SpinThread spin(exec);

  player->play();
  player->wait_for_playback_to_start();
  player->start();
  FluxPlayer & flux = player->flux();
  while (!g_stop) {
    if (!flux.finished()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else if (
      flux.error() || player->wait_for_playback_to_finish(std::chrono::milliseconds(100))) {
      break;
    }
  }
  flux.stop();
  player->stop();
  if (const std::exception_ptr e = flux.error()) std::rethrow_exception(e);
}

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
