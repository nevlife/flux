#include "flux/publisher.hpp"
#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/jpeg_frame.hpp"
#include "flux_bag/raw_meta.hpp"
#include "flux_bag/spin_thread.hpp"
#include "flux_bridge/tools.hpp"

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

constexpr const char * kFrameType = "flux_msgs/msg/FluxFrame";

struct Options
{
  rosbag2_storage::StorageOptions storage;
  rosbag2_transport::PlayOptions play;
};

constexpr const char * kUsage =
  "usage: ros2 bag play_flux BAG [-r RATE]\n"
  "\n"
  "Plays a bag. flux_msgs/FluxFrame topics go straight from the file back into the flux\n"
  "channel they were recorded from; every other topic is played by the stock player.\n";

auto parse(const std::vector<std::string> & args) -> Options
{
  Options o;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    auto value = [&]() -> const std::string & {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument(a + " needs a value");
      }
      return args[++i];
    };
    if (a == "-r" || a == "--rate") {
      o.play.rate = static_cast<float>(flux_bridge::positive_number("-r", value()));
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
    } else if (o.storage.uri.empty()) {
      o.storage.uri = a;
    } else {
      throw std::invalid_argument("one bag only");
    }
  }
  if (o.storage.uri.empty()) {
    throw std::invalid_argument("BAG is required");
  }
  if (!(o.play.rate > 0)) {
    throw std::invalid_argument("argument -r: Value must be positive");
  }
  return o;
}

auto decode(
  const rosbag2_storage::SerializedBagMessage & msg, flux_bag::FluxFrameFields & f,
  flux_bag::JpegMeta & meta, flux_bag::AuxTail & aux) -> bool
{
  const rcutils_uint8_array_t & raw = *msg.serialized_data;
  return flux_bag::read_frame(raw.buffer, raw.buffer_length, f, meta, aux);
}

// A waiting subscriber looks for its channel once per tick.
constexpr auto kAttachWait = std::chrono::nanoseconds(2 * FLUX_DEFAULT_TICK_NS);

// In bytes, not messages: a frame runs to several MiB.
constexpr std::size_t kReadAheadBytes = std::size_t{256} << 20;

using MessagePtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;

// The FluxFrame topics of one bag, published on flux at the bag's own pace. A loader thread reads
// the frames ahead of a publish thread, which sends each one when its own clock reaches the
// frame's receive time. A publisher per channel and geometry (slots and aux_size) the frames carry:
// a channel recorded across a restart with other slots is restarted the same way here.
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
      if (!first.has_next()) {
        continue;
      }
      const auto msg = first.read_next();
      flux_bag::FluxFrameFields f;
      flux_bag::JpegMeta meta;
      flux_bag::AuxTail aux;
      if (decode(*msg, f, meta, aux)) {
        publisher_for(f, meta.raw, aux.aux_size);
      }
    }
    if (!pubs_.empty()) {
      std::this_thread::sleep_for(kAttachWait);
    }
    reader_.open(storage);
    rosbag2_storage::StorageFilter filter;
    filter.topics = topics;
    reader_.set_filter(filter);
    loader_ = std::thread([this] { load(); });
  }

  FluxPlayer(const FluxPlayer &) = delete;
  auto operator=(const FluxPlayer &) -> FluxPlayer & = delete;
  FluxPlayer(FluxPlayer &&) = delete;
  auto operator=(FluxPlayer &&) -> FluxPlayer & = delete;
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
    if (loader_.joinable()) {
      loader_.join();
    }
    if (sender_.joinable()) {
      sender_.join();
    }
  }

  auto finished() const -> bool
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return finished_;
  }

  // What ended a thread early, rethrown on the main thread.
  auto error() const -> std::exception_ptr
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
  auto play_next() -> bool
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!clock_.is_paused() || finished_) {
      return false;
    }
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
    std::uint32_t aux_size = 0;
    std::unique_ptr<flux::Publisher> pub;
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
    if (!error_) {
      error_ = std::move(e);
    }
    wake_.notify_all();
  }

  void load()
  {
    try {
      while (reader_.has_next()) {
        MessagePtr msg = reader_.read_next();
        std::unique_lock<std::mutex> lock(mutex_);
        room_.wait(lock, [this] { return stop_ || queued_bytes_ < kReadAheadBytes; });
        if (stop_) {
          return;
        }
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
  auto next_due(bool & stepped) -> MessagePtr
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      if (stop_ || error_ || (queue_.empty() && loaded_)) {
        return nullptr;
      }
      if (queue_.empty()) {
        wake_.wait(lock);
        continue;
      }
      const rcutils_time_point_value_t due = queue_.front()->recv_timestamp;
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

  auto publisher_for(
    const flux_bag::FluxFrameFields & f, const flux_bag::RawMeta & meta,
    std::uint32_t aux_size) -> flux::Publisher *
  {
    Channel & c = pubs_[Key(f.flux_topic, f.fingerprint)];
    if (c.ignored) {
      return nullptr;
    }
    if (
      c.pub && c.slot_size == meta.slot_size && c.slot_count == meta.slot_count &&
      c.aux_size == aux_size) {
      return c.pub.get();
    }
    c.pub.reset();  // one publisher per channel: the old one goes first
    std::string why;
    if (meta.slot_size == 0 || meta.slot_count == 0) {
      // A recorded channel had both; 0 would be flux's default, not the recorded geometry.
      why = "the bag records no slot size or count";
    } else {
      try {
        c.pub = std::make_unique<flux::Publisher>(
          node_, std::string(f.flux_topic), f.fingerprint, meta.slot_size, meta.slot_count,
          flux::MemoryPolicy{}, flux::Publisher::kDefaultMaxSubscribers, aux_size);
      } catch (const flux::Error & e) {
        why = e.what();
      }
    }
    if (!c.pub) {
      c.ignored = true;
      RCLCPP_ERROR(
        node_.get_logger(), "Ignoring a flux channel '%s', reason: %s.",
        std::string(f.flux_topic).c_str(), why.c_str());
      return nullptr;
    }
    c.slot_size = meta.slot_size;
    c.slot_count = meta.slot_count;
    c.aux_size = aux_size;
    RCLCPP_INFO(
      node_.get_logger(), "flux channel %s: %u slots of %u bytes, %u aux bytes",
      std::string(f.flux_topic).c_str(), meta.slot_count, meta.slot_size, aux_size);
    return c.pub.get();
  }

  void publish(const rosbag2_storage::SerializedBagMessage & msg)
  {
    flux_bag::FluxFrameFields f;
    flux_bag::JpegMeta meta;
    flux_bag::AuxTail aux;
    if (!decode(msg, f, meta, aux)) {
      RCLCPP_ERROR_ONCE(
        node_.get_logger(), "%s: not a raw or jpeg FluxFrame, skipped", msg.topic_name.c_str());
      return;
    }
    const bool jpeg = f.codec == "jpeg";
    if (!jpeg && meta.raw.nbytes() != f.data_size) {
      RCLCPP_ERROR_ONCE(
        node_.get_logger(), "%s: a frame's data does not match its dtype and shape, skipped",
        msg.topic_name.c_str());
      return;
    }
    flux::Publisher * pub = publisher_for(f, meta.raw, aux.aux_size);
    if (pub == nullptr) {
      return;
    }
    flux::WriteSlot w = pub->loan(meta.raw.dtype, meta.raw.shape.data(), meta.raw.ndim);
    if (!w) {
      return;
    }
    if (!jpeg) {
      std::memcpy(w.data(), f.data, f.data_size);
    } else if (!flux_bag::unpack_jpeg(f, meta, static_cast<std::uint8_t *>(w.data()))) {
      RCLCPP_ERROR_ONCE(
        node_.get_logger(), "%s: a jpeg frame that does not decode, skipped",
        msg.topic_name.c_str());
      return;
    }
    // The publisher was made with aux.aux_size, which holds aux_len.
    if (aux.aux_len != 0) {
      std::memcpy(w.aux().data(), aux.aux, aux.aux_len);
    }
    w.set_aux_len(aux.aux_len);
    if (const flux::Published p = w.commit(); flux::faulted(p)) {
      RCLCPP_ERROR_ONCE(node_.get_logger(), "publish refused: %s", flux::to_string(p));
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

// The stock player for the ROS topics, its pause, resume and rate also driving the flux clock:
// its services and keys call these through the base class. seek is not virtual, so it moves the
// ROS topics only. play_next and burst step the ROS topics, the flux channels following to the
// last message played; once no ROS message is left they step the flux frames.
class BagPlayer : public rosbag2_transport::Player
{
public:
  BagPlayer(const Options & o, const std::vector<std::string> & flux_topics)
  : rosbag2_transport::Player(o.storage, o.play, "flux_bag_play"),
    options_(o.play),
    flux_(*this, o.storage, flux_topics, get_starting_time(), o.play.rate)
  {
    add_on_play_message_post_callback(
      [this](const MessagePtr & m) { played_ = m->recv_timestamp; });
  }

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

  auto flux() -> FluxPlayer & { return flux_; }

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

  auto set_rate(double rate) -> bool override
  {
    if (!rosbag2_transport::Player::set_rate(rate)) {
      return false;
    }
    flux_.set_rate(rate);
    return true;
  }

  auto play_next() -> bool override
  {
    if (!rosbag2_transport::Player::play_next()) {
      return flux_.play_next();
    }
    flux_.jump(played_);
    return true;
  }

  auto burst(const std::size_t num_messages) -> std::size_t override
  {
    std::size_t played = rosbag2_transport::Player::burst(num_messages);
    if (played > 0) {
      flux_.jump(played_);
    }
    while ((num_messages == 0 || played < num_messages) && flux_.play_next()) {
      ++played;
    }
    return played;
  }

private:
  rosbag2_transport::PlayOptions options_;
  FluxPlayer flux_;
  KeyboardHandler keyboard_;
  std::atomic<rcutils_time_point_value_t> played_{0};
};

// Global: a signal handler reaches nothing else.
std::atomic<bool> g_stop{false};  // NOLINT(*-avoid-non-const-global-variables)

void run(Options & opt)
{
  const rosbag2_storage::BagMetadata metadata =
    rosbag2_cpp::Info().read_metadata(opt.storage.uri, opt.storage.storage_id);
  std::vector<std::string> flux_topics;
  for (const auto & t : metadata.topics_with_message_count) {
    if (t.topic_metadata.type == kFrameType) {
      flux_topics.push_back(t.topic_metadata.name);
    }
  }
  opt.play.exclude_topics_to_filter = flux_topics;
  opt.play.disable_keyboard_controls = true;  // BagPlayer::start adds them
  opt.play.start_paused = true;               // until BagPlayer::start resumes both clocks

  auto player = std::make_shared<BagPlayer>(opt, flux_topics);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(player);
  const flux_bag::SpinThread spin(exec);

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
  if (const std::exception_ptr e = flux.error()) {
    std::rethrow_exception(e);
  }
}

}  // namespace

auto main(int argc, char ** argv) -> int
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
