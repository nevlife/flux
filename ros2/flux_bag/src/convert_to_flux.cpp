#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/raw_meta.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

#include <sensor_msgs/flux/compressed_image_ros.hpp>
#include <sensor_msgs/flux/image_ros.hpp>
#include <sensor_msgs/flux/point_cloud2_ros.hpp>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

constexpr const char * kFrameType = "flux_msgs/msg/FluxFrame";
constexpr const char * kBagTopicSuffix = "/flux";
// As many slots as a flux Slot publisher starts with.
constexpr std::uint32_t kSlotCount = 12;

constexpr const char * kUsage =
  "usage: ros2 bag convert_to_flux BAG -o OUT --topics TOPIC [TOPIC ...]\n"
  "\n"
  "Writes a copy of BAG in which each TOPIC becomes the flux channel of that name, stored as\n"
  "record_flux stores one (TOPIC/flux, flux_msgs/msg/FluxFrame), with the recorded times.\n"
  "Every other topic is copied as it is. TOPIC is a sensor_msgs/msg/PointCloud2, Image or\n"
  "CompressedImage topic.\n";

struct Options
{
  std::string in;
  std::string out;
  std::set<std::string> topics;
};

auto parse(const std::vector<std::string> & args) -> Options
{
  Options o;
  bool topics = false;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "-o" || a == "--output") {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument(a + " needs a value");
      }
      o.out = args[++i];
      topics = false;
    } else if (a == "--topics") {
      topics = true;
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
    } else if (topics) {
      o.topics.insert(a);
    } else if (o.in.empty()) {
      o.in = a;
    } else {
      throw std::invalid_argument("one bag only");
    }
  }
  if (o.in.empty()) {
    throw std::invalid_argument("BAG is required");
  }
  if (o.out.empty()) {
    throw std::invalid_argument("-o OUT is required");
  }
  if (o.topics.empty()) {
    throw std::invalid_argument("--topics needs at least one topic");
  }
  return o;
}

// The frame of the message in `cdr`, built into `frame` as a publisher's slot would hold it with
// every field in the slot. Its size.
using Build = std::size_t (*)(const rcutils_uint8_array_t & cdr, std::vector<std::uint8_t> & frame);

template <class Msg>
auto build(const rcutils_uint8_array_t & cdr, std::vector<std::uint8_t> & frame) -> std::size_t
{
  const rclcpp::SerializedMessage serialized(cdr);
  Msg m;
  rclcpp::Serialization<Msg>().deserialize_message(&serialized, &m);
  using Frame = typename flux::RosAdapter<Msg>::type;
  frame.resize(std::max(frame.size(), cdr.buffer_length + 4096));
  for (;;) {
    typename Frame::Builder b(frame.data(), frame.size());
    msg_to_frame(m, b);
    if (b.ok__()) {
      return b.size__();
    }
    // A builder past its buffer reports the size it wanted; anything else does not fit at all.
    if (b.size__() <= frame.size()) {
      throw std::runtime_error("a message that does not fit a flux frame");
    }
    frame.resize(b.size__());
  }
}

struct Type
{
  Build build = nullptr;
  std::uint64_t fingerprint = 0;
};

template <class Msg>
auto type() -> Type
{
  return {&build<Msg>, flux::RosAdapter<Msg>::type::kFingerprint};
}

auto types() -> const std::map<std::string, Type> &
{
  static const std::map<std::string, Type> t = {
    {"sensor_msgs/msg/CompressedImage", type<sensor_msgs::msg::CompressedImage>()},
    {"sensor_msgs/msg/Image", type<sensor_msgs::msg::Image>()},
    {"sensor_msgs/msg/PointCloud2", type<sensor_msgs::msg::PointCloud2>()},
  };
  return t;
}

struct Channel
{
  Type type;
  std::string ros_type;
  std::string bag_topic;
  // Grows to the largest frame so far, as record_flux sizes a copied frame.
  std::uint32_t slot_size = 4096;
  std::uint64_t frames = 0;
};

auto storage_at(const std::string & uri) -> rosbag2_storage::StorageOptions
{
  rosbag2_storage::StorageOptions s;
  s.uri = uri;
  return s;
}

class Converter
{
public:
  // A reader as ros2 bag play makes it, so a compressed bag reads too.
  explicit Converter(const Options & o)
  : reader_(rosbag2_transport::ReaderWriterFactory::make_reader(storage_at(o.in)))
  {
    reader_->open(storage_at(o.in));
    std::set<std::string> names;
    for (const rosbag2_storage::TopicMetadata & t : reader_->get_all_topics_and_types()) {
      names.insert(t.name);
    }
    for (const rosbag2_storage::TopicMetadata & t : reader_->get_all_topics_and_types()) {
      if (o.topics.count(t.name) == 0) {
        continue;
      }
      const auto it = types().find(t.type);
      if (it == types().end()) {
        throw std::runtime_error(t.name + ": no flux frame for " + t.type);
      }
      if (names.count(t.name + kBagTopicSuffix) != 0) {
        throw std::runtime_error(t.name + kBagTopicSuffix + " is already in the bag");
      }
      channels_[t.name] = Channel{it->second, t.type, t.name + kBagTopicSuffix};
    }
    for (const std::string & name : o.topics) {
      if (channels_.count(name) == 0) {
        throw std::runtime_error(name + " is not in the bag");
      }
    }
    writer_.open(storage_at(o.out));
    for (const rosbag2_storage::TopicMetadata & t : reader_->get_all_topics_and_types()) {
      if (const auto it = channels_.find(t.name); it != channels_.end()) {
        rosbag2_storage::TopicMetadata frame;
        frame.name = it->second.bag_topic;
        frame.type = kFrameType;
        frame.serialization_format = "cdr";
        writer_.create_topic(frame);
      } else {
        writer_.create_topic(t);
      }
    }
  }

  void run()
  {
    while (reader_->has_next()) {
      const auto msg = reader_->read_next();
      if (const auto it = channels_.find(msg->topic_name); it != channels_.end()) {
        convert(it->first, it->second, *msg);
      } else {
        writer_.write(msg);
      }
    }
    for (const auto & [name, c] : channels_) {
      std::cout << name << " -> " << c.bag_topic << ": " << c.frames << " frames, slots of "
                << c.slot_size << " bytes\n";
    }
  }

private:
  void convert(
    const std::string & name, Channel & c, const rosbag2_storage::SerializedBagMessage & msg)
  {
    std::size_t size = 0;
    try {
      size = c.type.build(*msg.serialized_data, frame_);
    } catch (const std::exception & e) {
      throw std::runtime_error(name + ": " + e.what());
    }
    while (c.slot_size < size) {
      if (c.slot_size >= 1u << 31) {
        throw std::runtime_error(name + ": a frame of " + std::to_string(size) + " bytes");
      }
      c.slot_size *= 2;
    }
    flux_bag::RawMeta meta;
    meta.slot_size = c.slot_size;
    meta.slot_count = kSlotCount;
    meta.ndim = 1;
    meta.shape[0] = size;
    std::vector<std::uint8_t> meta_bytes(flux_bag::RawMeta::kMaxSize);
    flux_bag::FluxFrameFields f;
    // As record_flux: the frame's header is when it was sent.
    f.sec = static_cast<std::int32_t>(msg.send_timestamp / 1'000'000'000);
    f.nanosec = static_cast<std::uint32_t>(msg.send_timestamp % 1'000'000'000);
    f.flux_topic = name;
    f.fingerprint = c.type.fingerprint;
    f.ros_type = c.ros_type;
    f.codec = "raw";
    f.meta = meta_bytes.data();
    f.meta_size = meta.write(meta_bytes.data());
    f.data = frame_.data();
    f.data_size = size;

    auto out = std::make_shared<rclcpp::SerializedMessage>(flux_bag::FluxFrameCdr::size(f));
    rcutils_uint8_array_t & cdr = out->get_rcl_serialized_message();
    cdr.buffer_length = flux_bag::FluxFrameCdr::write(f, cdr.buffer);
    auto frame = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    frame->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(out, &cdr);
    frame->recv_timestamp = msg.recv_timestamp;
    frame->send_timestamp = msg.send_timestamp;
    frame->topic_name = c.bag_topic;
    writer_.write(frame);
    ++c.frames;
  }

  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  rosbag2_cpp::Writer writer_;
  std::map<std::string, Channel> channels_;
  std::vector<std::uint8_t> frame_;
};

}  // namespace

auto main(int argc, char ** argv) -> int
{
  Options opt;
  try {
    opt = parse(std::vector<std::string>(argv, argv + argc));
  } catch (const std::exception & e) {
    std::cerr << kUsage << "error: " << e.what() << "\n";
    return 2;
  }
  try {
    Converter(opt).run();
  } catch (const std::exception & e) {
    std::cerr << "convert_to_flux: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
