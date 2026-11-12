#include "flux/schemas.hpp"
#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/jpeg_frame.hpp"
#include "flux_bag/raw_meta.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

#include <cstdint>
#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr const char * kFrameType = "flux_msgs/msg/FluxFrame";

constexpr const char * kUsage =
  "usage: ros2 bag convert_flux BAG -o OUT\n"
  "\n"
  "Writes a copy of BAG in which each flux_msgs/FluxFrame topic becomes the ROS message its\n"
  "fingerprint names, on the channel's own topic, with the recorded times. A jpeg frame\n"
  "becomes a sensor_msgs/CompressedImage on the channel's topic + /compressed, as\n"
  "compressed_image_transport publishes it. Every other topic is copied as it is. A frame\n"
  "whose type is not installed stays a FluxFrame.\n";

struct Options
{
  std::string in;
  std::string out;
};

auto parse(const std::vector<std::string> & args) -> Options
{
  Options o;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "-o" || a == "--output") {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument(a + " needs a value");
      }
      o.out = args[++i];
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
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
  return o;
}

// Where the frames of one channel and fingerprint go: their ROS topic, or the FluxFrame topic
// they came from when the type is not installed or the ROS topic is taken.
struct Route
{
  std::string topic;
  std::optional<flux::Schemas::Type> type;
  // A GPU memory channel: the frame is its aux (from the aux tail) and its payload.
  bool gpu = false;
  std::uint64_t converted = 0;
  std::uint64_t kept = 0;
};

class Converter
{
public:
  explicit Converter(const Options & o)
  {
    reader_.open(o.in);
    rosbag2_storage::StorageOptions out;
    out.uri = o.out;
    writer_.open(out);
    for (const rosbag2_storage::TopicMetadata & t : reader_.get_all_topics_and_types()) {
      if (t.type == kFrameType) {
        frame_topics_[t.name] = t;
      } else {
        writer_.create_topic(t);
        types_[t.name] = t.type;
      }
    }
  }

  void run()
  {
    while (reader_.has_next()) {
      const auto msg = reader_.read_next();
      if (frame_topics_.count(msg->topic_name) != 0) {
        frame(msg);
      } else {
        writer_.write(msg);
      }
    }
    for (const auto & [key, r] : routes_) {
      std::cout << key.first << " -> " << r.topic << ": " << r.converted << " converted";
      if (r.kept != 0) {
        std::cout << ", " << r.kept << " kept as FluxFrame";
      }
      std::cout << "\n";
    }
  }

private:
  void frame(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & msg)
  {
    const rcutils_uint8_array_t & raw = *msg->serialized_data;
    flux_bag::FluxFrameFields f;
    flux_bag::JpegMeta meta;
    // A ROS message has no place for the aux bytes; a frame kept as a FluxFrame keeps them.
    flux_bag::AuxTail aux;
    if (!flux_bag::read_frame(raw.buffer, raw.buffer_length, f, meta, aux)) {
      throw std::runtime_error(msg->topic_name + ": a FluxFrame that does not parse");
    }
    if (f.codec == "jpeg") {
      jpeg_frame(msg, f, meta);
      return;
    }
    Route & r = route(msg->topic_name, std::string(f.flux_topic), f.fingerprint);
    if (!r.type) {
      writer_.write(msg);
      ++r.kept;
      return;
    }
    const auto * data = static_cast<const std::uint8_t *>(f.data);
    std::size_t size = f.data_size;
    if (r.gpu) {
      if (aux.aux_size == 0) {
        throw std::runtime_error(
          msg->topic_name + ": a GPU memory channel's FluxFrame without aux");
      }
      r.type->with_data({aux.aux, aux.aux_len}, {data, size}, joined_);
      data = joined_.data();
      size = joined_.size();
    }
    const std::size_t n = r.type->cdr_size(data, size);
    auto out = std::make_shared<rclcpp::SerializedMessage>(n);
    rcutils_uint8_array_t & cdr = out->get_rcl_serialized_message();
    cdr.buffer_length = r.type->write_cdr(data, size, cdr.buffer, cdr.buffer_capacity);
    auto converted = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    converted->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(out, &cdr);
    converted->recv_timestamp = msg->recv_timestamp;
    converted->send_timestamp = msg->send_timestamp;
    converted->topic_name = r.topic;
    writer_.write(converted);
    ++r.converted;
  }

  void jpeg_frame(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & msg,
    const flux_bag::FluxFrameFields & f, const flux_bag::JpegMeta & meta)
  {
    fields_.resize(meta.raw.nbytes());
    const auto jpeg = flux_bag::restore_fields(f, meta, fields_.data());
    const sensor_msgs::flux_msg::Image::View img(fields_.data(), fields_.size());
    sensor_msgs::msg::CompressedImage out;
    out.header.stamp.sec = img.header__sec();
    out.header.stamp.nanosec = img.header__nanosec();
    out.header.frame_id = std::string(img.header__frame_id());
    const std::string encoding(img.encoding());
    if (jpeg.empty() || !img.ok__()) {
      throw std::runtime_error(msg->topic_name + ": a jpeg FluxFrame that does not parse");
    }
    // The JPEG decodes to the same picture whatever order its channels came in; the text after
    // the encoding is what OpenCV, which decodes to BGR, turns it into.
    out.format = encoding + "; jpeg compressed " + (encoding == "mono8" ? "mono8" : "bgr8");
    out.data.assign(jpeg.begin(), jpeg.end());
    const std::string topic = std::string(f.flux_topic) + "/compressed";
    Route & r = routes_[std::make_pair(topic, f.fingerprint)];
    if (r.topic.empty()) {
      r.topic = topic;
      if (types_.count(topic) == 0) {
        rosbag2_storage::TopicMetadata t;
        t.name = topic;
        t.type = kCompressedType;
        t.serialization_format = "cdr";
        writer_.create_topic(t);
        types_[topic] = kCompressedType;
      }
    }
    if (types_[topic] != kCompressedType) {
      if (kept_.insert(msg->topic_name).second) {
        writer_.create_topic(frame_topics_[msg->topic_name]);
      }
      writer_.write(msg);
      ++r.kept;
      return;
    }
    auto cdr = std::make_shared<rclcpp::SerializedMessage>();
    compressed_.serialize_message(&out, cdr.get());
    auto converted = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    converted->serialized_data =
      std::shared_ptr<rcutils_uint8_array_t>(cdr, &cdr->get_rcl_serialized_message());
    converted->recv_timestamp = msg->recv_timestamp;
    converted->send_timestamp = msg->send_timestamp;
    converted->topic_name = topic;
    writer_.write(converted);
    ++r.converted;
  }

  // Decided by the first frame of a channel and fingerprint.
  auto route(const std::string & bag_topic, const std::string & channel, std::uint64_t fp)
    -> Route &
  {
    const auto key = std::make_pair(channel, fp);
    if (const auto it = routes_.find(key); it != routes_.end()) {
      return it->second;
    }
    Route r;
    r.topic = bag_topic;
    r.gpu = !schemas_.contains(fp) && schemas_.contains_gpu(fp);
    if (schemas_.contains(fp) || r.gpu) {
      const flux::Schemas::Type type = r.gpu ? schemas_.at_gpu(fp) : schemas_.at(fp);
      const std::string ros_type = type.ros_type();
      const auto taken = types_.find(channel);
      if (taken == types_.end()) {
        rosbag2_storage::TopicMetadata t;
        t.name = channel;
        t.type = ros_type;
        t.serialization_format = "cdr";
        writer_.create_topic(t);
        types_[channel] = ros_type;
      }
      if (types_[channel] == ros_type) {
        r.topic = channel;
        r.type = type;
      }
    }
    if (!r.type && kept_.insert(bag_topic).second) {
      writer_.create_topic(frame_topics_[bag_topic]);
    }
    return routes_.emplace(key, std::move(r)).first->second;
  }

  static constexpr const char * kCompressedType = "sensor_msgs/msg/CompressedImage";

  flux::Schemas schemas_;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_;
  std::vector<std::uint8_t> fields_;
  std::vector<std::uint8_t> joined_;
  rosbag2_cpp::Reader reader_;
  rosbag2_cpp::Writer writer_;
  std::map<std::string, rosbag2_storage::TopicMetadata> frame_topics_;
  // Every topic of the output and its type: a channel whose topic a ROS topic of another type
  // already holds keeps its frames.
  std::map<std::string, std::string> types_;
  std::map<std::pair<std::string, std::uint64_t>, Route> routes_;
  std::set<std::string> kept_;
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
    std::cerr << "convert_flux: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
