#include "flux2/schemas.hpp"
#include "flux2_bag/bag_time.hpp"
#include "flux2_bag/flux_frame_cdr.hpp"
#include "flux2_bag/raw_meta.hpp"

#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

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

constexpr const char * kFrameType = "flux2_msgs/msg/FluxFrame";

constexpr const char * kUsage =
  "usage: ros2 bag convert_flux2 BAG -o OUT\n"
  "\n"
  "Writes a copy of BAG in which each flux2_msgs/FluxFrame topic becomes the ROS message its\n"
  "fingerprint names, on the channel's own topic, with the recorded times. Every other topic is\n"
  "copied as it is. A frame whose type is not installed stays a FluxFrame.\n";

struct Options
{
  std::string in;
  std::string out;
};

Options parse(const std::vector<std::string> & args)
{
  Options o;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "-o" || a == "--output") {
      if (i + 1 >= args.size()) throw std::invalid_argument(a + " needs a value");
      o.out = args[++i];
    } else if (!a.empty() && a[0] == '-') {
      throw std::invalid_argument("unknown option " + a);
    } else if (o.in.empty()) {
      o.in = a;
    } else {
      throw std::invalid_argument("one bag only");
    }
  }
  if (o.in.empty()) throw std::invalid_argument("BAG is required");
  if (o.out.empty()) throw std::invalid_argument("-o OUT is required");
  return o;
}

// Where the frames of one channel and fingerprint go: their ROS topic, or the FluxFrame topic
// they came from when the type is not installed or the ROS topic is taken.
struct Route
{
  std::string topic;
  std::optional<flux2::Schemas::Type> type;
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
      if (r.kept != 0) std::cout << ", " << r.kept << " kept as FluxFrame";
      std::cout << "\n";
    }
  }

private:
  void frame(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & msg)
  {
    const rcutils_uint8_array_t & raw = *msg->serialized_data;
    flux2_bag::FluxFrameFields f;
    flux2_bag::RawMeta meta;
    const bool ok = flux2_bag::FluxFrameCdr::read(raw.buffer, raw.buffer_length, f) &&
                    f.codec == "raw" && meta.read(f.meta, f.meta_size);
    if (!ok) throw std::runtime_error(msg->topic_name + ": a FluxFrame that does not parse");
    Route & r = route(msg->topic_name, std::string(f.flux_topic), f.fingerprint);
    if (!r.type) {
      writer_.write(msg);
      ++r.kept;
      return;
    }
    const std::size_t n = r.type->cdr_size(f.data, f.data_size);
    auto out = std::make_shared<rclcpp::SerializedMessage>(n);
    rcutils_uint8_array_t & cdr = out->get_rcl_serialized_message();
    cdr.buffer_length = r.type->write_cdr(f.data, f.data_size, cdr.buffer, cdr.buffer_capacity);
    auto converted = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    converted->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(out, &cdr);
    flux2_bag::copy_times(*msg, *converted);
    converted->topic_name = r.topic;
    writer_.write(converted);
    ++r.converted;
  }

  // Decided by the first frame of a channel and fingerprint.
  Route & route(const std::string & bag_topic, const std::string & channel, std::uint64_t fp)
  {
    const auto key = std::make_pair(channel, fp);
    if (const auto it = routes_.find(key); it != routes_.end()) return it->second;
    Route r;
    r.topic = bag_topic;
    if (schemas_.contains(fp)) {
      const flux2::Schemas::Type type = schemas_.at(fp);
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
    if (!r.type && kept_.insert(bag_topic).second) writer_.create_topic(frame_topics_[bag_topic]);
    return routes_.emplace(key, std::move(r)).first->second;
  }

  flux2::Schemas schemas_;
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

int main(int argc, char ** argv)
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
    std::cerr << "convert_flux2: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
