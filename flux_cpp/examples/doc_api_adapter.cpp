// Compile-only companion to docs/en/api.en.md section 2, on the same contract as doc_api.cpp: every
// region between [doc:id] and [doc:/id] is the code the fence tagged `doc:id` shows.
//
// What this file adds over doc_api.cpp is the generated side. The adapters it includes are
// emitted at build time -- my_pkg/Cloud from docs/examples/Cloud.msg, sensor_msgs/Image from the
// installed message -- so the names the generator invents (kFingerprint, Builder over a
// WriteSlot, commit, the bridge pair) cannot change without breaking this build.

#include "flux/ros/executor.hpp"
#include "flux/ros/message_filters/subscriber.hpp"
#include "flux/ros/publisher.hpp"
#include "flux/ros/subscription.hpp"
#include "my_pkg/flux/cloud.hpp"

#include <message_filters/sync_policies/approximate_time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/flux/image_ros.hpp"

#include <message_filters/synchronizer.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace flux::doc_examples
{

struct Lidar
{
  void read_into(float *, std::size_t) {}
};

void use(float)
{
}

void use(std::uint32_t, std::uint32_t)
{
}

void doc_adapter_publish(const rclcpp::Node::SharedPtr & node, Lidar & lidar, std::size_t n)
{
  // [doc:adapter_cpp_pub]
  using my_pkg::flux_msg::Cloud;

  flux::ros::Publisher pub(*node, "cloud", Cloud::kFingerprint, 16 << 20, 16);

  Cloud::Builder b = Cloud::build__(pub);
  if (b) {
    auto xs = b.alloc__x(n);
    lidar.read_into(xs.data(), n);
    b.set__width(static_cast<std::uint32_t>(n));
    b.set__label("front");
    b.commit__();
  }
  // [doc:/adapter_cpp_pub]
}

void doc_adapter_subscribe(const rclcpp::Node::SharedPtr & node)
{
  // [doc:adapter_cpp_sub]
  using my_pkg::flux_msg::Cloud;

  flux::ros::Subscription sub(*node, "cloud", Cloud::kFingerprint, [](const flux::FrameView & f) {
    Cloud::View c(f);
    for (float x : c.x()) {
      use(x);
    }
    if (!c.ok__()) {
      return;
    }
  });
  // [doc:/adapter_cpp_sub]
}

void doc_adapter_bridge(
  flux::WriteSlot & w, const sensor_msgs::msg::Image & img, const flux::FrameView & f)
{
  // [doc:adapter_cpp_bridge]
  using sensor_msgs::flux_msg::Image;

  Image::Builder b(w);
  msg_to_frame(img, b);
  b.commit__();

  sensor_msgs::msg::Image back = frame_to_msg(Image::View(f));
  // [doc:/adapter_cpp_bridge]
  (void)back;
}

void doc_message_filters(rclcpp::Node & node)
{
  // [doc:adapter_cpp_sync]
  namespace mf = message_filters;
  using sensor_msgs::flux_msg::Image;
  using Frame = flux::ros::message_filters::StampedFrame<Image>;

  flux::QoS qos;
  qos.depth = 4;
  qos.max_borrow = 16;  // inputs x queue_size: a filter holds every frame until its partner lands

  flux::ros::message_filters::Subscriber<Image> left(node, "cam/left", qos);
  flux::ros::message_filters::Subscriber<Image> right(node, "cam/right", qos);

  using Policy = mf::sync_policies::ApproximateTime<Frame, Frame>;
  mf::Synchronizer<Policy> sync(Policy(10), left, right);
  sync.registerCallback(std::bind(
    [](const std::shared_ptr<const Frame> & a, const std::shared_ptr<const Frame> & b) {
      Image::View l = a->view();
      Image::View r = b->view();
      use(l.width(), r.width());
    },
    std::placeholders::_1, std::placeholders::_2));

  flux::ros::Executor ex;
  ex.add(left);  // every input of one synchronizer must run on one thread
  ex.add(right);
  ex.spin();
  // [doc:/adapter_cpp_sync]
}

}  // namespace flux::doc_examples
