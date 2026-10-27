// Components the container test loads. Each callback records "<node name>/<label>" with its
// group as the key.
#include "probe.hpp"

#include <flux/subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace flux_components_test
{

using namespace std::chrono_literals;

class Recording : public rclcpp::Node
{
protected:
  Recording(const char * name, const rclcpp::NodeOptions & options) : Node(name, options) {}

  rclcpp::TimerBase::SharedPtr every_10ms(
    const std::string & label, const rclcpp::CallbackGroup::SharedPtr & group = nullptr)
  {
    const void * key =
      (group ? group : get_node_base_interface()->get_default_callback_group()).get();
    return create_wall_timer(
      10ms, [this, label, key] { probe::record(std::string(get_name()) + "/" + label, key); },
      group);
  }
};

// A timer in the default group and in two groups of its own.
class Plain : public Recording
{
public:
  explicit Plain(const rclcpp::NodeOptions & options)
  : Recording("plain", options),
    a_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    b_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  {
  }

private:
  rclcpp::CallbackGroup::SharedPtr a_;
  rclcpp::CallbackGroup::SharedPtr b_;
  rclcpp::TimerBase::SharedPtr in_default_{every_10ms("default")};
  rclcpp::TimerBase::SharedPtr in_a_{every_10ms("a", a_)};
  rclcpp::TimerBase::SharedPtr in_b_{every_10ms("b", b_)};
};

// A flux subscription in a group of its own, on the topic its "topic" parameter names.
class FluxSub : public Recording
{
public:
  explicit FluxSub(const rclcpp::NodeOptions & options)
  : Recording("flux_sub", options),
    frames_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    timer_(every_10ms("default")),
    sub_(
      *this, declare_parameter<std::string>("topic"), flux::kNoSchema, flux::QoS{},
      [this](const flux::FrameView &) {
        probe::record(std::string(get_name()) + "/frame", frames_.get());
      })
  {
    sub_.add_to_callback_group(*this, frames_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr frames_;
  rclcpp::TimerBase::SharedPtr timer_;
  flux::Subscription sub_;
};

// Creates a group with a timer from its first default-group callback.
class Late : public Recording
{
public:
  explicit Late(const rclcpp::NodeOptions & options)
  : Recording("late", options),
    timer_(every_10ms("default")),
    once_(create_wall_timer(50ms, [this] {
      once_->cancel();
      late_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      late_timer_ = every_10ms("late", late_);
    }))
  {
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr once_;
  rclcpp::CallbackGroup::SharedPtr late_;
  rclcpp::TimerBase::SharedPtr late_timer_;
};

class Reentrant : public Recording
{
public:
  explicit Reentrant(const rclcpp::NodeOptions & options)
  : Recording("reentrant", options),
    group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
    timer_(every_10ms("reentrant", group_))
  {
  }

private:
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Records the node's clock, ROS time once use_sim_time is set.
class Clocked : public rclcpp::Node
{
public:
  explicit Clocked(const rclcpp::NodeOptions & options)
  : Node("clocked", options), timer_(create_wall_timer(10ms, [this] {
      probe::record(std::string(get_name()) + "/now", nullptr, now().seconds());
    }))
  {
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace flux_components_test

RCLCPP_COMPONENTS_REGISTER_NODE(flux_components_test::Plain)
RCLCPP_COMPONENTS_REGISTER_NODE(flux_components_test::FluxSub)
RCLCPP_COMPONENTS_REGISTER_NODE(flux_components_test::Late)
RCLCPP_COMPONENTS_REGISTER_NODE(flux_components_test::Reentrant)
RCLCPP_COMPONENTS_REGISTER_NODE(flux_components_test::Clocked)
