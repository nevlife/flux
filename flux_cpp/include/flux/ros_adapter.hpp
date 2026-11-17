#ifndef FLUX_ROS_ADAPTER_HPP
#define FLUX_ROS_ADAPTER_HPP

namespace flux
{

// RosAdapter<pkg::msg::Foo>::type is pkg::flux_msg::Foo, set by the generated <snake>_ros.hpp.
// Code that knows only the ROS type finds the adapter so; without that header there is no type.
template <class RosMsg>
struct RosAdapter
{
};

}  // namespace flux

#endif  // FLUX_ROS_ADAPTER_HPP
