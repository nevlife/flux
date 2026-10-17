#ifndef FLUX2_BRIDGE_TOOLS_HPP
#define FLUX2_BRIDGE_TOOLS_HPP

#include <rmw/validate_full_topic_name.h>

#include <charconv>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <system_error>

// What the bridge and flux2_bag's recorder both do with a command line and a channel list.
namespace flux2_bridge
{

// The value of `option`: a decimal number with nothing after it, finite and above 0. For --poll
// SEC, nan would make the loop spin and inf would make it never look again.
inline double positive_number(const std::string & option, const std::string & text)
{
  double v = 0;
  const char * end = text.data() + text.size();
  const auto [at, ec] = std::from_chars(text.data(), end, v);
  if (text.empty() || ec != std::errc() || at != end) {
    throw std::invalid_argument("argument " + option + ": Expects a floating point number");
  }
  if (!std::isfinite(v))
    throw std::invalid_argument("argument " + option + ": Value must be finite");
  if (v <= 0.0) throw std::invalid_argument("argument " + option + ": Value must be positive");
  return v;
}

// A channel's key comes from another process. rclcpp throws on a name that breaks the ROS rules,
// and such a name can hold terminal escapes, so it is neither used nor printed.
inline bool is_ros_topic_name(const std::string & key)
{
  int result = RMW_TOPIC_VALID;
  std::size_t index = 0;
  return rmw_validate_full_topic_name(key.c_str(), &result, &index) == RMW_RET_OK &&
         result == RMW_TOPIC_VALID;
}

}  // namespace flux2_bridge

#endif  // FLUX2_BRIDGE_TOOLS_HPP
