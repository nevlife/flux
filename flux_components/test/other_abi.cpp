// A component built as if against another flux release: its headers name an ABI the loaded library
// does not have. Its first flux call must refuse it, even in a container whose own flux code is of
// the loaded library.
#include "flux.h"
#undef FLUX_ABI_VERSION
#define FLUX_ABI_VERSION 0  // NOLINT(cppcoreguidelines-macro-usage): stands in for another release

#include <flux/topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace flux_components_test
{

class OtherAbi : public rclcpp::Node
{
public:
  explicit OtherAbi(const rclcpp::NodeOptions & options) : Node("other_abi", options)
  {
    static_cast<void>(flux::enumerate_topics());
  }
};

}  // namespace flux_components_test

RCLCPP_COMPONENTS_REGISTER_NODE(flux_components_test::OtherAbi)
