#ifndef FLUX_TOOLS__RVIZ__TOPIC_LIST_HPP_
#define FLUX_TOOLS__RVIZ__TOPIC_LIST_HPP_

#include "flux/discovery.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"

#include <cstdint>

namespace flux_tools::rviz
{

// Live channels of one type in this process's domain, offered as options of a Topic property.
inline void fill_topic_list(
  rviz_common::properties::EditableEnumProperty * property, std::uint64_t fingerprint)
{
  property->clearOptions();
  for (const flux::TopicView & t : flux::enumerate_topics()) {
    if (
      t.fingerprint == fingerprint && t.domain == flux::process_domain() && t.key_exact &&
      flux::read_channel_stats(t.signpost).live)
    {
      property->addOptionStd(t.key);
    }
  }
}

}  // namespace flux_tools::rviz

#endif  // FLUX_TOOLS__RVIZ__TOPIC_LIST_HPP_
