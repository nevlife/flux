#ifndef FLUX_BAG_BAG_TIME_HPP
#define FLUX_BAG_BAG_TIME_HPP

#include <rosbag2_storage/serialized_bag_message.hpp>

#include <rcutils/time.h>

namespace flux_bag
{

// rosbag2 before 0.26 (Jazzy) stores one time per message, when it was received, and no send
// time.
inline rcutils_time_point_value_t received(const rosbag2_storage::SerializedBagMessage & m)
{
#if FLUX_ROSBAG2_MINOR >= 26
  return m.recv_timestamp;
#else
  return m.time_stamp;
#endif
}

inline void set_times(
  rosbag2_storage::SerializedBagMessage & m, rcutils_time_point_value_t received,
  [[maybe_unused]] rcutils_time_point_value_t sent)
{
#if FLUX_ROSBAG2_MINOR >= 26
  m.recv_timestamp = received;
  m.send_timestamp = sent;
#else
  m.time_stamp = received;
#endif
}

inline void copy_times(
  const rosbag2_storage::SerializedBagMessage & from, rosbag2_storage::SerializedBagMessage & to)
{
#if FLUX_ROSBAG2_MINOR >= 26
  set_times(to, from.recv_timestamp, from.send_timestamp);
#else
  to.time_stamp = from.time_stamp;
#endif
}

}  // namespace flux_bag

#endif  // FLUX_BAG_BAG_TIME_HPP
