#ifndef FLUX_WRAPPER_DETAIL_COPY_BUT_DATA_HPP
#define FLUX_WRAPPER_DETAIL_COPY_BUT_DATA_HPP

#include <rclcpp/get_message_type_support_handle.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace flux_wrapper::detail
{

// Every field of a ROS message but its root `data`, copied without reading `data`, for a type the
// wrapper knows nothing else of. C++ has no member reflection, so the members come from the
// message's introspection type support.
namespace introspection = rosidl_typesupport_introspection_cpp;

inline void copy_members(
  const introspection::MessageMembers & members, const void * src, void * dst, const char * skip);

inline auto primitive_size(std::uint8_t type) -> std::size_t
{
  switch (type) {
    case introspection::ROS_TYPE_DOUBLE:
    case introspection::ROS_TYPE_UINT64:
    case introspection::ROS_TYPE_INT64:
      return 8;
    case introspection::ROS_TYPE_LONG_DOUBLE:
      return sizeof(long double);
    case introspection::ROS_TYPE_FLOAT:
    case introspection::ROS_TYPE_UINT32:
    case introspection::ROS_TYPE_INT32:
      return 4;
    case introspection::ROS_TYPE_WCHAR:
    case introspection::ROS_TYPE_UINT16:
    case introspection::ROS_TYPE_INT16:
      return 2;
    default:  // char, bool, octet, uint8, int8
      return 1;
  }
}

// One value of a member that is not a message: a string, or a primitive of primitive_size bytes.
inline void copy_value(std::uint8_t type, const void * src, void * dst)
{
  if (type == introspection::ROS_TYPE_STRING) {
    *static_cast<std::string *>(dst) = *static_cast<const std::string *>(src);
  } else if (type == introspection::ROS_TYPE_WSTRING) {
    *static_cast<std::u16string *>(dst) = *static_cast<const std::u16string *>(src);
  } else {
    std::memcpy(dst, src, primitive_size(type));
  }
}

inline void copy_member(const introspection::MessageMember & m, const void * src, void * dst)
{
  const auto * nested = m.type_id_ == introspection::ROS_TYPE_MESSAGE
                          ? static_cast<const introspection::MessageMembers *>(m.members_->data)
                          : nullptr;
  if (!m.is_array_) {
    if (nested != nullptr) {
      copy_members(*nested, src, dst, nullptr);
    } else {
      copy_value(m.type_id_, src, dst);
    }
    return;
  }
  const bool fixed = m.array_size_ != 0 && !m.is_upper_bound_;
  const std::size_t n = fixed ? m.array_size_ : m.size_function(src);
  if (!fixed) {
    m.resize_function(dst, n);
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (nested != nullptr) {
      copy_members(*nested, m.get_const_function(src, i), m.get_function(dst, i), nullptr);
    } else if (m.type_id_ == introspection::ROS_TYPE_BOOLEAN) {
      // std::vector<bool> has no element addresses, only fetch and assign.
      bool v = false;
      m.fetch_function(src, i, &v);
      m.assign_function(dst, i, &v);
    } else {
      copy_value(m.type_id_, m.get_const_function(src, i), m.get_function(dst, i));
    }
  }
}

// Recursive through nested messages, as deep as the message definition nests: a ROS message
// cannot contain itself.
inline void copy_members(
  const introspection::MessageMembers & members, const void * src, void * dst, const char * skip)
{
  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    const introspection::MessageMember & m = members.members_[i];
    if (skip != nullptr && std::strcmp(m.name_, skip) == 0) {
      continue;
    }
    copy_member(
      m, static_cast<const std::byte *>(src) + m.offset_,
      static_cast<std::byte *>(dst) + m.offset_);
  }
}

// Throws std::runtime_error when MsgT's package has no introspection type support.
template <class MsgT>
auto members_of() -> const introspection::MessageMembers &
{
  static const introspection::MessageMembers * members = [] {
    const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
      &rclcpp::get_message_type_support_handle<MsgT>(), introspection::typesupport_identifier);
    if (ts == nullptr) {
      throw std::runtime_error(
        "flux_wrapper: the message has no rosidl_typesupport_introspection_cpp type support");
    }
    return static_cast<const introspection::MessageMembers *>(ts->data);
  }();
  return *members;
}

// dst gets every field of src but the root `data`, which it keeps.
template <class MsgT>
void copy_but_data(const MsgT & src, MsgT & dst)
{
  copy_members(members_of<MsgT>(), &src, &dst, "data");
}

}  // namespace flux_wrapper::detail

#endif  // FLUX_WRAPPER_DETAIL_COPY_BUT_DATA_HPP
