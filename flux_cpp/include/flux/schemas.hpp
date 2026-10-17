#ifndef FLUX_SCHEMAS_HPP
#define FLUX_SCHEMAS_HPP

#include "flux/channel.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace flux
{

// The installed message types (every .msg on AMENT_PREFIX_PATH the flat wire can carry), found
// by the fingerprint a channel carries. Read once at construction.
class Schemas
{
public:
  class Type
  {
  public:
    // pkg/msg/Name, as rclcpp names a type.
    std::string ros_type() const { return flux_schema_ros_type(s_); }
    // A frame of this type as the CDR bytes rmw publishes for the ROS message. Throws Error
    // (FLUX_MISMATCH) for a frame that does not match the schema.
    std::size_t cdr_size(const void * data, std::size_t size) const
    {
      std::size_t n = 0;
      detail::check(flux_schema_cdr_size(s_, static_cast<const std::uint8_t *>(data), size, &n));
      return n;
    }
    std::size_t write_cdr(
      const void * data, std::size_t size, std::uint8_t * out, std::size_t cap) const
    {
      std::size_t n = 0;
      detail::check(
        flux_schema_write_cdr(s_, static_cast<const std::uint8_t *>(data), size, out, cap, &n));
      return n;
    }

  private:
    friend class Schemas;
    explicit Type(const flux_schema * s) noexcept : s_(s) {}
    const flux_schema * s_;
  };

  // A .msg that does not parse is left out; throws Error only when a directory cannot be read.
  Schemas() : s_(load()) {}

  std::size_t size() const noexcept { return flux_schemas_size(s_.get()); }
  bool contains(std::uint64_t fingerprint) const noexcept
  {
    return flux_schemas_find(s_.get(), fingerprint) != nullptr;
  }
  // Requires contains(fingerprint). Valid while this Schemas lives.
  Type at(std::uint64_t fingerprint) const noexcept
  {
    return Type(flux_schemas_find(s_.get(), fingerprint));
  }

private:
  static flux_schemas * load()
  {
    detail::require_abi();
    flux_schemas * s = nullptr;
    detail::check(flux_schemas_load(&s));
    return s;
  }
  struct Free
  {
    void operator()(flux_schemas * s) const noexcept { flux_schemas_free(s); }
  };
  std::unique_ptr<flux_schemas, Free> s_;
};

}  // namespace flux

#endif  // FLUX_SCHEMAS_HPP
