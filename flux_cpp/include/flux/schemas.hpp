#ifndef FLUX_SCHEMAS_HPP
#define FLUX_SCHEMAS_HPP

#include "flux/channel.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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
    [[nodiscard]] auto ros_type() const -> std::string { return flux_schema_ros_type(s_); }
    // A frame of this type as the CDR bytes rmw publishes for the ROS message. Throws Error
    // (FLUX_MISMATCH) for a frame that does not match the schema.
    auto cdr_size(const void * data, std::size_t size) const -> std::size_t
    {
      std::size_t n = 0;
      detail::check(flux_schema_cdr_size(s_, static_cast<const std::uint8_t *>(data), size, &n));
      return n;
    }
    auto write_cdr(const void * data, std::size_t size, std::uint8_t * out, std::size_t cap) const
      -> std::size_t
    {
      std::size_t n = 0;
      detail::check(
        flux_schema_write_cdr(s_, static_cast<const std::uint8_t *>(data), size, out, cap, &n));
      return n;
    }

    // A frame whose fields are in its publisher's arena (FrameView::heap()) as one that holds them
    // itself, of the same schema, into `out`. Throws Error (FLUX_MISMATCH) for a frame that does
    // not match the schema or has a field in GPU memory.
    void inline_arena(
      const void * data, std::size_t size, wire::Span<const std::byte> heap,
      std::vector<std::uint8_t> & out) const
    {
      const auto * d = static_cast<const std::uint8_t *>(data);
      const auto * h = reinterpret_cast<const std::uint8_t *>(heap.data());  // NOLINT
      std::size_t n = 0;
      flux_status s =
        flux_schema_inline_arena(s_, d, size, h, heap.size(), out.data(), out.size(), &n);
      if (s == FLUX_INVALID_ARGUMENT && n > out.size()) {
        out.resize(n);
        s = flux_schema_inline_arena(s_, d, size, h, heap.size(), out.data(), out.size(), &n);
      }
      detail::check(s);
      out.resize(n);
    }

  private:
    friend class Schemas;
    explicit Type(const flux_schema * s) noexcept : s_(s) {}
    const flux_schema * s_;
  };

  // A .msg that does not parse is left out; throws Error only when a directory cannot be read.
  Schemas() : s_(load()) {}

  [[nodiscard]] auto size() const noexcept -> std::size_t { return flux_schemas_size(s_.get()); }
  [[nodiscard]] auto contains(std::uint64_t fingerprint) const noexcept -> bool
  {
    return flux_schemas_find(s_.get(), fingerprint) != nullptr;
  }
  // Requires contains(fingerprint). Valid while this Schemas lives.
  [[nodiscard]] auto at(std::uint64_t fingerprint) const noexcept -> Type
  {
    return Type(flux_schemas_find(s_.get(), fingerprint));
  }

private:
  static auto load() -> flux_schemas *
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
