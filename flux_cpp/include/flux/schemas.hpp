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

// Whether `v` has fields in its publisher's arenas, which Schemas::Type::inline_arena copies in.
inline auto in_arena(const FrameView & v) -> bool
{
  if (!v.heap().empty()) {
    return true;
  }
  const wire::GpuHeap g = v.gpu_heap();
  return g.count != 0 || g.host != nullptr;
}

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

    // `v` with its fields in its publisher's arenas copied in, into `out`: a frame of this type
    // that holds them itself. GPU arena fields come from the host copies a CPU reader gets
    // (FrameView::gpu_heap()). Throws Error (FLUX_MISMATCH) for a frame that does not match.
    void inline_arena(const FrameView & v, std::vector<std::uint8_t> & out) const
    {
      const auto * d = static_cast<const std::uint8_t *>(v.data());
      const auto heap = v.heap();
      const auto * h = reinterpret_cast<const std::uint8_t *>(heap.data());  // NOLINT
      const wire::GpuHeap g = v.gpu_heap();
      flux_gpu_heap raw{};
      raw.device = g.device;
      raw.host = reinterpret_cast<const std::uint8_t *>(g.host);  // NOLINT
      raw.size = g.size;
      raw.frame = g.frame;
      raw.blocks = reinterpret_cast<const flux_gpu_block *>(g.blocks);  // NOLINT: same layout
      raw.count = g.count;
      std::size_t n = 0;
      auto call = [&] {
        return flux_schema_inline_arena(
          s_, d, v.size(), h, heap.size(), &raw, out.data(), out.size(), &n);
      };
      flux_status s = call();
      if (s == FLUX_INVALID_ARGUMENT && n > out.size()) {
        out.resize(n);
        s = call();
      }
      detail::check(s);
      out.resize(n);
    }

    // A GPU memory channel's `aux` (a frame of this type with `data` empty) and `data` (its
    // payload, or a CPU reader's copy) as one frame of this type, into `out`. Throws Error
    // (FLUX_MISMATCH) for an aux that does not match.
    void with_data(
      wire::Span<const std::uint8_t> aux, wire::Span<const std::uint8_t> data,
      std::vector<std::uint8_t> & out) const
    {
      std::size_t n = 0;
      auto call = [&] {
        return flux_schema_with_data(
          s_, aux.data(), aux.size(), data.data(), data.size(), out.data(), out.size(), &n);
      };
      flux_status s = call();
      if (s == FLUX_INVALID_ARGUMENT && n > out.size()) {
        out.resize(n);
        s = call();
      }
      detail::check(s);
      out.resize(n);
    }
    void with_data(const FrameView & v, std::vector<std::uint8_t> & out) const
    {
      with_data(
        v.aux(), {static_cast<const std::uint8_t *>(v.data()), v.data() != nullptr ? v.size() : 0},
        out);
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
  // Whether `fingerprint` is a GPU memory channel's (gpu_fingerprint) of an installed type.
  [[nodiscard]] auto contains_gpu(std::uint64_t fingerprint) const noexcept -> bool
  {
    return flux_schemas_find_gpu(s_.get(), fingerprint) != nullptr;
  }
  // Requires contains_gpu(fingerprint): the type that channel carries.
  [[nodiscard]] auto at_gpu(std::uint64_t fingerprint) const noexcept -> Type
  {
    return Type(flux_schemas_find_gpu(s_.get(), fingerprint));
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
