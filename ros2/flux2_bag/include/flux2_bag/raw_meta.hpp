#ifndef FLUX2_BAG_RAW_META_HPP
#define FLUX2_BAG_RAW_META_HPP

#include "flux2/channel.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace flux2_bag
{

// What FluxFrame.meta carries for codec "raw": the channel geometry and the frame descriptor,
// so replay recreates the publisher and stamps each frame as the original did. Little-endian:
// slot_size u32, slot_count u32, dtype u8, ndim u8, shape u64[ndim].
struct RawMeta
{
  std::uint32_t slot_size = 0;
  std::uint32_t slot_count = 0;
  flux2::DType dtype = flux2::DType::U8;
  std::uint8_t ndim = 0;
  std::array<std::uint64_t, flux2::kMaxDims> shape{};

  static constexpr std::size_t kMaxSize = 4 + 4 + 1 + 1 + 8 * flux2::kMaxDims;

  std::size_t size() const { return 10 + 8 * static_cast<std::size_t>(ndim); }

  // What dtype and shape say the frame holds; UINT64_MAX when the product overflows.
  std::uint64_t nbytes() const
  {
    std::uint64_t n = flux2::dtype_size(dtype);
    for (std::size_t i = 0; i < ndim; ++i) {
      if (__builtin_mul_overflow(n, shape[i], &n)) return UINT64_MAX;
    }
    return n;
  }

  std::size_t write(std::uint8_t * out) const
  {
    std::memcpy(out, &slot_size, 4);
    std::memcpy(out + 4, &slot_count, 4);
    out[8] = static_cast<std::uint8_t>(dtype);
    out[9] = ndim;
    std::memcpy(out + 10, shape.data(), 8 * static_cast<std::size_t>(ndim));
    return size();
  }

  bool read(const void * in, std::size_t n)
  {
    const auto * p = static_cast<const std::uint8_t *>(in);
    if (n < 10 || p[9] > flux2::kMaxDims || n != 10 + 8 * static_cast<std::size_t>(p[9])) {
      return false;
    }
    std::memcpy(&slot_size, p, 4);
    std::memcpy(&slot_count, p + 4, 4);
    dtype = static_cast<flux2::DType>(p[8]);
    ndim = p[9];
    std::memcpy(shape.data(), p + 10, 8 * static_cast<std::size_t>(ndim));
    return flux2::dtype_size(dtype) != 0;
  }
};

}  // namespace flux2_bag

#endif  // FLUX2_BAG_RAW_META_HPP
