#ifndef FLUX_BAG_RAW_META_HPP
#define FLUX_BAG_RAW_META_HPP

#include "flux/channel.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace flux_bag
{

// What FluxFrame.meta carries for codec "raw": the channel geometry and the frame descriptor,
// so replay recreates the publisher and stamps each frame as the original did. Little-endian:
// slot_size u32, slot_count u32, dtype u8, ndim u8, shape u64[ndim].
struct RawMeta
{
  std::uint32_t slot_size = 0;
  std::uint32_t slot_count = 0;
  flux::DType dtype = flux::DType::U8;
  std::uint8_t ndim = 0;
  std::array<std::uint64_t, flux::kMaxDims> shape{};

  static constexpr std::size_t kMaxSize = 4 + 4 + 1 + 1 + 8 * flux::kMaxDims;

  [[nodiscard]] auto size() const -> std::size_t { return 10 + 8 * static_cast<std::size_t>(ndim); }

  // What dtype and shape say the frame holds; UINT64_MAX when the product overflows.
  [[nodiscard]] auto nbytes() const -> std::uint64_t
  {
    std::uint64_t n = flux::dtype_size(dtype);
    for (std::size_t i = 0; i < ndim; ++i) {
      if (__builtin_mul_overflow(n, shape[i], &n)) {
        return UINT64_MAX;
      }
    }
    return n;
  }

  auto write(std::uint8_t * out) const -> std::size_t
  {
    std::memcpy(out, &slot_size, 4);
    std::memcpy(out + 4, &slot_count, 4);
    out[8] = static_cast<std::uint8_t>(dtype);
    out[9] = ndim;
    std::memcpy(out + 10, shape.data(), 8 * static_cast<std::size_t>(ndim));
    return size();
  }

  // Reads the front of `in`, where an AuxTail may follow. The bytes it took; 0 when they are not
  // a RawMeta.
  auto read(const void * in, std::size_t n) -> std::size_t
  {
    const auto * p = static_cast<const std::uint8_t *>(in);
    if (n < 10 || p[9] > flux::kMaxDims || n < 10 + 8 * static_cast<std::size_t>(p[9])) {
      return 0;
    }
    std::memcpy(&slot_size, p, 4);
    std::memcpy(&slot_count, p + 4, 4);
    dtype = static_cast<flux::DType>(p[8]);
    ndim = p[9];
    std::memcpy(shape.data(), p + 10, 8 * static_cast<std::size_t>(ndim));
    return flux::dtype_size(dtype) != 0 ? size() : 0;
  }
};

// What FluxFrame.meta carries after the codec's meta for a channel made with an aux_size: the
// aux_size, so replay recreates the channel with it, and the frame's aux bytes. Little-endian:
// version u32, aux_size u32, aux_len u32, aux u8[aux_len]. A meta that ends with the codec's meta
// has no aux, as every bag before this tail.
struct AuxTail
{
  static constexpr std::uint32_t kVersion = 1;
  static constexpr std::size_t kHeader = 12;
  static constexpr std::size_t kMaxSize = kHeader + FLUX_MAX_AUX_SIZE;

  // 0: no tail.
  std::uint32_t aux_size = 0;
  // Into the bytes read, or the bytes to write.
  const std::uint8_t * aux = nullptr;
  std::uint32_t aux_len = 0;

  [[nodiscard]] auto size() const -> std::size_t { return aux_size == 0 ? 0 : kHeader + aux_len; }

  auto write(std::uint8_t * out) const -> std::size_t
  {
    if (aux_size == 0) {
      return 0;
    }
    std::memcpy(out, &kVersion, 4);
    std::memcpy(out + 4, &aux_size, 4);
    std::memcpy(out + 8, &aux_len, 4);
    if (aux_len != 0) {
      std::memcpy(out + kHeader, aux, aux_len);
    }
    return size();
  }

  // `in` is every byte after the codec's meta. False unless they are one tail, exactly.
  auto read(const void * in, std::size_t n) -> bool
  {
    *this = {};
    if (n == 0) {
      return true;
    }
    if (n < kHeader) {
      return false;
    }
    const auto * p = static_cast<const std::uint8_t *>(in);
    std::uint32_t version = 0;
    std::uint32_t capacity = 0;
    std::uint32_t len = 0;
    std::memcpy(&version, p, 4);
    std::memcpy(&capacity, p + 4, 4);
    std::memcpy(&len, p + 8, 4);
    if (
      version != kVersion || capacity == 0 || capacity > FLUX_MAX_AUX_SIZE || len > capacity ||
      n - kHeader != len) {
      return false;
    }
    aux_size = capacity;
    aux = p + kHeader;
    aux_len = len;
    return true;
  }
};

}  // namespace flux_bag

#endif  // FLUX_BAG_RAW_META_HPP
