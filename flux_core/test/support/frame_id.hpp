#ifndef FLUX_TEST_SUPPORT_FRAME_ID_HPP
#define FLUX_TEST_SUPPORT_FRAME_ID_HPP

// A frame whose identity is its payload: every byte is the same id, so a torn or stale read is
// visible without carrying a descriptor alongside. Six test files defined this identically; one
// definition is one place for the convention to change.

#include "flux/channel.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace flux::test
{

inline flux::Published publish_id(
  flux::Channel & ch, void * data, std::size_t n, std::uint8_t id) noexcept
{
  std::memset(data, id, n);
  return ch.publish(data, n);
}

inline std::uint8_t frame_id(const flux::FrameView & v) noexcept
{
  return v.size() == 0 ? 0u : *static_cast<const std::uint8_t *>(v.data());
}

}  // namespace flux::test

#endif  // FLUX_TEST_SUPPORT_FRAME_ID_HPP
