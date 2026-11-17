#ifndef FLUX_BAG_FRAME_ENCODER_HPP
#define FLUX_BAG_FRAME_ENCODER_HPP

#include "flux/subscription.hpp"
#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/raw_meta.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace flux_bag
{

// One flux channel's frames as the FluxFrame fields that record_flux stores and play_flux reads:
// slots, descriptor, aux, and the fields that were in the publisher's arena copied in.
class FrameEncoder
{
public:
  FrameEncoder(std::string flux_topic, std::uint64_t fingerprint);
  FrameEncoder(const FrameEncoder &) = delete;
  auto operator=(const FrameEncoder &) -> FrameEncoder & = delete;
  FrameEncoder(FrameEncoder &&) = delete;
  auto operator=(FrameEncoder &&) -> FrameEncoder & = delete;
  ~FrameEncoder() = default;

  // `f` for `v` taken by `sub`, sent at `sent_ns`; it points into `v` and this until the next
  // call. Null when filled, else why not: arena fields of a type not installed or not matched.
  auto encode(
    const flux::FrameView & v, const flux::Subscription & sub, std::int64_t sent_ns,
    FluxFrameFields & f) -> const char *;

private:
  std::string flux_topic_;
  std::uint64_t fingerprint_;
  std::uint32_t generation_ = 0;
  RawMeta meta_;
  std::uint32_t aux_size_ = 0;
  std::vector<std::uint8_t> meta_bytes_;  // the raw meta, then the aux tail
  std::vector<std::uint8_t> inlined_;     // a frame with its arena fields copied in
  std::uint64_t inlined_slot_ = 0;
};

}  // namespace flux_bag

#endif  // FLUX_BAG_FRAME_ENCODER_HPP
