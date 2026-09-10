#include "flux/channel.hpp"
#include "flux/segment.hpp"
#include "flux/segment_layout.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

// The other trust boundary: a consumer reading a segment some other process wrote. A corrupt or
// buggy publisher can leave any bytes in the control header, the slot headers and the holder
// table, and peek()/take() must still never hand out a view that runs past the payload region
// (threat model). The sanitizer is the oracle -- this asserts nothing.
//
// What is NOT fuzzed, deliberately: slot_size/slot_count. A consumer takes those from the header
// once at attach, where map_wait_validate has already checked them against the mapping size; the
// cached SegmentLayout is what every index is computed from afterwards. Fuzzing them here would
// exercise a path production does not have.

namespace
{

constexpr std::uint32_t kSlotSize = 256;
constexpr std::uint32_t kSlotCount = 4;

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  if (size < 2) return 0;

  flux::Segment seg = flux::Segment::create_heap(kSlotSize, kSlotCount);
  const flux::SegmentLayout layout = seg.layout();
  auto * base = reinterpret_cast<std::uint8_t *>(seg.base());

  // Overwrite the control + slot-header + holder region, and as much payload as the input covers.
  const std::size_t writable = layout.total_bytes();
  std::memcpy(base, data, size < writable ? size : writable);

  // Get past the attach gate: an input that fails it exercises one early return and nothing else.
  // Everything a publisher writes per frame -- latest, seq, refcount, commit_ticket, FrameMeta,
  // holders -- stays whatever the fuzzer made it.
  auto * ctrl = reinterpret_cast<flux::ControlHeader *>(base);
  ctrl->magic = flux::kMagic;
  ctrl->version = flux::kLayoutVersion;
  ctrl->slot_size = kSlotSize;
  ctrl->slot_count = kSlotCount;
  ctrl->init_state.store(flux::kInitReady, std::memory_order_release);

  flux::Channel ch(std::move(seg));

  const std::uint8_t plan = data[0];
  for (int step = 0; step < 4; ++step) {
    flux::FrameView v = ((plan >> step) & 1u) ? ch.peek() : ch.take();
    if (!v) continue;
    // Read every byte the view claims to own. If the bound came from corrupt meta rather than
    // from the slot, this is where it runs off the end.
    const auto * p = static_cast<const std::uint8_t *>(v.data());
    std::uint8_t acc = 0;
    for (std::size_t i = 0; i < v.size(); ++i) acc ^= p[i];
    (void)acc;
    (void)v.meta();
  }
  return 0;
}
