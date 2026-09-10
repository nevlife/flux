#include "flux/segment_layout.hpp"

#include <gtest/gtest.h>

// The ABI contract (sizes/offsets/alignment) is enforced by static_asserts in the
// header -- if this file compiles, that contract holds. These runtime cases cover
// the derived SegmentLayout offset math.

TEST(SegmentLayout, OffsetsAreAligned)
{
  flux::SegmentLayout l{/*slot_size=*/1024, /*slot_count=*/4};
  EXPECT_EQ(l.slots_offset() % alignof(flux::SlotHeader), 0u);
  EXPECT_EQ(l.payload_offset() % flux::kPayloadAlign, 0u);
  EXPECT_EQ(l.slot_stride(), sizeof(flux::SlotHeader));
}

TEST(SegmentLayout, PayloadRegionIsPageAligned)
{
  // v7: a subscriber mprotects [payload_offset, end), so the boundary and the stride are page
  // multiples -- anything finer would put subscriber-writable headers inside the protection.
  EXPECT_EQ((flux::SegmentLayout{100, 2}).payload_stride(), flux::kPayloadAlign);
  EXPECT_EQ((flux::SegmentLayout{4096, 2}).payload_stride(), 4096u);
  EXPECT_EQ((flux::SegmentLayout{4097, 2}).payload_stride(), 8192u);
  EXPECT_EQ((flux::SegmentLayout{100, 2}).payload_offset() % flux::kPayloadAlign, 0u);
}

TEST(SegmentLayout, TotalCoversAllRegions)
{
  flux::SegmentLayout l{/*slot_size=*/256, /*slot_count=*/8};
  EXPECT_GE(l.slots_offset(), sizeof(flux::ControlHeader));
  EXPECT_GE(l.payload_offset(), l.slots_offset() + l.slots_bytes());
  EXPECT_EQ(l.total_bytes(), l.payload_offset() + l.payload_stride() * 8);
}

TEST(SegmentLayout, DTypeWidths)
{
  // dtype_size is what meta_is_sane validates a publisher's itemsize against and what a consumer
  // sizes its view from, so every DType must answer with its real width. A value this build does
  // not know answers 0, which is what makes an unknown dtype a refused frame rather than a view
  // sized from a guess.
  EXPECT_EQ(flux::dtype_size(flux::DType::U8), 1u);
  EXPECT_EQ(flux::dtype_size(flux::DType::I64), 8u);
  EXPECT_EQ(flux::dtype_size(flux::DType::F16), 2u);
  EXPECT_EQ(flux::dtype_size(flux::DType::BF16), 2u);
  EXPECT_EQ(flux::dtype_size(flux::DType::F64), 8u);
  EXPECT_EQ(flux::dtype_size(static_cast<flux::DType>(200)), 0u);
}
