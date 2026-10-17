#include "flux2_tools/whole_pixel_rows.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

// rgb8, 3 pixels wide, rows padded to 4 bytes: a step of 12 is 4 whole pixels.
TEST(WholePixelRows, KeepsAStepOfWholePixels)
{
  std::vector<std::uint8_t> frame(24);
  std::vector<std::uint8_t> scratch;
  std::size_t pitch = 0;
  EXPECT_EQ(flux2_tools::whole_pixel_rows(frame.data(), 9, 2, 12, 3, scratch, pitch), frame.data());
  EXPECT_EQ(pitch, 4u);
  EXPECT_TRUE(scratch.empty());
}

// 641 rgb8 pixels in rows of 1924 bytes: a pitch of 1924 / 3 would start each row a byte early.
TEST(WholePixelRows, PacksAStepOfPartPixels)
{
  constexpr std::size_t kRow = std::size_t{641} * 3;
  constexpr std::size_t kStep = 1924;
  std::vector<std::uint8_t> frame(kStep * 2, 0);
  frame[0] = 1;
  frame[kStep] = 2;
  frame[kStep + kRow - 1] = 3;
  std::vector<std::uint8_t> scratch;
  std::size_t pitch = 0;
  const std::uint8_t * rows =
    flux2_tools::whole_pixel_rows(frame.data(), kRow, 2, kStep, 3, scratch, pitch);
  EXPECT_EQ(pitch, 641u);
  EXPECT_EQ(rows[0], 1);
  EXPECT_EQ(rows[kRow], 2);
  EXPECT_EQ(rows[2 * kRow - 1], 3);
}
