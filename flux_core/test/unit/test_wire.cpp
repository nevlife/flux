#include "flux/wire.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

// wire.hpp exists because a frame is written by another process: Reader must reject any
// descriptor that points outside the frame or breaks alignment (latching bad() instead of
// handing out a pointer), and Writer must never leave a stale descriptor behind.

namespace
{

using flux::wire::Desc;
using flux::wire::Reader;
using flux::wire::Writer;

constexpr std::size_t kWidthOff = 0;   // u32, inline
constexpr std::size_t kDataOff = 8;    // desc: u32[]
constexpr std::size_t kLabelOff = 16;  // desc: string
constexpr std::size_t kNamesOff = 24;  // desc: string[]
constexpr std::size_t kItemsOff = 32;  // desc: element blocks [str desc | u32]
constexpr std::size_t kScalarBytes = 40;
constexpr std::size_t kElemStride = 12;

std::vector<std::byte> dirty_buf(std::size_t n)
{
  return std::vector<std::byte>(n, std::byte{0xFF});
}

void put_desc(
  std::vector<std::byte> & buf, std::size_t off, std::uint32_t d_off, std::uint32_t d_len)
{
  const Desc d{d_off, d_len};
  std::memcpy(buf.data() + off, &d, sizeof(d));
}

}  // namespace

TEST(Wire, WriterReaderRoundTrip)
{
  auto buf = dirty_buf(4096);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  ASSERT_FALSE(w.bad());

  w.put<std::uint32_t>(kWidthOff, 640);
  auto data = w.alloc<std::uint32_t>(kDataOff, 3);
  ASSERT_EQ(data.size(), 3u);
  data[0] = 10;
  data[1] = 20;
  data[2] = 30;
  ASSERT_TRUE(w.put_str(kLabelOff, "front"));
  ASSERT_TRUE(w.alloc_strs(kNamesOff, 2));
  ASSERT_TRUE(w.put_str_at(kNamesOff, 0, "ab"));
  ASSERT_TRUE(w.put_str_at(kNamesOff, 1, "cde"));
  const std::size_t e0 = w.alloc_elems(kItemsOff, 2, kElemStride, alignof(Desc));
  ASSERT_NE(e0, 0u);
  ASSERT_TRUE(w.put_str(e0 + 0, "x"));
  w.put<std::uint32_t>(e0 + 8, 7);
  ASSERT_TRUE(w.put_str(e0 + kElemStride + 0, "yz"));
  w.put<std::uint32_t>(e0 + kElemStride + 8, 9);
  ASSERT_FALSE(w.bad());

  Reader r(buf.data(), w.size());
  EXPECT_EQ(r.get<std::uint32_t>(kWidthOff), 640u);
  auto rd = r.span<std::uint32_t>(kDataOff);
  ASSERT_EQ(rd.size(), 3u);
  EXPECT_EQ(rd[0], 10u);
  EXPECT_EQ(rd[2], 30u);
  EXPECT_EQ(r.str(kLabelOff), "front");
  EXPECT_EQ(r.len(kNamesOff), 2u);
  EXPECT_EQ(r.str_at(kNamesOff, 0), "ab");
  EXPECT_EQ(r.str_at(kNamesOff, 1), "cde");
  const std::size_t re0 = r.elem(kItemsOff, 0, kElemStride, alignof(Desc));
  const std::size_t re1 = r.elem(kItemsOff, 1, kElemStride, alignof(Desc));
  EXPECT_EQ(r.str(re0 + 0), "x");
  EXPECT_EQ(r.get<std::uint32_t>(re0 + 8), 7u);
  EXPECT_EQ(r.str(re1 + 0), "yz");
  EXPECT_EQ(r.get<std::uint32_t>(re1 + 8), 9u);
  EXPECT_FALSE(r.bad());
}

TEST(Wire, WriterZeroesTheScalarBlockOnly)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  for (std::size_t i = 0; i < kScalarBytes; ++i) {
    EXPECT_EQ(buf[i], std::byte{0}) << i;
  }
  EXPECT_EQ(buf[kScalarBytes], std::byte{0xFF});
}

TEST(Wire, AllocLeavesContentsAllocElemsZeroes)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  auto raw = w.alloc<std::uint8_t>(kDataOff, 4);
  ASSERT_EQ(raw.size(), 4u);
  EXPECT_EQ(raw[0], 0xFFu);  // documented: alloc does not zero, the caller overwrites

  const std::size_t e0 = w.alloc_elems(kItemsOff, 2, kElemStride, alignof(Desc));
  ASSERT_NE(e0, 0u);
  Reader r(buf.data(), w.size());
  EXPECT_EQ(r.str(e0 + 0), "");  // element descs zeroed: unset field reads empty, not stale
  EXPECT_FALSE(r.bad());
}

TEST(Wire, UnsetDescriptorReadsEmptyWithoutBad)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  Reader r(buf.data(), w.size());
  EXPECT_TRUE(r.span<std::uint32_t>(kDataOff).empty());
  EXPECT_EQ(r.str(kLabelOff), "");
  EXPECT_FALSE(r.bad());
}

TEST(Wire, FailedAllocWritesNoDescriptor)
{
  auto buf = dirty_buf(128);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  EXPECT_TRUE(w.alloc<std::uint64_t>(kDataOff, 1000).empty());
  EXPECT_TRUE(w.bad());
  Desc d{};
  std::memcpy(&d, buf.data() + kDataOff, sizeof(d));
  EXPECT_EQ(d.off, 0u);
  EXPECT_EQ(d.len, 0u);
}

TEST(Wire, WriterRejectsNullShortAndOutOfCapacity)
{
  EXPECT_TRUE(Writer(nullptr, 256, kScalarBytes).bad());
  auto buf = dirty_buf(16);
  EXPECT_TRUE(Writer(buf.data(), buf.size(), kScalarBytes).bad());

  auto ok = dirty_buf(256);
  Writer w(ok.data(), ok.size(), kScalarBytes);
  w.put<std::uint64_t>(ok.size() - 4, 1);
  EXPECT_TRUE(w.bad());
}

TEST(Wire, PutStrAtRejectsOutOfRangeIndex)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  ASSERT_TRUE(w.alloc_strs(kNamesOff, 2));
  EXPECT_FALSE(w.put_str_at(kNamesOff, 2, "x"));
  EXPECT_TRUE(w.bad());
}

TEST(Wire, VarAllocationsAreAligned)
{
  auto buf = dirty_buf(4096);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  ASSERT_TRUE(w.put_str(kLabelOff, "abc"));  // 3 bytes: leaves the cursor unaligned
  auto data = w.alloc<std::uint64_t>(kDataOff, 2);
  ASSERT_EQ(data.size(), 2u);
  data[0] = 1;
  data[1] = 2;

  Reader r(buf.data(), w.size());
  auto rd = r.span<std::uint64_t>(kDataOff);  // span() re-checks the alignment
  ASSERT_EQ(rd.size(), 2u);
  EXPECT_EQ(rd[1], 2u);
  EXPECT_FALSE(r.bad());
}

TEST(Wire, ReaderRejectsNullBase)
{
  Reader r(nullptr, 256);
  EXPECT_TRUE(r.bad());
  EXPECT_TRUE(r.span<std::uint32_t>(kDataOff).empty());
}

TEST(Wire, BadLatchesAndPoisonsLaterReads)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  w.put<std::uint32_t>(kWidthOff, 640);
  Reader r(buf.data(), w.size());
  EXPECT_EQ(r.get<std::uint32_t>(w.size()), 0u);  // out of bounds: latches
  EXPECT_TRUE(r.bad());
  EXPECT_EQ(r.get<std::uint32_t>(kWidthOff), 0u);  // a valid offset now reads empty too
}

TEST(Wire, ReaderRejectsDescriptorPastTheFrame)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  put_desc(buf, kDataOff, static_cast<std::uint32_t>(w.size()) + 8, 1);
  Reader r(buf.data(), w.size());
  EXPECT_TRUE(r.span<std::uint32_t>(kDataOff).empty());
  EXPECT_TRUE(r.bad());
}

TEST(Wire, ReaderRejectsOverflowingLength)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  put_desc(buf, kDataOff, kScalarBytes, 0xFFFFFFFFu);
  Reader r(buf.data(), w.size());
  EXPECT_TRUE(r.span<std::uint32_t>(kDataOff).empty());
  EXPECT_TRUE(r.bad());

  auto buf2 = dirty_buf(256);
  Writer w2(buf2.data(), buf2.size(), kScalarBytes);
  put_desc(buf2, kLabelOff, kScalarBytes, 0xFFFFFFFFu);
  Reader r2(buf2.data(), w2.size());
  EXPECT_EQ(r2.str(kLabelOff), "");
  EXPECT_TRUE(r2.bad());
}

TEST(Wire, ReaderRejectsMisalignedDescriptor)
{
  auto buf = dirty_buf(256);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  put_desc(buf, kDataOff, kScalarBytes + 2, 1);  // in bounds, but +2 is misaligned for u32
  Reader r(buf.data(), buf.size());
  EXPECT_TRUE(r.span<std::uint32_t>(kDataOff).empty());
  EXPECT_TRUE(r.bad());
}

TEST(Wire, ReaderRejectsMisalignedBlock)
{
  auto buf = dirty_buf(256);
  std::memset(buf.data(), 0, buf.size());
  Reader r(buf.data(), buf.size());
  EXPECT_TRUE(r.block<std::uint32_t>(2, 1).empty());
  EXPECT_TRUE(r.bad());
}

TEST(Wire, TruncatedFrameDoesNotReadPastItsEnd)
{
  auto buf = dirty_buf(4096);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  auto data = w.alloc<std::uint32_t>(kDataOff, 8);
  ASSERT_EQ(data.size(), 8u);
  ASSERT_TRUE(w.put_str(kLabelOff, "front"));
  ASSERT_FALSE(w.bad());

  Reader r(buf.data(), kScalarBytes);  // the var region was cut off
  EXPECT_EQ(r.get<std::uint32_t>(kWidthOff), 0u);
  EXPECT_TRUE(r.span<std::uint32_t>(kDataOff).empty());
  EXPECT_TRUE(r.bad());
}

TEST(Wire, StrAtAndElemRejectOutOfRangeIndex)
{
  auto buf = dirty_buf(4096);
  Writer w(buf.data(), buf.size(), kScalarBytes);
  ASSERT_TRUE(w.alloc_strs(kNamesOff, 1));
  ASSERT_TRUE(w.put_str_at(kNamesOff, 0, "a"));
  const std::size_t e0 = w.alloc_elems(kItemsOff, 1, kElemStride, alignof(Desc));
  ASSERT_NE(e0, 0u);

  Reader r(buf.data(), w.size());
  EXPECT_EQ(r.str_at(kNamesOff, 1), "");
  EXPECT_TRUE(r.bad());

  Reader r2(buf.data(), w.size());
  EXPECT_EQ(r2.elem(kItemsOff, 1, kElemStride, alignof(Desc)), 0u);
  EXPECT_TRUE(r2.bad());
}
