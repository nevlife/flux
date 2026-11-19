// flux/nvbufsurface.hpp on a Jetson: VIC reads an image slot through its surface, and a frame
// outside one through a copy.
#include "flux/channel.hpp"
#include "flux/nvbufsurface.hpp"

#include <gtest/gtest.h>
#include <nvbufsurftransform.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

namespace
{

auto unique_name(const std::string & what) -> std::string
{
  static int n = 0;
  return flux::detail::signpost_name(
    "/nvb" + std::to_string(getpid()) + "_" + std::to_string(n++) + "_" + what);
}

// One BGRA pixel value everywhere, rows `pitch` apart.
void fill_bgra(std::uint8_t * p, std::uint32_t w, std::uint32_t h, std::uint32_t pitch)
{
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      std::uint8_t * px = p + std::size_t{y} * pitch + std::size_t{x} * 4;
      px[0] = 10;
      px[1] = 20;
      px[2] = 30;
      px[3] = 40;
    }
  }
}

// VIC converts `src` to RGBA at half size; the first pixel of the result.
auto vic_rgba(NvBufSurface * src, std::uint32_t w, std::uint32_t h) -> std::uint32_t
{
  NvBufSurfaceCreateParams p{};
  p.width = w / 2;
  p.height = h / 2;
  p.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
  p.layout = NVBUF_LAYOUT_PITCH;
  p.memType = NVBUF_MEM_SURFACE_ARRAY;
  NvBufSurface * dst = nullptr;
  if (NvBufSurfaceCreate(&dst, 1, &p) != 0) {
    return 0;
  }
  dst->numFilled = 1;
  NvBufSurfTransformParams t{};
  t.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
  t.transform_filter = NvBufSurfTransformInter_Nearest;
  std::uint32_t px = 0;
  if (NvBufSurfTransform(src, dst, &t) == 0 && NvBufSurfaceMap(dst, 0, 0, NVBUF_MAP_READ) == 0) {
    std::memcpy(&px, dst->surfaceList[0].mappedAddr.addr[0], 4);
    NvBufSurfaceUnMap(dst, 0, 0);
  }
  NvBufSurfaceDestroy(dst);
  return px;
}

constexpr std::uint32_t kRgba = 30U | (20U << 8) | (10U << 16) | (40U << 24);

class NvBuf : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!flux::gpu::probe().integrated) {
      GTEST_SKIP() << "image slots need an integrated GPU";
    }
    NvBufSurfTransformConfigParams c{};
    c.compute_mode = NvBufSurfTransformCompute_VIC;
    NvBufSurfTransformSetSessionParams(&c);
  }
};

TEST_F(NvBuf, VicReadsAnImageSlotThroughItsSurfaceWithoutACopy)
{
  flux_publisher_config c = flux_publisher_config_default();
  c.alloc = FLUX_ALLOC_SLOT;
  c.slot_count = 4;
  c.image_encoding = FLUX_IMAGE_BGRA8;
  c.image_width = 848;
  c.image_height = 480;
  const std::string name = unique_name("slot");
  auto pub = flux::detail::Channel::create_config(name, c, flux::kNoSchema);
  flux::QoS q(4);
  q.max_borrow(2);
  flux::detail::Watch watch(name);
  auto sub = flux::detail::Channel::open(watch, flux::kNoSchema, q);
  flux::NvBufSurfaces surfaces;
  NvBufSurface * first = nullptr;
  for (int k = 0; k < 2; ++k) {
    flux::Loan w = pub.loan();
    ASSERT_TRUE(w);
    const auto l = w.image_layout();
    ASSERT_TRUE(l.has_value());
    fill_bgra(static_cast<std::uint8_t *>(w.data()), 848, 480, l->pitch[0]);
    ASSERT_EQ(w.commit(l->bytes), flux::Published::Ok);
    flux::Frame v = sub.take();
    ASSERT_TRUE(v);
    NvBufSurface * s = surfaces.surface(v);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->surfaceList[0].pitch, 3584U);
    EXPECT_EQ(vic_rgba(s, 848, 480), kRgba);
    EXPECT_EQ(surfaces.surface(v), s) << "a slot is imported once";
    if (k == 0) {
      first = s;
    } else {
      EXPECT_NE(s, first) << "the next frame is in the next slot";
    }
  }
  EXPECT_EQ(surfaces.copies(), 0U);
}

// Two caches import the same slot: two surfaces, both read right, both released.
TEST_F(NvBuf, TwoCachesOfOneSlotBothHandItToVic)
{
  flux_publisher_config c = flux_publisher_config_default();
  c.alloc = FLUX_ALLOC_SLOT;
  c.slot_count = 4;
  c.image_encoding = FLUX_IMAGE_BGRA8;
  c.image_width = 848;
  c.image_height = 480;
  const std::string name = unique_name("two");
  auto pub = flux::detail::Channel::create_config(name, c, flux::kNoSchema);
  flux::detail::Watch watch(name);
  auto sub = flux::detail::Channel::open(watch, flux::kNoSchema, flux::QoS(4));
  flux::Loan w = pub.loan();
  ASSERT_TRUE(w);
  fill_bgra(static_cast<std::uint8_t *>(w.data()), 848, 480, w.image_layout()->pitch[0]);
  auto mine = std::make_unique<flux::NvBufSurfaces>();
  NvBufSurface * from_loan = mine->surface(w);
  ASSERT_NE(from_loan, nullptr);
  ASSERT_EQ(w.commit(w.image_layout()->bytes), flux::Published::Ok);
  flux::Frame v = sub.take();
  ASSERT_TRUE(v);
  auto theirs = std::make_unique<flux::NvBufSurfaces>();
  NvBufSurface * from_view = theirs->surface(v);
  ASSERT_NE(from_view, nullptr);
  EXPECT_NE(from_view, from_loan);
  EXPECT_EQ(vic_rgba(from_loan, 848, 480), kRgba);
  EXPECT_EQ(vic_rgba(from_view, 848, 480), kRgba);
  mine.reset();
  EXPECT_EQ(vic_rgba(from_view, 848, 480), kRgba) << "the other import outlives the first";
}

TEST_F(NvBuf, AFrameOutsideAnImageSlotIsCopiedForVic)
{
  flux_publisher_config c = flux_publisher_config_default();
  c.alloc = FLUX_ALLOC_SLOT;
  c.slot_count = 4;
  const std::string name = unique_name("bytes");
  auto pub = flux::detail::Channel::create_config(name, c, flux::kNoSchema);
  flux::detail::Watch watch(name);
  auto sub = flux::detail::Channel::open(watch, flux::kNoSchema, flux::QoS(4));
  flux::Loan w = pub.loan();
  ASSERT_TRUE(w);
  fill_bgra(static_cast<std::uint8_t *>(w.data()), 640, 480, 640 * 4);
  ASSERT_EQ(w.commit(std::size_t{640} * 4 * 480), flux::Published::Ok);
  flux::Frame v = sub.take();
  ASSERT_TRUE(v);
  flux::NvBufSurfaces surfaces;
  EXPECT_EQ(surfaces.surface(v), nullptr) << "a byte frame with no hint is nothing";
  NvBufSurface * s = surfaces.surface(v, {flux::ImageEncoding::Bgra8, 640, 480, 640 * 4});
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(surfaces.copies(), 1U);
  EXPECT_EQ(vic_rgba(s, 640, 480), kRgba);
  EXPECT_EQ(surfaces.surface(v, {flux::ImageEncoding::Bgra8, 640, 480, 640 * 4}), s);
  EXPECT_EQ(surfaces.copies(), 2U);
}

}  // namespace
