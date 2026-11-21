// flux/nvbufsurface.hpp on a Jetson: VIC reads an image slot through its surface, and a frame
// outside one through a copy.
#include "flux/channel.hpp"
#include "flux/nvbufsurface.hpp"

#include <gtest/gtest.h>
#include <nvbufsurftransform.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace
{

auto unique_name(const std::string & what) -> std::string
{
  static int n = 0;
  return flux::detail::signpost_name(
    "/nvb" + std::to_string(getpid()) + "_" + std::to_string(n++) + "_" + what);
}

// One BGRA pixel value everywhere, rows `pitch` apart: 10, 20, 30, 40, each plus `add`.
void fill_bgra(
  std::uint8_t * p, std::uint32_t w, std::uint32_t h, std::uint32_t pitch, std::uint8_t add = 0)
{
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      std::uint8_t * px = p + std::size_t{y} * pitch + std::size_t{x} * 4;
      px[0] = 10 + add;
      px[1] = 20 + add;
      px[2] = 30 + add;
      px[3] = 40 + add;
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

// fill_bgra's pixel after VIC's conversion to RGBA.
constexpr auto rgba(std::uint32_t add) -> std::uint32_t
{
  return (30U + add) | ((20U + add) << 8) | ((10U + add) << 16) | ((40U + add) << 24);
}
constexpr std::uint32_t kRgba = rgba(0);

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

// The dmabufs this process has open: the slots its channels map, and each surface's dup.
auto dmabufs_open() -> int
{
  int n = 0;
  for (const auto & e : std::filesystem::directory_iterator("/proc/self/fd")) {
    std::error_code ec;
    const auto to = std::filesystem::read_symlink(e.path(), ec).string();
    n += static_cast<int>(!ec && to.find("dmabuf") != std::string::npos);
  }
  return n;
}

// Up to 2 s for the replaced channel's mapping to go, which the reaper thread lets go.
auto settles_at(int want) -> bool
{
  for (int i = 0; i < 200 && dmabufs_open() != want; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return dmabufs_open() == want;
}

// The publisher restarts and the subscriber re-attaches: its new slots' fds may take the numbers
// of the old ones, and the cache must hand VIC the new slot, not the old one under that number.
TEST_F(NvBuf, AReattachedChannelsSlotIsNotTheOldOneOfTheSameFd)
{
  flux_publisher_config c = flux_publisher_config_default();
  c.alloc = FLUX_ALLOC_SLOT;
  c.slot_count = 2;
  c.image_encoding = FLUX_IMAGE_BGRA8;
  c.image_width = 848;
  c.image_height = 480;
  const std::string name = unique_name("again");
  auto pub = std::make_unique<flux::detail::Channel>(
    flux::detail::Channel::create_config(name, c, flux::kNoSchema));
  flux::detail::Watch watch(name);
  auto sub = flux::detail::Channel::open(watch, flux::kNoSchema, flux::QoS(2));
  flux::NvBufSurfaces surfaces;
  std::vector<int> old_fds;
  auto send = [&](std::uint8_t add) {
    flux::Loan w = pub->loan();
    ASSERT_TRUE(w);
    fill_bgra(static_cast<std::uint8_t *>(w.data()), 848, 480, w.image_layout()->pitch[0], add);
    ASSERT_EQ(w.commit(w.image_layout()->bytes), flux::Published::Ok);
  };
  for (int k = 0; k < 2; ++k) {
    send(0);
    flux::Frame v = sub.take();
    ASSERT_TRUE(v);
    old_fds.push_back(v.dmabuf_fd());
    ASSERT_EQ(vic_rgba(surfaces.surface(v), 848, 480), kRgba);
  }

  // A re-attach opens the new slots before it closes the old ones, and the reaper closes those
  // later, so the numbers come back on a restart after the old ones are closed.
  const int settled = dmabufs_open();
  bool reused = false;
  for (std::uint8_t add = 40, r = 1; add <= 160 && !reused; add += 40, ++r) {
    pub.reset();
    pub = std::make_unique<flux::detail::Channel>(
      flux::detail::Channel::create_config(name, c, flux::kNoSchema));
    std::vector<int> fds;
    for (int k = 0; k < 2; ++k) {
      send(add);
      flux::Frame v = sub.take_blocking(5'000'000'000);
      ASSERT_TRUE(v);
      const int fd = v.dmabuf_fd();
      reused = reused || std::find(old_fds.begin(), old_fds.end(), fd) != old_fds.end();
      fds.push_back(fd);
      NvBufSurface * s = surfaces.surface(v);
      ASSERT_NE(s, nullptr);
      EXPECT_EQ(vic_rgba(s, 848, 480), rgba(add)) << "fd " << fd;
    }
    old_fds.insert(old_fds.end(), fds.begin(), fds.end());
    // Each generation's slots stay held by the cache's dups alone.
    ASSERT_TRUE(settles_at(settled + 2 * r)) << settled << " then " << dmabufs_open();
  }
  // Without a reused number the test above would pass on any cache.
  EXPECT_TRUE(reused);
}

// Each restart of the publisher leaves the cache a surface per old slot, holding its dmabuf;
// clear() lets them go, and the current slots are imported again on their next frame.
TEST_F(NvBuf, ClearLetsTheSlotsOfAReplacedChannelGo)
{
  flux_publisher_config c = flux_publisher_config_default();
  c.alloc = FLUX_ALLOC_SLOT;
  c.slot_count = 2;
  c.image_encoding = FLUX_IMAGE_BGRA8;
  c.image_width = 848;
  c.image_height = 480;
  const std::string name = unique_name("clear");
  auto pub = std::make_unique<flux::detail::Channel>(
    flux::detail::Channel::create_config(name, c, flux::kNoSchema));
  flux::detail::Watch watch(name);
  auto sub = flux::detail::Channel::open(watch, flux::kNoSchema, flux::QoS(2));
  flux::NvBufSurfaces surfaces;
  // Both slots once, each frame through VIC.
  auto both = [&](std::uint8_t add) {
    for (int k = 0; k < 2; ++k) {
      flux::Loan w = pub->loan();
      ASSERT_TRUE(w);
      fill_bgra(static_cast<std::uint8_t *>(w.data()), 848, 480, w.image_layout()->pitch[0], add);
      ASSERT_EQ(w.commit(w.image_layout()->bytes), flux::Published::Ok);
      flux::Frame v = sub.take_blocking(5'000'000'000);
      ASSERT_TRUE(v);
      ASSERT_EQ(vic_rgba(surfaces.surface(v), 848, 480), rgba(add));
    }
  };
  both(0);
  const int first = dmabufs_open();
  pub.reset();
  pub = std::make_unique<flux::detail::Channel>(
    flux::detail::Channel::create_config(name, c, flux::kNoSchema));
  both(50);
  // The old slots are gone from both channels, but the cache's two dups still hold them.
  EXPECT_TRUE(settles_at(first + 2)) << first << " then " << dmabufs_open();
  surfaces.clear();
  both(100);
  EXPECT_TRUE(settles_at(first)) << first << " then " << dmabufs_open();
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
