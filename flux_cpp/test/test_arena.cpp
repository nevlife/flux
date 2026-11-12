// Experimental arena: fields past the slot, read through the frame's trailer. Run once plain and
// once under LD_PRELOAD=libflux_heaphook.so, which the adopting test needs.
#include "flux/publisher.hpp"
#include "flux/subscription.hpp"
#include "flux_test/flux/blob.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{

using Blob = flux_test::flux_msg::Blob;

constexpr std::uint32_t kSlot = 4096;
constexpr std::size_t kRun = 1 << 20;

auto topic(const char * name) -> std::string
{
  return "/a" + std::to_string(getpid()) + "_" + name;
}

class Arena : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("FLUX_ARENA_BYTES", "67108864", 1);  // before the first loan makes the arena
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override { node_ = std::make_shared<rclcpp::Node>("flux_arena_test"); }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(Arena, AFrameLargerThanItsSlotGoesThroughTheArena)
{
  flux::Publisher pub(*node_, topic("big"), Blob::kFingerprint, kSlot, 4);
  flux::Subscription sub(*node_, topic("big"), Blob::kFingerprint);
  auto b = Blob::build__(pub);
  b.set__name("blob");
  const auto data = b.alloc__data(kRun);
  ASSERT_EQ(data.size(), kRun);
  for (std::size_t i = 0; i < kRun; ++i) {
    data[i] = static_cast<std::uint8_t>(i * 7);
  }
  b.alloc__tags(300);
  for (std::size_t i = 0; i < 300; ++i) {
    b.set__tags(i, "tag" + std::to_string(i));
  }
  auto items = b.alloc__items(200);
  for (std::size_t i = 0; i < items.size(); ++i) {
    items[i].set__label("item" + std::to_string(i));
    items[i].alloc__ids(3)[2] = static_cast<std::uint16_t>(i);
  }
  ASSERT_TRUE(b.ok__());
  ASSERT_EQ(b.commit__(), flux::Published::Ok);

  const flux::FrameView f = sub.take();
  ASSERT_TRUE(f.valid());
  EXPECT_LT(f.size(), kSlot);
  ASSERT_FALSE(f.heap().empty());
  const Blob::View v(f);
  EXPECT_EQ(v.name(), "blob");
  ASSERT_EQ(v.data().size(), kRun);
  bool same = true;
  for (std::size_t i = 0; i < kRun; ++i) {
    same = same && v.data()[i] == static_cast<std::uint8_t>(i * 7);
  }
  EXPECT_TRUE(same);
  ASSERT_EQ(v.tags__size(), 300U);
  EXPECT_EQ(v.tags(299), "tag299");
  ASSERT_EQ(v.items__size(), 200U);
  EXPECT_EQ(v.items(199).label(), "item199");
  EXPECT_EQ(v.items(199).ids()[2], 199);
  EXPECT_TRUE(v.ok__());
}

TEST_F(Arena, AnArenaOffsetPastTheArenaIsRefused)
{
  flux::Publisher pub(*node_, topic("bad"), Blob::kFingerprint, kSlot, 4);
  flux::Subscription sub(*node_, topic("bad"), Blob::kFingerprint);
  auto b = Blob::build__(pub);
  b.alloc__data(kRun);
  ASSERT_EQ(b.commit__(), flux::Published::Ok);
  const flux::FrameView f = sub.take();
  ASSERT_TRUE(f.valid());
  // The same frame read with an arena one byte short of the run.
  const auto heap = f.heap();
  const flux::wire::Reader r(f.data(), f.size(), {heap.data(), 0});
  EXPECT_TRUE(r.span<std::uint8_t>(8).empty());
  EXPECT_TRUE(r.bad());
}

// A descriptor past 1 GiB of the arena: offsets count 16-byte units, so a reader finds the run
// at its byte position. The arena here is sparse anonymous memory, touched at one page.
TEST(ArenaWire, AFieldPastOneGibIsReadAtItsPosition)
{
  using flux::wire::Desc;
  constexpr std::size_t kSize = std::size_t{2} << 30;
  constexpr std::size_t kAt = std::size_t{3} << 29;
  void * m = mmap(
    nullptr, kSize, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
  ASSERT_NE(m, MAP_FAILED);
  auto * heap = static_cast<std::byte *>(m);
  for (std::size_t i = 0; i < 64; ++i) {
    heap[kAt + i] = static_cast<std::byte>(i);
  }
  std::uint32_t w = 0;
  ASSERT_TRUE(flux::wire::wire(flux::wire::kHeapBase + kAt, &w));
  EXPECT_EQ(w, flux::wire::kWireHeap + kAt / flux::wire::kArenaUnit);
  EXPECT_FALSE(flux::wire::wire(flux::wire::kHeapBase + kAt + 8, &w)) << "off a unit";
  const Desc d{w, 64};
  std::vector<std::byte> frame(sizeof(Desc));
  std::memcpy(frame.data(), &d, sizeof(d));
  const flux::wire::Reader r(frame.data(), frame.size(), {heap, kSize});
  const auto s = r.span<std::uint8_t>(0);
  ASSERT_EQ(s.size(), 64U);
  EXPECT_EQ(s[63], 63);
  EXPECT_FALSE(r.bad());
  munmap(m, kSize);
}

TEST_F(Arena, AdoptCopiesAContainerTheHookDidNotPlace)
{
  flux::Publisher pub(*node_, topic("copy"), Blob::kFingerprint, kSlot, 4);
  flux::Subscription sub(*node_, topic("copy"), Blob::kFingerprint);
  auto b = Blob::build__(pub);
  EXPECT_FALSE(b.adopt__data(std::vector<std::uint8_t>(kRun, 5)));
  ASSERT_EQ(b.commit__(), flux::Published::Ok);
  const flux::FrameView f = sub.take();
  const Blob::View v(f);
  ASSERT_EQ(v.data().size(), kRun);
  EXPECT_EQ(v.data()[kRun - 1], 5);
}

TEST_F(Arena, AdoptTakesAScopesBufferWithoutACopy)
{
  flux::Publisher pub(*node_, topic("adopt"), Blob::kFingerprint, kSlot, 2);
  flux::Subscription sub(*node_, topic("adopt"), Blob::kFingerprint);
  const std::uint8_t * at = nullptr;
  {
    auto b = Blob::build__(pub);
    std::vector<std::uint8_t> v;
    {
      const flux::ArenaScope scope;
      if (!scope.open()) {
        GTEST_SKIP() << "needs LD_PRELOAD=libflux_heaphook.so";
      }
      v.assign(kRun, 9);  // as a library filling its own vector would
    }
    at = v.data();
    const flux::wire::Heap h = pub.loan().heap();  // only for the arena base
    ASSERT_TRUE(b.adopt__data(std::move(v)));
    ASSERT_EQ(b.commit__(), flux::Published::Ok);
    ASSERT_NE(h.base, nullptr);
  }
  const flux::FrameView f = sub.take();
  const Blob::View v(f);
  ASSERT_EQ(v.data().size(), kRun);
  EXPECT_EQ(v.data()[0], 9);
  EXPECT_EQ(v.data()[kRun - 1], 9);
  // The subscriber's mapping is another one, at the same offset into the arena.
  const auto heap = f.heap();
  const auto * mine = static_cast<const void *>(v.data().data());
  EXPECT_NE(mine, at);
  EXPECT_GE(mine, static_cast<const void *>(heap.data()));
  EXPECT_LT(mine, static_cast<const void *>(heap.data() + heap.size()));
}

}  // namespace
