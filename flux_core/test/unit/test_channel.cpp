#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "support/frame_id.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using flux::test::frame_id;
using flux::test::publish_id;

// A frame's identity is its payload: the descriptor is derived from the byte count, so an id in
// shape[0] would have to contradict nbytes to be an id at all. The whole buffer carries the tag,
// which is also what makes an overwrite of a held frame visible.

}  // namespace

TEST(Channel, PublishPeekRoundTrip)
{
  flux::Channel ch(128, 4);
  std::vector<std::byte> buf(128, std::byte{0x5A});
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 0x5A), flux::Published::Ok);

  flux::FrameView v = ch.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.size(), 128u);
  EXPECT_EQ(frame_id(v), 0x5Au);
}

TEST(Channel, PeekBeforePublishIsEmpty)
{
  flux::Channel ch(64, 2);
  EXPECT_FALSE(ch.peek());
}

TEST(Channel, LatestWins)
{
  flux::Channel ch(64, 4);
  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 3; ++f) {
    std::memset(buf.data(), f, buf.size());
    publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f));
  }
  flux::FrameView v = ch.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(frame_id(v), 3u);
  EXPECT_EQ(static_cast<const std::uint8_t *>(v.data())[0], 3u);
}

// Oversized payloads are rejected, not truncated, so meta.nbytes can never disagree with the
// bytes actually stored. The drop counter does not move: a payload larger than the slot can never
// succeed however long the caller waits, which is a wiring mistake and not backpressure.
TEST(Channel, OversizedPublishRejected)
{
  flux::Channel ch(64, 4);
  std::vector<std::byte> big(128, std::byte{0xCC});  // 128 > slot_size 64
  EXPECT_EQ(publish_id(ch, big.data(), big.size(), 0xCC), flux::Published::TooLarge);
  EXPECT_EQ(ch.dropped(), 0u);
  EXPECT_FALSE(ch.peek());  // nothing was published

  std::vector<std::byte> fit(64, std::byte{0xAB});  // exactly slot_size is fine
  ASSERT_EQ(publish_id(ch, fit.data(), fit.size(), 0xAB), flux::Published::Ok);
  flux::FrameView v = ch.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.size(), 64u);
}

// active => byte-locked, at the API level: a held view is never overwritten even as
// the publisher keeps publishing.
TEST(Channel, HeldFrameNotOverwritten)
{
  flux::Channel ch(64, 2);
  std::vector<std::byte> buf(64);
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 0xAA), flux::Published::Ok);

  flux::FrameView v = ch.peek();
  ASSERT_TRUE(v);
  ASSERT_EQ(frame_id(v), 0xAAu);

  for (int f = 2; f <= 20; ++f) {
    publish_id(ch, buf.data(), buf.size(), static_cast<std::uint8_t>(f));
  }

  const auto * p = static_cast<const std::uint8_t *>(v.data());
  for (std::size_t i = 0; i < v.size(); ++i) EXPECT_EQ(p[i], 0xAAu);
  EXPECT_EQ(frame_id(v), 0xAAu);
}

TEST(Channel, BorrowRefcountBalances)
{
  flux::Channel ch(64, 2);
  std::vector<std::byte> buf(64, std::byte{7});
  publish_id(ch, buf.data(), buf.size(), 1);
  {
    flux::FrameView v = ch.peek();
    ASSERT_TRUE(v);
    int ones = 0;
    for (std::uint32_t i = 0; i < ch.slot_count(); ++i)
      if (ch.slot_refcount(i) == 1) ++ones;
    EXPECT_EQ(ones, 1);
  }
  for (std::uint32_t i = 0; i < ch.slot_count(); ++i) EXPECT_EQ(ch.slot_refcount(i), 0u);
}

// max_borrow caps how many views one subscriber holds at once (docs/en/qos.en.md, default 2).
TEST(Channel, MaxBorrowCapsConcurrentViews)
{
  flux::Channel ch(64, 4);
  std::vector<std::byte> buf(64, std::byte{7});
  publish_id(ch, buf.data(), buf.size(), 1);

  flux::FrameView a = ch.peek();
  flux::FrameView b = ch.peek();
  ASSERT_TRUE(a);
  ASSERT_TRUE(b);
  EXPECT_FALSE(ch.peek());  // 3rd concurrent view exceeds max_borrow (2)

  a.release();
  EXPECT_TRUE(ch.peek());  // a lease freed -> peek succeeds again
}

TEST(Channel, ExhaustedMaxBorrowIsCountedNotSilent)
{
  // An empty view from a held lease looks exactly like an idle stream to the caller, and neither
  // lost() nor dropped() records it -- refused() is the only place it appears.
  flux::Channel ch(64, 4);
  std::vector<std::byte> buf(64, std::byte{7});
  publish_id(ch, buf.data(), buf.size(), 1);

  flux::FrameView a = ch.peek();
  flux::FrameView b = ch.peek();
  ASSERT_TRUE(a);
  ASSERT_TRUE(b);
  EXPECT_EQ(ch.refused().total(), 0u);
  EXPECT_TRUE(ch.can_borrow() == false);

  EXPECT_FALSE(ch.peek());
  EXPECT_FALSE(ch.take());
  EXPECT_EQ(ch.refused().max_borrow, 2u);
  EXPECT_EQ(ch.refused().total(), 2u);
  EXPECT_EQ(ch.lost(), 0u);  // nothing was missed: the frames are in this consumer's hands

  a.release();
  EXPECT_TRUE(ch.can_borrow());
  EXPECT_TRUE(ch.peek());
  EXPECT_EQ(ch.refused().max_borrow, 2u);  // cumulative, not a level
}

TEST(Channel, AnEmptyStreamIsNotARefusal)
{
  // The counters name why a view was withheld. "No frame yet" and "caught up" are the empty view
  // doing its job, and counting them would bury the refusals in noise.
  flux::Channel ch(64, 4);
  EXPECT_FALSE(ch.peek());
  EXPECT_FALSE(ch.take());

  std::vector<std::byte> buf(64, std::byte{3});
  publish_id(ch, buf.data(), buf.size(), 1);
  {
    flux::FrameView v = ch.take();
    ASSERT_TRUE(v);
  }
  EXPECT_FALSE(ch.take());  // caught up

  EXPECT_EQ(ch.refused().total(), 0u);
}

TEST(Channel, ViewIsMoveOnly)
{
  flux::Channel ch(64, 2);
  std::vector<std::byte> buf(64);
  publish_id(ch, buf.data(), buf.size(), 9);

  flux::FrameView a = ch.peek();
  ASSERT_TRUE(a);
  flux::FrameView b = std::move(a);
  EXPECT_FALSE(a);
  ASSERT_TRUE(b);
  EXPECT_EQ(static_cast<const std::uint8_t *>(b.data())[0], 9u);
}

// Runtime analog of the model check: one publisher, many subscribers. Each frame's
// payload is a single repeated byte (frame id mod 256); a coherent borrow observes
// one uniform value. A byte-lock or seqlock defect would surface as a torn read.
//
// A Channel per thread over one shm segment, rather than one shared Channel. A Channel's cursor
// and re-attach state are single-threaded (channel.hpp), so sharing one would race on those
// instead of exercising the borrow protocol -- and the races it raised were what forced the
// blanket file-level entries this suite's tsan.supp used to carry.
TEST(Channel, ConcurrentCoherence)
{
  constexpr std::uint32_t kSlotSize = 4096;
  constexpr std::uint32_t kSlots = 4;
  constexpr int kSubs = 3;
  constexpr std::uint64_t kFp = 0xC0DE;

  std::uint64_t frames = 200000;
  if (const char * e = std::getenv("FLUX_STRESS_FRAMES")) frames = std::strtoull(e, nullptr, 10);

  const std::string name =
    flux::segment_name("/flux_test/coherence." + std::to_string(::getpid()), kFp);
  flux::Channel ch = flux::Channel::create(name, kSlotSize, kSlots, kFp);

  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> torn{0};
  std::atomic<std::uint64_t> borrows{0};
  std::atomic<int> ready{0};
  std::atomic<int> attach_failed{0};

  auto sub = [&] {
    std::optional<flux::Channel> mine;
    try {
      mine.emplace(flux::Channel::open(name, kFp));
    } catch (...) {
      attach_failed.fetch_add(1, std::memory_order_relaxed);
      ready.fetch_add(1, std::memory_order_release);
      return;
    }
    ready.fetch_add(1, std::memory_order_release);
    while (!stop.load(std::memory_order_relaxed)) {
      flux::FrameView v = mine->peek();
      if (!v) continue;
      borrows.fetch_add(1, std::memory_order_relaxed);
      const auto * p = static_cast<const std::uint8_t *>(v.data());
      const std::uint8_t id = p[0];
      for (std::size_t i = 0; i < v.size(); ++i) {
        if (p[i] != id) {
          torn.fetch_add(1, std::memory_order_relaxed);
          break;
        }
      }
    }
  };

  std::vector<std::thread> subs;
  subs.reserve(kSubs);
  for (int i = 0; i < kSubs; ++i) subs.emplace_back(sub);
  // Publish only once every subscriber is attached: a subscriber that joined late would still
  // borrow, but the run could end with borrows == 0 on a slow machine.
  while (ready.load(std::memory_order_acquire) < kSubs) {
  }

  std::vector<std::byte> payload(kSlotSize);
  for (std::uint64_t f = 1; f <= frames; ++f) {
    std::memset(payload.data(), static_cast<int>(f & 0xFF), payload.size());
    publish_id(ch, payload.data(), payload.size(), f);
  }
  stop.store(true, std::memory_order_relaxed);
  for (auto & t : subs) t.join();

  EXPECT_EQ(attach_failed.load(), 0);
  EXPECT_EQ(torn.load(), 0u);
  EXPECT_GT(borrows.load(), 0u);
}

TEST(Channel, LoanCommitZeroCopyRoundTrip)
{
  flux::Channel ch(128, 4);
  flux::WriteSlot ws = ch.loan();
  ASSERT_TRUE(ws);
  ASSERT_GE(ws.capacity(), 128u);
  std::memset(ws.data(), 0x5A, 128);
  ASSERT_EQ(ws.commit(128), flux::Published::Ok);
  EXPECT_FALSE(ws);  // consumed

  flux::FrameView v = ch.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.size(), 128u);
  EXPECT_EQ(frame_id(v), 0x5Au);
}

TEST(Channel, LoanAbortDoesNotPublishOrResurrectTheOverwrittenFrame)
{
  // loan() hands out a slot that may still hold a frame nobody has consumed. data() aliases
  // those bytes, so writing into the loan destroys that frame -- abort cannot bring it back.
  // What abort must guarantee is that the destroyed frame is never delivered: before this,
  // the slot kept advertising the old ticket and a lagging consumer got its meta over the
  // new bytes.
  flux::Channel ch(128, 4);
  ch.qos([] {
    flux::QoS q;
    q.depth = 8;  // lag behind, so the frames below are still pending when the loan lands
    return q;
  }());

  for (std::uint8_t i = 1; i <= 4; ++i) {  // fill the ring: slots 0..3 hold tickets 1..4
    std::vector<std::byte> buf(128, static_cast<std::byte>(i));
    ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), i), flux::Published::Ok);
  }
  {
    flux::WriteSlot ws = ch.loan();  // lands on a slot holding a pending frame
    ASSERT_TRUE(ws);
    std::memset(ws.data(), 0xEE, 128);
    ws.abort();
  }

  std::vector<std::uint8_t> got;
  while (flux::FrameView v = ch.take()) {
    got.push_back(static_cast<const std::uint8_t *>(v.data())[0]);
    EXPECT_EQ(frame_id(v), got.back()) << "frame meta does not match its payload";
  }
  EXPECT_EQ(std::count(got.begin(), got.end(), 0xEEu), 0)
    << "an aborted loan was delivered as a committed frame";
  EXPECT_FALSE(got.empty()) << "abort took the whole stream with it";
  for (std::uint8_t b : got) {
    EXPECT_GE(b, 1u);
    EXPECT_LE(b, 4u);
  }
}

TEST(Channel, LoanInProgressNotVisibleUntilCommit)
{
  flux::Channel ch(128, 4);
  ASSERT_EQ(publish_id(ch, std::vector<std::byte>(128).data(), 128, 0x11), flux::Published::Ok);

  flux::WriteSlot ws = ch.loan();  // claimed but not committed
  ASSERT_TRUE(ws);
  std::memset(ws.data(), 0x22, 128);

  flux::FrameView v = ch.peek();  // must still see the committed frame, not the in-progress one
  ASSERT_TRUE(v);
  EXPECT_EQ(frame_id(v), 0x11u);

  v.release();
  ASSERT_EQ(ws.commit(128), flux::Published::Ok);
  flux::FrameView v2 = ch.peek();
  ASSERT_TRUE(v2);
  EXPECT_EQ(frame_id(v2), 0x22u);
}

TEST(Channel, LoanDropsWhenAllSlotsBorrowed)
{
  flux::Channel ch(128, 1);
  ASSERT_EQ(
    publish_id(ch, std::vector<std::byte>(128, std::byte{0x7}).data(), 128, 1),
    flux::Published::Ok);
  flux::FrameView v = ch.peek();  // borrows the only slot
  ASSERT_TRUE(v);

  flux::WriteSlot ws = ch.loan();
  EXPECT_FALSE(ws);  // no free slot
  EXPECT_GE(ch.dropped(), 1u);
}

// peek() reads current state and does not consume, so it keeps handing out the newest frame.
// take() consumes, so a frame arrives once (docs/en/qos.en.md 1).
TEST(Channel, PeekRepeatsAndTakeConsumes)
{
  flux::Channel ch(64, 4);
  std::vector<std::byte> buf(64, std::byte{7});
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 1), flux::Published::Ok);

  {
    flux::FrameView a = ch.peek();
    ASSERT_TRUE(a);
    EXPECT_EQ(frame_id(a), 1u);
  }
  {
    flux::FrameView b = ch.peek();  // nothing new published: the same frame again
    ASSERT_TRUE(b);
    EXPECT_EQ(frame_id(b), 1u);
  }
  {
    flux::FrameView c = ch.take();
    ASSERT_TRUE(c);
    EXPECT_EQ(frame_id(c), 1u);
  }
  EXPECT_FALSE(ch.take());  // consumed
  EXPECT_TRUE(ch.peek());   // still readable as state
}

// depth is the lag bound. The default 1 pins the cursor one frame back, so take() delivers the
// newest frame and reports what it skipped getting there.
TEST(Channel, DepthOneDeliversNewestOnly)
{
  flux::Channel ch(64, 8);
  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 5; ++f) {
    std::memset(buf.data(), f, buf.size());
    ASSERT_EQ(
      publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f)), flux::Published::Ok);
  }
  EXPECT_EQ(ch.qos().depth, 1u);

  {
    flux::FrameView v = ch.take();
    ASSERT_TRUE(v);
    EXPECT_EQ(frame_id(v), 5u);
    EXPECT_EQ(ch.lost(), 4u);  // 1..4 dropped by the depth window
  }
  EXPECT_FALSE(ch.take());

  std::memset(buf.data(), 6, buf.size());
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 6), flux::Published::Ok);
  flux::FrameView v6 = ch.take();
  ASSERT_TRUE(v6);
  EXPECT_EQ(frame_id(v6), 6u);
  EXPECT_EQ(ch.lost(), 4u);  // cumulative: nothing further was skipped
}

// depth N lets a consumer sit N frames behind, so a burst is consumed in publish order.
TEST(Channel, DepthNCatchesUpInOrder)
{
  flux::Channel ch(64, 8);
  flux::QoS q;
  q.depth = 8;
  ch.qos(q);

  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 5; ++f) {
    std::memset(buf.data(), f, buf.size());
    ASSERT_EQ(
      publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f)), flux::Published::Ok);
  }

  for (std::uint64_t expect = 1; expect <= 5; ++expect) {
    flux::FrameView v = ch.take();
    ASSERT_TRUE(v) << "expected frame " << expect;
    EXPECT_EQ(frame_id(v), expect);
    EXPECT_EQ(static_cast<const std::uint8_t *>(v.data())[0], static_cast<std::uint8_t>(expect));
    EXPECT_EQ(ch.lost(), 0u);
    EXPECT_EQ(ch.cursor(), expect);
  }
  EXPECT_FALSE(ch.take());
}

// Falling further behind than depth pulls the cursor forward; the skipped frames land in lost().
TEST(Channel, DepthPullsCursorForwardAndReportsLost)
{
  flux::Channel ch(64, 8);
  flux::QoS q;
  q.depth = 2;
  ch.qos(q);

  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 5; ++f) {
    std::memset(buf.data(), f, buf.size());
    ASSERT_EQ(
      publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f)), flux::Published::Ok);
  }

  {
    flux::FrameView v = ch.take();  // window is [4, 5]
    ASSERT_TRUE(v);
    EXPECT_EQ(frame_id(v), 4u);
    EXPECT_EQ(ch.lost(), 3u);  // 1, 2, 3
  }
  {
    flux::FrameView v = ch.take();
    ASSERT_TRUE(v);
    EXPECT_EQ(frame_id(v), 5u);
    EXPECT_EQ(ch.lost(), 3u);  // cumulative
  }
  EXPECT_FALSE(ch.take());
}

// The ring is the hard cap: a depth deeper than slot_count is not an error, it is just not
// achievable, and the shortfall shows up in lost() (docs/en/qos.en.md 4).
TEST(Channel, RingCapsDepth)
{
  flux::Channel ch(64, 2);  // ring depth 2
  flux::QoS q;
  q.depth = 8;
  ch.qos(q);

  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 5; ++f) {
    std::memset(buf.data(), f, buf.size());
    ASSERT_EQ(
      publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f)), flux::Published::Ok);
  }

  flux::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(frame_id(v), 4u);  // only 4 and 5 are still retained
  EXPECT_EQ(ch.lost(), 3u);
}

// What a lapped ring holds is the newest slot_count frames, with no holes. This is the property
// `depth` and TransientLocal(n) are defined against -- a ring that retained an arbitrary subset
// would cap both below its own slot_count without saying so. It comes from where publish() starts
// looking for a free slot: just past `latest`. A per-publisher scan
// offset measurably lowers claim contention and breaks exactly this, because a publisher with a
// fixed stride revisits only part of the ring.
//
// Nothing is borrowed here, which is the case where position order and age order coincide. A slot
// held across laps and then released is older than every other and is still not preferred, so the
// ring runs one frame short until the scan reaches it. This test does not cover
// that; no test does.
TEST(Channel, ALappedRingRetainsTheNewestSlotCountFrames)
{
  constexpr std::uint32_t kSlots = 8;
  flux::Channel ch(64, kSlots);
  flux::QoS q;
  q.depth = kSlots;
  ch.qos(q);

  std::vector<std::byte> buf(64);
  for (std::uint64_t f = 1; f <= 3 * kSlots; ++f) {
    ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), f), flux::Published::Ok);
  }

  std::vector<std::uint64_t> retained;
  while (flux::FrameView v = ch.take()) {
    retained.push_back(frame_id(v));
  }

  std::vector<std::uint64_t> want;
  for (std::uint64_t f = 2 * kSlots + 1; f <= 3 * kSlots; ++f) want.push_back(f);
  EXPECT_EQ(retained, want);
}

// volatile (the default) joins the stream where this consumer joined it, not where it first got
// around to reading: the backlog already in the ring is not replayed.
TEST(Channel, VolatileSkipsBacklog)
{
  flux::Channel ch(64, 8);
  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 3; ++f) {
    std::memset(buf.data(), f, buf.size());
    ASSERT_EQ(
      publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f)), flux::Published::Ok);
  }

  flux::QoS q;
  q.depth = 8;
  ch.qos(q);                // joins here
  EXPECT_FALSE(ch.take());  // backlog ignored
  EXPECT_TRUE(ch.peek());   // but current state is still readable

  std::memset(buf.data(), 4, buf.size());
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 4), flux::Published::Ok);
  flux::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(frame_id(v), 4u);
}

// transient_local(n) replays the last n retained frames first, then follows live.
TEST(Channel, TransientLocalReplaysBacklog)
{
  flux::Channel ch(64, 8);
  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 5; ++f) {
    std::memset(buf.data(), f, buf.size());
    ASSERT_EQ(
      publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f)), flux::Published::Ok);
  }

  flux::QoS q;
  q.depth = 8;
  q.durability = flux::Durability::TransientLocal(3);
  ch.qos(q);

  for (std::uint64_t expect = 3; expect <= 5; ++expect) {
    flux::FrameView v = ch.take();
    ASSERT_TRUE(v) << "expected frame " << expect;
    EXPECT_EQ(frame_id(v), expect);
    EXPECT_EQ(ch.lost(), 0u);
  }
  EXPECT_FALSE(ch.take());

  std::memset(buf.data(), 6, buf.size());
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 6), flux::Published::Ok);
  flux::FrameView v6 = ch.take();
  ASSERT_TRUE(v6);
  EXPECT_EQ(frame_id(v6), 6u);
}

// A replay deeper than the ring still holds is capped, and the shortfall is reported.
TEST(Channel, TransientLocalCappedByRing)
{
  flux::Channel ch(64, 2);
  std::vector<std::byte> buf(64);
  for (int f = 1; f <= 5; ++f) {
    std::memset(buf.data(), f, buf.size());
    ASSERT_EQ(
      publish_id(ch, buf.data(), buf.size(), static_cast<std::uint64_t>(f)), flux::Published::Ok);
  }

  flux::QoS q;
  q.depth = 8;
  q.durability = flux::Durability::TransientLocal(8);
  ch.qos(q);

  flux::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(frame_id(v), 4u);
  EXPECT_EQ(ch.lost(), 3u);
}

TEST(Channel, QosDefaults)
{
  flux::Channel ch(64, 8);
  EXPECT_EQ(ch.slot_count(), 8u);  // the ring, chosen by the publisher
  EXPECT_EQ(ch.qos().depth, 1u);
  EXPECT_TRUE(ch.qos().durability.is_volatile());
  EXPECT_EQ(ch.qos().max_borrow, 2u);
}

// A QoS flux cannot honour is rejected, never reinterpreted.
TEST(Channel, QosRejectsUnhonourableCombinations)
{
  flux::Channel ch(64, 8);
  {
    flux::QoS q;
    q.depth = 2;
    q.durability = flux::Durability::TransientLocal(3);  // replay deeper than the lag window
    EXPECT_THROW(ch.qos(q), std::invalid_argument);
  }
  {
    flux::QoS q;
    q.depth = 0;
    EXPECT_THROW(ch.qos(q), std::invalid_argument);
  }
  {
    flux::QoS q;
    q.max_borrow = 0;
    EXPECT_THROW(ch.qos(q), std::invalid_argument);
  }
  {
    flux::QoS q;
    q.reliability = flux::Reliability::Reliable;
    EXPECT_THROW(ch.qos(q), std::invalid_argument);
  }
  EXPECT_EQ(ch.qos().depth, 1u);  // a rejected setting never took effect
}

// A frame's itemsize cannot contradict its dtype: publish derives it rather than taking it. A
// consumer sizes its typed view from the dtype, so a disagreement would let that view run past
// the frame -- the Python binding once built an 8 MiB numpy array over a 1 MiB frame and read
// unmapped memory. The guard is now the signature, so what is asserted here is that the derived
// descriptor agrees with the bytes actually carried.
TEST(Channel, PublishDerivesItemsizeFromDtype)
{
  flux::Channel ch(1024, 4);
  const std::uint64_t vals[8] = {1, 2, 3, 4, 5, 6, 7, 8};

  ASSERT_EQ(ch.publish(vals, flux::DType::U64, {8}), flux::Published::Ok);

  flux::FrameView v = ch.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.meta().dtype, flux::DType::U64);
  EXPECT_EQ(v.meta().itemsize, 8u);
  EXPECT_EQ(v.meta().shape[0], 8u);
  EXPECT_EQ(v.meta().nbytes, sizeof(vals));
  EXPECT_EQ(v.size(), sizeof(vals));
}

// The engine carries a dtype it cannot compute with. bf16 exists on the wire so an inference
// publisher can hand its native type across untouched; the slot stays a byte range and only
// dtype_size decides how the far side sizes a view.
TEST(Channel, CarriesBFloat16Frames)
{
  flux::Channel ch(1024, 4);
  const std::uint16_t bits[4] = {0x3F80, 0xC020, 0x4070, 0x42C8};  // 1, -2.5, 3.75, 100

  ASSERT_EQ(ch.publish(bits, flux::DType::BF16, {4}), flux::Published::Ok);

  flux::FrameView v = ch.peek();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.meta().dtype, flux::DType::BF16);
  EXPECT_EQ(v.meta().itemsize, 2u);
  EXPECT_EQ(v.size(), sizeof(bits));
  EXPECT_EQ(std::memcmp(v.data(), bits, sizeof(bits)), 0);
}

// lost() must account for every frame this consumer never got, including the ones the depth
// window skipped on a take that then returned nothing. Crediting the skip only on the delivery
// path made frames vanish from the accounting whenever take() bailed out after the clamp --
// which is what happens under multiple publishers, or once a slot's holder table is full.
TEST(Channel, LostAccountsForFramesSkippedByATakeThatDeliversNothing)
{
  flux::Channel ch(64, 3);
  flux::QoS q;
  q.depth = 1;
  ch.qos(q);

  int published = 0;
  int received = 0;
  auto emit = [&](std::uint8_t v) {
    std::vector<std::byte> buf(64, static_cast<std::byte>(v));
    if (publish_id(ch, buf.data(), buf.size(), v) == flux::Published::Ok) ++published;
  };
  auto consume = [&] {
    if (flux::FrameView v = ch.take()) ++received;
  };

  for (std::uint8_t i = 1; i <= 3; ++i) emit(i);
  consume();  // delivers the newest, skips the rest
  emit(4);
  emit(5);

  // Claim every slot, so the next take clamps the cursor forward and then finds no candidate.
  std::vector<flux::WriteSlot> claims;
  for (std::uint32_t i = 0; i < ch.slot_count(); ++i) {
    flux::WriteSlot w = ch.loan();
    if (w) claims.push_back(std::move(w));
  }
  ASSERT_FALSE(claims.empty()) << "no slot could be claimed; the scenario did not set up";
  consume();       // clamps, delivers nothing -- the skipped span must still be counted
  claims.clear();  // aborts: those slots carry no frame afterwards

  emit(6);
  consume();

  EXPECT_EQ(received + static_cast<int>(ch.lost()), published)
    << "received=" << received << " lost=" << ch.lost() << " published=" << published;
}

// A view points into the segment, not at a copy, so it must keep that mapping alive by itself.
// While FrameView held a raw Channel pointer, outliving the Channel meant touching memory the
// Channel had already unmapped -- the Python binding avoided this by pinning the Subscription,
// but C++ had no equivalent.
TEST(Channel, ViewOutlivesTheChannelThatIssuedIt)
{
  flux::FrameView v;
  {
    flux::Channel ch(128, 4);
    std::vector<std::byte> buf(128);
    ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 0x77), flux::Published::Ok);
    v = ch.peek();
    ASSERT_TRUE(v);
  }  // the Channel is gone; only the view keeps the mapping

  EXPECT_EQ(v.size(), 128u);
  EXPECT_EQ(frame_id(v), 0x77u);
  const auto * p = static_cast<const std::uint8_t *>(v.data());
  for (std::size_t i = 0; i < v.size(); ++i) EXPECT_EQ(p[i], 0x77u);
  v.release();  // releasing after the Channel died must also be safe
}

// Same rule on the write side: a loan aliases the slot, so dropping it after the Channel is gone
// still writes the seq word to revert the claim.
TEST(Channel, LoanOutlivesTheChannelThatIssuedIt)
{
  flux::WriteSlot w;
  {
    flux::Channel ch(128, 4);
    w = ch.loan();
    ASSERT_TRUE(w);
    std::memset(w.data(), 0x33, 128);
  }
  EXPECT_TRUE(w);
  w.abort();
}

// `latest` names the newest slot and comes from another process. A publisher only ever packs a
// slot it selected, but a corrupt or rogue one need not: peek() indexes straight off that value,
// so an out-of-range slot put the borrow's refcount++ outside the mapping (found by
// test/fuzz/fuzz_frame). take() was never exposed -- its scan is bounded by slot_count.
TEST(Channel, PeekRejectsAnOutOfRangeLatestSlot)
{
  constexpr std::uint32_t kSlots = 4;
  flux::Segment seg = flux::Segment::create_heap(256, kSlots);
  auto * ctrl = reinterpret_cast<flux::ControlHeader *>(seg.base());
  ctrl->latest.store(flux::pack_latest(1, 4000), std::memory_order_release);

  flux::Channel ch(std::move(seg));
  EXPECT_FALSE(ch.peek().valid()) << "peek() handed out a view for a slot outside the ring";
  EXPECT_FALSE(ch.take().valid());
  for (std::uint32_t i = 0; i < kSlots; ++i) {
    EXPECT_EQ(ch.slot_refcount(i), 0u) << "a rejected borrow left a refcount behind on slot " << i;
  }
}

// The one line every publisher writes. Backpressure is a dropped frame on a best-effort
// transport; the other three do not clear on their own, and collapsing them into the same answer
// is what let a leaked slot go unreported.
TEST(Channel, FaultedSeparatesADroppedFrameFromAFault)
{
  EXPECT_FALSE(flux::faulted(flux::Published::Ok));
  EXPECT_FALSE(flux::faulted(flux::Published::Backpressure));
  EXPECT_TRUE(flux::faulted(flux::Published::TooLarge));
  EXPECT_TRUE(flux::faulted(flux::Published::WrongDevice));
  EXPECT_TRUE(flux::faulted(flux::Published::FenceFailed));

  EXPECT_STREQ(flux::to_string(flux::Published::Ok), "Ok");
  EXPECT_STREQ(flux::to_string(flux::Published::Backpressure), "Backpressure");
  EXPECT_STREQ(flux::to_string(flux::Published::FenceFailed), "FenceFailed");
}

// A one-slot ring, so the outcomes are reachable without a GPU: the borrow blocks the publisher,
// and a spent handle answers the same way in both languages.
TEST(Channel, PublishReportsBackpressureAndASpentHandleReportsTooLarge)
{
  flux::Channel ch(64, 1);
  const std::uint8_t byte = 7;

  EXPECT_EQ(ch.publish(&byte, 1), flux::Published::Ok);
  flux::FrameView held = ch.take();
  ASSERT_TRUE(held.valid());
  EXPECT_EQ(ch.publish(&byte, 1), flux::Published::Backpressure);
  EXPECT_EQ(ch.dropped(), 1u);
  held.release();

  flux::WriteSlot w = ch.loan();
  ASSERT_TRUE(w.valid());
  EXPECT_EQ(w.commit(), flux::Published::Ok);
  EXPECT_EQ(w.commit(), flux::Published::TooLarge) << "a spent handle must not report Ok";

  EXPECT_EQ(ch.publish(&byte, 65), flux::Published::TooLarge);
}

// bad_frame is the counter for a slot `latest` still names but that carries no frame. abort()
// makes exactly that state: it drops the ticket without moving `latest`, so on a one-slot ring
// the next reader is pointed at a slot whose commit_ticket is 0.
TEST(Channel, AnAbortedClaimLeavesLatestPointingAtNoFrameAndIsCounted)
{
  flux::Channel ch(64, 1);
  std::vector<std::byte> buf(64, std::byte{7});
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 1), flux::Published::Ok);
  ASSERT_TRUE(ch.peek());
  ASSERT_EQ(ch.refused().bad_frame, 0u);

  flux::WriteSlot slot = ch.loan(flux::DType::U8, {64});
  ASSERT_TRUE(slot.valid()) << "the only slot must be free here";
  slot.abort();

  EXPECT_FALSE(ch.peek()) << "a slot with no committed frame was handed out";
  EXPECT_EQ(ch.refused().bad_frame, 1u);
  EXPECT_EQ(ch.refused().max_borrow, 0u) << "nothing is held; the frame is what is missing";
  EXPECT_EQ(ch.lost(), 0u) << "an aborted claim is not a lapped frame";
}
