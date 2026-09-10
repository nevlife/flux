#include "flux/channel.hpp"
#include "support/frame_id.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

// Covers the in-segment futex wake/wait: a blocked subscriber is
// woken by publish, a timeout returns empty, and the wake generation advances per frame.

namespace
{
using flux::test::frame_id;
using flux::test::publish_id;

// A frame's identity is its payload: the descriptor is derived from the byte count, so an id in
// shape[0] would have to contradict nbytes to be an id at all. The whole buffer carries the tag,
// which is also what makes an overwrite of a held frame visible.

}  // namespace

TEST(Wake, TakeBlockingTimesOutOnEmpty)
{
  flux::Channel ch(256, 4);
  const auto t0 = std::chrono::steady_clock::now();
  flux::FrameView v = ch.take_blocking(/*timeout_ns=*/20'000'000);  // 20 ms
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_FALSE(v);
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 15);
}

TEST(Wake, WakeSeqAdvancesPerPublish)
{
  flux::Channel ch(64, 2);
  const std::uint32_t s0 = ch.wake_seq();
  std::vector<std::byte> buf(64, std::byte{0x11});
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 1), flux::Published::Ok);
  const std::uint32_t s1 = ch.wake_seq();
  EXPECT_NE(s1, s0);
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 2), flux::Published::Ok);
  EXPECT_NE(ch.wake_seq(), s1);
}

TEST(Wake, PublishWakesBlockedSubscriber)
{
  flux::Channel ch(256, 4);
  std::atomic<bool> got{false};
  std::thread sub([&] {
    flux::FrameView v = ch.take_blocking(/*timeout_ns=*/2'000'000'000);  // up to 2 s
    if (v && static_cast<const std::uint8_t *>(v.data())[0] == 0x5A) got.store(true);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));  // let the subscriber park
  std::vector<std::byte> buf(256);
  ASSERT_EQ(publish_id(ch, buf.data(), buf.size(), 0x5A), flux::Published::Ok);

  sub.join();
  EXPECT_TRUE(got.load());
}

// No lost wakeup even when publish races the park: hammer publishes while a subscriber
// repeatedly blocks. A lost wakeup would show as wait() timing out even though a publish
// advanced the wake generation during the wait -- the Dekker wake gate
// must make that impossible, so lost_wakeups stays 0.
TEST(Wake, NoLostWakeupUnderRace)
{
  flux::Channel ch(4096, 8);
  std::atomic<bool> stop{false};
  std::atomic<int> received{0};
  std::atomic<int> lost_wakeups{0};

  std::thread sub([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      const std::uint32_t s = ch.wake_seq();  // sample the generation, then look for data
      if (flux::FrameView v = ch.take()) {
        received.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      if (ch.wait(s, /*timeout_ns=*/200'000'000)) {  // woken (or the generation already moved)
        if (flux::FrameView v = ch.take()) received.fetch_add(1, std::memory_order_relaxed);
      } else if (ch.wake_seq() != s) {
        // Timed out even though a publish advanced the generation during the wait: a lost wakeup.
        lost_wakeups.fetch_add(1, std::memory_order_relaxed);
      }
    }
  });

  std::vector<std::byte> buf(4096, std::byte{0x24});
  for (int i = 0; i < 500; ++i) {
    publish_id(ch, buf.data(), buf.size(), static_cast<std::uint8_t>(i));
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  stop.store(true);
  publish_id(ch, buf.data(), buf.size(), 0xFF);  // unblock a final park
  sub.join();

  EXPECT_EQ(lost_wakeups.load(), 0);  // no publish went undelivered while the subscriber blocked
  EXPECT_GT(received.load(), 0);      // and the subscriber made progress
}
