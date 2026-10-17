// Qt defines `forever`, and a Qt header often comes first.
#define forever for (;;)
#include "flux2/detail/subscriber.hpp"
#undef forever

#include "flux2/channel.hpp"
#include "flux2/detail/manifest.hpp"
#include "flux2/topics.hpp"

#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace
{

std::string text(const flux2::FrameView & v)
{
  return std::string(static_cast<const char *>(v.data()), v.size());
}

std::string unique_name(const char * test, std::uint64_t fingerprint = flux2::kNoSchema)
{
  return flux2::detail::signpost_name("/cpp" + std::to_string(getpid()) + "_" + test, fingerprint);
}

flux2::Status status_of(void (*f)(const std::string &), const std::string & name)
{
  try {
    f(name);
  } catch (const flux2::Error & e) {
    return e.status();
  }
  return FLUX2_OK;
}

TEST(Channel, TakesWhatWasPublished)
{
  flux2::detail::Channel ch(16, 4);
  EXPECT_FALSE(ch.take());
  EXPECT_EQ(ch.publish("abc", 3), flux2::Published::Ok);
  flux2::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(text(v), "abc");
  EXPECT_EQ(v.ticket(), 1u);
  EXPECT_GT(v.publish_ns(), 0u);
  EXPECT_EQ(ch.publish(std::string(17, 'x').data(), 17), flux2::Published::TooLarge);
}

TEST(Channel, AMovedViewHoldsTheBorrow)
{
  flux2::detail::Channel ch(8, 2);
  ch.qos(flux2::QoS(2).max_borrow(2));
  ASSERT_EQ(ch.publish("a", 1), flux2::Published::Ok);
  ASSERT_EQ(ch.publish("b", 1), flux2::Published::Ok);
  flux2::FrameView a = ch.take();
  flux2::FrameView moved = std::move(a);
  EXPECT_FALSE(a);  // NOLINT(bugprone-use-after-move): a moved-from view is empty
  flux2::FrameView b = ch.take();
  EXPECT_EQ(text(moved), "a");
  EXPECT_EQ(ch.publish("c", 1), flux2::Published::Backpressure);
  moved = flux2::FrameView();
  EXPECT_EQ(ch.publish("c", 1), flux2::Published::Ok);
}

TEST(Channel, ALoanPublishesOnlyOnCommit)
{
  flux2::detail::Channel ch(16, 2);
  {
    flux2::WriteSlot w = ch.loan();
    ASSERT_TRUE(w);
    EXPECT_EQ(w.capacity(), 16u);
    std::memcpy(w.data(), "dropped", 7);
  }
  EXPECT_FALSE(ch.take());
  flux2::WriteSlot w = ch.loan();
  std::memcpy(w.data(), "hello", 5);
  flux2::WriteSlot moved = std::move(w);
  EXPECT_EQ(w.commit(), flux2::Published::InvalidArgument);  // NOLINT(bugprone-use-after-move)
  EXPECT_EQ(moved.commit(5), flux2::Published::Ok);
  EXPECT_FALSE(moved);
  EXPECT_EQ(text(ch.take()), "hello");
}

TEST(Channel, ATypedFrameCarriesItsShape)
{
  flux2::detail::Channel ch(64, 2);
  ch.qos(flux2::QoS().max_borrow(2));
  const std::array<float, 6> data{};
  ASSERT_EQ(ch.publish(data.data(), flux2::DType::F32, {2, 3}), flux2::Published::Ok);
  flux2::FrameView v = ch.take();
  ASSERT_TRUE(v);
  EXPECT_EQ(v.size(), sizeof(float) * data.size());
  EXPECT_EQ(v.meta().ndim, 2u);
  EXPECT_EQ(v.meta().dtype, FLUX2_DTYPE_F32);
  EXPECT_EQ(v.meta().shape[1], 3u);

  flux2::WriteSlot w = ch.loan(flux2::DType::U16, {4});
  ASSERT_TRUE(w);
  EXPECT_EQ(w.commit(), flux2::Published::Ok);
  EXPECT_EQ(ch.take().meta().nbytes, 8u);
}

TEST(Channel, ASubscriberSeesThePublisherAcrossShm)
{
  const std::string name = unique_name("pubsub");
  auto pub = flux2::detail::Channel::create(name, 64, 4);
  auto sub = flux2::detail::Channel::open(name);
  EXPECT_TRUE(sub.payload_readonly());
  EXPECT_EQ(pub.publish("frame", 5), flux2::Published::Ok);
  EXPECT_EQ(text(sub.take_blocking(1'000'000'000)), "frame");
  EXPECT_EQ(sub.publish("x", 1), flux2::Published::NotPublisher);
  EXPECT_EQ(sub.slot_size(), 64u);
}

TEST(Channel, FailuresCarryTheirStatus)
{
  const std::string name = unique_name("status");
  auto open = [](const std::string & n) { flux2::detail::Channel::open(n); };
  auto create = [](const std::string & n) { flux2::detail::Channel::create(n, 64, 2); };
  EXPECT_EQ(status_of(open, name), FLUX2_NOT_READY);
  auto pub = flux2::detail::Channel::create(name, 64, 2);
  EXPECT_EQ(status_of(create, name), FLUX2_PUBLISHER_EXISTS);

  const std::string typed = unique_name("typed", 7);
  auto pub7 = flux2::detail::Channel::create(typed, 64, 2, 7);
  EXPECT_EQ(status_of(open, typed), FLUX2_MISMATCH);
}

TEST(Channel, ThePublisherSetsHowManyProcessesMaySubscribe)
{
  const std::string name = unique_name("maxsubs");
  auto pub = flux2::detail::Channel::create(name, 64, 2, flux2::kNoSchema, {}, 64);
  EXPECT_EQ(flux2::read_channel_stats(name).max_subscribers, 64u);
  auto create = [](const std::string & n) {
    flux2::detail::Channel::create(n, 64, 2, flux2::kNoSchema, {}, FLUX2_MAX_SUBSCRIBERS + 1);
  };
  EXPECT_EQ(status_of(create, unique_name("maxbad")), FLUX2_INVALID_ARGUMENT);
}

TEST(Channel, TheChannelChecksQoS)
{
  flux2::detail::Channel ch(8, 4);
  ch.qos(flux2::QoS(3).transient_local(2).max_borrow(1));
  EXPECT_EQ(ch.qos().depth(), 3u);
  EXPECT_EQ(ch.qos().replay(), 2u);
  EXPECT_EQ(ch.qos().borrow_limit(), 1u);
  try {
    ch.qos(flux2::QoS(2).transient_local(3));
    FAIL() << "replay deeper than depth was accepted";
  } catch (const flux2::Error & e) {
    EXPECT_EQ(e.status(), FLUX2_INVALID_ARGUMENT);
    EXPECT_NE(std::string(e.what()), "");
  }
  EXPECT_EQ(ch.qos().depth(), 3u);
}

TEST(Channel, AMovedFromChannelIsEmpty)
{
  flux2::detail::Channel ch(16, 4);
  flux2::detail::Channel moved = std::move(ch);
  // NOLINTBEGIN(bugprone-use-after-move): what a moved-from channel does is the test
  EXPECT_EQ(ch.publish("a", 1), flux2::Published::InvalidArgument);
  EXPECT_EQ(ch.publish("a", flux2::DType::U8, {1}), flux2::Published::InvalidArgument);
  EXPECT_FALSE(ch.loan());
  EXPECT_FALSE(ch.take());
  EXPECT_FALSE(ch.take_blocking(-1));
  EXPECT_EQ(ch.slot_size(), 0u);
  EXPECT_EQ(ch.lost(), 0u);
  EXPECT_FALSE(ch.ready());
  EXPECT_EQ(ch.raw(), nullptr);
  EXPECT_THROW(ch.qos(flux2::QoS(2)), flux2::Error);
  // NOLINTEND(bugprone-use-after-move)
  EXPECT_EQ(moved.publish("a", 1), flux2::Published::Ok);
}

// Built with -fsanitize=signed-integer-overflow, which aborts on an overflowing deadline.
// Under an executor, a refused attach leaves the subscription out, even once a place frees, until
// its publisher restarts: the executor's other sources must not stop for it.
TEST(Subscriber, ARefusedAttachLeavesItOutUntilThePublisherRestarts)
{
  const std::string name = unique_name("leftout");
  flux2::detail::Subscriber sub(name, flux2::kNoSchema, flux2::QoS{}, {});
  // As a Subscription's does, it keeps the signpost and so its epoch count across the restart.
  const flux2::detail::Announcement announced(name, "/leftout", "", false);
  std::optional<flux2::detail::Channel> pub;
  pub.emplace(flux2::detail::Channel::create(name, 64, 2, flux2::kNoSchema, {}, 1));
  std::array<int, 2> ready{};
  ASSERT_EQ(pipe(ready.data()), 0);
  const pid_t child = fork();
  if (child == 0) {
    const auto held = flux2::detail::Channel::open(name);
    const char c = 1;
    if (write(ready[1], &c, 1) != 1) _exit(1);
    pause();
    _exit(0);
  }
  char c = 0;
  ASSERT_EQ(read(ready[0], &c, 1), 1);
  EXPECT_FALSE(sub.attach_for_executor());
  ASSERT_NE(sub.new_refusal(), nullptr);
  EXPECT_EQ(sub.new_refusal(), nullptr);
  kill(child, SIGKILL);
  waitpid(child, nullptr, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_FALSE(sub.attach_for_executor());
  EXPECT_FALSE(sub.take_for_executor());

  pub.reset();
  pub.emplace(flux2::detail::Channel::create(name, 64, 2, flux2::kNoSchema, {}, 1));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_TRUE(sub.attach_for_executor()) << sub.refusal();
  close(ready[0]);
  close(ready[1]);
}

// A publisher that restarts too fast for the orphan check and then has no place for this process:
// the take that finds the refusal leaves the subscription out like a refused attach.
TEST(Subscriber, ARefusedReAttachLeavesItOutUntilThePublisherRestarts)
{
  const std::string name = unique_name("leftout_fast");
  flux2::detail::Subscriber sub(name, flux2::kNoSchema, flux2::QoS{}, {});
  std::optional<flux2::detail::Channel> pub;
  pub.emplace(flux2::detail::Channel::create(name, 64, 2, flux2::kNoSchema, {}, 1));
  ASSERT_TRUE(sub.attach_for_executor());
  pub.reset();
  pub.emplace(flux2::detail::Channel::create(name, 64, 2, flux2::kNoSchema, {}, 1));
  std::array<int, 2> ready{};
  ASSERT_EQ(pipe(ready.data()), 0);
  const pid_t child = fork();
  if (child == 0) {
    const auto held = flux2::detail::Channel::open(name);
    const char c = 1;
    if (write(ready[1], &c, 1) != 1) _exit(1);
    pause();
    _exit(0);
  }
  char c = 0;
  ASSERT_EQ(read(ready[0], &c, 1), 1);
  for (int i = 0; i < 3 && sub.refusal().empty(); ++i) EXPECT_FALSE(sub.take_for_executor());
  kill(child, SIGKILL);
  waitpid(child, nullptr, 0);
  ASSERT_NE(sub.new_refusal(), nullptr);
  EXPECT_NE(sub.refusal().find("no free subscriber number"), std::string::npos) << sub.refusal();
  EXPECT_FALSE(sub.attached());

  pub.reset();
  pub.emplace(flux2::detail::Channel::create(name, 64, 2, flux2::kNoSchema, {}, 2));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_TRUE(sub.attach_for_executor()) << sub.refusal();
  close(ready[0]);
  close(ready[1]);
}

// Before its publisher, take_blocking sleeps on the signpost, which the publisher coming up wakes,
// rather than looking again every 100 ms.
TEST(Subscriber, TakeBlockingWakesWhenThePublisherComesUp)
{
  const std::string name = unique_name("tb_late");
  flux2::detail::Subscriber sub(name, flux2::kNoSchema, flux2::QoS{}, {});
  std::atomic<bool> stop{false};
  std::chrono::steady_clock::time_point up{};
  std::thread late([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto pub = flux2::detail::Channel::create(name, 64, 2);
    up = std::chrono::steady_clock::now();
    while (!stop) {
      (void)pub.publish("x", 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  const flux2::FrameView v = sub.take_blocking(2'000'000'000);
  const auto got = std::chrono::steady_clock::now();
  stop = true;
  late.join();
  ASSERT_TRUE(v);
  EXPECT_LT(got - up, std::chrono::milliseconds(40));
}

TEST(Subscriber, TheLongestTimeoutStillWaitsForAFrame)
{
  const std::string name = unique_name("tbmax");
  auto pub = flux2::detail::Channel::create(name, 64, 2);
  flux2::detail::Subscriber sub(name, flux2::kNoSchema, flux2::QoS{}, {});
  std::thread late([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    (void)pub.publish("x", 1);
  });
  const flux2::FrameView v = sub.take_blocking(std::numeric_limits<std::int64_t>::max());
  late.join();
  ASSERT_TRUE(v);
  EXPECT_EQ(text(v), "x");
}

TEST(Channel, OnlyAFullRingIsNotAFault)
{
  EXPECT_FALSE(flux2::faulted(flux2::Published::Ok));
  EXPECT_FALSE(flux2::faulted(flux2::Published::Backpressure));
  EXPECT_TRUE(flux2::faulted(flux2::Published::TooLarge));
  EXPECT_STREQ(flux2::to_string(flux2::Published::NotPublisher), "not a publisher");
}

}  // namespace
