#include "flux/spin_control.hpp"

#include <gtest/gtest.h>
#include <poll.h>

#include <chrono>
#include <functional>
#include <stdexcept>
#include <thread>

// The wake fd and the two loop flags both executors share. Each of
// these was previously reachable only through an executor, and one of them -- a waker outliving
// its owner -- only through flux::Executor, because the PartitionedExecutor closed its fd outright.

using namespace std::chrono_literals;

namespace
{
bool readable(int fd, int ms)
{
  struct pollfd p;
  p.fd = fd;
  p.events = POLLIN;
  p.revents = 0;
  ::poll(&p, 1, ms);
  return (p.revents & POLLIN) != 0;
}
}  // namespace

TEST(SpinControl, InterruptMakesTheFdReadableAndDrainClearsIt)
{
  flux::SpinControl ctl;
  EXPECT_FALSE(readable(ctl.fd(), 0));
  ctl.interrupt();
  EXPECT_TRUE(readable(ctl.fd(), 0));
  ctl.drain();
  EXPECT_FALSE(readable(ctl.fd(), 0));
}

TEST(SpinControl, StopRequestsAndPokes)
{
  flux::SpinControl ctl;
  EXPECT_FALSE(ctl.stop_requested());
  ctl.stop();
  EXPECT_TRUE(ctl.stop_requested());
  EXPECT_TRUE(readable(ctl.fd(), 0));
}

// An owner tearing itself down has nothing waiting, so the request must not leave a poke behind
// for a later wait to return on.
TEST(SpinControl, RequestStopLeavesTheFdAlone)
{
  flux::SpinControl ctl;
  ctl.request_stop();
  EXPECT_TRUE(ctl.stop_requested());
  EXPECT_FALSE(readable(ctl.fd(), 0));
}

TEST(SpinControl, ASecondSessionThrows)
{
  flux::SpinControl ctl;
  flux::SpinControl::Session s(ctl);
  EXPECT_TRUE(ctl.is_spinning());
  EXPECT_THROW((flux::SpinControl::Session{ctl}), std::logic_error);
}

// A stop that lands before the loop is entered must still end it, and must not survive
// into the next one.
TEST(SpinControl, AStopBeforeTheSessionIsHeldThenCleared)
{
  flux::SpinControl ctl;
  ctl.stop();
  {
    flux::SpinControl::Session s(ctl);
    EXPECT_TRUE(ctl.stop_requested()) << "the stop was lost on the way into the loop";
  }
  EXPECT_FALSE(ctl.stop_requested());
  EXPECT_FALSE(ctl.is_spinning());
  EXPECT_FALSE(readable(ctl.fd(), 0)) << "this session's interrupt would end the next one's wait";
}

TEST(SpinControl, TheSessionEndsEvenWhenTheLoopThrows)
{
  flux::SpinControl ctl;
  EXPECT_THROW(
    {
      flux::SpinControl::Session s(ctl);
      throw std::runtime_error("callback");
    },
    std::runtime_error);
  EXPECT_FALSE(ctl.is_spinning());
  flux::SpinControl::Session again(ctl);  // spinnable again
}

// The hook a foreign runtime still holds after the owner is gone. It reaches the fd weakly, so
// this writes to a still-open fd or does nothing, never to a recycled fd number.
TEST(SpinControl, TheWakerOutlivesItsOwner)
{
  std::function<void()> wake;
  {
    flux::SpinControl ctl;
    wake = ctl.waker();
    wake();
    EXPECT_TRUE(readable(ctl.fd(), 0));
  }
  wake();
}

TEST(SpinControl, TheWakerBreaksAWaitFromAnotherThread)
{
  flux::SpinControl ctl;
  auto wake = ctl.waker();
  std::thread poker([&] {
    std::this_thread::sleep_for(50ms);
    wake();
  });
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_TRUE(readable(ctl.fd(), 5000));
  const auto ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0)
      .count();
  poker.join();
  EXPECT_LT(ms, 3000);
}
