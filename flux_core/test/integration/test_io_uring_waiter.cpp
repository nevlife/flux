#include "flux/channel.hpp"
#include "flux/executor.hpp"
#include "flux/io_uring_waiter.hpp"
#include "support/frame_id.hpp"

#include <gtest/gtest.h>
#include <linux/filter.h>
#include <linux/seccomp.h>
#include <sys/eventfd.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// Exercises the merged io_uring FUTEX_WAIT layer against real channels: a publish wakes a
// blocked waiter across a thread boundary, a timeout returns nothing, an already-advanced
// word completes immediately, and a merged wait over two channels reports only the fired
// one. Skips cleanly on kernels without io_uring futex (< 6.7).

using namespace std::chrono_literals;

namespace
{
using flux::test::frame_id;
using flux::test::publish_id;

bool has_tag(const std::vector<flux::WakeEvent> & ev, std::uint64_t tag)
{
  return std::any_of(
    ev.begin(), ev.end(), [tag](const flux::WakeEvent & e) { return e.tag == tag; });
}

// A frame's identity is its payload: the descriptor is derived from the byte count, so an id in
// shape[0] would have to contradict nbytes to be an id at all. The whole buffer carries the tag,
// which is also what makes an overwrite of a held frame visible.

void publish_one(flux::Channel & ch, std::uint8_t v)
{
  std::vector<std::byte> buf(256, std::byte{v});
  (void)publish_id(ch, buf.data(), buf.size(), v);
}

class IoUringWaiterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Only a missing opcode is a legitimate skip. A ring that could not be created means this
    // host refuses io_uring on a kernel that has it -- a container seccomp profile, an fd or
    // memlock limit -- and skipping there reports the wait layer as untestable while blaming a
    // kernel that is fine. The engine refuses to fold those two together and neither
    // does this: an untested wait layer must be louder than an absent one, not identical to it.
    switch (flux::IoUringWaiter::support()) {
      case flux::IoUringSupport::Yes:
        break;
      case flux::IoUringSupport::NoOpcode:
        GTEST_SKIP() << "flux-cap:io-uring kernel lacks io_uring FUTEX_WAIT (needs 6.7+)";
      case flux::IoUringSupport::Forbidden:
        FAIL() << "io_uring is forbidden on this host (container seccomp profile, "
                  "kernel.io_uring_disabled), not missing from this kernel. The merged wait "
                  "layer went unchecked.";
      case flux::IoUringSupport::RingFailed:
        FAIL() << "io_uring is denied on this host, not missing from this kernel: the ring "
                  "could not be created (RLIMIT_NOFILE, RLIMIT_MEMLOCK). The merged wait layer "
                  "went unchecked.";
    }
  }
};

}  // namespace

// Tags encode (kind, index) so dispatch switches on the kind; no index value is reserved.
// Plain TEST: the encoding needs no ring, so it runs on kernels without io_uring futex too.
TEST(EventTag, RoundTripsKindAndIndex)
{
  const std::uint64_t ch = flux::make_tag(flux::EventKind::channel, 7);
  EXPECT_EQ(flux::tag_kind(ch), flux::EventKind::channel);
  EXPECT_EQ(flux::tag_index(ch), 7u);

  const std::uint64_t ctl = flux::make_tag(flux::EventKind::control, 0);
  EXPECT_EQ(flux::tag_kind(ctl), flux::EventKind::control);
  EXPECT_EQ(flux::tag_index(ctl), 0u);
  EXPECT_NE(ch, ctl);

  // ~0 used to be reserved as "not a channel"; the kind field frees every index.
  const std::uint64_t hi = flux::make_tag(flux::EventKind::channel, ~0u);
  EXPECT_EQ(flux::tag_kind(hi), flux::EventKind::channel);
  EXPECT_EQ(flux::tag_index(hi), ~0u);
}

TEST_F(IoUringWaiterTest, WakesOnPublishFromAnotherThread)
{
  flux::Channel ch(256, 4);
  flux::IoUringWaiter w;
  ASSERT_TRUE(w.valid());

  ch.add_waiter();  // tell the publisher to issue the wake syscall
  const std::uint32_t seq = ch.wake_seq();
  w.arm(ch.wake_word(), seq, 42);

  std::thread pub([&] {
    std::this_thread::sleep_for(50ms);
    publish_one(ch, 0x5A);
  });

  std::vector<flux::WakeEvent> ev;
  const int n = w.wait(ev, 2'000'000'000);  // up to 2 s
  pub.join();
  ch.remove_waiter();

  EXPECT_GE(n, 1);
  EXPECT_TRUE(has_tag(ev, 42));
  EXPECT_TRUE(ch.peek()) << "woken but no frame to peek";
}

TEST_F(IoUringWaiterTest, TimesOutWithNoPublish)
{
  flux::Channel ch(256, 4);
  flux::IoUringWaiter w;
  ch.add_waiter();
  w.arm(ch.wake_word(), ch.wake_seq(), 7);

  std::vector<flux::WakeEvent> ev;
  const auto t0 = std::chrono::steady_clock::now();
  const int n = w.wait(ev, 30'000'000);  // 30 ms
  const auto dt = std::chrono::steady_clock::now() - t0;
  ch.remove_waiter();

  EXPECT_EQ(n, 0);
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 20);
}

TEST_F(IoUringWaiterTest, CompletesImmediatelyWhenAlreadyAdvanced)
{
  flux::Channel ch(256, 4);
  publish_one(ch, 0x09);  // wake_seq is now 1
  flux::IoUringWaiter w;
  w.arm(ch.wake_word(), 0, 99);  // expected 0, but the word already moved -> -EAGAIN

  std::vector<flux::WakeEvent> ev;
  const int n = w.wait(ev, 1'000'000'000);
  EXPECT_GE(n, 1);
  EXPECT_TRUE(has_tag(ev, 99));
}

TEST_F(IoUringWaiterTest, MergedWaitReportsOnlyFiredChannel)
{
  flux::Channel a(256, 4);
  flux::Channel b(256, 4);
  flux::IoUringWaiter w;

  a.add_waiter();
  b.add_waiter();
  w.arm(a.wake_word(), a.wake_seq(), 1);
  w.arm(b.wake_word(), b.wake_seq(), 2);

  publish_one(b, 0x42);  // only b fires

  std::vector<flux::WakeEvent> ev;
  const int n = w.wait(ev, 1'000'000'000);
  a.remove_waiter();
  b.remove_waiter();

  EXPECT_GE(n, 1);
  EXPECT_TRUE(has_tag(ev, 2));   // b fired
  EXPECT_FALSE(has_tag(ev, 1));  // a stayed pending
}

// The whole point of the merged wait: one ring merges a channel's futex wake with a plain
// readiness fd (an eventfd standing in for the ROS side). Only the signaled one reports.
TEST_F(IoUringWaiterTest, MergedFutexAndPollFd)
{
  const int efd = ::eventfd(0, EFD_NONBLOCK);
  ASSERT_GE(efd, 0);
  flux::Channel ch(256, 4);
  ch.add_waiter();

  flux::IoUringWaiter w;
  w.arm(ch.wake_word(), ch.wake_seq(), 100);  // flux futex
  w.arm_poll(efd, 200);                       // ROS-side fd

  const std::uint64_t one = 1;
  ASSERT_EQ(::write(efd, &one, sizeof(one)), static_cast<ssize_t>(sizeof(one)));

  std::vector<flux::WakeEvent> ev;
  const int n = w.wait(ev, 1'000'000'000);
  ch.remove_waiter();
  ::close(efd);

  EXPECT_GE(n, 1);
  EXPECT_TRUE(has_tag(ev, 200));   // the fd fired
  EXPECT_FALSE(has_tag(ev, 100));  // the futex stayed pending
}

// cancel() reaps a pending wait: without it, a wait armed on a word that will never move again
// (a re-attached channel's old mapping) stays in the kernel for the ring's lifetime.
TEST_F(IoUringWaiterTest, CancelReapsAPendingWait)
{
  flux::Channel ch(256, 4);
  flux::IoUringWaiter w(8);
  std::vector<flux::WakeEvent> ev;

  w.arm(ch.wake_word(), ch.wake_seq(), 7);
  w.cancel(7);
  // The canceled wait completes under tag 7 and the cancel op under its own kind; drain both.
  int drained = 0;
  while (w.wait(ev, 200'000'000) > 0) {
    for (const flux::WakeEvent & e : ev) {
      EXPECT_TRUE(e.tag == 7u || flux::tag_kind(e.tag) == flux::EventKind::cancel) << e.tag;
      ++drained;
    }
  }
  EXPECT_GE(drained, 1);

  // The wait is gone: a wake on the word must not complete anything.
  ch.add_waiter();
  publish_one(ch, 1);
  ch.remove_waiter();
  EXPECT_EQ(w.wait(ev, 100'000'000), 0);
}

namespace
{

// Makes one syscall fail with `err` for the rest of this process, as a container's seccomp
// profile does. No privilege needed once no_new_privs is set.
bool deny_syscall(long nr, int err)
{
  sock_filter filter[] = {
    BPF_STMT(BPF_LD | BPF_W | BPF_ABS, offsetof(seccomp_data, nr)),
    BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, static_cast<std::uint32_t>(nr), 0, 1),
    BPF_STMT(
      BPF_RET | BPF_K, SECCOMP_RET_ERRNO | (static_cast<std::uint32_t>(err) & SECCOMP_RET_DATA)),
    BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW),
  };
  sock_fprog prog{static_cast<unsigned short>(sizeof(filter) / sizeof(filter[0])), filter};
  return ::prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) == 0 &&
         ::prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &prog) == 0;
}

struct ChildResult
{
  int status = -1;
  std::string err;  // what the child wrote to stderr
};

// Runs `body` in a child whose stderr is captured; its return value is the exit status.
template <typename Body>
ChildResult in_child(Body body)
{
  int fds[2];
  if (::pipe(fds) != 0) return {};
  const pid_t pid = ::fork();
  if (pid == 0) {
    ::close(fds[0]);
    ::dup2(fds[1], STDERR_FILENO);
    ::_exit(body());
  }
  ::close(fds[1]);
  ChildResult r;
  char buf[512];
  for (ssize_t n; (n = ::read(fds[0], buf, sizeof(buf))) > 0;) r.err.append(buf, n);
  ::close(fds[0]);
  ::waitpid(pid, &r.status, 0);
  return r;
}

std::size_t count(const std::string & s, const std::string & what)
{
  std::size_t n = 0;
  for (auto at = s.find(what); at != std::string::npos; at = s.find(what, at + 1)) ++n;
  return n;
}

}  // namespace

// A container that forbids io_uring is a host the fallback serves, not a resource leak: the
// executor still runs, and says once why it is not on the merged wait.
TEST(IoUringSupport, AForbiddenRingFallsBackAndSaysSoOnce)
{
  const ChildResult r = in_child([] {
    if (!deny_syscall(__NR_io_uring_setup, EPERM)) return 2;
    try {
      flux::Executor first;
      flux::Executor second;
    } catch (const std::exception &) {
      return 1;
    }
    return 0;
  });
  ASSERT_TRUE(WIFEXITED(r.status));
  EXPECT_EQ(WEXITSTATUS(r.status), 0) << r.err;
  EXPECT_EQ(count(r.err, "io_uring is forbidden"), 1u) << r.err;
}

// A kernel without the opcode, whether it has no io_uring (ENOSYS from setup) or rings only
// older layouts (refused with ENOSYS at construction), is the quiet old-kernel fallback.
TEST(IoUringSupport, AKernelWithoutTheOpcodeFallsBackQuietly)
{
  const ChildResult r = in_child([] {
    if (!deny_syscall(__NR_io_uring_setup, ENOSYS)) return 2;
    if (flux::IoUringWaiter::support() != flux::IoUringSupport::NoOpcode) return 3;
    try {
      flux::Executor ex;
    } catch (const std::exception &) {
      return 1;
    }
    return 0;
  });
  ASSERT_TRUE(WIFEXITED(r.status));
  EXPECT_EQ(WEXITSTATUS(r.status), 0) << r.err;
  EXPECT_EQ(r.err, "");
}

// A probe the kernel could not run says nothing about the opcode, so it is not the old-kernel
// fallback: the executor reports it instead of quietly running on parker threads.
TEST(IoUringSupport, AProbeThatCannotRunIsNotAMissingOpcode)
{
  if (flux::IoUringWaiter::support() != flux::IoUringSupport::Yes) {
    GTEST_SKIP() << "flux-cap:io-uring needs a host where the probe succeeds";
  }
  const ChildResult r = in_child([] {
    if (!deny_syscall(__NR_io_uring_enter, ENOMEM)) return 2;
    try {
      flux::Executor ex;
    } catch (const std::runtime_error &) {
      return 0;
    }
    return 1;
  });
  ASSERT_TRUE(WIFEXITED(r.status));
  EXPECT_EQ(WEXITSTATUS(r.status), 0) << r.err;
}
