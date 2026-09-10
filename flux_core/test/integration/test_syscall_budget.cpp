#include "flux/channel.hpp"
#include "flux/discovery.hpp"

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

// The RT claim: the borrow path does not enter the kernel while frames are flowing.
// Counting at the call sites we know about could only confirm what we already believe -- what
// this guards against is a syscall appearing somewhere nobody thought to instrument. ptrace
// counts what the process actually did, so any such addition shows up here.
//
// The assertions are on the rate, not an exact count: one syscall per borrow costs
// kIterations, none costs a handful of teardown calls, and the two are orders of magnitude
// apart. That gap is what makes this robust against libc and sanitizer noise; an exact count
// would not be.

namespace
{

constexpr std::uint64_t kFingerprint = 0xB0110C0DEULL;
constexpr std::uint32_t kSlotSize = 256;
constexpr std::uint32_t kSlotCount = 4;
constexpr int kIterations = 20000;
constexpr int kExitNoPtrace = 11;

enum class Mode { Flowing, Starved };

// Runs under ptrace. Everything the measurement must exclude happens before the second stop.
[[noreturn]] void child_main(const std::string & name, Mode mode)
{
  if (::ptrace(PTRACE_TRACEME, 0, nullptr, nullptr) != 0) ::_exit(kExitNoPtrace);
  ::raise(SIGSTOP);  // the tracer attaches here

  flux::Channel pub = flux::Channel::create(name, kSlotSize, kSlotCount, kFingerprint);
  flux::Channel sub = flux::Channel::open(name, kFingerprint);
  std::vector<std::byte> buf(kSlotSize, std::byte{0x11});
  if (pub.publish(buf.data(), buf.size()) != flux::Published::Ok) ::_exit(12);
  // Warm every lazy path a first borrow would otherwise pay for: identity, owner file,
  // signpost mapping, cursor placement.
  for (int i = 0; i < 64; ++i) {
    if (!sub.peek()) ::_exit(13);
  }

  ::raise(SIGSTOP);  // counting starts after this

  for (int i = 0; i < kIterations; ++i) {
    if (mode == Mode::Flowing) {
      if (pub.publish(buf.data(), buf.size()) != flux::Published::Ok) ::_exit(14);
      if (!sub.take()) ::_exit(15);
    } else {
      if (!sub.peek()) ::_exit(16);
    }
  }
  ::_exit(0);
}

// Syscalls the child made between its second stop and its exit, or -1 if ptrace is unavailable.
long measure(const std::string & name, Mode mode)
{
  pid_t pid = ::fork();
  if (pid < 0) return -2;
  if (pid == 0) child_main(name, mode);

  int status = 0;
  if (::waitpid(pid, &status, 0) != pid) return -2;
  if (WIFEXITED(status) && WEXITSTATUS(status) == kExitNoPtrace) return -1;
  if (!WIFSTOPPED(status)) return -2;
  if (::ptrace(PTRACE_SETOPTIONS, pid, 0, PTRACE_O_TRACESYSGOOD) != 0) return -1;

  // Setup runs without syscall stops; the second SIGSTOP opens the measured region.
  if (::ptrace(PTRACE_CONT, pid, 0, 0) != 0) return -2;
  if (::waitpid(pid, &status, 0) != pid) return -2;
  if (!WIFSTOPPED(status) || WSTOPSIG(status) != SIGSTOP) return -2;

  long stops = 0;
  for (;;) {
    if (::ptrace(PTRACE_SYSCALL, pid, 0, 0) != 0) return -2;
    if (::waitpid(pid, &status, 0) != pid) return -2;
    if (WIFEXITED(status) || WIFSIGNALED(status)) break;
    if (WSTOPSIG(status) == (SIGTRAP | 0x80)) ++stops;
  }

  ::shm_unlink(name.c_str());  // the signpost; sweep reclaims the segment the dead child made
  flux::sweep_dead();

  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) return -3 - WEXITSTATUS(status);
  return stops / 2;  // entry and exit are two stops for one syscall
}

}  // namespace

// A consumer keeping up with a publisher never enters the kernel: identity is cached per
// thread, rotation is read off a resident mapping, and a wake is skipped with no waiters.
TEST(SyscallBudget, BorrowDoesNotEnterTheKernelWhileFramesFlow)
{
  const std::string name = "/flux.test.syscall.flow." + std::to_string(::getpid());
  const long syscalls = measure(name, Mode::Flowing);
  if (syscalls == -1) GTEST_SKIP() << "flux-cap:ptrace ptrace is unavailable here";
  ASSERT_GE(syscalls, 0) << "the traced child failed, code " << syscalls;
  EXPECT_LT(syscalls, 200) << syscalls << " syscalls over " << kIterations
                           << " publish+take pairs; this path is meant to make none";
}

// A starved consumer does enter the kernel: every kProbeEvery stalls it asks whether the
// publisher group died (shm_open + try-lock + close). That is the
// recovery probe, and what matters is that it stays amortized rather than becoming per-borrow.
TEST(SyscallBudget, StarvedPollProbesAtAnAmortizedRate)
{
  const std::string name = "/flux.test.syscall.starve." + std::to_string(::getpid());
  const long syscalls = measure(name, Mode::Starved);
  if (syscalls == -1) GTEST_SKIP() << "flux-cap:ptrace ptrace is unavailable here";
  ASSERT_GE(syscalls, 0) << "the traced child failed, code " << syscalls;
  // Under one syscall per poll on average. Per-borrow probing would be three times kIterations.
  EXPECT_LT(syscalls, static_cast<long>(kIterations))
    << syscalls << " syscalls over " << kIterations << " starved polls; the probe is meant to be "
    << "amortized over a run of stalls, not paid on every one";
}
