#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include "flux/executor.hpp"
#include "flux/owner.hpp"
#include "flux/qos.hpp"
#include "flux/trace.h"
#include "trace/trace_counter.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

// Runs with trace_counter.c in LD_PRELOAD, which is how armo attaches without a rebuild: each
// event has to reach a library interposed by name, from the call site that owns it.

namespace
{

class Trace : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using Get = flux_trace_counter * (*)();
    auto get = reinterpret_cast<Get>(::dlsym(RTLD_DEFAULT, "flux_trace_counter_get"));
    ASSERT_NE(get, nullptr) << "run with LD_PRELOAD=libflux_trace_counter.so";
    c_ = get();
    *c_ = flux_trace_counter{};
  }

  flux_trace_counter * c_ = nullptr;
};

class OneChannel : public flux::Source
{
public:
  bool attach() override { return true; }
  int deliver_one() override { return ch.take() ? 1 : 0; }
  flux::Channel * channel() noexcept override { return &ch; }

  flux::Channel ch{64, 4};
};

TEST_F(Trace, PublishTakeReleaseShareOneChannelAndTicket)
{
  flux::Channel ch(64, 4);
  ASSERT_EQ(c_->channel_init, 1u);
  const void * handle = c_->init_channel;
  EXPECT_EQ(c_->init_publisher, 0);
  EXPECT_EQ(c_->init_slot_count, 4u);

  const std::uint8_t bytes[8] = {};
  ASSERT_EQ(ch.publish(bytes, sizeof bytes), flux::Published::Ok);
  EXPECT_EQ(c_->claim, 1u);
  EXPECT_EQ(c_->claim_result, FLUX_TRACE_CLAIM_OK);
  EXPECT_EQ(c_->claim_channel, handle);
  EXPECT_EQ(c_->commit, 1u);
  EXPECT_EQ(c_->commit_channel, handle);
  EXPECT_EQ(c_->commit_nbytes, sizeof bytes);

  {
    flux::FrameView v = ch.take();
    ASSERT_TRUE(v);
    EXPECT_EQ(c_->take, 1u);
    EXPECT_EQ(c_->take_channel, handle);
    EXPECT_EQ(c_->take_ticket, c_->commit_ticket);
    EXPECT_EQ(c_->release, 0u);
  }
  EXPECT_EQ(c_->release, 1u);
  EXPECT_EQ(c_->release_channel, handle);
  EXPECT_EQ(c_->release_ticket, c_->commit_ticket);
  EXPECT_EQ(c_->release_fenced, 1);
}

TEST_F(Trace, ClaimReportsBackpressure)
{
  flux::Channel ch(64, 1);
  const std::uint8_t bytes[8] = {};
  ASSERT_EQ(ch.publish(bytes, sizeof bytes), flux::Published::Ok);
  flux::FrameView held = ch.take();
  ASSERT_TRUE(held);
  ASSERT_EQ(ch.publish(bytes, sizeof bytes), flux::Published::Backpressure);
  EXPECT_EQ(c_->claim, 2u);
  EXPECT_EQ(c_->claim_result, FLUX_TRACE_CLAIM_BACKPRESSURE);
  EXPECT_EQ(c_->commit, 1u);
}

TEST_F(Trace, RefusedNamesTheReason)
{
  flux::Channel ch(64, 4);
  ch.qos(flux::QoS{}.max_borrow(1));
  const std::uint8_t bytes[8] = {};
  ASSERT_EQ(ch.publish(bytes, sizeof bytes), flux::Published::Ok);
  flux::FrameView held = ch.take();
  ASSERT_TRUE(held);
  EXPECT_FALSE(ch.take());
  EXPECT_EQ(c_->refused, 1u);
  EXPECT_EQ(c_->refused_channel, c_->init_channel);
  EXPECT_EQ(c_->refused_reason, FLUX_TRACE_REFUSED_MAX_BORROW);
}

TEST_F(Trace, ExecutorBracketsEachDelivery)
{
  OneChannel src;
  flux::Executor ex;
  ex.add(src);
  const std::uint8_t bytes[8] = {};
  ASSERT_EQ(src.ch.publish(bytes, sizeof bytes), flux::Published::Ok);

  ex.wait_for_work(0);
  EXPECT_GE(c_->wake, 1u);
  EXPECT_EQ(c_->wake_executor, &ex);

  ASSERT_EQ(ex.dispatch(), 1);
  EXPECT_GE(c_->callback_start, 1u);
  EXPECT_EQ(c_->callback_start, c_->callback_end);
  EXPECT_EQ(c_->callback_executor, &ex);
  EXPECT_EQ(c_->callback_delivered, 1);
  EXPECT_EQ(c_->take, 1u);
  EXPECT_EQ(c_->callback_channel, c_->init_channel);
  EXPECT_EQ(c_->callback_channel, src.ch.trace_handle());
}

std::string read_owner_file()
{
  int fd = ::shm_open(flux::owner_file_name(flux::OwnerFile::self()).c_str(), O_RDONLY, 0600);
  if (fd < 0) return {};
  std::string text;
  char buf[4096];
  for (ssize_t n; (n = ::read(fd, buf, sizeof(buf))) > 0;)
    text.append(buf, static_cast<std::size_t>(n));
  ::close(fd);
  return text;
}

// A tracer that attaches after channel_init reads the handle-to-segment map from here instead.
TEST_F(Trace, SegmentChannelIsRecordedInTheOwnerFile)
{
  const std::string sp =
    flux::signpost_name("/flux_test/trace_record." + std::to_string(::getpid()), 0x7AC);
  {
    flux::Channel pub = flux::Channel::create(sp, 64, 4, 0x7AC);
    char head[64];
    std::snprintf(
      head, sizeof(head), "\tch\t%" PRIxPTR "\t",
      reinterpret_cast<std::uintptr_t>(c_->init_channel));
    const std::string text = read_owner_file();
    const std::size_t at = text.rfind(head);  // the latest record for an address wins
    ASSERT_NE(at, std::string::npos) << text;
    const std::string line = text.substr(at, text.find('\n', at) - at);
    EXPECT_NE(line.find("\t1\t4\t64\t" + sp + "."), std::string::npos) << line;

    for (const auto & e :
         flux::OwnerFile::read_manifest(flux::owner_file_name(flux::OwnerFile::self()))) {
      EXPECT_FALSE(e.signpost.empty()) << "a channel record was read as an endpoint";
    }
  }
  ::shm_unlink(sp.c_str());
}

// A heap channel can take the address of a recorded shm channel; its own record has to come last.
TEST_F(Trace, HeapChannelIsRecordedWithoutASegment)
{
  const std::string sp =
    flux::signpost_name("/flux_test/trace_heap." + std::to_string(::getpid()), 0x7AD);
  {
    flux::Channel pub = flux::Channel::create(sp, 64, 4, 0x7AD);
  }  // the owner file exists now
  flux::Channel ch(64, 4);
  char head[64];
  std::snprintf(
    head, sizeof(head), "\tch\t%" PRIxPTR "\t", reinterpret_cast<std::uintptr_t>(c_->init_channel));
  const std::string text = read_owner_file();
  const std::size_t at = text.rfind(head);
  ASSERT_NE(at, std::string::npos) << text;
  EXPECT_EQ(text.substr(at + std::strlen(head)), "0\t0\t0\t4\t64\t\n");
  ::shm_unlink(sp.c_str());
}

}  // namespace
