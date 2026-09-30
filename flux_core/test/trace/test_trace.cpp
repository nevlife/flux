#include "flux/channel.hpp"
#include "flux/executor.hpp"
#include "flux/qos.hpp"
#include "flux/trace.h"
#include "trace/trace_counter.h"

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <cstdint>

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
}

}  // namespace
