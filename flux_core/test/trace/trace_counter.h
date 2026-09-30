#ifndef FLUX_TEST_TRACE_COUNTER_H
#define FLUX_TEST_TRACE_COUNTER_H

#include <stdint.h>

struct flux_trace_counter
{
  uint64_t channel_init, claim, commit, take, release, refused, wake, callback_start, callback_end;

  const void * init_channel;
  int init_publisher;
  uint32_t init_slot_count;
  const void * claim_channel;
  int claim_result;
  const void * commit_channel;
  uint64_t commit_ticket;
  uint64_t commit_nbytes;
  const void * take_channel;
  uint64_t take_ticket;
  const void * release_channel;
  uint64_t release_ticket;
  int release_fenced;
  const void * refused_channel;
  int refused_reason;
  const void * wake_executor;
  const void * callback_executor;
  int callback_delivered;
};

#endif  // FLUX_TEST_TRACE_COUNTER_H
