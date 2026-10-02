// Loaded with LD_PRELOAD in front of libflux_tracetools.so, the way an external tool attaches.
// Records each event and forwards it, resolving the next definition on first call: a process
// may load libflux_tracetools.so after this library's constructors have run.
#include "trace_counter.h"

#include "flux/trace.h"

#include <dlfcn.h>

static struct flux_trace_counter counter;

__attribute__((visibility("default"))) struct flux_trace_counter * flux_trace_counter_get(void)
{
  return &counter;
}

#define NEXT(fn) ((__typeof__(&(fn)))dlsym(RTLD_NEXT, #fn))

void flux_trace_channel_init(
  const void * channel, uint64_t dev, uint64_t ino, const char * segment, int publisher,
  uint32_t slot_count, uint32_t slot_size)
{
  ++counter.channel_init;
  counter.init_channel = channel;
  counter.init_publisher = publisher;
  counter.init_slot_count = slot_count;
  static __typeof__(&flux_trace_channel_init) next;
  if (next == 0) next = NEXT(flux_trace_channel_init);
  if (next != 0) next(channel, dev, ino, segment, publisher, slot_count, slot_size);
}

void flux_trace_claim(const void * channel, uint32_t slot, int result, int reclaimed)
{
  ++counter.claim;
  counter.claim_channel = channel;
  counter.claim_result = result;
  static __typeof__(&flux_trace_claim) next;
  if (next == 0) next = NEXT(flux_trace_claim);
  if (next != 0) next(channel, slot, result, reclaimed);
}

void flux_trace_commit(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t nbytes, int woke)
{
  ++counter.commit;
  counter.commit_channel = channel;
  counter.commit_ticket = ticket;
  counter.commit_nbytes = nbytes;
  static __typeof__(&flux_trace_commit) next;
  if (next == 0) next = NEXT(flux_trace_commit);
  if (next != 0) next(channel, ticket, slot, nbytes, woke);
}

void flux_trace_take(const void * channel, uint64_t ticket, uint32_t slot, uint64_t lost)
{
  ++counter.take;
  counter.take_channel = channel;
  counter.take_ticket = ticket;
  static __typeof__(&flux_trace_take) next;
  if (next == 0) next = NEXT(flux_trace_take);
  if (next != 0) next(channel, ticket, slot, lost);
}

void flux_trace_release(const void * channel, uint64_t ticket, int fenced)
{
  ++counter.release;
  counter.release_channel = channel;
  counter.release_ticket = ticket;
  counter.release_fenced = fenced;
  static __typeof__(&flux_trace_release) next;
  if (next == 0) next = NEXT(flux_trace_release);
  if (next != 0) next(channel, ticket, fenced);
}

void flux_trace_refused(const void * channel, int reason)
{
  ++counter.refused;
  counter.refused_channel = channel;
  counter.refused_reason = reason;
  static __typeof__(&flux_trace_refused) next;
  if (next == 0) next = NEXT(flux_trace_refused);
  if (next != 0) next(channel, reason);
}

void flux_trace_wake(const void * executor, uint32_t events)
{
  ++counter.wake;
  counter.wake_executor = executor;
  static __typeof__(&flux_trace_wake) next;
  if (next == 0) next = NEXT(flux_trace_wake);
  if (next != 0) next(executor, events);
}

void flux_trace_callback_start(const void * executor, uint32_t entry)
{
  ++counter.callback_start;
  counter.callback_executor = executor;
  static __typeof__(&flux_trace_callback_start) next;
  if (next == 0) next = NEXT(flux_trace_callback_start);
  if (next != 0) next(executor, entry);
}

void flux_trace_callback_end(
  const void * executor, uint32_t entry, int delivered, const void * channel)
{
  ++counter.callback_end;
  if (delivered > 0) counter.callback_delivered += delivered;
  counter.callback_channel = channel;
  static __typeof__(&flux_trace_callback_end) next;
  if (next == 0) next = NEXT(flux_trace_callback_end);
  if (next != 0) next(executor, entry, delivered, channel);
}
