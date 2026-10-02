// The provider is a separate library loaded with LD_PRELOAD, so this one needs no liblttng-ust
// and a process that never loads it pays one load and branch per event.
#define LTTNG_UST_TRACEPOINT_DEFINE
#define LTTNG_UST_TRACEPOINT_PROBE_DYNAMIC_LINKAGE
#include "flux/trace.h"
#include "flux_tp.h"

#define FLUX_TRACE_EXPORT __attribute__((visibility("default"), noinline))

FLUX_TRACE_EXPORT void flux_trace_channel_init(
  const void * channel, uint64_t dev, uint64_t ino, const char * segment, int publisher,
  uint32_t slot_count, uint32_t slot_size)
{
  lttng_ust_tracepoint(
    flux, channel_init, channel, dev, ino, segment, publisher, slot_count, slot_size);
}

FLUX_TRACE_EXPORT void flux_trace_claim(
  const void * channel, uint32_t slot, int result, int reclaimed)
{
  lttng_ust_tracepoint(flux, claim, channel, slot, result, reclaimed);
}

FLUX_TRACE_EXPORT void flux_trace_commit(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t nbytes, int woke)
{
  lttng_ust_tracepoint(flux, commit, channel, ticket, slot, nbytes, woke);
}

FLUX_TRACE_EXPORT void flux_trace_take(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t lost)
{
  lttng_ust_tracepoint(flux, take, channel, ticket, slot, lost);
}

FLUX_TRACE_EXPORT void flux_trace_release(const void * channel, uint64_t ticket, int fenced)
{
  lttng_ust_tracepoint(flux, release, channel, ticket, fenced);
}

FLUX_TRACE_EXPORT void flux_trace_refused(const void * channel, int reason)
{
  lttng_ust_tracepoint(flux, refused, channel, reason);
}

FLUX_TRACE_EXPORT void flux_trace_wake(const void * executor, uint32_t events)
{
  lttng_ust_tracepoint(flux, wake, executor, events);
}

FLUX_TRACE_EXPORT void flux_trace_callback_start(const void * executor, uint32_t entry)
{
  lttng_ust_tracepoint(flux, callback_start, executor, entry);
}

FLUX_TRACE_EXPORT void flux_trace_callback_end(
  const void * executor, uint32_t entry, int delivered, const void * channel)
{
  lttng_ust_tracepoint(flux, callback_end, executor, entry, delivered, channel);
}

FLUX_TRACE_EXPORT void flux_trace_py_gil(const void * executor)
{
  lttng_ust_tracepoint(flux, py_gil, executor);
}

FLUX_TRACE_EXPORT void flux_trace_contract_2(void)
{
}

FLUX_TRACE_EXPORT void flux_trace_py_ready(const void * channel)
{
  lttng_ust_tracepoint(flux, py_ready, channel);
}
