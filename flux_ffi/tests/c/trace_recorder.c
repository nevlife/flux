// Stands in for a collector: defines every flux_trace_* and writes each call's arguments, one line
// per event, to the file FLUX_TRACE_LOG names. LD_PRELOAD puts it in front of a shared library;
// linked before a static library whose trace symbols were made weak, it replaces them.
#include "flux/trace.h"

#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

static FILE * out;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

__attribute__((constructor)) static void open_log(void)
{
  const char * path = getenv("FLUX_TRACE_LOG");
  if (path == NULL || (out = fopen(path, "w")) == NULL) {
    abort();
  }
}

__attribute__((destructor)) static void close_log(void)
{
  fclose(out);
}

#define LOG(...)                 \
  do {                           \
    pthread_mutex_lock(&lock);   \
    fprintf(out, __VA_ARGS__);   \
    fputc('\n', out);            \
    fflush(out);                 \
    pthread_mutex_unlock(&lock); \
  } while (0)

#define P(p) ((uint64_t)(uintptr_t)(p))

void flux_trace_channel_init(
  const void * channel, uint64_t dev, uint64_t ino, const char * segment, int32_t publisher,
  uint32_t slot_count, uint32_t slot_size, int32_t stamp_offset)
{
  LOG(
    "channel_init %" PRIu64 " %" PRIu64 " %" PRIu64 " s:%s %d %u %u %d", P(channel), dev, ino,
    segment, publisher, slot_count, slot_size, stamp_offset);
}

void flux_trace_claim(const void * channel, uint32_t slot, int32_t result, int32_t loan)
{
  LOG("claim %" PRIu64 " %u %d %d", P(channel), slot, result, loan);
}

void flux_trace_loan(const void * channel, uint32_t slot)
{
  LOG("loan %" PRIu64 " %u", P(channel), slot);
}

void flux_trace_loan_end(const void * channel, uint32_t slot, int32_t result)
{
  LOG("loan_end %" PRIu64 " %u %d", P(channel), slot, result);
}

void flux_trace_commit(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t nbytes, int32_t woke,
  uint64_t stamp)
{
  LOG(
    "commit %" PRIu64 " %" PRIu64 " %u %" PRIu64 " %d %" PRIu64, P(channel), ticket, slot, nbytes,
    woke, stamp);
}

void flux_trace_take(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t lost, uint64_t stamp)
{
  LOG(
    "take %" PRIu64 " %" PRIu64 " %u %" PRIu64 " %" PRIu64, P(channel), ticket, slot, lost, stamp);
}

void flux_trace_release(const void * channel, uint64_t ticket, int32_t held)
{
  LOG("release %" PRIu64 " %" PRIu64 " %d", P(channel), ticket, held);
}

void flux_trace_refused(const void * channel, int32_t reason)
{
  LOG("refused %" PRIu64 " %d", P(channel), reason);
}

void flux_trace_gpu_wedged(int32_t result)
{
  LOG("gpu_wedged %d", result);
}

void flux_trace_wait(const void * executor, uint32_t sources, int64_t timeout_ns)
{
  LOG("wait %" PRIu64 " %u %" PRId64, P(executor), sources, timeout_ns);
}

void flux_trace_wake(const void * executor, int32_t result, uint32_t entry)
{
  LOG("wake %" PRIu64 " %d %u", P(executor), result, entry);
}

void flux_trace_callback_start(const void * executor, uint32_t entry, const void * endpoint)
{
  LOG("callback_start %" PRIu64 " %u %" PRIu64, P(executor), entry, P(endpoint));
}

void flux_trace_callback_end(
  const void * executor, uint32_t entry, int32_t delivered, const void * channel,
  const char * segment)
{
  LOG(
    "callback_end %" PRIu64 " %u %d %" PRIu64 " s:%s", P(executor), entry, delivered, P(channel),
    segment);
}

void flux_trace_endpoint_init(
  const void * endpoint, const char * node, const char * key, const char * signpost,
  uint64_t fingerprint, int32_t role, int32_t lang)
{
  LOG(
    "endpoint_init %" PRIu64 " s:%s s:%s s:%s %" PRIu64 " %d %d", P(endpoint), node, key, signpost,
    fingerprint, role, lang);
}

void flux_trace_py_gil(int32_t phase)
{
  LOG("py_gil %d", phase);
}

void flux_trace_py_ready(const void * channel, uint64_t ticket)
{
  LOG("py_ready %" PRIu64 " %" PRIu64, P(channel), ticket);
}

void flux_trace_record_drop(const void * channel, uint64_t ticket, uint32_t count, int32_t reason)
{
  LOG("record_drop %" PRIu64 " %" PRIu64 " %u %d", P(channel), ticket, count, reason);
}

void flux_trace_contract_2(void)
{
  LOG("contract_2");
}
