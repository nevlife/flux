#ifndef FLUX_TRACE_H
#define FLUX_TRACE_H

#include <stdint.h>

// Defined in libflux_tracetools.so, not in the static flux_core, so a uprobe or an LD_PRELOAD
// library can attach to each event by name. The names and arguments are that contract.

#ifdef __cplusplus
extern "C" {
#endif

enum flux_trace_claim_result {
  FLUX_TRACE_CLAIM_OK = 0,
  FLUX_TRACE_CLAIM_BACKPRESSURE = 1,
  FLUX_TRACE_CLAIM_NO_IDENTITY = 2
};

enum flux_trace_refusal {
  FLUX_TRACE_REFUSED_MAX_BORROW = 0,
  FLUX_TRACE_REFUSED_FENCE = 1,
  FLUX_TRACE_REFUSED_NO_OWNER_FILE = 2,
  FLUX_TRACE_REFUSED_BAD_FRAME = 3,
  FLUX_TRACE_REFUSED_HOLDER_TABLE = 4,
  FLUX_TRACE_REFUSED_CONTENDED = 5
};

void flux_trace_channel_init(
  const void * channel, uint64_t dev, uint64_t ino, const char * segment, int publisher,
  uint32_t slot_count, uint32_t slot_size);
void flux_trace_claim(const void * channel, uint32_t slot, int result, int reclaimed);
void flux_trace_commit(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t nbytes, int woke);
void flux_trace_take(const void * channel, uint64_t ticket, uint32_t slot, uint64_t lost);
void flux_trace_release(const void * channel, uint64_t ticket, int fenced);
void flux_trace_refused(const void * channel, int reason);
void flux_trace_wake(const void * executor, uint32_t events);
void flux_trace_callback_start(const void * executor, uint32_t entry);
void flux_trace_callback_end(
  const void * executor, uint32_t entry, int delivered, const void * channel);
void flux_trace_py_gil(const void * executor);
void flux_trace_py_ready(const void * channel);

// Emits nothing. A tool checks for this name before attaching: the number changes whenever an
// event's name or arguments change, so a library from another version is refused, not misread.
void flux_trace_contract_2(void);

#ifdef __cplusplus
}
#endif

#ifdef FLUX_TRACING
#define FLUX_TRACE(call) call
#else
#define FLUX_TRACE(call) ((void)0)
#endif

#endif  // FLUX_TRACE_H
