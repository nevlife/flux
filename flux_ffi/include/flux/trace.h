/* flux trace contract 2: the events libflux_ffi exports for collectors (orma) to attach to by
 * name. Each function does nothing. flux_core/src/trace.rs defines them; the bindings call the
 * ones about endpoints, the GIL and recording. Strings are NUL-terminated and live until the call
 * returns; pointers other than strings are identities, never read. */
#ifndef FLUX_TRACE_H
#define FLUX_TRACE_H

#include <stdint.h>

#define FLUX_TRACE_CLAIM_OK 0
#define FLUX_TRACE_CLAIM_RECLAIMED 1
#define FLUX_TRACE_CLAIM_BACKPRESSURE 2
#define FLUX_TRACE_CLAIM_NOT_PUBLISHER 3
#define FLUX_TRACE_CLAIM_WRONG_DEVICE 4
#define FLUX_TRACE_CLAIM_TOO_LARGE 5
#define FLUX_TRACE_CLAIM_FENCE_FAILED 6
#define FLUX_TRACE_CLAIM_GPU_WEDGED 7

#define FLUX_TRACE_LOAN_COMMITTED 0
#define FLUX_TRACE_LOAN_ABORTED 1
#define FLUX_TRACE_LOAN_TOO_LARGE 2
#define FLUX_TRACE_LOAN_FENCE_FAILED 3
#define FLUX_TRACE_LOAN_NOT_PUBLISHER 4
#define FLUX_TRACE_LOAN_GPU_WEDGED 5

#define FLUX_TRACE_WAKE_TIMEOUT 0
#define FLUX_TRACE_WAKE_SOURCE 1
#define FLUX_TRACE_WAKE_CONTROL 2
#define FLUX_TRACE_WAKE_ALREADY 3
#define FLUX_TRACE_WAKE_SIGNAL 4

#define FLUX_TRACE_REFUSED_MAX_BORROW 0
#define FLUX_TRACE_REFUSED_NO_READER 1
#define FLUX_TRACE_REFUSED_CONTENDED 2
#define FLUX_TRACE_REFUSED_BAD_FRAME 3
#define FLUX_TRACE_REFUSED_FENCE 4
#define FLUX_TRACE_REFUSED_GPU_WEDGED 5

#define FLUX_TRACE_RECORD_CACHE_FULL 0
#define FLUX_TRACE_RECORD_WRITE_FAILED 1
#define FLUX_TRACE_RECORD_OVERRUN 2

#define FLUX_TRACE_NONE UINT32_MAX

#define FLUX_TRACE_ROLE_PUBLISHER 0
#define FLUX_TRACE_ROLE_SUBSCRIPTION 1
#define FLUX_TRACE_LANG_CPP 0
#define FLUX_TRACE_LANG_PYTHON 1

#ifdef __cplusplus
extern "C" {
#endif

void flux_trace_channel_init(
  const void * channel, uint64_t dev, uint64_t ino, const char * segment, int32_t publisher,
  uint32_t slot_count, uint32_t slot_size, int32_t stamp_offset);
void flux_trace_claim(const void * channel, uint32_t slot, int32_t result, int32_t loan);
void flux_trace_loan(const void * channel, uint32_t slot);
void flux_trace_loan_end(const void * channel, uint32_t slot, int32_t result);
void flux_trace_commit(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t nbytes, int32_t woke,
  uint64_t stamp);
void flux_trace_take(
  const void * channel, uint64_t ticket, uint32_t slot, uint64_t lost, uint64_t stamp);
void flux_trace_release(const void * channel, uint64_t ticket, int32_t held);
void flux_trace_refused(const void * channel, int32_t reason);
void flux_trace_gpu_wedged(int32_t result);
void flux_trace_wait(const void * executor, uint32_t sources, int64_t timeout_ns);
void flux_trace_wake(const void * executor, int32_t result, uint32_t entry);
void flux_trace_callback_start(const void * executor, uint32_t entry, const void * endpoint);
void flux_trace_callback_end(
  const void * executor, uint32_t entry, int32_t delivered, const void * channel,
  const char * segment);
void flux_trace_endpoint_init(
  const void * endpoint, const char * node, const char * key, const char * signpost,
  uint64_t fingerprint, int32_t role, int32_t lang);
void flux_trace_py_gil(int32_t phase);
void flux_trace_py_ready(const void * channel, uint64_t ticket);
void flux_trace_record_drop(const void * channel, uint64_t ticket, uint32_t count, int32_t reason);
void flux_trace_contract_2(void);

#ifdef __cplusplus
}
#endif

#endif /* FLUX_TRACE_H */
