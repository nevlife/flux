// Drives every trace event once or more for tests/abi.rs, which reads what trace_recorder.c logged
// and checks the values: a build that leaves argument registers unset shows up as wrong values.
#include "check.h"
#include "flux2/trace.h"

#include <stdint.h>

#define STAMP_AT 4
#define FRAMES 5

static flux2_channel * sub;
static int endpoint;

static int32_t attach(void * ctx)
{
  (void)ctx;
  return 1;
}

static int32_t deliver_one(void * ctx)
{
  (void)ctx;
  flux2_view v = {0};
  const bool got = flux2_channel_take(sub, &v);
  flux2_view_release(&v);
  return got ? 1 : 0;
}

static const flux2_channel * channel(void * ctx)
{
  (void)ctx;
  return sub;
}

static uint64_t stamp_of(uint32_t i)
{
  return (uint64_t)i | ((uint64_t)(i * 1000U) << 32U);
}

static void publish(flux2_channel * p, uint32_t i)
{
  uint8_t frame[16] = {0};
  const uint64_t stamp = stamp_of(i);
  memcpy(frame + STAMP_AT, &stamp, sizeof stamp);
  CHECK(flux2_channel_publish(p, frame, sizeof frame) == FLUX2_PUBLISHED_OK);
}

int main(void)
{
  char name[256];
  test_signpost("trace_args", name, sizeof name);
  const flux2_memory_policy mem = {false, false};
  flux2_channel * p = NULL;
  CHECK(flux2_channel_create(name, 64, 4, 4, 0, mem, NULL, STAMP_AT, &p) == FLUX2_OK);
  CHECK(flux2_channel_open(name, 0, mem, NULL, STAMP_AT, &sub) == FLUX2_OK);

  const void * id = NULL;
  for (uint32_t i = 1; i <= FRAMES; ++i) {
    publish(p, i);
    flux2_view v = {0};
    CHECK(flux2_channel_take(sub, &v));
    id = flux2_view_trace_channel(&v);
    flux2_trace_py_ready(id, flux2_view_ticket(&v));
    flux2_view_release(&v);
  }

  flux2_loan l = {0};
  CHECK(flux2_channel_loan(p, &l));
  size_t cap = 0;
  uint8_t * d = flux2_loan_data(&l, &cap);
  const uint64_t stamp = stamp_of(FRAMES + 1);
  memset(d, 0, 16);
  memcpy(d + STAMP_AT, &stamp, sizeof stamp);
  CHECK(flux2_loan_commit_len(&l, 16) == FLUX2_PUBLISHED_OK);
  flux2_loan_abort(&l);
  CHECK(flux2_channel_loan(p, &l));
  flux2_loan_abort(&l);
  uint8_t big[65] = {0};
  CHECK(flux2_channel_publish(p, big, sizeof big) == FLUX2_PUBLISHED_TOO_LARGE);

  flux2_executor * ex = test_executor();
  const flux2_source src = {NULL, attach, deliver_one, channel, NULL, NULL, &endpoint};
  CHECK(flux2_executor_add(ex, src, 0) == FLUX2_OK);
  size_t n = 0;
  CHECK(flux2_executor_dispatch(ex, &n) == FLUX2_OK && n == 1);
  CHECK(flux2_executor_wait_for_work(ex, 0) == FLUX2_OK);
  flux2_executor_free(ex);

  flux2_trace_endpoint_init(
    &endpoint, "/ns/node", "/key", name, 0xF00D, FLUX2_TRACE_ROLE_SUBSCRIPTION,
    FLUX2_TRACE_LANG_CPP);
  flux2_trace_py_gil(0);
  flux2_trace_py_gil(1);
  flux2_trace_record_drop(id, 0, 3, FLUX2_TRACE_RECORD_CACHE_FULL);
  flux2_trace_gpu_wedged(700);
  flux2_trace_contract_2();

  flux2_channel_free(sub);
  flux2_channel_free(p);
  return 0;
}
