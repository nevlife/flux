#include "check.h"

typedef struct
{
  flux_channel * ch;
  int fail;
  int delivered;
  int dropped;
} source_ctx;

static int32_t attach(void * ctx)
{
  return ((source_ctx *)ctx)->ch != NULL;
}

static int32_t deliver_one(void * ctx)
{
  source_ctx * s = ctx;
  if (s->fail) {
    return -5;
  }
  flux_frame v = {0};
  if (flux_channel_take(s->ch, &v) != FLUX_OK) {
    return 0;
  }
  flux_frame_release(&v);
  s->delivered++;
  return 1;
}

static const flux_channel * channel(void * ctx)
{
  return ((source_ctx *)ctx)->ch;
}

static void drop_ctx(void * ctx)
{
  ((source_ctx *)ctx)->dropped++;
}

static flux_source source(source_ctx * s)
{
  flux_source src = {sizeof(flux_source), s, attach, deliver_one, channel, drop_ctx, NULL, NULL};
  return src;
}

static void dispatches_and_drops_once(void)
{
  flux_executor * ex = test_executor();
  source_ctx s = {NULL, 0, 0, 0};
  CHECK(flux_channel_new(16, 4, &s.ch) == FLUX_OK);
  flux_qos q = flux_qos_default();
  q.depth = 1;
  CHECK(flux_channel_set_qos(s.ch, q) == FLUX_OK);
  CHECK(test_add(ex, source(&s), 0) == FLUX_OK);
  CHECK(flux_executor_size(ex) == 1);
  flux_channel_publish(s.ch, (const uint8_t *)"a", 1);
  flux_channel_publish(s.ch, (const uint8_t *)"b", 1);
  size_t n = 0;
  CHECK(flux_executor_dispatch(ex, &n) == FLUX_OK);
  CHECK(n == 1 && s.delivered == 1);  // depth 1 keeps only the newest
  CHECK(flux_executor_dispatch(ex, &n) == FLUX_OK && n == 0);
  flux_executor_free(ex);
  CHECK(s.dropped == 1);
  flux_channel_free(s.ch);
}

static void a_failing_callback_ends_the_spin(void)
{
  flux_executor * ex = test_executor();
  source_ctx s = {NULL, 1, 0, 0};
  CHECK(flux_channel_new(16, 4, &s.ch) == FLUX_OK);
  flux_source bad = source(&s);
  bad.deliver_one = NULL;
  CHECK(test_add(ex, bad, 0) == FLUX_INVALID_ARGUMENT);
  CHECK(test_add(ex, source(&s), 0) == FLUX_OK);
  flux_channel_publish(s.ch, (const uint8_t *)"a", 1);
  CHECK(flux_executor_spin(ex, 10000000) == FLUX_CALLBACK);
  CHECK(strstr(flux_last_error(), "-5") != NULL);
  flux_executor_free(ex);
  flux_channel_free(s.ch);
}

// Every failed add takes the source, so the caller never frees what drop also frees.
static void a_failed_add_drops_the_source(void)
{
  flux_executor * ex = test_executor();
  source_ctx s = {NULL, 0, 0, 0};
  flux_source bad = source(&s);
  bad.attach = NULL;
  CHECK(test_add(ex, bad, 0) == FLUX_INVALID_ARGUMENT);
  CHECK(s.dropped == 1);
  int added = 0;
  while (test_add(ex, source(&s), 0) == FLUX_OK) {
    added++;
  }
  CHECK(s.dropped == 2);
  flux_executor_free(ex);
  CHECK(s.dropped == 2 + added);
}

static int32_t refuse(void * ctx)
{
  (void)ctx;
  return -3;
}

static void a_failing_attach_ends_the_spin(void)
{
  flux_executor * ex = test_executor();
  source_ctx s = {NULL, 0, 0, 0};
  flux_source src = source(&s);
  src.attach = refuse;
  CHECK(test_add(ex, src, 0) == FLUX_OK);
  CHECK(flux_executor_spin(ex, 10000000) == FLUX_CALLBACK);
  CHECK(strstr(flux_last_error(), "attach returned -3") != NULL);
  flux_executor_free(ex);
}

// A source is read up to its size: a newer header's unknown bytes must be zero, and a refusal
// drops it once its size reaches drop.
static void a_source_is_read_up_to_its_size(void)
{
  flux_executor * ex = test_executor();
  source_ctx s = {NULL, 0, 0, 0};
  CHECK(flux_channel_new(16, 4, &s.ch) == FLUX_OK);
  struct
  {
    flux_source src;
    uint64_t newer;
  } big;
  memset(&big, 0, sizeof big);
  big.src = source(&s);
  big.src.size = sizeof big;
  CHECK(flux_executor_add(ex, &big.src, 0) == FLUX_OK);
  big.newer = 1;
  CHECK(flux_executor_add(ex, &big.src, 0) == FLUX_UNSUPPORTED);
  CHECK(s.dropped == 1);
  flux_source small = source(&s);
  small.size = 4;
  CHECK(flux_executor_add(ex, &small, 0) == FLUX_UNSUPPORTED);
  CHECK(s.dropped == 1);  // too small to reach its drop
  flux_executor_free(ex);
  flux_channel_free(s.ch);
}

int main(void)
{
  dispatches_and_drops_once();
  a_failing_callback_ends_the_spin();
  a_failing_attach_ends_the_spin();
  a_failed_add_drops_the_source();
  a_source_is_read_up_to_its_size();
  printf("ok\n");
  return 0;
}
