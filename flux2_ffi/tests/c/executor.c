#include "check.h"

#include <pthread.h>
#include <stdatomic.h>

typedef struct
{
  flux2_channel * ch;
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
  if (s->fail) return -5;
  flux2_view v = {0};
  if (!flux2_channel_take(s->ch, &v)) return 0;
  flux2_view_release(&v);
  s->delivered++;
  return 1;
}

static const flux2_channel * channel(void * ctx)
{
  return ((source_ctx *)ctx)->ch;
}

static void drop_ctx(void * ctx)
{
  ((source_ctx *)ctx)->dropped++;
}

static flux2_source source(source_ctx * s)
{
  flux2_source src = {s, attach, deliver_one, channel, drop_ctx, NULL, NULL};
  return src;
}

static void dispatches_and_drops_once(void)
{
  flux2_executor * ex = test_executor();
  source_ctx s = {NULL, 0, 0, 0};
  CHECK(flux2_channel_new(16, 4, &s.ch) == FLUX2_OK);
  flux2_qos q = flux2_qos_default();
  q.depth = 1;
  CHECK(flux2_channel_set_qos(s.ch, q) == FLUX2_OK);
  CHECK(flux2_executor_add(ex, source(&s), 0) == FLUX2_OK);
  CHECK(flux2_executor_size(ex) == 1);
  flux2_channel_publish(s.ch, (const uint8_t *)"a", 1);
  flux2_channel_publish(s.ch, (const uint8_t *)"b", 1);
  size_t n = 0;
  CHECK(flux2_executor_dispatch(ex, &n) == FLUX2_OK);
  CHECK(n == 1 && s.delivered == 1);  // depth 1 keeps only the newest
  CHECK(flux2_executor_dispatch(ex, &n) == FLUX2_OK && n == 0);
  flux2_executor_free(ex);
  CHECK(s.dropped == 1);
  flux2_channel_free(s.ch);
}

static void a_failing_callback_ends_the_spin(void)
{
  flux2_executor * ex = test_executor();
  source_ctx s = {NULL, 1, 0, 0};
  CHECK(flux2_channel_new(16, 4, &s.ch) == FLUX2_OK);
  flux2_source bad = source(&s);
  bad.deliver_one = NULL;
  CHECK(flux2_executor_add(ex, bad, 0) == FLUX2_INVALID_ARGUMENT);
  CHECK(flux2_executor_add(ex, source(&s), 0) == FLUX2_OK);
  flux2_channel_publish(s.ch, (const uint8_t *)"a", 1);
  CHECK(flux2_executor_spin(ex, 10000000) == FLUX2_CALLBACK);
  CHECK(strstr(flux2_last_error(), "-5") != NULL);
  flux2_executor_free(ex);
  flux2_channel_free(s.ch);
}

// Every failed add takes the source, so the caller never frees what drop also frees.
static void a_failed_add_drops_the_source(void)
{
  flux2_executor * ex = test_executor();
  source_ctx s = {NULL, 0, 0, 0};
  flux2_source bad = source(&s);
  bad.attach = NULL;
  CHECK(flux2_executor_add(ex, bad, 0) == FLUX2_INVALID_ARGUMENT);
  CHECK(s.dropped == 1);
  int added = 0;
  while (flux2_executor_add(ex, source(&s), 0) == FLUX2_OK) added++;
  CHECK(s.dropped == 2);
  flux2_executor_free(ex);
  CHECK(s.dropped == 2 + added);
}

static int32_t refuse(void * ctx)
{
  (void)ctx;
  return -3;
}

static void a_failing_attach_ends_the_spin(void)
{
  flux2_executor * ex = test_executor();
  source_ctx s = {NULL, 0, 0, 0};
  flux2_source src = source(&s);
  src.attach = refuse;
  CHECK(flux2_executor_add(ex, src, 0) == FLUX2_OK);
  CHECK(flux2_executor_spin(ex, 10000000) == FLUX2_CALLBACK);
  CHECK(strstr(flux2_last_error(), "attach returned -3") != NULL);
  flux2_executor_free(ex);
}

static atomic_bool stopped;

static int32_t run_until_stopped(void * ctx)
{
  (void)ctx;
  while (!atomic_load(&stopped)) sched_yield();
  return 0;
}

static int32_t set_stop(void * ctx)
{
  (void)ctx;
  atomic_store(&stopped, true);
  return 0;
}

static void * spin(void * px)
{
  return (void *)(intptr_t)flux2_partitioned_spin(px, 10000000);
}

static void a_partitioned_stop_reaches_an_external_loop(void)
{
  flux2_executor * ex = test_executor();
  flux2_partitioned * px = NULL;
  size_t i = 9;
  CHECK(flux2_partitioned_new(&px) == FLUX2_OK);
  flux2_partitioned_add(px, ex, &i);
  CHECK(i == 0);
  CHECK(flux2_partitioned_add_external(px, run_until_stopped, set_stop, NULL, &i) == FLUX2_OK);
  CHECK(i == 1 && flux2_partitioned_size(px) == 2);
  CHECK(flux2_partitioned_on_thread_start(px, 7, set_stop, NULL) == FLUX2_INVALID_ARGUMENT);
  flux2_handle * h = flux2_partitioned_handle(px);
  pthread_t t;
  CHECK(pthread_create(&t, NULL, spin, px) == 0);
  while (!flux2_handle_is_spinning(h)) sched_yield();
  flux2_handle_stop(h);
  void * r = NULL;
  pthread_join(t, &r);
  CHECK((intptr_t)r == FLUX2_OK);
  CHECK(atomic_load(&stopped));
  flux2_handle_free(h);
  flux2_partitioned_free(px);
}

int main(void)
{
  dispatches_and_drops_once();
  a_failing_callback_ends_the_spin();
  a_failing_attach_ends_the_spin();
  a_failed_add_drops_the_source();
  a_partitioned_stop_reaches_an_external_loop();
  printf("ok\n");
  return 0;
}
