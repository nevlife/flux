#include "check.h"

#include <pthread.h>
#include <time.h>

static void heap_views_and_loans(void)
{
  CHECK(flux_abi_version() == FLUX_ABI_VERSION);
  flux_channel * ch = NULL;
  CHECK(flux_channel_new(64, 4, &ch) == FLUX_OK);
  CHECK(flux_channel_publish(ch, (const uint8_t *)"abc", 3) == FLUX_PUBLISHED_OK);

  flux_view v = {0};
  CHECK(flux_channel_take(ch, &v));
  size_t len = 0;
  const uint8_t * data = flux_view_data(&v, &len);
  CHECK(len == 3 && memcmp(data, "abc", 3) == 0);
  CHECK(flux_view_meta(&v)->dtype == FLUX_DTYPE_U8 && flux_view_meta(&v)->nbytes == 3);
  CHECK(flux_view_ticket(&v) == 1);
  flux_view_release(&v);
  CHECK(!flux_view_valid(&v) && flux_view_data(&v, &len) == NULL);

  flux_loan l = {0};
  const uint64_t dims[2] = {2, 3};
  CHECK(flux_channel_loan_typed(ch, FLUX_DTYPE_U16, dims, 2, &l));
  size_t cap = 0;
  uint8_t * p = flux_loan_data(&l, &cap);
  CHECK(cap == 64);
  memset(p, 9, 12);
  CHECK(flux_loan_commit(&l) == FLUX_PUBLISHED_OK);
  CHECK(!flux_loan_valid(&l));
  CHECK(flux_channel_take(ch, &v));
  const flux_frame_meta * m = flux_view_meta(&v);
  CHECK(m->dtype == FLUX_DTYPE_U16 && m->ndim == 2 && m->nbytes == 12 && m->shape[1] == 3);

  CHECK(flux_channel_loan(ch, &l));
  memset(flux_loan_data(&l, &cap), 7, 5);
  CHECK(flux_loan_commit_len(&l, 5) == FLUX_PUBLISHED_OK);
  CHECK(flux_loan_commit(&l) == FLUX_PUBLISHED_INVALID_ARGUMENT);

  CHECK(!flux_channel_loan_typed(ch, 200, dims, 2, &l));
  CHECK(flux_channel_publish_typed(ch, p, 1, 200, dims, 1) == FLUX_PUBLISHED_INVALID_ARGUMENT);

  // A view keeps its mapping alive past the channel handle.
  flux_channel_free(ch);
  CHECK(flux_view_data(&v, &len)[0] == 9);
  flux_view_release(&v);
  flux_channel_free(NULL);
}

static void failures_say_why(void)
{
  flux_channel * ch = NULL;
  CHECK(flux_channel_new(0, 1, &ch) == FLUX_INVALID_ARGUMENT);
  CHECK(flux_channel_new(16, 2, &ch) == FLUX_OK);
  flux_qos q = {0, 0, 1};
  CHECK(flux_channel_set_qos(ch, q) == FLUX_INVALID_ARGUMENT);
  CHECK(strstr(flux_last_error(), "depth") != NULL);
  q.depth = 2;
  q.replay = 1;
  CHECK(flux_channel_set_qos(ch, q) == FLUX_OK);
  CHECK(flux_channel_qos(ch).replay == 1);
  flux_channel_free(ch);

  char small[4];
  size_t len = 0;
  CHECK(flux_signpost_name("/k", 0, small, sizeof small, &len) == FLUX_INVALID_ARGUMENT);
  CHECK(len > sizeof small);
}

static void shm_publisher_and_subscriber(void)
{
  char name[256];
  test_signpost("shm", name, sizeof name);
  flux_memory_policy mem = {false, false};
  flux_channel *p = NULL, *p2 = NULL, *s = NULL;
  CHECK(flux_channel_open(name, 0, mem, NULL, FLUX_NO_STAMP, &s) == FLUX_NOT_READY);
  CHECK(
    flux_channel_create(
      name, 32, 2, FLUX_DEFAULT_MAX_SUBSCRIBERS, 0, 0, mem, NULL, FLUX_NO_STAMP, &p) == FLUX_OK);
  CHECK(
    flux_channel_create(
      name, 32, 2, FLUX_DEFAULT_MAX_SUBSCRIBERS, 0, 0, mem, NULL, FLUX_NO_STAMP, &p2) ==
    FLUX_PUBLISHER_EXISTS);
  CHECK(flux_channel_open(name, 0, mem, NULL, FLUX_NO_STAMP, &s) == FLUX_OK);
  CHECK(flux_channel_payload_readonly(s));
  // A bound on fences without a declared stream: nothing to wait for, nothing fails.
  flux_channel_set_fence_timeout(p, 1000000);
  flux_channel_set_fence_timeout(s, 1000000);
  // Deferred without a stream: nothing to defer, the release gives the slot back at once.
  CHECK(flux_channel_set_release_mode(s, FLUX_RELEASE_DEFERRED) == FLUX_OK);
  CHECK(flux_channel_set_release_mode(s, 7) == FLUX_INVALID_ARGUMENT);
  CHECK(flux_channel_publish(p, (const uint8_t *)"hi", 2) == FLUX_PUBLISHED_OK);
  flux_view v = {0};
  CHECK(flux_channel_take_blocking(s, 1000000000, &v));
  CHECK(flux_view_publish_ns(&v) > 0);
  flux_view_release(&v);
  CHECK(flux_channel_fence_failed(p) == 0 && flux_channel_fence_failed(s) == 0);

  flux_channel_stats st;
  CHECK(flux_read_channel_stats(name, &st) == FLUX_OK);
  CHECK(st.live && st.publish_seq == 1 && st.slot_size == 32);
  flux_channel_free(s);
  flux_channel_free(p);
  shm_unlink(name);
}

// The aux bytes a loan sets arrive with its frame; a length past the region is refused.
static void a_loan_carries_aux_bytes(void)
{
  char name[256];
  test_signpost("aux", name, sizeof name);
  flux_memory_policy mem = {false, false};
  flux_channel *p = NULL, *s = NULL;
  CHECK(
    flux_channel_create(
      name, 32, 2, FLUX_DEFAULT_MAX_SUBSCRIBERS, FLUX_MAX_AUX_SIZE + 1, 0, mem, NULL, FLUX_NO_STAMP,
      &p) == FLUX_INVALID_ARGUMENT);
  CHECK(
    flux_channel_create(
      name, 32, 2, FLUX_DEFAULT_MAX_SUBSCRIBERS, 40, 0, mem, NULL, FLUX_NO_STAMP, &p) == FLUX_OK);
  CHECK(flux_channel_open(name, 0, mem, NULL, FLUX_NO_STAMP, &s) == FLUX_OK);
  CHECK(flux_channel_aux_size(p) == 40 && flux_channel_aux_size(s) == 40);

  flux_loan l = {0};
  size_t cap = 0;
  CHECK(flux_loan_aux(&l, &cap) == NULL);
  CHECK(flux_channel_loan(p, &l));
  uint8_t * aux = flux_loan_aux(&l, &cap);
  CHECK(aux != NULL && cap == 40);
  memcpy(aux, "header", 6);
  CHECK(flux_loan_aux(&l, &cap) == aux);
  flux_loan_set_aux_len(&l, 6);
  memcpy(flux_loan_data(&l, &cap), "px", 2);
  CHECK(flux_loan_commit_len(&l, 2) == FLUX_PUBLISHED_OK);

  flux_view v = {0};
  size_t len = 0;
  CHECK(flux_view_aux(&v, &len) == NULL);
  CHECK(flux_channel_take(s, &v));
  const uint8_t * got = flux_view_aux(&v, &len);
  CHECK(len == 6 && memcmp(got, "header", 6) == 0);
  CHECK(flux_view_meta(&v)->aux_len == 6);
  CHECK(memcmp(flux_view_data(&v, &len), "px", 2) == 0 && len == 2);
  flux_view_release(&v);

  CHECK(flux_channel_loan(p, &l));
  flux_loan_set_aux_len(&l, 41);
  CHECK(flux_loan_commit(&l) == FLUX_PUBLISHED_TOO_LARGE);
  CHECK(flux_channel_publish(p, (const uint8_t *)"q", 1) == FLUX_PUBLISHED_OK);
  CHECK(flux_channel_take(s, &v));
  flux_view_aux(&v, &len);
  CHECK(len == 0);
  flux_view_release(&v);
  flux_channel_free(s);
  flux_channel_free(p);
  shm_unlink(name);
}

static uint64_t now_ns(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (uint64_t)t.tv_sec * 1000000000u + (uint64_t)t.tv_nsec;
}

typedef struct
{
  flux_channel * pub;
  uint64_t sent_ns;
} late_publish;

static void * publish_late(void * arg)
{
  late_publish * lp = arg;
  const struct timespec nap = {0, 20000000};
  nanosleep(&nap, NULL);
  lp->sent_ns = now_ns();
  CHECK(flux_channel_publish(lp->pub, (const uint8_t *)"w", 1) == FLUX_PUBLISHED_OK);
  return NULL;
}

// The publish itself wakes a parked take_blocking: a poll would show as a median of tens of ms.
static void take_blocking_wakes_on_the_publish(void)
{
  char name[256];
  test_signpost("wake", name, sizeof name);
  flux_memory_policy mem = {false, false};
  flux_channel *p = NULL, *s = NULL;
  CHECK(
    flux_channel_create(
      name, 32, 2, FLUX_DEFAULT_MAX_SUBSCRIBERS, 0, 0, mem, NULL, FLUX_NO_STAMP, &p) == FLUX_OK);
  CHECK(flux_channel_open(name, 0, mem, NULL, FLUX_NO_STAMP, &s) == FLUX_OK);
  enum { kRounds = 9 };
  uint64_t lat[kRounds];
  for (int i = 0; i < kRounds; ++i) {
    late_publish lp = {p, 0};
    pthread_t t;
    CHECK(pthread_create(&t, NULL, publish_late, &lp) == 0);
    flux_view v = {0};
    CHECK(flux_channel_take_blocking(s, 2000000000, &v));
    const uint64_t got = now_ns();
    CHECK(pthread_join(t, NULL) == 0);
    lat[i] = got - lp.sent_ns;
    flux_view_release(&v);
  }
  for (int i = 1; i < kRounds; ++i) {
    for (int j = i; j > 0 && lat[j - 1] > lat[j]; --j) {
      const uint64_t x = lat[j];
      lat[j] = lat[j - 1];
      lat[j - 1] = x;
    }
  }
  CHECK(lat[kRounds / 2] < 10000000);
  flux_channel_free(s);
  flux_channel_free(p);
  shm_unlink(name);
}

static pthread_key_t at_exit_key;

static void fail_at_exit(void * st)
{
  flux_channel * ch = NULL;
  *(flux_status *)st = flux_channel_new(0, 1, &ch);
  CHECK(flux_last_error() != NULL);
}

static void * fail_then_exit(void * st)
{
  flux_channel * ch = NULL;
  CHECK(flux_channel_new(0, 1, &ch) == FLUX_INVALID_ARGUMENT);
  CHECK(pthread_setspecific(at_exit_key, st) == 0);
  return NULL;
}

// glibc runs pthread key destructors after the thread's Rust storage is gone.
static void a_failure_during_thread_exit_returns_its_status(void)
{
  flux_status st = FLUX_OK;
  CHECK(pthread_key_create(&at_exit_key, fail_at_exit) == 0);
  pthread_t t;
  CHECK(pthread_create(&t, NULL, fail_then_exit, &st) == 0);
  CHECK(pthread_join(t, NULL) == 0);
  CHECK(st == FLUX_INVALID_ARGUMENT);
  CHECK(pthread_key_delete(at_exit_key) == 0);
}

static void * take_until_timeout(void * ch)
{
  flux_view v = {0};
  CHECK(!flux_channel_take_blocking(ch, 300000000, &v));
  return NULL;
}

// The other thread parks in take_blocking, so every call here meets the handle in use.
static void a_second_thread_is_refused(void)
{
  flux_channel * ch = NULL;
  CHECK(flux_channel_new(16, 4, &ch) == FLUX_OK);
  pthread_t t;
  CHECK(pthread_create(&t, NULL, take_until_timeout, ch) == 0);
  const struct timespec nap = {0, 100000000};
  nanosleep(&nap, NULL);
  CHECK(flux_channel_publish(ch, (const uint8_t *)"a", 1) == FLUX_PUBLISHED_IN_USE);
  flux_view v = {0};
  CHECK(!flux_channel_take(ch, &v));
  flux_loan l = {0};
  CHECK(!flux_channel_loan(ch, &l));
  CHECK(flux_channel_set_qos(ch, (flux_qos){1, 0, 2}) == FLUX_IN_USE);
  CHECK(pthread_join(t, NULL) == 0);
  CHECK(flux_channel_publish(ch, (const uint8_t *)"a", 1) == FLUX_PUBLISHED_OK);
  flux_channel_free(ch);
}

// A watch opened before the publisher: armed at 0, it sees the publisher's epoch, and a channel
// opened through it attaches.
static void a_watch_sees_the_publisher_come_up(void)
{
  char name[256];
  test_signpost("watch", name, sizeof name);
  flux_watch * w = NULL;
  CHECK(flux_watch_open(name, &w) == FLUX_OK);
  CHECK(flux_watch_arm(w) == 0 && flux_watch_epoch(w) == 0);
  CHECK(!flux_watch_wait(w, 1000000));
  const flux_qos q = {1, 0, 2};
  const flux_memory_policy mem = {0};
  flux_channel * s = NULL;
  CHECK(flux_channel_open_watched(w, 0, q, mem, NULL, FLUX_NO_STAMP, &s) == FLUX_NOT_READY);
  flux_channel * p = NULL;
  CHECK(
    flux_channel_create(
      name, 32, 2, FLUX_DEFAULT_MAX_SUBSCRIBERS, 0, 0, mem, NULL, FLUX_NO_STAMP, &p) == FLUX_OK);
  CHECK(flux_watch_wait(w, 0) && flux_watch_epoch(w) != 0);
  CHECK(flux_channel_open_watched(w, 0, q, mem, NULL, FLUX_NO_STAMP, &s) == FLUX_OK);
  CHECK(flux_channel_attach_status(s) == FLUX_OK);
  flux_channel_free(s);
  flux_channel_free(p);
  flux_watch_free(w);
}

int main(void)
{
  heap_views_and_loans();
  failures_say_why();
  shm_publisher_and_subscriber();
  a_loan_carries_aux_bytes();
  a_second_thread_is_refused();
  take_blocking_wakes_on_the_publish();
  a_failure_during_thread_exit_returns_its_status();
  a_watch_sees_the_publisher_come_up();
  printf("ok\n");
  return 0;
}
