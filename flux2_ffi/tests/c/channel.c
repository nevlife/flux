#include "check.h"

#include <pthread.h>
#include <time.h>

static void heap_views_and_loans(void)
{
  CHECK(flux2_abi_version() == FLUX2_ABI_VERSION);
  flux2_channel * ch = NULL;
  CHECK(flux2_channel_new(64, 4, &ch) == FLUX2_OK);
  CHECK(flux2_channel_publish(ch, (const uint8_t *)"abc", 3) == FLUX2_PUBLISHED_OK);

  flux2_view v = {0};
  CHECK(flux2_channel_take(ch, &v));
  size_t len = 0;
  const uint8_t * data = flux2_view_data(&v, &len);
  CHECK(len == 3 && memcmp(data, "abc", 3) == 0);
  CHECK(flux2_view_meta(&v)->dtype == FLUX2_DTYPE_U8 && flux2_view_meta(&v)->nbytes == 3);
  CHECK(flux2_view_ticket(&v) == 1);
  flux2_view_release(&v);
  CHECK(!flux2_view_valid(&v) && flux2_view_data(&v, &len) == NULL);

  flux2_loan l = {0};
  const uint64_t dims[2] = {2, 3};
  CHECK(flux2_channel_loan_typed(ch, FLUX2_DTYPE_U16, dims, 2, &l));
  size_t cap = 0;
  uint8_t * p = flux2_loan_data(&l, &cap);
  CHECK(cap == 64);
  memset(p, 9, 12);
  CHECK(flux2_loan_commit(&l) == FLUX2_PUBLISHED_OK);
  CHECK(!flux2_loan_valid(&l));
  CHECK(flux2_channel_take(ch, &v));
  const flux2_frame_meta * m = flux2_view_meta(&v);
  CHECK(m->dtype == FLUX2_DTYPE_U16 && m->ndim == 2 && m->nbytes == 12 && m->shape[1] == 3);

  CHECK(flux2_channel_loan(ch, &l));
  memset(flux2_loan_data(&l, &cap), 7, 5);
  CHECK(flux2_loan_commit_len(&l, 5) == FLUX2_PUBLISHED_OK);
  CHECK(flux2_loan_commit(&l) == FLUX2_PUBLISHED_INVALID_ARGUMENT);

  CHECK(!flux2_channel_loan_typed(ch, 200, dims, 2, &l));
  CHECK(flux2_channel_publish_typed(ch, p, 1, 200, dims, 1) == FLUX2_PUBLISHED_INVALID_ARGUMENT);

  // A view keeps its mapping alive past the channel handle.
  flux2_channel_free(ch);
  CHECK(flux2_view_data(&v, &len)[0] == 9);
  flux2_view_release(&v);
  flux2_channel_free(NULL);
}

static void failures_say_why(void)
{
  flux2_channel * ch = NULL;
  CHECK(flux2_channel_new(0, 1, &ch) == FLUX2_INVALID_ARGUMENT);
  CHECK(flux2_channel_new(16, 2, &ch) == FLUX2_OK);
  flux2_qos q = {0, 0, 1};
  CHECK(flux2_channel_set_qos(ch, q) == FLUX2_INVALID_ARGUMENT);
  CHECK(strstr(flux2_last_error(), "depth") != NULL);
  q.depth = 2;
  q.replay = 1;
  CHECK(flux2_channel_set_qos(ch, q) == FLUX2_OK);
  CHECK(flux2_channel_qos(ch).replay == 1);
  flux2_channel_free(ch);

  char small[4];
  size_t len = 0;
  CHECK(flux2_signpost_name("/k", 0, small, sizeof small, &len) == FLUX2_INVALID_ARGUMENT);
  CHECK(len > sizeof small);
}

static void shm_publisher_and_subscriber(void)
{
  char name[256];
  test_signpost("shm", name, sizeof name);
  flux2_memory_policy mem = {false, false};
  flux2_channel *p = NULL, *p2 = NULL, *s = NULL;
  CHECK(flux2_channel_open(name, 0, mem, NULL, FLUX2_NO_STAMP, &s) == FLUX2_NOT_READY);
  CHECK(
    flux2_channel_create(
      name, 32, 2, FLUX2_DEFAULT_MAX_SUBSCRIBERS, 0, mem, NULL, FLUX2_NO_STAMP, &p) == FLUX2_OK);
  CHECK(
    flux2_channel_create(
      name, 32, 2, FLUX2_DEFAULT_MAX_SUBSCRIBERS, 0, mem, NULL, FLUX2_NO_STAMP, &p2) ==
    FLUX2_PUBLISHER_EXISTS);
  CHECK(flux2_channel_open(name, 0, mem, NULL, FLUX2_NO_STAMP, &s) == FLUX2_OK);
  CHECK(flux2_channel_payload_readonly(s));
  // A bound on fences without a declared stream: nothing to wait for, nothing fails.
  flux2_channel_set_fence_timeout(p, 1000000);
  flux2_channel_set_fence_timeout(s, 1000000);
  // Deferred without a stream: nothing to defer, the release gives the slot back at once.
  CHECK(flux2_channel_set_release_mode(s, FLUX2_RELEASE_DEFERRED) == FLUX2_OK);
  CHECK(flux2_channel_set_release_mode(s, 7) == FLUX2_INVALID_ARGUMENT);
  CHECK(flux2_channel_publish(p, (const uint8_t *)"hi", 2) == FLUX2_PUBLISHED_OK);
  flux2_view v = {0};
  CHECK(flux2_channel_take_blocking(s, 1000000000, &v));
  CHECK(flux2_view_publish_ns(&v) > 0);
  flux2_view_release(&v);
  CHECK(flux2_channel_fence_failed(p) == 0 && flux2_channel_fence_failed(s) == 0);

  flux2_channel_stats st;
  CHECK(flux2_read_channel_stats(name, &st) == FLUX2_OK);
  CHECK(st.live && st.publish_seq == 1 && st.slot_size == 32);
  flux2_channel_free(s);
  flux2_channel_free(p);
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
  flux2_channel * pub;
  uint64_t sent_ns;
} late_publish;

static void * publish_late(void * arg)
{
  late_publish * lp = arg;
  const struct timespec nap = {0, 20000000};
  nanosleep(&nap, NULL);
  lp->sent_ns = now_ns();
  CHECK(flux2_channel_publish(lp->pub, (const uint8_t *)"w", 1) == FLUX2_PUBLISHED_OK);
  return NULL;
}

// The publish itself wakes a parked take_blocking: a poll would show as a median of tens of ms.
static void take_blocking_wakes_on_the_publish(void)
{
  char name[256];
  test_signpost("wake", name, sizeof name);
  flux2_memory_policy mem = {false, false};
  flux2_channel *p = NULL, *s = NULL;
  CHECK(
    flux2_channel_create(
      name, 32, 2, FLUX2_DEFAULT_MAX_SUBSCRIBERS, 0, mem, NULL, FLUX2_NO_STAMP, &p) == FLUX2_OK);
  CHECK(flux2_channel_open(name, 0, mem, NULL, FLUX2_NO_STAMP, &s) == FLUX2_OK);
  enum { kRounds = 9 };
  uint64_t lat[kRounds];
  for (int i = 0; i < kRounds; ++i) {
    late_publish lp = {p, 0};
    pthread_t t;
    CHECK(pthread_create(&t, NULL, publish_late, &lp) == 0);
    flux2_view v = {0};
    CHECK(flux2_channel_take_blocking(s, 2000000000, &v));
    const uint64_t got = now_ns();
    CHECK(pthread_join(t, NULL) == 0);
    lat[i] = got - lp.sent_ns;
    flux2_view_release(&v);
  }
  for (int i = 1; i < kRounds; ++i) {
    for (int j = i; j > 0 && lat[j - 1] > lat[j]; --j) {
      const uint64_t x = lat[j];
      lat[j] = lat[j - 1];
      lat[j - 1] = x;
    }
  }
  CHECK(lat[kRounds / 2] < 10000000);
  flux2_channel_free(s);
  flux2_channel_free(p);
  shm_unlink(name);
}

static pthread_key_t at_exit_key;

static void fail_at_exit(void * st)
{
  flux2_channel * ch = NULL;
  *(flux2_status *)st = flux2_channel_new(0, 1, &ch);
  CHECK(flux2_last_error() != NULL);
}

static void * fail_then_exit(void * st)
{
  flux2_channel * ch = NULL;
  CHECK(flux2_channel_new(0, 1, &ch) == FLUX2_INVALID_ARGUMENT);
  CHECK(pthread_setspecific(at_exit_key, st) == 0);
  return NULL;
}

// glibc runs pthread key destructors after the thread's Rust storage is gone.
static void a_failure_during_thread_exit_returns_its_status(void)
{
  flux2_status st = FLUX2_OK;
  CHECK(pthread_key_create(&at_exit_key, fail_at_exit) == 0);
  pthread_t t;
  CHECK(pthread_create(&t, NULL, fail_then_exit, &st) == 0);
  CHECK(pthread_join(t, NULL) == 0);
  CHECK(st == FLUX2_INVALID_ARGUMENT);
  CHECK(pthread_key_delete(at_exit_key) == 0);
}

static void * take_until_timeout(void * ch)
{
  flux2_view v = {0};
  CHECK(!flux2_channel_take_blocking(ch, 300000000, &v));
  return NULL;
}

// The other thread parks in take_blocking, so every call here meets the handle in use.
static void a_second_thread_is_refused(void)
{
  flux2_channel * ch = NULL;
  CHECK(flux2_channel_new(16, 4, &ch) == FLUX2_OK);
  pthread_t t;
  CHECK(pthread_create(&t, NULL, take_until_timeout, ch) == 0);
  const struct timespec nap = {0, 100000000};
  nanosleep(&nap, NULL);
  CHECK(flux2_channel_publish(ch, (const uint8_t *)"a", 1) == FLUX2_PUBLISHED_IN_USE);
  flux2_view v = {0};
  CHECK(!flux2_channel_take(ch, &v));
  flux2_loan l = {0};
  CHECK(!flux2_channel_loan(ch, &l));
  CHECK(flux2_channel_set_qos(ch, (flux2_qos){1, 0, 2}) == FLUX2_IN_USE);
  CHECK(pthread_join(t, NULL) == 0);
  CHECK(flux2_channel_publish(ch, (const uint8_t *)"a", 1) == FLUX2_PUBLISHED_OK);
  flux2_channel_free(ch);
}

// A watch opened before the publisher: armed at 0, it sees the publisher's epoch, and a channel
// opened through it attaches.
static void a_watch_sees_the_publisher_come_up(void)
{
  char name[256];
  test_signpost("watch", name, sizeof name);
  flux2_watch * w = NULL;
  CHECK(flux2_watch_open(name, &w) == FLUX2_OK);
  CHECK(flux2_watch_arm(w) == 0 && flux2_watch_epoch(w) == 0);
  CHECK(!flux2_watch_wait(w, 1000000));
  const flux2_qos q = {1, 0, 2};
  const flux2_memory_policy mem = {0};
  flux2_channel * s = NULL;
  CHECK(flux2_channel_open_watched(w, 0, q, mem, NULL, FLUX2_NO_STAMP, &s) == FLUX2_NOT_READY);
  flux2_channel * p = NULL;
  CHECK(
    flux2_channel_create(
      name, 32, 2, FLUX2_DEFAULT_MAX_SUBSCRIBERS, 0, mem, NULL, FLUX2_NO_STAMP, &p) == FLUX2_OK);
  CHECK(flux2_watch_wait(w, 0) && flux2_watch_epoch(w) != 0);
  CHECK(flux2_channel_open_watched(w, 0, q, mem, NULL, FLUX2_NO_STAMP, &s) == FLUX2_OK);
  CHECK(flux2_channel_attach_status(s) == FLUX2_OK);
  flux2_channel_free(s);
  flux2_channel_free(p);
  flux2_watch_free(w);
}

int main(void)
{
  heap_views_and_loans();
  failures_say_why();
  shm_publisher_and_subscriber();
  a_second_thread_is_refused();
  take_blocking_wakes_on_the_publish();
  a_failure_during_thread_exit_returns_its_status();
  a_watch_sees_the_publisher_come_up();
  printf("ok\n");
  return 0;
}
