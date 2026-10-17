// A view release against an executor at max_borrow (channel.rs end_borrow, arm_release,
// wake_on_release): the release drops the count and then reads the flag; the executor sets the
// flag and then reads the count. Forbidden: the executor sleeps at the limit and nobody pokes it.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>

static atomic_int outstanding = 1;
static atomic_int armed;
static int flag_seen = 1;
static int count_seen;

static void * release(void * arg)
{
  (void)arg;
  atomic_fetch_sub_explicit(&outstanding, 1, memory_order_seq_cst);
  flag_seen = atomic_load_explicit(&armed, memory_order_seq_cst);
  return NULL;
}

static void * executor(void * arg)
{
  (void)arg;
  atomic_store_explicit(&armed, 1, memory_order_seq_cst);
  count_seen = atomic_load_explicit(&outstanding, memory_order_seq_cst);
  return NULL;
}

int main(void)
{
  pthread_t r, e;
  pthread_create(&r, NULL, release, NULL);
  pthread_create(&e, NULL, executor, NULL);
  pthread_join(r, NULL);
  pthread_join(e, NULL);
  assert(!(flag_seen == 0 && count_seen == 1));
  return 0;
}
