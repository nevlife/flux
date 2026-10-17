// The release side alone (channel.rs end_borrow, try_claim): the subscriber reads the payload and
// then gives its borrow back; the publisher sees the count at 0 and overwrites. The decrement is
// Release and the publisher's read acquires, so the read happens before the overwrite. RC11 forbids
// the LB outcome even when relaxed, so a weakening shows as a race on the plain payload.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>

static atomic_uint readers = 1;
static int payload = 1;
static int seen;

static void * subscriber(void * arg)
{
  (void)arg;
  seen = payload;
  atomic_fetch_sub_explicit(&readers, 1, memory_order_release);
  return NULL;
}

static void * publisher(void * arg)
{
  (void)arg;
  if (atomic_load_explicit(&readers, memory_order_acquire) == 0) payload = 2;
  return NULL;
}

int main(void)
{
  pthread_t s, p;
  pthread_create(&s, NULL, subscriber, NULL);
  pthread_create(&p, NULL, publisher, NULL);
  pthread_join(s, NULL);
  pthread_join(p, NULL);
  assert(seen == 1);
  return 0;
}
