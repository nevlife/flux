// Segment publication (segment.rs init_header, discovery.rs open_publisher_segment and
// map_validate, manifest.rs read_channel_stats): the publisher writes the header, stores
// INIT_READY with Release, then the signpost epoch with Release. A subscriber acquires the epoch
// and then reads INIT_READY: seeing the new epoch with the segment not ready would refuse a good
// segment until the next epoch. An enumerator opens the segment by name and has only INIT_READY.
// One object stands for both mappings of the segment.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>

static int header;
static atomic_uint init_state;
static atomic_uint epoch;

static void * publisher(void * arg)
{
  (void)arg;
  header = 42;
  atomic_store_explicit(&init_state, 1, memory_order_release);
  atomic_store_explicit(&epoch, 1, memory_order_release);
  return NULL;
}

static void * subscriber(void * arg)
{
  (void)arg;
  if (atomic_load_explicit(&epoch, memory_order_acquire) != 1) return NULL;
  assert(atomic_load_explicit(&init_state, memory_order_acquire) == 1);  // redundant: the epoch
  assert(header == 42);
  return NULL;
}

static void * enumerator(void * arg)
{
  (void)arg;
  if (atomic_load_explicit(&init_state, memory_order_acquire) == 1) assert(header == 42);
  return NULL;
}

int main(void)
{
  pthread_t p, s, e;
  pthread_create(&p, NULL, publisher, NULL);
  pthread_create(&s, NULL, subscriber, NULL);
  pthread_create(&e, NULL, enumerator, NULL);
  pthread_join(p, NULL);
  pthread_join(s, NULL);
  pthread_join(e, NULL);
  return 0;
}
