// The ticket index (channel.rs finish_commit, take, next_frame): the commit writes the index entry
// and the slot's ticket before it moves `latest`, so a take that read `latest` finds the frame
// through the index. Relaxed entry and ticket reads rely on that one Release/Acquire pair. Here the
// ticket store is relaxed: its Release serves the borrow (sb_claim.c), not this shape.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>

#define SLOT 1
#define NO_SLOT 7

static atomic_uint index_entry = NO_SLOT;
static atomic_ulong slot_ticket;
static atomic_ulong latest;

static void * publisher(void * arg)
{
  (void)arg;
  atomic_store_explicit(&index_entry, SLOT, memory_order_relaxed);
  atomic_store_explicit(&slot_ticket, 1, memory_order_relaxed);
  atomic_store_explicit(&latest, 1, memory_order_release);
  return NULL;
}

static void * subscriber(void * arg)
{
  (void)arg;
  if (atomic_load_explicit(&latest, memory_order_acquire) < 1) return NULL;
  assert(atomic_load_explicit(&index_entry, memory_order_relaxed) == SLOT);
  assert(atomic_load_explicit(&slot_ticket, memory_order_relaxed) == 1);
  return NULL;
}

int main(void)
{
  pthread_t p, s;
  pthread_create(&p, NULL, publisher, NULL);
  pthread_create(&s, NULL, subscriber, NULL);
  pthread_join(p, NULL);
  pthread_join(s, NULL);
  return 0;
}
