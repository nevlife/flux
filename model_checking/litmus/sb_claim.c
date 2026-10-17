// Claim and borrow (channel.rs try_claim, take, end_borrow; segment.rs raise_readers): an SB pair
// of SeqCst accesses, the commit's MP through the ticket, and the release side's LB. The publisher
// commits frame 5, then claims the slot again and either writes frame 6 or reverts; the claim reads
// the counters up to `readers_high`. The subscriber raises `readers_high`, borrows under number 1,
// checks the ticket and reads the payload, which is plain: a missing order is a data race.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>

#define CLAIMED 0xffff

static atomic_ulong ticket;
static atomic_uint readers_high;
static atomic_uint readers1;
static int payload;

static void * publisher(void * arg)
{
  (void)arg;
  payload = 5;
  atomic_store_explicit(&ticket, 5, memory_order_release);
  unsigned long old = 5;
  if (!atomic_compare_exchange_strong_explicit(
        &ticket, &old, CLAIMED, memory_order_seq_cst, memory_order_relaxed))
    return NULL;
  if (
    atomic_load_explicit(&readers_high, memory_order_seq_cst) >= 2 &&
    atomic_load_explicit(&readers1, memory_order_seq_cst) != 0) {
    atomic_store_explicit(&ticket, 5, memory_order_release);
    return NULL;
  }
  payload = 6;
  atomic_store_explicit(&ticket, 6, memory_order_release);
  return NULL;
}

static void * subscriber(void * arg)
{
  (void)arg;
  atomic_fetch_or_explicit(&readers_high, 2, memory_order_seq_cst);
  atomic_fetch_add_explicit(&readers1, 1, memory_order_seq_cst);
  unsigned long t = atomic_load_explicit(&ticket, memory_order_seq_cst);
  if (t == 5 || t == 6) assert(payload == (int)t);
  atomic_fetch_sub_explicit(&readers1, 1, memory_order_release);
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
