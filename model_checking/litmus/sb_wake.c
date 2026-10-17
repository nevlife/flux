// The wake gate (channel.rs finish_commit, wait; segment.rs raise_readers): the publisher bumps the
// word and then reads the waiter counts up to `readers_high`; a waiter raises `readers_high`,
// counts itself under its number, and the futex call compares the word. Forbidden: the waiter
// sleeps on the old word and the publisher skips the wake. The futex's comparison runs after the
// kernel's full barrier (futex_wait, smp_mb (A) in kernel/futex/waitwake.c), the SeqCst fence here,
// which already orders the waiter's side: its own SeqCst accesses are redundant.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>

static atomic_uint wakeup;
static atomic_uint readers_high;
static atomic_int waiters1;
static unsigned word_seen = 1;
static int count_seen;

static void * waiter(void * arg)
{
  (void)arg;
  atomic_fetch_or_explicit(&readers_high, 2, memory_order_seq_cst);  // redundant: the futex fence
  atomic_fetch_add_explicit(&waiters1, 1, memory_order_seq_cst);     // redundant: the futex fence
  atomic_thread_fence(memory_order_seq_cst);
  word_seen = atomic_load_explicit(&wakeup, memory_order_relaxed);
  return NULL;
}

static void * publisher(void * arg)
{
  (void)arg;
  atomic_fetch_add_explicit(&wakeup, 1, memory_order_seq_cst);
  if (atomic_load_explicit(&readers_high, memory_order_seq_cst) >= 2)
    count_seen = atomic_load_explicit(&waiters1, memory_order_seq_cst);
  return NULL;
}

int main(void)
{
  pthread_t w, p;
  pthread_create(&w, NULL, waiter, NULL);
  pthread_create(&p, NULL, publisher, NULL);
  pthread_join(w, NULL);
  pthread_join(p, NULL);
  assert(!(word_seen == 0 && count_seen == 0));
  return 0;
}
