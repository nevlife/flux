// The single-writer seqlock of the roster (manifest.rs write_entry, read_entry) and of the parker's
// aim (executor.rs Aim::post, Aim::read): odd store, Release fence, relaxed data, even store with
// Release; Acquire load, relaxed data, Acquire fence, relaxed recheck. Forbidden: a read that
// passes the recheck with the two fields from different writes.

#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>

static atomic_uint seq;
static atomic_uint a;
static atomic_uint b;

static void * writer(void * arg)
{
  (void)arg;
  unsigned s = atomic_load_explicit(&seq, memory_order_relaxed) | 1;
  atomic_store_explicit(&seq, s, memory_order_relaxed);
  atomic_thread_fence(memory_order_release);
  atomic_store_explicit(&a, 1, memory_order_relaxed);
  atomic_store_explicit(&b, 1, memory_order_relaxed);
  atomic_store_explicit(&seq, s + 1, memory_order_release);
  return NULL;
}

static void * reader(void * arg)
{
  (void)arg;
  unsigned s1 = atomic_load_explicit(&seq, memory_order_acquire);
  if (s1 & 1) return NULL;
  unsigned x = atomic_load_explicit(&a, memory_order_relaxed);
  unsigned y = atomic_load_explicit(&b, memory_order_relaxed);
  atomic_thread_fence(memory_order_acquire);
  if (atomic_load_explicit(&seq, memory_order_relaxed) == s1) assert(x == y);
  return NULL;
}

int main(void)
{
  pthread_t w, r;
  pthread_create(&w, NULL, writer, NULL);
  pthread_create(&r, NULL, reader, NULL);
  pthread_join(w, NULL);
  pthread_join(r, NULL);
  return 0;
}
