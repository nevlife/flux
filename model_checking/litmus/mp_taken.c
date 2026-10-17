// A replaced parker word (executor.rs Aim::park, Parker::reclaim): the parker's last touch of the
// old word comes before its Release store of `taken`; the executor frees the word's owner only
// after an Acquire load of `taken` reaches the post that replaced it.

#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>

static atomic_uint * word;
static atomic_uint taken;
static int freed;

static void * parker(void * arg)
{
  (void)arg;
  (void)atomic_load_explicit(word, memory_order_relaxed);
  atomic_store_explicit(&taken, 2, memory_order_release);
  return NULL;
}

static void * executor(void * arg)
{
  (void)arg;
  if (atomic_load_explicit(&taken, memory_order_acquire) >= 2) {
    free(word);
    freed = 1;
  }
  return NULL;
}

int main(void)
{
  word = malloc(sizeof *word);
  atomic_init(word, 0);
  pthread_t k, e;
  pthread_create(&k, NULL, parker, NULL);
  pthread_create(&e, NULL, executor, NULL);
  pthread_join(k, NULL);
  pthread_join(e, NULL);
  if (!freed) free(word);
  return 0;
}
