// Exhaustive-interleaving explorer: state pool, hash-consed visited set and parent chain for
// counterexample traces. It knows no invariant, thread or step function. Copied from
// flux_model_checking, so flux2's harnesses build on their own.
//
// Include it AFTER `typedef struct { ... } State;`. Configure by defining before the include:
//   MAX_STATES, HT_BITS, FLUX2_MODEL_ACT_LEN (bytes per transition label),
//   FLUX2_MODEL_TRACE_MAX (longest trace printed), FLUX2_MODEL_ON_NEW_STATE(s) (coverage hook).

#ifndef FLUX2_MODEL_EXPLORE_H
#define FLUX2_MODEL_EXPLORE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MAX_STATES
#define MAX_STATES 8000000
#endif
#ifndef HT_BITS
#define HT_BITS 23
#endif
#define HT_SIZE (1u << HT_BITS)
#define HT_MASK (HT_SIZE - 1)

#ifndef FLUX2_MODEL_ACT_LEN
#define FLUX2_MODEL_ACT_LEN 56
#endif
#ifndef FLUX2_MODEL_TRACE_MAX
#define FLUX2_MODEL_TRACE_MAX 8192
#endif
#ifndef FLUX2_MODEL_ON_NEW_STATE
#define FLUX2_MODEL_ON_NEW_STATE(s) ((void)(s))
#endif

static State * pool;
static int * parent;
static char (*action)[FLUX2_MODEL_ACT_LEN];
static int nstates;
static int32_t * ht;

static uint64_t hash_state(const State * s)
{
  const uint8_t * p = (const uint8_t *)s;
  uint64_t h = 1469598103934665603ULL;
  for (size_t i = 0; i < sizeof(State); i++) {
    h ^= p[i];
    h *= 1099511628211ULL;
  }
  return h;
}

// Insert s if new. Returns its id; *is_new set to 1 if freshly inserted.
static int intern(const State * s, int par, const char * act, int * is_new)
{
  uint64_t h = hash_state(s);
  uint32_t i = (uint32_t)h & HT_MASK;
  for (;;) {
    int32_t id = ht[i];
    if (id < 0) break;
    if (memcmp(&pool[id], s, sizeof(State)) == 0) {
      *is_new = 0;
      return id;
    }
    i = (i + 1) & HT_MASK;
  }
  if (nstates >= MAX_STATES) {
    fprintf(stderr, "state pool exhausted (MAX_STATES=%d); shrink config\n", MAX_STATES);
    exit(2);
  }
  int id = nstates++;
  FLUX2_MODEL_ON_NEW_STATE(s);
  pool[id] = *s;
  parent[id] = par;
  snprintf(action[id], sizeof(action[id]), "%s", act);
  ht[i] = id;
  *is_new = 1;
  return id;
}

static void print_trace(int id)
{
  int path[FLUX2_MODEL_TRACE_MAX];
  int n = 0;
  while (id >= 0 && n < FLUX2_MODEL_TRACE_MAX) {
    path[n++] = id;
    id = parent[id];
  }
  for (int k = n - 1; k >= 0; k--) printf("  %s\n", action[path[k]]);
}

// Returns 0, or 2 for main to return directly.
static int model_alloc(void)
{
  pool = malloc((size_t)MAX_STATES * sizeof(State));
  parent = malloc((size_t)MAX_STATES * sizeof(int));
  action = malloc((size_t)MAX_STATES * sizeof(*action));
  ht = malloc((size_t)HT_SIZE * sizeof(int32_t));
  if (!pool || !parent || !action || !ht) {
    fprintf(stderr, "alloc failed\n");
    return 2;
  }
  memset(ht, 0xff, (size_t)HT_SIZE * sizeof(int32_t));
  return 0;
}

#endif  // FLUX2_MODEL_EXPLORE_H
