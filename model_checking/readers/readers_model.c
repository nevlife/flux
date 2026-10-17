// Exhaustive interleaving model of flux2's reader counters under sequential consistency: one
// publisher, subscriber processes whose threads share one subscriber number, crash, rejoin onto a
// dead process's number, the publisher's lock-gated reclaim, and a loan dropped after writing
// (WriteSlot abort), and the high-water mark of numbers taken that bounds the publisher's reads.
// Encodes flux2_core channel.rs (claim, take, reclaim_dead, abort) and reader.rs (lease_in). See
// README.md.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef N_SLOTS
#define N_SLOTS 2
#endif
#ifndef N_NUMS
#define N_NUMS 2
#endif
#ifndef N_PROCS
#define N_PROCS 2
#endif
#ifndef N_THREADS
#define N_THREADS 3
#endif
#ifndef PUB_FRAMES
#define PUB_FRAMES 3
#endif
#ifndef PUB_ABORTS
#define PUB_ABORTS 2
#endif
#ifndef SUB_TAKES
#define SUB_TAKES 1
#endif
#ifndef SUB_RETRIES
#define SUB_RETRIES 1
#endif
// QoS depth: take never returns a frame more than DEPTH behind the newest.
#ifndef DEPTH
#define DEPTH 1
#endif
#ifndef MAX_CRASHES
#define MAX_CRASHES 0
#endif
// Loans dropped after their bytes were written.
#ifndef LOAN_ABORTS
#define LOAN_ABORTS 0
#endif

#define PROC_OF(t) ((t) % N_PROCS)
#define FREE 0
#define BY_PUB 1
#define BY_PROC(p) (2 + (p))
#define EMPTY 0
#define CLAIMED 255

enum {
  P_SELECT,
  P_RECL,
  P_RZERO,
  P_RUNLOCK,
  P_CLAIM,
  P_HIGH,
  P_RECHECK,
  P_WRITE,
  P_COMMIT,
  P_LATEST,
  P_DONE
};
enum { T_SCAN, T_INC, T_CHECK, T_READ, T_RELEASE, T_DEC, T_DONE };
enum { PR_UNJOINED, PR_ZERO, PR_RAISE, PR_ACTIVE, PR_DEAD, PR_GAVEUP };

typedef struct
{
  uint8_t ticket[N_SLOTS];  // EMPTY, CLAIMED, or the frame's ticket
  uint8_t data[N_SLOTS];    // the ticket of the frame whose bytes the payload holds
  uint8_t latest_t;
  uint8_t readers[N_NUMS][N_SLOTS];
  uint8_t lockown[N_NUMS];
  uint8_t used[N_NUMS];  // a process that died held this number
  uint8_t high;          // one past the highest number ever taken
  int8_t crashes;
  uint8_t ppc, pslot, pold, pk, pz_pub, pframe, paborts, pnext, ppass, preclaimed, ploans, phigh;
  uint8_t prc[N_PROCS], pnum[N_PROCS], pz[N_PROCS], praised[N_PROCS];
  uint8_t tpc[N_THREADS], tslot[N_THREADS], tbest[N_THREADS];
  uint8_t tcursor[N_THREADS], ttakes[N_THREADS], tretries[N_THREADS];
} State;

enum { OK = 0, V_BYTELOCK, V_COHERENT, V_UNDERFLOW, V_FLOOR, V_LEAK };
static const char * viol_name[] = {"OK",        "INV_BYTELOCK", "INV_COHERENT", "INV_UNDERFLOW",
                                   "INV_FLOOR", "INV_LEAK"};

// Each must be non-zero, or the PASS is VACUOUS (exit 3).
static long cov_abort, cov_retry, cov_shared_hold, cov_cross_hold, cov_crash_holding, cov_reclaimed,
  cov_refused, cov_rejoin, cov_residue, cov_loan_abort, cov_below_high;

static void note_coverage(const State * s);
#define FLUX2_MODEL_ON_NEW_STATE(s) note_coverage(s)
#include "../explore.h"

static int pub_done(const State * s)
{
  return s->ppc == P_DONE;
}

// Validated and not yet released: the publisher must not write this slot.
static int holds(const State * s, int t)
{
  if (s->prc[PROC_OF(t)] != PR_ACTIVE) return 0;
  int pc = s->tpc[t];
#ifdef BUG_INC_LAST
  return pc == T_INC || pc == T_READ || pc == T_RELEASE;
#else
  return pc == T_READ || pc == T_RELEASE;
#endif
}

// The counter carries this thread's +1.
static int contributes(const State * s, int t)
{
  int pc = s->tpc[t];
#ifdef BUG_INC_LAST
  return pc == T_READ || pc == T_RELEASE;
#else
  return pc == T_CHECK || pc == T_READ || pc == T_RELEASE || pc == T_DEC;
#endif
}

static void note_coverage(const State * s)
{
  for (int a = 0; a < N_THREADS; a++)
    for (int b = a + 1; b < N_THREADS; b++) {
      if (!holds(s, a) || !holds(s, b) || s->tslot[a] != s->tslot[b]) continue;
      if (PROC_OF(a) == PROC_OF(b))
        cov_shared_hold++;
      else
        cov_cross_hold++;
    }
}

// INV_FLOOR: a live number's counter covers every +1 its process's threads still carry.
static int check_floor(const State * s)
{
  for (int p = 0; p < N_PROCS; p++) {
    if (s->prc[p] != PR_ACTIVE) continue;
    for (int sl = 0; sl < N_SLOTS; sl++) {
      int need = 0;
      for (int t = 0; t < N_THREADS; t++)
        if (PROC_OF(t) == p && contributes(s, t) && s->tslot[t] == sl) need++;
      if (s->readers[s->pnum[p]][sl] < need) return V_FLOOR;
    }
  }
  return OK;
}

static int all_zero(const State * s, int sl)
{
  for (int n = 0; n < s->high; n++)
    if (s->readers[n][sl] != 0) return 0;
  return 1;
}

static int any_held(const State * s, int n)
{
  for (int sl = 0; sl < N_SLOTS; sl++)
    if (s->readers[n][sl] != 0) return 1;
  return 0;
}

#define LABEL(...) snprintf(label, FLUX2_MODEL_ACT_LEN, __VA_ARGS__)

static int pub_step(const State * in, State * out, char * label, int * viol)
{
  *out = *in;
  int s = in->pslot;
  switch (in->ppc) {
    case P_SELECT: {
      if (in->pframe >= PUB_FRAMES) {
        out->ppc = P_DONE;
        LABEL("pub:done");
        return 1;
      }
      int pick = -1;
      for (int k = 0; k < N_SLOTS; k++) {
        if (in->ticket[k] == CLAIMED || !all_zero(in, k)) continue;
        if (pick < 0 || in->ticket[k] < in->ticket[pick]) pick = k;
      }
      if (pick >= 0) {
        out->pslot = pick;
        out->ppc = P_CLAIM;
        LABEL("pub:select s%d (oldest free)", pick);
        return 1;
      }
      if (in->ppass == 0) {
        out->ppc = P_RECL;
        out->pk = 0;
        out->preclaimed = 0;
        LABEL("pub:starved, reclaim");
        return 1;
      }
      out->pframe++;
      out->ppass = 0;
      LABEL("pub:drop f%d", in->pframe + 1);
      return 1;
    }
    case P_RECL: {
      int n = in->pk;
      if (n >= in->high) {
        out->ppc = P_SELECT;
        out->ppass = 1;
        LABEL("pub:reclaim done (%d)", in->preclaimed);
        return 1;
      }
      if (!any_held(in, n)) {
        out->pk++;
        LABEL("pub:n%d not held", n);
        return 1;
      }
#ifdef BUG_RECLAIM_NO_LOCK
      out->ppc = P_RZERO;
      out->pz_pub = 0;
      LABEL("pub:n%d zero WITHOUT lock", n);
      return 1;
#else
      if (in->lockown[n] != FREE) {
        cov_refused++;
        out->pk++;
        LABEL("pub:n%d lock refused (live)", n);
        return 1;
      }
      out->lockown[n] = BY_PUB;
#ifdef BUG_UNLOCK_EARLY
      out->lockown[n] = FREE;
#endif
      out->ppc = P_RZERO;
      out->pz_pub = 0;
      LABEL("pub:n%d locked (dead)", n);
      return 1;
#endif
    }
    case P_RZERO: {
      int n = in->pk, sl = in->pz_pub;
      out->readers[n][sl] = 0;
      out->pz_pub = sl + 1;
      if (sl + 1 == N_SLOTS) out->ppc = P_RUNLOCK;
      if (in->readers[n][sl] != 0) cov_reclaimed++;
      LABEL("pub:n%d s%d zero", n, sl);
      return 1;
    }
    case P_RUNLOCK:
#ifndef BUG_RECLAIM_NO_LOCK
#ifndef BUG_UNLOCK_EARLY
      out->lockown[in->pk] = FREE;
#endif
#endif
      out->preclaimed = 1;
      out->pk++;
      out->ppc = P_RECL;
      LABEL("pub:n%d unlock", in->pk);
      return 1;
    case P_CLAIM:
      out->pold = in->ticket[s];
      out->ticket[s] = CLAIMED;
      out->pk = 0;
      out->ppc = P_HIGH;
      LABEL("pub:claim s%d (was t%d)", s, in->ticket[s]);
      return 1;
    case P_HIGH:
      out->phigh = in->high;
      if (in->high < N_NUMS) cov_below_high++;
      out->ppc = in->high == 0 ? P_WRITE : P_RECHECK;
      LABEL("pub:s%d reads %d numbers", s, in->high);
      return 1;
    case P_RECHECK: {
      int n = in->pk;
#ifndef BUG_NO_DEKKER
      if (in->readers[n][s] != 0) {
        out->ticket[s] = in->pold;  // no byte written yet, so the old frame stands
        out->paborts++;
        out->ppass = 0;
        out->ppc = out->paborts >= PUB_ABORTS ? P_DONE : P_SELECT;
        cov_abort++;
        LABEL("pub:s%d n%d borrowed, abort", s, n);
        return 1;
      }
#endif
      out->pk = n + 1;
      if (n + 1 >= in->phigh) out->ppc = P_WRITE;
      LABEL("pub:s%d n%d free", s, n);
      return 1;
    }
    case P_WRITE:
      for (int t = 0; t < N_THREADS; t++)
        if (holds(in, t) && in->tslot[t] == s) *viol = V_BYTELOCK;
      out->data[s] = in->pnext + 1;
      out->ppc = P_COMMIT;
      LABEL("pub:write s%d=f%d", s, in->pnext + 1);
      return 1;
    case P_COMMIT:
      out->ticket[s] = in->pnext + 1;
      out->ppc = P_LATEST;
      LABEL("pub:commit s%d t%d", s, in->pnext + 1);
      return 1;
    case P_LATEST:
      out->pnext++;
      out->latest_t = out->pnext;
      out->pframe++;
      out->ppass = 0;
      out->ppc = P_SELECT;
      LABEL("pub:latest=%d@s%d", out->pnext, s);
      return 1;
    default:
      return 0;
  }
}

// The loan is dropped instead of committed. Its bytes replaced the slot's frame, so the slot goes
// empty; restoring the old ticket would label the loan's bytes as that frame.
static int loan_abort_step(const State * in, State * out, char * label)
{
#if LOAN_ABORTS == 0
  (void)in;
  (void)out;
  (void)label;
  return 0;
#else
  if (in->ppc != P_COMMIT || in->ploans >= LOAN_ABORTS) return 0;
  *out = *in;
  int s = in->pslot;
#ifdef BUG_ABORT_RESTORE
  out->ticket[s] = in->pold;
#else
  out->ticket[s] = EMPTY;
#endif
  if (in->pold != EMPTY) cov_loan_abort++;
  out->ploans++;
  out->ppass = 0;
  out->ppc = P_SELECT;
  LABEL("pub:loan s%d dropped (was t%d)", s, in->pold);
  return 1;
#endif
}

static int proc_step(const State * in, State * out, int p, char * label)
{
  *out = *in;
  switch (in->prc[p]) {
    case PR_UNJOINED:
      for (int n = 0; n < N_NUMS; n++) {
        if (in->lockown[n] != FREE) continue;
        out->lockown[n] = BY_PROC(p);
        out->pnum[p] = n;
        out->pz[p] = 0;
        out->prc[p] = PR_ZERO;
        if (in->used[n]) cov_rejoin++;
        LABEL("proc%d:join n%d", p, n);
        return 1;
      }
      if (in->crashes < MAX_CRASHES) return 0;  // a crash may still free a number
      out->prc[p] = PR_GAVEUP;
      LABEL("proc%d:no free number", p);
      return 1;
    case PR_ZERO: {
      int n = in->pnum[p], sl = in->pz[p];
      if (in->readers[n][sl] != 0) cov_residue++;
#ifndef BUG_JOIN_NO_ZERO
      out->readers[n][sl] = 0;
#endif
      out->pz[p] = sl + 1;
#ifdef BUG_RAISE_LATE
      if (sl + 1 == N_SLOTS) out->prc[p] = PR_ACTIVE;
#else
      if (sl + 1 == N_SLOTS) out->prc[p] = PR_RAISE;
#endif
      LABEL("proc%d:zero n%d s%d", p, n, sl);
      return 1;
    }
    case PR_RAISE:
    case PR_ACTIVE: {
      // Late, the raise races the threads' first borrows instead of preceding them.
      if (in->praised[p]) return 0;
      int n = in->pnum[p];
      if (in->high < n + 1) out->high = n + 1;
      out->praised[p] = 1;
      out->prc[p] = PR_ACTIVE;
      LABEL("proc%d:raise high to %d", p, out->high);
      return 1;
    }
    default:
      return 0;
  }
}

static int crash_step(const State * in, State * out, int p, char * label)
{
  if (in->prc[p] != PR_ACTIVE || in->crashes >= MAX_CRASHES) return 0;
  *out = *in;
  int n = in->pnum[p];
  if (any_held(in, n)) cov_crash_holding++;
  out->prc[p] = PR_DEAD;
  out->lockown[n] = FREE;  // the kernel drops the OFD lock at death
  out->used[n] = 1;
  out->crashes++;
  LABEL("proc%d:CRASH (n%d)", p, n);
  return 1;
}

static int dec(State * out, int n, int sl, int * viol)
{
  if (out->readers[n][sl] == 0) {
    *viol = V_UNDERFLOW;
    return 0;
  }
  out->readers[n][sl]--;
  return 1;
}

static int thread_step(const State * in, State * out, int t, char * label, int * viol)
{
  int p = PROC_OF(t);
  if (in->prc[p] != PR_ACTIVE) return 0;
  *out = *in;
  int n = in->pnum[p], sl = in->tslot[t];
  switch (in->tpc[t]) {
    case T_SCAN: {
      if (in->ttakes[t] >= SUB_TAKES || in->tretries[t] >= SUB_RETRIES) {
        out->tpc[t] = T_DONE;
        LABEL("t%d:done", t);
        return 1;
      }
      int floor = in->latest_t > DEPTH ? in->latest_t - DEPTH : 0;
      if (in->tcursor[t] < floor) out->tcursor[t] = floor;
      int best = -1;
      for (int k = 0; k < N_SLOTS; k++) {
        if (in->ticket[k] == CLAIMED || in->ticket[k] <= out->tcursor[t]) continue;
        if (best < 0 || in->ticket[k] < in->ticket[best]) best = k;
      }
      if (best < 0) {
        if (!pub_done(in)) {
          *out = *in;
          return 0;
        }
        out->tpc[t] = T_DONE;
        LABEL("t%d:nothing, done", t);
        return 1;
      }
      out->tslot[t] = best;
      out->tbest[t] = in->ticket[best];
#ifdef BUG_INC_LAST
      out->tpc[t] = T_CHECK;
#else
      out->tpc[t] = T_INC;
#endif
      LABEL("t%d:best s%d t%d", t, best, in->ticket[best]);
      return 1;
    }
    case T_INC:
      out->readers[n][sl]++;
#ifdef BUG_INC_LAST
      out->tpc[t] = T_READ;
#else
      out->tpc[t] = T_CHECK;
#endif
      LABEL("t%d:n%d s%d +1", t, n, sl);
      return 1;
    case T_CHECK: {
#ifdef BUG_NO_CHECK
      int ok = 1;
#else
      int ok = in->ticket[sl] == in->tbest[t];
#endif
      if (!ok) {
        cov_retry++;
        out->tretries[t]++;
#ifdef BUG_INC_LAST
        out->tpc[t] = T_SCAN;
#else
        out->tpc[t] = T_DEC;
#endif
        LABEL("t%d:s%d invalid, retry", t, sl);
        return 1;
      }
      out->tcursor[t] = in->tbest[t];
#ifdef BUG_INC_LAST
      out->tpc[t] = T_INC;
#else
      out->tpc[t] = T_READ;
#endif
      LABEL("t%d:s%d valid t%d", t, sl, in->tbest[t]);
      return 1;
    }
    case T_READ:
      if (in->data[sl] != in->tcursor[t]) *viol = V_COHERENT;
      out->tpc[t] = T_RELEASE;
      LABEL("t%d:read s%d=f%d", t, sl, in->data[sl]);
      return 1;
    case T_RELEASE:
      dec(out, n, sl, viol);
      out->ttakes[t]++;
      out->tpc[t] = T_SCAN;
      LABEL("t%d:release s%d", t, sl);
      return 1;
    case T_DEC:
      dec(out, n, sl, viol);
      out->tpc[t] = T_SCAN;
      LABEL("t%d:n%d s%d -1", t, n, sl);
      return 1;
    default:
      return 0;
  }
}

// At a state with no enabled step, a live number's counters equal what its threads still carry.
static int check_terminal(const State * s)
{
  for (int p = 0; p < N_PROCS; p++) {
    if (s->prc[p] != PR_ACTIVE) continue;
    for (int sl = 0; sl < N_SLOTS; sl++) {
      int need = 0;
      for (int t = 0; t < N_THREADS; t++)
        if (PROC_OF(t) == p && contributes(s, t) && s->tslot[t] == sl) need++;
      if (s->readers[s->pnum[p]][sl] != need) return V_LEAK;
    }
  }
  return OK;
}

static int report(int from, const char * label, int viol)
{
  printf("VIOLATION %s\n", viol_name[viol]);
  print_trace(from);
  printf("  %s   <== %s\n", label, viol_name[viol]);
  return 1;
}

int main(void)
{
  if (model_alloc() != 0) return 2;
  State init;
  memset(&init, 0, sizeof init);
  int is_new;
  intern(&init, -1, "init", &is_new);

  long transitions = 0;
  for (int i = 0; i < nstates; i++) {
    State next;
    char label[FLUX2_MODEL_ACT_LEN];
    int enabled = 0;
    for (int actor = -1; actor < 1 + 2 * N_PROCS + N_THREADS; actor++) {
      int viol = OK, ran;
      if (actor == -1)
        ran = loan_abort_step(&pool[i], &next, label);
      else if (actor == 0)
        ran = pub_step(&pool[i], &next, label, &viol);
      else if (actor <= N_PROCS)
        ran = proc_step(&pool[i], &next, actor - 1, label);
      else if (actor <= 2 * N_PROCS)
        ran = crash_step(&pool[i], &next, actor - 1 - N_PROCS, label);
      else
        ran = thread_step(&pool[i], &next, actor - 1 - 2 * N_PROCS, label, &viol);
      if (!ran) continue;
      enabled++;
      transitions++;
      if (viol == OK) viol = check_floor(&next);
      if (viol != OK) return report(i, label, viol);
      intern(&next, i, label, &is_new);
    }
    if (enabled == 0 && check_terminal(&pool[i]) != OK) return report(i, "terminal", V_LEAK);
  }

  printf(
    "PASS  DEPTH=%d N_SLOTS=%d N_NUMS=%d N_PROCS=%d N_THREADS=%d PUB_FRAMES=%d PUB_ABORTS=%d "
    "SUB_TAKES=%d SUB_RETRIES=%d MAX_CRASHES=%d LOAN_ABORTS=%d\n",
    DEPTH, N_SLOTS, N_NUMS, N_PROCS, N_THREADS, PUB_FRAMES, PUB_ABORTS, SUB_TAKES, SUB_RETRIES,
    MAX_CRASHES, LOAN_ABORTS);
  printf("      states=%d transitions=%ld\n", nstates, transitions);
  printf(
    "      abort=%ld retry=%ld shared_hold=%ld cross_hold=%ld crash_holding=%ld reclaimed=%ld "
    "refused=%ld rejoin=%ld residue=%ld loan_abort=%ld below_high=%ld\n",
    cov_abort, cov_retry, cov_shared_hold, cov_cross_hold, cov_crash_holding, cov_reclaimed,
    cov_refused, cov_rejoin, cov_residue, cov_loan_abort, cov_below_high);

  const char * missing = NULL;
  if (!cov_abort)
    missing = "the publisher never lost a claim to a borrow";
  else if (!cov_retry)
    missing = "a subscriber never failed validation";
  else if (N_THREADS > N_PROCS && !cov_shared_hold)
    missing = "two threads of one process never held a slot together";
  else if (N_NUMS > 1 && N_PROCS > 1 && !cov_cross_hold)
    missing = "two processes never held a slot together";
  else if (MAX_CRASHES && !cov_crash_holding)
    missing = "no process died holding a count";
  else if (MAX_CRASHES && !cov_reclaimed)
    missing = "the publisher never reclaimed a count";
  else if (MAX_CRASHES && !cov_refused)
    missing = "a reclaim never met a live number";
  else if (MAX_CRASHES && !cov_rejoin)
    missing = "no process joined onto a dead number";
  else if (MAX_CRASHES && !cov_residue)
    missing = "a joiner never zeroed a dead holder's count";
  else if (LOAN_ABORTS && !cov_loan_abort)
    missing = "a dropped loan never replaced a committed frame";
  else if (!cov_below_high)
    missing = "a claim never read fewer numbers than the segment has";
  if (missing) {
    printf("VACUOUS  %s -- widen the config\n", missing);
    return 3;
  }
  return 0;
}
