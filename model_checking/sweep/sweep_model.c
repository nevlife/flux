// Exhaustive interleaving model of flux2's /dev/shm naming under sequential consistency: sweepers
// with no turn-taking lock, publishers that create and drop a signpost and a segment, subscribers
// that attach and drop, and process death at any step. Names, objects (inodes), open file
// descriptions and their OFD locks are explicit. Encodes flux2_core discovery.rs (SignpostMap::open
// and drop, open_publisher_segment, remove_if_dead, sweep_dead) and segment.rs (Segment::drop).
// See README.md.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef N_PUBS
#define N_PUBS 1
#endif
#ifndef N_SUBS
#define N_SUBS 1
#endif
#ifndef N_SWEEPERS
#define N_SWEEPERS 2
#endif
#ifndef MAX_CRASHES
#define MAX_CRASHES 1
#endif
// Segment names .e1 .. .eN_EPOCHS. A publisher that runs past the last gives up.
#ifndef N_EPOCHS
#define N_EPOCHS 2
#endif
// What a crashed run of an earlier process left: a signpost naming epoch 1, and its segment.
#ifndef LEFTOVER
#define LEFTOVER 1
#endif

#define N_ACT (N_PUBS + N_SUBS + N_SWEEPERS)
#define NAMES (1 + N_EPOCHS)
#define SIGNPOST 0
#define MAX_INODES 14

// Lock byte A is the liveness byte: SIGNPOST_USER_BYTE of a signpost, PUBLISHER_LOCK_BYTE of a
// segment. Byte B is SIGNPOST_PUBLISHER_BYTE.
enum { UNLOCKED, READ, WRITE };
enum { FD_S, FD_G, FD_T, N_FDS };

typedef struct
{
  uint8_t ino, a, b;
} Fd;

enum {
  // SignpostMap::open
  OPEN_S,
  LOCK_S,
  CHECK_S,
  // publisher
  LOCK_PUB,
  READ_EPOCH,
  CREATE_G,
  CREATE_TAKEN,
  LOCK_G,
  CHECK_G,
  STORE_EPOCH,
  LIVE,
  UNLINK_G,
  CLOSE_G,
  // subscriber
  SUB_EPOCH,
  SUB_OPEN_G,
  SUB_CHECK_PUB,
  ATTACHED,
  SUB_CLOSE_G,
  // SignpostMap::drop
  DROP_LOCK,
  DROP_UNLOCK,
  DROP_RELOCK,
  DROP_CHECK,
  DROP_UNLINK,
  DROP_CLOSE,
  // sweeper
  SWEEP_NEXT,
  // remove_if_dead
  RM_OPEN,
  RM_LOCK,
  RM_CHECK,
  RM_UNLINK,
  RM_CLOSE,
  DONE,
  DEAD
};

typedef struct
{
  uint8_t bound[NAMES];       // the inode a name binds, 0 for none
  uint8_t epoch[MAX_INODES];  // a signpost's epoch word
  uint8_t next_ino, crashes;
  uint8_t pc[N_ACT];
  uint8_t ret[N_ACT];  // where remove_if_dead returns
  uint8_t rm_name[N_ACT], rm_ok[N_ACT];
  uint8_t e[N_ACT];                      // a publisher's epoch, a sweeper's next name
  uint8_t hold_s[N_ACT], hold_g[N_ACT];  // the actor relies on the name binding its object
  Fd fd[N_ACT][N_FDS];
} State;

enum { OK = 0, V_UNLINK_LIVE, V_REBOUND, V_LEAK };
static const char * viol_name[] = {"OK", "INV_UNLINK_LIVE", "INV_REBOUND", "INV_LEAK"};

static long cov_drop_retry, cov_drop_rebound, cov_swept, cov_refused, cov_rebound, cov_open_retry,
  cov_create_retry, cov_live, cov_attached, cov_exists, cov_dead_holder;

#include "../explore.h"

static int is_pub(int a)
{
  return a < N_PUBS;
}

// OFD locks: descriptions conflict, a description never conflicts with itself.
static int can_lock(const State * s, int actor, int slot, int byte_b, int mode)
{
  int ino = s->fd[actor][slot].ino;
  for (int x = 0; x < N_ACT; x++)
    for (int f = 0; f < N_FDS; f++) {
      if (x == actor && f == slot) continue;
      const Fd * d = &s->fd[x][f];
      if (d->ino != ino) continue;
      int held = byte_b ? d->b : d->a;
      if (held != UNLOCKED && (mode == WRITE || held == WRITE)) return 0;
    }
  return 1;
}

static void close_fd(State * s, int actor, int slot)
{
  memset(&s->fd[actor][slot], 0, sizeof(Fd));
}

// A live object is one whose liveness byte another description holds a read lock on.
static int live_elsewhere(const State * s, int ino, int actor)
{
  for (int x = 0; x < N_ACT; x++)
    for (int f = 0; f < N_FDS; f++)
      if (x != actor && s->fd[x][f].ino == ino && s->fd[x][f].a == READ) return 1;
  return 0;
}

static int unlink_name(State * s, int actor, int name)
{
  int ino = s->bound[name];
  s->bound[name] = 0;
  return ino && live_elsewhere(s, ino, actor) ? V_UNLINK_LIVE : OK;
}

static int seg_name(int e)
{
  return e;
}  // names 1..N_EPOCHS

#define LABEL(...) snprintf(label, FLUX2_MODEL_ACT_LEN, __VA_ARGS__)

static void remove_if_dead(State * o, int a, int name, int ret)
{
  o->rm_name[a] = name;
  o->ret[a] = ret;
  o->pc[a] = RM_OPEN;
}

static int step(const State * in, State * o, int a, char * label, int * viol)
{
  *o = *in;
  Fd * fs = &o->fd[a][FD_S];
  Fd * fg = &o->fd[a][FD_G];
  Fd * ft = &o->fd[a][FD_T];
  switch (in->pc[a]) {
    case OPEN_S:
      if (!o->bound[SIGNPOST]) {
        if (o->next_ino >= MAX_INODES) return 0;
        o->bound[SIGNPOST] = o->next_ino++;
      }
      fs->ino = o->bound[SIGNPOST];
      o->pc[a] = LOCK_S;
      LABEL("a%d:open(O_CREAT) S -> i%d", a, fs->ino);
      return 1;
    case LOCK_S:
      if (!can_lock(o, a, FD_S, 0, READ)) {
        close_fd(o, a, FD_S);
        o->pc[a] = OPEN_S;
        cov_open_retry++;
        LABEL("a%d:read lock S refused, retry", a);
        return 1;
      }
      fs->a = READ;
      o->pc[a] = CHECK_S;
      LABEL("a%d:read lock S i%d", a, fs->ino);
      return 1;
    case CHECK_S:
#ifndef BUG_OPEN_NO_CHECK
      if (o->bound[SIGNPOST] != fs->ino) {
        close_fd(o, a, FD_S);
        o->pc[a] = OPEN_S;
        cov_open_retry++;
        LABEL("a%d:S rebound, retry", a);
        return 1;
      }
#endif
      o->hold_s[a] = 1;
      o->pc[a] = is_pub(a) ? LOCK_PUB : SUB_EPOCH;
      LABEL("a%d:S bound to i%d", a, fs->ino);
      return 1;

    case LOCK_PUB:
      if (!can_lock(o, a, FD_S, 1, WRITE)) {
        cov_exists++;
        o->pc[a] = DROP_LOCK;
        LABEL("a%d:publisher exists", a);
        return 1;
      }
      fs->b = WRITE;
      o->pc[a] = READ_EPOCH;
      LABEL("a%d:publisher lock", a);
      return 1;
    case READ_EPOCH: {
      int last = o->epoch[fs->ino];
      o->e[a] = last + 1;
      if (last != 0)
        remove_if_dead(o, a, seg_name(last), CREATE_G);
      else
        o->pc[a] = CREATE_G;
      LABEL("a%d:epoch %d", a, last);
      return 1;
    }
    case CREATE_TAKEN:
      // Removed: the name is free again. Not: the real code draws another epoch.
      if (!o->rm_ok[a]) o->e[a]++;
      o->pc[a] = CREATE_G;
      LABEL("a%d:.e%d %s", a, in->e[a], o->rm_ok[a] ? "was dead, retry" : "taken, next");
      return 1;
    case CREATE_G: {
      int e = o->e[a];
      if (e > N_EPOCHS) {
        o->pc[a] = DROP_LOCK;
        LABEL("a%d:no free epoch, give up", a);
        return 1;
      }
      if (o->bound[seg_name(e)]) {
        remove_if_dead(o, a, seg_name(e), CREATE_TAKEN);
        LABEL("a%d:O_EXCL .e%d exists", a, e);
        return 1;
      }
      if (o->next_ino >= MAX_INODES) return 0;
      o->bound[seg_name(e)] = o->next_ino++;
      fg->ino = o->bound[seg_name(e)];
      o->pc[a] = LOCK_G;
      LABEL("a%d:O_EXCL .e%d -> i%d", a, e, fg->ino);
      return 1;
    }
    case LOCK_G:
      if (!can_lock(o, a, FD_G, 0, READ)) {
        close_fd(o, a, FD_G);
        o->pc[a] = CREATE_G;
        cov_create_retry++;
        LABEL("a%d:read lock .e%d refused, retry", a, o->e[a]);
        return 1;
      }
      fg->a = READ;
      o->pc[a] = CHECK_G;
      LABEL("a%d:read lock .e%d", a, o->e[a]);
      return 1;
    case CHECK_G:
#ifndef BUG_CREATE_NO_CHECK
      if (o->bound[seg_name(o->e[a])] != fg->ino) {
        close_fd(o, a, FD_G);
        o->pc[a] = CREATE_G;
        cov_create_retry++;
        LABEL("a%d:.e%d unlinked under it, retry", a, o->e[a]);
        return 1;
      }
#endif
      o->hold_g[a] = 1;
      o->pc[a] = STORE_EPOCH;
      LABEL("a%d:.e%d bound to i%d", a, o->e[a], fg->ino);
      return 1;
    case STORE_EPOCH:
      o->epoch[fs->ino] = o->e[a];
      o->pc[a] = LIVE;
      cov_live++;
      LABEL("a%d:store epoch %d", a, o->e[a]);
      return 1;
    case LIVE:
      o->pc[a] = UNLINK_G;
      LABEL("a%d:drop", a);
      return 1;
    case UNLINK_G:
      o->hold_g[a] = 0;
      *viol = unlink_name(o, a, seg_name(o->e[a]));
      o->pc[a] = CLOSE_G;
      LABEL("a%d:unlink .e%d", a, o->e[a]);
      return 1;
    case CLOSE_G:
      close_fd(o, a, FD_G);
      o->pc[a] = DROP_LOCK;
      LABEL("a%d:close segment", a);
      return 1;

    case SUB_EPOCH: {
      int e = o->epoch[fs->ino];
      if (e == 0) {
        o->pc[a] = DROP_LOCK;
        LABEL("a%d:no publisher", a);
        return 1;
      }
      o->e[a] = e;
      o->pc[a] = SUB_OPEN_G;
      LABEL("a%d:epoch %d", a, e);
      return 1;
    }
    case SUB_OPEN_G:
      if (!o->bound[seg_name(o->e[a])]) {
        o->pc[a] = DROP_LOCK;
        LABEL("a%d:.e%d gone", a, o->e[a]);
        return 1;
      }
      fg->ino = o->bound[seg_name(o->e[a])];
      o->pc[a] = SUB_CHECK_PUB;
      LABEL("a%d:open .e%d -> i%d", a, o->e[a], fg->ino);
      return 1;
    case SUB_CHECK_PUB:
      if (!live_elsewhere(o, fg->ino, a)) {
        close_fd(o, a, FD_G);
        o->pc[a] = DROP_LOCK;
        LABEL("a%d:no live publisher", a);
        return 1;
      }
      o->pc[a] = ATTACHED;
      cov_attached++;
      LABEL("a%d:attached", a);
      return 1;
    case ATTACHED:
      o->pc[a] = SUB_CLOSE_G;
      LABEL("a%d:drop", a);
      return 1;
    case SUB_CLOSE_G:
      close_fd(o, a, FD_G);
      o->pc[a] = DROP_LOCK;
      LABEL("a%d:close segment", a);
      return 1;

    case DROP_LOCK:
      o->hold_s[a] = 0;
#ifdef BUG_DROP_NO_LOCK
      o->pc[a] = DROP_UNLINK;
      LABEL("a%d:drop S without lock", a);
      return 1;
#else
      if (!can_lock(o, a, FD_S, 0, WRITE)) {
#ifdef BUG_DROP_NO_RETRY
        o->pc[a] = DROP_CLOSE;
#else
        o->pc[a] = DROP_UNLOCK;
#endif
        LABEL("a%d:S has other users", a);
        return 1;
      }
      fs->a = WRITE;
      o->pc[a] = DROP_UNLINK;
      LABEL("a%d:last user of S", a);
      return 1;
#endif
    // Two last users dropping at once each see the other's read lock. Each lets go of its own and
    // tries once more; whoever lets go last finds none left.
    case DROP_UNLOCK:
      fs->a = UNLOCKED;
      o->pc[a] = DROP_RELOCK;
      LABEL("a%d:release S", a);
      return 1;
    case DROP_RELOCK:
      if (!can_lock(o, a, FD_S, 0, WRITE)) {
        o->pc[a] = DROP_CLOSE;
        LABEL("a%d:S still has users", a);
        return 1;
      }
      fs->a = WRITE;
      o->pc[a] = DROP_CHECK;
      LABEL("a%d:write lock S", a);
      return 1;
    // Between the release and the lock, a sweeper may have removed S and someone made it anew.
    case DROP_CHECK:
#ifndef BUG_DROP_NO_RECHECK
      if (o->bound[SIGNPOST] != fs->ino) {
        cov_drop_rebound++;
        o->pc[a] = DROP_CLOSE;
        LABEL("a%d:S rebound", a);
        return 1;
      }
#endif
      cov_drop_retry++;
      o->pc[a] = DROP_UNLINK;
      LABEL("a%d:S still bound", a);
      return 1;
    case DROP_UNLINK:
      *viol = unlink_name(o, a, SIGNPOST);
      o->pc[a] = DROP_CLOSE;
      LABEL("a%d:unlink S", a);
      return 1;
    case DROP_CLOSE:
      close_fd(o, a, FD_S);
      o->pc[a] = DONE;
      LABEL("a%d:close S, done", a);
      return 1;

    case SWEEP_NEXT:
      if (o->e[a] == NAMES) {
        o->pc[a] = DONE;
        LABEL("a%d:sweep done", a);
        return 1;
      }
      remove_if_dead(o, a, o->e[a]++, SWEEP_NEXT);
      LABEL("a%d:sweep name %d", a, o->rm_name[a]);
      return 1;

    case RM_OPEN:
      o->rm_ok[a] = 0;
      if (!o->bound[in->rm_name[a]]) {
        o->pc[a] = in->ret[a];
        LABEL("a%d:rm %d: not there", a, in->rm_name[a]);
        return 1;
      }
      ft->ino = o->bound[in->rm_name[a]];
#ifdef BUG_LOCK_AFTER_CHECK
      o->pc[a] = RM_CHECK;
#else
      o->pc[a] = RM_LOCK;
#endif
      LABEL("a%d:rm %d: open -> i%d", a, in->rm_name[a], ft->ino);
      return 1;
    case RM_LOCK:
#ifndef BUG_RM_NO_LOCK
      if (!can_lock(o, a, FD_T, 0, WRITE)) {
        cov_refused++;
        o->pc[a] = RM_CLOSE;
        LABEL("a%d:rm %d: lock refused", a, in->rm_name[a]);
        return 1;
      }
      ft->a = WRITE;
#endif
#ifdef BUG_LOCK_AFTER_CHECK
      o->pc[a] = RM_UNLINK;
#else
      o->pc[a] = RM_CHECK;
#endif
      LABEL("a%d:rm %d: write lock i%d", a, in->rm_name[a], ft->ino);
      return 1;
    case RM_CHECK:
#ifndef BUG_RM_NO_CHECK
      if (o->bound[in->rm_name[a]] != ft->ino) {
        cov_rebound++;
        o->pc[a] = RM_CLOSE;
        LABEL("a%d:rm %d: rebound", a, in->rm_name[a]);
        return 1;
      }
#endif
#ifdef BUG_LOCK_AFTER_CHECK
      o->pc[a] = RM_LOCK;
#else
      o->pc[a] = RM_UNLINK;
#endif
      LABEL("a%d:rm %d: still bound", a, in->rm_name[a]);
      return 1;
    case RM_UNLINK:
      if (in->bound[in->rm_name[a]] == ft->ino) cov_swept++;
      *viol = unlink_name(o, a, in->rm_name[a]);
      o->rm_ok[a] = 1;
      o->pc[a] = RM_CLOSE;
      LABEL("a%d:rm %d: unlink", a, in->rm_name[a]);
      return 1;
    case RM_CLOSE:
      close_fd(o, a, FD_T);
      o->pc[a] = in->ret[a];
      LABEL("a%d:rm %d: close", a, in->rm_name[a]);
      return 1;
    default:
      return 0;
  }
}

// Death releases every lock the process held; its objects stay as they are.
static int crash_step(const State * in, State * o, int a, char * label)
{
#if MAX_CRASHES == 0
  (void)in;
  (void)o;
  (void)a;
  (void)label;
  return 0;
#else
  if (in->crashes >= MAX_CRASHES || in->pc[a] == DONE || in->pc[a] == DEAD) return 0;
  *o = *in;
  for (int f = 0; f < N_FDS; f++)
    if (o->fd[a][f].a == READ) cov_dead_holder++;
  memset(o->fd[a], 0, sizeof(o->fd[a]));
  o->hold_s[a] = o->hold_g[a] = 0;
  o->pc[a] = DEAD;
  o->crashes++;
  LABEL("a%d:CRASH", a);
  return 1;
#endif
}

// Every name an actor relies on still binds its object.
static int check_bound(const State * s)
{
  for (int a = 0; a < N_ACT; a++) {
    if (s->hold_s[a] && s->bound[SIGNPOST] != s->fd[a][FD_S].ino) return V_REBOUND;
    if (s->hold_g[a] && s->bound[seg_name(s->e[a])] != s->fd[a][FD_G].ino) return V_REBOUND;
  }
  return OK;
}

// With no death, every process left as it came in: nothing it made is bound any more.
static int check_terminal(const State * s)
{
  if (s->crashes) return OK;
  for (int n = 0; n < NAMES; n++)
    if (s->bound[n]) return V_LEAK;
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
  init.next_ino = 1;
#if LEFTOVER
  init.bound[SIGNPOST] = init.next_ino++;
  init.epoch[init.bound[SIGNPOST]] = 1;
  init.bound[seg_name(1)] = init.next_ino++;
#endif
  for (int a = 0; a < N_ACT; a++) init.pc[a] = a < N_PUBS + N_SUBS ? OPEN_S : SWEEP_NEXT;
  int is_new;
  intern(&init, -1, "init", &is_new);

  long transitions = 0;
  for (int i = 0; i < nstates; i++) {
    State next;
    char label[FLUX2_MODEL_ACT_LEN];
    int enabled = 0;
    for (int a = 0; a < N_ACT; a++) {
      for (int kind = 0; kind < 2; kind++) {
        int viol = OK;
        int ran = kind == 0 ? step(&pool[i], &next, a, label, &viol)
                            : crash_step(&pool[i], &next, a, label);
        if (!ran) continue;
        enabled++;
        transitions++;
        if (viol == OK) viol = check_bound(&next);
        if (viol != OK) return report(i, label, viol);
        intern(&next, i, label, &is_new);
      }
    }
    if (enabled == 0 && check_terminal(&pool[i]) != OK) return report(i, "terminal", V_LEAK);
  }

  printf(
    "PASS  N_PUBS=%d N_SUBS=%d N_SWEEPERS=%d MAX_CRASHES=%d N_EPOCHS=%d LEFTOVER=%d\n", N_PUBS,
    N_SUBS, N_SWEEPERS, MAX_CRASHES, N_EPOCHS, LEFTOVER);
  printf("      states=%d transitions=%ld\n", nstates, transitions);
  printf(
    "      swept=%ld refused=%ld rebound=%ld open_retry=%ld create_retry=%ld live=%ld "
    "attached=%ld exists=%ld dead_holder=%ld drop_retry=%ld drop_rebound=%ld\n",
    cov_swept, cov_refused, cov_rebound, cov_open_retry, cov_create_retry, cov_live, cov_attached,
    cov_exists, cov_dead_holder, cov_drop_retry, cov_drop_rebound);

  const char * missing = NULL;
  if (!cov_swept)
    missing = "nothing was ever removed";
  else if (!cov_refused)
    missing = "a remover never met a lock it could not take";
  else if (!cov_rebound)
    missing = "a remover never found its name bound to another object";
  else if (!cov_open_retry)
    missing = "a signpost open never had to retry";
  else if (!cov_create_retry)
    missing = "a fresh segment was never unlinked before its publisher locked it";
  else if (!cov_live)
    missing = "no publisher went live";
  else if (N_SUBS && !cov_attached)
    missing = "no subscriber attached";
  else if (N_PUBS > 1 && !cov_exists)
    missing = "a second publisher never met the first";
  else if (N_PUBS + N_SUBS > 1 && !cov_drop_retry)
    missing = "two users never dropped the signpost together";
  else if (MAX_CRASHES && !cov_dead_holder)
    missing = "no process died holding a liveness lock";
  if (missing) {
    printf("VACUOUS  %s -- widen the config\n", missing);
    return 3;
  }
  return 0;
}
