// Exhaustive interleaving model of flux2's wake protocol under sequential consistency: the
// publisher's gate (channel.rs finish_commit: bump wakeup, then read waiters), the bare wait
// (ChannelShared::wait: announce, then FUTEX_WAIT), and the executor (executor.rs dispatch and
// wait_for_work: sample before take, futex_waitv with that sample, hold the gate while bound,
// re-gate on re-attach). Latest-wins, so the property is waking, not delivering everything:
// a subscriber in the wait protocol never sleeps while a newer frame is committed. With CRASH, a
// second waiter on its own subscriber number may die at any step, and the publisher's sweep
// (reclaim_dead -> clear_reader) takes its count back. STYLE_PARKED is the executor without
// futex_waitv: a parker thread sleeps on the word and rings the control word (executor.rs Aim,
// Parker), re-aimed by a seqlock post, with replaced words kept mapped until the parker has read
// the post. See README.md.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef FRAMES
#define FRAMES 3
#endif
#ifndef RESTART_AFTER
#define RESTART_AFTER 0  // 0: one segment; N: the publisher restarts after N frames
#endif
#ifndef TICK_BUDGET
#define TICK_BUDGET 0  // 0: tickless, the wake protocol alone must suffice
#endif

#if defined(STYLE_BARE) && RESTART_AFTER > 0
#error "the bare style models one segment; restart configs use the executor style"
#endif
#if defined(CRASH) && RESTART_AFTER > 0
#error "the crashing waiter lives on the first segment; crash configs do not restart"
#endif
#if defined(STYLE_BARE) && defined(STYLE_PARKED)
#error "the parker is the executor's fallback; the bare style has none"
#endif

#if RESTART_AFTER > 0
#define FINAL_SEG 1
#define FINAL_TICKET (FRAMES - RESTART_AFTER)
#else
#define FINAL_SEG 0
#define FINAL_TICKET FRAMES
#endif

enum { P_COMMIT, P_BUMP, P_GATE, P_WAKE, P_DONE };
#ifdef STYLE_BARE
enum { W_SAMPLE, W_TAKE, W_ANNOUNCE, W_PARK, W_PARKED, W_REMOVE, S_DONE };
#else
// Parked, E_PARK aims the parker and E_WAIT sleeps on the control word; the rest are its own.
enum {
  E_ATTACH,
  E_SAMPLE,
  E_TAKE,
  E_PARK,
  E_WAIT,
  S_DONE,
  E_AIMWAKE,
  E_SLEEP,
  E_DRAIN,
  E_STOP,
  E_STOPWAKE,
  E_JOIN
};
#endif
enum { K_TOP, K_GEN, K_WAIT, K_SLEPT, K_WOKEN, K_RING, K_DONE };

// The crashing waiter: a bare wait() under subscriber number 1.
enum { D_SAMPLE, D_ANNOUNCE, D_ANNOUNCE2, D_PARK, D_PARKED, D_REMOVE, D_REMOVE2, D_DONE };

typedef struct
{
  uint8_t wakeup[2];
  int8_t waiters[2];  // the main subscriber's count, under its number; it never dies
  int8_t dnum;        // the crashing waiter's count under number 1
  int8_t dsum;        // its share of the old protocol's header sum (BUG_TWO_STORES)
  uint8_t dpc, dexp, dparked, dead;
  uint8_t latest[2];
  uint8_t parked[2];  // kernel: a waiter sleeps on this segment's word
  uint8_t cur;        // the segment the signpost names
  uint8_t ppc, pframe, gsaw;
  uint8_t spc, att, exp, last, stall;
  uint8_t ticks;
  // STYLE_PARKED. Words: 0 none, 1 + segment. Generations move by 2 per post.
  uint8_t bell, heard, eparked;       // the executor's control word (executor.rs Control)
  uint8_t agen, aword, aseen, taken;  // the aim, which only the executor posts
  uint8_t owner, ewake, stop;         // the executor's side of the parker
  uint8_t retired[2];                 // per segment: the post that replaced it, 0 = released
  uint8_t kpc, kgen, kword, kseen, kslept, kstop, kparked;  // the parker thread
} State;

enum { OK = 0, V_LOST_WAKE, V_STALE_WAIT, V_SKIP_PARKED, V_GATE, V_LEAK, V_UNMAPPED };
static const char * viol_name[] = {"OK",     "V_LOST_WAKE", "V_STALE_WAIT", "V_SKIP_PARKED",
                                   "V_GATE", "V_LEAK",      "V_UNMAPPED"};

static long cov_deliver, cov_park, cov_wake, cov_wake_void, cov_skip_idle, cov_eagain, cov_dodge,
  cov_reattach, cov_tick, cov_deliver_b, cov_crash_counted, cov_swept, cov_ring, cov_reaim,
  cov_window, cov_window_aim, cov_reclaimed;

#include "../explore.h"

#define LABEL(...) snprintf(label, FLUX2_MODEL_ACT_LEN, __VA_ARGS__)

// What the publisher reads: every number's count (the header word stands for the numberless,
// whom the model does not have), or under BUG_TWO_STORES the old header sum.
static int gate(const State * s, int c)
{
  if (c != 0) return s->waiters[c];
#if defined(BUG_TWO_STORES)
  return s->waiters[0] + s->dsum;
#elif defined(BUG_GATE_ONE_NUMBER)
  return s->waiters[0];
#else
  return s->waiters[0] + s->dnum;
#endif
}

static int live_parked(const State * s, int c)
{
  return s->parked[c] || (c == 0 && s->dparked) || (s->kparked && s->kword == c + 1);
}

static int pub_step(const State * in, State * out, char * label, int * viol)
{
  *out = *in;
  const int c = in->cur;
  switch (in->ppc) {
    case P_COMMIT:
#if RESTART_AFTER > 0
      if (in->pframe == RESTART_AFTER && in->cur == 0) {
        out->cur = 1;  // fresh word and tickets; the old word never moves again
        LABEL("pub:restart seg0->seg1");
        return 1;
      }
#endif
      if (in->pframe >= FRAMES) {
        out->ppc = P_DONE;
        LABEL("pub:done");
        return 1;
      }
      out->latest[c]++;
      out->pframe++;
      out->ppc = P_BUMP;
      LABEL("pub:commit seg%d t%d", c, out->latest[c]);
      return 1;
    case P_BUMP:
#ifdef BUG_GATE_EARLY
      out->gsaw = gate(in, c) > 0;
#endif
      out->wakeup[c]++;
      out->ppc = P_GATE;
      LABEL("pub:bump seg%d w%d", c, out->wakeup[c]);
      return 1;
    case P_GATE: {
#ifdef BUG_GATE_EARLY
      const int announced = in->gsaw;
#else
      const int announced = gate(in, c) > 0;
#endif
      if (announced) {
        out->ppc = P_WAKE;
        LABEL("pub:gate seg%d waiters>0", c);
        return 1;
      }
      if (live_parked(in, c)) *viol = V_SKIP_PARKED;
      cov_skip_idle++;
      out->ppc = P_COMMIT;
      LABEL("pub:gate seg%d skip", c);
      return 1;
    }
    case P_WAKE:
      if (live_parked(in, c)) {
        out->parked[c] = 0;
        if (c == 0) out->dparked = 0;
        if (in->kword == c + 1) out->kparked = 0;
        cov_wake++;
        LABEL("pub:wake seg%d fires", c);
      } else {
        cov_wake_void++;
        LABEL("pub:wake seg%d nobody", c);
      }
      out->ppc = P_COMMIT;
      return 1;
    default:
      return 0;
  }
}

#ifdef STYLE_BARE

// take_blocking: sample, take, then wait(): announce, FUTEX_WAIT (the kernel's word check and the
// userspace re-read fold into one SC step), remove.
static int sub_step(const State * in, State * out, char * label, int * viol)
{
  (void)viol;
  *out = *in;
  switch (in->spc) {
    case W_SAMPLE:
      out->exp = in->wakeup[0];
      out->spc = W_TAKE;
      LABEL("sub:sample w%d", in->wakeup[0]);
      return 1;
    case W_TAKE:
      if (in->latest[0] > in->last) {
        out->last = in->latest[0];
        cov_deliver++;
        if (out->last == FINAL_TICKET) {
          out->spc = S_DONE;
          LABEL("sub:deliver t%d, done", out->last);
          return 1;
        }
        LABEL("sub:deliver t%d", out->last);
      } else {
        LABEL("sub:take empty");
      }
#ifdef BUG_NO_ANNOUNCE
      out->spc = W_PARK;
#else
      out->spc = W_ANNOUNCE;
#endif
      return 1;
    case W_ANNOUNCE:
      out->waiters[0]++;
      out->spc = W_PARK;
      LABEL("sub:announce waiters=%d", out->waiters[0]);
      return 1;
    case W_PARK:
#ifndef BUG_PARK_NOCHECK
      if (in->wakeup[0] != in->exp) {
        cov_dodge++;
        out->spc = W_REMOVE;
        LABEL("sub:park refused w%d!=%d", in->wakeup[0], in->exp);
        return 1;
      }
#endif
      out->parked[0] = 1;
      out->spc = W_PARKED;
      cov_park++;
      LABEL("sub:parked w%d", in->exp);
      return 1;
    case W_PARKED:
      if (in->parked[0]) return 0;
      out->spc = W_REMOVE;
      LABEL("sub:woken");
      return 1;
    case W_REMOVE:
#ifndef BUG_NO_ANNOUNCE
      out->waiters[0]--;
#endif
      out->spc = W_SAMPLE;
      LABEL("sub:remove waiters=%d", out->waiters[0]);
      return 1;
    default:
      return 0;
  }
}

static int sub_blocked(const State * s)
{
  return s->spc == W_PARKED && s->parked[0];
}

#else

#ifdef BUG_NO_RESYNC
#define NOTICES_ROTATION 0
#else
#define NOTICES_ROTATION 1
#endif

#ifdef BUG_SAMPLE_AFTER
#define LOOP_HEAD E_TAKE
#else
#define LOOP_HEAD E_SAMPLE
#endif

#ifndef STYLE_PARKED
#define SUB_END S_DONE
#else
#define SUB_END E_STOP  // Executor drop: stop the parker, then join it

// A word stays mapped while the channel, the parker's owner or a retired entry holds it.
static int mapped(const State * s, int seg)
{
  return (s->att == seg && !s->stop) || s->owner == seg + 1 || s->retired[seg] != 0;
}

// Aim::post: one seqlock write, which the parker reads whole or not at all. The replaced owner is
// retired with this post's generation.
static void post(State * out, int word, int seen)
{
  out->agen += 2;
  out->aword = (uint8_t)word;
  out->aseen = (uint8_t)seen;
  out->ewake = out->owner;
  if (out->owner != 0) out->retired[out->owner - 1] = out->agen;
  out->owner = (uint8_t)word;
}

// After a post: futex_wake on the replaced word, or on the aim's generation if there was none.
static void wake_replaced(const State * in, State * out)
{
#ifdef BUG_REAIM_NO_WAKE
  if (in->ewake != 0) return;
#endif
  if (in->kparked && in->kword == in->ewake) out->kparked = 0;
}
#endif

// A take after a stalled one checks the signpost epoch first (Channel::maybe_reattach), and the
// executor's sync then moves the gate before resampling. No wait outlives its call, so none is
// left on the old word.
static int reattach(const State * in, State * out, char * label)
{
  const int a = in->att;
  out->waiters[a]--;
  out->att = in->cur;
#ifndef BUG_NO_REGATE
  out->waiters[in->cur]++;
#endif
  out->last = 0;
  out->stall = 0;
  out->spc = LOOP_HEAD;
  cov_reattach++;
  LABEL("sub:reattach seg%d->seg%d", a, in->cur);
  return 1;
}

static int sub_step(const State * in, State * out, char * label, int * viol)
{
  *out = *in;
  const int a = in->att;
  switch (in->spc) {
    case E_ATTACH:
      out->waiters[0]++;
      out->spc = LOOP_HEAD;
      LABEL("sub:attach seg0");
      return 1;
    case E_SAMPLE:
      out->exp = in->wakeup[a];
#ifdef BUG_SAMPLE_AFTER
      out->spc = E_PARK;
#else
      out->spc = E_TAKE;
#endif
      LABEL("sub:sample seg%d w%d", a, in->wakeup[a]);
      return 1;
    case E_TAKE:
      if (NOTICES_ROTATION && in->stall > 0 && in->cur != a) return reattach(in, out, label);
      if (in->latest[a] > in->last) {
        out->last = in->latest[a];
        out->stall = 0;
        cov_deliver++;
        if (a == 1) cov_deliver_b++;
        if (a == FINAL_SEG && out->last == FINAL_TICKET) {
          out->spc = SUB_END;
          LABEL("sub:deliver seg%d t%d, done", a, out->last);
          return 1;
        }
        LABEL("sub:deliver seg%d t%d", a, out->last);
      } else {
        out->stall = 1;
        if (a == FINAL_SEG && in->last == FINAL_TICKET) {
          out->spc = SUB_END;
          LABEL("sub:seg%d idle, done", a);
          return 1;
        }
        LABEL("sub:take seg%d empty", a);
      }
#ifdef BUG_SAMPLE_AFTER
      out->spc = E_SAMPLE;
#else
      out->spc = E_PARK;
#endif
      return 1;
#ifdef STYLE_PARKED
    case E_PARK:
      // Executor::park: release what the parker has moved past, then aim it at this word with
      // the seq the pass sampled.
      for (int g = 0; g < 2; g++) {
#ifndef BUG_RECLAIM_EARLY
        if (in->retired[g] == 0 || in->taken < in->retired[g]) continue;
#endif
        if (out->retired[g] != 0) cov_reclaimed++;
        out->retired[g] = 0;
      }
      if (in->waiters[a] <= 0) *viol = V_GATE;
      if (in->owner == a + 1) {
        out->spc = E_SLEEP;
        LABEL("sub:aim kept seg%d", a);
        return 1;
      }
      if (in->owner != 0) cov_reaim++;
#ifdef BUG_SEEN_AT_POST
      post(out, a + 1, in->wakeup[a]);
#else
      post(out, a + 1, in->exp);
#endif
      out->spc = E_AIMWAKE;
      LABEL("sub:aim seg%d seen w%d gen %d", a, out->aseen, out->agen);
      return 1;
    case E_AIMWAKE:
    case E_STOPWAKE:
      wake_replaced(in, out);
      out->spc = in->spc == E_AIMWAKE ? E_SLEEP : E_JOIN;
      LABEL("sub:wake replaced word %d", in->ewake);
      return 1;
    case E_SLEEP:
#ifdef BUG_HEARD_AT_SLEEP
      out->heard = in->bell;
#else
      if (in->bell != in->heard) {
        cov_eagain++;
        out->spc = E_DRAIN;
        LABEL("sub:sleep refused bell %d!=%d", in->bell, in->heard);
        return 1;
      }
#endif
      out->eparked = 1;
      cov_park++;
      out->spc = E_WAIT;
      LABEL("sub:sleep bell %d", in->bell);
      return 1;
    case E_WAIT:
      if (in->eparked) return 0;
      out->spc = E_DRAIN;
      LABEL("sub:woken");
      return 1;
    case E_DRAIN:
      out->heard = in->bell;
      out->spc = LOOP_HEAD;
      LABEL("sub:drain bell %d", in->bell);
      return 1;
    case E_STOP:
      out->stop = 1;
      post(out, 0, 0);
      out->spc = E_STOPWAKE;
      LABEL("sub:stop parker gen %d", out->agen);
      return 1;
    case E_JOIN:
      if (in->kpc != K_DONE) return 0;
      out->spc = S_DONE;
      LABEL("sub:joined");
      return 1;
#else
    case E_PARK:
      // futex_waitv: the kernel's check of the word and the sleep are one step.
      if (in->waiters[a] <= 0) *viol = V_GATE;
      if (in->wakeup[a] != in->exp) {
        cov_eagain++;
        out->spc = LOOP_HEAD;
        LABEL("sub:wait eagain w%d!=%d", in->wakeup[a], in->exp);
        return 1;
      }
      out->parked[a] = 1;
      cov_park++;
      out->spc = E_WAIT;
      LABEL("sub:wait parked w%d", in->exp);
      return 1;
    case E_WAIT:
      if (in->parked[a]) return 0;
      out->spc = LOOP_HEAD;
      LABEL("sub:woken");
      return 1;
#endif
    default:
      return 0;
  }
}

static int sub_blocked(const State * s)
{
#ifdef STYLE_PARKED
  return s->spc == E_WAIT && s->eparked;
#else
  return s->spc == E_WAIT && s->parked[s->att];
#endif
}

#endif

// Aim::park. A futex_wait compares one word, never the stop flag, and a channel word's wait never
// the aim: a stop or a post landing between the parker's check and its sleep is seen only at the
// PARK_NS timeout (park_timeout).
static int parker_step(const State * in, State * out, char * label, int * viol)
{
#ifdef STYLE_PARKED
  *out = *in;
  const int w = in->kword - 1;
  switch (in->kpc) {
    case K_TOP:
      out->kpc = in->stop ? K_DONE : K_GEN;
      LABEL("parker:%s", in->stop ? "stop" : "loop");
      return 1;
    case K_GEN:
      if (in->agen != in->kgen) {
        out->kgen = in->agen;
        out->kword = in->aword;
        out->kseen = in->aseen;
        out->taken = in->agen;
        LABEL("parker:read aim gen %d word %d seen %d", in->agen, in->aword, in->aseen);
      } else {
        LABEL("parker:aim unchanged");
      }
      out->kpc = K_WAIT;
      return 1;
    case K_WAIT:
      if (in->kword == 0 ? in->agen != in->kgen : in->wakeup[w] != in->kseen) {
        if (in->kword != 0 && !mapped(in, w)) *viol = V_UNMAPPED;
        out->kpc = in->kword == 0 ? K_TOP : K_WOKEN;
        LABEL("parker:wait refused");
        return 1;
      }
      if (in->kword != 0 && !mapped(in, w)) *viol = V_UNMAPPED;
      out->kparked = 1;
      out->kslept = in->agen;
      out->kstop = in->stop;
      out->kpc = K_SLEPT;
      LABEL("parker:sleep on word %d", in->kword);
      return 1;
    case K_SLEPT:
      if (in->kparked) return 0;
      out->kpc = in->kword == 0 ? K_TOP : K_WOKEN;
      LABEL("parker:woken");
      return 1;
    case K_WOKEN:
      if (!mapped(in, w)) *viol = V_UNMAPPED;
      if (in->wakeup[w] == in->kseen) {
        out->kpc = K_TOP;
        LABEL("parker:word unchanged");
        return 1;
      }
      out->kseen = in->wakeup[w];
      out->kpc = K_RING;
      LABEL("parker:word moved to w%d", in->wakeup[w]);
      return 1;
    case K_RING:
      out->bell++;
      out->eparked = 0;
      cov_ring++;
      out->kpc = K_TOP;
      LABEL("parker:ring bell %d", out->bell);
      return 1;
    default:
      return 0;
  }
#else
  (void)in;
  (void)out;
  (void)label;
  (void)viol;
  return 0;
#endif
}

// The intended exception: only a parker that went to sleep past a stop, or on a channel word past
// a post it had not read, times out.
static int park_timeout(const State * in, State * out, char * label)
{
  if (!in->kparked || !(in->kstop || (in->kword != 0 && in->kslept != in->kgen))) return 0;
  *out = *in;
  out->kparked = 0;
  if (in->kstop)
    cov_window++;
  else
    cov_window_aim++;
  LABEL("parker:PARK_NS timeout (%s window)", in->kstop ? "stop" : "re-aim");
  return 1;
}

// One bare wait() under number 1, then done; it may die at any step before that.
static int doomed_step(const State * in, State * out, char * label)
{
#ifdef CRASH
  if (in->dead) return 0;
  *out = *in;
  switch (in->dpc) {
    case D_SAMPLE:
      out->dexp = in->wakeup[0];
      out->dpc = D_ANNOUNCE;
      LABEL("doomed:sample w%d", in->wakeup[0]);
      return 1;
    case D_ANNOUNCE:
#ifdef BUG_TWO_STORES
      out->dsum++;
      out->dpc = D_ANNOUNCE2;
      LABEL("doomed:announce sum");
#else
      out->dnum++;
      out->dpc = D_PARK;
      LABEL("doomed:announce n1");
#endif
      return 1;
    case D_ANNOUNCE2:
      out->dnum++;
      out->dpc = D_PARK;
      LABEL("doomed:announce n1");
      return 1;
    case D_PARK:
      if (in->wakeup[0] != in->dexp) {
        out->dpc = D_REMOVE;
        LABEL("doomed:park refused");
        return 1;
      }
      out->dparked = 1;
      out->dpc = D_PARKED;
      LABEL("doomed:parked");
      return 1;
    case D_PARKED:
      if (in->dparked) return 0;
      out->dpc = D_REMOVE;
      LABEL("doomed:woken");
      return 1;
    case D_REMOVE:
      out->dnum--;
#ifdef BUG_TWO_STORES
      out->dpc = D_REMOVE2;
#else
      out->dpc = D_DONE;
#endif
      LABEL("doomed:remove n1");
      return 1;
    case D_REMOVE2:
      out->dsum--;
      out->dpc = D_DONE;
      LABEL("doomed:remove sum");
      return 1;
    default:
      return 0;
  }
#else
  (void)in;
  (void)out;
  (void)label;
  return 0;
#endif
}

// SIGKILL at any step: the kernel drops the park and the number's lock, the counts stay.
static int crash_step(const State * in, State * out, char * label)
{
#ifdef CRASH
  if (in->dead || in->dpc == D_DONE) return 0;
  *out = *in;
  out->dead = 1;
  out->dparked = 0;
  if (in->dnum != 0 || in->dsum != 0) cov_crash_counted++;
  LABEL("doomed:CRASH at %d", in->dpc);
  return 1;
#else
  (void)in;
  (void)out;
  (void)label;
  return 0;
#endif
}

// The publisher's sweep finds number 1's lock free once its holder is dead and clears its counts.
static int sweep_step(const State * in, State * out, char * label)
{
  if (!in->dead || in->dnum == 0) return 0;
  *out = *in;
#ifdef BUG_TWO_STORES
  out->dsum -= in->dnum;
#endif
  out->dnum = 0;
  cov_swept++;
  LABEL("pub:sweep n1 (dead)");
  return 1;
}

// The bounded wait's timeout. Enabled only after the replacement exists (or the publisher is
// done): before it the wake protocol must work alone, and budget burnt there would fake a stuck
// recovery later.
static int tick_step(const State * in, State * out, char * label)
{
#if TICK_BUDGET > 0
  if (!sub_blocked(in) || (in->ppc != P_DONE && in->cur == 0) || in->ticks >= TICK_BUDGET) return 0;
  *out = *in;
  out->ticks++;
#if defined(STYLE_BARE)
  out->parked[0] = 0;
  out->spc = W_REMOVE;
#elif defined(STYLE_PARKED)
  out->eparked = 0;
  out->spc = E_DRAIN;
#else
  out->parked[in->att] = 0;
  out->spc = LOOP_HEAD;
#endif
  cov_tick++;
  LABEL("tick %d/%d", out->ticks, TICK_BUDGET);
  return 1;
#else
  (void)in;
  (void)out;
  (void)label;
  (void)sub_blocked;
  return 0;
#endif
}

static int check_terminal(const State * s)
{
  if (s->dead && gate(s, 0) != s->waiters[0]) return V_LEAK;
  if (s->ppc == P_DONE && s->spc == S_DONE) return OK;
  return s->att == s->cur ? V_LOST_WAKE : V_STALE_WAIT;
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
    for (int actor = 0; actor < 8; actor++) {
      int viol = OK, ran;
      if (actor == 0)
        ran = pub_step(&pool[i], &next, label, &viol);
      else if (actor == 1)
        ran = sub_step(&pool[i], &next, label, &viol);
      else if (actor == 2)
        ran = tick_step(&pool[i], &next, label);
      else if (actor == 3)
        ran = doomed_step(&pool[i], &next, label);
      else if (actor == 4)
        ran = crash_step(&pool[i], &next, label);
      else if (actor == 5)
        ran = sweep_step(&pool[i], &next, label);
      else if (actor == 6)
        ran = parker_step(&pool[i], &next, label, &viol);
      else
        ran = park_timeout(&pool[i], &next, label);
      if (!ran) continue;
      enabled++;
      transitions++;
      if (viol != OK) return report(i, label, viol);
      intern(&next, i, label, &is_new);
    }
    int viol = enabled ? OK : check_terminal(&pool[i]);
    if (viol != OK) return report(i, "terminal", viol);
  }

  printf(
    "PASS  FRAMES=%d RESTART_AFTER=%d TICK_BUDGET=%d%s%s\n", FRAMES, RESTART_AFTER, TICK_BUDGET,
#if defined(STYLE_BARE)
    " STYLE_BARE",
#elif defined(STYLE_PARKED)
    " STYLE_PARKED",
#else
    "",
#endif
#ifdef CRASH
    " CRASH"
#else
    ""
#endif
  );
  printf("      states=%d transitions=%ld\n", nstates, transitions);
  printf(
    "      deliver=%ld park=%ld wake=%ld wake_void=%ld skip_idle=%ld eagain=%ld dodge=%ld "
    "reattach=%ld tick=%ld deliver_b=%ld crash_counted=%ld swept=%ld ring=%ld reaim=%ld "
    "window=%ld window_aim=%ld reclaimed=%ld\n",
    cov_deliver, cov_park, cov_wake, cov_wake_void, cov_skip_idle, cov_eagain, cov_dodge,
    cov_reattach, cov_tick, cov_deliver_b, cov_crash_counted, cov_swept, cov_ring, cov_reaim,
    cov_window, cov_window_aim, cov_reclaimed);

  int vac = 0;
#define REQUIRE(c)                      \
  do {                                  \
    if ((c) == 0) {                     \
      printf("VACUOUS  " #c " is 0\n"); \
      vac = 1;                          \
    }                                   \
  } while (0)
  REQUIRE(cov_deliver);
  REQUIRE(cov_park);
  REQUIRE(cov_wake);
  REQUIRE(cov_wake_void);
  REQUIRE(cov_skip_idle);
#ifdef STYLE_BARE
  REQUIRE(cov_dodge);
#else
  REQUIRE(cov_eagain);
#endif
#ifdef CRASH
  REQUIRE(cov_crash_counted);
  REQUIRE(cov_swept);
#endif
#if RESTART_AFTER > 0
  REQUIRE(cov_reattach);
  REQUIRE(cov_tick);
  REQUIRE(cov_deliver_b);
#endif
#ifdef STYLE_PARKED
  REQUIRE(cov_ring);
  REQUIRE(cov_window);
#if RESTART_AFTER > 0
  REQUIRE(cov_reaim);
  REQUIRE(cov_window_aim);
  REQUIRE(cov_reclaimed);
#endif
#endif
  return vac ? 3 : 0;
}
