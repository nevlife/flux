//! Operation sequences over real processes, checked against a model (syzkaller style). A publisher
//! and two subscriber processes run the ops a coordinator sends one at a time, so a sequential
//! model of the ring predicts every reply: each publish's ticket or backpressure, the frame a take
//! returns with its `lost` and refusals, and every subscriber number's borrows in shared memory.
//! Frames carry their ticket, shape and hash, so a subscriber that reads a torn, rewritten or
//! mislabelled frame (a reused slot that kept the last frame's length or shape) fails on its own.
//!
//! A failing sequence is shrunk and saved under `tests/op_fuzz/`, and
//! `fixed_sequences_match_the_model` replays every saved one. `FLUX_OPFUZZ_SECS` (default 2) sets
//! the length and `FLUX_OPFUZZ_SEED` the first seed.

mod common;

use std::collections::BTreeMap;
use std::fmt::{self, Write as _};
use std::io::BufRead;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

use common::{Rng, Worker, check, frame, leftovers, reply, shape_of, violation};
use flux_core::{
    Channel, DEFAULT_MAX_SUBSCRIBERS, DType, FrameView, MemoryPolicy, NO_SCHEMA, Published, QoS,
    WriteSlot, fuzz, signpost_name,
};

const SLOT: u32 = 512;
const SUBS: usize = 2;
const ROLE: &str = "FLUX_OPFUZZ_ROLE";
const SIGNPOST: &str = "FLUX_OPFUZZ_SIGNPOST";
const ARGS: &str = "FLUX_OPFUZZ_ARGS";

// ---- ops

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Who {
    Pub,
    Sub(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Op {
    Publish(u32),
    Loan(u32),
    Commit(usize),
    CommitBad(usize),
    Abort(usize),
    Restart,
    Take(usize),
    Release(usize, usize),
    Kill(Who),
    Stop(Who),
    Cont(Who),
}

impl fmt::Display for Who {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Who::Pub => write!(f, "p"),
            Who::Sub(j) => write!(f, "s{j}"),
        }
    }
}

impl fmt::Display for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Op::Publish(c) => write!(f, "p publish {c}"),
            Op::Loan(c) => write!(f, "p loan {c}"),
            Op::Commit(k) => write!(f, "p commit {k}"),
            Op::CommitBad(k) => write!(f, "p commitbad {k}"),
            Op::Abort(k) => write!(f, "p abort {k}"),
            Op::Restart => write!(f, "p restart"),
            Op::Take(j) => write!(f, "s{j} take"),
            Op::Release(j, k) => write!(f, "s{j} release {k}"),
            Op::Kill(w) => write!(f, "{w} kill"),
            Op::Stop(w) => write!(f, "{w} stop"),
            Op::Cont(w) => write!(f, "{w} cont"),
        }
    }
}

fn parse_who(s: &str) -> Who {
    match s.strip_prefix('s') {
        Some(j) => Who::Sub(j.parse().unwrap()),
        None => Who::Pub,
    }
}

fn parse_op(line: &str) -> Op {
    let t: Vec<&str> = line.split_whitespace().collect();
    let n = |i: usize| t[i].parse::<u32>().unwrap();
    let k = |i: usize| t[i].parse::<usize>().unwrap();
    match (t[0], t[1]) {
        ("p", "publish") => Op::Publish(n(2)),
        ("p", "loan") => Op::Loan(n(2)),
        ("p", "commit") => Op::Commit(k(2)),
        ("p", "commitbad") => Op::CommitBad(k(2)),
        ("p", "abort") => Op::Abort(k(2)),
        ("p", "restart") => Op::Restart,
        (_, "take") => Op::Take(t[0][1..].parse().unwrap()),
        (_, "release") => Op::Release(t[0][1..].parse().unwrap(), k(2)),
        (w, "kill") => Op::Kill(parse_who(w)),
        (w, "stop") => Op::Stop(parse_who(w)),
        (w, "cont") => Op::Cont(parse_who(w)),
        _ => panic!("unknown op {line:?}"),
    }
}

#[derive(Clone, Copy, Debug)]
struct SubQos {
    depth: u32,
    borrow: u32,
    replay: u32,
}

#[derive(Clone, Debug)]
struct Config {
    slots: u32,
    subs: [SubQos; SUBS],
}

impl fmt::Display for Config {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "slots={}", self.slots)?;
        for (j, q) in self.subs.iter().enumerate() {
            write!(f, " s{j}={},{},{}", q.depth, q.borrow, q.replay)?;
        }
        Ok(())
    }
}

fn parse_config(line: &str) -> Config {
    let mut t = line.split_whitespace();
    let slots = t.next().unwrap()["slots=".len()..].parse().unwrap();
    let subs = [(); SUBS].map(|()| {
        let v: Vec<u32> = t.next().unwrap()[3..]
            .split(',')
            .map(|x| x.parse().unwrap())
            .collect();
        SubQos {
            depth: v[0],
            borrow: v[1],
            replay: v[2],
        }
    });
    Config { slots, subs }
}

// ---- workers

struct Publisher {
    ch: Channel,
    nonce: u64,
    next: u64,
    restarts: u64,
    loans: Vec<(WriteSlot, u32)>,
}

impl Publisher {
    fn new(signpost: &str, slots: u32, restarts: u64) -> Publisher {
        let ch = Channel::create(
            signpost,
            SLOT,
            slots,
            DEFAULT_MAX_SUBSCRIBERS,
            NO_SCHEMA,
            MemoryPolicy::default(),
        )
        .unwrap_or_else(|e| violation(&format!("create: {e}")));
        Publisher {
            ch,
            nonce: u64::from(std::process::id()) << 20 | restarts,
            next: 1,
            restarts,
            loans: Vec::new(),
        }
    }

    fn outcome(&mut self, r: Published) -> String {
        match r {
            Published::Ok => {
                self.next += 1;
                format!("ok {}", self.next - 1)
            }
            Published::Backpressure => "backpressure".into(),
            other => format!("{other}"),
        }
    }

    fn take_loan(&mut self, k: usize) -> Option<(WriteSlot, u32)> {
        (!self.loans.is_empty()).then(|| self.loans.remove(k % self.loans.len()))
    }

    fn run(&mut self, cmd: &str, arg: usize) -> String {
        let code = u32::try_from(arg).unwrap_or(0);
        match cmd {
            "publish" => {
                let (dt, shape) = shape_of(code, SLOT);
                let f = frame(self.next, self.nonce, dt, &shape);
                let r = self.ch.publish_typed(&f, dt, &shape);
                self.outcome(r)
            }
            "loan" => {
                let (dt, shape) = shape_of(code, SLOT);
                let w = if code % 2 == 1 {
                    self.ch.loan()
                } else {
                    self.ch.loan_typed(dt, &shape)
                };
                let Some(mut w) = w else { return "none".into() };
                // A writer part way through: whatever the slot held before is gone.
                w.data_mut().fill(0xA5);
                self.loans.push((w, code));
                "loan".into()
            }
            "commit" => {
                let Some((mut w, code)) = self.take_loan(arg) else {
                    return "empty".into();
                };
                let (dt, shape) = shape_of(code, SLOT);
                let f = frame(self.next, self.nonce, dt, &shape);
                w.data_mut()[..f.len()].copy_from_slice(&f);
                let r = if code % 2 == 1 {
                    w.commit_len(f.len())
                } else {
                    w.commit()
                };
                self.outcome(r)
            }
            "commitbad" => {
                let Some((w, _)) = self.take_loan(arg) else {
                    return "empty".into();
                };
                let past = w.capacity() + 1;
                match w.commit_len(past) {
                    Published::TooLarge => "toolarge".into(),
                    r => violation(&format!("a commit past the slot gave {r}")),
                }
            }
            "abort" => match self.take_loan(arg) {
                Some(_) => "aborted".into(),
                None => "empty".into(),
            },
            _ => violation(&format!("unknown command {cmd}")),
        }
    }
}

fn publisher_role(signpost: &str, slots: u32) {
    let mut p = Publisher::new(signpost, slots, 0);
    reply(&format!("ready {}", p.nonce));
    for line in std::io::stdin().lock().lines() {
        let line = line.unwrap();
        let mut t = line.split(' ');
        let cmd = t.next().unwrap();
        let arg = t.next().map_or(0, |a| a.parse().unwrap());
        if cmd == "restart" {
            let restarts = p.restarts + 1;
            drop(p);
            p = Publisher::new(signpost, slots, restarts);
            reply(&format!("ready {}", p.nonce));
            continue;
        }
        let r = p.run(cmd, arg);
        reply(&r);
    }
}

fn subscriber_role(signpost: &str, q: &[u32]) {
    let mut ch = Channel::open(signpost, NO_SCHEMA, MemoryPolicy::default())
        .unwrap_or_else(|e| violation(&format!("open: {e}")));
    let qos = QoS::new(q[0])
        .unwrap()
        .transient_local(q[2])
        .max_borrow(q[1]);
    ch.set_qos(qos.unwrap()).unwrap();
    reply("ready");
    let mut held: Vec<FrameView> = Vec::new();
    let mut nonces = BTreeMap::new();
    for line in std::io::stdin().lock().lines() {
        let line = line.unwrap();
        let r = if line == "take" {
            let got = ch.take();
            let refused = *ch.refused();
            if refused.no_reader + refused.contended + refused.bad_frame + refused.fence != 0 {
                violation(&format!("refused {refused:?}"));
            }
            let (lost, max) = (ch.lost(), refused.max_borrow);
            match got {
                Some(v) => {
                    let nonce = check(&v).unwrap_or_else(|e| violation(&e));
                    if *nonces.entry(ch.attach_generation()).or_insert(nonce) != nonce {
                        violation("two publishers' frames in one attach");
                    }
                    let r = format!("frame {} {nonce} {lost} {max}", v.ticket());
                    held.push(v);
                    r
                }
                None => format!("none {lost} {max}"),
            }
        } else {
            let k: usize = line["release ".len()..].parse().unwrap();
            if held.is_empty() {
                "empty".into()
            } else {
                let v = held.remove(k % held.len());
                if let Err(e) = check(&v) {
                    violation(&format!("rewritten while held: {e}"));
                }
                format!("released {}", v.ticket())
            }
        };
        reply(&r);
    }
}

/// Runs only as a child of the tests below.
#[test]
fn op_worker() {
    let (Ok(role), Ok(signpost), Ok(args)) = (
        std::env::var(ROLE),
        std::env::var(SIGNPOST),
        std::env::var(ARGS),
    ) else {
        return;
    };
    let args: Vec<u32> = args
        .split_whitespace()
        .map(|a| a.parse().unwrap())
        .collect();
    match role.as_str() {
        "pub" => publisher_role(&signpost, args[0]),
        _ => subscriber_role(&signpost, &args),
    }
}

// ---- model

#[derive(Clone, Copy, PartialEq, Eq)]
enum Slot {
    Empty,
    Claimed,
    Frame(u64),
}

/// One publisher's segment.
struct Seg {
    slots: Vec<Slot>,
    latest: u64,
    next: u64,
    /// Borrows per subscriber number and slot.
    counts: Vec<Vec<u32>>,
    nonce: u64,
}

struct View {
    seg: usize,
    number: usize,
    slot: usize,
    ticket: u64,
}

struct SubModel {
    alive: bool,
    q: SubQos,
    seg: usize,
    number: usize,
    cursor: u64,
    lost: u64,
    stalled: u32,
    refused: u64,
    held: Vec<View>,
}

#[derive(Default)]
struct Model {
    segs: Vec<Seg>,
    subs: Vec<SubModel>,
    loans: Vec<usize>,
}

/// Counted over every run, to show what the fuzzer reached.
#[derive(Default, Debug)]
struct Stats {
    sequences: u64,
    ops: u64,
    frames: u64,
    backpressure: u64,
    reclaims: u64,
    reattaches: u64,
    kills: u64,
    stops: u64,
    /// Releases by how many frames were committed after the view's: at the slot count or more,
    /// the slot would have been rewritten had the borrow not held it.
    lag: BTreeMap<u64, u64>,
}

const PROBE_EVERY: u32 = 8;

impl Model {
    fn cur(&self) -> usize {
        self.segs.len() - 1
    }

    fn new_segment(&mut self, slots: u32, nonce: u64) {
        self.segs.push(Seg {
            slots: vec![Slot::Empty; slots as usize],
            latest: 0,
            next: 1,
            counts: Vec::new(),
            nonce,
        });
        self.loans.clear();
    }

    /// The lowest number no live subscriber of the current segment holds, cleared of what a dead
    /// holder left.
    fn lease(&mut self, j: usize) -> (usize, usize) {
        let cur = self.cur();
        let taken: Vec<usize> = (0..self.subs.len())
            .filter(|&k| k != j && self.subs[k].alive && self.subs[k].seg == cur)
            .map(|k| self.subs[k].number)
            .collect();
        let number = (0..=SUBS).find(|n| !taken.contains(n)).unwrap();
        let seg = &mut self.segs[cur];
        let slots = seg.slots.len();
        while seg.counts.len() <= number {
            seg.counts.push(vec![0; slots]);
        }
        seg.counts[number].fill(0);
        (cur, number)
    }

    fn spawn_sub(&mut self, j: usize, q: SubQos) {
        let sub = SubModel {
            alive: true,
            q,
            seg: usize::MAX,
            number: usize::MAX,
            cursor: 0,
            lost: 0,
            stalled: 0,
            refused: 0,
            held: Vec::new(),
        };
        if j == self.subs.len() {
            self.subs.push(sub);
        } else {
            self.subs[j] = sub;
        }
        let (seg, number) = self.lease(j);
        let s = &mut self.subs[j];
        (s.seg, s.number) = (seg, number);
        s.cursor = self.segs[seg].latest.saturating_sub(u64::from(q.replay));
    }

    fn claim(&mut self, stats: &mut Stats) -> Option<usize> {
        let cur = self.cur();
        let live: Vec<usize> = self
            .subs
            .iter()
            .filter(|s| s.alive && s.seg == cur)
            .map(|s| s.number)
            .collect();
        for pass in 0..2 {
            let seg = &mut self.segs[cur];
            let pick = (0..seg.slots.len())
                .filter(|&s| seg.slots[s] != Slot::Claimed && seg.counts.iter().all(|c| c[s] == 0))
                .min_by_key(|&s| match seg.slots[s] {
                    Slot::Frame(t) => t,
                    _ => 0,
                });
            if let Some(s) = pick {
                seg.slots[s] = Slot::Claimed;
                return Some(s);
            }
            let dead: Vec<usize> = (0..seg.counts.len())
                .filter(|n| !live.contains(n) && seg.counts[*n].iter().any(|&c| c > 0))
                .collect();
            if pass == 1 || dead.is_empty() {
                break;
            }
            for n in dead {
                seg.counts[n].fill(0);
            }
            stats.reclaims += 1;
        }
        stats.backpressure += 1;
        None
    }

    fn commit(&mut self, s: usize) -> String {
        let seg = self.segs.last_mut().unwrap();
        let t = seg.next;
        seg.next += 1;
        seg.slots[s] = Slot::Frame(t);
        seg.latest = t;
        format!("ok {t}")
    }

    fn take_loan(&mut self, k: usize) -> Option<usize> {
        (!self.loans.is_empty()).then(|| self.loans.remove(k % self.loans.len()))
    }

    /// The reply the worker must give.
    fn apply(&mut self, op: Op, stats: &mut Stats) -> String {
        let cur = self.cur();
        match op {
            Op::Publish(_) => match self.claim(stats) {
                Some(s) => self.commit(s),
                None => "backpressure".into(),
            },
            Op::Loan(_) => match self.claim(stats) {
                Some(s) => {
                    self.loans.push(s);
                    "loan".into()
                }
                None => "none".into(),
            },
            Op::Commit(k) => self
                .take_loan(k)
                .map_or_else(|| "empty".into(), |s| self.commit(s)),
            Op::CommitBad(k) | Op::Abort(k) => match self.take_loan(k) {
                Some(s) => {
                    self.segs[cur].slots[s] = Slot::Empty;
                    if matches!(op, Op::Abort(_)) {
                        "aborted".into()
                    } else {
                        "toolarge".into()
                    }
                }
                None => "empty".into(),
            },
            Op::Take(j) => self.take(j, stats),
            Op::Release(j, k) => {
                let held = &mut self.subs[j].held;
                if held.is_empty() {
                    return "empty".into();
                }
                let v = held.remove(k % held.len());
                let seg = &mut self.segs[v.seg];
                seg.counts[v.number][v.slot] -= 1;
                *stats.lag.entry(seg.latest - v.ticket).or_default() += 1;
                format!("released {}", v.ticket)
            }
            Op::Restart | Op::Kill(_) | Op::Stop(_) | Op::Cont(_) => unreachable!(),
        }
    }

    /// `Channel::take`, sequentially: re-attach, `max_borrow`, then the oldest frame in reach.
    fn take(&mut self, j: usize, stats: &mut Stats) -> String {
        let cur = self.cur();
        let outstanding = |s: &SubModel| s.held.iter().filter(|v| v.seg == s.seg).count();
        let s = &self.subs[j];
        if s.stalled > 0 || outstanding(s) >= s.q.borrow as usize {
            if s.seg != cur {
                let (seg, number) = self.lease(j);
                let s = &mut self.subs[j];
                (s.seg, s.number, s.cursor, s.stalled) = (seg, number, 0, 0);
                stats.reattaches += 1;
            } else if s.stalled >= PROBE_EVERY {
                self.subs[j].stalled = 0;
            }
        }
        let s = &mut self.subs[j];
        if outstanding(s) >= s.q.borrow as usize {
            s.refused += 1;
            return format!("none {} {}", s.lost, s.refused);
        }
        let seg = &mut self.segs[s.seg];
        let floor = seg.latest.saturating_sub(u64::from(s.q.depth));
        if seg.latest > 0 && s.cursor < floor {
            s.lost += floor - s.cursor;
            s.cursor = floor;
        }
        let found = (0..seg.slots.len())
            .filter_map(|i| match seg.slots[i] {
                Slot::Frame(t) if t > s.cursor && t <= seg.latest => Some((t, i)),
                _ => None,
            })
            .min();
        let Some((t, slot)) = found else {
            s.stalled += 1;
            return format!("none {} {}", s.lost, s.refused);
        };
        seg.counts[s.number][slot] += 1;
        s.lost += t - s.cursor - 1;
        s.cursor = t;
        s.stalled = 0;
        s.held.push(View {
            seg: s.seg,
            number: s.number,
            slot,
            ticket: t,
        });
        stats.frames += 1;
        format!("frame {t} {} {} {}", seg.nonce, s.lost, s.refused)
    }

    /// Borrows per number on the current segment, without trailing zeros.
    fn borrows(&self) -> Vec<u64> {
        let mut v: Vec<u64> = self.segs[self.cur()]
            .counts
            .iter()
            .map(|c| c.iter().map(|&n| u64::from(n)).sum())
            .collect();
        while v.last() == Some(&0) {
            v.pop();
        }
        v
    }
}

// ---- runs

struct Run<'a> {
    cfg: &'a Config,
    signpost: String,
    workers: Vec<Worker>,
    model: Model,
}

fn sub_args(q: SubQos) -> String {
    format!("{} {} {}", q.depth, q.borrow, q.replay)
}

fn ready_nonce(r: &str) -> Result<u64, String> {
    r.strip_prefix("ready ")
        .and_then(|n| n.parse().ok())
        .ok_or_else(|| format!("publisher answered {r:?}"))
}

impl Run<'_> {
    fn index(w: Who) -> usize {
        match w {
            Who::Pub => 0,
            Who::Sub(j) => 1 + j,
        }
    }

    fn worker(&self, role: &str, args: &str) -> Worker {
        Worker::spawn(
            "op_worker",
            &[(ROLE, role), (SIGNPOST, &self.signpost), (ARGS, args)],
        )
    }

    fn spawn(&mut self, w: Who) -> Result<(), String> {
        let worker = match w {
            Who::Pub => self.worker("pub", &self.cfg.slots.to_string()),
            Who::Sub(j) => self.worker("sub", &sub_args(self.cfg.subs[j])),
        };
        let i = Run::index(w);
        if i == self.workers.len() {
            self.workers.push(worker);
        } else {
            self.workers[i] = worker;
        }
        let r = self.workers[i].reply()?;
        match w {
            Who::Pub => self.model.new_segment(self.cfg.slots, ready_nonce(&r)?),
            Who::Sub(j) => self.model.spawn_sub(j, self.cfg.subs[j]),
        }
        Ok(())
    }

    fn step(&mut self, op: Op, stats: &mut Stats) -> Result<(), String> {
        let who = match op {
            Op::Kill(w) => {
                let i = Run::index(w);
                self.workers[i].signal(libc::SIGKILL);
                let _ = self.workers[i].child.wait();
                stats.kills += 1;
                // A dead subscriber comes back on its next op, so claims meanwhile meet its
                // borrows with nobody holding its number.
                if let Who::Sub(j) = w {
                    self.workers[i].dead = true;
                    self.model.subs[j].alive = false;
                    return Ok(());
                }
                return self.spawn(w);
            }
            Op::Stop(w) | Op::Cont(w) => {
                let worker = &mut self.workers[Run::index(w)];
                if !worker.dead && worker.stopped != matches!(op, Op::Stop(_)) {
                    worker.signal(if worker.stopped {
                        libc::SIGCONT
                    } else {
                        libc::SIGSTOP
                    });
                    stats.stops += u64::from(worker.stopped);
                }
                return Ok(());
            }
            Op::Take(j) | Op::Release(j, _) => Who::Sub(j),
            _ => Who::Pub,
        };
        if self.workers[Run::index(who)].dead {
            self.spawn(who)?;
        }
        let worker = &mut self.workers[Run::index(who)];
        if worker.stopped {
            return Ok(());
        }
        let line = op.to_string();
        let got = worker.send(line.split_once(' ').unwrap().1)?;
        if op == Op::Restart {
            self.model.new_segment(self.cfg.slots, ready_nonce(&got)?);
            return Ok(());
        }
        let want = self.model.apply(op, stats);
        if got != want {
            return Err(format!("replied {got:?}, the model {want:?}"));
        }
        Ok(())
    }

    fn check_borrows(&self) -> Result<(), String> {
        let mut real = fuzz::borrows(&self.signpost, NO_SCHEMA).unwrap_or_default();
        while real.last() == Some(&0) {
            real.pop();
        }
        let want = self.model.borrows();
        if real != want {
            return Err(format!("borrows per number {real:?}, the model {want:?}"));
        }
        Ok(())
    }
}

#[derive(Debug)]
struct Failure {
    at: Option<usize>,
    why: String,
}

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.at {
            Some(i) => write!(f, "at op {i}: {}", self.why),
            None => write!(f, "at the end: {}", self.why),
        }
    }
}

fn run(cfg: &Config, ops: &[Op], stats: &mut Stats) -> Result<(), Failure> {
    static N: AtomicU32 = AtomicU32::new(0);
    let key = format!(
        "/of{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    );
    let mut r = Run {
        cfg,
        signpost: signpost_name(&key, NO_SCHEMA).unwrap(),
        workers: Vec::new(),
        model: Model::default(),
    };
    let at = |i| move |why| Failure { at: Some(i), why };
    for w in [Who::Pub, Who::Sub(0), Who::Sub(1)] {
        r.spawn(w).map_err(at(0))?;
    }
    for (i, &op) in ops.iter().enumerate() {
        r.step(op, stats)
            .and_then(|()| r.check_borrows())
            .map_err(at(i))?;
        stats.ops += 1;
    }
    let end = |why| Failure { at: None, why };
    for w in r.workers.drain(..).filter(|w| !w.dead) {
        w.finish().map_err(end)?;
    }
    let left = leftovers(&r.signpost);
    if !left.is_empty() {
        return Err(end(format!("left behind: {left:?}")));
    }
    stats.sequences += 1;
    Ok(())
}

// ---- generation, shrinking, fixed cases

fn generate(seed: u64) -> (Config, Vec<Op>) {
    let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let slots = 2 + rng.below(3);
    let subs = [(); SUBS].map(|()| {
        // Past twice the slot count a cursor can fall behind the ticket index.
        let depth = 1 + rng.below(4 * slots + 1);
        SubQos {
            depth,
            borrow: 1 + rng.below(3),
            replay: rng.below(depth + 1),
        }
    });
    let len = 40 + rng.below(160);
    let ops = (0..len)
        .map(|_| {
            let who = [Who::Pub, Who::Sub(0), Who::Sub(1)][rng.index(3)];
            let j = rng.index(2);
            match rng.below(100) {
                0..24 => Op::Publish(rng.below(1024)),
                24..30 => Op::Loan(rng.below(1024)),
                30..36 => Op::Commit(rng.index(3)),
                36..38 => Op::CommitBad(rng.index(3)),
                38..41 => Op::Abort(rng.index(3)),
                41..43 => Op::Restart,
                43..69 => Op::Take(j),
                69..88 => Op::Release(j, rng.index(4)),
                88..92 => Op::Kill(who),
                92..96 => Op::Stop(who),
                _ => Op::Cont(who),
            }
        })
        .collect();
    (Config { slots, subs }, ops)
}

/// Drops chunks of ops, halving the chunk, while the sequence still fails.
fn shrink(cfg: &Config, ops: &[Op]) -> Vec<Op> {
    let mut ops = ops.to_vec();
    let mut chunk = ops.len().div_ceil(2);
    while chunk > 0 {
        let mut i = 0;
        while i < ops.len() {
            let cand: Vec<Op> = [&ops[..i], &ops[(i + chunk).min(ops.len())..]].concat();
            if run(cfg, &cand, &mut Stats::default()).is_err() {
                ops = cand;
            } else {
                i += chunk;
            }
        }
        chunk /= 2;
    }
    ops
}

fn fixed_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/op_fuzz")
}

fn save(cfg: &Config, ops: &[Op], seed: u64, failure: &Failure) -> PathBuf {
    let path = fixed_dir().join(format!("seed_{seed:x}.ops"));
    let mut text = format!("# {failure}\n{cfg}\n");
    for op in ops {
        writeln!(text, "{op}").unwrap();
    }
    std::fs::create_dir_all(fixed_dir()).unwrap();
    std::fs::write(&path, text).unwrap();
    path
}

fn load(path: &Path) -> (Config, Vec<Op>) {
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'));
    let cfg = parse_config(lines.next().unwrap());
    (cfg, lines.map(parse_op).collect())
}

fn env_or(name: &str, default: u64) -> u64 {
    std::env::var(name).map_or(default, |v| {
        let v = v.trim_start_matches("0x");
        u64::from_str_radix(v, 16).unwrap()
    })
}

#[test]
fn op_sequences_match_the_model() {
    let secs = std::env::var("FLUX_OPFUZZ_SECS").map_or(2, |s| s.parse().unwrap());
    let mut seed = env_or("FLUX_OPFUZZ_SEED", 0x5eed_0000);
    let end = Instant::now() + Duration::from_secs(secs);
    let mut stats = Stats::default();
    while Instant::now() < end {
        let (cfg, ops) = generate(seed);
        if let Err(f) = run(&cfg, &ops, &mut stats) {
            let small = shrink(&cfg, &ops);
            let path = save(&cfg, &small, seed, &f);
            panic!(
                "seed {seed:x}: {f}\nshrunk from {} to {} ops, saved as {}; commit it",
                ops.len(),
                small.len(),
                path.display()
            );
        }
        seed += 1;
    }
    eprintln!("{stats:?}");
    assert!(stats.sequences > 0);
}

#[test]
fn fixed_sequences_match_the_model() {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(fixed_dir())
        .unwrap()
        .filter_map(|e| Some(e.ok()?.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "ops"))
        .collect();
    paths.sort();
    assert!(!paths.is_empty());
    for path in paths {
        let (cfg, ops) = load(&path);
        if let Err(f) = run(&cfg, &ops, &mut Stats::default()) {
            panic!("{}: {f}", path.display());
        }
    }
}

/// The oracle itself: a frame under the wrong ticket, under another shape, cut short or with a
/// byte flipped fails the check.
#[test]
fn the_frame_check_finds_a_wrong_ticket_shape_length_or_byte() {
    let mut ch = Channel::new(SLOT, 8).unwrap();
    ch.set_qos(QoS::new(8).unwrap()).unwrap();
    let (dt, shape) = shape_of(34, SLOT);
    assert_eq!((dt, shape.as_slice()), (DType::U16, &[64][..]));
    let mut flipped = frame(5, 9, dt, &shape);
    flipped[100] ^= 1;
    let cases = [
        (frame(1, 9, dt, &shape), dt, vec![64], true),
        (frame(7, 9, dt, &shape), dt, vec![64], false),
        (frame(3, 9, dt, &shape), DType::U8, vec![128], false),
        (
            frame(4, 9, dt, &shape)[..96].to_vec(),
            DType::U8,
            vec![96],
            false,
        ),
        (flipped, dt, vec![64], false),
    ];
    for (bytes, dt, shape, whole) in cases {
        assert_eq!(ch.publish_typed(&bytes, dt, &shape), Published::Ok);
        let v = ch.take().unwrap();
        assert_eq!(check(&v).is_ok(), whole, "ticket {}", v.ticket());
    }
}
