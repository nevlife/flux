//! One thread's dispatch loop: one `futex_waitv` over every channel's wake word and a control
//! word, then a pass that serves ready sources by priority, one frame per visit. Without
//! `futex_waitv` a thread per source sleeps on its word and rings the control word.

use std::cmp::Reverse;
use std::ffi::c_void;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering, fence};
use std::thread::{self, JoinHandle};
use std::{mem, ptr};

use crate::channel::{Channel, ChannelShared, PublisherWatch, Waiter};
use crate::discovery::SignpostMap;
use crate::error::{Error, Result};
use crate::sys;
use crate::trace::{self, event};

/// Default pass budget. A consumer slower than its publisher finds a frame on every take, so the
/// budget is the only thing that ends its pass.
pub const MAX_DRAIN: u32 = 64;
pub const DEFAULT_TICK_NS: i64 = 100_000_000;
/// Set, it makes `Executor::new` choose the parker threads a kernel below 5.16 runs: the tests'
/// way onto the fallback on a newer kernel.
const DISABLE_FUTEX_WAITV_ENV: &str = "FLUX_DISABLE_FUTEX_WAITV";
/// Bounds a parker's sleep: a stop landing between its check and its wait is seen this late.
const PARK_NS: i64 = 50_000_000;

pub trait Source: Send {
    /// False while there is nothing to attach to yet. Asked again every pass.
    fn attach(&mut self) -> Result<bool>;
    /// Takes at most one frame and runs its callback. True if it did.
    fn deliver_one(&mut self) -> Result<bool>;
    /// None until `attach` succeeds, and again after the source drops an orphaned channel.
    fn channel(&self) -> Option<&Channel>;
    /// While there is no channel: the watch whose publisher would end the wait, armed by the last
    /// attach attempt. Without one, an unattached source is retried once a tick.
    fn watch(&self) -> Option<&PublisherWatch> {
        None
    }
    /// The endpoint handle trace events name this source by (`flux_trace_endpoint_init`).
    fn endpoint(&self) -> *const c_void {
        ptr::null()
    }
}

/// "Spinning" and "stop" are two flags: arming one shared flag on loop entry would erase a stop
/// that landed before the loop started.
pub(crate) struct Control {
    /// Bumped and woken by every poke. A wait passes the value it last heard, so a poke landing
    /// before the wait ends it at once.
    bell: AtomicU32,
    heard: AtomicU32,
    /// The entry whose parker rang last, for the wake event; `trace::NONE` once read.
    rang: AtomicU32,
    spinning: AtomicBool,
    stop: AtomicBool,
    interrupted: AtomicBool,
}

impl Control {
    pub(crate) fn new() -> Arc<Control> {
        Arc::new(Control {
            bell: AtomicU32::new(0),
            heard: AtomicU32::new(0),
            rang: AtomicU32::new(trace::NONE),
            spinning: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            interrupted: AtomicBool::new(false),
        })
    }

    pub(crate) fn poke(&self) {
        self.bell.fetch_add(1, Ordering::SeqCst);
        sys::futex_wake_private(&self.bell);
    }

    pub(crate) fn drain(&self) {
        self.heard
            .store(self.bell.load(Ordering::Acquire), Ordering::Relaxed);
    }

    fn waitv(&self) -> sys::Waitv {
        sys::Waitv::private(&self.bell, self.heard.load(Ordering::Relaxed))
    }

    /// Does not count the poke heard.
    fn sleep(&self, timeout_ns: i64) -> sys::Woke {
        sys::futex_wait_private(&self.bell, self.heard.load(Ordering::Relaxed), timeout_ns)
    }

    /// A poke from the parker of `entry`. The bell's increment publishes the store to the `drain`
    /// that reads it.
    fn ring(&self, entry: u32) {
        self.rang.store(entry, Ordering::Relaxed);
        self.poke();
    }

    /// Waits for a poke up to `timeout_ns`, then counts it heard.
    pub(crate) fn wait(&self, timeout_ns: i64) {
        self.sleep(timeout_ns);
        self.drain();
    }

    pub(crate) fn stop_requested(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }

    pub(crate) fn take_interrupt(&self) -> bool {
        self.interrupted.swap(false, Ordering::AcqRel)
    }

    pub(crate) fn set_spinning(&self, on: bool) {
        self.spinning.store(on, Ordering::Release);
    }

    /// On leaving a loop, so the next one neither starts stopped nor wakes on this one's interrupt.
    pub(crate) fn reset(&self) {
        self.stop.store(false, Ordering::Relaxed);
        self.interrupted.store(false, Ordering::Relaxed);
        self.drain();
    }
}

/// Reaches a loop from other threads, and stays valid after the loop's owner is gone.
#[derive(Clone)]
pub struct ExecutorHandle(Arc<Control>);

impl ExecutorHandle {
    pub(crate) fn new(ctl: Arc<Control>) -> ExecutorHandle {
        ExecutorHandle(ctl)
    }

    /// Held if no loop is running yet, so a stop racing the start of `spin` is not lost.
    pub fn stop(&self) {
        self.0.stop.store(true, Ordering::Relaxed);
        self.0.interrupted.store(true, Ordering::Release);
        self.0.poke();
    }

    /// Ends the current wait without ending the loop: `spin` runs `on_pass` and goes on. One
    /// interrupt ends at most one wait.
    pub fn interrupt(&self) {
        self.0.interrupted.store(true, Ordering::Release);
        self.0.poke();
    }

    /// Breaks the wait so a pass runs. With nothing ready, `spin` waits on.
    pub fn wake(&self) {
        self.0.poke();
    }

    pub fn is_spinning(&self) -> bool {
        self.0.spinning.load(Ordering::Acquire)
    }

    pub(crate) fn control(&self) -> &Control {
        &self.0
    }
}

/// Our +1 on the segment's waiter gate and the armed wait live on the block they were placed on.
/// A source can re-attach on its own; removing on its current block would drive that gate negative.
struct Bound {
    sh: Arc<ChannelShared>,
    generation: u32,
    waiter: Waiter,
}

/// A word a parker sleeps on, kept mapped while the parker may touch it.
trait Parked: Send + Sync {
    fn word(&self) -> &AtomicU32;
}

impl Parked for ChannelShared {
    fn word(&self) -> &AtomicU32 {
        self.wake_word()
    }
}

impl Parked for SignpostMap {
    fn word(&self) -> &AtomicU32 {
        self.epoch_word()
    }
}

/// Where a parker sleeps. Only the executor posts, and `generation` is odd while it writes.
struct Aim {
    generation: AtomicU32,
    word: AtomicPtr<AtomicU32>,
    seen: AtomicU32,
    /// The last post the parker read. It touches no word an earlier post replaced.
    taken: AtomicU32,
    stop: AtomicBool,
}

impl Aim {
    fn post(&self, word: *const AtomicU32, seen: u32) -> u32 {
        let g = self.generation.load(Ordering::Relaxed);
        self.generation.store(g.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        self.word.store(word.cast_mut(), Ordering::Relaxed);
        self.seen.store(seen, Ordering::Relaxed);
        self.generation.store(g.wrapping_add(2), Ordering::Release);
        g.wrapping_add(2)
    }

    fn read(&self) -> (u32, *const AtomicU32, u32) {
        loop {
            let g = self.generation.load(Ordering::Acquire);
            let word = self.word.load(Ordering::Relaxed);
            let seen = self.seen.load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if g & 1 == 0 && self.generation.load(Ordering::Relaxed) == g {
                return (g, word, seen);
            }
            std::hint::spin_loop();
        }
    }

    fn park(&self, entry: u32, ctl: &Control) {
        let (mut generation, mut word, mut seen) = self.read();
        while !self.stop.load(Ordering::Acquire) {
            if self.generation.load(Ordering::Acquire) != generation {
                (generation, word, seen) = self.read();
                self.taken.store(generation, Ordering::Release);
            }
            // SAFETY: the executor keeps the word's owner until `taken` reaches the post after it.
            match unsafe { word.as_ref() } {
                None => {
                    sys::futex_wait_private(&self.generation, generation, PARK_NS);
                }
                Some(w) => {
                    sys::futex_wait(w, seen, PARK_NS);
                    let now = w.load(Ordering::Acquire);
                    if now != seen {
                        seen = now;
                        ctl.ring(entry);
                    }
                }
            }
        }
    }
}

/// The fallback for one source: a thread sleeping on the word the merged wait would name, which
/// rings the control word each time it moves. Made when the source is added and only re-aimed
/// after: the executor thread is the real-time one, and a thread's start or exit maps or unmaps
/// its stack.
struct Parker {
    aim: Arc<Aim>,
    owner: Option<Arc<dyn Parked>>,
    /// Replaced owners, each with the post that replaced it.
    retired: Vec<(u32, Arc<dyn Parked>)>,
    thread: Option<JoinHandle<()>>,
    born: u32,
}

impl Parker {
    fn spawn(entry: u32, ctl: &Arc<Control>) -> Result<Parker> {
        sys::watch_fork();
        let aim = Arc::new(Aim {
            generation: AtomicU32::new(0),
            word: AtomicPtr::new(ptr::null_mut()),
            seen: AtomicU32::new(0),
            taken: AtomicU32::new(0),
            stop: AtomicBool::new(false),
        });
        let (a, ctl) = (Arc::clone(&aim), Arc::clone(ctl));
        let thread = sys::spawn_helper(
            thread::Builder::new().name("flux-parker".into()),
            move || a.park(entry, &ctl),
        )
        .map_err(|e| Error::io("spawn", "flux-parker", e))?;
        Ok(Parker {
            aim,
            owner: None,
            retired: Vec::new(),
            thread: Some(thread),
            born: sys::fork_gen(),
        })
    }

    /// `seen` is the value the merged wait would pass, so a move since then rings at once.
    fn retarget(&mut self, target: Option<(Arc<dyn Parked>, u32)>) {
        let (word, seen) = target
            .as_ref()
            .map_or((ptr::null(), 0), |(o, s)| (ptr::from_ref(o.word()), *s));
        let posted = self.aim.post(word, seen);
        match mem::replace(&mut self.owner, target.map(|(o, _)| o)) {
            Some(old) => {
                // Other waiters on the word take this as a stray wake.
                sys::futex_wake(old.word());
                self.retired.push((posted, old));
            }
            None => sys::futex_wake_private(&self.aim.generation),
        }
    }

    fn reclaim(&mut self) {
        let taken = self.aim.taken.load(Ordering::Acquire);
        while let Some(i) = self
            .retired
            .iter()
            .position(|&(posted, _)| taken.wrapping_sub(posted).cast_signed() >= 0)
        {
            sys::retire(self.retired.swap_remove(i).1);
        }
    }

    fn parks_on(&self, word: &AtomicU32) -> bool {
        self.owner.as_ref().is_some_and(|o| ptr::eq(o.word(), word))
    }
}

impl Drop for Parker {
    fn drop(&mut self) {
        self.aim.stop.store(true, Ordering::Release);
        self.retarget(None);
        // A forked child has no copy of the thread to join.
        if let Some(t) = self.thread.take()
            && sys::fork_gen() == self.born
        {
            let _ = t.join();
        }
    }
}

struct Entry<'a> {
    src: Box<dyn Source + 'a>,
    priority: i32,
    bound: Option<Bound>,
    parker: Option<Parker>,
    // Reset at the top of every pass.
    seq: u32,
    probed: bool,
    blocked: bool,
}

pub struct Executor<'a> {
    entries: Vec<Entry<'a>>,
    order: Vec<usize>,
    set: Vec<sys::Waitv>,
    ctl: Arc<Control>,
    /// No `futex_waitv`: every entry's word gets a `Parker`.
    parked: bool,
    pass_budget: u32,
    more: bool,
    on_pass: Option<Box<dyn FnMut() -> Result<()> + Send + 'a>>,
}

/// False below Linux 5.16, or when forced by `DISABLE_FUTEX_WAITV_ENV`.
fn merged_wait() -> Result<bool> {
    if std::env::var_os(DISABLE_FUTEX_WAITV_ENV).is_some() {
        return Ok(false);
    }
    // A word that is not its expected value returns at once, so this only asks whether the call
    // is allowed.
    let probe = AtomicU32::new(0);
    let Err(e) = sys::futex_waitv(&[sys::Waitv::private(&probe, 1)], 0) else {
        return Ok(true);
    };
    match e.raw_os_error() {
        Some(libc::ENOSYS) => Ok(false),
        Some(libc::EPERM) => Err(Error::Unsupported(
            "futex_waitv is forbidden here (a seccomp profile)".into(),
        )),
        _ => Err(Error::io("futex_waitv", "executor", e)),
    }
}

fn entry_number(i: usize) -> u32 {
    u32::try_from(i).expect("entries are at most WAITV_MAX")
}

fn park_target(e: &Entry<'_>) -> Option<(Arc<dyn Parked>, u32)> {
    match &e.bound {
        Some(b) => Some((Arc::clone(&b.sh) as Arc<dyn Parked>, e.seq)),
        None => {
            unattached_watch(e).map(|w| (Arc::clone(w.signpost()) as Arc<dyn Parked>, w.armed()))
        }
    }
}

fn detach(e: &mut Entry<'_>) {
    if let Some(b) = e.bound.take() {
        b.sh.remove_waiter(b.waiter);
        sys::retire(b.sh);
    }
}

fn unattached_watch<'e>(e: &'e Entry<'_>) -> Option<&'e PublisherWatch> {
    if e.src.channel().is_some() {
        return None;
    }
    e.src.watch()
}

/// True if the entry moved to another mapping, which stales its sampled seq.
fn sync(e: &mut Entry<'_>) -> bool {
    let Some(ch) = e.src.channel() else {
        return false;
    };
    let generation = ch.attach_generation();
    if e.bound.as_ref().is_some_and(|b| b.generation == generation) {
        return false;
    }
    let sh = ch.wait_handle();
    detach(e);
    // Held while bound: every wait names this word, and a publisher skips the wake without it.
    let waiter = sh.add_waiter();
    e.bound = Some(Bound {
        sh,
        generation,
        waiter,
    });
    true
}

impl<'a> Executor<'a> {
    pub fn new() -> Result<Executor<'a>> {
        Ok(Executor {
            entries: Vec::new(),
            order: Vec::new(),
            set: Vec::new(),
            ctl: Control::new(),
            parked: !merged_wait()?,
            pass_budget: MAX_DRAIN,
            more: false,
            on_pass: None,
        })
    }

    /// Equal priorities start in registration order and then take turns. One wait holds at most
    /// 127 sources beside the control word.
    pub fn add<S: Source + 'a>(&mut self, src: S, priority: i32) -> Result<()> {
        if self.entries.len() + 1 >= sys::WAITV_MAX {
            return Err(Error::InvalidArgument(format!(
                "an executor waits on at most {} sources; spread them over more executors \
                 (in a PartitionedExecutor, more callback groups)",
                sys::WAITV_MAX - 1
            )));
        }
        let parker = if self.parked {
            Some(Parker::spawn(entry_number(self.entries.len()), &self.ctl)?)
        } else {
            None
        };
        self.entries.push(Entry {
            src: Box::new(src),
            priority,
            bound: None,
            parker,
            seq: 0,
            probed: false,
            blocked: false,
        });
        self.order = (0..self.entries.len()).collect();
        let entries = &self.entries;
        self.order.sort_by_key(|&i| Reverse(entries[i].priority));
        Ok(())
    }

    pub fn size(&self) -> usize {
        self.entries.len()
    }

    /// Bounds a pass, not throughput: the next pass takes the rest.
    pub fn set_pass_budget(&mut self, n: u32) -> Result<()> {
        if n == 0 {
            return Err(Error::InvalidArgument(
                "pass budget must be at least 1".into(),
            ));
        }
        self.pass_budget = n;
        Ok(())
    }

    pub fn pass_budget(&self) -> u32 {
        self.pass_budget
    }

    /// Work a `wait_for_work(0)` would find, read without a syscall: a frame or an attach since the
    /// last pass sampled, or a pass that ran out of budget.
    pub fn ready(&self) -> bool {
        self.more
            || self.a_source_moved()
            || self.entries.iter().any(|e| match &e.bound {
                Some(b) => b.sh.wake_word().load(Ordering::Acquire) != e.seq,
                None => unattached_watch(e).is_some_and(|w| w.epoch() != w.armed()),
            })
    }

    /// The last pass ran out of budget, not of frames: skip the next wait.
    pub fn has_more(&self) -> bool {
        self.more
    }

    /// Runs after every pass of `spin`. An error ends the spin.
    pub fn on_pass(&mut self, f: impl FnMut() -> Result<()> + Send + 'a) {
        self.on_pass = Some(Box::new(f));
    }

    pub fn handle(&self) -> ExecutorHandle {
        ExecutorHandle::new(Arc::clone(&self.ctl))
    }

    /// For a loop built on `wait_for_work` and `dispatch`, which never reads the interrupt flag.
    pub fn clear_interrupt(&self) {
        self.ctl.interrupted.store(false, Ordering::Release);
    }

    pub fn dispatch(&mut self) -> Result<usize> {
        self.dispatch_yielding(|| false)
    }

    /// `yield_now` is asked between callbacks, never before the first: the frame that woke the
    /// wait is always delivered.
    pub fn dispatch_yielding(&mut self, mut yield_now: impl FnMut() -> bool) -> Result<usize> {
        for e in &mut self.entries {
            e.probed = false;
            e.blocked = false;
            if e.src.channel().is_none() {
                detach(e);
                if !e.src.attach()? {
                    e.blocked = true;
                    continue;
                }
            }
            sync(e);
            // Sampled before anything is taken, so a publish racing the arm fires it at once.
            e.seq = e.bound.as_ref().map_or(0, |b| b.sh.wake_seq());
            if let Some(ch) = e.src.channel() {
                ch.serve_late(&self.ctl);
            }
        }

        let mut dispatched = 0usize;
        self.more = false;
        loop {
            if dispatched >= self.pass_budget as usize || (dispatched > 0 && yield_now()) {
                self.more = self.ready_entry().is_some();
                break;
            }
            let Some(at) = self.ready_entry() else { break };
            let pick = self.order[at];
            let e = &mut self.entries[pick];
            e.probed = true;
            let id = Arc::as_ptr(&self.ctl).cast::<c_void>();
            let entry = entry_number(pick);
            event!(flux_trace_callback_start(id, entry, e.src.endpoint()));
            let delivered = e.src.deliver_one()?;
            event!(flux_trace_callback_end(id, entry, i32::from(delivered), ch, seg)
            where (ch, seg) = e.src.channel().map_or((ptr::null(), c"".as_ptr()), |c| {
                (c.trace_handle(), c.trace_segment())
            }));
            dispatched += usize::from(delivered);
            // Read after delivery: a re-attach inside deliver_one replaces the channel.
            match e.src.channel() {
                None => {
                    detach(e);
                    e.blocked = true;
                }
                Some(ch) => {
                    // Ready but nothing taken: take refused (max_borrow), and retrying would spin.
                    e.blocked = !delivered && ch.ready();
                    if e.blocked && !ch.can_borrow() {
                        e.blocked = ch.wake_on_release(&self.ctl);
                    }
                    if sync(e) {
                        e.seq = e.bound.as_ref().map_or(0, |b| b.sh.wake_seq());
                        e.probed = false;
                    }
                }
            }
            if delivered {
                let p = self.entries[pick].priority;
                let end = self.order[at..]
                    .iter()
                    .position(|&i| self.entries[i].priority != p)
                    .map_or(self.order.len(), |k| at + k);
                self.order[at..end].rotate_left(1);
            }
        }

        Ok(dispatched)
    }

    fn ready_entry(&self) -> Option<usize> {
        self.order.iter().position(|&i| {
            let e = &self.entries[i];
            // An unprobed entry goes without asking: only take notices a replaced segment.
            !e.blocked && e.src.channel().is_some_and(|ch| !e.probed || ch.ready())
        })
    }

    fn a_source_moved(&self) -> bool {
        self.entries.iter().any(|e| match e.src.channel() {
            None => e.bound.is_some(),
            Some(ch) => e
                .bound
                .as_ref()
                .is_none_or(|b| b.generation != ch.attach_generation()),
        })
    }

    /// Waits on every bound word at the seq the last pass sampled, so a frame that came after the
    /// sample ends the wait at once. Must never overlap `dispatch`. `timeout_ns` < 0 waits forever.
    pub fn wait_for_work(&mut self, timeout_ns: i64) -> Result<()> {
        let timeout_ns = if self.a_source_moved() { 0 } else { timeout_ns };
        let id = Arc::as_ptr(&self.ctl).cast::<c_void>();
        let woke = if self.parked {
            let sources = self.park()?;
            event!(flux_trace_wait(id, sources, timeout_ns));
            self.ctl.sleep(timeout_ns)
        } else {
            self.set.clear();
            self.set.push(self.ctl.waitv());
            self.set
                .extend(self.entries.iter().filter_map(|e| match &e.bound {
                    Some(b) => Some(sys::Waitv::shared(b.sh.wake_word(), e.seq)),
                    None => unattached_watch(e).map(|w| sys::Waitv::shared(w.word(), w.armed())),
                }));
            event!(flux_trace_wait(
                id,
                entry_number(self.set.len() - 1),
                timeout_ns
            ));
            sys::futex_waitv(&self.set, timeout_ns)
                .map_err(|e| Error::io("futex_waitv", "executor", e))?
        };
        self.ctl.drain();
        event!(flux_trace_wake(id, result, entry) where (result, entry) = self.woken_by(woke));
        Ok(())
    }

    /// The wake event's `result` and `entry`. A parker names its entry through the control block;
    /// a merged wait's word index counts the entries that had a word in the set.
    fn woken_by(&self, woke: sys::Woke) -> (i32, u32) {
        let rang = if self.parked {
            self.ctl.rang.swap(trace::NONE, Ordering::Relaxed)
        } else {
            trace::NONE
        };
        match woke {
            sys::Woke::Word(_) if self.parked && rang == trace::NONE => {
                (trace::WAKE_CONTROL, trace::NONE)
            }
            sys::Woke::Word(_) if self.parked => (trace::WAKE_SOURCE, rang),
            sys::Woke::Word(0) => (trace::WAKE_CONTROL, trace::NONE),
            sys::Woke::Word(i) => {
                let waited = self
                    .entries
                    .iter()
                    .enumerate()
                    .filter(|(_, e)| e.bound.is_some() || unattached_watch(e).is_some());
                let entry = waited
                    .map(|(n, _)| n)
                    .nth(i - 1)
                    .expect("the kernel indexes the waited set");
                (trace::WAKE_SOURCE, entry_number(entry))
            }
            sys::Woke::Timeout => (trace::WAKE_TIMEOUT, trace::NONE),
            sys::Woke::Already => (trace::WAKE_ALREADY, trace::NONE),
        }
    }

    /// Aims each entry's parker at the word the merged wait would name, with the value it would
    /// pass; this thread then sleeps on the control word alone. A parker that rang since the last
    /// `drain` ends the sleep at once. Returns how many parkers have a word.
    fn park(&mut self) -> Result<u32> {
        let mut parked = 0;
        for (i, e) in self.entries.iter_mut().enumerate() {
            let target = park_target(e);
            let p = e
                .parker
                .as_mut()
                .expect("a parked executor's entries have parkers");
            if p.born != sys::fork_gen() {
                // The parent's thread did not come along.
                *p = Parker::spawn(entry_number(i), &self.ctl)?;
            }
            p.reclaim();
            match target {
                Some((owner, _)) if p.parks_on(owner.word()) => {}
                None if p.owner.is_none() => {}
                target => p.retarget(target),
            }
            parked += u32::from(p.owner.is_some());
        }
        Ok(parked)
    }

    /// One pass of `spin`. Never blocks when work is already ready. Every exit consumes the
    /// interrupt flag, so one interrupt ends at most one call.
    fn spin_once(&mut self, timeout_ns: i64) -> Result<usize> {
        let ready = self.dispatch()?;
        if ready > 0 {
            self.ctl.take_interrupt();
            return Ok(ready);
        }
        if timeout_ns < 0 {
            self.wait_for_work(timeout_ns)?;
            let ready = self.dispatch()?;
            self.ctl.take_interrupt();
            return Ok(ready);
        }
        // A wake can come with nothing ready (a poke, a cancel), so wait out the deadline.
        let deadline = sys::monotonic_ns() + timeout_ns.unsigned_abs();
        loop {
            let left = deadline.saturating_sub(sys::monotonic_ns());
            self.wait_for_work(i64::try_from(left).unwrap_or(i64::MAX))?;
            let ready = self.dispatch()?;
            let interrupted = self.ctl.take_interrupt();
            if ready > 0 || interrupted {
                return Ok(ready);
            }
            if sys::monotonic_ns() >= deadline {
                return Ok(0);
            }
        }
    }

    pub fn spin(&mut self, tick_ns: i64) -> Result<()> {
        self.spin_while(&AtomicBool::new(true), tick_ns)
    }

    /// `tick_ns` bounds each wait, which is how often an unattached source retries.
    pub fn spin_while(&mut self, run: &AtomicBool, tick_ns: i64) -> Result<()> {
        self.ctl.set_spinning(true);
        let r = self.spin_loop(run, tick_ns);
        self.ctl.reset();
        self.ctl.set_spinning(false);
        r
    }

    fn spin_loop(&mut self, run: &AtomicBool, tick_ns: i64) -> Result<()> {
        while run.load(Ordering::Relaxed) && !self.ctl.stop_requested() {
            self.spin_once(tick_ns)?;
            if let Some(f) = self.on_pass.as_mut() {
                f()?;
            }
        }
        Ok(())
    }
}

impl Drop for Executor<'_> {
    fn drop(&mut self) {
        for e in &mut self.entries {
            detach(e);
        }
    }
}

#[cfg(test)]
mod trace_events {
    use std::time::Duration;

    use super::{Channel, Executor, Result, Source, merged_wait, thread};
    use crate::discovery::signpost_name;
    use crate::layout::NO_SCHEMA;
    use crate::qos::MemoryPolicy;
    use crate::trace::{self, Word};

    struct Sub(Channel);

    impl Source for Sub {
        fn attach(&mut self) -> Result<bool> {
            Ok(true)
        }
        fn deliver_one(&mut self) -> Result<bool> {
            Ok(self.0.take().is_some())
        }
        fn channel(&self) -> Option<&Channel> {
            Some(&self.0)
        }
    }

    fn wait_and_wake(parked: bool) -> Vec<(&'static str, Vec<u64>)> {
        let key = format!("/t{}_trace_wake_{parked}", std::process::id());
        let name = signpost_name(&key, NO_SCHEMA).unwrap();
        let mem = MemoryPolicy::default();
        let mut publisher = Channel::create(&name, 8, 4, 4, NO_SCHEMA, mem).unwrap();
        let mut ex = Executor::new().unwrap();
        ex.parked = parked;
        ex.add(Sub(Channel::open(&name, NO_SCHEMA, mem).unwrap()), 0)
            .unwrap();
        ex.dispatch().unwrap();
        let late = thread::spawn(move || {
            thread::sleep(Duration::from_millis(50));
            assert_eq!(publisher.publish(b"x"), crate::Published::Ok);
            publisher
        });
        trace::drain();
        ex.wait_for_work(-1).unwrap();
        let events = trace::drain();
        drop(late.join().unwrap());
        events
    }

    #[test]
    fn both_waits_name_the_entry_a_publish_woke() {
        let paths: &[bool] = if merged_wait().unwrap() {
            &[false, true]
        } else {
            &[true]
        };
        for &parked in paths {
            let ev = wait_and_wake(parked);
            let names: Vec<_> = ev.iter().map(|(n, _)| *n).collect();
            assert_eq!(names, ["wait", "wake"], "parked {parked}");
            assert_eq!(ev[0].1[1..], [1, (-1i64).word()], "parked {parked}");
            assert_eq!(
                ev[1].1[1..],
                [trace::WAKE_SOURCE.word(), 0],
                "parked {parked}"
            );
        }
    }
}

#[cfg(test)]
mod refused_wait {
    use super::{merged_wait, thread};
    use crate::error::Error;

    /// Filters only the calling thread, which ends with the test.
    fn refuse_futex_waitv(errno: i32) {
        let op = |code: u32, k: u32, jf: u8| libc::sock_filter {
            code: u16::try_from(code).unwrap(),
            jt: 0,
            jf,
            k,
        };
        let filter = [
            op(libc::BPF_LD | libc::BPF_W | libc::BPF_ABS, 0, 0),
            op(
                libc::BPF_JMP | libc::BPF_JEQ | libc::BPF_K,
                u32::try_from(libc::SYS_futex_waitv).unwrap(),
                1,
            ),
            op(
                libc::BPF_RET | libc::BPF_K,
                libc::SECCOMP_RET_ERRNO | errno.cast_unsigned(),
                0,
            ),
            op(libc::BPF_RET | libc::BPF_K, libc::SECCOMP_RET_ALLOW, 0),
        ];
        let prog = libc::sock_fprog {
            len: 4,
            filter: filter.as_ptr().cast_mut(),
        };
        // SAFETY: plain prctl with integer arguments.
        let no_new_privs = unsafe { libc::prctl(libc::PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) };
        assert_eq!(no_new_privs, 0);
        // SAFETY: prog and the filter it points at outlive the call, which copies them.
        let set = unsafe {
            libc::prctl(
                libc::PR_SET_SECCOMP,
                libc::SECCOMP_MODE_FILTER,
                &raw const prog,
            )
        };
        assert_eq!(set, 0);
    }

    fn probe_refused_with(errno: i32) -> crate::Result<bool> {
        thread::spawn(move || {
            refuse_futex_waitv(errno);
            merged_wait()
        })
        .join()
        .unwrap()
    }

    #[test]
    fn a_missing_futex_waitv_takes_the_parkers() {
        assert!(!probe_refused_with(libc::ENOSYS).unwrap());
    }

    #[test]
    fn a_forbidden_futex_waitv_is_unsupported() {
        assert!(matches!(
            probe_refused_with(libc::EPERM),
            Err(Error::Unsupported(_))
        ));
    }
}

/// The re-aim protocol under Miri: a replaced word is freed only after the parker has read the
/// post that replaced it, so a parker that touches a freed word is a use after free.
#[cfg(test)]
mod race {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};
    use std::{ptr, thread};

    use super::{Aim, Control, Parked, Parker};
    use crate::sys;

    struct Word(AtomicU32);

    impl Parked for Word {
        fn word(&self) -> &AtomicU32 {
            &self.0
        }
    }

    #[test]
    fn a_retired_word_outlives_the_parkers_last_touch() {
        let ctl = Control::new();
        let aim = Arc::new(Aim {
            generation: AtomicU32::new(0),
            word: AtomicPtr::new(ptr::null_mut()),
            seen: AtomicU32::new(0),
            taken: AtomicU32::new(0),
            stop: AtomicBool::new(false),
        });
        let (a, c) = (Arc::clone(&aim), Arc::clone(&ctl));
        let mut p = Parker {
            aim,
            owner: None,
            retired: Vec::new(),
            thread: Some(thread::spawn(move || a.park(0, &c))),
            born: sys::fork_gen(),
        };
        // No wait for the rings: the parker falls behind the posts, as behind an executor that
        // other sources keep waking.
        for _ in 0..if cfg!(miri) { 6 } else { 100 } {
            let w: Arc<dyn Parked> = Arc::new(Word(AtomicU32::new(0)));
            p.reclaim();
            p.retarget(Some((w, 0)));
            let w = p.owner.as_ref().unwrap().word();
            w.fetch_add(1, Ordering::SeqCst);
            sys::futex_wake(w);
            thread::yield_now();
        }
    }
}
