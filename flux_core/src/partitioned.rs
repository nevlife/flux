//! One thread per partition. A partition is an `Executor`, or an external loop (an rclcpp executor)
//! given as `run` and `stop`. The calling thread runs no callbacks: it waits, and fans out stop
//! and interrupt.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

use crate::error::{Error, Result};
use crate::executor::{Control, Executor, ExecutorHandle};

type Hook<'a> = Box<dyn FnMut() -> Result<()> + Send + 'a>;

enum Body<'a> {
    Flux(Executor<'a>),
    External {
        run: Hook<'a>,
        stop: Box<dyn Fn() + Send + Sync + 'a>,
    },
}

struct Partition<'a> {
    body: Body<'a>,
    init: Option<Hook<'a>>,
}

pub struct PartitionedExecutor<'a> {
    parts: Vec<Partition<'a>>,
    ctl: Arc<Control>,
}

fn record(failure: &Mutex<Option<Error>>, ctl: &Control, r: Result<()>) {
    if let Err(e) = r {
        failure
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get_or_insert(e);
        ctl.poke();
    }
}

fn failed(failure: &Mutex<Option<Error>>) -> bool {
    failure
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .is_some()
}

fn run_init(init: &mut Option<Hook<'_>>) -> Result<()> {
    init.as_mut().map_or(Ok(()), |f| f())
}

impl<'a> PartitionedExecutor<'a> {
    pub fn new() -> Result<PartitionedExecutor<'a>> {
        Ok(PartitionedExecutor {
            parts: Vec::new(),
            ctl: Control::new(),
        })
    }

    /// Returns the partition's index.
    pub fn add(&mut self, ex: Executor<'a>) -> usize {
        self.push(Body::Flux(ex))
    }

    /// `run` returns once `stop` is called. `stop` may come before `run` starts, and `run` must
    /// then return at once.
    pub fn add_external(
        &mut self,
        run: impl FnMut() -> Result<()> + Send + 'a,
        stop: impl Fn() + Send + Sync + 'a,
    ) -> usize {
        self.push(Body::External {
            run: Box::new(run),
            stop: Box::new(stop),
        })
    }

    fn push(&mut self, body: Body<'a>) -> usize {
        self.parts.push(Partition { body, init: None });
        self.parts.len() - 1
    }

    pub fn size(&self) -> usize {
        self.parts.len()
    }

    /// Runs on the partition's own thread before its loop, on every `spin`. An error ends the spin.
    pub fn on_thread_start(
        &mut self,
        part: usize,
        init: impl FnMut() -> Result<()> + Send + 'a,
    ) -> Result<()> {
        let n = self.parts.len();
        let p = self
            .parts
            .get_mut(part)
            .ok_or_else(|| Error::InvalidArgument(format!("no partition {part}; there are {n}")))?;
        p.init = Some(Box::new(init));
        Ok(())
    }

    /// `stop` and `interrupt` reach every partition.
    pub fn handle(&self) -> ExecutorHandle {
        ExecutorHandle::new(Arc::clone(&self.ctl))
    }

    pub fn spin(&mut self, tick_ns: i64) -> Result<()> {
        self.spin_while(&AtomicBool::new(true), tick_ns)
    }

    /// The first error on any partition stops every partition and is returned here.
    pub fn spin_while(&mut self, run: &AtomicBool, tick_ns: i64) -> Result<()> {
        self.ctl.set_spinning(true);
        let children: Vec<ExecutorHandle> = self
            .parts
            .iter()
            .filter_map(|p| match &p.body {
                Body::Flux(ex) => Some(ex.handle()),
                Body::External { .. } => None,
            })
            .collect();
        let child_run = AtomicBool::new(true);
        let failure = Mutex::new(None);
        let (child_run, failure, ctl) = (&child_run, &failure, &*self.ctl);

        std::thread::scope(|s| {
            let mut stops: Vec<&(dyn Fn() + Send + Sync)> = Vec::new();
            for p in &mut self.parts {
                let init = &mut p.init;
                match &mut p.body {
                    Body::Flux(ex) => {
                        s.spawn(move || {
                            let r = run_init(init).and_then(|()| ex.spin_while(child_run, tick_ns));
                            record(failure, ctl, r);
                        });
                    }
                    Body::External { run, stop } => {
                        stops.push(&**stop);
                        s.spawn(move || record(failure, ctl, run_init(init).and_then(|()| run())));
                    }
                }
            }
            while run.load(Ordering::Relaxed) && !ctl.stop_requested() && !failed(failure) {
                ctl.wait(tick_ns);
                if ctl.take_interrupt() {
                    children.iter().for_each(ExecutorHandle::interrupt);
                }
            }
            child_run.store(false, Ordering::Relaxed);
            children.iter().for_each(ExecutorHandle::stop);
            for stop in &stops {
                stop();
            }
        });

        // A child that ended on its own error never consumed the stop sent to it.
        for c in &children {
            c.control().reset();
        }
        self.ctl.reset();
        self.ctl.set_spinning(false);
        failure
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take()
            .map_or(Ok(()), Err)
    }
}
