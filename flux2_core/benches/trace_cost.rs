//! What the trace calls cost while nothing is attached: run once as built and once built with
//! `RUSTFLAGS="--cfg flux2_trace_off"`, which removes the call sites, and divide the difference by
//! the events per iteration. Pin it to one CPU (`taskset -c N`). Prints the median of 9 runs.

use std::hint::black_box;
use std::time::Instant;

use flux2_core::{Channel, Executor, Result, Source};

const ITERS: u32 = 1_000_000;
const RUNS: usize = 9;
const FRAME: [u8; 64] = [7; 64];

/// Publishes the frame it then delivers, so every dispatch delivers one.
struct Fed(Channel);

impl Source for Fed {
    fn attach(&mut self) -> Result<bool> {
        Ok(true)
    }
    fn deliver_one(&mut self) -> Result<bool> {
        let _ = black_box(self.0.publish(&FRAME));
        Ok(black_box(self.0.take()).is_some())
    }
    fn channel(&self) -> Option<&Channel> {
        Some(&self.0)
    }
}

fn median_ns(iters: u32, mut f: impl FnMut()) -> f64 {
    let mut runs: Vec<f64> = (0..RUNS)
        .map(|_| {
            let t = Instant::now();
            for _ in 0..iters {
                f();
            }
            t.elapsed().as_secs_f64() * 1e9 / f64::from(iters)
        })
        .collect();
    runs.sort_by(f64::total_cmp);
    runs[RUNS / 2]
}

fn main() {
    let mut ch = Channel::new(64, 4).unwrap();
    let ns = median_ns(ITERS, || {
        let _ = black_box(ch.publish(&FRAME));
        drop(black_box(ch.take()));
    });
    println!("publish+take+release\t{ns:.2}\tns/iter\t4 events");

    let mut ex = Executor::new().unwrap();
    ex.add(Fed(Channel::new(64, 4).unwrap()), 0).unwrap();
    let ns = median_ns(ITERS, || {
        black_box(ex.dispatch().unwrap());
    });
    println!("dispatch\t{ns:.2}\tns/iter\t6 events");

    let ns = median_ns(ITERS / 10, || ex.wait_for_work(0).unwrap());
    println!("wait_for_work(0)\t{ns:.2}\tns/iter\t2 events");
}
