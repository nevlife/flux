//! What the multi-process tests share: a frame format that checks itself, and worker processes
//! that are this test binary run again under one test name.
#![allow(dead_code, reason = "each test crate uses part of this")]

use std::io::{BufRead, BufReader, Read, Write};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use flux2_core::{DType, FrameView};

pub const VIOLATION: i32 = 3;
pub const REPLY: Duration = Duration::from_secs(10);
const HEAD: usize = 56;

const DTYPES: [DType; 4] = [DType::U8, DType::U16, DType::F32, DType::U64];

/// A frame of 64 to `slot` bytes, `slot` a multiple of 32. Odd codes are untyped loans,
/// committed by length.
pub fn shape_of(code: u32, slot: u32) -> (DType, Vec<u64>) {
    let nbytes = 64 + u64::from(code / 2 % (slot / 32 - 1)) * 32;
    if code % 2 == 1 {
        return (DType::U8, vec![nbytes]);
    }
    let dt = DTYPES[(code / 32 % 4) as usize];
    let n = nbytes / u64::from(dt.size());
    if (code / 128).is_multiple_of(2) {
        (dt, vec![n])
    } else {
        (dt, vec![2, n / 2])
    }
}

fn fnv(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| {
        (h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3)
    })
}

fn words(seq: u64, nonce: u64, nbytes: u64, dt: u8, shape: &[u64]) -> [u64; 6] {
    [
        seq,
        nonce,
        nbytes,
        u64::from(dt) | (shape.len() as u64) << 8,
        shape.first().copied().unwrap_or(0),
        shape.get(1).copied().unwrap_or(0),
    ]
}

fn seal(f: &[u8]) -> u64 {
    fnv(&f[..48]) ^ fnv(&f[HEAD..])
}

pub fn frame(seq: u64, nonce: u64, dt: DType, shape: &[u64]) -> Vec<u8> {
    let nbytes = shape.iter().product::<u64>() * u64::from(dt.size());
    let mut f = vec![0u8; usize::try_from(nbytes).unwrap()];
    let mut x = seq ^ nonce.rotate_left(17) | 1;
    for b in &mut f[HEAD..] {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        *b = x.to_le_bytes()[0];
    }
    for (i, w) in words(seq, nonce, nbytes, dt as u8, shape)
        .iter()
        .enumerate()
    {
        f[i * 8..i * 8 + 8].copy_from_slice(&w.to_le_bytes());
    }
    let h = seal(&f);
    f[48..HEAD].copy_from_slice(&h.to_le_bytes());
    f
}

/// The frame's nonce, if the view holds whole the frame its ticket and header name.
pub fn check(v: &FrameView) -> Result<u64, String> {
    let (d, m) = (v.data(), v.meta());
    if d.len() < HEAD {
        return Err(format!("ticket {}: {} bytes", v.ticket(), d.len()));
    }
    let w = |i: usize| u64::from_le_bytes(d[i * 8..i * 8 + 8].try_into().unwrap());
    let got: Vec<u64> = (0..6).map(w).collect();
    let want = words(v.ticket(), w(1), m.nbytes, m.dtype, m.shape());
    if got != want || d.len() as u64 != m.nbytes || w(6) != seal(d) {
        return Err(format!(
            "ticket {}: frame says {got:?}, view {want:?}",
            v.ticket()
        ));
    }
    Ok(w(1))
}

pub fn reply(s: &str) {
    let mut o = std::io::stdout().lock();
    writeln!(o, "@@ {s}").unwrap();
    o.flush().unwrap();
}

pub fn violation(what: &str) -> ! {
    eprintln!("VIOLATION {what}");
    std::process::exit(VIOLATION);
}

pub struct Worker {
    pub child: Child,
    stdin: Option<ChildStdin>,
    lines: Receiver<String>,
    pub stopped: bool,
    pub dead: bool,
}

impl Worker {
    /// This binary, running only `test`, with `env`. It answers on stdout lines marked `@@ `.
    pub fn spawn(test: &str, env: &[(&str, &str)]) -> Worker {
        let mut child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", test, "--nocapture", "--test-threads=1"])
            .envs(env.iter().copied())
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let out = child.stdout.take().unwrap();
        let (tx, lines) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for l in BufReader::new(out).lines().map_while(Result::ok) {
                // libtest prints "test <name> ... " ahead of the first reply.
                if let Some((_, r)) = l.split_once("@@ ")
                    && tx.send(r.to_owned()).is_err()
                {
                    break;
                }
            }
        });
        Worker {
            stdin: child.stdin.take(),
            child,
            lines,
            stopped: false,
            dead: false,
        }
    }

    pub fn pid(&self) -> libc::pid_t {
        libc::pid_t::try_from(self.child.id()).unwrap()
    }

    pub fn signal(&mut self, sig: libc::c_int) {
        // SAFETY: plain syscall on our own child.
        unsafe { libc::kill(self.pid(), sig) };
        self.stopped = sig == libc::SIGSTOP;
    }

    pub fn reply(&mut self) -> Result<String, String> {
        self.lines.recv_timeout(REPLY).map_err(|_| self.death())
    }

    pub fn send(&mut self, cmd: &str) -> Result<String, String> {
        let stdin = self.stdin.as_mut().unwrap();
        if writeln!(stdin, "{cmd}")
            .and_then(|()| stdin.flush())
            .is_err()
        {
            return Err(self.death());
        }
        self.reply()
    }

    /// Replies so far, unread.
    pub fn drain(&self) -> Vec<String> {
        self.lines.try_iter().collect()
    }

    /// End of input: the worker's cue to finish.
    pub fn close(&mut self) {
        drop(self.stdin.take());
    }

    /// Why the worker stopped answering.
    pub fn death(&mut self) -> String {
        if self.stopped {
            self.signal(libc::SIGCONT);
        }
        let _ = self.child.kill();
        let status = self.child.wait().unwrap();
        let mut err = String::new();
        if let Some(mut e) = self.child.stderr.take() {
            let _ = e.read_to_string(&mut err);
        }
        let found: Vec<&str> = err.lines().filter(|l| l.contains("VIOLATION")).collect();
        format!("worker {status}: {}", found.join("; "))
    }

    /// Ends the worker the way a process ends normally: it drops what it holds.
    pub fn finish(mut self) -> Result<(), String> {
        if self.stopped {
            self.signal(libc::SIGCONT);
        }
        self.close();
        let until = Instant::now() + REPLY;
        while self.child.try_wait().unwrap().is_none() {
            if Instant::now() > until {
                return Err(format!("worker did not exit; {}", self.death()));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        match self.child.wait().unwrap().code() {
            Some(0) => Ok(()),
            _ => Err(self.death()),
        }
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_none() {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

/// What a finished run left in /dev/shm.
pub fn leftovers(signpost: &str) -> Vec<String> {
    std::fs::read_dir("/dev/shm")
        .unwrap()
        .filter_map(|e| e.ok()?.file_name().into_string().ok())
        .filter(|n| n.starts_with(&signpost[1..]))
        .collect()
}

pub struct Rng(pub u64);

impl Rng {
    pub fn below(&mut self, n: u32) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        u32::try_from(self.0 % u64::from(n)).unwrap()
    }

    pub fn index(&mut self, n: u32) -> usize {
        self.below(n) as usize
    }
}
