use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use flux_cli::format::{
    display_name, endpoint_summary, human_bytes, printable, storage_name, table,
};
use flux_cli::{Command, USAGE, complete, find, parse, sorted};
use flux_core::{ROSTER_ENTRIES, Topic, enumerate_topics, read_channel_stats};

fn list(topics: &[Topic]) -> u8 {
    if topics.is_empty() {
        println!("no flux channels on this host");
        return 0;
    }
    let rows: Vec<Vec<String>> = topics
        .iter()
        .map(|t| {
            let live = read_channel_stats(&t.signpost).live;
            vec![
                display_name(t),
                if live { "up" } else { "down" }.into(),
                endpoint_summary(t),
            ]
        })
        .collect();
    for line in table(&rows, &["CHANNEL", "SEGMENT", "ENDPOINTS"]) {
        println!("{line}");
    }
    0
}

fn info(t: &Topic) -> u8 {
    let s = read_channel_stats(&t.signpost);
    println!("channel      {}", display_name(t));
    println!("fingerprint  {:#018x}", t.fingerprint);
    println!("signpost     {}", printable(&t.signpost));
    if s.live {
        println!("segment      up, epoch {}", s.epoch);
        println!(
            "slots        {} x {}",
            s.slot_count,
            human_bytes(u64::from(s.slot_size))
        );
        println!("subscribers  up to {} processes", s.max_subscribers);
        println!("storage      {}", storage_name(s.storage_kind));
        println!("published    {} frames on this segment", s.publish_seq);
        println!("parked       {} subscriber(s) on the wake gate", s.waiters);
    } else {
        // A subscriber or an announcement keeps the signpost: a channel at rest, not a fault.
        println!("segment      down (no publisher has one up)");
    }
    if t.endpoints.is_empty() {
        println!("endpoints    none");
        return 0;
    }
    let mut eps: Vec<_> = t.endpoints.iter().collect();
    eps.sort_by_key(|e| (!e.publisher, e.pid));
    let rows: Vec<Vec<String>> = eps
        .iter()
        .map(|e| {
            vec![
                if e.publisher { "pub" } else { "sub" }.into(),
                e.pid.to_string(),
                if e.label.is_empty() {
                    "-".into()
                } else {
                    printable(&e.label)
                },
            ]
        })
        .collect();
    println!("endpoints");
    for line in table(&rows, &["ROLE", "PID", "LABEL"]) {
        println!("  {line}");
    }
    if eps.len() + t.busy >= ROSTER_ENTRIES {
        println!("  (the list is full at {ROSTER_ENTRIES}; endpoints past it are not shown)");
    }
    if t.busy > 0 {
        println!("  ({} being rewritten, not shown)", t.busy);
    }
    0
}

/// From the segment's commit counter: frames the publisher committed, received or not.
#[allow(clippy::cast_precision_loss)]
fn hz(t: &Topic, name: &str, window: Duration) -> u8 {
    let first = read_channel_stats(&t.signpost);
    if !first.live {
        eprintln!("flux: '{name}' has no segment up; nothing is publishing");
        return 1;
    }
    println!(
        "sampling {} every {}s (Ctrl-C to stop)",
        display_name(t),
        window.as_secs_f64()
    );
    let mut prev = Some((first.publish_seq, first.epoch));
    let mut at = Instant::now();
    loop {
        thread::sleep(window);
        let now = read_channel_stats(&t.signpost);
        let elapsed = at.elapsed().as_secs_f64();
        at = Instant::now();
        if !now.live {
            println!("  segment down");
            prev = None;
            continue;
        }
        match prev {
            // A restarted publisher's segment counts from zero again.
            Some((seq, epoch)) if epoch == now.epoch => {
                let frames = now.publish_seq.saturating_sub(seq);
                println!(
                    "  {:8.2} Hz   ({frames} frames in {elapsed:.2}s)",
                    frames as f64 / elapsed
                );
            }
            _ => println!("  publisher restarted; counter is new"),
        }
        prev = Some((now.publish_seq, now.epoch));
    }
}

fn run(command: &Command) -> Result<u8, String> {
    let topics = sorted(enumerate_topics());
    Ok(match command {
        Command::List => list(&topics),
        Command::Info(name) => info(find(&topics, name)?),
        Command::Hz { channel, window } => hz(find(&topics, channel)?, channel, *window),
    })
}

fn main() -> ExitCode {
    // `flux topic list | head` closes the pipe early: stop quietly instead of failing a print.
    // SAFETY: restoring a signal's default disposition.
    unsafe { libc::signal(libc::SIGPIPE, libc::SIG_DFL) };
    let argv: Vec<String> = std::env::args().skip(1).collect();
    // Called by the bash completion hook; the words may contain -h, so this comes first.
    if argv.first().is_some_and(|a| a == "__complete") {
        for c in complete(&argv[1..], &sorted(enumerate_topics())) {
            println!("{c}");
        }
        return ExitCode::SUCCESS;
    }
    if argv.iter().any(|a| a == "-h" || a == "--help") {
        println!("{USAGE}");
        return ExitCode::SUCCESS;
    }
    let command = match parse(&argv) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    match run(&command) {
        Ok(code) => ExitCode::from(code),
        Err(e) => {
            eprintln!("{e}");
            ExitCode::from(1)
        }
    }
}
