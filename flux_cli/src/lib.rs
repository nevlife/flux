//! `flux`: inspect flux channels on this host. Reads /dev/shm only, with no lock and no borrow,
//! so the processes it reports on cannot notice it.

pub mod format;

use std::fmt::Write as _;

use flux_core::{Topic, flatten_key};

pub const USAGE: &str = "usage: flux topic list
       flux topic info <channel>
       flux topic hz <channel>

hz prints each second the rate of frames the publisher committed, not frames any subscriber
received.";

#[derive(Debug, PartialEq)]
pub enum Command {
    List,
    Info(String),
    Hz(String),
}

/// `argv` without the program name. Options may follow the verb in any order.
pub fn parse(argv: &[String]) -> Result<Command, String> {
    let bad = |why: String| Err(format!("flux: {why}\n{USAGE}"));
    let (Some("topic"), Some(verb)) = (argv.first().map(String::as_str), argv.get(1)) else {
        return bad("expected `topic list`, `topic info` or `topic hz`".into());
    };
    let mut positional = Vec::new();
    for a in &argv[2..] {
        match a.as_str() {
            o if o.starts_with('-') => return bad(format!("unknown option {o}")),
            p => positional.push(p.to_owned()),
        }
    }
    let one = |positional: Vec<String>| match <[String; 1]>::try_from(positional) {
        Ok([c]) => Ok(c),
        Err(_) => Err(format!("flux: topic {verb} takes one channel\n{USAGE}")),
    };
    Ok(match verb.as_str() {
        "list" if positional.is_empty() => Command::List,
        "list" => return bad("topic list takes no channel".into()),
        "info" => Command::Info(one(positional)?),
        "hz" => Command::Hz(one(positional)?),
        v => return bad(format!("unknown verb topic {v}")),
    })
}

/// Shell completion candidates for `words`, the arguments after `flux` up to and including the
/// one being typed. Channel keys come from `topics`; `find` accepts each of them back.
pub fn complete(words: &[String], topics: &[Topic]) -> Vec<String> {
    let Some((typed, before)) = words.split_last() else {
        return Vec::new();
    };
    let before: Vec<&str> = before.iter().map(String::as_str).collect();
    let mut out: Vec<String> = match before.as_slice() {
        [] => vec!["topic".into()],
        ["topic"] => vec!["list".into(), "info".into(), "hz".into()],
        ["topic", "info" | "hz", rest @ ..] => {
            if !typed.starts_with('-') && !rest.iter().any(|a| !a.starts_with('-')) {
                topics.iter().map(|t| t.key.clone()).collect()
            } else {
                Vec::new()
            }
        }
        _ => Vec::new(),
    };
    out.retain(|c| c.starts_with(typed.as_str()));
    out.dedup();
    out
}

/// Sorted by key, then signpost, so output is stable.
pub fn sorted(mut topics: Vec<Topic>) -> Vec<Topic> {
    topics.sort_by(|a, b| (&a.key, &a.signpost).cmp(&(&b.key, &b.signpost)));
    topics
}

/// The one topic `name` means. A channel nobody announced reports its flattened key, so the
/// name's flattened spelling also matches one of those; an announced channel matches by its own
/// key only. Two matches are separate channels and are refused.
pub fn find<'a>(topics: &'a [Topic], name: &str) -> Result<&'a Topic, String> {
    let mut hits: Vec<&Topic> = topics
        .iter()
        .filter(|t| t.key_exact && t.key == name)
        .collect();
    if hits.is_empty() {
        let flat = flatten_key(name);
        hits = topics
            .iter()
            .filter(|t| !t.key_exact && t.key == flat)
            .collect();
    }
    match hits.as_slice() {
        [t] => Ok(t),
        [] if topics.is_empty() => Err(format!(
            "flux: no channel named '{name}'\n      no flux channel is visible on this host"
        )),
        [] => {
            let mut m = format!("flux: no channel named '{name}'\n      visible here:");
            for t in topics {
                let _ = write!(m, "\n        {}", format::display_name(t));
            }
            Err(m)
        }
        many => {
            let mut m = format!("flux: '{name}' names {} channels:", many.len());
            for t in many {
                let _ = write!(
                    m,
                    "\n        {}  fingerprint {:#018x}",
                    format::display_name(t),
                    t.fingerprint
                );
            }
            Err(m + "\n      they are separate channels and do not talk to each other")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flux_core::Endpoint;

    fn argv(s: &str) -> Vec<String> {
        s.split_whitespace().map(str::to_owned).collect()
    }

    fn topic(key: &str, exact: bool, fp: u64) -> Topic {
        Topic {
            signpost: format!("/flux.v1.{}.{fp:016x}", flatten_key(key)),
            key: if exact { key.into() } else { flatten_key(key) },
            fingerprint: fp,
            key_exact: exact,
            endpoints: vec![Endpoint {
                pid: 1,
                label: String::new(),
                publisher: true,
            }],
            busy: 0,
        }
    }

    #[test]
    fn hz_takes_one_channel() {
        assert_eq!(
            parse(&argv("topic hz /cam")).unwrap(),
            Command::Hz("/cam".into())
        );
        assert_eq!(parse(&argv("topic list")).unwrap(), Command::List);
    }

    #[test]
    fn malformed_invocations_are_refused() {
        for bad in [
            "",
            "topic",
            "domain list",
            "topic info",
            "topic info /a /b",
            "topic list /a",
            "topic hz /a --window 1",
            "topic info /a --window 1",
            "topic list --domain 3",
            "topic list --bogus",
            "topic echo /a",
        ] {
            assert!(parse(&argv(bad)).is_err(), "accepted: {bad}");
        }
    }

    fn completions(line: &str, topics: &[Topic]) -> Vec<String> {
        let mut words = argv(line);
        if line.is_empty() || line.ends_with(' ') {
            words.push(String::new());
        }
        complete(&words, topics)
    }

    #[test]
    fn completion_offers_verbs_then_channels() {
        let t = sorted(vec![
            topic("/cam/left", true, 1),
            topic("/cam/left", true, 2),
            topic("/imu", false, 1),
        ]);
        assert_eq!(completions("", &t), ["topic"]);
        assert_eq!(completions("topic ", &t), ["list", "info", "hz"]);
        assert_eq!(completions("topic h", &t), ["hz"]);
        assert_eq!(completions("topic info ", &t), [".imu", "/cam/left"]);
        assert_eq!(completions("topic hz /c", &t), ["/cam/left"]);
        assert!(completions("topic list ", &t).is_empty());
        assert!(completions("topic info /imu ", &t).is_empty());
    }

    #[test]
    fn completion_offers_no_options() {
        let t = [topic("/cam", true, 1)];
        assert!(completions("topic hz /cam --", &t).is_empty());
        assert!(completions("topic info --", &t).is_empty());
    }

    #[test]
    fn topics_are_sorted_by_key() {
        let all = vec![
            topic("/b", true, 1),
            topic("/a", true, 2),
            topic("/a", true, 1),
        ];
        let got: Vec<(String, u64)> = sorted(all)
            .into_iter()
            .map(|t| (t.key, t.fingerprint))
            .collect();
        assert_eq!(got, [("/a".into(), 1), ("/a".into(), 2), ("/b".into(), 1)]);
    }

    #[test]
    fn an_unannounced_channel_matches_by_its_flattened_name() {
        let t = [topic("/cam/left", false, 1)];
        assert_eq!(find(&t, "/cam/left").unwrap().key, ".cam.left");
    }

    #[test]
    fn an_exact_key_wins_over_one_that_only_flattens_alike() {
        let t = [topic("/cam.left", true, 1), topic("/cam/left", true, 1)];
        assert_eq!(find(&t, "/cam/left").unwrap().key, "/cam/left");
    }

    #[test]
    fn an_announced_channel_is_not_found_under_a_name_that_only_flattens_alike() {
        let t = [topic("/cam.left", true, 1)];
        assert!(
            find(&t, "/cam/left")
                .unwrap_err()
                .contains("no channel named")
        );
    }

    #[test]
    fn two_channels_under_one_name_are_refused() {
        let t = [topic("/cam", true, 1), topic("/cam", true, 2)];
        assert!(find(&t, "/cam").unwrap_err().contains("names 2 channels"));
        assert!(find(&t, "/nope").unwrap_err().contains("visible here"));
        assert!(find(&[], "/nope").unwrap_err().contains("no flux channel"));
    }
}
