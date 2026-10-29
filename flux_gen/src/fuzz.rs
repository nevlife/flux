//! Fuzz target for the frame-to-CDR writer, run by cargo-fuzz (`fuzz/`) and over a fixed corpus by
//! `cargo test`. Another process writes the frame the bridge and a publisher's ROS 2 mirror read,
//! so the oracle is: no panic, and a frame that sizes writes exactly that size into any buffer.
#![allow(clippy::missing_panics_doc)]

use std::path::Path;
use std::sync::OnceLock;

use crate::cdr::{cdr_size, write_cdr};
use crate::flatten::flatten_message;
use crate::layout::{Layout, build_layout};
use crate::parse::{Registry, load_dir};

/// The layouts of `test/msg`, by name: strings, sequences, nesting, fixed arrays and tensors.
pub fn layouts() -> &'static [Layout] {
    static LAYOUTS: OnceLock<Vec<Layout>> = OnceLock::new();
    LAYOUTS.get_or_init(|| {
        let mut reg = Registry::new();
        let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("test")
            .join("msg");
        load_dir(&dir, "test", &mut reg, false).expect("flux_gen/test/msg loads");
        let mut v: Vec<Layout> = reg
            .messages()
            .filter_map(|(_, m)| build_layout(&flatten_message(m, &reg).ok()?).ok())
            .collect();
        // A message is listed under its plain and its qualified name.
        v.sort_by_key(|l| l.fingerprint);
        v.dedup_by_key(|l| l.fingerprint);
        v.sort_by(|a, b| a.name.cmp(&b.name));
        v
    })
}

/// The first byte picks the layout, the rest is the frame. True if the frame converted.
pub fn cdr(data: &[u8]) -> bool {
    let Some((&pick, frame)) = data.split_first() else {
        return false;
    };
    let all = layouts();
    let layout = &all[usize::from(pick) % all.len()];
    let Ok(n) = cdr_size(layout, frame) else {
        return false;
    };
    let mut out = vec![0u8; n];
    assert_eq!(write_cdr(layout, frame, &mut out), Ok(n), "{}", layout.name);
    assert_eq!(
        write_cdr(layout, frame, &mut out[..n / 2]),
        Ok(n),
        "{}",
        layout.name
    );
    true
}

#[cfg(test)]
mod corpus {
    use super::{cdr, layouts};

    // Deterministic, so a failure names one seed.
    fn next(s: &mut u64) -> u64 {
        *s ^= *s << 13;
        *s ^= *s >> 7;
        *s ^= *s << 17;
        *s
    }

    /// Per layout: frames of each length filled with one byte or with one (offset, count) pair over
    /// and over, so descriptors point at aligned bytes in the frame and a fixed array can hold its
    /// N, and bit flips on each. Each layout must convert some, or the corpus never reaches the
    /// writer.
    #[test]
    fn cdr_corpus() {
        let mut seed = 0x9E37_79B9_7F4A_7C15;
        for pick in 0..u8::try_from(layouts().len()).unwrap() {
            let mut converted = 0;
            for len in (0..=512).step_by(8) {
                let pairs = (8..=64)
                    .step_by(8)
                    .flat_map(|at| (0..=3).map(move |n| [at, n]));
                let fills = [[0u32, 0], [u32::MAX, u32::MAX]].into_iter().chain(pairs);
                for pair in fills {
                    let bytes: Vec<u8> = pair.iter().flat_map(|w| w.to_ne_bytes()).collect();
                    let mut buf = vec![pick];
                    buf.extend(bytes.iter().cycle().take(len));
                    converted += usize::from(cdr(&buf));
                    for _ in 0..8 {
                        if len == 0 {
                            break;
                        }
                        let at = 1 + usize::try_from(next(&mut seed) % len as u64).unwrap();
                        buf[at] ^= 1 << (next(&mut seed) % 8);
                        converted += usize::from(cdr(&buf));
                    }
                }
            }
            let name = &layouts()[usize::from(pick)].name;
            eprintln!("{name}: {converted}");
            assert!(converted > 0, "{name}: no frame converted");
        }
    }
}
