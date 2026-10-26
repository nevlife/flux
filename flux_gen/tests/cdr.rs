//! The bridge's frame-to-CDR writer, on hand-built frames of test/msg/Labeled.msg.

mod common;

use common::test_registry;
use flux_gen::cdr::{cdr_size, write_cdr};
use flux_gen::flatten::flatten_message;
use flux_gen::layout::{Layout, build_layout};

fn labeled() -> Layout {
    let reg = test_registry();
    build_layout(&flatten_message(reg.get("Labeled").unwrap(), &reg).unwrap()).unwrap()
}

/// A Labeled frame: `label` = `text`, `values` empty.
fn frame(text: &[u8]) -> Vec<u8> {
    let mut f = vec![0u8; 16];
    f[..4].copy_from_slice(&16u32.to_ne_bytes());
    f[4..8].copy_from_slice(&u32::try_from(text.len()).unwrap().to_ne_bytes());
    f.extend_from_slice(text);
    f
}

#[test]
fn a_string_stops_at_its_first_nul_as_rclpy_publishes_it() {
    let (layout, f) = (labeled(), frame(b"a\0b"));
    let mut out = vec![0u8; cdr_size(&layout, &f).unwrap()];
    assert_eq!(write_cdr(&layout, &f, &mut out), Ok(16));
    assert_eq!(
        out,
        [0, 1, 0, 0, 2, 0, 0, 0, b'a', 0, 0, 0, 0, 0, 0, 0],
        "string 'a', then an empty values"
    );
}

#[test]
fn a_short_buffer_gets_the_size_it_needs_instead_of_a_panic() {
    let (layout, f) = (labeled(), frame(b"hello"));
    let need = cdr_size(&layout, &f).unwrap();
    for cap in [0, 1, 4, need - 1] {
        let mut out = vec![0u8; cap];
        assert_eq!(write_cdr(&layout, &f, &mut out), Ok(need), "cap {cap}");
    }
}
