//! Enumeration structs to text, apart from main so tests read it without a subprocess.

use flux2_core::Topic;

/// Keys, labels and signposts come from other processes: a control character in one is shown
/// escaped rather than sent to the terminal.
pub fn printable(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_control() {
                c.escape_default().to_string()
            } else {
                c.to_string()
            }
        })
        .collect()
}

/// A key nobody announced is the name's flattened spelling. Marked, so `.a.b` is not read as a
/// topic literally called that.
pub fn display_name(t: &Topic) -> String {
    if t.key_exact {
        printable(&t.key)
    } else {
        format!("{} (from name)", printable(&t.key))
    }
}

pub fn endpoint_summary(t: &Topic) -> String {
    let pubs = t.endpoints.iter().filter(|e| e.publisher).count();
    let subs = t.endpoints.len() - pubs;
    let mut parts = Vec::new();
    if pubs > 0 {
        parts.push(format!("{pubs} pub"));
    }
    if subs > 0 {
        parts.push(format!("{subs} sub"));
    }
    if parts.is_empty() {
        "-".into()
    } else {
        parts.join(", ")
    }
}

pub fn table(rows: &[Vec<String>], headers: &[&str]) -> Vec<String> {
    let mut widths: Vec<usize> = headers.iter().map(|h| h.len()).collect();
    for row in rows {
        for (w, cell) in widths.iter_mut().zip(row) {
            *w = (*w).max(cell.len());
        }
    }
    let line = |cells: Vec<&str>| {
        cells
            .iter()
            .zip(&widths)
            .map(|(c, w)| format!("{c:<w$}"))
            .collect::<Vec<_>>()
            .join("  ")
            .trim_end()
            .to_owned()
    };
    let mut out = vec![line(headers.to_vec())];
    out.push(
        widths
            .iter()
            .map(|w| "-".repeat(*w))
            .collect::<Vec<_>>()
            .join("  "),
    );
    out.extend(
        rows.iter()
            .map(|r| line(r.iter().map(String::as_str).collect())),
    );
    out
}

/// Whole units as they are, anything else to one decimal: 1.5 MiB is not 2 MiB.
#[allow(clippy::cast_precision_loss)]
pub fn human_bytes(n: u64) -> String {
    let (scale, unit) = [(1 << 30, "GiB"), (1 << 20, "MiB"), (1 << 10, "KiB")]
        .into_iter()
        .find(|&(scale, _)| n >= scale)
        .unwrap_or((1, "B"));
    if n.is_multiple_of(scale) {
        format!("{} {unit}", n / scale)
    } else {
        format!("{:.1} {unit}", n as f64 / scale as f64)
    }
}

/// Unknown values print their number, not a guess.
pub fn storage_name(kind: u32) -> String {
    match kind {
        0 => "host".into(),
        1 => "cuda-ipc".into(),
        k => format!("unknown({k})"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_table_sizes_columns_and_leaves_no_trailing_spaces() {
        let rows = vec![vec!["/long/name".to_owned(), "1".to_owned()]];
        let t = table(&rows, &["CHANNEL", "N"]);
        assert_eq!(t, ["CHANNEL     N", "----------  -", "/long/name  1"]);
    }

    #[test]
    fn bytes_use_binary_units() {
        assert_eq!(human_bytes(64), "64 B");
        assert_eq!(human_bytes(4096), "4 KiB");
        assert_eq!(human_bytes(3 << 20), "3 MiB");
        assert_eq!(human_bytes(5 << 30), "5 GiB");
        assert_eq!(human_bytes(3 << 19), "1.5 MiB");
        assert_eq!(human_bytes(6_220_800), "5.9 MiB");
    }

    #[test]
    fn control_characters_are_shown_escaped() {
        assert_eq!(printable("/cam\u{1b}[31m\n"), "/cam\\u{1b}[31m\\n");
        assert_eq!(printable("/caf\u{e9}"), "/caf\u{e9}");
    }

    #[test]
    fn an_unknown_storage_kind_is_reported_as_itself() {
        assert_eq!(storage_name(0), "host");
        assert_eq!(storage_name(9), "unknown(9)");
    }
}
