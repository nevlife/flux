//! Schema fingerprint: 64-bit `BLAKE2b` of the top-level type name and the flat layout tokens.
//!
//! Emitted into both adapters and the segment name. Field and nested type names stay out
//! (nesting moves no byte); Header and Stamp hash as the fields they occupy. A token change
//! changes every fingerprint across C++, Python and versions: bump the version with it.

use std::fmt::Write;

use crate::flatten::{Leaf, LeafKind};

pub const FINGERPRINT_SCHEMA_VERSION: u32 = 5;

const STAMP_TOKENS: &str = "fixed:i32:1;fixed:u32:1";

fn leaf_token(leaf: &Leaf, out: &mut String) {
    match leaf.kind {
        LeafKind::Header => {
            out.push_str(STAMP_TOKENS);
            out.push_str(";string:-:1");
        }
        LeafKind::Stamp => out.push_str(STAMP_TOKENS),
        kind => {
            let dt = leaf.dtype.map_or("-", |d| d.token());
            write!(out, "{}:{dt}:{}", kind.token(), leaf.count).expect("writing to a String");
            if !leaf.record_leaves.is_empty() {
                out.push('{');
                for (i, r) in leaf.record_leaves.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    leaf_token(r, out);
                }
                out.push('}');
            }
        }
    }
}

pub fn canonical(leaves: &[Leaf], type_name: &str) -> String {
    // '#' is in no type name or leaf token.
    let mut out = format!("v{FINGERPRINT_SCHEMA_VERSION}#{type_name}");
    for leaf in leaves {
        out.push(';');
        leaf_token(leaf, &mut out);
    }
    out
}

pub fn fingerprint(leaves: &[Leaf], type_name: &str) -> u64 {
    u64::from_le_bytes(blake2b(canonical(leaves, type_name).as_bytes()))
}

const IV: [u64; 8] = [
    0x6a09_e667_f3bc_c908,
    0xbb67_ae85_84ca_a73b,
    0x3c6e_f372_fe94_f82b,
    0xa54f_f53a_5f1d_36f1,
    0x510e_527f_ade6_82d1,
    0x9b05_688c_2b3e_6c1f,
    0x1f83_d9ab_fb41_bd6b,
    0x5be0_cd19_137e_2179,
];

const SIGMA: [[usize; 16]; 10] = [
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
    [14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3],
    [11, 8, 12, 0, 5, 2, 15, 13, 10, 14, 3, 6, 7, 1, 9, 4],
    [7, 9, 3, 1, 13, 12, 11, 14, 2, 6, 5, 10, 4, 0, 15, 8],
    [9, 0, 5, 7, 2, 4, 10, 15, 14, 1, 11, 12, 6, 8, 3, 13],
    [2, 12, 6, 10, 0, 11, 8, 3, 4, 13, 7, 5, 15, 14, 1, 9],
    [12, 5, 1, 15, 14, 13, 4, 10, 0, 7, 6, 3, 9, 2, 8, 11],
    [13, 11, 7, 14, 12, 1, 3, 9, 5, 0, 15, 4, 8, 6, 2, 10],
    [6, 15, 14, 9, 11, 3, 0, 8, 12, 2, 13, 7, 1, 4, 10, 5],
    [10, 2, 8, 4, 7, 6, 1, 5, 15, 11, 9, 14, 3, 12, 13, 0],
];

fn compress(state: &mut [u64; 8], block: &[u8; 128], counter: u128, last: bool) {
    let mut words = [0u64; 16];
    for (w, b) in words.iter_mut().zip(block.as_chunks::<8>().0) {
        *w = u64::from_le_bytes(*b);
    }
    let m = words;
    let mut v = [0u64; 16];
    v[..8].copy_from_slice(state);
    v[8..].copy_from_slice(&IV);
    #[allow(clippy::cast_possible_truncation)]
    {
        v[12] ^= counter as u64;
        v[13] ^= (counter >> 64) as u64;
    }
    if last {
        v[14] = !v[14];
    }
    let g = |v: &mut [u64; 16], a: usize, b: usize, c: usize, d: usize, x: u64, y: u64| {
        v[a] = v[a].wrapping_add(v[b]).wrapping_add(x);
        v[d] = (v[d] ^ v[a]).rotate_right(32);
        v[c] = v[c].wrapping_add(v[d]);
        v[b] = (v[b] ^ v[c]).rotate_right(24);
        v[a] = v[a].wrapping_add(v[b]).wrapping_add(y);
        v[d] = (v[d] ^ v[a]).rotate_right(16);
        v[c] = v[c].wrapping_add(v[d]);
        v[b] = (v[b] ^ v[c]).rotate_right(63);
    };
    for s in SIGMA.iter().cycle().take(12) {
        g(&mut v, 0, 4, 8, 12, m[s[0]], m[s[1]]);
        g(&mut v, 1, 5, 9, 13, m[s[2]], m[s[3]]);
        g(&mut v, 2, 6, 10, 14, m[s[4]], m[s[5]]);
        g(&mut v, 3, 7, 11, 15, m[s[6]], m[s[7]]);
        g(&mut v, 0, 5, 10, 15, m[s[8]], m[s[9]]);
        g(&mut v, 1, 6, 11, 12, m[s[10]], m[s[11]]);
        g(&mut v, 2, 7, 8, 13, m[s[12]], m[s[13]]);
        g(&mut v, 3, 4, 9, 14, m[s[14]], m[s[15]]);
    }
    for (i, s) in state.iter_mut().enumerate() {
        *s ^= v[i] ^ v[i + 8];
    }
}

/// Unkeyed `BLAKE2b` (RFC 7693) with an `N`-byte digest, as Python's `hashlib.blake2b`.
pub fn blake2b<const N: usize>(data: &[u8]) -> [u8; N] {
    const { assert!(N >= 1 && N <= 64) };
    let mut state = IV;
    state[0] ^= 0x0101_0000 ^ N as u64;
    let mut counter: u128 = 0;
    let mut chunks = data.chunks(128).peekable();
    if chunks.peek().is_none() {
        compress(&mut state, &[0; 128], 0, true);
    }
    while let Some(chunk) = chunks.next() {
        counter += chunk.len() as u128;
        let mut block = [0u8; 128];
        block[..chunk.len()].copy_from_slice(chunk);
        compress(&mut state, &block, counter, chunks.peek().is_none());
    }
    let mut out = [0u8; N];
    for (o, b) in out
        .iter_mut()
        .zip(state.iter().flat_map(|w| w.to_le_bytes()))
    {
        *o = b;
    }
    out
}

#[cfg(test)]
mod tests {
    use std::fmt::Write;

    use super::blake2b;

    fn hex(b: &[u8]) -> String {
        b.iter().fold(String::new(), |mut s, x| {
            write!(s, "{x:02x}").expect("writing to a String");
            s
        })
    }

    #[test]
    fn blake2b_matches_the_rfc_7693_vector() {
        assert_eq!(
            hex(&blake2b::<64>(b"abc")),
            "ba80a53f981c4d0d6a2797b69f12f6e94c212f14685ac4b74b12bb6fdbffa2d1\
             7d87c5392aab792dc252d5de4533cc9518d38aa8dbf1925ab92386edd4009923"
        );
    }

    #[test]
    fn blake2b_short_digests_match_hashlib() {
        // hashlib.blake2b(x, digest_size=8).hexdigest()
        assert_eq!(hex(&blake2b::<8>(b"")), "e4a6a0577479b2b4");
        assert_eq!(hex(&blake2b::<8>(&[b'a'; 128])), "f06643fe9c7e18da");
        assert_eq!(hex(&blake2b::<8>(&[b'a'; 200])), "b40af6d2f2f34472");
    }
}
