#!/usr/bin/env bash
# Regenerates flux2_ffi/include/flux2.h from the Rust sources. With --check, fails if the committed
# header differs from what the sources give, so the header cannot drift from the ABI.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
header="$root/flux2_ffi/include/flux2.h"
command -v cbindgen >/dev/null || { echo "cbindgen not found (cargo install cbindgen --locked)"; exit 1; }
if [ "${1:-}" = "--check" ]; then
  tmp=$(mktemp)
  trap 'rm -f "$tmp"' EXIT
  cbindgen --quiet --config "$root/flux2_ffi/cbindgen.toml" --crate flux2_ffi --output "$tmp" "$root/flux2_ffi"
  diff -u "$header" "$tmp" || { echo "flux2.h is stale: run scripts/gen_c_header.sh"; exit 1; }
else
  cbindgen --quiet --config "$root/flux2_ffi/cbindgen.toml" --crate flux2_ffi --output "$header" "$root/flux2_ffi"
fi
