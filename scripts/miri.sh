#!/usr/bin/env bash
# The ordering tests (`::race::`) under Miri, which runs the real Rust orderings: a data race, a
# use after free or a deadlock fails them. The toolchain is the nightly the fuzz crate pins.
# MIRI_SEEDS sets the seed range (default 0..64).
set -eu
cd "$(dirname "$0")/.."
toolchain=$(sed -n 's/^channel = "\(.*\)"/\1/p' fuzz/rust-toolchain.toml)
export PATH="${CARGO_HOME:-$HOME/.cargo}/bin:$PATH"
if ! cargo "+$toolchain" miri --version >/dev/null 2>&1; then
  echo "miri missing: rustup component add --toolchain $toolchain miri rust-src" >&2
  exit 1
fi
MIRIFLAGS="${MIRIFLAGS:-} -Zmiri-many-seeds=${MIRI_SEEDS:-0..64}" \
  exec cargo "+$toolchain" miri test -p flux_core --lib ::race:: "$@"
