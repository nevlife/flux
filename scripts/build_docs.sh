#!/usr/bin/env bash
# Builds the API reference from the sources, outside the tracked tree:
#   OUT/rust/doc/   rustdoc of flux_core and flux_ffi
#   OUT/cpp/html/   Doxygen of the C++ headers and the C header flux.h
#   OUT/python/     pdoc of the flux module
# OUT is the first argument, target/docs by default (git-ignored). The Python part imports flux,
# so source the workspace's install/setup.bash first. DOXYGEN and PYTHON name other programs.
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
OUT=$(realpath -m "${1:-$ROOT/target/docs}")
DOXYGEN=${DOXYGEN:-doxygen}
PYTHON=${PYTHON:-python3}
# rustup's cargo reads rust-toolchain.toml; a distribution's does not.
export PATH="${CARGO_HOME:-$HOME/.cargo}/bin:$PATH"

missing=()
command -v cargo >/dev/null || missing+=("cargo (rustup)")
command -v "$DOXYGEN" >/dev/null || missing+=("doxygen")
"$PYTHON" -c "import pdoc" 2>/dev/null || missing+=("pdoc for $PYTHON")
"$PYTHON" -c "import flux" 2>/dev/null || missing+=("the flux module for $PYTHON (source install/setup.bash)")
if [ ${#missing[@]} -gt 0 ]; then
  printf 'build_docs.sh: missing %s\n' "${missing[@]}" >&2
  exit 1
fi
mkdir -p "$OUT"

echo "rustdoc -> $OUT/rust/doc"
(cd "$ROOT" && cargo doc --quiet --no-deps --locked -p flux_core -p flux_ffi --target-dir "$OUT/rust")

echo "doxygen -> $OUT/cpp/html"
mkdir -p "$OUT/cpp"
# The headers comment with //, which Doxygen does not read; the filter turns a whole-line //
# comment into ///, so the comment above a declaration documents it. Lint markers stay plain.
"$DOXYGEN" - <<EOF
PROJECT_NAME          = flux
OUTPUT_DIRECTORY      = $OUT/cpp
INPUT                 = $ROOT/flux_cpp/include $ROOT/flux_ffi/include
RECURSIVE             = YES
FILE_PATTERNS         = *.hpp *.h
EXCLUDE_PATTERNS      = */detail/*
EXCLUDE_SYMBOLS       = flux::detail
STRIP_FROM_PATH       = $ROOT/flux_cpp/include $ROOT/flux_ffi/include
INPUT_FILTER          = "sed -e '/NOLINT/!s|^\([[:space:]]*\)// |\1/// |'"
EXTRACT_ALL           = YES
ENABLE_PREPROCESSING  = YES
MACRO_EXPANSION       = YES
GENERATE_LATEX        = NO
QUIET                 = YES
WARNINGS              = NO
EOF

echo "pdoc -> $OUT/python"
"$PYTHON" -m pdoc --docformat markdown -o "$OUT/python" flux flux.wire flux.message_filters

echo "open $OUT/rust/doc/flux_core/index.html, $OUT/cpp/html/index.html, $OUT/python/flux.html"
