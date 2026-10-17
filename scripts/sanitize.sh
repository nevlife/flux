#!/usr/bin/env bash
# flux2_core's in-process tests under a sanitizer: nightly -Zsanitizer with std rebuilt the same
# way. design.md, in its section on verification, says what each run covers and why each
# suppression is there.
#   scripts/sanitize.sh address|thread [extra cargo test arguments]
set -eu
cd "$(dirname "$0")/.."
kind=${1:?address or thread}
shift
toolchain=nightly-2026-10-05
host=$(rustc "+$toolchain" -vV | sed -n 's/^host: //p')
# The test harness unwinds; a profile that aborts would build std both ways.
export CARGO_PROFILE_DEV_PANIC=unwind
export RUSTFLAGS="-Zsanitizer=$kind"
export RUSTDOCFLAGS="$RUSTFLAGS"
# allocator_may_return_null: a heap test asks for a channel too large to allocate, on purpose.
export ASAN_OPTIONS="allocator_may_return_null=1:detect_leaks=1:suppressions=$PWD/scripts/sanitize/lsan.supp"
export LSAN_OPTIONS="suppressions=$PWD/scripts/sanitize/lsan.supp"
export TSAN_OPTIONS="allocator_may_return_null=1:halt_on_error=1:second_deadlock_stack=1:suppressions=$PWD/scripts/sanitize/tsan.supp"
# Left out: the GPU tests (the driver is not instrumented and maps where ASan keeps its shadow) and
# a test that times a cycle, which instrumentation slows past its bound.
exec cargo "+$toolchain" test -Zbuild-std --target "$host" --target-dir "target/$kind" \
  -p flux2_core --lib --test heap --test executor "$@" -- \
  --skip gpu:: --skip publish_and_take_do_not_scan_every_slot
