#!/usr/bin/env python3
"""Weaken one memory ordering at a time and report which weakenings a checker catches.

  rust  [--only REGEX] [FILE.rs ...]   each Ordering site in flux2_core's protocol files, under
                                       the Miri `::race::` tests (scripts/miri.sh)
  genmc [--strict] HARNESS.c [-DFOO ...]
                                       each memory_order_* site, under GenMC (flux's tool)

SeqCst becomes the acquire/release form of its operation, so a catch shows the site needs the
single total order (an SB pair), not just acquire/release; Acquire, Release and AcqRel become
Relaxed; a Rust fence is removed (a C fence becomes relaxed). Each Rust mutant builds in a copy
of the workspace under TMPDIR, never in place. A missed site is redundant under the others or
outside what the tests observe.
"""

import argparse
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
import time

ROOT = pathlib.Path(__file__).resolve().parents[2]
RUST_FILES = ["channel.rs", "segment.rs", "executor.rs", "manifest.rs", "discovery.rs"]
MEMBERS = ["flux2_core", "flux2_ffi", "flux2_cli", "flux2_gen"]
ORDER = re.compile(r"Ordering::(SeqCst|AcqRel|Acquire|Release)\b")
FENCE = re.compile(r"\bfence\(Ordering::\w+\);")
LOADS = ("load", "readers_free", "readers_in_use")


def op_of(line, col):
    calls = re.findall(r"\.?(\w+)\(", line[:col])
    return calls[-1] if calls else ""


def weaker(line, m):
    tok = m.group(1)
    if tok != "SeqCst":
        return "Relaxed"
    op = op_of(line, m.start())
    if op in LOADS:
        return "Acquire"
    if op == "store":
        return "Release"
    if op.startswith("compare_exchange") and line[: m.start()].count("Ordering::") == 1:
        return None
    return "AcqRel"


def rust_sites(root, files, only):
    for f in files:
        lines = (root / "flux2_core/src" / f).read_text().splitlines()
        for no, line in enumerate(lines, 1):
            if line == "#[cfg(test)]":
                break
            if only and not re.search(only, line):
                continue
            if FENCE.search(line):
                yield f, no, "fence", None
                continue
            for m in ORDER.finditer(line):
                to = weaker(line, m)
                if to:
                    yield f, no, m, to


def rust_mutant(tree, f, no, m, to):
    path = tree / "flux2_core/src" / f
    lines = path.read_text().splitlines(keepends=True)
    line = lines[no - 1]
    if m == "fence":
        lines[no - 1] = FENCE.sub("();", line)
    else:
        lines[no - 1] = line[: m.start()] + "Ordering::" + to + line[m.end() :]
    path.write_text("".join(lines))
    return line


def run_miri(tree, seeds, timeout):
    # A fresh target each time: cargo-miri's dep-info misses edits to a copied tree.
    shutil.rmtree(tree / "target", ignore_errors=True)
    env = dict(os.environ, MIRI_SEEDS=seeds, CARGO_TARGET_DIR=str(tree / "target"))
    t0 = time.time()
    try:
        p = subprocess.run(
            [str(tree / "scripts/miri.sh")],
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return "TIMEOUT", int(time.time() - t0)
    out = p.stdout + p.stderr
    if p.returncode == 0:
        return "missed", int(time.time() - t0)
    for key, name in (
        ("error[E", "BUILD"),
        ("Data race", "race"),
        ("deadlocked", "deadlock"),
        ("Undefined Behavior", "UB"),
        ("panicked", "assert"),
    ):
        if key in out:
            return name, int(time.time() - t0)
    return "FAILED", int(time.time() - t0)


def rust(a):
    with tempfile.TemporaryDirectory() as tmp:
        tree = pathlib.Path(tmp)
        for name in ["Cargo.toml", "Cargo.lock", "rust-toolchain.toml", "fuzz", "scripts"]:
            src = ROOT / name
            (shutil.copytree if src.is_dir() else shutil.copy)(src, tree / name)
        for name in MEMBERS:
            shutil.copytree(ROOT / name, tree / name)
        v, t = run_miri(tree, a.seeds, a.timeout)
        print(f"baseline                         {'pass' if v == 'missed' else v:8s} {t:4d}s")
        for f, no, m, to in rust_sites(tree, a.files or RUST_FILES, a.only):
            pristine = (tree / "flux2_core/src" / f).read_text()
            line = rust_mutant(tree, f, no, m, to)
            v, t = run_miri(tree, a.seeds, a.timeout)
            (tree / "flux2_core/src" / f).write_text(pristine)
            what = "fence -> gone" if m == "fence" else f"{m.group(1)} -> {to}"
            print(f"{f}:{no:<5d} {what:22s} {v:8s} {t:4d}s  {line.strip()[:70]}", flush=True)


C_ORDER = re.compile(r"memory_order_(seq_cst|acq_rel|acquire|release)")


def c_weaker(line, m):
    if m.group(1) != "seq_cst":
        return "relaxed"
    op = op_of(line, m.start())
    if op == "atomic_load_explicit":
        return "acquire"
    if op == "atomic_store_explicit":
        return "release"
    if "compare_exchange" in op and line[: m.start()].count("memory_order_") == 1:
        return None
    return "acq_rel"


def genmc(a):
    src = pathlib.Path(a.harness)
    text = src.read_text()
    lines = text.splitlines()

    def run(code):
        with tempfile.NamedTemporaryFile("w", suffix=".c", dir=src.parent, delete=False) as f:
            f.write(code)
        try:
            p = subprocess.run([a.genmc, "--", *a.flags, f.name], capture_output=True, text=True)
            out = p.stdout + p.stderr
            return "VIOLATION" if "Error" in out else ("ok" if p.returncode == 0 else "FAILED")
        finally:
            os.unlink(f.name)

    base = run(text)
    print(f"{src.name} {' '.join(a.flags)}: baseline {base}")
    wrong = base != "ok"
    for m in C_ORDER.finditer(text):
        no = text.count("\n", 0, m.start()) + 1
        line = lines[no - 1]
        to = c_weaker(line, m)
        if not to:
            continue
        v = run(text[: m.start()] + "memory_order_" + to + text[m.end() :])
        want = "ok" if "// redundant" in line else "VIOLATION"
        wrong |= v != want
        print(f"L{no:<4d} {m.group(1)}->{to:8s} {v:9s} {line.strip()[:64]}")
    if a.strict and wrong:
        raise SystemExit(f"{src.name}: a weakening did not give its expected result")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    r = sub.add_parser("rust")
    r.add_argument("--only", help="mutate only lines matching this regex")
    r.add_argument("--seeds", default="0..64", help="-Zmiri-many-seeds range")
    r.add_argument("--timeout", type=int, default=900)
    r.add_argument("files", nargs="*")
    g = sub.add_parser("genmc")
    g.add_argument("--genmc", default="genmc")
    g.add_argument(
        "--strict",
        action="store_true",
        help="fail unless each weakening breaks the litmus (lines marked redundant: not)",
    )
    g.add_argument("harness")
    g.add_argument("flags", nargs="*")
    a = ap.parse_args()
    (rust if a.mode == "rust" else genmc)(a)


if __name__ == "__main__":
    main()
