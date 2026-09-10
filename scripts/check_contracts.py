#!/usr/bin/env python3
"""Check that docs/en/contracts.en.md and docs/ko/contracts.ko.md point at tests that exist, on both sides.

The document is the list of behaviour C++ and Python have to agree on, and each row names the
test that holds each side to it. A renamed or deleted test turns such a row into a claim nobody
checks, which is worse than no row at all -- so resolving every name here is what keeps the list
honest. It does not read what the tests assert; it only refuses to let the list go stale.

Two tables. X-NNN in section 1 is what both bindings honour, and both names must resolve. S-NNN
in section 3 is what they deliberately do not share, so one side may be `-` -- but not both.
Section 3 rows used to carry prose instead of test
names, and that prose drifted from the code with nothing to catch it.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

DOCS = [ROOT / "docs" / "en" / "contracts.en.md", ROOT / "docs" / "ko" / "contracts.ko.md"]
CPP_TESTS = [ROOT / "flux_core" / "test", ROOT / "flux_cpp" / "test"]
PY_TESTS = [ROOT / "flux_py" / "test"]

ROW = re.compile(r"^\|\s*(X-\d{3})\s*\|(.+?)\|\s*`([^`]+)`\s*\|\s*`([^`]+)`\s*\|\s*$")
SPLIT_ROW = re.compile(
    r"^\|\s*(S-\d{3})\s*\|(.+?)\|\s*`([^`]+)`\s*\|\s*`([^`]+)`\s*\|\s*$")
GTEST = re.compile(r"^\s*TEST(?:_F)?\(\s*([A-Za-z_][\w]*)\s*,\s*([A-Za-z_][\w]*)\s*\)")
PYTEST = re.compile(r"^def (test_\w+)\s*\(")


def _gtests(dirs):
    found = set()
    for d in dirs:
        for path in sorted(d.rglob("*.cpp")):
            for line in path.read_text().splitlines():
                m = GTEST.match(line)
                if m:
                    found.add(f"{m.group(1)}.{m.group(2)}")
    return found


def _pytests(dirs):
    found = set()
    for d in dirs:
        for path in sorted(d.rglob("test_*.py")):
            for line in path.read_text().splitlines():
                m = PYTEST.match(line)
                if m:
                    found.add(m.group(1))
    return found



def _check(doc, cpp, py):
    errors = []
    rows = []
    splits = []
    for lineno, line in enumerate(doc.read_text().splitlines(), 1):
        m = ROW.match(line)
        if m:
            rows.append((lineno, m.group(1), m.group(3), m.group(4)))
            continue
        m = SPLIT_ROW.match(line)
        if m:
            splits.append((lineno, m.group(1), m.group(3), m.group(4)))

    if not rows:
        return [f"{doc}: no contract rows parsed -- the table format changed"], rows, splits
    if not splits:
        return [f"{doc}: no S-NNN rows parsed -- section 3's table format changed"], rows, splits

    ids = [r[1] for r in rows] + [s[1] for s in splits]
    for dup in sorted({i for i in ids if ids.count(i) > 1}):
        errors.append(f"{doc}: contract {dup} appears more than once")

    for lineno, cid, cpp_name, py_name in rows:
        if cpp_name not in cpp:
            errors.append(f"{doc}:{lineno}: {cid} names C++ test {cpp_name}, which does not exist")
        if py_name not in py:
            errors.append(f"{doc}:{lineno}: {cid} names Python test {py_name}, which does not exist")

    for lineno, sid, cpp_name, py_name in splits:
        # `-` is the deliberate "this side has nowhere to put a test". Both sides `-` would be a
        # row that claims a divergence nothing observes.
        if cpp_name == "-" and py_name == "-":
            errors.append(f"{doc}:{lineno}: {sid} names no test on either side")
        if cpp_name != "-" and cpp_name not in cpp:
            errors.append(f"{doc}:{lineno}: {sid} names C++ test {cpp_name}, which does not exist")
        if py_name != "-" and py_name not in py:
            errors.append(
                f"{doc}:{lineno}: {sid} names Python test {py_name}, which does not exist")
    return errors, rows, splits


def main():
    cpp, py = _gtests(CPP_TESTS), _pytests(PY_TESTS)
    errors = []
    parsed = {}
    for doc in DOCS:
        doc_errors, rows, splits = _check(doc, cpp, py)
        errors.extend(doc_errors)
        parsed[doc] = ([r[1:] for r in rows], [s[1:] for s in splits])

    # The two languages are the same list. A row edited in one and not the other is a claim
    # only one reader can check.
    first = parsed[DOCS[0]]
    for doc in DOCS[1:]:
        if parsed[doc] != first:
            errors.append(f"{doc}: contract rows differ from {DOCS[0]}")

    for e in errors:
        print(e, file=sys.stderr)
    if errors:
        print(f"{len(errors)} stale contract reference(s)", file=sys.stderr)
        return 1
    rows, splits = first
    print(f"{len(rows)} contracts and {len(splits)} recorded splits, every name resolved")
    return 0


if __name__ == "__main__":
    sys.exit(main())
