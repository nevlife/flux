#!/usr/bin/env python3
"""Check that the tagged code fences in docs/en/api.en.md are the compiled example sources.

A fence tagged ```<lang> doc:<id> must match the region between [doc:<id>] and [doc:/<id>] in
the matching example file. The example files are built (C++) and imported (Python) by the test
suite, so a renamed API breaks the build; this check is what keeps the document from drifting
away from the code that does compile.

Comments are ignored on both sides: a fence may translate a comment, and the Korean documents
(docs/ko/*.ko.md) carry the same fences, so only the code has to agree. Include and import lines in a
fence are checked for presence in the example source rather than position.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# One entry per user document. A source file belongs to exactly one document: the check requires
# every anchored region to have a fence in its paired document, so a file cannot be split across
# two. That is why the raw and core paths have example files of their own rather than sharing
# doc_api.cpp / doc_api.py.
PAIRS = [
    (
        ROOT / "docs" / "en" / "api.en.md",
        [
            ROOT / "flux_cpp" / "examples" / "doc_api.cpp",
            ROOT / "flux_cpp" / "examples" / "doc_api_adapter.cpp",
            ROOT / "flux_py" / "examples" / "doc_api.py",
            ROOT / "flux_py" / "examples" / "doc_api_adapter.py",
        ],
    ),
    (
        ROOT / "docs" / "en" / "raw_api.en.md",
        [
            ROOT / "flux_cpp" / "examples" / "doc_api_raw.cpp",
            ROOT / "flux_py" / "examples" / "doc_api_raw.py",
        ],
    ),
    (
        ROOT / "docs" / "en" / "core_api.en.md",
        [
            ROOT / "flux_core" / "examples" / "doc_api_core.cpp",
            ROOT / "flux_py" / "examples" / "doc_api_core.py",
        ],
    ),
]

ANCHOR = re.compile(r"^\s*(?://|#)\s*\[doc:(/?)([a-z0-9_]+)\]\s*$")
FENCE = re.compile(r"^```([a-z0-9+]+)\s+doc:([a-z0-9_]+)\s*$")
PREAMBLE = re.compile(r"^(#include\s|import\s|from\s)")
BLOCK_COMMENT = re.compile(r"/\*.*?\*/")


def strip_comments(line):
    """Drop a trailing // or # comment, honouring quotes."""
    line = BLOCK_COMMENT.sub("", line)
    quote = None
    i = 0
    while i < len(line):
        c = line[i]
        if quote:
            if c == "\\":
                i += 2
                continue
            if c == quote:
                quote = None
        elif c in "\"'":
            quote = c
        elif c == "#":
            return line[:i]
        elif c == "/" and line[i + 1 : i + 2] == "/":
            return line[:i]
        i += 1
    return line


def normalize(lines):
    """Comment-free, blank-free, dedented code lines."""
    code = [strip_comments(ln).rstrip() for ln in lines]
    code = [ln for ln in code if ln.strip()]
    if not code:
        return []
    indent = min(len(ln) - len(ln.lstrip()) for ln in code)
    return [ln[indent:] for ln in code]


def read_anchors(path, errors):
    """id -> normalized region, from one example source."""
    regions = {}
    open_id = None
    buf = []
    for lineno, line in enumerate(path.read_text().splitlines(), 1):
        m = ANCHOR.match(line)
        if not m:
            if open_id:
                buf.append(line)
            continue
        closing, name = m.group(1), m.group(2)
        if not closing:
            if open_id:
                errors.append(f"{path}:{lineno}: [doc:{name}] opens inside [doc:{open_id}]")
            open_id, buf = name, []
        elif name != open_id:
            errors.append(f"{path}:{lineno}: [doc:/{name}] closes nothing")
        else:
            if name in regions:
                errors.append(f"{path}:{lineno}: [doc:{name}] defined twice")
            regions[name] = normalize(buf)
            open_id = None
    if open_id:
        errors.append(f"{path}: [doc:{open_id}] is never closed")
    return regions


def read_fences(path, errors):
    """id -> (line number, fence body lines), from one document."""
    fences = {}
    lines = path.read_text().splitlines()
    i = 0
    while i < len(lines):
        m = FENCE.match(lines[i])
        if not m:
            i += 1
            continue
        name, start = m.group(2), i + 1
        body = []
        i += 1
        while i < len(lines) and not lines[i].startswith("```"):
            body.append(lines[i])
            i += 1
        if i == len(lines):
            errors.append(f"{path}:{start}: fence doc:{name} is never closed")
            break
        if name in fences:
            errors.append(f"{path}:{start}: fence doc:{name} appears twice")
        fences[name] = (start, body)
        i += 1
    return fences


def check(doc, sources):
    errors = []
    regions = {}
    source_of = {}
    source_lines = {}
    for src in sources:
        if not src.exists():
            errors.append(f"{src}: missing")
            continue
        source_lines[src] = normalize(src.read_text().splitlines())
        for name, region in read_anchors(src, errors).items():
            if name in regions:
                errors.append(f"{src}: [doc:{name}] also defined in {source_of[name]}")
            regions[name] = region
            source_of[name] = src

    fences = read_fences(doc, errors)
    for name in sorted(set(regions) - set(fences)):
        errors.append(f"{source_of[name]}: [doc:{name}] has no fence in {doc.name}")

    for name, (lineno, body) in sorted(fences.items()):
        if name not in regions:
            errors.append(f"{doc}:{lineno}: fence doc:{name} has no [doc:{name}] in any example")
            continue
        src = source_of[name]
        # Includes and imports are checked for presence, not position, on both sides: an example
        # groups them at the top of its file while a fence shows them beside the code they serve.
        want = [ln for ln in regions[name] if not PREAMBLE.match(ln)]
        got = [ln for ln in normalize(body) if not PREAMBLE.match(ln)]
        for ln in normalize(body):
            if PREAMBLE.match(ln) and ln not in source_lines[src]:
                errors.append(f"{doc}:{lineno}: fence doc:{name} has `{ln}`, missing from {src}")
        if got != want:
            errors.append(
                f"{doc}:{lineno}: fence doc:{name} does not match {src}\n"
                + "\n".join(f"    doc  | {ln}" for ln in got)
                + "\n"
                + "\n".join(f"    code | {ln}" for ln in want)
            )
    return errors


def main():
    errors = []
    for doc, sources in PAIRS:
        errors.extend(check(doc, sources))
        errors.extend(check(doc.parent.parent / "ko" / doc.name.replace(".en.md", ".ko.md"), sources))
    for e in errors:
        print(e, file=sys.stderr)
    if errors:
        print(f"{len(errors)} doc/example mismatch(es)", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
