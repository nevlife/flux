#!/usr/bin/env python3
"""The code fences of the user documents are the tested example regions, code for code.

A fence written ```cpp doc:<id> or ```python doc:<id> in README.md or under docs/ must equal the
region between [doc:<id>] and [doc:/<id>] in an example source, comments and blank lines aside.
The sources are tests (test_doc_examples in flux_cpp and flux_py), so a fence that drifts from
them is a fence nothing runs. Every fence needs a region, and every region at least one fence.
"""

import re
import sys
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DOCS = [ROOT / "README.md", *sorted((ROOT / "docs").rglob("*.md"))]
SOURCES = (
    ROOT / "flux_cpp" / "test" / "test_doc_examples.cpp",
    ROOT / "flux_py" / "test" / "test_doc_examples.py",
)
FENCE = re.compile(r"^```(cpp|python) doc:([\w-]+)\n(.*?)^```$", re.M | re.S)
REGION = re.compile(
    r"^[ \t]*(?://|#) \[doc:([\w-]+)\]\n(.*?)^[ \t]*(?://|#) \[doc:/\1\]", re.M | re.S
)


def code(text, lang):
    if lang == "cpp":
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
        text = re.sub(r"//.*", "", text)
    else:
        text = re.sub(r"#.*", "", text)
    lines = [line.rstrip() for line in textwrap.dedent(text).splitlines()]
    return "\n".join(line for line in lines if line.strip())


def main():
    regions = {}
    for src in SOURCES:
        lang = "cpp" if src.suffix == ".cpp" else "python"
        for rid, body in REGION.findall(src.read_text()):
            regions[rid] = (lang, code(body, lang))
    errors = []
    fenced = set()
    for doc in DOCS:
        rel = doc.relative_to(ROOT)
        for lang, rid, body in FENCE.findall(doc.read_text()):
            fenced.add(rid)
            if rid not in regions:
                errors.append(f"{rel}: fence doc:{rid} has no [doc:{rid}] region in the examples")
            elif regions[rid] != (lang, code(body, lang)):
                errors.append(f"{rel}: fence doc:{rid} differs from its example region")
    for rid in sorted(set(regions) - fenced):
        errors.append(f"region [doc:{rid}] has no fence in README.md or docs/")
    for e in errors:
        print(e, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
