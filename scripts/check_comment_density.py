#!/usr/bin/env python3
"""Hold the comment budget in scripts/comment_budget.txt.

Two numbers, both ratchets. Whole-tree comment density may not rise, and no file may grow a
comment block longer than the limit. Neither says a comment is wrong; they say the tree may not
accumulate more of them than it has, which is the only thing a checker can decide without
reading them. Why the numbers exist at all is in the budget file.

A file listed as block-exempt records the block it already has. That number may only shrink,
so an exemption decays rather than becoming permission.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUDGET = ROOT / "scripts" / "comment_budget.txt"
PACKAGES = ("flux_core", "flux_cpp", "flux_py")
SUBDIRS = ("src", "include")
SUFFIXES = {".cpp", ".hpp", ".h", ".cu"}
# A line that is only a // comment. A trailing comment after code is not counted: it annotates
# the line it sits on rather than standing in for prose.
COMMENT = re.compile(r"^\s*//")


def sources():
    for pkg in PACKAGES:
        for sub in SUBDIRS:
            d = ROOT / pkg / sub
            if not d.is_dir():
                continue
            for path in sorted(d.rglob("*")):
                if path.suffix in SUFFIXES:
                    yield path


def measure(path):
    """(comment lines, code lines, longest run of consecutive comment lines)."""
    comments = code = longest = run = 0
    for line in path.read_text().splitlines():
        stripped = line.strip()
        if not stripped:
            run = 0  # a blank line ends a block, so two paragraphs are not read as one
            continue
        if COMMENT.match(line):
            comments += 1
            run += 1
            longest = max(longest, run)
        else:
            code += 1
            run = 0
    return comments, code, longest


def budget():
    density = None
    block = None
    exempt = {}
    for lineno, line in enumerate(BUDGET.read_text().splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if parts[0] == "density" and len(parts) == 2:
            density = float(parts[1])
        elif parts[0] == "block" and len(parts) == 2:
            block = int(parts[1])
        elif parts[0] == "block-exempt" and len(parts) == 3:
            exempt[parts[1]] = int(parts[2])
        else:
            raise SystemExit(f"{BUDGET}:{lineno}: cannot read '{line}'")
    if density is None or block is None:
        raise SystemExit(f"{BUDGET}: both 'density' and 'block' must be set")
    return density, block, exempt


def main():
    max_density, max_block, exempt = budget()
    errors = []
    total_comments = total_code = 0
    seen = set()

    for path in sources():
        rel = path.relative_to(ROOT).as_posix()
        comments, code, longest = measure(path)
        total_comments += comments
        total_code += code
        limit = exempt.get(rel, max_block)
        if rel in exempt:
            seen.add(rel)
        if longest > limit:
            where = "its recorded exemption" if rel in exempt else "the limit"
            errors.append(
                f"{rel}: a comment block of {longest} lines exceeds {where} of {limit}; "
                "move the prose to docs/ or shorten it")

    for rel in sorted(set(exempt) - seen):
        errors.append(
            f"{BUDGET}: block-exempt names {rel}, which is not a source file here -- "
            "drop the row rather than leaving an exemption nothing applies to")

    if total_code == 0:
        raise SystemExit("no sources measured -- the layout changed")
    density = total_comments / total_code
    # Rounded to the recorded precision so a one-line edit does not trip on a third decimal.
    if round(density, 3) > max_density:
        errors.append(
            f"comment density {density:.3f} ({total_comments}/{total_code}) is above the budget "
            f"of {max_density:.3f}; this number is a ratchet, so lower it or delete a comment")

    for e in errors:
        print(e, file=sys.stderr)
    if errors:
        return 1
    print(
        f"comment density {density:.3f} <= {max_density:.3f} "
        f"({total_comments}/{total_code}), longest block within budget")
    return 0


if __name__ == "__main__":
    sys.exit(main())
