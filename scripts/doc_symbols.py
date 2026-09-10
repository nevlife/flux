"""The symbols docs/en/api.en.md actually spells, for the checks that ask whether a name is documented.

Prose is not a mention. `data`, `close` and `lost` all occur inside ordinary sentences by accident,
so a substring search over the whole document passes for any short name whether or not the
document ever writes it down. Only code carries the claim: an inline span or a fenced block.
"""

import re

FENCE = re.compile(r"^\s*```")
SPAN = re.compile(r"`([^`]+)`")
IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
DOTTED = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*")


def _code_chunks(text):
    """Every stretch the document writes as code: a fenced line, or one inline span's contents."""
    in_fence = False
    for line in text.splitlines():
        if FENCE.match(line):
            in_fence = not in_fence
            continue
        if in_fence:
            yield line
        else:
            yield from SPAN.findall(line)


def documented_symbols(text):
    """Every identifier the document writes as code."""
    symbols = set()
    for chunk in _code_chunks(text):
        symbols.update(IDENT.findall(chunk))
    return symbols


def documented_paths(text):
    """Every dotted path the document writes as code, and every prefix of each.

    documented_symbols answers "is this word written down anywhere", which cannot tell
    `flux.rt.apply` from the C++ `apply` documented in the section above it -- the two share a
    last segment, and this document deliberately spells both languages. A path carries the
    module, so it can. Prefixes are included because writing `flux.rt.Policy.Fifo` documents
    `flux.rt.Policy` as well.
    """
    paths = set()
    for chunk in _code_chunks(text):
        for match in DOTTED.findall(chunk):
            parts = match.split(".")
            for i in range(1, len(parts) + 1):
                paths.add(".".join(parts[:i]))
    return paths
