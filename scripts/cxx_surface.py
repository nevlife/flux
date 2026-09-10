"""Public declarations of a C++ header, by name.

A regex pass, not a compiler: the flux headers are hand-written and formatted by clang-format,
and the checks that consume this only need the set of names a user could type. Anything harder
to parse than these headers is out of scope by construction -- the input list is fixed.
"""

import re

ACCESS = ("public", "private", "protected")
TYPE_HEAD = re.compile(r"\b(class|struct|union)\s+(?:alignas\s*\([^)]*\)\s*)?([A-Za-z_]\w*)\s*$")
ENUM_HEAD = re.compile(r"\benum\s+(?:class|struct)?\s*([A-Za-z_]\w*)")
NAMESPACE_HEAD = re.compile(r"\bnamespace\s+([A-Za-z_][\w:]*)\s*$")
USING_ALIAS = re.compile(r"\busing\s+([A-Za-z_]\w*)\s*=")
IDENT = re.compile(r"[A-Za-z_]\w*")
SKIP_DECL = re.compile(r"\boperator\b|=\s*delete\b|\boverride\b|\bfriend\b|\bstatic_assert\b")


def strip_noise(src):
    """Comments, string and char literals, and preprocessor lines, replaced by spaces."""
    out = []
    i = 0
    at_line_start = True
    while i < len(src):
        c = src[i]
        if c == "\n":
            at_line_start = True
            out.append(c)
            i += 1
            continue
        if at_line_start and c == "#":
            while i < len(src) and src[i] != "\n":
                i += 1
            continue
        if not c.isspace():
            at_line_start = False
        if src.startswith("//", i):
            while i < len(src) and src[i] != "\n":
                i += 1
            continue
        if src.startswith("/*", i):
            end = src.find("*/", i + 2)
            end = len(src) if end < 0 else end + 2
            out.append(" " * (end - i))
            i = end
            continue
        if c in "\"'":
            j = i + 1
            while j < len(src) and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            out.append(" " * (min(j, len(src) - 1) - i + 1))
            i = j + 1
            continue
        out.append(c)
        i += 1
    return "".join(out)


def _match_brace(src, open_index):
    depth = 0
    for i in range(open_index, len(src)):
        if src[i] == "{":
            depth += 1
        elif src[i] == "}":
            depth -= 1
            if depth == 0:
                return i
    return len(src) - 1


def _decl_name(decl, enclosing):
    """The name a declaration introduces, or None when it introduces nothing to document."""
    decl = decl.strip()
    if not decl or SKIP_DECL.search(decl):
        return None
    alias = USING_ALIAS.search(decl)
    if alias:
        return alias.group(1)
    angle = 0
    for i, c in enumerate(decl):
        if c == "<":
            angle += 1
        elif c == ">":
            angle -= 1
        elif c == "(" and angle == 0:
            head = decl[:i]
            if head.rstrip().endswith("~"):
                return None
            names = IDENT.findall(head)
            return names[-1] if names and names[-1] != enclosing else None
    head = decl.split("=", 1)[0].split("[", 1)[0]
    names = IDENT.findall(head)
    return names[-1] if names else None


def _walk(body, kind, enclosing, in_flux, out):
    """Collect (name, public) pairs declared directly in one class, enum or namespace body."""
    public = kind != "class"
    buf = ""
    i = 0
    paren = 0
    while i < len(body):
        c = body[i]
        if c in "()":
            # A default argument can hold both a brace and a semicolon-free comma list, so only
            # the top level of a declaration ends one.
            paren += 1 if c == "(" else -1
            buf += c
            i += 1
            continue
        if paren > 0 and c not in "{}":
            buf += c
            i += 1
            continue
        if paren > 0 and c == "{":
            end = _match_brace(body, i)
            buf += body[i : end + 1]
            i = end + 1
            continue
        if c == ";":
            name = _decl_name(buf, enclosing)
            if name and public and in_flux:
                out.setdefault(name, set()).add(enclosing)
            buf = ""
            i += 1
            continue
        if c == ":" and buf.strip() in ACCESS:
            public = buf.strip() == "public"
            buf = ""
            i += 1
            continue
        if c == "{":
            end = _match_brace(body, i)
            inner = body[i + 1 : end]
            _open(buf, inner, enclosing, public, in_flux, out)
            buf = ""
            i = end + 1
            while i < len(body) and body[i].isspace():
                i += 1
            if i < len(body) and body[i] == ";":
                i += 1
            continue
        buf += c
        i += 1


def _open(head, inner, enclosing, public, in_flux, out):
    """Dispatch on what the brace that just opened belongs to."""
    ns = NAMESPACE_HEAD.search(head)
    if ns:
        parts = ns.group(1).split("::")
        # `detail` is this repo's marker for "reachable but not surface". Walking into it would
        # ask the user documents to name implementation helpers, and the answer would be to
        # document them rather than to stop exporting them.
        inside = False if parts[-1] == "detail" else (in_flux or parts[0] == "flux")
        _walk(inner, "namespace", None, inside, out)
        return
    enum = ENUM_HEAD.search(head)
    if enum:
        if public and in_flux:
            out.setdefault(enum.group(1), set()).add(enclosing)
            for element in inner.split(","):
                names = IDENT.findall(element.split("=", 1)[0])
                if names:
                    out.setdefault(names[0], set()).add(enum.group(1))
        return
    typed = TYPE_HEAD.search(head.split(":", 1)[0])
    if typed:
        if public and in_flux:
            out.setdefault(typed.group(2), set()).add(enclosing)
            _walk(inner, typed.group(1), typed.group(2), in_flux, out)
        return
    name = _decl_name(head, enclosing)
    if name and public and in_flux:
        out.setdefault(name, set()).add(enclosing)


def public_names(source, only_type=None):
    """name -> set of enclosing type names (None for namespace scope).

    With `only_type`, just that type's own name and its public members; without it, every public
    name the header declares inside a flux namespace.
    """
    src = strip_noise(source)
    out = {}
    _walk(src, "namespace", None, False, out)
    if only_type is None:
        return out
    return {n: o for n, o in out.items() if only_type in o or n == only_type}
