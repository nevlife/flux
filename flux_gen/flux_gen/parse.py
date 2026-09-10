"""Minimal ROS 2 .msg parser. Enough for fields, arrays, nested types and constants; ignores
default values. Self-contained -- no rosidl dependency.

Malformed input is rejected with a ParseError naming the file and line. That matters more here
than it looks: this parser decides the flat wire layout and the fingerprint both ends are
generated from, so a field silently dropped or an array size silently misread does not fail
loudly -- it produces a wrong layout that both sides then agree on. Every ambiguous shape is an
error, never a guess.
"""

import os
import re

from .dtypes import ROS_PRIMITIVE
from .model import ArrayKind, Constant, Field, Message

# A fixed array is materialized leaf-by-leaf when flattening (T[N] with a struct element expands
# to N copies), so an absurd N is a generation-time memory bomb. The largest fixed array in the
# 385 messages shipped with ROS jazzy is [36]; this cap is far above any real schema and only
# catches typos and hostile input. Unbounded data belongs in T[].
MAX_FIXED_ARRAY = 1 << 16

_TYPE_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*$")
# ROS 2's own field-name rule: lowercase, starts with a letter, single underscores between
# alphanumerics. Stricter than "a C identifier" on purpose -- the generated Python adapter names
# its own slots `_r`/`_at`/`_w`, so a field starting with an underscore would collide with one.
_NAME_RE = re.compile(r"^[a-z](?:_?[a-z0-9]+)*$")
# str.isdigit() is true for unicode digits, so it would take Arabic-Indic '٣' as 3 and let
# superscript '²' through to an int() that raises ValueError past every ParseError handler.
_DECIMAL_RE = re.compile(r"[0-9]+")
# rosidl's constant rule, the uppercase twin of _NAME_RE. Neither admits a doubled or trailing
# underscore, which is what makes every name the adapter defines for itself unreachable from a
# .msg.
_CONST_NAME_RE = re.compile(r"^[A-Z](?:_?[A-Z0-9]+)*$")


class ParseError(Exception):
    """A .msg that cannot be read unambiguously. Carries file and line when known."""

    def __init__(self, message, path=None, lineno=None, line=None):
        self.path = path
        self.lineno = lineno
        self.line = line
        where = ""
        if path is not None or lineno is not None:
            where = f"{path or '<msg>'}:{lineno or '?'}: "
        text = f" in '{line.strip()}'" if line else ""
        super().__init__(f"flux_gen: {where}{message}{text}")


def _strip_comment(raw):
    """Cut an unquoted '#' and everything after it. A '#' inside a quoted default value is
    literal text, so a naive split would truncate the line and turn a valid field into a parse
    error (or, worse, into something that still parses but means something else)."""
    quote = None
    i = 0
    while i < len(raw):
        c = raw[i]
        if quote is not None:
            if c == "\\":
                i += 2  # escaped character cannot close the quote
                continue
            if c == quote:
                quote = None
        elif c in ("'", '"'):
            quote = c
        elif c == "#":
            return raw[:i]
        i += 1
    return raw


def _parse_size(inner, tok, ctx):
    """Parse the N of T[N]. Only a plain non-negative decimal is accepted -- no signs, no
    whitespace, no underscores, no unicode digits -- so nothing turns into a size by accident."""
    if not _DECIMAL_RE.fullmatch(inner):
        raise ParseError(f"bad fixed array size '{inner}' in type '{tok}' (expected T[N], T[] or T[<=N])", **ctx)
    size = int(inner)
    if size == 0:
        raise ParseError(f"zero-length fixed array in type '{tok}' (use T[] for a dynamic array)", **ctx)
    if size > MAX_FIXED_ARRAY:
        raise ParseError(
            f"fixed array size {size} in type '{tok}' exceeds the {MAX_FIXED_ARRAY} limit "
            f"(use a dynamic T[] instead)",
            **ctx,
        )
    return size


def _parse_type(tok, ctx=None):
    """Split a type token into (base, kind, size). Handles T, T[], T[N], T[<=N] and the
    string bound form string<=N (bound dropped -- a bounded string is still a var tail)."""
    ctx = ctx or {}
    base = tok
    kind = ArrayKind.SCALAR
    size = 0
    if tok.endswith("]"):
        i = tok.rfind("[")
        if i < 0:
            raise ParseError(f"unmatched ']' in type '{tok}'", **ctx)
        inner = tok[i + 1 : -1].strip()
        base = tok[:i]
        if not base:
            raise ParseError(f"array with no element type: '{tok}'", **ctx)
        if inner == "":
            kind = ArrayKind.DYNAMIC
        elif inner.startswith("<="):
            kind = ArrayKind.BOUNDED  # rejected later by flatten, but parsed so the reason is exact
            size = _parse_size(inner[2:].strip(), tok, ctx)
        else:
            kind = ArrayKind.FIXED
            size = _parse_size(inner, tok, ctx)
    elif "[" in tok:
        raise ParseError(f"unterminated array in type '{tok}' (missing ']')", **ctx)

    # string<=N / wstring<=N: the bound is a capacity hint with no effect on the flat wire.
    # Split on the bound rather than testing a prefix: "stringgg<=3" starts with "string" and
    # would otherwise be accepted as one, so a typo'd type name became a silent string field.
    if "<=" in base:
        bounded = base.split("<=", 1)[0]
        if bounded not in ("string", "wstring"):
            raise ParseError(f"'<=' bound is only valid on string/wstring, got '{tok}'", **ctx)
        base = bounded

    if not _TYPE_RE.match(base):
        raise ParseError(f"malformed type name '{base}' in '{tok}'", **ctx)
    return base, kind, size


def _constant_value(base, text, ctx):
    """Parse a constant's literal. ROS allows only primitives and string here."""
    text = text.strip()
    if base in ("string", "wstring"):
        # rosidl semantics: a value wrapped in matching quotes is unquoted; anything else is
        # taken verbatim (inner spaces kept, comment already stripped).
        if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
            return text[1:-1]
        return text
    if base == "bool":
        if text in ("0", "1", "True", "true", "False", "false"):
            return text in ("1", "True", "true")
        raise ParseError(f"bool constant must be 0/1/true/false, got '{text}'", **ctx)
    if base not in ROS_PRIMITIVE:
        raise ParseError(f"only primitive and string types can be constants, got '{base}'", **ctx)
    try:
        return float(text) if base.startswith("float") else int(text, 0)
    except ValueError:
        raise ParseError(f"constant value '{text}' is not a {base}", **ctx) from None


def _split_field(rest, ctx):
    """Split the text after the type token into (name, is_constant).

    A constant is `TYPE NAME=value` (the '=' may be spaced out); a field with a default is
    `TYPE name value`. Deciding on "is there an '=' anywhere after the type" gets this wrong for
    a string default containing '=' -- that field would be dropped as a constant, silently
    changing the wire layout. So the name is read first and only what directly follows it counts.
    """
    m = re.match(r"[A-Za-z_][A-Za-z0-9_]*", rest)
    if not m:
        raise ParseError("missing or malformed field name after the type", **ctx)
    name = m.group(0)
    tail = rest[m.end() :].lstrip()
    return name, (tail[1:] if tail.startswith("=") else None)


def parse_msg(text, name, path=None, package=None):
    fields = []
    constants = []
    seen = set()
    seen_constants = set()
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = _strip_comment(raw).strip()
        if not line:
            continue
        ctx = {"path": path, "lineno": lineno, "line": raw}

        parts = line.split(None, 1)
        type_tok = parts[0]
        # Type first, so a mangled type is reported as one instead of surfacing as a confusing
        # "bad field name" once whitespace inside the brackets has split the token.
        base, kind, size = _parse_type(type_tok, ctx)
        if len(parts) < 2:
            raise ParseError(f"type '{type_tok}' with no field name", **ctx)

        field_name, const_text = _split_field(parts[1], ctx)
        if const_text is not None:
            if kind != ArrayKind.SCALAR:
                raise ParseError(f"array constant '{field_name}' is not valid in a .msg", **ctx)
            if not _CONST_NAME_RE.match(field_name):
                raise ParseError(
                    f"malformed constant name '{field_name}' (rosidl requires UPPER_CASE)", **ctx)
            if field_name == name:
                raise ParseError(
                    f"constant name '{field_name}' collides with the namespace the generated "
                    f"adapter defines", **ctx)
            if field_name in seen_constants:
                raise ParseError(f"duplicate constant name '{field_name}'", **ctx)
            seen_constants.add(field_name)
            constants.append(
                Constant(field_name, base, _constant_value(base, const_text, ctx)))
            continue

        if not _NAME_RE.match(field_name):
            raise ParseError(f"malformed field name '{field_name}'", **ctx)
        if field_name in seen:
            raise ParseError(f"duplicate field name '{field_name}'", **ctx)
        seen.add(field_name)
        fields.append(Field(name=field_name, type_name=base, kind=kind, size=size))
    return Message(
        name=name, fields=fields, package=package, path=path, constants=constants)


class Registry(dict):
    """Message registry keyed by qualified 'pkg/Msg'.

    A bare 'Msg' is also keyed, but only while exactly one package defines that name. The second
    definition withdraws the bare key and records the clash instead, because 35 of the 385 .msg
    files shipped with ROS jazzy share a basename and two of them -- action_msgs/GoalStatus and
    actionlib_msgs/GoalStatus -- differ in both layout and tier. Handing back either one for an
    unqualified reference is a wrong fingerprint that both generated ends then agree on.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ambiguous = {}
        self._owners = {}

    def add(self, msg, if_absent=False):
        bare = msg.name
        key = f"{msg.package}/{bare}" if msg.package else bare
        owners = self._owners.setdefault(bare, [])
        if key in owners:
            if if_absent:
                return
            raise ParseError(f"duplicate message '{key}' in the registry")
        owners.append(key)
        if msg.package:
            self[key] = msg
        if len(owners) == 1:
            self[bare] = msg
        else:
            self.ambiguous[bare] = list(owners)
            self.pop(bare, None)


def _infer_package(path):
    """ROS lays messages out as <pkg>/msg/*.msg, in both an install share/ tree and a source
    tree, so the directory above 'msg' names the package."""
    parent, base = os.path.split(os.path.abspath(path))
    return os.path.basename(parent) if base == "msg" else None


def load_dir(path, package=None, into=None, if_absent=False):
    """Parse every .msg in a directory into a Registry.

    package defaults to the enclosing package directory; pass "" to load without one. into merges
    into an existing Registry -- dict.update() would skip the bare-name bookkeeping and restore
    the silent same-basename overwrite. if_absent keeps what is already registered, which is how
    an installed copy of a message yields to the source the caller named.
    """
    if package is None:
        package = _infer_package(path)
    reg = into if into is not None else Registry()
    for entry in sorted(os.listdir(path)):
        if not entry.endswith(".msg"):
            continue
        stem = entry[:-4]
        full = os.path.join(path, entry)
        with open(full, encoding="utf-8") as f:
            reg.add(parse_msg(f.read(), stem, path=full, package=package or None), if_absent)
    return reg


def resolve(registry, base, package=None):
    """Look up a nested type.

    A qualified 'pkg/Msg' resolves exactly and never falls back to the basename. A bare name
    resolves inside the referring package first, then as the registry-wide alias, and raises if
    that alias is ambiguous -- an unresolvable reference has to fail loudly, because the layout it
    would otherwise pick is what the fingerprint is computed from.
    """
    if "/" in base:
        return registry.get(base)
    if package:
        msg = registry.get(f"{package}/{base}")
        if msg is not None:
            return msg
    clash = getattr(registry, "ambiguous", {}).get(base)
    if clash:
        raise ParseError(
            f"ambiguous type name '{base}': defined by {', '.join(clash)} -- qualify it as pkg/Msg"
        )
    return registry.get(base)
