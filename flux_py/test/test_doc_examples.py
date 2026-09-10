"""The user documents and the Python bindings must name the same things, in both directions.

Forward: nothing in examples/doc_api.py runs, so a renamed binding would not raise anywhere.
Walking the file for flux-rooted attribute paths and resolving each one against the imported
package is what gives the Python fences the same guarantee compiling gives the C++ ones.

Reverse: a binding added without a line in the document is invisible rather than broken, so
nothing catches it. Walking the exported surface and requiring each name to appear as code in
the document is the other direction of the same check. scripts/check_api_surface.py runs the
reverse direction for C++, off the headers instead of a live module.
"""

import ast
import inspect
import pathlib
import sys

import flux
import flux.ros
import flux.rt

ROOT = pathlib.Path(__file__).resolve().parents[2]
# One example file per user document, and the union of the three is the documented surface:
# api.md is the .msg path, raw_api.md the path without one, core_api.md flux without ROS.
EXAMPLES = [
    ROOT / "flux_py" / "examples" / "doc_api.py",
    ROOT / "flux_py" / "examples" / "doc_api_raw.py",
    ROOT / "flux_py" / "examples" / "doc_api_core.py",
]
USER_DOCS = [
    ROOT / "docs" / "en" / "api.en.md",
    ROOT / "docs" / "en" / "raw_api.en.md",
    ROOT / "docs" / "en" / "core_api.en.md",
]

sys.path.insert(0, str(ROOT / "scripts"))

from doc_symbols import documented_paths, documented_symbols  # noqa: E402

# Exported names docs/en/api.en.md deliberately leaves out, each with the reason it is not a gap.
# Empty is the intended state: document the name instead of adding to this.
UNDOCUMENTED = frozenset()


def _flux_paths(tree):
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute):
            continue
        parts = []
        cur = node
        while isinstance(cur, ast.Attribute):
            parts.append(cur.attr)
            cur = cur.value
        if isinstance(cur, ast.Name) and cur.id == "flux":
            yield ("flux", *reversed(parts))


def test_example_compiles():
    for example in EXAMPLES:
        compile(example.read_text(), str(example), "exec")


def test_example_names_resolve():
    paths = set()
    for example in EXAMPLES:
        paths |= set(_flux_paths(ast.parse(example.read_text(), str(example))))
    assert paths, "the examples use no flux binding at all"
    for path in sorted(paths):
        obj = flux
        for part in path[1:]:
            dotted = ".".join(path)
            assert hasattr(obj, part), f"{dotted} does not exist"
            obj = getattr(obj, part)


def _own_members(cls):
    """Public members the binding itself contributes.

    An exported exception inherits BaseException's add_note/args/with_traceback. Those are
    Python's surface, not flux's, and a line about them in the document would say nothing --
    so drop whatever a builtin ancestor already provides. Ancestors outside builtins (an enum
    base) are left alone: a name flux declares there is still flux's.
    """
    inherited = set()
    for base in inspect.getmro(cls)[1:]:
        if base.__module__ == "builtins":
            inherited.update(dir(base))
    return [m for m in dir(cls) if not m.startswith("_") and m not in inherited]


def _exported():
    """Every name the modules export, plus each exported class's public members.

    Three entries per name: the dotted path, the last segment, and whether the path is the
    spelling a user writes. It is for a module-level name (`flux.rt.apply` is how the document
    has to spell it) and not for a class member, which the document reaches through a variable
    (`pub.loan()`, never `flux.Publisher.loan`). test_every_exported_name_is_in_the_document
    asks for the stricter form wherever it is the real one.

    flux.rt is walked as well as flux and flux.ros: it is an extension submodule, so its names
    are not in any package __all__ and without this line everything inside it is unchecked.
    """
    modules = (("flux", flux), ("flux.ros", flux.ros), ("flux.rt", flux.rt))
    for mod_name, mod in modules:
        names = getattr(mod, "__all__", None)
        if names is None:
            names = [n for n in dir(mod) if not n.startswith("_")]
        for name in names:
            obj = getattr(mod, name)
            # flux.ros re-exports flux's own names unchanged -- same object, two spellings. The
            # documents write each once, under flux. Asking for flux.ros.QoS as well would be
            # asking for a spelling no example uses.
            if mod_name != "flux" and getattr(flux, name, None) is obj:
                continue
            yield f"{mod_name}.{name}", name, True
            if not inspect.isclass(obj):
                continue
            for member in _own_members(obj):
                yield f"{mod_name}.{name}.{member}", member, False


def test_all_covers_every_public_name_the_binding_defines():
    """`__all__` is what the reverse check walks, so a name missing from it is unchecked.

    `flux.Loan` shipped that way: defined by the extension, absent from `__all__`, and therefore
    invisible to test_every_exported_name_is_in_the_document below. Omitting a name here has to
    be a decision with a reason, not something that happens by forgetting an import line.
    """
    from flux import _flux

    defined = {name for name in dir(_flux) if not name.startswith("_")}
    missing = sorted(defined - set(flux.__all__))
    assert not missing, (
        "defined by the binding but not in flux.__all__, so nothing checks them: "
        + ", ".join(missing)
    )


def test_every_exported_name_is_in_the_document():
    """A module-level name has to be documented as its own path, not as a word that occurs.

    The bare-word form is what a member gets, because the documents reach a member through a
    variable and no stricter form exists to ask for. A module-level name has one, and asking for
    the word alone made whole modules unfalsifiable: flux.rt mirrors flux::rt, so every name in
    it already occurred in the C++ sections and the Python section could have been deleted whole
    without failing this.
    """
    documented, paths = set(), set()
    for doc in USER_DOCS:
        text = doc.read_text()
        documented |= documented_symbols(text)
        paths |= documented_paths(text)
    missing = sorted(
        dotted for dotted, name, as_path in _exported()
        if dotted not in UNDOCUMENTED
        and (dotted not in paths if as_path else name not in documented)
    )
    assert not missing, "exported but absent from the user documents: " + ", ".join(missing)


def test_prose_is_not_a_mention():
    """The reverse check is only worth running if a word in a sentence does not satisfy it."""
    document = "close drops the data.\n\n`sub.take()`\n\n```python\nex.stop()\n```\n"
    assert documented_symbols(document) == {"sub", "take", "ex", "stop"}
