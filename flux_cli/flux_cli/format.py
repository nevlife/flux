"""Turning enumeration structs into lines. Kept apart from main so the tests can read the text
without a subprocess, and so no command formats its own columns twice."""


def display_name(topic):
    """The channel name to print.

    A topic with no live participant has no one to ask for the real key, so what comes back is
    the name's spelling with every non-alnum character flattened to '.'. Marking it is the point:
    an unmarked `.a.b` reads as a topic literally called that.
    """
    return topic.key if topic.key_exact else topic.key + " (from name)"


def endpoint_summary(topic):
    """`2 pub, 1 sub` for one topic, or `-` when nobody holds it."""
    pubs = sum(1 for e in topic.endpoints if e.publisher)
    subs = len(topic.endpoints) - pubs
    if not topic.endpoints:
        return "-"
    parts = []
    if pubs:
        parts.append(f"{pubs} pub")
    if subs:
        parts.append(f"{subs} sub")
    return ", ".join(parts)


def table(rows, headers):
    """Left-aligned columns, sized to their contents. Returns a list of lines."""
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(cell))
    out = ["  ".join(h.ljust(widths[i]) for i, h in enumerate(headers)).rstrip()]
    out.append("  ".join("-" * widths[i] for i in range(len(headers))))
    for row in rows:
        out.append("  ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)).rstrip())
    return out


def human_bytes(n):
    value = float(n)
    for unit in ("B", "KiB", "MiB"):
        if value < 1024:
            return f"{value:.0f} {unit}"
        value /= 1024
    return f"{value:.1f} GiB"


def storage_name(kind):
    # Mirrors flux::StorageKind. Unknown values print their number rather than a guess: this
    # command is often the thing you run when something is wrong.
    return {0: "host", 1: "cuda-ipc", 2: "dma-buf"}.get(kind, f"unknown({kind})")
