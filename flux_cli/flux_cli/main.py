"""Argument parsing and the three commands.

Everything here reads: enumeration takes no lock, and the stats path takes neither a lock nor a
borrow, so running this against a live system cannot perturb it or be noticed by it.
"""

import argparse
import signal
import sys
import time

import flux

from .format import display_name, endpoint_summary, human_bytes, storage_name, table


def _topics(args):
    """Every topic, filtered to one domain unless --all-domains. Sorted so output is stable."""
    found = flux.enumerate_topics()
    if not args.all_domains:
        found = [t for t in found if t.domain == args.domain]
    return sorted(found, key=lambda t: (t.domain, t.key))


def _match(topics, name):
    """Resolve a user-typed name to one topic, or exit saying why it could not."""
    exact = [t for t in topics if t.key == name]
    if not exact:
        # A channel with nobody live on it reports the flattened spelling, so the name its own
        # publisher used cannot match on ==. That is the state you look a channel up in: the
        # publisher is gone and you are asking why. flux.flatten_key is the core's rule, not a
        # copy of it.
        flat = flux.flatten_key(name)
        exact = [t for t in topics if flux.flatten_key(t.key) == flat]
    if len(exact) == 1:
        return exact[0]
    if not exact:
        print(f"flux: no channel named '{name}'", file=sys.stderr)
        if topics:
            print("      visible here:", file=sys.stderr)
            for t in topics:
                print(f"        {display_name(t)}", file=sys.stderr)
        else:
            print("      nothing is visible in this domain; try --all-domains", file=sys.stderr)
        raise SystemExit(1)
    # Two channels under one typed name: either one key with two schemas, or two keys that
    # flatten alike. Both are separate channels, so name each by what actually differs.
    print(f"flux: '{name}' names {len(exact)} channels:", file=sys.stderr)
    for t in exact:
        print(f"        {display_name(t)}  fingerprint {t.fingerprint:#018x}", file=sys.stderr)
    print("      they are separate channels and do not talk to each other", file=sys.stderr)
    raise SystemExit(1)


def cmd_list(args):
    topics = _topics(args)
    if not topics:
        where = "anywhere" if args.all_domains else f"in domain '{args.domain}'"
        print(f"no flux channels {where}")
        return 0
    rows = []
    for t in topics:
        stats = flux.read_channel_stats(t.signpost)
        rows.append([
            display_name(t),
            t.domain,
            "up" if stats.live else "down",
            endpoint_summary(t),
        ])
    print("\n".join(table(rows, ["CHANNEL", "DOMAIN", "SEGMENT", "ENDPOINTS"])))
    return 0


def cmd_info(args):
    t = _match(_topics(args), args.channel)
    stats = flux.read_channel_stats(t.signpost)

    print(f"channel      {display_name(t)}")
    print(f"domain        {t.domain}")
    print(f"fingerprint  {t.fingerprint:#018x}")
    print(f"signpost     {t.signpost}")
    if stats.live:
        print(f"segment      up, epoch {stats.epoch}")
        print(f"slots        {stats.slot_count} x {human_bytes(stats.slot_size)}")
        print(f"storage      {storage_name(stats.storage_kind)}")
        print(f"published    {stats.publish_seq} frames on this segment")
        print(f"parked       {stats.waiters} subscriber(s) on the wake gate")
    else:
        # The signpost outlives every participant by design, so this is the normal
        # resting state of a channel nobody is using, not a fault.
        print("segment      down (no publisher has one up; the signpost is persistent)")

    if not t.endpoints:
        print("endpoints    none")
        return 0
    rows = [
        ["pub" if e.publisher else "sub", str(e.pid), e.label or "-"]
        for e in sorted(t.endpoints, key=lambda e: (not e.publisher, e.pid))
    ]
    print("endpoints")
    for line in table(rows, ["ROLE", "PID", "LABEL"]):
        print("  " + line)
    return 0


def cmd_hz(args):
    t = _match(_topics(args), args.channel)
    first = flux.read_channel_stats(t.signpost)
    if not first.live:
        print(f"flux: '{args.channel}' has no segment up; nothing is publishing", file=sys.stderr)
        return 1

    # Flushed line by line: this command streams, and a block-buffered pipe would make it look
    # like it had hung.
    print(f"sampling {display_name(t)} every {args.window:g}s (Ctrl-C to stop)", flush=True)
    prev_seq, prev_epoch = first.publish_seq, first.epoch
    prev_at = time.monotonic()
    try:
        while True:
            time.sleep(args.window)
            now = flux.read_channel_stats(t.signpost)
            elapsed = time.monotonic() - prev_at
            prev_at = time.monotonic()
            if not now.live:
                print("  segment down", flush=True)
                prev_seq = None
                continue
            if prev_seq is None or now.epoch != prev_epoch:
                # A restarted publisher group is a new segment with its own counter, so the
                # difference across that boundary is not a frame count.
                print("  publisher restarted; counter is new", flush=True)
                prev_seq, prev_epoch = now.publish_seq, now.epoch
                continue
            frames = now.publish_seq - prev_seq
            prev_seq = now.publish_seq
            print(f"  {frames / elapsed:8.2f} Hz   ({frames} frames in {elapsed:.2f}s)", flush=True)
    except KeyboardInterrupt:
        print()
        return 0


def cmd_domain_list(args):
    """Every domain that has a name in /dev/shm, and which one this process resolves to.

    Two processes in different domains never meet and neither reports anything -- the subscriber
    sees an idle stream, which is what a publisher that has not started yet also looks like. This
    is the command that separates them: the peer shows up on another row.
    """
    counts = {}
    for t in flux.enumerate_topics():
        row = counts.setdefault(t.domain, [0, 0, 0])  # channels, segments up, endpoints
        row[0] += 1
        row[1] += 1 if flux.read_channel_stats(t.signpost).live else 0
        row[2] += len(t.endpoints)
    # The resolved domain is always a row, even with nothing in it. Dropping it would hide the one
    # value the caller came to check.
    counts.setdefault(args.domain, [0, 0, 0])

    rows = [
        [domain + (" (here)" if domain == args.domain else ""), str(c[0]), str(c[1]), str(c[2])]
        for domain, c in sorted(counts.items())
    ]
    print("\n".join(table(rows, ["DOMAIN", "CHANNELS", "UP", "ENDPOINTS"])))
    if len(counts) > 1:
        print()
        print("more than one domain here: processes in different domains never meet.")
        print("domain comes from FLUX_DOMAIN, else ROS_DOMAIN_ID, else '0'.")
    return 0


def build_parser():
    # --domain and --all-domains hang off every verb, not off the root, so `flux topic list
    # --all-domains` works. Put on the root, argparse would only accept them before the
    # subcommand, which is not where anyone types them.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--domain", default=None,
        help="Domain to look in (default: this process's, from FLUX_DOMAIN or ROS_DOMAIN_ID).")
    common.add_argument(
        "--all-domains", action="store_true", help="Do not filter by domain.")

    parser = argparse.ArgumentParser(
        prog="flux",
        description="Inspect flux channels on this host. Reads /dev/shm; daemonless, needs no "
                    "ROS, and cannot be seen by the processes it reports on.")
    groups = parser.add_subparsers(dest="group", required=True)
    topic = groups.add_parser("topic", help="Commands about channels.")
    verbs = topic.add_subparsers(dest="verb", required=True)

    verbs.add_parser(
        "list", parents=[common], help="List channels.").set_defaults(func=cmd_list)

    info = verbs.add_parser("info", parents=[common], help="Show one channel in detail.")
    info.add_argument("channel")
    info.set_defaults(func=cmd_info)

    hz = verbs.add_parser(
        "hz", parents=[common],
        help="Publish rate, sampled from the segment's frame counter.")
    hz.add_argument("channel")
    hz.add_argument(
        "--window", type=float, default=1.0, metavar="SEC",
        help="Seconds between samples (default: 1.0).")
    hz.set_defaults(func=cmd_hz)

    # No --domain here on purpose: this verb reports which domain the environment puts this process
    # in, and an override would answer a question the caller did not ask.
    domain = groups.add_parser("domain", help="Commands about domains.")
    domain.add_subparsers(dest="verb", required=True).add_parser(
        "list", help="List domains present on this host and mark this process's."
    ).set_defaults(func=cmd_domain_list)
    return parser


def main(argv=None):
    # `flux topic list | head` closes the pipe early. Without this Python raises BrokenPipeError
    # out of print() and prints a traceback where a reporting tool should just stop.
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    args = build_parser().parse_args(argv)
    if getattr(args, "domain", None) is None:
        # Resolved the same way a node resolves it, so `flux topic list` and the nodes on this
        # host agree about which domain "here" means without the user restating it.
        args.domain = flux.process_domain()
    return args.func(args)
