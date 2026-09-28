from ros2bag.api import check_positive_float
from ros2bag.verb import VerbExtension

from flux_bag.verb import run


class RecordFluxVerb(VerbExtension):
    """Record flux channels and ROS topics into one bag."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            "topics", nargs="*", metavar="TOPIC", help="Channels and topics to record, by name."
        )
        parser.add_argument(
            "-a", "--all", action="store_true", help="Record every flux channel and ROS topic."
        )
        parser.add_argument("-e", "--regex", help="Record the names this expression matches.")
        parser.add_argument("-x", "--exclude-regex", help="Skip the names this expression matches.")
        parser.add_argument(
            "--exclude-topics", nargs="+", default=[], metavar="TOPIC", help="Names to skip."
        )
        parser.add_argument(
            "-o", "--output", help="Bag directory to create (default: a timestamped name)."
        )
        parser.add_argument(
            "--poll",
            type=check_positive_float,
            metavar="SEC",
            help="Seconds between looks for new flux channels (default: 1).",
        )

    def main(self, *, args):
        argv = list(args.topics)
        if args.all:
            argv.append("-a")
        for flag, value in (
            ("-e", args.regex),
            ("-x", args.exclude_regex),
            ("-o", args.output),
            ("--poll", args.poll),
        ):
            if value is not None:
                argv += [flag, str(value)]
        # Last: in the recorder every name after --exclude-topics is excluded.
        if args.exclude_topics:
            argv += ["--exclude-topics", *args.exclude_topics]
        run("record", argv)
