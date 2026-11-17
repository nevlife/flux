from argparse import ArgumentTypeError
import math

from ros2bag.verb import VerbExtension
from ros2bag.verb import record as ros2bag_record

from flux_bag.verb import rosbag2_options, run_with_options


def positive_seconds(value):
    try:
        s = float(value)
    except ValueError:
        raise ArgumentTypeError("Expects a floating point number") from None
    if not math.isfinite(s):
        raise ArgumentTypeError("Value must be finite")
    if s <= 0:
        raise ArgumentTypeError("Value must be positive")
    return s


class RecordFluxVerb(VerbExtension):
    """Record flux channels and ROS topics into one bag, with the options of ros2 bag record."""

    def add_arguments(self, parser, cli_name):
        ros2bag_record.add_recorder_arguments(parser)
        parser.add_argument("-x", dest="exclude_regex", default="", help="Same as --exclude-regex.")
        parser.add_argument(
            "--poll",
            type=positive_seconds,
            default=0.1,
            metavar="SEC",
            help="flux: seconds between looks for new flux channels (default: %(default)s).",
        )

    def main(self, *, args):
        built = program_options(args)
        if isinstance(built, str):
            return built
        run_with_options("record", *built)


def program_options(args):
    """The YAML options and ROS arguments the record program runs with, or why ros2 bag record
    would not run."""
    built = rosbag2_options.built_by(ros2bag_record, "Recorder", ros2bag_record.RecordVerb(), args)
    if isinstance(built, str):
        return built
    storage, record, log_level, node_name = built
    options = {
        "storage": rosbag2_options.storage(storage),
        "record": rosbag2_options.record(record),
        "node_name": node_name,
        "poll": args.poll,
    }
    return rosbag2_options.dump(options), ["--ros-args", "--log-level", log_level]
