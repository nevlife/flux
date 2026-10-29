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
        parser.add_argument(
            "--max-cache-size",
            type=int,
            metavar="BYTES",
            help="Memory flux frames may wait in for the bag; a frame past it is dropped and "
            "reported (default: 1 GiB).",
        )
        parser.add_argument(
            "-s", "--storage", help="rosbag2 storage plugin, as in ros2 bag record."
        )
        parser.add_argument(
            "--storage-preset-profile",
            metavar="PROFILE",
            help="Storage preset, as in ros2 bag record; zstd_fast compresses an mcap bag.",
        )
        parser.add_argument(
            "--image-codec",
            choices=("raw", "jpeg"),
            help="jpeg stores the pixels of rgb8, bgr8 and mono8 sensor_msgs/Image frames as "
            "JPEG (default: raw).",
        )
        parser.add_argument(
            "--jpeg-quality",
            type=int,
            metavar="Q",
            help="JPEG quality from 1 to 100 (default: 90).",
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
            ("--max-cache-size", args.max_cache_size),
            ("-s", args.storage),
            ("--storage-preset-profile", args.storage_preset_profile),
            ("--image-codec", args.image_codec),
            ("--jpeg-quality", args.jpeg_quality),
        ):
            if value is not None:
                argv += [flag, str(value)]
        # Last: in the recorder every name after --exclude-topics is excluded.
        if args.exclude_topics:
            argv += ["--exclude-topics", *args.exclude_topics]
        run("record", argv)
