from ros2bag.api import check_positive_float
from ros2bag.verb import VerbExtension

from flux_bag.verb import run


class PlayFluxVerb(VerbExtension):
    """Play a bag, putting its flux channels back onto flux."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("bag_path", metavar="BAG", help="Bag to play.")
        parser.add_argument(
            "-r", "--rate", type=check_positive_float, help="Playback rate (default: 1)."
        )

    def main(self, *, args):
        argv = [args.bag_path]
        if args.rate is not None:
            argv += ["-r", str(args.rate)]
        run("play", argv)
