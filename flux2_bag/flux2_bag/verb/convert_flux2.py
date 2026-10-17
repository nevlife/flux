from ros2bag.verb import VerbExtension

from flux2_bag.verb import run


class ConvertFlux2Verb(VerbExtension):
    """Copy a bag with its flux2 frames turned into the ROS messages they are."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("bag_path", metavar="BAG", help="Bag to read.")
        parser.add_argument("-o", "--output", required=True, help="Bag to write.")

    def main(self, *, args):
        run("convert", [args.bag_path, "-o", args.output])
