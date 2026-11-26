from ros2bag.verb import VerbExtension

from flux_bag.verb import run


class ConvertToFluxVerb(VerbExtension):
    """Copy a bag with the named ROS topics turned into flux channels, as record_flux stores them."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("bag_path", metavar="BAG", help="Bag to read.")
        parser.add_argument("-o", "--output", required=True, help="Bag to write.")
        parser.add_argument(
            "--topics",
            nargs="+",
            required=True,
            help="Topics to turn into flux channels: sensor_msgs PointCloud2, Image, CompressedImage.",
        )

    def main(self, *, args):
        run("convert_to_flux", [args.bag_path, "-o", args.output, "--topics", *args.topics])
