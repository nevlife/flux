import math

from ros2bag.verb import VerbExtension
from ros2bag.verb import play as ros2bag_play

from flux_bag.verb import rosbag2_options, run_with_options


class PlayFluxVerb(VerbExtension):
    """Play a bag, its flux channels onto flux, with the options of ros2 bag play."""

    def add_arguments(self, parser, cli_name):
        ros2bag_play.PlayVerb().add_arguments(parser, cli_name)

    def main(self, *, args):
        built = program_options(args)
        if isinstance(built, str):
            return built
        run_with_options("play", *built)


def program_options(args):
    """The YAML options and ROS arguments the play program runs with, or why ros2 bag play
    would not run."""
    # The flux clock waits rate-scaled times: a rate that is not finite has none.
    if not math.isfinite(args.rate):
        return "argument -r/--rate: Value must be finite"
    built = rosbag2_options.built_by(ros2bag_play, "Player", ros2bag_play.PlayVerb(), args)
    if isinstance(built, str):
        return built
    storage, play, log_level = built
    if len(storage) != 1:
        return "play_flux plays one bag"
    options = {
        "storage": rosbag2_options.storage(storage[0]),
        "play": rosbag2_options.play(play),
    }
    return rosbag2_options.dump(options), ["--ros-args", "--log-level", log_level]
