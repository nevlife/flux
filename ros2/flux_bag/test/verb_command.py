"""The program `ros2 bag record_flux` or `play_flux` becomes, to start as a test's subprocess."""

import argparse
import os
import tempfile

from ament_index_python.packages import get_package_prefix

from flux_bag.verb import play_flux, record_flux

VERBS = {
    "record": (record_flux.RecordFluxVerb, record_flux.program_options),
    "play": (play_flux.PlayFluxVerb, play_flux.program_options),
}

_options = tempfile.TemporaryDirectory()


def command(name, *argv):
    """[program, options file, ROS arguments] for `ros2 bag <name>_flux argv`."""
    verb, options = VERBS[name]
    parser = argparse.ArgumentParser()
    verb().add_arguments(parser, "ros2 bag")
    built = options(parser.parse_args([str(a) for a in argv]))
    assert not isinstance(built, str), built
    text, ros_args = built
    fd, path = tempfile.mkstemp(suffix=".yaml", dir=_options.name)
    with os.fdopen(fd, "w") as f:
        f.write(text)
    return [os.path.join(get_package_prefix("flux_bag"), "lib", "flux_bag", name), path, *ros_args]
