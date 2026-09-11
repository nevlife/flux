from ros2topic.verb.hz import HzVerb

from flux_bridge.activate import add_arguments, run_with_bridge


class TopicHzFluxVerb(HzVerb):
    """Print the average publishing rate of a flux channel through the flux bridge."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        add_arguments(parser)

    def main(self, *, args):
        return run_with_bridge(args, args.topic_name, lambda a: HzVerb.main(self, args=a))
