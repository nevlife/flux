from ros2topic.verb.hz import HzVerb

from flux2_bridge.activate import add_arguments, run_with_bridge


class TopicHzFlux2Verb(HzVerb):
    """Print the average publishing rate of a flux2 channel through the flux2 bridge."""

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        add_arguments(parser)

    def main(self, *, args):
        return run_with_bridge(args, args.topic_name, lambda a: HzVerb.main(self, args=a))
