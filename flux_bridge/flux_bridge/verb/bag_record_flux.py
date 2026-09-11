from ros2bag.verb.record import RecordVerb

from flux_bridge.activate import Activator, selector


class BagRecordFluxVerb(RecordVerb):
    """Record ROS data to a bag, keeping the flux bridge on for the selected flux channels."""

    def main(self, *, args):
        with Activator(selector(args)):
            return super().main(args=args)
