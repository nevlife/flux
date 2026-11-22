"""Topics as message_filters inputs over flux or ROS, as flux_wrapper.Subscription picks."""

import message_filters

from . import Subscription


class Stamped:
    """What a synchronizer queues: the header and the View, which holds its slot over flux."""

    __slots__ = ("header", "view")

    def __init__(self, header, view):
        self.header = header
        self.view = view


class Subscriber(message_filters.SimpleFilter):
    """A topic as a message_filters input over flux or ROS; msg_type needs a root `header`.
    max_borrow is queue_size + 1, as in C++. A negative stamp is counted in `unreadable`.
    """

    def __init__(
        self, node, msg_type, topic, qos, queue_size, *, callback_group=None, device="cpu"
    ):
        if "header" not in msg_type.get_fields_and_field_types():
            raise TypeError(
                "flux_wrapper: a message_filters input needs a std_msgs/Header `header`"
            )
        super().__init__()
        self.forwarded = 0
        self.unreadable = 0
        self.subscription = Subscription(
            node,
            msg_type,
            topic,
            self._forward,
            qos,
            callback_group=callback_group,
            device=device,
            max_borrow=queue_size + 1,
        )
        self._topic = topic

    def getTopic(self):  # noqa: N802 message_filters' name
        return self._topic

    def _forward(self, view):
        header = view.header
        if header.stamp.sec < 0:
            self.unreadable += 1
            return
        self.forwarded += 1
        self.signalMessage(Stamped(header, view))
