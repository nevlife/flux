"""flux2 topics as message_filters inputs."""

from builtin_interfaces.msg import Time
import message_filters

from . import QoS, Subscription


class _Header:
    __slots__ = ("stamp", "frame_id")


class StampedFrame:
    """What a synchronizer queues: the frame view, still holding its slot, and its header."""

    def __init__(self, frame, adapter, sec, nanosec, frame_id):
        self.frame = frame
        self._adapter = adapter
        self.header = _Header()
        self.header.stamp = Time(sec=sec, nanosec=nanosec)
        self.header.frame_id = frame_id

    def view(self):
        return self._adapter.View(self.frame)


class Subscriber(message_filters.SimpleFilter):
    """A flux2 topic as a message_filters input. `adapter` is a flux2_gen adapter with a header.

    `queue_size` is the synchronizer's or Cache's. Queued frames hold slots and a filter adds
    before it drops, so max_borrow is raised to queue_size + 1, and a publisher with no more
    slots than the queue is refused with flux2.Error.
    """

    def __init__(self, node, adapter, topic, queue_size, qos=None):
        if not all(hasattr(adapter.View, f) for f in ("header__stamp", "header__frame_id")):
            raise TypeError("flux2: a message_filters input needs a std_msgs/Header `header`")
        super().__init__()
        self._adapter = adapter
        self.forwarded = 0
        self.unreadable = 0
        qos = qos if qos is not None else QoS()
        if qos.max_borrow <= queue_size:
            qos = QoS(depth=qos.depth, replay=qos.replay, max_borrow=queue_size + 1)
        self.subscription = Subscription(
            node, topic, callback=self._forward, fingerprint=adapter.FINGERPRINT__, qos=qos
        )
        self.subscription._require_slots(queue_size + 1)

    def getTopic(self):  # noqa: N802 message_filters' name
        return self.subscription.topic_name

    # A negative stamp is no ROS time: rclpy.time.Time refuses it inside the filters.
    def _forward(self, frame):
        try:
            view = self._adapter.View(frame)
            sec, nanosec = view.header__stamp
            frame_id = view.header__frame_id
        except ValueError:
            self.unreadable += 1
            return
        if sec < 0:
            self.unreadable += 1
            return
        self.forwarded += 1
        self.signalMessage(StampedFrame(frame, self._adapter, sec, nanosec, frame_id))
