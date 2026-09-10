"""message_filters sources over flux channels, so flux and DDS topics synchronize in one graph.

The C++ counterpart is `flux::ros::message_filters` (docs/en/api.en.md 3). Same shape, one difference
that the language forces: there, a callback is handed a `const FrameView &` whose borrow ends
when it returns, so the filter has to `take()` the frame to own it. Here the object a callback
receives already owns its borrow -- the numpy view's capsule holds it -- so queueing that object
is all it takes, and upstream `message_filters` runs unmodified over it.

    from sensor_msgs_flux.image import Image      # the flux adapter
    from sensor_msgs.msg import Image as RosImage

    left  = flux.ros.message_filters.Subscriber(node, Image, "left")
    right = message_filters.Subscriber(node, RosImage, "right")   # a DDS topic, upstream's own

    sync = message_filters.ApproximateTimeSynchronizer([left, right], 10, 0.02)
    sync.registerCallback(on_pair)

    ex = flux.ros.Executor()
    ex.add_flux(left)          # the flux half is driven by the flux executor
    ex.add_ros_node(node)      # the DDS half by rclpy, on the same thread
    ex.spin()

Every input of one synchronizer must be serviced by the same thread. The sync policies run the
matched callback holding their own `threading.Lock`, and inputs on two threads couple through it.
`flux.ros.Executor` satisfies this by construction. `PartitionedExecutor` cannot for a mixed
graph -- rclpy has no `add_callback_group`, so a DDS input stays on its node's thread -- and
`add_sync_group` there refuses the mix rather than let it run coupled.

The filter path allocates: the policy queues are dicts and each frame carries a StampedFrame.
Plain flux delivery allocates nothing. This path does not belong in a latency-critical chain
.
"""

from builtin_interfaces.msg import Time
from message_filters.simple_filter import SimpleFilter

from . import Subscription

__all__ = ["StampedFrame", "Subscriber"]


class _Header:
    """Only `stamp`, because `header.stamp` is all upstream reads. The real message type, not a
    duck: `rclpy.time.Time.from_msg` isinstance-checks its argument."""

    __slots__ = ("stamp",)

    def __init__(self, stamp):
        self.stamp = stamp


class StampedFrame:
    """What a filter queue holds: the borrow and the stamp, never the bytes.

    The payload stays in the segment for as long as this lives, which is what keeps the filter
    path zero-copy -- and also what makes `max_borrow` the filter's business. A synchronizer
    holds up to `queue_size` frames per input while it waits for partners, so `max_borrow` must
    cover inputs x queue_size or the consumer runs out of leases and stops taking
    (docs/en/borrow_lifetime.en.md).
    """

    __slots__ = ("frame", "header", "_adapter")

    def __init__(self, frame, adapter, sec, nanosec):
        self.frame = frame
        self.header = _Header(Time(sec=sec, nanosec=nanosec))
        self._adapter = adapter

    def view(self):
        """The adapter's View over this frame. Valid while this object is alive."""
        return self._adapter.View(self.frame)


class Subscriber(SimpleFilter):
    """A flux subscription that is also a message_filters source.

    Named after the upstream class it stands in for: porting a graph to flux changes the
    namespace, not the shape. `adapter` is a flux_gen module (`from my_pkg_flux.image import
    Image`); its `FINGERPRINT__` is what this subscribes with, and its `View` is what `view()`
    returns.

    The synchronization key is the message's header stamp. A schema with no `std_msgs/Header`
    cannot go here: `FrameMeta` carries no time of its own, so there is no key. C++ stops at
    compile time; here the first frame raises.

    Hand this to an executor directly -- `ex.add_flux(sub)` reads the flux Subscription out of
    it. `.subscription` is that Subscription, for the pull surface and the QoS counters.
    """

    def __init__(self, node, adapter, topic, **kwargs):
        SimpleFilter.__init__(self)
        if not hasattr(adapter, "FINGERPRINT__"):
            raise TypeError(
                "flux: message_filters.Subscriber(node, adapter, topic) expects a flux_gen "
                "adapter module as its second argument, e.g. "
                "`from sensor_msgs_flux.image import Image`"
            )
        self.adapter = adapter
        self.topic = topic
        self.forwarded = 0
        self.unreadable = 0
        self.subscription = Subscription(
            node, topic, callback=self._forward, fingerprint=adapter.FINGERPRINT__, **kwargs
        )

    def getTopic(self):  # noqa: N802 - upstream spelling
        return self.topic

    def _forward(self, frame):
        from flux_gen.wire import WireError

        try:
            sec, nanosec = self.adapter.View(frame).header__stamp
        except AttributeError as exc:
            raise TypeError(
                f"flux: {self.adapter.TYPE_NAME__} has no std_msgs/Header, so there is no stamp "
                "to synchronize on. message_filters keys on the header stamp and FrameMeta "
                "carries no time of its own."
            ) from exc
        except WireError:
            # A frame that disagrees with the descriptor is a wrong message, not a late one:
            # nothing is forwarded and the synchronizer never sees it.
            self.unreadable += 1
            return
        self.forwarded += 1
        self.signalMessage(StampedFrame(frame, self.adapter, sec, nanosec))
