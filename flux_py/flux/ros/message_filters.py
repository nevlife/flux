"""message_filters sources over flux channels, so flux and DDS topics synchronize in one graph.

Usage and the same-thread rule are in docs/en/api.en.md (message_filters, Python). The object a
callback receives already owns its borrow, so queueing it is all it takes and upstream
`message_filters` runs unmodified over it.

The filter path allocates: the policy queues are dicts and each frame carries a StampedFrame.
Plain flux delivery allocates nothing. This path does not belong in a latency-critical chain.
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

    The payload stays in the segment for as long as this lives, so `max_borrow` must cover
    inputs x queue_size (docs/en/api.en.md).
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

    Hand this to an executor directly; `ex.add_flux(sub)` reads the flux Subscription out of
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
