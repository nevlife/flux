"""One publisher and subscription API for a node that runs with or without flux.

The path is chosen once per process: flux when ENABLE_FLUX=1 is set and the flux module imports,
ROS 2 otherwise. A flux subscriber gets nothing from a publisher on the ROS path, so a launch sets
ENABLE_FLUX once for every node.
"""

import array
import functools
import os

import numpy as np
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

try:
    import flux as _flux
    from sensor_msgs_flux import image as _image
    from sensor_msgs_flux import image_ros as _image_ros
    from sensor_msgs_flux import point_cloud2 as _point_cloud2
    from sensor_msgs_flux import point_cloud2_ros as _point_cloud2_ros
except ImportError:
    _flux = None


@functools.cache
def use_flux():
    """True when this process's publishers and subscriptions go over flux."""
    requested = os.environ.get("ENABLE_FLUX") == "1"
    if requested and _flux is None:
        import warnings

        warnings.warn("ENABLE_FLUX=1, but the flux module does not import: ROS path", stacklevel=2)
    return requested and _flux is not None


def _adapters():
    return {
        Image: (_image, _image_ros),
        PointCloud2: (_point_cloud2, _point_cloud2_ros),
    }


_SCALARS = {
    Image: ("height", "width", "encoding", "is_bigendian", "step"),
    PointCloud2: ("height", "width", "is_bigendian", "point_step", "row_step", "is_dense"),
}


def _depth(qos):
    return qos if isinstance(qos, int) else qos.depth


class Loan:
    """A message being written: set the fields of the ROS message, then Publisher.publish(loan).

    `fields` (PointCloud2) and `header` are set by assignment; `alloc_data(n)` returns a writable
    uint8 array over the message's data. A false Loan had no free flux slot.
    """

    def __init__(self, msg_type, msg=None, builder=None):
        object.__setattr__(self, "_names", _SCALARS[msg_type])
        object.__setattr__(self, "_msg", msg)
        object.__setattr__(self, "_builder", builder)

    def __bool__(self):
        return self._msg is not None or bool(self._builder)

    def __setattr__(self, name, value):
        b = self._builder
        if b is None:
            setattr(self._msg, name, value)
        elif name == "header":
            b.set__header__stamp(value.stamp.sec, value.stamp.nanosec)
            b.header__frame_id = value.frame_id
        elif name == "fields":
            for out, f in zip(b.alloc__fields(len(value)), value):
                out.name, out.offset, out.datatype, out.count = (
                    f.name,
                    f.offset,
                    f.datatype,
                    f.count,
                )
        elif name in self._names:
            setattr(b, name, value)
        else:
            raise AttributeError(name)

    def alloc_data(self, n):
        if self._builder is not None:
            return self._builder.alloc__data(n)
        # The setter copies what it is given, so the array returned is over the copy it kept.
        self._msg.data = array.array("B", bytes(n))
        return np.frombuffer(self._msg.data, np.uint8)


class Publisher:
    """One topic over flux or ROS. On the flux path `max_message_size` bounds a message and
    `slot_count` is how many are in flight (0: the flux defaults). `mirror_to_ros` also publishes
    each one as its ROS message while a ROS subscriber wants it."""

    def __init__(
        self, node, msg_type, topic, qos, *, max_message_size=0, slot_count=0, mirror_to_ros=False
    ):
        self._type = msg_type
        self._flux = None
        self._ros = None
        self._logger = node.get_logger()
        self._fault_logged = False
        if use_flux():
            self._adapter, self._ros_adapter = _adapters()[msg_type]
            self._flux = _flux.Publisher(
                node,
                topic,
                fingerprint=self._adapter.FINGERPRINT__,
                slot_size=max_message_size or _flux.DEFAULT_SLOT_SIZE,
                slot_count=slot_count or _flux.DEFAULT_SLOT_COUNT,
                mirror_to_ros=mirror_to_ros,
            )
        else:
            self._ros = node.create_publisher(msg_type, topic, qos)

    @property
    def uses_flux(self):
        return self._flux is not None

    def loan(self):
        if self._flux is not None:
            return Loan(self._type, builder=self._adapter.build__(self._flux))
        return Loan(self._type, msg=self._type())

    def publish(self, msg):
        """Publish a Loan or a ROS message (the flux path copies it into a slot once). False when
        nothing went out."""
        if isinstance(msg, Loan):
            if msg._builder is not None:
                return self._report(msg._builder.commit__())
            msg = msg._msg
        if self._flux is not None:
            b = self._adapter.build__(self._flux)
            if not b:
                return False
            self._ros_adapter.msg_to_frame(msg, b)
            return self._report(b.commit__())
        self._ros.publish(msg)
        return True

    def get_subscription_count(self):
        """flux keeps no subscriber count, so the flux path reports 1."""
        return 1 if self._flux is not None else self._ros.get_subscription_count()

    def _report(self, p):
        if _flux.faulted(p) and not self._fault_logged:
            self._fault_logged = True
            self._logger.error(
                f"flux publish on {self._flux.topic_name} failed: {p}; later failures are not logged"
            )
        return p == _flux.Published.OK


class View:
    """The fields of a received message. On the flux path `data` and the View alias the slot and
    are valid only during the callback; copy out (to_msg()) what is kept longer."""

    def __init__(self, msg_type, msg=None, flux_view=None):
        self._type = msg_type
        self._msg = msg
        self._v = flux_view

    def __getattr__(self, name):
        if name not in _SCALARS[self._type]:
            raise AttributeError(name)
        return getattr(self._msg if self._msg is not None else self._v, name)

    @property
    def header(self):
        if self._msg is not None:
            return self._msg.header
        sec, nanosec = self._v.header__stamp
        h = Header(frame_id=self._v.header__frame_id)
        h.stamp.sec, h.stamp.nanosec = sec, nanosec
        return h

    @property
    def fields(self):
        if self._msg is not None:
            return self._msg.fields
        return [
            PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
            for f in self._v.fields
        ]

    @property
    def data(self):
        if self._msg is not None:
            return np.frombuffer(self._msg.data, np.uint8)
        return self._v.data

    @property
    def ros_message(self):
        """The message itself on the ROS path; None on the flux path."""
        return self._msg

    def to_msg(self):
        if self._msg is not None:
            return self._msg
        return _adapters()[self._type][1].frame_to_msg(self._v)


class Subscription:
    """One topic over flux or ROS; the callback gets a View. Add it to a flux_wrapper.Executor."""

    def __init__(self, node, msg_type, topic, callback, qos, *, callback_group=None):
        self.callback_group = callback_group
        self._flux = None
        if use_flux():
            adapter = _adapters()[msg_type][0]
            self._flux = _flux.Subscription(
                node,
                topic,
                callback=lambda f: callback(View(msg_type, flux_view=adapter.View(f))),
                fingerprint=adapter.FINGERPRINT__,
                qos=_flux.QoS(depth=_depth(qos)),
            )
        else:
            self._ros = node.create_subscription(
                msg_type,
                topic,
                lambda m: callback(View(msg_type, msg=m)),
                qos,
                callback_group=callback_group,
            )

    @property
    def uses_flux(self):
        return self._flux is not None


class Executor:
    """flux.Executor on the flux path, rclpy's SingleThreadedExecutor otherwise."""

    def __init__(self):
        # The rclpy executor is made by the first add_node(), so it takes that node's context.
        self._ex = _flux.Executor() if use_flux() else None

    def add(self, subscription):
        if subscription._flux is not None:
            self._ex.add(subscription._flux)

    def add_node(self, node):
        if self._ex is None:
            self._ex = SingleThreadedExecutor(context=node.context)
        self._ex.add_node(node)

    def spin(self):
        self._ex.spin()

    def stop(self):
        """Ends spin(), from any thread."""
        if use_flux():
            self._ex.stop()
        else:
            self._ex.shutdown(timeout_sec=0)
