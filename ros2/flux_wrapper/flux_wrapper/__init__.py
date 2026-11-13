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


# The aux bytes of a device="cuda" publisher's frames: a frame of the type with data empty, as the
# C++ side writes it (flux_wrapper::detail::Gpu); the payload is the data field. The fingerprint is
# flux.gpu_fingerprint of the type's.
_GPU_AUX_BYTES = 4096
_GPU_OVER = "a field other than data is over the aux's 4096 bytes"


def _gpu_fingerprint(msg_type):
    return _flux.gpu_fingerprint(_adapters()[msg_type][0].FINGERPRINT__)


def _pack(msg_type, m):
    """The aux bytes of m's fields but data, or None when they are over the aux."""
    adapter, ros_adapter = _adapters()[msg_type]
    buf = np.zeros(_GPU_AUX_BYTES, np.uint8)
    b = adapter.Builder(buf)
    data, m.data = m.data, []
    try:
        ros_adapter.msg_to_frame(m, b)
    except _flux.wire.WireError:
        return None
    finally:
        m.data = data
    if not b:
        return None
    return bytes(buf[: b.size__])


def _first_over(msg_type, m):
    """The first field but data of m that does not fit the aux, as (field, bytes, entries, bytes
    used before it), in the order the C++ Loan writes them; None when all fit. For the log line
    only: it writes the fields again."""
    adapter, _ = _adapters()[msg_type]
    b = adapter.Builder(np.zeros(_GPU_AUX_BYTES, np.uint8))
    b.set__header__stamp(m.header.stamp.sec, m.header.stamp.nanosec)

    def fields():
        out = b.alloc__fields(len(m.fields))
        for i, f in enumerate(m.fields):
            out[i].name, out[i].offset = f.name, f.offset
            out[i].datatype, out[i].count = f.datatype, f.count

    steps = [
        (
            "header.frame_id",
            len(m.header.frame_id.encode()),
            0,
            lambda: setattr(b, "header__frame_id", m.header.frame_id),
        )
    ]
    if msg_type is Image:
        steps.append(
            ("encoding", len(m.encoding.encode()), 0, lambda: setattr(b, "encoding", m.encoding))
        )
    else:
        steps.append(("fields", sum(len(f.name.encode()) for f in m.fields), len(m.fields), fields))
    for field, nbytes, entries, put in steps:
        used = b.size__
        try:
            put()
        except _flux.wire.WireError:
            return field, nbytes, entries, used
        if not b:
            return field, nbytes, entries, used
    return None


def _unpack(msg_type, aux):
    """The fields but data of a GPU frame, or None when its aux bytes, another process's, do not
    parse."""
    adapter, ros_adapter = _adapters()[msg_type]
    try:
        return ros_adapter.frame_to_msg(adapter.View(bytes(aux)))
    except (_flux.wire.WireError, ValueError, IndexError):
        return None


def _cuda_usable(logger, topic, device):
    """Never raises for a "cuda" that cannot be had: it is "cpu", with the reason logged for this
    topic."""
    if device not in ("cpu", "cuda"):
        raise ValueError(f'flux_wrapper: device is "cpu" or "cuda", not {device!r}')
    if device == "cpu":
        return False
    if not use_flux():
        # shortcut: the ROS path needs a device buffer and copies to and from it, which the flux
        # Python module does not give; it stays "cpu" until one does.
        logger.warning(
            f'{topic}: device="cuda" on the ROS path is not supported in Python; using "cpu"'
        )
        return False
    p = _flux.probe()
    if p.route == _flux.Route.NONE:
        logger.warning(f'{topic}: device="cuda": no flux GPU route ({p.reason}); using "cpu"')
        return False
    return True


class Loan:
    """A message being written: set the fields of the ROS message, then Publisher.publish(loan).

    `fields` (PointCloud2) and `header` are set by assignment; `alloc_data(n)` returns a writable
    uint8 array over the message's data. A false Loan had no free flux slot.

    On a publisher with device="cuda" the fields go to the frame's aux bytes as in C++; fields over
    its 4096 bytes make the Loan false. The slot
    is taken by alloc_buffer(n) or alloc_data(n), so the Loan is true until they found none free.
    """

    def __init__(self, msg_type, msg=None, builder=None, gpu=None):
        object.__setattr__(self, "_type", msg_type)
        object.__setattr__(self, "_names", _SCALARS[msg_type])
        object.__setattr__(self, "_msg", msg)
        object.__setattr__(self, "_builder", builder)
        # A device="cuda" flux publisher's loan keeps the fields in _msg and the data in _slot: a
        # flux.Loan once taken, False when none was free.
        object.__setattr__(self, "_gpu", gpu)
        object.__setattr__(self, "_slot", None)
        object.__setattr__(self, "_too_large", False)  # the data asked for was past the slot
        # On the flux path: the first string or list that did not fit the slot, and header.stamp,
        # for the publisher's log line.
        object.__setattr__(self, "_over", None)
        object.__setattr__(self, "_stamp", (0, 0))

    def __bool__(self):
        if self._gpu is not None:
            return self._slot is not False and _pack(self._type, self._msg) is not None
        return self._msg is not None or bool(self._builder)

    def __setattr__(self, name, value):
        b = self._builder
        if b is None:
            setattr(self._msg, name, value)
        elif not b and (name in self._names or name in ("header", "fields")):
            pass  # a field did not fit or no slot was free: the frame will not go out, as in C++
        elif name == "header":
            b.set__header__stamp(value.stamp.sec, value.stamp.nanosec)
            object.__setattr__(self, "_stamp", (value.stamp.sec, value.stamp.nanosec))
            self._put(
                "header.frame_id",
                len(value.frame_id.encode()),
                0,
                lambda: setattr(b, "header__frame_id", value.frame_id),
            )
        elif name == "fields":

            def fields():
                for out, f in zip(b.alloc__fields(len(value)), value):
                    out.name, out.offset, out.datatype, out.count = (
                        f.name,
                        f.offset,
                        f.datatype,
                        f.count,
                    )

            self._put("fields", sum(len(f.name.encode()) for f in value), len(value), fields)
        elif name in self._names and isinstance(value, str):
            self._put(name, len(value.encode()), 0, lambda: setattr(b, name, value))
        elif name in self._names:
            setattr(b, name, value)
        else:
            raise AttributeError(name)

    def _put(self, field, nbytes, entries, put):
        """Runs put(), which writes `field`; notes it when it is the first that does not fit. The
        Loan is false then, as in C++."""
        b = self._builder
        fit, used = bool(b), b.size__
        try:
            put()
        except _flux.wire.WireError:
            pass  # the Builder is bad now
        if fit and not b and self._over is None:
            object.__setattr__(self, "_over", (field, nbytes, entries, used))

    def alloc_data(self, n):
        """On a device="cuda" publisher, None when n bytes do not fit the slot or the slot is device
        memory: use alloc_buffer(n)."""
        if self._gpu is not None:
            slot = None if self._gpu.device_backed else self._take(n)
            return slot.array if slot else None
        if self._builder is not None:
            return self._builder.alloc__data(n)
        # The setter copies what it is given, so the array returned is over the copy it kept.
        self._msg.data = array.array("B", bytes(n))
        return np.frombuffer(self._msg.data, np.uint8)

    def alloc_buffer(self, n):
        """n bytes of the data field. On a device="cuda" publisher a flux.Loan of uint8[n], written
        on the device through __dlpack__ or __cuda_array_interface__ (the publish waits for its
        stream), or None when no slot was free or n does not fit one; else as alloc_data(n)."""
        if self._gpu is not None:
            return self._take(n) or None
        return self.alloc_data(n)

    def _take(self, n):
        if self._slot:
            self._slot.abort()
        object.__setattr__(self, "_too_large", n > self._gpu.slot_size)
        slot = None if self._too_large else self._gpu.loan((n,), "uint8")
        object.__setattr__(self, "_slot", slot if slot is not None else False)
        return slot


class Publisher:
    """One topic over flux or ROS. On the flux path `slot_count` is how many messages are in flight
    (0: the flux default), and with device="cuda" `max_device_data_size` bounds the data field (0:
    8 MiB); on "cpu" the data field goes to the publisher's arena, whatever its size. `mirror_to_ros` also publishes
    each one as its ROS message while a ROS subscriber wants it.

    device="cuda" puts the data field in GPU memory (Loan.alloc_buffer()), on a flux GPU channel
    that the C++ flux_wrapper shares. Where that cannot be had (no flux GPU route, the ROS path)
    the publisher is "cpu", with a warning; uses_device says which. mirror_to_ros is ignored then.
    """

    def __init__(
        self,
        node,
        msg_type,
        topic,
        qos,
        *,
        max_device_data_size=0,
        slot_count=0,
        mirror_to_ros=False,
        device="cpu",
    ):
        self._type = msg_type
        self._flux = None
        self._ros = None
        self._logger = node.get_logger()
        self._fault_logged = False
        self._device_logged = False
        self._over_logged = False
        self._cuda = _cuda_usable(self._logger, topic, device)
        if use_flux():
            self._adapter = _adapters()[msg_type][0]
            fingerprint, aux_size, slot = self._adapter.FINGERPRINT__, 0, _flux.DEFAULT_SLOT_SIZE
            if self._cuda:
                fingerprint, aux_size = _gpu_fingerprint(msg_type), _GPU_AUX_BYTES
                slot = max_device_data_size
            self._flux = _flux.Publisher(
                node,
                topic,
                fingerprint=fingerprint,
                slot_size=slot,
                slot_count=slot_count,
                mirror_to_ros=mirror_to_ros,
                device="cuda" if self._cuda else "cpu",
                aux_size=aux_size,
            )
        else:
            self._ros = node.create_publisher(msg_type, topic, qos)

    @property
    def uses_flux(self):
        return self._flux is not None

    @property
    def uses_device(self):
        """device="cuda" was asked for and is used."""
        return self._cuda

    def loan(self):
        if self._cuda:
            return Loan(self._type, msg=self._type(), gpu=self._flux)
        if self._flux is not None:
            return Loan(self._type, builder=self._adapter.build__(self._flux))
        return Loan(self._type, msg=self._type())

    def publish(self, msg):
        """Publish a Loan or a ROS message (the flux path copies it into a slot once). False when
        nothing went out."""
        if isinstance(msg, Loan):
            if msg._gpu is not None:
                return self._commit_gpu(msg)
            if msg._builder is not None:
                if not msg._builder and msg._over is not None:
                    msg._builder.commit__()  # refused
                    self._overflow("", "slot", self._flux.slot_size, msg._over, msg._stamp)
                    return False
                return self._report(msg._builder.commit__())
            msg = msg._msg
        if self._cuda:
            return self._publish_gpu(msg)
        if self._flux is not None:
            return self._publish_host(msg)
        self._ros.publish(msg)
        return True

    def get_subscription_count(self):
        """The flux path counts subscribing processes."""
        if self._flux is not None:
            return self._flux.subscriber_count
        return self._ros.get_subscription_count()

    def _commit_gpu(self, loan):
        slot = loan._slot if loan._slot is not None else loan._take(0)
        object.__setattr__(loan, "_slot", False)
        if not slot:
            return loan._too_large and self._report(_flux.Published.TOO_LARGE)
        aux = _pack(self._type, loan._msg)
        if aux is None:
            slot.abort()
            over = _first_over(self._type, loan._msg)
            if over is None:
                self._device_error(_GPU_OVER)
            else:
                stamp = (loan._msg.header.stamp.sec, loan._msg.header.stamp.nanosec)
                self._overflow(' on device="cuda"', "aux", _GPU_AUX_BYTES, over, stamp)
            return False
        slot.aux[: len(aux)] = np.frombuffer(aux, np.uint8)
        slot.aux_len = len(aux)
        return self._report(slot.commit())

    def _publish_gpu(self, msg):
        # The loan only reads msg's fields. Its data goes into the slot in host or device memory.
        loan = Loan(self._type, msg=msg, gpu=self._flux)
        slot = loan._take(len(msg.data))
        if slot is not None and not slot.copy_from(np.frombuffer(msg.data, np.uint8)):
            slot.abort()
            self._device_error("the copy of data into the slot was not queued")
            return False
        return self._commit_gpu(loan)

    def _device_error(self, what):
        if not self._device_logged:
            self._device_logged = True
            self._logger.error(
                f'device="cuda" publish failed: {what}; later failures are not logged'
            )

    def _overflow(self, on, where, size, over, stamp):
        """Once per publisher: which field of which message did not fit, to size the aux or slot
        by."""
        if self._over_logged:
            return
        self._over_logged = True
        field, nbytes, entries, used = over
        what = (
            f"{field} ({entries} entries, names {nbytes} B" if entries else f"{field} ({nbytes} B"
        )
        self._logger.error(
            f"{self._flux.topic_name}: a message did not go out{on}: {what}) did not fit the "
            f"{where} ({size} B, {used} B used before it); header.stamp {stamp[0]}.{stamp[1]:09d}. "
            "Later ones are not logged"
        )

    def _publish_host(self, msg):
        """As a Loan filled from msg, so a field that does not fit is logged as one set by hand is."""
        loan = self.loan()
        if not loan:
            return False
        loan.header = msg.header
        for name in loan._names:
            setattr(loan, name, getattr(msg, name))
        if self._type is PointCloud2:
            loan.fields = msg.fields
        n = len(msg.data)
        if loan and n:
            data = loan.alloc_data(n)
            if data is not None and len(data) == n:
                data[:] = msg.data
        return self.publish(loan)

    def _report(self, p):
        if _flux.faulted(p) and not self._fault_logged:
            self._fault_logged = True
            self._logger.error(
                f"flux publish on {self._flux.topic_name} failed: {p}; later failures are not logged"
            )
        return p == _flux.Published.OK


class View:
    """The fields of a received message. On the flux path `data` and the View alias the slot and
    are valid only during the callback; copy out (to_msg()) what is kept longer.

    A device="cuda" subscription has `data` on the device too (on_device): device_data then has
    __dlpack__, __cuda_array_interface__ and nbytes, and stream is the stream to queue its work on.
    Of a device="cuda" publisher's frame it is a flux.Frame of uint8 and data is None; of a "cpu"
    publisher's, a flux.wire.DeviceSpan beside data in host memory."""

    def __init__(self, msg_type, msg=None, flux_view=None, frame=None, device=None, stream=None):
        self._type = msg_type
        self._msg = msg
        self._v = flux_view
        # A GPU channel's frame, with its fields in _msg: a host array, or a flux.Frame.
        self._frame = frame
        # A "cpu" publisher's data on the device, for a device="cuda" subscription.
        self._device = device
        self._stream = stream

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
        if self._frame is not None:
            return None if self.on_device else self._frame
        if self._msg is not None:
            return np.frombuffer(self._msg.data, np.uint8)
        return self._v.data

    @property
    def on_device(self):
        if self._device is not None:
            return True
        return self._frame is not None and not isinstance(self._frame, np.ndarray)

    @property
    def device_data(self):
        if self._device is not None:
            return self._device
        return self._frame if self.on_device else None

    @property
    def stream(self):
        """The stream device_data is read on, which the release waits for; None off the device."""
        if self._device is not None:
            return self._stream
        return self._frame.stream if self.on_device else None

    @property
    def ros_message(self):
        """The message itself on the ROS path; None on the flux path."""
        return self._msg if self._frame is None else None

    def to_msg(self):
        """The message with its data in host memory: a frame on the device is copied to the host,
        which waits for its stream."""
        if self._frame is not None:
            data = self._frame.copy_to_host() if self.on_device else self._frame
            self._msg.data = data.tobytes()
            return self._msg
        if self._msg is not None:
            return self._msg
        return _adapters()[self._type][1].frame_to_msg(self._v)


class Subscription:
    """One topic over flux or ROS; the callback gets a View. Add it to a flux_wrapper.Executor.
    On the flux path it also takes the frames of a device="cuda" publisher, whose fields come in
    the frame's aux bytes; a frame whose aux bytes do not parse is skipped and counted in rejected.

    device="cuda" has every frame's data on the device (View.on_device): a device="cuda"
    publisher's in its slot, a "cpu" publisher's as this subscription's copy beside its host bytes.
    """

    def __init__(self, node, msg_type, topic, callback, qos, *, callback_group=None, device="cpu"):
        self.callback_group = callback_group
        self._flux = None
        self._gpu = None
        self._rejects = [0]
        logger = node.get_logger()
        cuda = _cuda_usable(logger, topic, device)
        if use_flux():
            adapter = _adapters()[msg_type][0]
            q = _flux.QoS(depth=_depth(qos))

            def on_typed(f):
                v = adapter.View(f)
                if isinstance(f, _flux.Frame):
                    callback(View(msg_type, flux_view=v, device=v.data__device, stream=f.stream))
                else:
                    callback(View(msg_type, flux_view=v))

            self._flux = _flux.Subscription(
                node,
                topic,
                callback=on_typed,
                fingerprint=adapter.FINGERPRINT__,
                qos=q,
                device="cuda" if cuda else "cpu",
            )
            rejects = self._rejects

            def on_gpu(frame):
                m = _unpack(msg_type, _flux.aux(frame))
                if m is None:
                    rejects[0] += 1
                    if rejects[0] == 1:
                        logger.error(
                            f"{topic}: skipped a GPU frame with a bad aux header; later ones are "
                            "counted in rejected, not logged"
                        )
                    return
                callback(View(msg_type, msg=m, frame=frame))

            self._gpu = _flux.Subscription(
                node,
                topic,
                callback=on_gpu,
                fingerprint=_gpu_fingerprint(msg_type),
                qos=q,
                device="cuda" if cuda else "cpu",
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

    @property
    def rejected(self):
        """Frames of a device="cuda" publisher skipped for a bad aux header; the first is logged."""
        return self._rejects[0]


class Executor:
    """flux.Executor on the flux path, rclpy's SingleThreadedExecutor otherwise."""

    def __init__(self):
        # The rclpy executor is made by the first add_node(), so it takes that node's context.
        self._ex = _flux.Executor() if use_flux() else None

    def add(self, subscription):
        if subscription._flux is not None:
            self._ex.add(subscription._flux)
            self._ex.add(subscription._gpu)

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
