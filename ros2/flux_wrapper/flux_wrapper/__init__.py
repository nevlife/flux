"""One publisher and subscription API for a node that runs with or without flux.

The path is chosen once per process: flux when ENABLE_FLUX=1 is set and the flux module imports,
ROS 2 otherwise. A flux subscriber gets nothing from a publisher on the ROS path, so a launch sets
ENABLE_FLUX once for every node.
"""

import array
import functools
import os
import struct

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


# The aux bytes of a device="cuda" publisher's frames, in host byte order: the C++ structs
# flux_wrapper::detail::GpuImageAux and GpuPointCloud2Aux, whose layouts these must keep. The
# fingerprints are the FNV-1a of "flux_wrapper/GpuImage" and "flux_wrapper/GpuPointCloud2".
_GPU_VERSION = 1
_FRAME_ID_CAP = 256
_ENCODING_CAP = 32
_FIELD_CAP = 32
_NAME_CAP = 32
_GPU_IMAGE = struct.Struct("=IiIIIIBBH32s256s")
_GPU_CLOUD = struct.Struct("=IiIIIIIBBHI256s")
_GPU_FIELD = struct.Struct("=IIBBH32s")
_GPU_CLOUD_SIZE = _GPU_CLOUD.size + _FIELD_CAP * _GPU_FIELD.size
_GPU = {
    Image: (
        0x780CD7C35A0C29C8,
        _GPU_IMAGE.size,
        "encoding or frame_id longer than its cap (32, 256 bytes)",
    ),
    PointCloud2: (
        0xC900A921C6D8E9D6,
        _GPU_CLOUD_SIZE,
        "frame_id, a field name or the field count over its cap (256 bytes, 32 bytes, 32 fields)",
    ),
}


def _pack(msg_type, m):
    """The aux bytes of m's fields but data, or None when one is over its cap."""
    frame = m.header.frame_id.encode()
    stamp = m.header.stamp
    if len(frame) > _FRAME_ID_CAP:
        return None
    if msg_type is Image:
        enc = m.encoding.encode()
        if len(enc) > _ENCODING_CAP:
            return None
        return _GPU_IMAGE.pack(
            _GPU_VERSION,
            stamp.sec,
            stamp.nanosec,
            m.height,
            m.width,
            m.step,
            m.is_bigendian,
            len(enc),
            len(frame),
            enc,
            frame,
        )
    if len(m.fields) > _FIELD_CAP:
        return None
    out = bytearray(_GPU_CLOUD_SIZE)
    _GPU_CLOUD.pack_into(
        out,
        0,
        _GPU_VERSION,
        stamp.sec,
        stamp.nanosec,
        m.height,
        m.width,
        m.point_step,
        m.row_step,
        m.is_bigendian,
        m.is_dense,
        len(frame),
        len(m.fields),
        frame,
    )
    for i, f in enumerate(m.fields):
        name = f.name.encode()
        if len(name) > _NAME_CAP:
            return None
        at = _GPU_CLOUD.size + i * _GPU_FIELD.size
        _GPU_FIELD.pack_into(out, at, f.offset, f.count, f.datatype, len(name), 0, name)
    return bytes(out)


def _unpack(msg_type, aux):
    """The fields but data of a GPU frame, or None when its aux bytes, another process's, do not
    parse. Each value is read once."""
    if len(aux) != _GPU[msg_type][1]:
        return None
    if msg_type is Image:
        version, sec, nanosec, h, w, step, big, enc_len, frame_len, enc, frame = _GPU_IMAGE.unpack(
            aux
        )
        if version != _GPU_VERSION or enc_len > _ENCODING_CAP or frame_len > _FRAME_ID_CAP:
            return None
        m = Image(height=h, width=w, step=step, is_bigendian=big)
        m.encoding = enc[:enc_len].decode(errors="replace")
    else:
        version, sec, nanosec, h, w, point_step, row_step, big, dense, frame_len, n, frame = (
            _GPU_CLOUD.unpack_from(aux)
        )
        if version != _GPU_VERSION or frame_len > _FRAME_ID_CAP or n > _FIELD_CAP:
            return None
        fields = []
        for i in range(n):
            offset, count, datatype, name_len, _, name = _GPU_FIELD.unpack_from(
                aux, _GPU_CLOUD.size + i * _GPU_FIELD.size
            )
            if name_len > _NAME_CAP:
                return None
            name = name[:name_len].decode(errors="replace")
            fields.append(PointField(name=name, offset=offset, datatype=datatype, count=count))
        m = PointCloud2(height=h, width=w, point_step=point_step, row_step=row_step, fields=fields)
        m.is_bigendian, m.is_dense = bool(big), bool(dense)
    m.header.frame_id = frame[:frame_len].decode(errors="replace")
    m.header.stamp.sec, m.header.stamp.nanosec = sec, nanosec
    return m


_warned = set()


def _warn_once(logger, text):
    if text not in _warned:
        _warned.add(text)
        logger.warning(text)


def _cuda_usable(logger, device):
    """Never raises for a "cuda" that cannot be had: it is "cpu", with the reason logged once."""
    if device not in ("cpu", "cuda"):
        raise ValueError(f'flux_wrapper: device is "cpu" or "cuda", not {device!r}')
    if device == "cpu":
        return False
    if not use_flux():
        # shortcut: the ROS path needs a device buffer and copies to and from it, which the flux
        # Python module does not give; it stays "cpu" until one does.
        _warn_once(logger, 'device="cuda" on the ROS path is not supported in Python; using "cpu"')
        return False
    p = _flux.probe()
    if p.route == _flux.Route.NONE:
        _warn_once(logger, f'device="cuda": no flux GPU route ({p.reason}); using "cpu"')
        return False
    return True


class Loan:
    """A message being written: set the fields of the ROS message, then Publisher.publish(loan).

    `fields` (PointCloud2) and `header` are set by assignment; `alloc_data(n)` returns a writable
    uint8 array over the message's data. A false Loan had no free flux slot.

    On a publisher with device="cuda" the fields go to the frame's aux bytes, where strings and
    PointCloud2's fields have caps as in C++; a field over its cap makes the Loan false. The slot
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

    def __bool__(self):
        if self._gpu is not None:
            return self._slot is not False and _pack(self._type, self._msg) is not None
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
        slot = self._gpu.loan((n,), "uint8") if n <= self._gpu.slot_size else None
        object.__setattr__(self, "_slot", slot if slot is not None else False)
        return slot


class Publisher:
    """One topic over flux or ROS. On the flux path `max_message_size` bounds a message and
    `slot_count` is how many are in flight (0: the flux defaults). `mirror_to_ros` also publishes
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
        max_message_size=0,
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
        self._cuda = _cuda_usable(self._logger, device)
        if use_flux():
            self._adapter, self._ros_adapter = _adapters()[msg_type]
            fingerprint, aux_size = self._adapter.FINGERPRINT__, 0
            if self._cuda:
                fingerprint, aux_size, _ = _GPU[msg_type]
                if mirror_to_ros:
                    _warn_once(self._logger, 'mirror_to_ros is ignored with device="cuda"')
            self._flux = _flux.Publisher(
                node,
                topic,
                fingerprint=fingerprint,
                slot_size=max_message_size or _flux.DEFAULT_SLOT_SIZE,
                slot_count=slot_count or _flux.DEFAULT_SLOT_COUNT,
                mirror_to_ros=mirror_to_ros and not self._cuda,
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
                return self._report(msg._builder.commit__())
            msg = msg._msg
        if self._cuda:
            return self._publish_gpu(msg)
        if self._flux is not None:
            b = self._adapter.build__(self._flux)
            if not b:
                return False
            self._ros_adapter.msg_to_frame(msg, b)
            return self._report(b.commit__())
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
            return False
        aux = _pack(self._type, loan._msg)
        if aux is None:
            slot.abort()
            self._device_error(_GPU[self._type][2])
            return False
        slot.aux[: len(aux)] = np.frombuffer(aux, np.uint8)
        slot.aux_len = len(aux)
        return self._report(slot.commit())

    def _publish_gpu(self, msg):
        if self._flux.device_backed:
            # shortcut: the flux Python module has no host to device copy into a loan; publish(msg)
            # works once it has one.
            self._device_error(
                "publish(msg) needs a host to device copy, which the flux Python module does not "
                "give for a slot in device memory; write the data through Loan.alloc_buffer()"
            )
            return False
        if len(msg.data) > self._flux.slot_size:
            return self._report(_flux.Published.TOO_LARGE)
        # The loan only reads msg's fields.
        loan = Loan(self._type, msg=msg, gpu=self._flux)
        data = loan.alloc_data(len(msg.data))
        if data is None:
            return False
        data[:] = np.frombuffer(msg.data, np.uint8)
        return self._commit_gpu(loan)

    def _device_error(self, what):
        if not self._device_logged:
            self._device_logged = True
            self._logger.error(
                f'device="cuda" publish failed: {what}; later failures are not logged'
            )

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

    A frame of a device="cuda" publisher read by a device="cuda" subscription is on the device
    (on_device): device_data is then a flux.Frame of uint8 with __dlpack__,
    __cuda_array_interface__ and stream, and data is None."""

    def __init__(self, msg_type, msg=None, flux_view=None, frame=None):
        self._type = msg_type
        self._msg = msg
        self._v = flux_view
        # A GPU channel's frame, with its fields in _msg: a host array, or a flux.Frame.
        self._frame = frame

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
        return self._frame is not None and not isinstance(self._frame, np.ndarray)

    @property
    def device_data(self):
        return self._frame if self.on_device else None

    @property
    def ros_message(self):
        """The message itself on the ROS path; None on the flux path."""
        return self._msg if self._frame is None else None

    def to_msg(self):
        """A frame on the device raises RuntimeError: the flux Python module has no device to host
        copy of a frame."""
        if self._frame is not None:
            if self.on_device:
                raise RuntimeError(
                    "flux_wrapper: to_msg() of a frame in device memory needs a device to host "
                    "copy, which the flux Python module does not give; read device_data"
                )
            self._msg.data = self._frame.tobytes()
            return self._msg
        if self._msg is not None:
            return self._msg
        return _adapters()[self._type][1].frame_to_msg(self._v)


class Subscription:
    """One topic over flux or ROS; the callback gets a View. Add it to a flux_wrapper.Executor.
    On the flux path it also takes the frames of a device="cuda" publisher, whose fields come in
    the frame's aux bytes; a frame whose aux bytes do not parse is skipped and counted in rejected.

    device="cuda" keeps the frames of a device="cuda" publisher on the device (View.on_device).
    Frames of a "cpu" publisher still come in host memory, with a warning: the flux Python module
    does not give a frame's host bytes beside its device ones, which reading their fields needs.
    """

    def __init__(self, node, msg_type, topic, callback, qos, *, callback_group=None, device="cpu"):
        self.callback_group = callback_group
        self._flux = None
        self._gpu = None
        self._rejects = [0]
        logger = node.get_logger()
        cuda = _cuda_usable(logger, device)
        if use_flux():
            adapter = _adapters()[msg_type][0]
            q = _flux.QoS(depth=_depth(qos))

            def on_host(f):
                if cuda:
                    # shortcut: in host memory until the flux Python module gives a Frame's host
                    # bytes.
                    _warn_once(
                        logger,
                        f'device="cuda": frames of a "cpu" publisher on {topic} arrive in host '
                        "memory in Python",
                    )
                callback(View(msg_type, flux_view=adapter.View(f)))

            self._flux = _flux.Subscription(
                node, topic, callback=on_host, fingerprint=adapter.FINGERPRINT__, qos=q
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
                fingerprint=_GPU[msg_type][0],
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
