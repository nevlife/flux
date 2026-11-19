"""One publisher and subscription API for a node that runs with or without flux.

The path is chosen once per process: flux when ENABLE_FLUX=1 is set and the flux module imports,
ROS 2 otherwise. A flux subscriber gets nothing from a publisher on the ROS path, so a launch sets
ENABLE_FLUX once for every node.
"""

import array
import functools
import importlib
import keyword
import os

import numpy as np
from ament_index_python.packages import PackageNotFoundError as _PackageNotFoundError
from ament_index_python.resources import has_resource as _has_resource_in
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

__all__ = [
    "Executor",
    "Loan",
    "PublishResult",
    "Publisher",
    "Subscription",
    "View",
    "create_publisher",
    "create_subscription",
    "use_flux",
]


def _has_resource(kind, name):
    try:
        return _has_resource_in(kind, name)
    except _PackageNotFoundError:
        return False


try:
    import flux as _flux
except ImportError:
    _flux = None


@functools.cache
def use_flux():
    """True when this process's publishers and subscriptions go over flux: ENABLE_FLUX=1 at run
    time and flux_wrapper built with flux, as the C++ side decides."""
    requested = os.environ.get("ENABLE_FLUX") == "1"
    if not requested:
        return False
    import warnings

    if not _has_resource("flux_wrapper_with_flux", "flux_wrapper"):
        warnings.warn(
            "ENABLE_FLUX=1, but flux_wrapper was built without flux: ROS path", stacklevel=2
        )
        return False
    if _flux is None:
        warnings.warn("ENABLE_FLUX=1, but the flux module does not import: ROS path", stacklevel=2)
        return False
    return True


@functools.cache
def _adapter(msg_type):
    """The flux adapter of msg_type and its ROS bridge, named from the rosidl module (pkg.msg._snake)
    as flux_generate_adapters() names them: pkg_flux.snake and pkg_flux.snake_ros."""
    pkg = msg_type.__module__.split(".")[0]
    snake = msg_type.__module__.rsplit(".", 1)[1].removeprefix("_")
    base = f"{pkg}_flux.{snake}_" if keyword.iskeyword(snake) else f"{pkg}_flux.{snake}"
    ros = f"{pkg}_flux.{snake}_ros"
    try:
        adapter, bridge = importlib.import_module(base), importlib.import_module(ros)
    except ModuleNotFoundError as e:
        raise TypeError(
            f"flux_wrapper: no flux adapter for {pkg}/{msg_type.__name__}: {ros} does not import "
            f"({e}); flux_generate_adapters() in {pkg} generates it"
        ) from e
    if bridge.MESSAGE is not msg_type:
        raise TypeError(f"flux_wrapper: {ros} is the adapter of {bridge.MESSAGE}, not {msg_type}")
    return adapter, bridge


@functools.cache
def _root_data(msg_type):
    """Whether msg_type has a root data of bytes (uint8[] or byte[]), which the wrapper writes and
    reads in place and device="cuda" puts in GPU memory."""
    kind = msg_type.get_fields_and_field_types().get("data")
    return kind in ("sequence<uint8>", "sequence<octet>")


@functools.cache
def _root_header(msg_type):
    return msg_type.get_fields_and_field_types().get("header") == "std_msgs/Header"


def _copy_but_data(m):
    """A message with every field of m but data, which stays empty. The fields are m's own
    objects: rclpy's setters keep what they are given."""
    out = type(m)()
    for name in m.get_fields_and_field_types():
        if name != "data":
            setattr(out, name, getattr(m, name))
    return out


def _data_bytes(data):
    """A data field as a buffer. rclpy holds a byte[] it received as a list of one-byte bytes."""
    return b"".join(data) if isinstance(data, list) else data


def _set_data(m, buf):
    """buf (bytes) as m's data, in the form rclpy itself gives that field."""
    octets = m.get_fields_and_field_types()["data"] == "sequence<octet>"
    m.data = [bytes((x,)) for x in buf] if octets else buf


# The types with an accessor per field; any other is set from and read as its ROS message.
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
    return _flux.gpu_fingerprint(_adapter(msg_type)[0].FINGERPRINT__)


def _unpack(msg_type, aux):
    """The fields but data of a GPU frame, or None when its aux bytes, another process's, do not
    parse."""
    adapter, bridge = _adapter(msg_type)
    try:
        return bridge.frame_to_msg(adapter.View(bytes(aux)))
    except (_flux.wire.WireError, ValueError, IndexError):
        return None


def _cuda_usable(logger, topic, device, msg_type):
    """Never raises for a "cuda" that cannot be had: it is "cpu", with the reason logged for this
    topic."""
    if device not in ("cpu", "cuda"):
        raise ValueError(f'flux_wrapper: device is "cpu" or "cuda", not {device!r}')
    if device == "cpu":
        return False
    if not _root_data(msg_type):
        logger.warning(
            f'{topic}: device="cuda", but the type has no root uint8[] data to put there; '
            'using "cpu"'
        )
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


class PublishResult:
    """What publish() gives: truthy when the message went out, `status` why not otherwise, one of
    "ok", "backpressure" (no free slot), "too_large", "invalid_argument", "in_use",
    "not_publisher", "fence_failed", "gpu_wedged" and "device_error", as C++ flux_wrapper::Published.
    """

    __slots__ = ("status",)

    def __init__(self, status):
        self.status = status

    def __bool__(self):
        return self.status == "ok"

    def __repr__(self):
        return f"PublishResult({self.status!r})"


_OK = PublishResult("ok")


class Loan:
    """A message being written: set the fields of the ROS message, then Publisher.publish(loan).

    Image and PointCloud2 set each field by assignment; any other type, as in C++, all but data at
    once with set_from(m) and `header` alone when it has a root header. `alloc_data(n)` returns a
    writable uint8 array over the message's root data, written in place. A loan starts from a new
    message, so a field not set has the type's default. A false Loan had no free flux slot.

    On the flux path each field goes to the slot as it is set, so one that does not fit makes the
    Loan false at once. On a publisher with device="cuda" they go to the slot's aux bytes, as in
    C++, and fields over its 4096 bytes make the Loan false.
    """

    def __init__(self, msg_type, msg=None, builder=None, gpu=None):
        """gpu, of a device="cuda" flux publisher's loan: its flux.Loan (None when no slot was free),
        which builder writes the aux of, and whether that slot is in device memory."""
        object.__setattr__(self, "_type", msg_type)
        object.__setattr__(self, "_names", _SCALARS.get(msg_type))
        object.__setattr__(self, "_msg", msg)
        object.__setattr__(self, "_builder", builder)
        object.__setattr__(self, "_gpu", gpu is not None)
        object.__setattr__(self, "_slot", gpu[0] if gpu else None)
        object.__setattr__(self, "_device_backed", bool(gpu and gpu[1]))
        # The data asked for of the slot, and whether some asked for was past it.
        object.__setattr__(self, "_len", 0)
        object.__setattr__(self, "_too_large", False)
        # On the flux path: the first string or list that did not fit the slot, and header.stamp,
        # for the publisher's log line.
        object.__setattr__(self, "_over", None)
        object.__setattr__(self, "_stamp", (0, 0))
        object.__setattr__(self, "_step", None)

    def __bool__(self):
        if self._gpu:
            return bool(self._slot) and not self._too_large and bool(self._builder)
        if self._builder is not None:
            return bool(self._builder)
        return self._msg is not None

    def __setattr__(self, name, value):
        names = self._names or ()
        if not names and (name != "header" or not _root_header(self._type)):
            raise AttributeError(
                f"{self._type.__name__}: set {name} with set_from(m), as in C++; only a root header "
                "is set alone"
            )
        if (
            names
            and name not in names
            and name != "header"
            and (name != "fields" or self._type is not PointCloud2)
        ):
            raise AttributeError(
                f"{self._type.__name__}: no setter for {name}, as in C++; data is written with "
                "alloc_data(n) or alloc_buffer(n)"
            )
        if name == "step":
            object.__setattr__(self, "_step", value)
        b = self._builder
        if b is None:
            setattr(self._msg, name, value)
        elif not b:
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
        elif isinstance(value, str):
            self._put(name, len(value.encode()), 0, lambda: setattr(b, name, value))
        else:
            setattr(b, name, value)

    def set_from(self, m):
        """Every field of the ROS message m but data (every field, without a root data)."""
        root = _root_data(self._type)
        names = [n for n in m.get_fields_and_field_types() if n != "data" or not root]
        if self._names is not None:
            for name in names:
                setattr(self, name, getattr(m, name))
            return
        b = self._builder
        if b is None:
            for name in names:
                setattr(self._msg, name, getattr(m, name))
            return
        # As the C++ Loan: the fields go to the slot in one call, so which one did not fit is not
        # known.
        bridge = _adapter(self._type)[1]
        write = bridge.msg_to_frame_without_data if root else bridge.msg_to_frame
        if _root_header(self._type):
            object.__setattr__(self, "_stamp", (m.header.stamp.sec, m.header.stamp.nanosec))
        self._put("", 0, 0, lambda: write(m, b))

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
        """None, as C++'s empty span, when the Loan is false (publish() would send nothing), when n
        bytes do not fit the flux slot, and on a slot in device memory: use alloc_buffer(n)."""
        if not _root_data(self._type):
            raise AttributeError(f"{self._type.__name__} has no root uint8[] data")
        if not self:
            return None
        if self._gpu:
            if not self._device_backed:
                slot = self._data(n)
                return None if slot is None else slot.array
            if n > self._slot.capacity:
                object.__setattr__(self, "_too_large", True)  # as _data(): the Loan is false
            return None
        if self._builder is not None:
            try:
                return self._builder.alloc__data(n)
            except _flux.wire.WireError:
                return None  # past the slot: the Builder is bad now
        # The setter keeps the array it is given, and rclpy serializes a byte[] from a buffer too.
        self._msg.data = array.array("B", bytes(n))
        return np.frombuffer(self._msg.data, np.uint8)

    def alloc_buffer(self, n):
        """n bytes of the data field. On a device="cuda" publisher a flux.Loan of uint8[n], written
        on the device through __dlpack__ or __cuda_array_interface__ (the publish waits for its
        stream), or None when the Loan is false or n does not fit the slot; else as alloc_data(n)."""
        if not self._gpu:
            return self.alloc_data(n)
        return self._data(n) if self else None

    def _data(self, n):
        """The slot of a true Loan as uint8[n], or None when n is past it. Past it, the Loan is
        false from then on, as in C++."""
        if n > self._slot.capacity:
            object.__setattr__(self, "_too_large", True)
            return None
        self._slot.resize(n)
        object.__setattr__(self, "_len", n)
        return self._slot


class Publisher:
    """One topic over flux or ROS. On the flux path `slot_size`, `slot_count`, `max_slot_size` and
    `max_slot_count` are as flux.Publisher's, 0 being flux's default: with device="cuda" the slot
    holds the data field (4 MiB by default); with alloc="arena" on "cpu" the data field goes to the
    publisher's arena, whatever its size. `mirror_to_ros` also publishes each one as its ROS message,
    with `qos` as the ROS path would, while a ROS subscriber wants it. A type other than Image and PointCloud2 needs its flux adapter on the flux path.

    device="cuda" puts the root data field in GPU memory (Loan.alloc_buffer()), on a flux GPU
    channel that the C++ flux_wrapper shares. Where that cannot be had (a type without a root data,
    no flux GPU route, the ROS path) the publisher is "cpu", with a warning; uses_device says which.


    alloc="slot" keeps every variable-length field of a "cpu" message in its flux slot, where an
    integrated GPU subscriber reads it in place; a message past the slot is refused.
    alloc_on_subscriber, with alloc="slot" or device="cuda", gives the slots memory once a
    subscriber attaches; publish() is False until then. image_slots, image_encoding ("bgra8",
    "nv12", ...), image_width and image_height go to the flux publisher as they are; a format
    needs device="cuda".
    """

    def __init__(
        self,
        node,
        msg_type,
        topic,
        qos,
        *,
        slot_size=0,
        slot_count=0,
        max_slot_size=0,
        max_slot_count=0,
        mirror_to_ros=False,
        device="cpu",
        alloc="arena",
        alloc_on_subscriber=False,
        image_slots="auto",
        image_encoding="",
        image_width=0,
        image_height=0,
    ):
        self._type = msg_type
        self._flux = None
        self._ros = None
        self._logger = node.get_logger()
        self._fault_logged = False
        self._device_logged = False
        self._over_logged = False
        self._cuda = _cuda_usable(self._logger, topic, device, msg_type)
        if use_flux():
            if image_encoding and not self._cuda:
                # Asked of "cpu" it is a mistake; of a "cuda" that could not be had, the fallback
                # drops it.
                if device != "cuda":
                    raise ValueError("flux_wrapper: an image format needs device='cuda'")
                self._logger.warning(
                    f'{topic}: device="cuda" could not be had, so its image format is dropped'
                )
                image_encoding, image_width, image_height, image_slots = "", 0, 0, "auto"
            self._adapter = _adapter(msg_type)[0]
            fingerprint, aux_size = self._adapter.FINGERPRINT__, 0
            if self._cuda:
                fingerprint, aux_size = _gpu_fingerprint(msg_type), _GPU_AUX_BYTES
            self._flux = _flux.Publisher(
                node,
                topic,
                fingerprint=fingerprint,
                slot_size=slot_size,
                slot_count=slot_count,
                max_slot_size=max_slot_size,
                max_slot_count=max_slot_count,
                mirror_to_ros=mirror_to_ros,
                mirror_qos=qos,
                device="cuda" if self._cuda else "cpu",
                aux_size=aux_size,
                alloc=alloc,
                alloc_on_subscriber=alloc_on_subscriber,
                image_slots=image_slots,
                image_encoding=image_encoding,
                image_width=image_width,
                image_height=image_height,
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
            slot = self._flux.loan()
            b = self._adapter.Builder(None if slot is None else slot.aux)
            return Loan(self._type, builder=b, gpu=(slot, self._flux.device_backed))
        if self._flux is not None:
            return Loan(self._type, builder=self._adapter.build__(self._flux))
        return Loan(self._type, msg=self._type())

    def publish(self, msg):
        """Publish a Loan or a ROS message (the flux path copies it into a slot once). A
        PublishResult, false when nothing went out."""
        if isinstance(msg, Loan):
            loan = msg
            if loan._gpu:
                return self._commit_gpu(loan)
            if loan._builder is not None:
                if not loan._builder and loan._over is not None:
                    loan._builder.commit__()  # refused
                    self._overflow("", "slot", self._flux.slot_size, loan)
                    return PublishResult("too_large")
                return self._report(loan._builder.commit__())
            self._ros.publish(loan._msg)
            return _OK
        if self._cuda:
            return self._publish_gpu(msg)
        if self._flux is not None:
            return self._publish_host(msg)
        self._ros.publish(msg)
        return _OK

    def get_subscription_count(self):
        """The flux path counts subscribing processes."""
        if self._flux is not None:
            return self._flux.subscriber_count
        return self._ros.get_subscription_count()

    def _commit_gpu(self, loan):
        slot = loan._slot
        if not slot:
            return PublishResult("backpressure")
        # The slot is the subscribers' after the commit, and the slot of nobody after an abort.
        if loan._too_large or not loan._builder:
            slot.abort()
            if loan._over is not None:
                self._overflow(' on device="cuda"', "aux", _GPU_AUX_BYTES, loan)
            elif loan._too_large:
                self._report(_flux.Published.TOO_LARGE)  # data past the slot, as publish(msg)
            else:
                self._device_error(_GPU_OVER)
            return PublishResult("too_large")
        pitch = slot._image_pitch
        if pitch and loan._step != pitch:
            slot.abort()
            self._device_error(f"step {loan._step} is not the image slot's pitch {pitch}")
            return PublishResult("device_error")
        slot.aux_len = loan._builder.size__
        return self._report(slot.commit(nbytes=loan._len))

    def _publish_gpu(self, msg):
        loan = self.loan()
        loan.set_from(msg)
        data = _data_bytes(msg.data)
        # A false Loan, data past the slot among its causes, gives no buffer; _commit_gpu() logs
        # why.
        buf = loan.alloc_buffer(len(data))
        if buf is not None and not buf.copy_from(np.frombuffer(data, np.uint8)):
            buf.abort()
            self._device_error("the host to device copy failed")
            return PublishResult("device_error")
        return self._commit_gpu(loan)

    def _device_error(self, what):
        if not self._device_logged:
            self._device_logged = True
            self._logger.error(
                f'device="cuda" publish failed: {what}; later failures are not logged'
            )

    def _overflow(self, on, where, size, loan):
        """Once per publisher: which field of which message did not fit, to size the aux or slot
        by. Of fields set with set_from() only the size: they are written at once."""
        if self._over_logged:
            return
        self._over_logged = True
        field, nbytes, entries, used = loan._over
        sec, nanosec = loan._stamp
        stamp = f"; header.stamp {sec}.{nanosec:09d}" if _root_header(self._type) else ""
        if not field:
            what = (
                f"the fields set from it did not fit the {where} ({size} B, {used} B used before "
            )
            what += "them)"
        else:
            what = (
                f"{field} ({entries} entries, names {nbytes} B"
                if entries
                else f"{field} ({nbytes} B"
            )
            what += f") did not fit the {where} ({size} B, {used} B used before it)"
        self._logger.error(
            f"{self._flux.topic_name}: a message did not go out{on}: {what}{stamp}. "
            "Later ones are not logged"
        )

    def _publish_host(self, msg):
        """As a Loan filled from msg, so a field that does not fit is logged as one set by hand is."""
        loan = self.loan()
        if not loan:
            return PublishResult("backpressure")
        loan.set_from(msg)
        if _root_data(self._type):
            data = _data_bytes(msg.data)
            if loan and len(data):
                out = loan.alloc_data(len(data))
                if out is not None and len(out) == len(data):
                    out[:] = np.frombuffer(data, np.uint8)
        return self.publish(loan)

    def _report(self, p):
        if _flux.faulted(p) and not self._fault_logged:
            self._fault_logged = True
            self._logger.error(
                f"flux publish on {self._flux.topic_name} failed: {p} (slot "
                f"{self._flux.slot_size} bytes); later failures are not logged"
            )
        return PublishResult(p.name.lower())


class View:
    """The fields of a received message. On the flux path `data` and the View alias the slot and
    are valid only during the callback; copy out (to_msg()) what is kept longer.

    A field is read as an attribute of its ROS name. Image and PointCloud2 read each from the
    frame; any other type reads its fields from without_data(), made once per View, so that its
    root data stays in place.

    A device="cuda" subscription has `data` on the device too (on_device): device_data then has
    __dlpack__, __cuda_array_interface__ and nbytes, and stream is the stream to queue its work on.
    Of a device="cuda" publisher's frame it is a flux.Frame of uint8 and data is None; of a "cpu"
    publisher's, a flux.wire.DeviceSpan beside data in host memory."""

    def __init__(self, msg_type, msg=None, flux_view=None, frame=None, device=None, stream=None):
        self._type = msg_type
        self._msg = msg
        self._v = flux_view
        # A GPU channel's flux.Frame, on the host or the device, with its fields in _msg.
        self._frame = frame
        # A "cpu" publisher's data on the device, for a device="cuda" subscription.
        self._device = device
        self._stream = stream
        self._fields = None

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        if self._msg is not None:
            return getattr(self._msg, name)
        names = _SCALARS.get(self._type)
        if names is None:
            if self._fields is None:
                self._fields = self.without_data()
            return getattr(self._fields, name)
        if name in names:
            return getattr(self._v, name)
        if name == "header":
            sec, nanosec = self._v.header__stamp
            h = Header(frame_id=self._v.header__frame_id)
            h.stamp.sec, h.stamp.nanosec = sec, nanosec
            return h
        if name == "fields" and self._type is PointCloud2:
            return [
                PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                for f in self._v.fields
            ]
        raise AttributeError(name)

    @property
    def data(self):
        """The root data in place: a uint8 array. Of a type without a root data, its field data."""
        if not _root_data(self._type):
            return self.__getattr__("data")
        if self._frame is not None:
            return None if self.on_device else self._frame.array
        if self._msg is not None:
            return np.frombuffer(_data_bytes(self._msg.data), np.uint8)
        return self._v.data

    @property
    def on_device(self):
        if self._device is not None:
            return True
        return self._frame is not None and self._frame.on_device

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
            data = self._frame.copy_to_host() if self.on_device else self._frame.array
            m = _copy_but_data(self._msg)
            _set_data(m, data.tobytes())
            return m
        if self._msg is not None:
            return self._msg
        return _adapter(self._type)[1].frame_to_msg(self._v)

    def without_data(self):
        """As to_msg(), but data is left empty and not read, on either path: the other fields of a
        large message, whose data is read in place. Of a type without a root data, to_msg()."""
        if not _root_data(self._type):
            return self.to_msg()
        if self._msg is not None:
            return _copy_but_data(self._msg)
        return _adapter(self._type)[1].frame_to_msg_without_data(self._v)


class Subscription:
    """One topic over flux or ROS; the callback gets a View. Add it to a flux_wrapper.Executor.
    On the flux path it also takes the frames of a device="cuda" publisher of a type with a root
    data, whose fields come in the frame's aux bytes; a frame whose aux bytes do not parse is
    skipped and counted in rejected.

    device="cuda" has every frame's data on the device (View.on_device): a device="cuda"
    publisher's in its slot, a "cpu" publisher's as this subscription's copy beside its host bytes.
    A type without a root data is "cpu", with a warning.
    """

    def __init__(self, node, msg_type, topic, callback, qos, *, callback_group=None, device="cpu"):
        self.callback_group = callback_group
        self._flux = None
        self._gpu = None
        self._rejects = [0]
        logger = node.get_logger()
        cuda = _cuda_usable(logger, topic, device, msg_type)
        self._cuda = cuda
        if use_flux():
            adapter = _adapter(msg_type)[0]
            q = _flux.QoS(depth=_depth(qos))

            def on_typed(f):
                v = adapter.View(f)
                if f.on_device:
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
            if not _root_data(msg_type):
                return
            rejects = self._rejects

            def on_gpu(frame):
                aux = _flux.aux(frame)
                m = _unpack(msg_type, aux)
                if m is None:
                    rejects[0] += 1
                    if rejects[0] == 1:
                        logger.error(
                            f"{topic}: skipped a GPU frame with a bad aux header "
                            f"({len(aux)} bytes); later ones are counted in "
                            "rejected, not logged"
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
    def uses_device(self):
        """device="cuda" was asked for and is used."""
        return self._cuda

    @property
    def rejected(self):
        """Frames of a device="cuda" publisher skipped for a bad aux header; the first is logged."""
        return self._rejects[0]


def create_publisher(node, msg_type, topic, qos, **options):
    """As flux_wrapper::create_publisher<msg_type>, and create_subscription as its C++ twin."""
    return Publisher(node, msg_type, topic, qos, **options)


def create_subscription(node, msg_type, topic, qos, callback, **options):
    return Subscription(node, msg_type, topic, callback, qos, **options)


class Executor:
    """flux.Executor on the flux path, rclpy's SingleThreadedExecutor otherwise."""

    def __init__(self):
        # The rclpy executor is made by the first add_node(), so it takes that node's context.
        self._ex = _flux.Executor() if use_flux() else None

    def add(self, subscription):
        if subscription._flux is not None:
            self._ex.add(subscription._flux)
        if subscription._gpu is not None:
            self._ex.add(subscription._gpu)

    def add_node(self, node):
        if self._ex is None:
            self._ex = SingleThreadedExecutor(context=node.context)
        self._ex.add_node(node)

    def spin(self):
        self._ex.spin()

    def cancel(self):
        """Ends spin(), from any thread, as flux.Executor.cancel() and rclcpp's cancel()."""
        if use_flux():
            self._ex.cancel()
        else:
            self._ex.shutdown(timeout_sec=0)
