# Python 사용법

`flux_py`는 rclpy 노드에서 사용하는 Python 모듈 `flux`다. 받은 프레임은 공유 메모리를 가리키는 읽기 전용 numpy 배열이다. 채널, 슬롯, 전달 규칙은 [concepts.md](concepts.md)에 있다.

## 패키지 설정

```xml
<exec_depend>flux_py</exec_depend>
<exec_depend>flux_std_adapters</exec_depend>  <!-- sensor_msgs_flux 같은 표준 메시지 어댑터를 사용하는 경우 -->
```

이 문서의 예제가 사용하는 import다.

```python
import threading

import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import flux
from sensor_msgs_flux.image import Image  # flux_std_adapters의 어댑터
```

생성자의 첫 인자는 rclpy 노드다. 토픽 이름은 `node.resolve_topic_name()`으로 해석한다.

## 발행

```python doc:py_publish
pub = flux.Publisher(node, "image")

pixels = np.zeros((480, 640, 3), np.uint8)
p = pub.publish(pixels)
if flux.faulted(p):
    node.get_logger().error(f"publish: {p}")
```

```python
flux.Publisher(node, topic, fingerprint=flux.NO_SCHEMA, slot_size=0, slot_count=16,
               memory=None, max_subscribers=16, device="cpu", fence_timeout=None,
               mirror_to_ros=False, aux_size=0)
```

- 기본값은 C++와 같고, 모듈 상수 `flux.DEFAULT_SLOT_SIZE`, `flux.DEFAULT_SLOT_COUNT`, `flux.DEFAULT_MAX_SUBSCRIBERS`, `flux.UNTYPED_SLOT_SIZE`, `flux.SLOT_STRING_BYTES`로도 읽기 가능하다.
- `publish(array)`는 numpy 배열이나 `bytes`를 dtype, shape와 함께 슬롯에 한 번 복사한다.
- 같은 채널에 실행 중인 발행자가 있으면 생성자가 `flux.Error`(`status`는 `Status.PUBLISHER_EXISTS`)를 던진다.
- `dropped`, `slot_size`, `topic_name`, `signpost_name`, `subscriber_count`는 속성이다. `subscriber_count`의 뜻은 C++와 같다([cpp.md](cpp.md#발행)).

`publish()`가 받는 배열은 다음과 같다.

| 배열 | 결과 |
| --- | --- |
| dtype `uint8`~`uint64`, `int8`~`int64`, `float16`, `float32`, `float64` | 발행 |
| 그 밖의 dtype(`bool`, `complex64` 등) | `TypeError` |
| 8차원 초과 | `Published.TOO_LARGE` 반환 |
| GPU 배열 | `TypeError`. loan을 사용한다([gpu.md](gpu.md)) |
| `bfloat16` | `loan(shape, "bfloat16")`로 loan한다. ml_dtypes가 필요하다 |

`publish()`의 결과는 `flux.Published`다. `bool(p)`는 `OK`인 경우에만 참이고, `flux.faulted(p)`는 `OK`도 `BACKPRESSURE`도 아닌 경우에 참이다. 값의 뜻은 C++와 같다([cpp.md](cpp.md#발행)).

## loan으로 슬롯에 직접 쓰기

```python doc:py_loan
loan = pub.loan((1024, 3), "float32")
if loan is not None:
    loan.array[:] = 0.0
    loan.commit()
```

- `loan(shape=None, dtype="uint8")`은 빈 슬롯이 없으면 `None`을 반환한다. shape를 생략하면 슬롯 전체를 `uint8`로 loan한다. 슬롯보다 큰 shape는 `ValueError`다.
- `loan.array`는 슬롯을 가리키는 쓰기 가능한 numpy 배열이고, commit이나 abort 후에는 읽기 전용이 된다.
- `commit(nbytes=None)`은 `flux.Published`를 반환한다. 1차원 loan은 `nbytes`로 줄여 발행 가능하다. 실패한 commit(`TOO_LARGE`)도 loan을 끝낸다.
- commit하지 않은 `Loan`이 사라지거나 `abort()`를 호출하면 프레임은 발행되지 않는다.
- `capacity`는 loan한 크기(bytes)다.

발행자를 `aux_size`와 함께 생성하면 aux를 사용 가능하다.

- `loan.aux`는 슬롯의 aux 영역을 가리키는 쓰기 가능한 `uint8` 배열이고, `loan.aux_len`은 프레임이 쓴 길이다. commit 후에는 읽기 전용이 된다.
- 구독 측은 take가 반환한 배열이나 `Frame`을 `flux.aux(frame)`에 넘겨 `aux_len` bytes의 읽기 전용 배열로 읽는다.

## 구독과 executor

```python doc:py_subscribe
def on_frame(frame):
    print(frame.shape, frame.dtype)

sub = flux.Subscription(node, "image", callback=on_frame)

ex = flux.Executor()
ex.add(sub)
ex.add_node(node)
ex.spin()
```

```python
flux.Subscription(node, topic, callback=None, fingerprint=flux.NO_SCHEMA, qos=None, memory=None,
                  device="cpu", fence_timeout=None, release="wait")
```

- 콜백은 읽기 전용 numpy 배열 하나를 받는다. `bfloat16` 프레임은 bit가 같은 `uint16` 배열로 읽힌다.
- 배열(과 그 slice, view)이 남아 있는 동안 구독은 그 슬롯을 점유한다. 콜백 밖에서 쓸 값은 `np.array(frame)`처럼 복사한다. 동시에 보유 가능한 배열 수는 QoS `max_borrow`(기본 1)다.
- QoS에 거절되는 값이 있으면 생성자가 `flux.Error`를 던진다.

### flux.Executor

`flux.Executor`는 flux 구독과 rclpy 노드를 함께 실행한다. 모든 콜백은 `spin()`을 호출한 스레드에서 실행된다.

| 멤버 | 뜻 |
| --- | --- |
| `Executor(rclpy_executor=None)` | 노드를 실행할 rclpy executor를 지정한다. 생략하면 첫 `add_node()`가 `SingleThreadedExecutor`를 생성한다 |
| `add(source, priority=0)` | 콜백이 있는 `Subscription`이나 `flux.message_filters.Subscriber`를 추가한다. 콜백이 없으면 `ValueError` |
| `add_node(node)` | rclpy 노드를 추가한다 |
| `spin()` | `stop()`, 콜백 예외, rclpy context 종료 중 하나가 발생할 시점까지 실행한다 |
| `stop()` | 어느 스레드, 콜백, signal handler에서든 spin을 끝낸다. spin 전에 호출하면 다음 `spin()`이 바로 끝난다 |

- flux 구독만 있으면 executor는 rclpy executor 없이 실행된다. 대기 중에는 GIL을 해제하고, 콜백 호출 시와 100 ms 주기마다 GIL을 획득한다.
- 노드가 있으면 helper 스레드가 GIL 없이 프레임을 기다렸다가 rclpy executor에 task를 넣는다.
- rclpy executor로는 `SingleThreadedExecutor`와 `EventsExecutor`가 사용 가능하다. `MultiThreadedExecutor`는 사용하지 않는다. 바쁜 구독이 있으면 다른 스레드가 넣은 task를 늦게 실행하거나 놓치기 때문이다(flux 없이도 재현된다).
- flux 처리가 한 번에 1 ms를 넘으면 ROS 콜백에 차례를 넘긴다. 우선순위 규칙은 C++와 같다(값이 큰 구독이 먼저, 같으면 한 프레임씩 번갈아).
- 연결 시 거절된 구독은 로그를 한 번 남기고, 발행자 재시작 시까지 executor에서 제외된다. 나머지 구독은 계속 실행된다.

## executor 없이 읽기

```python doc:py_take
sub = flux.Subscription(node, "image")
```

```python doc:py_take_frames
while (frame := sub.take()) is not None:
    print("frame of", frame.nbytes, "bytes")
    del frame
frame = sub.take_blocking(100_000_000)
```

- `take()`는 아직 받지 않은 가장 오래된 프레임을 반환하고, 없으면 `None`이다.
- 루프의 `del frame`은 필수다. 변수가 앞 프레임을 점유한 채로 다음 `take()`를 호출하면 `max_borrow`(기본 1)에 걸려 `None`이 나온다.
- executor가 처리하는 구독의 `take()`는 `RuntimeError`를 던진다.

`take_blocking(timeout_ns=-1)`은 프레임 도착 시까지 GIL을 해제하고 기다린다. 발행자가 아직 없으면 발행자도 기다린다.

| 경우 | 결과 |
| --- | --- |
| 음수 timeout | 끝없이 기다린다 |
| timeout 경과, 노드 context 종료 | `None` |
| view를 `max_borrow`만큼 점유한 상태 | `None` |
| 대기 중 Python signal handler가 예외를 낸 경우(Ctrl-C의 `KeyboardInterrupt` 등) | 대기를 멈추고 그 예외를 낸다 |

구독의 상태 속성은 어느 스레드에서든 읽기 가능하다.

| 속성 | 뜻 |
| --- | --- |
| `attached` | 발행자의 세그먼트에 연결된 상태 |
| `lost` | 받지 못한 프레임 수. 재시작과 관계없이 누적한다 |
| `refused` | 빈 결과의 이유별 수(`flux.Refused`, [qos.md](qos.md)) |
| `attach_generation` | 재연결마다 바뀌는 번호 |
| `qos`, `topic_name`, `signpost_name` | 생성 시 정한 값 |

## QoS

```python doc:py_qos
qos = flux.QoS(depth=4, replay=2, max_borrow=2)
sub = flux.Subscription(node, "cmd", qos=qos)
```

생략한 필드는 기본값이다. 값의 뜻은 [qos.md](qos.md)에 있다.

## 생성 메시지 사용

`.msg`에서 생성한 어댑터 모듈은 `FINGERPRINT__`, `View`, `Builder`를 제공한다. 생성 방법과 접근자 규칙은 [messages.md](messages.md)에 있다.

```python doc:py_adapter_publish
pub = flux.Publisher(node, "image", fingerprint=Image.FINGERPRINT__)

b = Image.build__(pub)
if b:
    stamp = node.get_clock().now().to_msg()
    b.set__header__stamp(stamp.sec, stamp.nanosec)
    b.header__frame_id = "camera"
    b.height = 480
    b.width = 640
    b.encoding = "rgb8"
    b.step = 640 * 3
    b.alloc__data(480 * 640 * 3)[:] = 0
    if flux.faulted(b.commit__()):
        node.get_logger().error("image not published")
```

`build__(pub)`가 슬롯을 loan하고, `Builder`가 필드를 그 슬롯에 직접 쓴다. 빈 슬롯이 없으면 `Builder`는 거짓이고, 필드에 쓴 값은 버려지며, `commit__()`은 `BACKPRESSURE`를 반환한다.

받은 프레임은 `View`로 읽는다.

```python doc:py_adapter_read
img = Image.View(frame)
print(img.width, img.height, img.encoding, img.data.shape)
```

배열 필드(`img.data`)는 슬롯을 가리키는 읽기 전용 numpy 배열이다. 프레임이 스키마와 맞지 않으면 접근자가 `flux.wire.WireError`를 던진다.

ROS 메시지 객체가 이미 있으면 `<pkg>_flux.<name>_ros` 모듈의 함수로 변환한다. 필드 전체를 복사한다.

```python
from sensor_msgs_flux.image_ros import frame_to_msg, msg_to_frame
```

```python doc:py_ros_bridge_publish
b = Image.build__(pub)
msg_to_frame(msg, b)
if flux.faulted(b.commit__()):
    node.get_logger().error("image not published")
```

```python doc:py_ros_bridge_read
copy = frame_to_msg(Image.View(frame))
```

ROS 2 노드도 같은 토픽을 구독해야 하면 발행자를 `mirror_to_ros=True`로 생성해 같은 이름의 ROS 2 토픽에도 발행한다([ros2_tools.md](ros2_tools.md#발행자의-ros-2-mirror)).

## PartitionedExecutor

`flux.PartitionedExecutor`는 flux 구독을 넣은 콜백 그룹마다 스레드 하나, 노드마다 rclpy 스레드 하나를 생성한다.

```python doc:py_partitioned
def on_frame(frame):
    print(threading.current_thread().name, frame.nbytes, "bytes")

def name_thread():
    threading.current_thread().name = "camera"

camera = MutuallyExclusiveCallbackGroup()
sub = flux.Subscription(node, "image", callback=on_frame)

ex = flux.PartitionedExecutor()
ex.add(sub, camera)
ex.add_node(node)
ex.on_thread_start(camera, name_thread)
ex.spin()
```

| 멤버 | 뜻 |
| --- | --- |
| `add(source, group, priority=0)` | 구독을 그룹에 추가한다. Reentrant 그룹은 `ValueError`다 |
| `add_node(node)` | 노드의 ROS 엔티티를 그 노드의 스레드에서 실행한다 |
| `on_thread_start(unit, init)` | 그룹이나 노드의 스레드가 실행되기 전에 `init()`을 그 스레드에서 호출한다 |
| `add_sync_group(*inputs)` | 한 synchronizer의 입력들을 알린다. 입력은 모두 flux 입력이고 한 그룹에 있어야 한다 |
| `spin()` | 어느 스레드에서든 첫 오류가 발생하면 모든 스레드를 멈추고 그 오류를 여기서 낸다 |
| `stop()` | 어느 스레드에서든 spin을 끝낸다 |

- flux 구독을 넣은 그룹에 ROS 엔티티(구독, 타이머, 서비스 등)가 있으면 `spin()`이 거절한다. rclpy executor에는 엔티티를 그룹 단위로 넣는 것이 불가능해서, 그 엔티티가 노드의 스레드에서 실행되어 그룹의 상호 배제가 깨지기 때문이다.
- flux 입력과 ROS 입력이 섞인 synchronizer는 `flux.Executor`로 실행한다.

## message_filters

```python
import message_filters
from flux.message_filters import Subscriber
```

```python doc:py_message_filters
def on_pair(left, right):
    print("pair at", left.header.stamp.sec, left.view().width, right.view().width)

left = Subscriber(node, Image, "left/image", 10)
right = Subscriber(node, Image, "right/image", 10)
sync = message_filters.ApproximateTimeSynchronizer([left, right], 10, 0.01)
sync.registerCallback(on_pair)

ex = flux.Executor()
ex.add(left)
ex.add(right)
ex.spin()
```

- `Subscriber(node, adapter, topic, queue_size, qos=None)`의 `queue_size`는 synchronizer나 Cache의 큐 크기다.
- 큐에는 `StampedFrame`이 들어간다. `StampedFrame`은 `frame`(슬롯을 점유한 배열), `header`(stamp, `frame_id`), `view()`(어댑터의 `View`)를 가진다.
- `Subscriber`는 `max_borrow`를 `queue_size + 1` 이상으로 올리고, 슬롯이 `queue_size`개 이하인 발행자는 거절한다. 거절은 생성자에서 연결하는 경우 `flux.Error`이고, executor 아래에서는 로그를 한 번 남기고, 발행자 재시작 시까지 그 입력을 제외한다.
- 어댑터에 `std_msgs/Header header`가 없으면 생성자가 `TypeError`를 낸다.
- header를 읽지 못하거나 stamp가 음수인 프레임은 넘기지 않고 `unreadable`로 센다. `forwarded`는 넘긴 프레임 수다.

## 오류

| 예외 | 발생 경우 |
| --- | --- |
| `flux.Error`(`RuntimeError`) | 채널이 거절했다. `status` 속성이 `flux.Status` 값이다. 값의 뜻은 C++와 같다([cpp.md](cpp.md#오류)) |
| `ValueError` | 잘못된 인자(슬롯보다 큰 loan, `device`, `fence_timeout`, `release` 값 등) |
| `TypeError` | 받지 않는 dtype, 콜백이 아닌 값 |
| `RuntimeError` | 두 스레드가 한 구독을 동시에 사용했다. fork로 상속받은 구독을 사용했다 |
| `flux.wire.WireError` | 어댑터의 `View`가 스키마와 맞지 않는 프레임을 읽었다 |

## 채널 목록

```python doc:py_enumerate
for t in flux.enumerate_topics():
    s = flux.read_channel_stats(t.signpost)
    print(t.key, "up" if s.live else "down", len(t.endpoints), "endpoints")
```

`flux.ros_type_of(fingerprint)`는 그 fingerprint에 해당하는 설치된 ROS 메시지 타입의 이름(예: `sensor_msgs/msg/Image`)을 반환하고, 없으면 `None`이다.
