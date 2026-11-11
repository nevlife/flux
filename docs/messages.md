# 메시지 타입과 adapter

타입 없이도 배열(dtype, shape, 바이트)을 보낼 수 있다. ROS 메시지 구조를 그대로 쓰려면 `.msg`에서 adapter를 생성한다. adapter는 메시지를 슬롯 안에 평평한 바이트 배치로 쓰고, 그 배치에서 읽는다. 직렬화가 없고, 배열 필드는 슬롯을 그대로 가리킨다.

## adapter 만들기

메시지 패키지에서 `.msg`를 골라 `flux_generate_adapters()`에 넘긴다. 패키지의 일반 ROS interface 생성은 바뀌지 않는다.

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(flux_gen REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Cloud.msg" DEPENDENCIES std_msgs geometry_msgs)
flux_generate_adapters(msg/Cloud.msg)
```

```xml
<buildtool_depend>flux_gen</buildtool_depend>
<exec_depend>flux_py</exec_depend>  <!-- 생성된 Python 모듈이 flux를 import한다 -->
```

메시지마다 네 파일이 생긴다.

| 파일 | 이름 |
| --- | --- |
| `include/<pkg>/flux/<snake>.hpp` | `<pkg>::flux_msg::<Msg>` |
| `<pkg>_flux/<snake>.py` | `<pkg>_flux.<snake>.<Msg>`, `<pkg>_flux.<Msg>` |
| `include/<pkg>/flux/<snake>_ros.hpp` | ROS 메시지 객체와 오가는 `msg_to_frame()`, `frame_to_msg()` |
| `<pkg>_flux/<snake>_ros.py` | 같은 함수의 Python 판 |

- ROS 쪽 두 파일은 같은 패키지가 그 메시지를 `rosidl_generate_interfaces()`에 먼저 넘겼을 때만 생긴다. 이 두 파일만 rosidl 메시지 타입을 include·import한다.
- 생성된 C++ 헤더는 `flux/channel.hpp`를 include한다. 그러므로 adapter를 쓰는 쪽이 `flux_cpp`를 링크한다. 메시지 패키지 자신은 `flux_cpp`에 빌드 의존하지 않는다.
- 같은 패키지의 target이 adapter를 include하려면 `${PROJECT_NAME}_flux_adapters`를 링크한다. 다른 패키지에서는 메시지 패키지의 `${<pkg>_TARGETS}`를 링크하면 된다. adapter 헤더가 rosidl 헤더와 같은 include 경로에 설치되기 때문이다.
- fingerprint에는 중첩된 타입을 펼친 모양까지 반영된다. 중첩된 `.msg`를 고치면 그것을 쓰는 adapter가 다시 생성된다. 발행 쪽과 구독 쪽 adapter가 서로 다른 `.msg`에서 생성되면 fingerprint가 달라 서로 다른 채널이 된다. 이때 오류는 나지 않는다.

## 표준 메시지

`flux_std_adapters`는 설치된 `.msg`에서 아래 타입의 adapter를 한 벌 만든다. 이 타입들의 adapter는 각 패키지에서 따로 생성하지 않는다. 같은 타입을 여러 패키지가 생성하면 C++ 정의가 둘이 되고, Python 모듈은 경로에서 앞선 하나만 보인다.

| 패키지 | 타입 |
| --- | --- |
| `sensor_msgs` | `CameraInfo`, `CompressedImage`, `Image`, `Imu`, `LaserScan`, `PointCloud2` |
| `stereo_msgs` | `DisparityImage` |
| `nav_msgs` | `OccupancyGrid`, `Odometry` |

```cpp
#include "sensor_msgs/flux/image.hpp"      // sensor_msgs::flux_msg::Image
#include "sensor_msgs/flux/image_ros.hpp"  // msg_to_frame, frame_to_msg
```

```python
from sensor_msgs_flux.image import Image
from sensor_msgs_flux.image_ros import frame_to_msg, msg_to_frame
```

## adapter가 주는 것

| C++ | Python | 뜻 |
| --- | --- | --- |
| `kFingerprint` | `FINGERPRINT__` | Publisher·Subscription에 넘기는 스키마 지문 |
| `kTypeName` | `TYPE_NAME__` | `pkg/Name` |
| `View` | `View` | 받은 프레임을 복사 없이 읽는다 |
| `Builder` | `Builder` | 빌린 슬롯에 복사 없이 쓴다 |
| `build__(pub)` | `build__(pub)` | 슬롯을 빌려 `Builder`를 돌려준다 |
| `.msg`의 상수 | `.msg`의 상수 | 같은 이름의 상수 |

사용 예는 [cpp.md](cpp.md#메시지-타입-붙이기)와 [python.md](python.md#메시지-타입-붙이기)에 있다.

## 접근자 규칙

아래 표는 `.msg`가 다음과 같을 때의 접근자다.

```text
std_msgs/Header header
uint32 width
float32[] x
float64[9] m
string label
string[] tags
geometry_msgs/Point[] pts
geometry_msgs/Pose pose
```

| `.msg` | C++ View | Python View | C++ Builder | Python Builder |
| --- | --- | --- | --- | --- |
| `uint32 width` | `v.width()` | `v.width` | `b.set__width(n)` | `b.width = n` |
| `float32[] x` | `v.x()` -> `Span<const float>` | `v.x` -> ndarray | `b.alloc__x(n)` -> `Span<float>` | `b.alloc__x(n)` -> ndarray |
| `float64[9] m` | `v.m()` -> `Span` | `v.m` -> ndarray | `b.m()` -> `Span` | `b.m` -> ndarray |
| `string label` | `v.label()` -> `string_view` | `v.label` -> `str` | `b.set__label(s)` | `b.label = s` |
| `string[] tags` | `v.tags__size()`, `v.tags(i)` | `v.tags` -> list | `b.alloc__tags(n)`, `b.set__tags(i, s)` | `b.tags = [...]` |
| `Point[] pts` | `v.pts__size()`, `v.pts(i).x()` | `v.pts[i].x` | `b.alloc__pts(n)[i].set__x(v)` | `b.alloc__pts(n)[i].x = v` |
| `Pose pose` | `v.pose_position_x()` | `v.pose_position_x` | `b.set__pose_position_x(v)` | `b.pose_position_x = v` |
| `Header header` | `v.header__sec()`, `v.header__nanosec()`, `v.header__frame_id()` | `v.header__stamp`(튜플), `v.header__frame_id` | `b.set__header__stamp(s, ns)`, `b.set__header__frame_id(f)` | `b.set__header__stamp(s, ns)`, `b.header__frame_id = f` |

- 중첩된 메시지는 이름을 `_`로 이어 펼친다. `pose.position.x`는 `pose_position_x`다. 최상위의 `std_msgs/Header`와 `builtin_interfaces/Time` 필드는 `__`로 잇는다(`header__sec`, `stamp__sec`, Python `stamp__stamp`). 더 깊은 곳에서는 다른 메시지처럼 펼친다.
- 가변 배열은 `alloc__<name>(n)`으로 슬롯 안에, 슬롯에 들어가지 않으면 arena에 자리를 잡는다([concepts.md](concepts.md#arena)). 이 함수는 잡은 자리를 돌려주고, 거기에 바로 쓰면 복사가 없다. 호출 순서는 상관없다. 고정 길이 배열은 자리가 이미 정해져 있으므로 `alloc__` 없이 바로 쓴다.
- 슬롯과 arena 모두 자리가 모자라면(arena를 껐거나 찼다) C++ `alloc__`은 빈 `Span`을 준다. 그때부터 `Builder`는 `false`이고, `commit__()`은 발행하지 않고 `TooLarge`를 돌려준다.
- Python 접근자와 `frame_to_msg()`는 문자열을 첫 NUL에서 끊는다. rclpy도 같다. C++ `View` 접근자는 쓰인 바이트를 그대로 준다.
- C++ 이름이 C++ 키워드와 겹치면 끝에 `_`를 붙인다. Python은 Python 키워드만 피한다. 메시지 이름 `View`, `Builder`는 쓸 수 없다.

## 받는 쪽의 검사

프레임은 다른 프로세스가 쓴 것이므로 받는 쪽에서 검사한다. 길이나 위치를 담은 칸이 프레임 밖을 가리키면 읽지 않는다.

- C++ `View`는 잘못된 접근을 만나면 `ok__()`를 `false`로 고정하고, 그 뒤 접근자는 빈 값을 준다. 읽은 다음에 `ok__()`를 확인한다.
- Python `View`는 `flux.wire.WireError`를 던진다.
- `frame_to_msg()`는 C++에서 `std::runtime_error`, Python에서 `WireError`를 던진다. 고정 길이 배열의 길이가 틀리면 C++은 `std::length_error`, Python은 `ValueError`다.

## 생성하지 못하는 메시지

아래 모양의 필드가 있는 메시지는 adapter 생성이 실패하고 빌드가 멈춘다. 이런 메시지는 일반 ROS 토픽으로 보낸다.

| 모양 | 이유 |
| --- | --- |
| `bool[]` | 가변 bool 배열은 지원하지 않는다. `uint8[]`을 쓴다. 고정 길이 `bool[N]`은 된다 |
| `T[<=N]` | 상한 있는 배열은 지원하지 않는다. `T[]`를 쓴다. `string<=N`은 상한을 무시하고 받는다 |
| `wstring` | 지원하지 않는다 |
| `@optional` 필드 | 지원하지 않는다 |
| `T[N]`에서 N > 65536 | `T[]`를 쓴다 |
| 자기 자신을 포함하는 타입, 필드가 없는 중첩 타입 | 펼칠 수 없다 |

프레임 하나는 4 GiB를 넘지 못한다. 길이와 위치를 32비트로 적기 때문이다. 슬롯 안의 부분은 발행자의 `slot_size`, arena에 둔 부분은 arena 크기(최대 1 GiB)가 상한이다.

## 라이브러리가 만든 버퍼 보내기

라이브러리가 돌려준 버퍼(디코더 출력, `std::vector`)는 보통 슬롯으로 한 번 복사한다. 그 버퍼를 arena에서 만들게 하면 복사하지 않고 필드로 넘길 수 있다.

```cpp
std::vector<std::uint8_t> pixels;
{
  flux::ArenaScope scope;  // 이 스레드의 할당이 arena로 간다
  pixels = decode(jpeg);
}
auto b = sensor_msgs::flux_msg::Image::build__(pub);
b.adopt__data(std::move(pixels));  // arena에 있으면 복사 없이 필드가 된다
```

```python
with flux.arena_scope():
    pixels = decode(jpeg)  # numpy 배열
b = Image.build__(pub)
b.adopt__data(pixels)
```

- 프로세스를 `LD_PRELOAD=libflux_heaphook.so`로 띄워야 한다. 라이브러리는 flux_ffi와 같은 `lib`에 설치된다. 없으면 `ArenaScope::open()`과 `arena_scope()`가 `false`이고 아무것도 바뀌지 않는다.
- scope 안에서는 그 스레드의 모든 할당(`malloc`, `new`, numpy 배열)이 arena로 간다. 버퍼를 만드는 호출만 감싼다. arena가 차면 그 할당은 일반 힙으로 간다.
- `adopt__<name>`은 기본 타입 가변 배열 필드에 있다. 버퍼가 arena에 없으면 복사하고 `false`를 돌려준다. Python은 C-contiguous이고 원소 크기가 필드와 같은 numpy 배열만 받는다.
- 넘긴 버퍼는 프레임이 가진다. 그 슬롯을 다시 쓸 때 C++ 컨테이너를 지우고 Python 배열의 참조를 놓는다.
- preload한 프로세스의 모든 `malloc`에 검사 하나가 더해진다.

## tensor 메시지

DLPack `DLTensor`의 필드만 가진 메시지는 tensor 메시지로 다룬다. `tensor_msgs/ExperimentalTensor`(Lyrical)가 그 모양이다. 패키지 이름은 보지 않으므로 같은 필드를 복사한 메시지도 tensor 메시지가 된다.

```text
uint8 dtype_code
uint8 dtype_bits
uint16 dtype_lanes
int64[] shape
int64[] strides
uint64 byte_offset
uint8[] data
```

| 언어 | View | Builder |
| --- | --- | --- |
| Python | `as_tensor__()`는 numpy view다. numpy에 없는 dtype(bf16, fp8)은 `flux.wire.Tensor`(`.bits`, `__dlpack__`)다. View 자체도 `__dlpack__`을 낸다 | `alloc_tensor__(dtype, shape)`. dtype은 numpy dtype이나 DLPack 이름(`"bfloat16"`, `"float8_e4m3fn"` 등)이다 |
| C++ | `as_tensor__()`는 `flux::wire::Tensor`(첫 원소 포인터, `DlType`, shape, strides). `as<T>()`는 dtype이 `T`일 때만 포인터를 준다 | `alloc_tensor__(DlType, shape)`는 `void *`, `alloc_tensor__<T>(shape)`는 `Span<T>` |

- strides는 원소 단위이고 비어 있으면 C-contiguous다. 범위 안이면 음수 stride도 받는다.
- 모든 원소가 `data` 안에 있어야 하고 첫 원소가 원소 크기에 맞게 정렬돼 있어야 한다. 어기면 Python은 `WireError`, C++은 빈 결과와 `ok__() == false`다.
- 받는 dtype은 정수·부호 없는 정수 8~64비트, float 16·32·64, complex 64·128, bool 8, bfloat16, float8 계열이다.
- `Builder`는 `data`를 64바이트 경계에 둔다.
- GPU 구독(`device="cuda"`)의 `Frame`을 View에 넘기면 `__dlpack__`이 장치 tensor를 낸다([gpu.md](gpu.md)).
