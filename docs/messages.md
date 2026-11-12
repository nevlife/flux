# 메시지 타입과 어댑터

타입 없이 배열(dtype, shape, bytes)을 보내는 것도 가능하다. ROS 메시지 구조를 사용하려면 `.msg`에서 어댑터를 생성한다. 어댑터는 메시지를 슬롯 안에 평평한 배치로 쓰고 읽는다. 직렬화가 없고, 배열 필드는 슬롯을 직접 가리킨다.

## 어댑터 만들기

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

메시지마다 파일 네 개가 생긴다.

| 파일 | 이름 |
| --- | --- |
| `include/<pkg>/flux/<snake>.hpp` | `<pkg>::flux_msg::<Msg>` |
| `<pkg>_flux/<snake>.py` | `<pkg>_flux.<snake>.<Msg>`, `<pkg>_flux.<Msg>` |
| `include/<pkg>/flux/<snake>_ros.hpp` | ROS 메시지 객체와 변환하는 `msg_to_frame()`, `frame_to_msg()` |
| `<pkg>_flux/<snake>_ros.py` | 같은 함수의 Python 판 |

- ROS 쪽 두 파일은 같은 패키지가 그 메시지를 `rosidl_generate_interfaces()`에 먼저 넘긴 경우에만 생긴다. rosidl 메시지 타입을 include하거나 import하는 것은 이 두 파일뿐이다.
- 생성된 C++ 헤더는 `flux/channel.hpp`를 include하므로, 어댑터를 사용하는 쪽이 `flux_cpp`를 링크한다. 메시지 패키지 자신은 `flux_cpp`에 빌드 의존하지 않는다.
- 같은 패키지의 target이 어댑터를 include하려면 `${PROJECT_NAME}_flux_adapters`를 링크한다. 다른 패키지에서는 메시지 패키지의 `${<pkg>_TARGETS}`를 링크한다. 어댑터 헤더가 rosidl 헤더와 같은 include 경로에 설치되기 때문이다.
- fingerprint에는 중첩된 타입을 펼친 모양까지 반영되고, 중첩된 `.msg`를 고치면 그것을 사용하는 어댑터가 다시 생성된다.
- 발행 쪽과 구독 쪽 어댑터가 다른 `.msg`에서 생성되면 fingerprint가 달라 다른 채널이 된다. 오류는 발생하지 않는다.

## 표준 메시지

`flux_std_adapters`는 설치된 `.msg`에서 아래 타입의 어댑터를 한 벌 생성하므로, 각 패키지에서 따로 생성하지 않는다. 같은 타입을 여러 패키지가 생성하면 C++ 정의가 둘이 되고, Python 모듈은 경로에서 앞선 하나만 보인다.

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

## 어댑터가 제공하는 것

| C++ | Python | 뜻 |
| --- | --- | --- |
| `kFingerprint` | `FINGERPRINT__` | Publisher와 Subscription에 넘기는 fingerprint |
| `kTypeName` | `TYPE_NAME__` | `pkg/Name` |
| `View` | `View` | 받은 프레임을 복사 없이 읽는다 |
| `Builder` | `Builder` | loan한 슬롯에 복사 없이 쓴다 |
| `build__(pub)` | `build__(pub)` | 슬롯을 loan해 `Builder`를 반환한다 |
| `.msg`의 상수 | `.msg`의 상수 | 같은 이름의 상수 |

사용 예는 [cpp.md](cpp.md#생성-메시지-사용)와 [python.md](python.md#생성-메시지-사용)에 있다.

## 접근자 규칙

아래 표는 `.msg`가 다음과 같은 경우의 접근자다.

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
- 가변 배열은 `alloc__<name>(n)`이 슬롯 안에, 들어가지 않으면 arena에 할당한다([concepts.md](concepts.md#arena)). 반환한 자리에 바로 쓰면 복사가 없다. 호출 순서는 상관없다.
- 고정 길이 배열은 `alloc__` 없이 바로 쓴다.
- 숫자 가변 배열은 `alloc_device__<name>(n)`으로 GPU에서 쓰고 `<name>__device`(C++는 `<name>__device()`)로 GPU에서 읽는 것도 가능하다([gpu.md](gpu.md#생성-메시지의-gpu-필드)).
- 슬롯과 arena 모두 자리가 모자라면(arena가 비활성화되었거나 가득 찬 경우) C++ `alloc__`은 빈 `Span`을 반환하고, 그 뒤 `Builder`는 `false`, `commit__()`은 발행 없이 `TooLarge`를 반환한다.
- Python 접근자와 `frame_to_msg()`는 문자열을 첫 NUL에서 자른다. rclpy도 같다. C++ `View` 접근자는 쓰인 bytes를 그대로 반환한다.
- C++ 이름이 C++ 키워드와 겹치면 끝에 `_`를 추가한다. Python은 Python 키워드만 피한다. 메시지 이름 `View`, `Builder`는 사용 불가능하다.

## 받는 쪽의 검사

받는 쪽은 다른 프로세스가 쓴 프레임을 검사하고, 길이나 위치를 담은 칸이 프레임 밖을 가리키면 읽지 않는다.

| 대상 | 잘못된 프레임을 만난 경우 |
| --- | --- |
| C++ `View` | `ok__()`를 `false`로 고정하고, 그 뒤 접근자는 빈 값을 반환한다. 읽은 뒤 `ok__()`를 확인한다 |
| Python `View` | `flux.wire.WireError`를 던진다 |
| `frame_to_msg()` | C++는 `std::runtime_error`, Python은 `WireError`를 던진다. 고정 길이 배열의 길이가 틀리면 C++는 `std::length_error`, Python은 `ValueError`다 |

## 생성 불가능한 메시지

아래 모양의 필드가 있는 메시지는 어댑터 생성이 실패해 빌드가 멈춘다. 이런 메시지는 일반 ROS 토픽으로 보낸다.

| 모양 | 이유 |
| --- | --- |
| `bool[]` | 가변 bool 배열은 지원하지 않는다. `uint8[]`을 사용한다. 고정 길이 `bool[N]`은 가능하다 |
| `T[<=N]` | 상한 있는 배열은 지원하지 않는다. `T[]`를 사용한다. `string<=N`은 상한을 무시하고 받는다 |
| `wstring` | 지원하지 않는다 |
| `@optional` 필드 | 지원하지 않는다 |
| `T[N]`에서 N > 65536 | `T[]`를 사용한다 |
| 자기 자신을 포함하는 타입, 필드가 없는 중첩 타입 | 펼치기가 불가능하다 |

프레임 하나의 최대 크기는 4 GiB다. 길이와 위치를 32-bit로 적기 때문이다. 슬롯 안의 부분은 발행자의 `slot_size`가, arena에 저장한 부분은 arena 크기(최대 16 GiB)가 상한이다.

## 라이브러리가 만든 버퍼 보내기

라이브러리가 반환한 버퍼(디코더 출력, `std::vector`)는 보통 슬롯으로 한 번 복사한다. 그 버퍼를 arena에서 만들게 하면 복사 없이 필드로 넘긴다.

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

- 프로세스를 `LD_PRELOAD=libflux_heaphook.so`로 시작해야 한다(flux_ffi와 같은 `lib`에 설치된다). 없으면 `ArenaScope::open()`과 `arena_scope()`가 `false`이고 아무것도 바뀌지 않는다.
- scope 안에서는 그 스레드의 모든 할당(`malloc`, `new`, numpy 배열)이 arena로 가므로, 버퍼를 만드는 호출만 감싼다. arena가 가득 차면 일반 힙으로 간다.
- `adopt__<name>`은 기본 타입 가변 배열 필드에 있다. 버퍼가 arena에 없으면 복사하고 `false`를 반환한다. Python은 C-contiguous이고 원소 크기가 필드와 같은 numpy 배열만 받는다.
- 넘긴 버퍼는 프레임이 소유한다. 그 슬롯에 다음 프레임을 쓰는 시점에 C++ 컨테이너를 지우고 Python 배열의 참조를 해제한다.
- preload한 프로세스는 모든 `malloc`에 검사가 하나 더해진다.

## tensor 메시지

DLPack `DLTensor`의 필드만 가진 메시지는 tensor 메시지로 다룬다(`tensor_msgs/ExperimentalTensor`, Lyrical). 패키지 이름은 보지 않으므로 같은 필드를 가진 메시지도 tensor 메시지가 된다.

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
| Python | `as_tensor__()`는 numpy view다. numpy에 없는 dtype(bf16, fp8)은 `flux.wire.Tensor`(`.bits`, `__dlpack__`)다. View 자체도 `__dlpack__`을 제공한다 | `alloc_tensor__(dtype, shape)`. dtype은 numpy dtype이나 DLPack 이름(`"bfloat16"`, `"float8_e4m3fn"` 등)이다 |
| C++ | `as_tensor__()`는 `flux::wire::Tensor`(첫 원소 포인터, `DlType`, shape, strides)다. `as<T>()`는 dtype이 `T`인 경우에만 포인터를 반환한다 | `alloc_tensor__(DlType, shape)`는 `void *`, `alloc_tensor__<T>(shape)`는 `Span<T>`를 반환한다 |

- strides는 원소 단위이고 비어 있으면 C-contiguous다. 범위 안이면 음수 stride도 받는다.
- 모든 원소가 `data` 안에 있어야 하고, 첫 원소가 원소 크기에 맞게 정렬되어 있어야 한다. 어기면 Python은 `WireError`, C++는 빈 결과와 `ok__() == false`다.
- 받는 dtype은 정수와 부호 없는 정수 8~64-bit, float 16, 32, 64-bit, complex 64, 128-bit, bool 8-bit, bfloat16, float8 계열이다.
- `Builder`는 `data`를 64 B 경계에 정렬한다.
- GPU 구독(`device="cuda"`)의 `Frame`을 View에 넘기면 `__dlpack__`이 GPU 텐서를 제공한다. 외장 GPU에서는 data가 arena에 있어도 같다([gpu.md](gpu.md)).
