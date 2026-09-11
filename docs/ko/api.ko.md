# flux 사용자 API

ROS 2 노드에서 쓰는 표면 전부다.

## 1. 먼저 알 것

**rendezvous 키는 토픽 이름과 fingerprint와 domain다.** 이 셋에서 `/dev/shm` 세그먼트 이름을 유도하므로, 양쪽이 같은 이름을 만들어야 붙는다. 한 글자라도 다르면 안 붙고 에러도 안 난다. ROS가 `"img"`를 `"/robot1/img"`로 바꾸므로, 노드를 넘기는 생성자를 써서 remap이 끝난 이름을 쓴다.

**domain은 한 호스트 안의 구획이다.** 기본값은 `0`이고, 보통 신경 쓸 일이 없다. `ROS_DOMAIN_ID`를 설정해 두었으면 flux도 그것으로 갈린다 — domain으로 나눠 놓은 두 시스템이 같은 토픽 이름을 써도 서로 섞이지 않는다. 직접 이름을 붙이려면 `FLUX_DOMAIN`을 쓴다(alnum 1..32자). 둘 다 있으면 `FLUX_DOMAIN`이 이긴다. 값이 alnum이 아니거나 `ROS_DOMAIN_ID`가 정수가 아니면 생성자가 던진다 — 오타가 조용히 기본 domain으로 떨어지면 격리가 사라지기 때문이다. 컨테이너는 이것과 무관하게 `/dev/shm`이 갈라 준다.

**fingerprint는 스키마 해시다.** 양쪽이 다르면 attach가 거절된다. `.msg`로 타입을 붙이면 생성기가 채워 준다. 0은 확인하지 않는다는 뜻이다.

**받은 view는 공유메모리를 가리킨다.** 살아 있는 동안 그 슬롯을 발행자가 못 쓴다. 쓰고 바로 놓는다.

**전달은 best-effort다.** 발행자는 느린 구독자를 기다리지 않는다.

산 발행자와 `slot_size`/`slot_count`가 어긋나면 attach가 `flux::SegmentMismatch`(Python은 `flux.SegmentMismatch`)를 던진다. 재시도로 안 고쳐지므로 잡지 않는다.

## 2. 타입 붙이기 (`.msg` -> adapter)

메시지 패키지에서 `.msg`를 opt-in하면 `flux_gen`이 C++ 헤더와 Python 모듈을 낸다. 배치·fingerprint가 양쪽에 상수로 박히므로 두 언어가 어긋날 수 없다. 배치 규칙은 `docs/message_shapes.md`.

```cmake
# my_pkg/CMakeLists.txt
find_package(flux_gen REQUIRED)
flux_generate_adapters(msg/Cloud.msg msg/Detection.msg)
```

```xml
<!-- my_pkg/package.xml -->
<buildtool_depend>flux_gen</buildtool_depend>
<exec_depend>flux_gen</exec_depend>   <!-- Python adapter가 flux_gen.wire를 import한다 -->
```

```text
include/my_pkg/flux/cloud.hpp    ->  my_pkg::flux_msg::Cloud
my_pkg_flux/cloud.py             ->  my_pkg_flux.cloud.Cloud
```

```text
# my_pkg/msg/Cloud.msg
std_msgs/Header header
float32[] x
float32[] y
float32[] z
uint32 width
string label
```

타입마다 셋이 나온다.

| | 뜻 |
| --- | --- |
| `kFingerprint` / `FINGERPRINT__` | 이 스키마의 지문. Publisher·Subscription에 그대로 넘긴다 |
| `View` | 받은 프레임을 읽는다. 0복사 |
| `Builder` | 빌린 슬롯에 직접 쓴다. 0복사 |

필드가 접근자로 나오는 규칙이다.

| `.msg` | C++ View | Python View | C++ Builder | Python Builder |
| --- | --- | --- | --- | --- |
| `uint32 width` | `v.width()` | `v.width` | `b.set__width(n)` | `b.width = n` |
| `float32[] x` | `v.x()` -> Span | `v.x` -> ndarray | `b.alloc__x(n)` | `b.alloc__x(n)` |
| `float64[9] m` | `v.m()` -> Span | `v.m` -> ndarray | `b.m()` -> Span | `b.m` -> ndarray |
| `string label` | `v.label()` | `v.label` | `b.set__label(s)` | `b.label = s` |
| `string[] tags` | `v.tags__size()` · `v.tags(i)` | `v.tags` -> list | `b.alloc__tags(n)` · `b.set__tags(i, s)` | `b.tags = [...]` |
| `Point[] pts` | `v.pts__size()` · `v.pts(i)` | `v.pts` -> 시퀀스 | `b.alloc__pts(n)[i]` | `b.alloc__pts(n)[i]` |
| `Header header` | `v.header__sec()` · `v.header__nanosec()` · `v.header__frame_id()` | `v.header__stamp` · `v.header__frame_id` | `b.set__header__stamp(s, ns)` · `b.set__header__frame_id(f)` | `b.set__header__stamp(s, ns)` · `b.header__frame_id = f` |

중첩은 이름 prefix로 펴진다 -- `pose.position.x`는 `pose_position_x`다.

`alloc__*`은 슬롯 안 메모리를 돌려준다. 거기에 쓰는 것이 0복사 경로다. 호출 순서는 자유다.

`.msg`가 거부되면(`bool[]`, `T[<=N]`, `wstring`, 필드 없음) 생성이 실패한다. 그 메시지는 plain ROS로 보낸다.

### C++

```cpp doc:adapter_cpp_pub
#include "my_pkg/flux/cloud.hpp"
using my_pkg::flux_msg::Cloud;

flux::ros::Publisher pub(*node, "cloud", Cloud::kFingerprint, 16 << 20, 16);

Cloud::Builder b = Cloud::build__(pub);   // 슬롯을 loan하고 그 위에 Builder를 얹는다
if (b) {
  auto xs = b.alloc__x(n);            // 슬롯을 가리킨다
  lidar.read_into(xs.data(), n);     // 데이터가 슬롯에서 만들어진다
  b.set__width(static_cast<std::uint32_t>(n));
  b.set__label("front");
  b.commit__();                        // ok()가 false면 발행하지 않는다
}
```

`build__(pub)`는 슬롯을 Builder가 소유하게 만든다. 살려 둘 객체가 하나다. 빈 슬롯이 없으면 Builder가 false이고, 그 상태로 써도 무해하지만 `commit__()`은 `Backpressure`를 돌려준다 -- 나간 척하지 않는다. Python의 `Cloud.build__(pub)`와 같은 것이다.

```cpp doc:adapter_cpp_sub
using my_pkg::flux_msg::Cloud;

flux::ros::Subscription sub(*node, "cloud", Cloud::kFingerprint, [](const flux::FrameView & f) {
  Cloud::View c(f);
  for (float x : c.x()) {
    use(x);
  }
  if (!c.ok__()) {                     // 프레임이 어긋났다: 읽은 값을 버린다
    return;
  }
});
```

### Python

```python doc:adapter_py_pub
from my_pkg_flux.cloud import Cloud

pub = flux.ros.Publisher(node, "cloud", fingerprint=Cloud.FINGERPRINT__)

b = Cloud.build__(pub)                  # 슬롯을 loan하고 Builder를 준다. 빈 슬롯 없으면 None
if b:
    b.alloc__x(n)[:] = xs              # 슬롯에 바로 쓴다
    b.width = n
    b.label = "front"
    b.commit__()
```

```python doc:adapter_py_sub
sub = flux.ros.Subscription(node, "cloud", fingerprint=Cloud.FINGERPRINT__)

f = sub.take()
if f is not None:
    c = Cloud.View(f)
    print(c.x, c.width, c.label)      # c.x는 슬롯을 가리키는 읽기전용 ndarray
```

수신 쪽은 프레임을 다른 프로세스가 썼다고 보고 descriptor를 검사한다. C++은 어긋나면 `ok()`가 false로 래치되고 그 뒤 접근자는 빈 값을 준다. Python은 `flux_gen.wire.WireError`를 던진다.

### ROS 메시지 객체와 오갈 때

이미 `sensor_msgs::msg::Image` 같은 ROS 메시지 객체를 들고 있으면 브릿지로 한 줄에 넣고 뺀다. 별도 헤더(`<pkg>/flux/<name>_ros.hpp`)·모듈(`<pkg>_flux.<name>_ros`)로 나와 있다 -- 이 브릿지만 rosidl 메시지 타입을 include하고, 안 쓰면 어댑터는 rclcpp·rclpy를 안 본다. 브릿지는 같은 타입의 두 표현을 오가므로 아래 예제는 `sensor_msgs/Image`로 통일한다.

```cpp doc:adapter_cpp_bridge
#include "sensor_msgs/flux/image_ros.hpp"
using sensor_msgs::flux_msg::Image;

// 보낼 때: ROS 객체 -> 프레임
Image::Builder b(w);
msg_to_frame(img, b);                 // img의 모든 필드를 슬롯에 복사
b.commit__();

// 받을 때: 프레임 -> ROS 객체
sensor_msgs::msg::Image back = frame_to_msg(Image::View(f));
```

```python doc:adapter_py_bridge
from sensor_msgs_flux.image import Image
from sensor_msgs_flux.image_ros import frame_to_msg, msg_to_frame

b = Image.build__(pub)
msg_to_frame(img, b)
b.commit__()

back = frame_to_msg(Image.View(f))
```

둘 다 필드를 전부 복사한다. `frame_to_msg`는 구독자마다 한 번씩 복사가 붙어 구독자 수와 무관한 0복사 성질이 사라진다. ROS 토픽에서 받은 걸 flux로 넘기거나 기존 함수 시그니처가 메시지 객체를 요구할 때만 쓴다. 데이터를 아직 안 만들었으면 `Builder`/`View`를 직접 쓴다(`docs/copy_model.md`).

stock ROS 2 도구(`ros2 bag`, `rqt`, `ros2 topic`)는 `flux_bridge`를 통해 flux 채널을 본다. 이 `frame_to_msg` 경로에 보는 채널마다 DDS publisher 하나를 얹은 것이다([bridge.md](bridge.ko.md)).

## 3. C++

`#include "flux/ros/publisher.hpp"` · `"flux/ros/subscription.hpp"` · `"flux/ros/executor.hpp"`

### Publisher

```cpp doc:publisher
flux::ros::Publisher pub(*node, "img", fingerprint, slot_size, slot_count);

pub.loan();                        // 0-copy. 빈 슬롯 없으면 invalid handle
pub.dropped();
pub.slot_size();                    // 슬롯 하나의 바이트. 생성 어댑터의 build__()가 읽는다
pub.segment_name();
```

기본값은 `fingerprint = flux::kNoSchema`, `slot_size = 16 MiB`(`Publisher::kDefaultSlotSize`), `slot_count = 16`(`Publisher::kDefaultSlotCount`)이다. 어댑터 경로는 fingerprint를 항상 넘기므로 뒤 둘만 생략한다. `slot_size`는 상한이지 할당이 아니다 -- payload 영역은 init 때 안 건드리므로 tmpfs가 sparse로 둔다. `slot_count`는 ring 깊이이고, 구독자 QoS의 하드 캡이다.

어댑터 경로의 발행은 `Builder::commit()`이다. 반환값 `flux::Published`(Python은 `flux.Published`)가 backpressure와 영구 오류를 가르고, 값의 뜻은 [`raw_api.ko.md`](raw_api.ko.md) 2절에 있다. 다섯을 나눠 볼 필요는 대개 없다 -- `flux::faulted(p)` / `flux.faulted(p)`가 "떨어뜨린 프레임인가, 스스로 안 풀리는 fault인가" 하나로 접는다.

`segment_name()`이 돌려주는 것은 signpost 이름(토픽·fingerprint에서 나온 고정 이름)이다. 실제 세그먼트 이름은 그 뒤에 인스턴스 suffix가 붙고 발행자 재시작마다 바뀐다.

일곱째 인자가 페이지 사전 커밋이다. 이름은 언어마다 다르다. C++ 생성자의 파라미터는 `mem`(`flux_cpp/include/flux/ros/publisher.hpp`)이고 Python 키워드 인자는 `memory`다. 기본은 꺼져 있고, 아래 `MemoryPolicy` 절이 그 거래를 적는다.

### WriteSlot (loan이 돌려주는 것)

`loan`은 슬롯을 먼저 찜해서 준다. 거기에 직접 쓰면 복사가 없다(`docs/copy_model.md`). 타입을 붙였으면 `Builder`가 이걸 감싸므로, 어댑터 경로에서 `WriteSlot`을 직접 만지는 자리는 아래 GPU 절 하나다.

찜한 슬롯은 commit·abort 전까지 ring에서 빠진다. 오래 쥐고 있으면 free 슬롯이 줄어 `dropped`가 오른다. `FrameView`와 마찬가지로 move-only다. 표면 전체와 `abort`의 의미는 [`raw_api.ko.md`](raw_api.ko.md) 3절에 있다.

### Subscription

```cpp doc:subscription
flux::ros::Subscription sub(
  *node, "img", fingerprint, [](const flux::FrameView & v) { handle(v); }, flux::QoS{});
```

구독은 자기를 돌리지 않는다. 읽는 방법이 둘이고 콜백 유무가 그걸 가른다.

콜백을 주면 push다. `flux::ros::Executor`나 `PartitionedExecutor`에 넘기고, 그 spin 스레드에서 프레임마다 콜백이 돈다. 둘 다 채널의 futex에서 깨므로 폴링이 없다.

콜백을 안 주면 pull이다. 이미 갖고 있는 루프에서 직접 읽는다. Python의 `flux.ros.Subscription`과 같은 모양이다.

```cpp doc:subscription_pull
flux::ros::Subscription sub(*node, "img", fingerprint);  // 콜백 없이

flux::FrameView newest = sub.peek();              // 최신 상태. 소비하지 않는다
flux::FrameView next = sub.take();                // 다음 프레임. 없으면 invalid
flux::FrameView blocked = sub.take_blocking(-1);  // 올 때까지 futex에서 잔다
```

여섯째·일곱째 인자가 `Device`와 `MemoryPolicy`다. 후자는 이 구독 자신의 매핑에 대한 페이지 사전 커밋이고 아래 `MemoryPolicy` 절에 있다. `sub.pages_committed()`·`sub.pages_locked()`가 그 결과를 보고한다 -- attach 전에는 둘 다 false다.

콜백 없는 구독을 executor에 넣으면 `add()`가 throw한다. 실행할 것이 없는데 등록만 되는 상태를 만들지 않는다.

wall timer로 자기를 돌리는 `poll_period` 인자가 있었고 없앴다. 주기로 깨는 것이라 구독마다 지연 상한과 idle wakeup을 물었고, `flux_py`가 제공할 수 없는 유일한 배치라 같은 노드를 두 언어로 짜면 executor가 달라졌다.

```cpp doc:subscription_api
bool attached = sub.attached();
bool driven = sub.has_callback();  // false면 pull 전용. executor가 거절한다
const flux::QoS & qos = sub.qos();
std::uint64_t lost = sub.lost();   // 못 받은 누적 수
bool can_borrow = sub.can_borrow();               // false면 내가 view를 쥐고 있어서 빈 것이 온다
flux::Channel::Refused refused = sub.refused();   // 4절 "빈 것이 왔을 때"와 같은 필드
const std::string & domain = sub.domain();          // 이 프로세스의 domain. pub.domain()도 같다
```

`attached()`와 `domain()`를 같이 읽는다. 프레임이 안 오는데 `attached()`가 false면 발행자가 아직 없는 것이고, `domain()`가 예상과 다르면 발행자는 있는데 다른 구획에 있는 것이다. 둘은 겉이 같다 -- `take()`가 빈 것을 주고 `refused()`가 전부 0이다. 어느 domain에 무엇이 사는지는 `flux domain list`가 보여준다([cli.md](cli.ko.md)).

domain은 프로세스당 하나다. 처음 필요할 때 한 번 정해지고 그 뒤로 안 바뀌므로, 같은 프로세스의 publisher와 subscription이 다른 값을 볼 수 없다. rcl이 `ROS_DOMAIN_ID`를 다루는 방식과 같다 — 도중에 환경변수를 바꿔도 이미 뜬 노드의 domain이 안 바뀌는 것과 같은 규약이다.

### MemoryPolicy (페이지 사전 커밋)

`slot_size`는 상한이지 할당이 아니다. payload는 sparse tmpfs로 남고 처음 닿는 페이지마다 fault가 난다. 그 fault는 init이 아니라 첫 발행과 첫 take에 얹힌다. `MemoryPolicy`가 그것을 attach로 옮긴다.

```cpp doc:memory_policy
flux::MemoryPolicy mem;
mem.precommit = true;   // attach 때 전 페이지를 fault-in 한다
mem.lock = true;        // mlock까지. RLIMIT_MEMLOCK이 세그먼트를 덮어야 한다

flux::ros::Publisher pub(
  *node, "img", fingerprint, slot_size, slot_count, flux::Device::Cpu, mem);
flux::ros::Subscription sub(*node, "img", fingerprint, {}, flux::QoS{}, flux::Device::Cpu, mem);

bool committed = pub.pages_committed();
bool locked = pub.pages_locked();
```

- 기본은 둘 다 꺼짐이다. 켜면 sparse의 이점(큰 `slot_size`를 idle RAM 거의 0으로)을 포기하는 거래다.
- 프로세스마다 따로 선언한다. 발행자가 커밋했다는 것은 구독자의 매핑에 대해 아무것도 말하지 않는다. fault는 매핑마다 따로 난다.
- 전부이거나 아무것도 아니다. 헤더·슬롯·holder 테이블·payload 전체를 덮는다. 일부만 커밋하면 상한이 안 서고, 상한을 세우는 것이 이 옵션의 목적이다.
- 재attach마다 다시 적용된다. 발행자가 재시작하면 구독자는 새 매핑을 얻고 그 매핑은 이전 매핑의 residency를 하나도 안 물려받는다.
- 거부는 조용히 낮추지 않고 던진다. `mlock`이 `RLIMIT_MEMLOCK`에 걸리면 `std::system_error`이고 attach가 실패한다. 선언한 상한을 못 받은 채로 도는 채널이 이 옵션이 막으려는 것이다.
- `lock`은 `precommit`을 포함한다. `mlock`은 잠그는 것을 populate한다. `precommit`만 켜면 커밋은 하되 메모리 압박에 다시 swap-out될 수 있다.
- 커널 5.14 미만에서는 `precommit`이 던진다. `MADV_POPULATE_WRITE`가 없다.
- dGPU 채널의 payload는 이 매핑 안에 없다. 덮이는 것은 control plane뿐이다.

`pages_committed()`·`pages_locked()`는 실제로 무엇을 받았는지다. 기본 정책이면 둘 다 false이고, 요청했는데 거부됐으면 false가 아니라 throw다.

### FrameView (콜백이 받는 것)

```cpp doc:frame_view_api
bool valid = static_cast<bool>(v);       // operator bool은 explicit이다
const void * data = v.data();
std::size_t size = v.size();
v.release();                             // 소멸자도 같은 일을 한다
```

콜백이 도는 동안만 유효하다. move-only라 복사되지 않는다. `meta()`는 어댑터 경로에서 안 쓴다 -- 스키마가 fingerprint에 있다. 직접 읽는 자리는 [`raw_api.ko.md`](raw_api.ko.md) 3절이다.

### GPU (CUDA)

`Device::Cuda`를 붙이면 같은 채널이 GPU 경로가 된다. 안 붙이면 `Device::Cpu`가 기본이라 위 절들 그대로다.

```cpp doc:gpu_publisher
flux::ros::Publisher pub(*node, "img", fingerprint, slot_size, slot_count, flux::Device::Cuda);

flux::WriteSlot w = pub.loan(flux::DType::U8, {480, 640, 3});
if (w) {
  render_into(w.device_ptr(), w.capacity(), w.stream());   // 커널을 이 stream에 올린다
  w.commit();                                              // stream을 기다린 뒤 발행
}
pub.fence_failed();
pub.fence_wait();
```

```cpp doc:gpu_subscription
flux::ros::Subscription sub(
  *node, "img", fingerprint,
  [](const flux::FrameView & v) { use(v.device_ptr(), v.size(), v.stream()); }, flux::QoS{},
  flux::Device::Cuda);

sub.fence_failed();
sub.fence_wait();
```

CPU와 다른 곳은 이렇다.

| | CPU | CUDA |
| --- | --- | --- |
| 생성자 | 없음 | `flux::Device::Cuda` |
| 주소 | `data()` | `device_ptr()` |
| 커널 인자 | -- | `stream()`을 함께 넘긴다 |

`device_ptr()`은 `Device::Cuda`가 아니면 `nullptr`이다. `data()`를 커널에 넘기면 안 된다 -- 장비에 따라 GPU가 보는 주소가 다르다.

`host_addressable()`은 반대쪽 질문이다 -- `data()`를 CPU로 읽어도 되는가. iGPU에서는 슬롯이 host 메모리이기도 해서 `true`이고, dGPU에서는 슬롯이 VRAM이라 `false`다. `device_ptr()`이 `nullptr`이 아니라는 것으로 이걸 대신 판정하면 안 된다 -- iGPU는 둘 다 성립하는 유일한 경우다. `FrameView`·`WriteSlot`·`Channel` 셋 다 같은 이름의 접근자를 가진다.

`host_addressable()`이 `false`면 `data()`는 `nullptr`이다. 물어보지 않은 호출자를 첫 접근의 fault가 아니라 그 자리에서 멈추게 하려는 것이다 -- `wire::Reader`와 `wire::Writer`가 null base에서 `bad()`를 래치하므로, 생성 adapter의 `View`·`Builder`는 `ok() == false`가 되고 `commit()`이 거부한다.

dGPU에서는 넷이 닫힌다. iGPU는 슬롯이 host 메모리이기도 해서 전부 열려 있다.

| 닫히는 것 | 대신 |
| --- | --- |
| `publish(host 배열)` | `loan()` + `commit()` |
| `Device::Cpu` 구독 | attach에서 거절된다. GPU 구독만 붙는다 |
| Python `.array` · `.bits` | `__cuda_array_interface__` 또는 `__dlpack__` |
| C++ `data()`와 그 위의 adapter | `device_ptr()`. flat wire는 host 배치라 못 쓴다 |

`stream()`은 채널에 선언된 stream을 돌려준다. 자기 stream 변수를 따로 들고 다니지 말고 이걸 쓴다. flux가 기다리는 stream과 일이 올라간 stream이 달라지면 기다림이 헛돌기 때문이다.

`commit()`과 view 해제는 stream을 기다린 뒤에 일어난다. 그래서 "발행됐다"는 CPU에서와 똑같이 "읽어도 된다"는 뜻이다. iGPU에서는 CPU로 구독하는 노드가 같은 채널에서 그대로 돈다 -- 슬롯이 host 메모리이기도 하기 때문이다. dGPU에는 그 성질이 없어 CPU 구독이 거절된다(위 표).

기다림이 실패하면 flux는 아무것도 안 놓는다. 그 슬롯은 다시 안 쓰이고 `fence_failed()`가 오른다. 0이 아니면 rate가 아니라 고장이다.

`fence_wait()`는 그 기다림이 실제로 얼마나 걸렸는지를 준다. `Channel::FenceWait` 하나에 두 seam이 따로 담긴다.

| 필드 | 뜻 |
| --- | --- |
| `commit_ns` · `commit_count` · `commit_max_ns` | 발행 seam. 합·횟수·최댓값 |
| `release_ns` · `release_count` · `release_max_ns` | 해제 seam. 같은 셋 |

seam을 나눠 두는 이유는 막는 스레드가 다르기 때문이다. 발행 쪽은 발행하는 스레드를 막고, 그 스레드는 kernel launch와 `commit()` 사이에 다른 일을 끼워 대기를 그만큼 줄일 수 있다. 해제 쪽은 view를 놓는 스레드를 막고, 줄일 방법이 없다 — fence는 stream 단위라 view를 오래 쥐면 그동안 쌓인 것까지 기다리게 된다. 받아 쓰고 바로 놓는 것이 최선이고, 콜백 경로가 이미 그렇게 한다.

`Device::Cpu` 채널에서는 전부 0이다 -- 시계조차 안 읽는다. 실패한 기다림도 시간은 썼으므로 함께 센다. 누적값이고, `_max_ns` 둘만 지금까지의 최댓값이라 줄지 않는다.

생성자는 이 호스트가 못 하는 선언을 그 자리에서 거절한다 -- GPU가 없으면 던진다. 조용히 CPU로 안 내려간다. 등록이 필요한 iGPU에서는 거절하는 대신 payload를 등록하고, 드라이버가 그 등록을 거절하면 그때 던진다.

### FrameMeta

프레임마다 슬롯에 함께 실리는 서술자다. 생성된 어댑터가 쓴 프레임은 엔진이 해석하지 않는 바이트열이므로 `u8[nbytes]`로 찍힌다. 스키마는 메타가 아니라 fingerprint에 있고, 그래서 이 경로에는 `FrameMeta`를 읽거나 쓰는 자리가 없다. 칸의 뜻과 읽는 법은 [`raw_api.ko.md`](raw_api.ko.md) 2절에 있다.

### Executor

flux 구독과 ROS 구독을 io_uring 하나로 같이 기다린다. 리눅스 6.7 미만에서는 채널당 스레드 폴백으로 내려간다 -- `uses_io_uring()`이 어느 경로인지 보고한다.

대기 자체는 `flux::Executor`(flux_core)다. ROS를 모르는 부분은 거기 있고, 이 클래스가 얹는 것은 readiness를 그 대기로 보내는 다리와 rclcpp 실행 경로다. `flux_py`도 같은 `flux::Executor`를 쓴다(`core_api.ko.md` 6).

```cpp doc:executor
flux::ros::Executor ex(32);      // max_channels
ex.add(flux_sub);                // 콜백 없는 구독이면 throw. max_channels 초과면 throw
ex.add(control_sub, 10);         // priority: 프레임이 있으면 먼저 돈다. 기본 0
ex.add_ros_node(node);           // 노드째 넘긴다. 구독·타이머·서비스를 rclcpp가 꺼낸다
ex.spin();                       // tick_ns 기본값 100 ms. spin(tick_ns)로 바꾼다
ex.stop();                       // spin을 끝낸다. 콜백에서 불러도 된다
```

`add`의 둘째 인자는 우선순위다. 큰 것이 먼저 가고 같으면 등록순이며 음수도 쓴다. 선택은 콜백마다 다시 한다 -- 낮은 채널의 콜백이 도는 중에 높은 채널로 프레임이 오면 낮은 채널의 다음 프레임보다 그것이 먼저 돈다. 선점 우선순위는 아니다. 이미 도는 콜백은 밀리지 않고, 한 pass의 상한도 안 바뀐다. 정렬되는 것은 flux 채널뿐이다. 같은 executor의 ROS entity는 `pump_ros()`가 rclcpp 순서로 돌린다.

등록은 spin 전에만 한다. spin 중 `add`/`add_ros_node`/`add_ros_callback_group`은 throw — spin 스레드가 락 없이 등록 목록을 순회한다. spin이 반환한 뒤에는 다시 등록할 수 있다. 노드에 늦게 생긴 ROS 구독은 예외로, 다음 pass가 자동으로 잇는다(아래).

`add_ros_node`는 노드를 rclcpp에 등록하고, 그 노드의 구독마다 on-new-message 콜백을 걸어 readiness를 이 executor의 eventfd로 보낸다. 기다리는 것은 flux이고 꺼내는 것은 rclcpp다 -- 직렬화 구독·intra-process·loaned message 분기를 rclcpp가 이미 갖고 있고, 손으로 꺼내면 그걸 전부 다시 짜야 한다. 그 대가로 타이머·서비스도 같이 돈다.

호출 시점에 없던 구독은 다음 spin pass 초입의 재스캔이 잇는다. 두 콜백 다 설정될 때 밀린 count를 재생하므로 다리가 걸리기 전에 도착한 메시지도 그 pass에서 깨운다. 감지 지연 상한은 tick 하나다.

`tick_ns`는 폴링 주기가 아니다. stop 재확인과 늦게 뜬 발행자 attach를 위한 대기 상한이다. 노드에 tick보다 짧은 주기의 타이머가 있으면 대기는 그 deadline까지만 잡으므로, tick을 길게 줘도 타이머 주기는 안 늘어난다.

끝내는 방법은 `stop()`이다. 자기 종료 플래그를 이미 들고 있으면 `spin(run, tick_ns)`가 그것도 함께 본다.

상속받은 rclcpp 진입점 중 셋은 flux에서 뜻이 정확히 하나라 구현했다 -- `spin()`은 기본 tick의 `spin(tick_ns)`, `cancel()`은 `stop()`, `spin_once(timeout)`은 `spin_once(timeout.count())`다. 그래서 `rclcpp::Executor &`로 들고 있는 호출자도 이 executor를 제대로 돌린다. 나머지(`spin_some`·`spin_all`·`spin_node_some`·`spin_node_all`·`spin_until_future_complete`)는 throw다 -- 이 executor에 없는 wait set과 duration 예산에 대한 계약이고, 그럴듯한 근사는 flux 채널을 조용히 건너뛴다.

```cpp doc:executor_api
bool merged = ex.uses_io_uring();   // false면 채널당 스레드 폴백으로 돌고 있다
std::size_t channels = ex.size();   // 등록된 flux 구독 수
ex.interrupt();                     // 루프는 두고 대기만 깬다
int frames = ex.dispatch();         // 블록 없이 준비된 flux 프레임만 전달
ex.wait_for_work(1'000'000);        // 전달 없이 대기만
int ros_ran = ex.pump_ros();        // rclcpp가 준비됐다고 보고한 것 실행. 상한은 ros_budget
ex.set_ros_budget(16);              // 한 pass의 ROS 콜백 상한. 기본 64. spin 전에만
int budget = ex.ros_budget();
ex.set_pass_budget(16);             // 한 pass의 flux 콜백 상한. 채널 전체에 하나. spin 전에만
int flux_budget = ex.pass_budget();
bool more = ex.has_more();          // 예산이 자른 나머지가 있다. 다음 대기를 건너뛴다
bool ros_woke = ex.take_ros_ready();  // false면 ROS는 아무것도 안 왔다. pump_ros 생략 가능
```

`has_more()`는 직전 `dispatch()`가 프레임이 남은 채로 예산에서 멈췄는지다. `wait_for_work()`와 `dispatch()`를 직접 부르는 루프는 이 값이 참이면 대기를 건너뛴다 -- 이미 와 있던 프레임에 대해 아무도 ring을 다시 두드리지 않는다. `spin()`과 `spin_once()`는 알아서 한다.

`pass_budget`은 flux 쪽의 같은 것이다. `dispatch()` 한 번이 돌릴 콜백 수의 상한이고, 채널마다가 아니라 채널 전체에 대한 하나의 예산이다. 기본값은 `flux::kMaxDrain`(64)이다. 낮추면 높은 우선순위 채널이 낮은 채널 뒤에서 기다리는 시간이 줄고, 올리면 pass당 arm 비용이 더 많은 프레임에 나뉜다.

`ros_budget`은 한 pass가 돌릴 ROS 콜백 수의 상한이다. 없으면 `pump_ros()`가 rclcpp 큐가 빌 때까지 돌고, 콜백보다 빠른 ROS 스트림이 그 스레드를 스트림이 끝날 때까지 잡는다 -- 그동안 같은 executor의 flux 프레임은 안 돈다. 상한을 넘긴 나머지는 다음 pass가 블록 없이 이어 받으므로 처리율이 아니라 pass 길이를 묶는다. 한 pass의 상한에 `ros_budget x ROS 콜백 WCET`이 들어간다.

아래 셋은 남의 이벤트 루프에 flux를 얹을 때 쓴다. `dispatch()`로 프레임을 꺼내고 `pump_ros()`로 ROS를 돌리고, 블로킹은 `wait_for_work()`가 맡는다. `take_ros_ready()`는 직전 호출 이후 브릿지된 ROS entity가 신호했는지 보고하고 읽으면서 지운다 -- false면 `pump_ros()`와 그것이 무는 rclcpp pass를 건너뛸 수 있다. `spin()`이 그 판정을 이렇게 한다. ring에서 읽지 않고 훅이 직접 세우는 이유는 콜백 실행 중에 온 도착이 그 pass의 대기가 끝난 뒤에 놓이기 때문이다. `dispatch()`와 `wait_for_work()`는 같은 ring을 쓰므로 동시에 돌면 안 된다 -- 호출자가 그 handshake를 진다. `flux_py`의 rclpy bridge가 이 셋으로 짜여 있다.

### message_filters 동기화

여러 토픽을 시각으로 짝지어 한 콜백으로 받는 ROS 표준 경로가 [`message_filters`](https://index.ros.org/p/message_filters/)다. flux 토픽을 그 그래프에 소스로 넣는다. flux 토픽끼리도, flux와 ROS 토픽을 섞어서도 된다.

```cpp doc:adapter_cpp_sync
namespace mf = message_filters;
using sensor_msgs::flux_msg::Image;
using Frame = flux::ros::message_filters::StampedFrame<Image>;

flux::QoS qos;
qos.depth = 4;
qos.max_borrow = 16;  // inputs x queue_size: 짝이 올 때까지 필터가 프레임을 쥔다

flux::ros::message_filters::Subscriber<Image> left(node, "cam/left", qos);
flux::ros::message_filters::Subscriber<Image> right(node, "cam/right", qos);

using Policy = mf::sync_policies::ApproximateTime<Frame, Frame>;
mf::Synchronizer<Policy> sync(Policy(10), left, right);
sync.registerCallback(std::bind(
  [](const std::shared_ptr<const Frame> & a, const std::shared_ptr<const Frame> & b) {
    Image::View l = a->view();
    Image::View r = b->view();
    use(l.width(), r.width());
  },
  std::placeholders::_1, std::placeholders::_2));

flux::ros::Executor ex;
ex.add(left);   // 한 synchronizer의 모든 입력은 한 스레드에서 돌아야 한다
ex.add(right);
ex.spin();
```

큐에 들어가는 것은 바이트가 아니라 `StampedFrame`이다. borrow 하나와 stamp를 든다. payload는 세그먼트에 그대로 있으므로 이 경로도 복사가 없다. `view()`가 어댑터의 `View`를 준다.

- 동기화 키는 메시지의 header stamp다. `Subscriber`가 프레임에서 `header__sec()`·`header__nanosec()` 둘만 읽어 `StampedFrame::stamp`에 넣는다. header 없는 스키마는 여기 못 들어간다 -- `FrameMeta`에 시각이 없어서 대안이 없고, 컴파일이 그 자리에서 멈춘다.
- `Subscriber`는 `flux::Source`다. `ex.add(sub)`로 executor에 직접 넣는다. 기본 생성 후 `subscribe(node, topic, qos)`로 나중에 붙일 수도 있다 -- 노드 멤버로 선언될 때 필요하다. `subscribed()`가 붙었는지 보고하고, `unsubscribe()`가 뗀다. 안 붙은 동안 executor는 이 소스에서 아무것도 못 받고 그것으로 끝이다.
- `Subscriber<Image>::Message`가 큐에 들어가는 타입, 즉 `StampedFrame<Image>`다. `Synchronizer` 정책의 타입 인자를 쓸 때 어느 쪽 이름을 써도 같다.
- `forwarded()`가 필터로 내보낸 수, `unreadable()`이 스키마에 안 맞아 버린 수다. synchronizer는 짝을 못 찾은 메시지를 조용히 버리므로, `forwarded()`를 사용자 콜백 횟수와 diff하는 것이 그 손실을 보는 방법이다.
- `queue_size`(위의 `Policy(10)`)만큼을 입력마다 쥐므로 `max_borrow`가 그 곱을 덮어야 한다. 안 덮으면 소비자가 자기 lease를 다 써서 더 못 가져온다.

**한 synchronizer의 모든 입력은 같은 스레드에서 서비스돼야 한다.** 스타일 규칙이 아니다. 동기화 정책은 짝이 맞은 콜백을 자기 `std::mutex`를 쥔 채로 돌리고 그 뮤텍스에는 priority inheritance가 없다. 입력이 두 스레드에 걸치면 그 둘의 우선순위가 락 하나로 묶인다. `flux::ros::Executor`는 flux와 ROS를 한 스레드에서 돌리므로 자동으로 만족한다. `PartitionedExecutor`에서는 그 synchronizer의 입력을 전부 같은 콜백 그룹에 배정한다.

`PartitionedExecutor`에서는 그 배정을 선언하면 spin이 검사한다. synchronizer는 자기 입력이 무엇인지 아무에게도 말하지 않으므로 집합을 밖에서 대야 한다.

```cpp
ex.add(flux_in, g);
ex.add_sync_group(flux_in, ros_in);   // 이 둘이 한 synchronizer의 입력이다
ex.spin();                            // 둘이 다른 그룹이면 여기서 던진다
```

- `add_sync_group`은 flux 입력과 순정 `message_filters::Subscriber`를 같이 받는다. flux 토픽과 DDS 토픽을 섞은 그래프가 이 검사의 대상이다. 그쪽은 `dispatch()`와 `pump_ros()`라는 다른 기계가 꺼내므로 같은 그룹만이 둘을 한 스레드에 놓는다.
- 배정 안 된 flux 입력은 던진다. 어느 자식도 그것을 돌리지 않아 짝이 영영 안 온다.
- 자리를 정할 수 없는 입력은 판정하지 않고 `unplaced_sync_inputs()`가 센다. 체인 중간 필터, 그리고 `add_ros_node()`에 안 넘긴 노드의 구독이 그것이다. 0이면 선언한 입력이 전부 판정됐다는 뜻이다.
- 선언하지 않으면 검사도 없다. 지금까지와 같다.

필터 경로는 할당이 있는 경로다. 정책의 큐가 `std::deque`·`std::map`이고 프레임마다 `shared_ptr` control block 둘이 붙는다. 필터 없는 flux 배달에는 할당이 없다 -- hard RT 체인에는 필터를 놓지 않는다.

### PartitionedExecutor

콜백 그룹마다 자식 executor와 스레드 하나를 붙여 그룹 사이를 격리한다. flux 구독이 든 그룹은 자식 `flux::ros::Executor`(그룹당 io_uring 하나), 없는 그룹은 순정 `SingleThreadedExecutor`가 맡는다.

그룹당 스레드 하나라는 구성은 autowarefoundation/callback_isolated_executor와 agnocast의 `CallbackIsolatedAgnocastExecutor`를 참고했다. 구현은 공유하지 않는다.

```cpp doc:partitioned
flux::ros::PartitionedExecutor ex;
ex.add(flux_sub, group);   // 그룹에 배정. 같은 그룹 = 같은 스레드
ex.add_ros_node(node);     // 노드의 콜백 그룹마다 자식이 붙는다
ex.schedule(group, {flux::rt::Policy::Fifo, 90, {3}});  // 그룹 스레드의 스케줄링 선언 (opt-in)
ex.spin();                 // tick_ns 기본값 100 ms
ex.stop();                 // spin과 자식 전부를 끝낸다
ex.interrupt();            // 부모의 스캔 대기만 깬다
```

- `add`의 그룹은 `add_ros_node`로 등록된 노드의 것이어야 한다. 아니면 spin이 throw.
- `add`의 그룹은 ROS entity를 가져도 된다. 그 그룹의 ROS 콜백도 같은 자식이 서비스한다. 한 pass 안에서 flux 콜백과 섞여도 ROS entity는 콜백 하나만큼만 뒤로 밀린다.
- `add(sub, group, priority)`의 셋째 인자는 그 그룹 자신의 pass 안 방문 순서다. 그룹을 넘지 않는다 -- 그룹은 서로 다른 스레드이고 그 사이 순서는 `schedule`이 건 스레드 우선순위가 정한다.
- 등록은 spin 전에만 한다. spin 중 `add`/`add_ros_node`/`schedule`은 throw.
- spin 시작 후 생긴 콜백 그룹은 tick 스캔이 잡아 자식을 붙인다.
- 자식 `flux::ros::Executor`의 `max_channels`는 그룹에 배정된 flux 구독 수로 자동 산정된다.
- `schedule`은 그 그룹을 맡은 자식 스레드가 첫 콜백 전에 자기에게 `rt::apply`하는 예약이다. 거부(권한 없음 등)는 spin()의 에러로 올라온다. 그룹당 하나, 어느 자식도 맡지 않는 그룹의 schedule은 spin이 거부한다.
- `schedule(group, stage)`는 stage가 부르는 이름과 넘긴 그룹을 대조한다. stage가 가리키는 노드의 그룹이어야 하고, 라벨이 비었으면 그 노드의 기본 콜백 그룹이어야 한다. 한 stage를 두 그룹에 주면 그 자리에서 던진다.
- reentrant 그룹은 거절한다. 그룹당 스레드가 하나라 그 그룹의 콜백은 직렬로 돌고, 동시 실행을 선언한 그룹을 조용히 직렬화하지 않는다. 병렬이 필요하면 mutually exclusive 그룹 여럿으로 쪼갠다 -- 각자 스레드와 우선순위를 갖는다.
- 노드가 자동 등록으로 만든 그룹은 flux 구독이 없어도 자식이 붙는다. `automatically_add_to_executor_with_node()`가 false인 수동 그룹은 `add(sub, group)`으로 배정된 것만 서비스한다.
- 그룹이 다르면 콜백은 실제로 동시에 돈다. 그룹 사이에 공유하는 상태는 호출자가 지킨다 -- PartitionedExecutor는 격리를 주지 상호배제를 주지 않는다(6).
- GPU 채널을 받는 구독은 자기 그룹에 둔다. 해제 fence가 소비 커널이 끝날 때까지 그 스레드를 막으므로, 같은 그룹의 다른 콜백이 그만큼 밀린다.

### RtSpec (체인 선언 파일)

체인 하나의 스케줄링을 파일 하나에 적고 각 노드가 뜰 때 자기 몫을 찾아 적용한다. 노드가 launch로 각각 뜨고 죽어도 성립한다 — 미는 쪽이 없고 각자 읽기 때문이다.

```yaml
chains:
  perception_to_control:
    target: hard                # Warn을 막는다. soft면 기록만
    stages:
      - node: /camera_node
        group: dds_listener
        external: true          # flux가 안 건다. rmw 리스너 같은 남의 스레드
        expect_priority: 60
      - node: /camera_node      # group 생략 = 노드의 기본 콜백 그룹
        policy: fifo
        priority: 70
        cpus: [2]
      - node: /perception_node
        group: infer
        policy: fifo
        priority: 75
      - node: /control_node
        group: loop
        policy: fifo
        priority: 90
```

```cpp doc:rt_spec
flux::ros::RtSpec spec = flux::ros::RtSpec::load();   // FLUX_RT_SPEC. 없으면 빈 spec
if (!spec.empty()) {
  const flux::ros::RtStage & st = spec.stage(*node, "infer");
  ex.schedule(group, st);              // target과 control_priority가 체인에서 따라온다
  log(st.chain, st.node);              // 어느 체인의 어느 단계인지

  std::thread worker([&spec, &node] {  // 내가 만든 스레드는 그 스레드가 자기에게 건다
    flux::ros::apply_checked(spec.stage(*node, "infer_worker"));
    flux::ros::verify(spec.stage(*node, "infer_worker"), flux::rt::this_tid());
  });
  worker.join();
}
for (const flux::ros::RtStage * e : spec.external()) {   // flux가 안 거는 단계
  log(e->node, flux::ros::verify(*e, foreign_tid).to_string());   // 기록만 하지 않고 확인한다
}
```

`RtStage`는 파일이 그 단계에 대해 아는 전부를 담는다 — `node`·`group`·`external`·`expect_priority`·`opts`·`strict`·`control_priority`, 그리고 어느 체인이 선언했는지(`chain`). `RtSpec`은 `stages()`로 전부, `find()`로 하나, `external()`로 flux가 안 거는 것만 준다.

- `stages`는 데이터가 흐르는 순서다. RT 우선순위가 그 방향으로 커져야 한다 — 상류가 하류를 선점하면 굶긴다. `policy: other`처럼 RT가 아닌 단계는 이 규칙 밖이다.
- `control_priority`는 파일이 유도한다 — 체인의 마지막 RT 우선순위가 control이고, 그 단계 자신은 0이다.
- 모르는 키는 거절한다. 버전 필드는 없다.
- 한 단계가 체인 둘에 나오는 것은 정상이다. 두 체인이 서로 다르게 선언하면 거절한다.
- hard 체인은 RT 단계마다 코어를 따로 준다. 둘이 같은 코어에 핀되면 거절한다 — `SCHED_FIFO`에는 타임슬라이스가 없어 하나가 다른 하나를 완전히 막는다. soft 체인은 허용한다.
- `external` 단계는 `expect_priority`만 적는다. `policy`/`priority`/`cpus`는 못 가지고, `schedule`에 넘기면 거절한다.
- 파일이 없거나 `FLUX_RT_SPEC`이 비면 빈 spec이다. 아무것도 선언 안 한 지금의 no-op 그대로다.
- 파일에 없는 라벨을 `stage()`로 찾으면 던진다.
- `verify(stage, tid)`는 아무것도 안 걸고 선언과 실제만 대조한다. finding은 `observed-policy`·`observed-priority`·`observed-cpus` 셋이고, 파일이 주장하지 않는 항목은 `Unknown`이다. 읽을 수 없는 스레드는 `observed-thread` Fail이다. `tid`는 호출자가 댄다.
- 스레드에 거는 방법이 둘이다. 콜백 그룹은 `schedule(group, stage)`, 직접 만든 스레드는 그 안에서 `apply_checked(stage)`. 둘 다 stage를 통째로 넘긴다 — `strict`와 `control_priority`를 필드로 풀면 흘린다. `external` 단계는 둘 다 거절한다.

### rt (스레드 스케줄링)

hard RT 경로용 opt-in이다. flux_core 소속이라 ROS 없이도 쓴다. QoS와 별개다.

```cpp doc:rt
#include "flux/rt.hpp"

flux::rt::Options o;
o.policy = flux::rt::Policy::Fifo;   // Inherit / Other / Fifo / RoundRobin
o.priority = 80;                     // Fifo/RoundRobin일 때 1..99
o.cpus = {3};                        // 비우면 affinity를 안 건드린다

flux::rt::Report rep = flux::rt::preflight(o, /*control_priority=*/90);  // 판정만, 변경 없음
flux::rt::apply(o);                  // 호출한 스레드에 적용. 거부되면 throw
flux::rt::ThreadState st = flux::rt::current();   // 커널에서 되읽기
flux::rt::ThreadState other = flux::rt::observe(flux::rt::this_tid());  // 남의 스레드도 같은 값

rep = flux::rt::apply_checked(o, flux::rt::Strictness::Hard, /*control_priority=*/90);
```

- `apply`는 호출한 스레드 자신에게 건다. all-or-nothing이다 — 커널이 거부하면 되돌리고 `std::system_error`를 던진다. 조용한 다운그레이드는 없다.
- `preflight`는 아무것도 안 바꾸고 finding 목록(`rep.ok()`, `rep.to_string()`)으로 왜 안 되는지를 보고한다.
- `apply_checked`는 그 둘을 잇는다. `preflight`를 먼저 돌리고 판정이 막으면 스레드를 건드리기 전에 `std::runtime_error`를 던진다. 커널은 마감을 못 지키는 호스트에서도 요청을 받아주므로, `apply`만 쓰면 "성공"이 "실시간으로 돈다"를 뜻하지 않는다.
- `Strictness`가 `Warn`을 어떻게 읽을지 정한다. `Soft`는 감수하고 report로 돌려주며, `Hard`는 거절한다. `Fail`은 둘 다 거절한다. 기본은 `Soft`다.
- 아무것도 요청하지 않는 `Options`는 `apply_checked`에서도 완전한 no-op이다. 판정할 대상이 없다.
- `current`는 자기 스레드를, `observe(tid)`는 남의 스레드를 커널에서 읽는다. 거는 것은 자기 스레드만 되지만 읽는 것은 프로세스를 넘어서도 된다. `tid`는 `this_tid()`가 준다 — `std::thread::id`는 커널이 아는 이름이 아니다. 읽을 수 없는 스레드는 `std::system_error`다.
- 자기 스레드는 `apply`나 `apply_checked`를 직접 부른다. PartitionedExecutor가 만드는 그룹 스레드는 `schedule(group, opts, strict, control_priority)`로 선언하면 자식이 `apply_checked`로 스스로 건다(위 PartitionedExecutor 절).

한 줄 요약 대신 항목을 직접 보려면 `rep.findings`를 순회하거나 `rep.find(id)`로 하나를 집는다.

```cpp doc:rt_report
for (const flux::rt::Finding & f : rep.findings) {
  if (f.verdict == flux::rt::Verdict::Fail) {
    log(f.id, f.detail);
  }
}
const flux::rt::Finding * one = rep.find("cpu-online");   // 안 봤으면 nullptr
```

`Finding`은 셋이다 — `id`(안정 슬러그), `verdict`, `detail`(왜 그 판정인지). `Verdict`는 `Ok`·`Warn`·`Fail`·`Unknown`이고 `ok()`는 "`Fail` 없음"이다. `Unknown`은 커널이 소스를 안 내준 것이지 통과가 아니다. id별 소스와 Fail·Warn의 뜻은 아래 표다.

| finding id | 소스 | Fail/Warn의 뜻 |
| --- | --- | --- |
| `policy-permission` | `CapEff`(CAP_SYS_NICE) + `RLIMIT_RTPRIO` | Fail: 권한 없음. `apply`가 EPERM으로 던진다 |
| `rt-throttle` | `/proc/sys/kernel/sched_rt_runtime_us` | Warn: throttle이 busy RT 스레드를 우선순위와 무관하게 preempt한다 |
| `priority-order` | opts.priority vs `control_priority` | Fail: transport가 control 이상. transport는 control을 절대 preempt하지 않는다 |
| `cpu-online` | `/sys/devices/system/cpu/online` | Fail: 없는 코어. `apply`가 EINVAL로 던진다 |
| `cpu-isolation` | `/sys/devices/system/cpu/isolated`·`nohz_full` | Warn: pinning은 마이그레이션만 없앤다. 격리 없는 코어는 남과 공유된다 |
| `rcu-offload` | `/proc/cmdline`의 `rcu_nocbs` + `nohz_full` | Warn: 미뤄둔 커널 해제가 RT 코어에서 돈다. 시각도 길이도 안 묶인다 |
| `irq-affinity` | `/proc/irq/*/effective_affinity_list` | Warn: 핸들러는 RT 우선순위와 무관하게 그 코어의 모든 태스크를 선점한다 |
| `cpu-governor` | `cpufreq/scaling_governor` | Warn: DVFS ramp가 wakeup 지연을 늘린다. RT 코어는 performance |
| `kernel-preemption` | `/sys/kernel/realtime`·`/proc/version` | Warn: PREEMPT_RT 아님. 커널 내부 구간의 지연 상한이 없다 |
| `memory-lock` | `/proc/self/status`의 `VmLck` | Warn: 잠긴 메모리 없음. page fault는 상한 없는 지연이다 |

## 4. Python

```python doc:py_imports
import flux
import flux.ros
```

### Publisher

```python doc:py_publisher
pub = flux.ros.Publisher(node, "img", fingerprint=FP, slot_size=16 << 20, slot_count=16)

dropped = pub.dropped                           # 셋 다 property다
slot_size = pub.slot_size
segment_name = pub.segment_name
```

발행은 어댑터가 한다 -- `Cloud.build__(pub)`가 슬롯을 loan하고 `commit()`이 발행한다(2절). ndarray를 직접 넣고 빼는 표면은 [`raw_api.ko.md`](raw_api.ko.md) 4절이다.

### Subscription

```python doc:py_subscription
sub = flux.ros.Subscription(node, "img", callback=_sink, fingerprint=FP, qos=flux.QoS())

newest = sub.peek()                        # 최신, 소비 안 함. 없으면 None
nxt = sub.take()                           # 다음 것, 소비. 없으면 None
blocking = sub.take_blocking(timeout_ns=-1)  # 올 때까지. 타임아웃이면 None. Ctrl-C는 KeyboardInterrupt
lost = sub.lost                            # 전부 property다
qos = sub.qos
segment_name = sub.segment_name
domain = sub.domain                          # 이 프로세스의 domain. pub.domain도 같다

if nxt is None and not sub.can_borrow:     # 내가 view를 안 놓아서 비었다
    held = sub.refused.max_borrow          # 그 일이 몇 번 있었나
if nxt is None and not sub.attached:       # 이 domain에 발행자가 없다
    where = sub.domain                      # 다른 domain에 있는 것과 구분되는 자리
```

돌려주는 것은 읽기 전용 numpy view다. 데이터는 공유메모리를 그대로 가리킨다. 마지막 참조가 사라질 때 borrow가 풀리므로 `v = None`으로 놓거나 스코프를 벗어나게 한다.

### 빈 것이 왔을 때

`None`(C++은 빈 `FrameView`)은 두 가지를 뜻한다. 프레임이 아직 없거나, 프레임과 무관한 이유로 거절됐거나다. 후자는 `refused`가 센다 -- `lost`에도 `dropped`에도 안 잡히는 것들이다.

| `flux.Refused` 필드 | 언제 |
| --- | --- |
| `max_borrow` | 쥔 view를 안 놓고 또 불렀다. 발행이 와도 안 풀린다 |
| `holder_table` | 한 슬롯을 `kMaxHolders`(10) 프로세스가 이미 쥐고 있다 |
| `not_ready` | 세그먼트가 아직 초기화 중이다 |
| `contended` | seqlock 검증이 재시도 예산(64회)을 다 썼다 |
| `bad_frame` | meta가 슬롯 범위를 벗어났거나 그 슬롯에 커밋된 프레임이 없다 |
| `no_owner_file` | 이 프로세스가 owner 파일을 못 잡았다(fd 고갈) |
| `fence` | 실패한 fence가 리스를 `max_borrow`개 다 새게 했다. 이 소비자는 끝났다 |
| `total` | 위 일곱의 합 |

`max_borrow`만은 미리 물어볼 수 있다. `can_borrow`가 false면 그 다음 `take`는 프레임 유무와 무관하게 빈 것을 준다. `take_blocking`도 park하지 않고 즉시 돌려주므로, 여기서 재시도 루프를 돌면 CPU만 태운다 -- view를 먼저 놓아야 한다.

`fence`는 대응이 없다. 새어 나간 리스는 안 돌아오므로 view를 놓아도 안 풀리고 재시도도 무의미하다. 그 채널을 버리고 다시 열어야 한다. `max_borrow`와 갈라 세는 이유가 그것이다 -- 하나는 호출자가 고치고 하나는 못 고친다.

나머지 다섯은 재시도하면 된다. 정상 동작이다.

### 카운터를 diff할 때

`lost`와 `refused`는 둘 다 누적이지만 재접속(발행자 재시작)에서 갈린다.

| 카운터 | 재접속 때 | 왜 |
| --- | --- | --- |
| `lost` | 0으로 되돌아간다 | 새 세그먼트는 티켓을 1부터 다시 센다. 못 받은 수는 스트림에 매인 값이라 옛 스트림 것을 새 스트림에 이어 붙이면 뜻이 없다 |
| `refused` | 이어진다 | 거절은 이 소비자에게 매인 값이다. 세그먼트가 바뀐다고 없던 일이 되지 않는다 |
| `dropped` | 이어지지 않는다 | 발행자 쪽 카운터다. 세그먼트마다 따로다 |

그래서 초당 비율을 뽑을 때 `lost`의 diff는 음수가 될 수 있다. `attach_generation()`(C++)이 재접속마다 오르므로, 앞뒤 샘플의 generation이 같을 때만 diff를 비율로 쓴다. 다르면 그 구간은 비율이 아니라 재시작이다.

### GPU (CUDA)

`device="cuda"`를 붙이면 같은 채널이 GPU 경로가 된다. 안 붙이면 host 경로 그대로다.

```python doc:py_gpu_publisher
pub = flux.ros.Publisher(node, "img", fingerprint=FP, device="cuda")

loan = pub.loan((480, 640, 3), dtype="uint8")
if loan:
    render_into(cp.asarray(loan), loan.stream)   # 커널을 이 stream에 올린다
    loan.commit()                                # stream을 기다린 뒤 발행
fence_failed = pub.fence_failed
worst_commit_ns = pub.fence_wait.commit_max_ns
```

```python doc:py_gpu_subscription
sub = flux.ros.Subscription(node, "img", fingerprint=FP, qos=flux.QoS(), device="cuda")

frame = sub.take()
if frame:
    with frame as v:
        use(cp.asarray(v), v.stream)             # 스코프 종료: stream 대기 후 해제

fence_failed = sub.fence_failed
worst_release_ns = sub.fence_wait.release_max_ns
```

`device`는 `"cpu"` / `"cuda"` 문자열이나 `flux.Device.Cpu` / `flux.Device.Cuda`를 받는다. 이 호스트가 못 받는 선언은 생성자가 `ValueError`로 거절한다 -- 못 지킬 선언을 조용히 host로 내려보내지 않는다.

host 경로와 다른 곳은 셋뿐이다.

| | host | CUDA |
| --- | --- | --- |
| 생성자 | 없음 | `device="cuda"` |
| 주소 | `.array` (numpy) | `__cuda_array_interface__` -- `cp.asarray()`가 집어간다 |
| 해제 | GC | `with` 스코프 |

`loan`은 둘 다 준다. `.array`는 host view, `cp.asarray(loan)`은 같은 바이트의 device view다 -- ShmDirect에서 둘은 같은 주소이고, 어느 쪽을 잡느냐가 그 프레임을 커널이 만드는지 CPU가 만드는지가 갈린다.

`take()`/`peek()`이 돌려주는 것은 `device="cuda"`에서 numpy 배열이 아니라 `flux.Frame`이다. `with` 밖에서는 `__cuda_array_interface__`가 거절한다.

| `flux.Frame` | |
| --- | --- |
| `with frame as v` | borrow를 이 스코프에 묶는다. 나갈 때 stream을 기다린 뒤 해제한다 |
| `v.__cuda_array_interface__` | 읽기 전용 device view. 스코프 밖에서 부르면 `RuntimeError` |
| `v.__dlpack__()` · `v.__dlpack_device__()` | DLPack capsule. `torch.from_dlpack(v)`가 집어간다. 스코프 규칙은 CAI와 같다 |
| `v.bits` | 같은 바이트를 같은 폭의 unsigned numpy view로. 해석은 호출자 몫 |
| `v.stream` | 소비 커널을 올릴 stream -- 해제가 기다리는 바로 그것 |
| `v.shape` · `v.dtype` · `v.nbytes` | 프레임 모양. 스코프 밖에서도 읽힌다 |
| `v.released` | 이미 해제됐나 |

스코프를 요구하는 이유는 해제 시점이 곧 GPU 동기화 시점이기 때문이다. numpy view는 GC 수명이라 그 시점이 비결정적이고, 임의의 스레드에서 stream을 기다리게 된다. `with` 없이 `take()`를 쓰면 그 비결정성이 조용히 돌아오므로 거절한다.

`publish(array)`는 host 배열만 받는다. device 배열은 `ValueError`로 거절하고 `loan()`을 가리킨다 -- flux는 host 바이트를 슬롯에 복사하고, device 바이트를 옮기는 것은 별개의 CUDA 복사다. GPU에서 발행하는 길은 `loan()`이고 그쪽은 복사가 아예 없다.

`fence_failed`는 stream을 못 기다려서 놓지 못한 slot 수다. 0이 아니면 비율이 아니라 고장이다. `fence_wait`는 두 seam이 실제로 막힌 시간이고 `flux.FenceWait` 하나에 `commit_ns` · `commit_count` · `commit_max_ns` · `release_ns` · `release_count` · `release_max_ns`가 담긴다. host 채널에서는 둘 다 0이다.

### bfloat16

bf16은 어댑터 경로에 안 나온다 -- ROS IDL에 그 타입이 없다. numpy에 이름이 없어 경로가 갈리는 것도 원시 경로의 얘기이므로 표면 전체는 [`raw_api.ko.md`](raw_api.ko.md) 4절에 있다.

### QoS

```python doc:py_qos
qos = flux.QoS(
    depth=1, durability=flux.Volatile(), max_borrow=2, reliability=flux.Reliability.BEST_EFFORT
)

volatile = flux.Volatile()          # 붙은 뒤 발행분만
transient = flux.TransientLocal(n)  # ring에 남은 것 중 n개를 먼저 재생

kind = qos.durability               # flux.Durability. 둘 다 이 타입을 준다
live_only = kind.is_volatile
replay = kind.replay                # TransientLocal이면 n, Volatile이면 0
```

안 맞는 조합은 `ValueError`다. 표는 5절.

### Executor

ROS 콜백과 flux 콜백을 한 스레드에서 돌린다.

```python doc:py_ros_executor
ex = flux.ros.Executor()
ex.add_flux(sub)                  # 콜백은 구독이 들고 있다
ex.add_ros_node(node)             # 노드째 넘긴다. C++의 add_ros_node와 같다
ex.spin()                         # stop()까지. tick_ns 기본값 100 ms
ex.spin_once(timeout_ns=100_000_000)
merged = ex.uses_io_uring
ex.interrupt()                    # 루프는 두고 대기만 깬다
ex.stop()
ex.close()                        # 마지막 stop() 뒤, 노드를 destroy하기 전에

resolved = flux.ros.resolve(node, "img")   # 노드 네임스페이스·remap을 적용한 절대 이름
```

조립 순서와 이름이 C++과 같다. 생성하고, flux 구독과 노드를 건네고, spin한다. ROS 구독은 따로 넘기지 않는다 -- 노드에 만들면 이미 서비스된다.

`close()`는 rclpy executor에서 노드를 뗀다. 안 부르고 노드를 destroy하면 브릿지 스레드가 이미 없는 노드에 대해 task를 걸 수 있다.

노드를 하나도 안 넘기면 flux만 도는 executor다. C++에서 `add_ros_node` 없이 쓰는 것과 같고, ROS 쪽을 `rclpy.spin(node)`가 다른 스레드에서 맡는 배치(`flux_example_executor`의 `split_sub`)가 그 자리다.

rclpy는 on-new-message 콜백을 Python에 안 내준다. 그래서 C++처럼 한 링에 합치지 못한다. 대신 flux 쪽을 별도 스레드가 기다리다 `create_task`로 dispatch를 spin 스레드에 넘기고, 채널을 만지는 일은 전부 spin 스레드에서만 한다. task는 executor 자체 기능이라 `rclpy_executor=`로 어떤 rclpy executor를 넘겨도 같은 브릿지가 돈다.

### PartitionedExecutor (Python)

분할 단위마다 스레드를 하나씩 붙인다. C++판(위)은 콜백 그룹 하나가 곧 단위지만, rclpy에는 `add_callback_group`이 없다 -- executor 단위가 노드다(`add_node`가 `node.executor`를 쓴다). 그래서 단위가 둘로 갈린다.

```python doc:py_partitioned
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
ex = flux.ros.PartitionedExecutor()
ex.add_ros_node(node)                                       # 노드마다 스레드 하나 (ROS 콜백)
ex.add_flux(sub_a, MutuallyExclusiveCallbackGroup())        # 그룹마다 스레드 하나 (flux 프레임)
ex.add_flux(sub_b, MutuallyExclusiveCallbackGroup(), priority=10)
ex.spin(tick_ns=100_000_000)                                # stop()까지. 이 스레드는 콜백을 안 돈다
ex.stop()
ex.close()
```

- `add_flux`의 그룹은 ROS 엔티티를 하나도 갖지 않아야 한다. rclpy가 그룹을 자식 executor에 넘기지 못하므로, 그 그룹의 ROS 콜백은 노드 스레드에 남고 flux 프레임만 여기서 돈다 -- 그룹의 상호배제가 말없이 깨진다. spin이 거절하고, spin 중 tick이 다시 확인한다.
- ROS와 flux를 한 스레드에서 돌려야 하는 그룹은 `flux.ros.Executor`다. 그것이 그 클래스의 일이다.
- reentrant 그룹은 C++과 같은 이유로 거절한다.
- `priority`는 C++과 같다. 한 그룹에 구독이 여럿일 때 그 그룹의 pass 안 방문 순서를 정하고, 그룹을 넘지 않는다.
- 등록은 spin 전에만 한다. spin 중 `add_flux`/`add_ros_node`는 throw.
- 자식 스레드의 예외는 모든 자식을 세우고 `spin()`에서 다시 던진다.
- flux 그룹의 자식은 rclpy 브릿지 없이 `flux.Executor.spin`을 직접 돈다. 그룹에 ROS 엔티티가 없으니 합칠 것이 없다.
- 자식 스레드의 정책·우선순위·CPU는 `set_thread_scheduling`이 선언한다. C++의 `schedule`과 이름이 다른 이유는 아래 rt 절에 있다.
- 스레드가 늘어도 GIL을 놓는 일만 겹친다. numpy·zlib·디코딩은 겹치고, 순수 Python 바이트코드는 스레드가 몇이든 직렬이다.

flux 채널만 볼 때는 안쪽의 `flux.Executor`를 직접 써도 된다. rclpy가 안 끼는 표면이라 [`core_api.ko.md`](core_api.ko.md) 8절이 그것을 다룬다 -- `spin_once`·`stop`·`is_spinning`과, 다른 이벤트 루프에 끼울 때 쓰는 `wait_for_work`/`dispatch` 분리가 거기 있다.

### rt (Python)

`flux.rt`는 C++ `flux::rt`를 그대로 묶은 것이다(3절의 rt 절). 적용하는 쪽만 나와 있다 -- `preflight`·`Report`·`Strictness`는 hard RT 판정 장치라 Python 표면에 없다.

RT 보장이 아니다. 어떤 우선순위를 걸어도 GIL과 GC는 상한 없는 지연원으로 남는다. 정하는 것은 여러 스레드가 동시에 runnable일 때 커널이 누구를 고르는지와 각자 어느 코어를 쓰는지이고, 그 이상은 아니다. 그래도 정할 값이 있는 이유는 GIL을 놓는 구간에서 스레드가 실제로 병렬로 돌기 때문이다 -- numpy·zlib·디코딩이 그 구간이다.

```python doc:py_rt
ex = flux.ros.PartitionedExecutor()
ex.add_flux(sub, group)
ex.add_ros_node(node)
ex.set_thread_scheduling(group, cpus=[4, 5])
ex.set_thread_scheduling(node, policy=flux.rt.Policy.Fifo, priority=20)

report = flux.rt.apply(cpus=[4])          # 부른 스레드 자신. 자식이 아니다
state = flux.rt.current()                 # 커널에서 되읽기
klass, prio, where = state.policy, state.priority, state.cpus
tid = flux.rt.this_tid()                  # top -H와 /proc이 부르는 번호
```

- `set_thread_scheduling(unit, policy=, priority=, cpus=)`의 `unit`은 `add_flux`에 준 콜백 그룹이거나 `add_ros_node`에 준 노드다. C++은 콜백 그룹 하나지만 여기서는 단위가 둘로 갈려 있다.
- 자식이 첫 콜백 전에 자기에게 건다. 스레드 정책은 자기 스레드만 바꿀 수 있고, 거절은 `spin()`의 예외가 된다 -- 약한 설정으로 안 내려간다.
- 한 unit에 두 번 선언하면 그 자리에서 거절한다. 이 executor가 안 돌리는 unit에 건 선언은 `spin()`이 거절한다. 조용히 안 걸리는 선언을 막는 것이 이 표면의 요지다.
- `policy`는 `flux.rt.Policy`의 `Inherit`(기본, 안 건드림)·`Other`(SCHED_OTHER)·`Fifo`(SCHED_FIFO)·`RoundRobin`(SCHED_RR)이다. `Fifo`·`RoundRobin`은 `priority` 1..99와 `RLIMIT_RTPRIO` 또는 `CAP_SYS_NICE`가 필요하다. affinity만 걸 때는 권한이 필요 없다.
- `flux.rt.apply`는 부른 스레드에 건다. 건 뒤 커널에서 되읽어 대조하므로, cpuset이 마스크를 좁혀 요청이 그대로 안 앉은 경우가 성공으로 안 지나간다. 커널 거절은 `OSError`, 요청이 이 호스트에서 성립 안 하거나 되읽기가 어긋나면 `RuntimeError`, 값 범위가 틀리면 `ValueError`다. 아무것도 요청 안 하면 아무것도 안 한다.
- `apply`가 돌려주는 문자열은 호스트 판정 보고다. 사람이 읽으라고 있는 것이고 파싱할 형식이 아니다. 대부분 hard RT 항목이라 Python이 주장하지 않는 것들이고, 요청이 없으면 빈 문자열이다.
- `flux.rt.current()`는 `flux.rt.ThreadState`를 준다 -- `policy`는 `Policy`가 아니라 raw `SCHED_*` 정수다(`os.SCHED_FIFO`와 비교한다), `priority`는 정수, `cpus`는 코어 번호 목록이다.
- `flux.rt.this_tid()`는 커널 thread id다. `threading.get_ident()`가 주는 번호와 다르다.
- `flux.ros.Executor`에는 이 항이 없다. 그 executor는 부른 쪽 스레드에서 도므로 `spin()` 전에 `flux.rt.apply()`를 직접 부른다. C++ `flux::ros::Executor`도 같은 이유로 없다.

### message_filters (Python)

`import flux.ros.message_filters`. `flux.ros` 자체는 이 모듈을 import하지 않으므로, 안 쓰면 upstream `message_filters` 의존이 안 붙는다.

```python
import message_filters
import flux.ros.message_filters as fmf
from sensor_msgs_flux.image import Image

left = fmf.Subscriber(node, Image, "left", qos=flux.QoS(depth=8, max_borrow=32))
right = message_filters.Subscriber(node, sensor_msgs.msg.Image, "right")   # DDS 토픽

sync = message_filters.ApproximateTimeSynchronizer([left, right], 10, 0.02)
sync.registerCallback(lambda a, b: use(a.view().width, b.width))

ex = flux.ros.Executor()
ex.add_flux(left)      # flux 쪽은 flux executor가, DDS 쪽은 rclpy가, 같은 스레드에서
ex.add_ros_node(node)
ex.spin()
```

C++판(3절)과 같은 모양이고, 차이는 셋이다.

- 둘째 인자가 어댑터 모듈이다. `FINGERPRINT__`로 구독하고 `View`로 읽는다. `m.view()`가 그 `View`를 준다.
- 큐가 프레임 객체를 그대로 문다. Python 콜백이 받는 것은 이미 자기 borrow를 쥔 객체라 C++처럼 `take()`로 소유권을 옮길 필요가 없다.
- header 없는 스키마는 첫 프레임에서 `TypeError`다. C++은 컴파일이 멈추고, 이 언어가 멈출 수 있는 가장 이른 자리가 거기다.

주의할 값이 둘이다.

- `depth`. 기본 1은 wake마다 최신 하나만 준다. 중간 프레임이 필터에 닿지 않고 사라져 짝이 안 맞는 것으로만 보인다. synchronizer 입력에는 올린다.
- `max_borrow`와 `slot_count`. 큐에 든 프레임은 쥐고 있는 borrow다. `입력수 x queue_size`를 `max_borrow`가 덮어야 하고, 발행자의 `slot_count`도 그만큼 여유가 있어야 한다. 안 그러면 소비자는 더 못 가져오고 발행자는 쓸 슬롯이 없다.

`forwarded`가 필터로 내보낸 수, `unreadable`이 스키마에 안 맞아 버린 수다. C++과 같고 쓰임도 같다 -- synchronizer가 짝 못 찾은 것을 조용히 버리므로 `forwarded`와 사용자 콜백 횟수의 차가 그 손실이다.

`PartitionedExecutor`에서는 C++과 갈린다.

```python
ex.add_flux(left, g)
ex.add_flux(right, g)
ex.add_sync_group(left, right)   # 이 둘이 한 synchronizer의 입력이다
ex.spin()                        # 한 스레드에 안 모이면 여기서 던진다
```

- flux 입력끼리는 같은 그룹에 넣으면 된다. C++과 같다.
- flux 입력과 DDS 입력을 섞은 synchronizer는 거절한다. rclpy에 `add_callback_group`이 없어 DDS 입력을 flux 그룹의 스레드로 옮길 수단이 없다. 그 그래프의 자리는 `flux.ros.Executor`다(`contracts.ko.md` S-003).
- 순정 ROS 입력이 두 노드에 걸쳐도 거절한다. 노드마다 스레드를 주므로 그것도 갈린 배치다.
- 자리를 못 정하는 입력은 `unplaced_sync_inputs()`가 센다. 체인 중간 필터와 `Cache`가 그것이다. 0이면 선언한 입력이 전부 판정됐다.

### 열거 (도구용)

이 호스트의 flux 채널과 붙어 있는 참가자를 읽는 `flux.enumerate_topics()`·`flux.read_channel_stats()`는 노드를 안 쓴다. [`core_api.ko.md`](core_api.ko.md) 9절에 있고, 그 위에 얹힌 명령줄 도구는 [`cli.ko.md`](cli.ko.md)다.

## 5. QoS 한 장

| 값 | 기본 | 뜻 |
| --- | --- | --- |
| `depth` | 1 | 최신에서 몇 프레임까지 뒤처져도 되나. 1이면 최신만 |
| `durability` | `Volatile()` | 붙기 전 발행분을 받나. `TransientLocal(n)`이면 n개 재생 |
| `max_borrow` | 2 | 동시에 쥐는 view 수 |
| `reliability` | `BestEffort` | 유일한 값. C++ `flux::Reliability::BestEffort`, Python `flux.Reliability.BEST_EFFORT` |

거절하는 조합: `TransientLocal(n)`에서 `n > depth`, `depth == 0`, `max_borrow == 0`, reliable(C++ `flux::Reliability::Reliable`, Python `flux.Reliability.RELIABLE`). 이 값은 거절당하려고 enum에 있다. 조용히 best-effort로 낮추지 않는다.

발행자의 `slot_count`보다 깊은 QoS는 에러가 아니다. 보관된 만큼으로 잘리고 못 받은 수는 `lost`에 잡힌다.

## 6. 지켜야 할 것

- view를 오래 들고 있지 않는다. 쥔 슬롯은 발행자가 못 쓰므로 `dropped`가 오른다. 오래 필요하면 복사한다.
- 구독 하나를 여러 스레드가 동시에 쓰지 않는다. 커서와 재attach 상태가 스레드 안전하지 않다. 스레드마다 자기 구독을 만든다.
- 발행자 하나도 같다. 발행 경로가 자기 신원과 owner 파일 준비 여부를 atomic이 아닌 멤버로 들고 있어, 두 스레드가 같은 `Publisher`를 부르면 레이스다. 프로세스를 넘는 다중 발행은 이것과 별개로 안전하다 -- 그때는 객체가 각각이고 공유 링이 맡는다.
- `PartitionedExecutor`로 그룹을 쪼개면 콜백이 서로 다른 스레드에서 돈다. 다른 그룹의 콜백에서 남의 구독·발행자를 만지는 것도 위 두 줄에 걸린다 -- 카운터 조회까지 포함이다. 한 그룹 안의 콜백끼리는 스레드가 하나라 이 문제가 없다.
- ROS 타이머를 flux 구독과 한 executor에 두면 그 타이머는 flux 콜백 하나만큼 밀린다. `add`의 priority로는 그것도 못 줄인다 -- priority는 flux 채널끼리만 정렬한다. 더 줄이려면 `PartitionedExecutor`로 그 타이머를 flux 없는 그룹에 둔다. 그 그룹은 앞에 dispatch 자체가 없다.
- 같은 구독을 두 곳에서 돌리지 않는다. 두 드라이버가 스트림을 나눠 갖는다.
- 발행자와 구독자의 fingerprint를 맞춘다. 타입을 붙였으면 자동이다.
- flux Executor에 넘긴 ROS 구독을 rclcpp executor에 또 넘기지 않는다.

## 7. 지금 없는 것

- reliability(무손실)와 overflow 정책 (`docs/qos.md` 6). sub_buffer는 별도 노브가 아니다 — 뒤처짐 상한은 `depth`이고, 예약은 저 둘과 같은 막힘이다.
- 크로스 호스트. GPU 경로 셋은 다 있다 -- iGPU 직접(`ShmDirect`), 등록이 필요한 iGPU(`ShmRegistered`), dGPU 핸들(`DeviceHandle`). 어느 것을 타는지는 호스트가 정하고 표면은 같다. 3절·4절의 GPU 절이 그 표면이다.
