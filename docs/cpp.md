# C++ 사용법

`flux_cpp`는 rclcpp 노드에서 쓰는 C++ API다. 네임스페이스는 `flux::`이고 C++17이다. 채널, 슬롯, 전달 규칙은 [concepts.md](concepts.md)에 있다.

## 패키지 설정

```xml
<depend>flux_cpp</depend>
<depend>flux_std_adapters</depend>  <!-- sensor_msgs/Image 같은 표준 메시지 adapter를 쓸 때 -->
```

```cmake
find_package(rclcpp REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(flux_std_adapters REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node flux_cpp::flux_cpp rclcpp::rclcpp ${sensor_msgs_TARGETS})
target_include_directories(camera_node PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
```

| 헤더 | 내용 |
| --- | --- |
| `flux/publisher.hpp` | `flux::Publisher`, `WriteSlot`, `Published`, `DType` |
| `flux/subscription.hpp` | `flux::Subscription`, `FrameView`, `QoS` |
| `flux/executor.hpp` | `flux::Executor` |
| `flux/partitioned_executor.hpp` | `flux::PartitionedExecutor` |
| `flux/component_manager.hpp` | `flux::ComponentManager`, `flux::run_component_container`(패키지 `flux_components`) |
| `flux/message_filters/subscriber.hpp` | `flux::message_filters::Subscriber` |
| `flux/topics.hpp` | `enumerate_topics`, `read_channel_stats` |
| `sensor_msgs/flux/image.hpp` | `flux_std_adapters`의 `sensor_msgs::flux_msg::Image` adapter |

생성자의 첫 인자는 노드다. `rclcpp::Node`, `rclcpp_lifecycle::LifecycleNode`, 그 밖에 노드 interface getter를 가진 타입을 받는다. 토픽 이름은 rclcpp처럼 노드의 namespace와 remap으로 푼다.

## 발행

```cpp doc:cpp_publish
flux::Publisher pub(*node, "image");

constexpr std::size_t kHeight = 480;
constexpr std::size_t kWidth = 640;
std::vector<std::uint8_t> pixels(kHeight * kWidth * 3);
const flux::Published p = pub.publish(pixels.data(), flux::DType::U8, {kHeight, kWidth, 3});
if (flux::faulted(p)) {
  RCLCPP_ERROR(node->get_logger(), "publish: %s", flux::to_string(p));
}
```

```cpp
flux::Publisher(node, topic, fingerprint = flux::kNoSchema,
                slot_size = 8 MiB, slot_count = 16, MemoryPolicy mem = {},
                max_subscribers = 16);
```

- 기본값은 `flux.h`의 `FLUX_DEFAULT_SLOT_SIZE`, `FLUX_DEFAULT_SLOT_COUNT`, `FLUX_DEFAULT_MAX_SUBSCRIBERS`다. Python과 같은 값이다.

- `publish(data, dtype, shape)`는 배열을 슬롯에 한 번 복사한다. `publish(data, nbytes)`는 `u8[nbytes]`로 보낸다.
- 같은 채널에 살아 있는 발행자가 있으면 생성자가 `flux::Error`(`FLUX_PUBLISHER_EXISTS`)를 던진다.
- `dropped()`는 링이 가득 차서 버린 프레임 수, `slot_size()`는 슬롯 크기, `topic_name()`은 풀린 토픽 이름이다.

`publish`와 commit은 `flux::Published`를 돌려준다. `flux::faulted(p)`는 값이 `Ok`나 `Backpressure`가 아닐 때 참이다. `Backpressure`는 그 프레임 하나만 버렸다는 뜻이다. 나머지 값은 고칠 것이 있다는 뜻이다. 다만 `InUse`와, fence 상한을 둔 채널의 `FenceFailed`는 다른 호출이나 GPU 작업이 끝나면 저절로 풀린다.

| 값 | 뜻 |
| --- | --- |
| `Ok` | 발행했다 |
| `Backpressure` | 모든 슬롯을 구독자가 쥐고 있어 이 프레임을 버렸다 |
| `TooLarge` | 슬롯보다 크거나 8차원을 넘는다. 1차원이 아닌 loan의 `commit(nbytes)`, loan보다 큰 `nbytes`도 이 값이다 |
| `NotPublisher` | 발행할 수 없는 채널이다 |
| `InvalidArgument` | 알 수 없는 dtype이거나 이미 끝난 loan의 commit이다 |
| `InUse` | 다른 스레드가 같은 발행자를 쓰는 중이다 |
| `WrongDevice` | 지금은 돌려주지 않는다. 장치 메모리 채널의 `publish()`는 호스트 배열을 복사해 발행한다([gpu.md](gpu.md)) |
| `FenceFailed` | CUDA stream을 기다리지 못했다. fence 상한이 없으면 그 슬롯은 다시 쓰이지 않고, 있으면 작업이 끝난 뒤 돌아온다 |
| `GpuWedged` | 프로세스의 GPU가 복구할 수 없는 CUDA 오류로 멈췄다. 프로세스를 다시 띄운다 |

## loan으로 슬롯에 직접 쓰기

데이터를 만들 곳이 정해져 있으면 슬롯을 먼저 빌려 거기에 바로 쓴다. 이렇게 하면 복사가 없다([copy_model.md](copy_model.md)).

```cpp doc:cpp_loan
constexpr std::size_t kPoints = 1024;
flux::WriteSlot slot = pub.loan(flux::DType::F32, {kPoints, 3});
if (slot) {
  auto * xyz = static_cast<float *>(slot.data());
  std::fill_n(xyz, kPoints * 3, 0.0F);
  if (flux::faulted(slot.commit())) {
    RCLCPP_ERROR(node->get_logger(), "commit failed");
  }
}
```

- 빈 슬롯이 없으면 `WriteSlot`이 비어 있다(`false`).
- `loan()`은 shape 없이 슬롯 전체를 `u8`로 빌린다. `commit(nbytes)`는 1차원 loan을 `nbytes`로 줄여 발행한다.
- commit하지 않은 `WriteSlot`이 사라지거나 `abort()`를 부르면 프레임은 발행되지 않는다. 다만 그 슬롯에 있던 이전 프레임은 이미 덮인 뒤다.
- loan을 쥔 동안 그 슬롯은 링에서 빠진다. 오래 쥐면 `Backpressure`가 늘어난다.
- `WriteSlot`과 `FrameView`는 이동만 할 수 있고 복사할 수 없다.

## 구독과 executor

구독에 콜백을 주고 executor에 넣는다.

```cpp doc:cpp_subscribe
flux::Subscription sub(
  *node, "image", flux::kNoSchema, flux::QoS{}, [](const flux::FrameView & frame) {
    std::cout << frame.size() << " bytes, " << unsigned{frame.meta().ndim} << " dimensions\n";
  });

flux::Executor ex;
ex.add(sub);
ex.add_node(node);
ex.spin();
```

```cpp
flux::Subscription(node, topic, fingerprint = flux::kNoSchema, const QoS & qos = {},
                   Callback cb = {}, MemoryPolicy mem = {});
```

- 콜백이 받는 `FrameView`는 그 호출 동안만 유효하다. 더 오래 둘 값은 복사한다.
- QoS에 거절되는 값이 있으면 생성자가 예외를 던진다([qos.md](qos.md)).

`FrameView`의 멤버는 다음과 같다.

| 멤버 | 뜻 |
| --- | --- |
| `data()`, `size()` | 슬롯 안 프레임의 주소와 바이트 수. 읽기 전용이다 |
| `meta()` | `ndim`, `dtype`(`FLUX_DTYPE_*`), `itemsize`, `nbytes`, `shape[]` |
| `ticket()` | 채널 안 프레임 번호. 발행 순서대로 오른다 |
| `publish_ns()` | commit 시각(`CLOCK_MONOTONIC`) |
| `release()` | 슬롯을 일찍 돌려준다. 소멸자도 같은 일을 한다 |

### flux::Executor

`flux::Executor`는 `rclcpp::Executor`를 상속한다. flux 구독과 노드의 ROS 엔티티(구독, 타이머, 서비스, 클라이언트, waitable)를 대기 한 번으로 함께 기다린다. 콜백은 모두 `spin()`을 부른 스레드에서 돈다.

| 멤버 | 뜻 |
| --- | --- |
| `add(sub, priority = 0)` | 콜백이 있는 구독을 넣는다. 콜백이 없으면 `std::invalid_argument`. 구독은 executor보다 오래 살아야 하고 다른 executor에 넣지 않는다 |
| `add_node(node)` | rclcpp와 같다. 노드에 나중에 생긴 엔티티와 콜백 그룹에 넣은 flux 구독([컴포넌트 컨테이너](#컴포넌트-컨테이너))도 찾는다 |
| `spin()`, `spin_until_future_complete()` | `cancel()`, context 종료(Ctrl-C), 콜백 예외까지 돈다 |
| `cancel()` | 어느 스레드에서든 `spin()`을 끝낸다. `spin()` 전에 부르면 다음 `spin()`이 바로 끝난다 |
| `spin_once()`, `spin_some()`, `spin_all()` | 예외를 던진다. 이 함수들은 rclcpp wait set을 기준으로 예산을 세어 flux 구독을 건너뛰기 때문이다 |

- 우선순위가 큰 구독이 먼저 돈다. 같은 우선순위는 한 프레임씩 번갈아 돈다. pass 한 번에 flux 콜백을 64개까지 돌린다.
- ROS 작업이 준비되면 executor는 flux 콜백과 콜백 사이에서 flux 처리를 멈추고 ROS 작업을 돌린다. ROS 작업을 돌린 지 1 ms가 지나면, 기다리는 flux 작업에 차례를 넘긴다. 그래서 한쪽이 계속 바빠도 다른 쪽이 밀리지 않는다.
- 한 executor에 flux 구독은 127개까지다. 더 많으면 `PartitionedExecutor`로 나눈다.
- 구독이 붙을 때 거절되면(구독 프로세스 자리 부족, 메모리 부족 등) executor는 이유를 로그로 한 번 남기고 그 구독을 뺀다. 나머지 구독은 계속 돈다. 뺀 구독은 발행자가 재시작하면 다시 붙기를 시도한다.
- 콜백의 예외는 `spin()` 밖으로 나온다.
- Humble(rclcpp 28 미만)에서는 `cancel()`과 `spin_until_future_complete()`를 `flux::Executor` 타입으로 부른다. `rclcpp::Executor &`로 부르면 rclcpp의 구현이 돈다.

## executor 없이 읽기

콜백 없이 만든 구독은 직접 읽는다.

```cpp doc:cpp_take
flux::Subscription sub(*node, "image");
```

```cpp doc:cpp_take_frames
while (flux::FrameView frame = sub.take()) {
  std::cout << "frame " << frame.ticket() << ", " << frame.size() << " bytes\n";
}
if (flux::FrameView frame = sub.take_blocking(100'000'000)) {
  std::cout << "one more, " << frame.size() << " bytes\n";
}
```

- `take()`는 아직 받지 않은 가장 오래된 프레임을 준다. 없으면 빈 `FrameView`다.
- `take_blocking(timeout_ns)`는 프레임이 올 때까지 futex에서 기다린다. 음수를 주면 끝없이 기다린다. 발행자가 아직 없으면 발행자도 기다린다. 노드의 context가 끝나면 100 ms 안에 빈 결과로 돌아온다. view를 `max_borrow`만큼 쥐고 있으면 기다리지 않고 바로 빈 결과를 준다.
- executor가 한 번이라도 서비스한 구독의 `take()`와 `take_blocking()`은 `std::logic_error`를 던진다. executor와 사용자가 프레임을 나눠 갖게 되기 때문이다.

아래는 구독의 상태 값이다. 어느 스레드에서든 읽을 수 있다.

| 멤버 | 뜻 |
| --- | --- |
| `attached()` | 발행자의 세그먼트에 붙어 있다 |
| `lost()` | 받지 못하고 지나간 프레임 수. 재시작을 넘어 누적한다 |
| `refused()` | 빈 결과의 이유별 수([qos.md](qos.md)) |
| `attach_generation()` | 다시 붙을 때마다 바뀐다 |
| `qos()`, `topic_name()`, `has_callback()` | 생성 때 정한 값 |

## QoS

```cpp doc:cpp_qos
const flux::QoS qos = flux::QoS(4).transient_local(2).max_borrow(2);
flux::Subscription sub(*node, "cmd", flux::kNoSchema, qos);
```

값의 뜻은 [qos.md](qos.md)에 있다.

## 메시지 타입 붙이기

`.msg`에서 생성한 adapter는 fingerprint와 `View`·`Builder`를 준다. 만드는 방법과 접근자 규칙은 [messages.md](messages.md)에 있다.

```cpp doc:cpp_adapter_publish
flux::Publisher pub(*node, "image", Image::kFingerprint);

constexpr std::uint32_t kHeight = 480;
constexpr std::uint32_t kWidth = 640;

Image::Builder b = Image::build__(pub);
if (b) {
  const builtin_interfaces::msg::Time stamp = node->now();
  b.set__header__stamp(stamp.sec, stamp.nanosec);
  b.set__header__frame_id("camera");
  b.set__height(kHeight);
  b.set__width(kWidth);
  b.set__encoding("rgb8");
  b.set__step(kWidth * 3);
  flux::wire::Span<std::uint8_t> data = b.alloc__data(std::size_t{kHeight} * kWidth * 3);
  std::fill(data.begin(), data.end(), 0);
  if (flux::faulted(b.commit__())) {
    RCLCPP_ERROR(node->get_logger(), "image not published");
  }
}
```

여기서 `Image`는 `sensor_msgs::flux_msg::Image`다. `build__(pub)`가 슬롯을 빌리고 `Builder`가 필드를 그 슬롯에 바로 쓴다. 빈 슬롯이 없으면 `Builder`가 `false`이고 `commit__()`은 `Backpressure`를 돌려준다. `alloc__data(n)`이 주는 span은 슬롯 안을 가리킨다. 슬롯에 자리가 없으면 이 span은 비어 있다. 그러므로 `n`이 아니라 span의 `size()`만큼 채운다.

받은 프레임은 `View`로 읽는다.

```cpp doc:cpp_adapter_read
const Image::View img(frame);
const std::string_view encoding = img.encoding();
const flux::wire::Span<const std::uint8_t> data = img.data();
if (img.ok__()) {
  std::cout << img.width() << 'x' << img.height() << ' ' << encoding << ", " << data.size()
            << " bytes\n";
}
```

`View`의 접근자가 돌려주는 값은 슬롯을 가리키므로 `FrameView`가 살아 있는 동안만 유효하다. 프레임이 스키마와 맞지 않으면 `ok__()`가 `false`가 되고, 그 뒤의 접근자는 빈 값을 준다. 읽은 다음에 `ok__()`를 확인한다.

이미 ROS 메시지 객체가 있으면 `<pkg>/flux/<name>_ros.hpp`의 함수로 옮긴다. 이 함수는 필드 전체를 복사한다.

```cpp doc:cpp_ros_bridge_publish
Image::Builder b = Image::build__(pub);
sensor_msgs::flux_msg::msg_to_frame(msg, b);
if (flux::faulted(b.commit__())) {
  RCLCPP_ERROR(node->get_logger(), "image not published");
}
```

```cpp doc:cpp_ros_bridge_read
const sensor_msgs::msg::Image copy = sensor_msgs::flux_msg::frame_to_msg(Image::View(frame));
```

## PartitionedExecutor

`PartitionedExecutor`는 콜백 그룹마다 스레드 하나와 `flux::Executor` 하나를 둔다. 그래서 콜백이 느려도 그 콜백의 그룹만 늦어진다.

```cpp doc:cpp_partitioned
auto camera = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
flux::Subscription sub(
  *node, "image", flux::kNoSchema, flux::QoS{}, [](const flux::FrameView & frame) {
    std::array<char, 16> name{};
    pthread_getname_np(pthread_self(), name.data(), name.size());
    std::cout << name.data() << ": " << frame.size() << " bytes\n";
  });

flux::PartitionedExecutor ex;
ex.add(sub, camera);
ex.add_node(node);
ex.on_thread_start(camera, [] { pthread_setname_np(pthread_self(), "camera"); });
ex.spin();
```

| 멤버 | 뜻 |
| --- | --- |
| `add(sub, group, priority = 0)` | 구독을 그룹에 넣는다. 그룹은 `add_node()`로 넣은 노드의 것이어야 한다 |
| `add_node(node)` | 노드의 그룹마다 스레드를 둔다. spin 중에 생긴 그룹도 포함한다. spin 중에 부르면 그 노드의 스레드가 모두 뜬 뒤에 돌아온다. 거절할 그룹이 있으면 스레드를 하나도 띄우지 않고 예외를 던진다 |
| `remove_node(node)` | 노드의 그룹 스레드를 멈추고 join한 뒤 돌아온다. 돌아온 뒤에는 노드를 없애도 된다. 그 그룹 스레드 안에서는 부르지 않는다 |
| `on_thread_start(group, fn)` | 그 그룹의 스레드가 돌기 전에 `fn`을 그 스레드에서 부른다. 스레드 우선순위와 CPU를 정하는 자리다 |
| `on_thread_start(fn)` | 모든 그룹의 스레드에서 돌기 전에 `fn(node, group)`을 부른다. 그룹별 hook보다 먼저 돈다. 하나만 둘 수 있다 |
| `add_sync_group(inputs...)` | 한 message_filters synchronizer의 입력들을 알린다. 입력들이 여러 그룹에 흩어져 있으면 `spin()`이 거절한다 |
| `spin()` | 어느 스레드에서든 첫 오류가 나면 모든 스레드를 멈추고 그 오류를 여기서 다시 던진다 |
| `cancel()` | 어느 스레드에서든, 신호 처리기에서도 부를 수 있다 |

- Reentrant 그룹은 거절한다. 한 스레드는 그 그룹의 콜백을 동시에 돌릴 수 없기 때문이다.
- executor에 자동으로 붙지 않는 그룹(`automatically_add_to_executor_with_node = false`)은 구독을 배정받았을 때만 스레드를 받는다.
- 다른 executor가 이미 돌리는 그룹은 건드리지 않는다. rclcpp의 `add_node`도 같다. 다만 그 그룹에 `add()`로 구독을 배정했으면 `spin()`이 거절한다.
- hook의 예외는 콜백의 예외처럼 모든 스레드를 멈추고 `spin()`에서 다시 던진다.
- 그룹이 다르면 콜백이 실제로 동시에 돈다. 그룹 사이에서 공유하는 상태는 사용자가 직접 보호한다.

## 컴포넌트 컨테이너

컴포넌트 노드의 코드에서는 어떤 executor가 그 노드를 돌릴지 알 수 없다. 그래서 flux 구독을 executor에 `add()`하지 않고 콜백 그룹에 넣는다. 그 그룹을 돌리는 flux executor(`flux::Executor`, `PartitionedExecutor`, 아래 컨테이너)가 구독을 찾아 그 그룹의 스레드에서 돌린다.

```cpp doc:cpp_component
class Camera : public rclcpp::Node
{
public:
  explicit Camera(const rclcpp::NodeOptions & options)
  : Node("camera", options),
    group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    sub_(*this, "image", flux::kNoSchema, flux::QoS{}, [](const flux::FrameView & frame) {
      std::cout << "camera: " << frame.size() << " bytes\n";
    })
  {
    sub_.add_to_callback_group(*this, group_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr group_;
  flux::Subscription sub_;
};
```

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(Camera)
```

```cpp
void Subscription::add_to_callback_group(node, group = nullptr, priority = 0);
void message_filters::Subscriber<Adapter>::add_to_callback_group(node, group = nullptr, priority = 0);
```

- `group`을 비워 두면 노드의 기본 그룹에 넣는다. 이 함수는 한 번만 부른다. 같은 구독을 executor의 `add()`에 또 넣지 않는다.
- 구독은 그 그룹을 돌리는 executor보다 오래 살아야 한다. 구독을 노드의 멤버로 두면 된다. 컨테이너는 노드를 지우기 전에 그 노드의 스레드를 먼저 멈추기 때문이다. spin 중에 구독만 없애면(rclcpp 구독을 `reset()`하듯) executor가 해제된 메모리를 읽는다.
- `flux::Executor`에서는 `remove_node()` 뒤에도 그 노드에서 찾은 flux 구독이 executor가 없어질 때까지 돈다.
- rclcpp executor(rclcpp_components의 `component_container` 등)가 그 그룹을 돌리면 구독에 프레임이 전달되지 않는다. 이때 flux는 error 로그를 한 번 남긴다.
- 한 message_filters synchronizer의 입력은 같은 그룹에 넣는다.

### 컨테이너 실행

`flux_components` 패키지의 `component_container`는 rclcpp_components의 컨테이너와 같은 서비스(`~/_container/load_node`, `unload_node`, `list_nodes`)를 제공한다. 그래서 `ros2 component load`와 launch의 `ComposableNodeContainer`를 그대로 쓸 수 있다.

```bash
ros2 run flux_components component_container --ros-args -r __node:=perception_container
ros2 component load /perception_container my_pkg my_pkg::Camera
```

```python
ComposableNodeContainer(
    name="perception_container",
    namespace="",
    package="flux_components",
    executable="component_container",
    composable_node_descriptions=[ComposableNode(package="my_pkg", plugin="my_pkg::Camera")],
)
```

- 로드한 노드의 콜백 그룹마다 스레드 하나(`flux-g<N>`)가 돈다. 컨테이너 자신의 서비스도 자기 그룹의 스레드에서 돈다.
- 컨테이너는 Reentrant 그룹이 있는 노드의 로드를 거절한다. 응답의 `error_message`에 노드와 이유가 있다. 로드한 뒤에 Reentrant 그룹이 생기면 컨테이너가 멈춘다.
- 자동으로 붙지 않는 그룹(`automatically_add_to_executor_with_node = false`)과 다른 executor가 이미 돌리는 그룹은 건드리지 않는다. `use_sim_time`의 `/clock` 그룹이 그 예다. 이 그룹은 rclcpp의 TimeSource가 자기 스레드에서 돌린다.
- 로드한 뒤에 생긴 그룹은 100 ms 안에 스레드를 받는다.
- unload는 그 노드의 스레드를 모두 멈춘 뒤 노드를 지운다.
- 프로세스 이름(`/proc/<pid>/comm`)은 컨테이너 노드 이름의 앞 15바이트다. 그래서 컨테이너가 여럿이어도 `ps`나 스케줄러에서 구별된다.
- 콜백의 예외는 컨테이너를 끝낸다.

### 컨테이너 확장

`flux::ComponentManager`를 상속해 hook을 override한다. 이 hook에서 그룹마다 스레드 우선순위와 CPU를 정한다.

```cpp
class MyManager : public flux::ComponentManager
{
protected:
  void on_node_added(const NodeBase::SharedPtr & node) override;
  void on_thread_start(
    const NodeBase::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & group) override;
};

int main(int argc, char ** argv)
{
  return flux::run_component_container<MyManager>(argc, argv);
}
```

| 멤버 | 뜻 |
| --- | --- |
| `ComponentManager(node_name = "ComponentManager", options = ...)` | 기본 `options`는 rclcpp_components의 매니저와 같다 |
| `on_node_added(node)` | 노드를 로드하는 스레드에서, 그 노드의 스레드가 뜨기 전에 부른다. 예외를 던지면 로드를 거절한다 |
| `on_thread_start(node, group)` | 각 그룹의 스레드에서 첫 콜백 전에 부른다. 컨테이너 자신의 그룹과 나중에 생긴 그룹도 포함한다. 예외를 던지면 컨테이너가 멈춘다 |
| `spin()`, `cancel()` | 컨테이너를 돌리고 멈춘다. `cancel()`은 신호 처리기에서도 부를 수 있다. `spin()`은 매니저가 사라지기 전에 돌아와야 한다 |
| `run_component_container<Manager>(argc, argv)` | `rclcpp::init`, `Manager` 생성, 프로세스 이름 설정, `spin()`, `rclcpp::shutdown`을 하는 `main`이다. 종료 코드를 돌려준다. `Manager`는 인자 없이 만들 수 있어야 한다 |

```cmake
find_package(flux_components REQUIRED)
add_executable(my_container src/my_container.cpp)
target_link_libraries(my_container flux_components::flux_components)
```

## message_filters

flux 토픽을 message_filters의 입력으로 쓴다. flux 토픽끼리 묶어도 되고, flux 토픽과 ROS 토픽을 섞어도 된다.

```cpp doc:cpp_message_filters
using Input = flux::message_filters::Subscriber<Image>;
using Frame = std::shared_ptr<const Input::Message>;
using Policy = message_filters::sync_policies::ApproximateTime<Input::Message, Input::Message>;

Input left(*node, "left/image", 10);
Input right(*node, "right/image", 10);
message_filters::Synchronizer<Policy> sync(Policy(10), left, right);
const auto on_pair = [](const Frame & l, const Frame & r) {
  std::cout << "pair at " << l->header.stamp.sec << " s: " << l->view().width() << ", "
            << r->view().width() << '\n';
};
sync.registerCallback(std::bind(on_pair, std::placeholders::_1, std::placeholders::_2));

flux::Executor ex;
ex.add(left);
ex.add(right);
ex.spin();
```

- `Subscriber<Adapter>(node, topic, queue, qos = {})`의 `queue`는 synchronizer나 Cache의 큐 크기다.
- 큐에 들어가는 것은 바이트가 아니라 `StampedFrame`이다. `StampedFrame`은 슬롯을 쥔 프레임과 header(stamp, `frame_id`)를 담고, `view()`로 adapter의 `View`를 준다. 프레임 바이트는 복사하지 않고 header(`frame_id` 포함)만 복사한다.
- 큐에 든 프레임은 슬롯을 쥔다. 그래서 `Subscriber`는 `max_borrow`를 `queue + 1` 이상으로 올리고, 슬롯이 `queue`개 이하인 발행자는 거절한다. 생성자에서 붙을 때는 `flux::Error`를 던진다. executor 아래에서는 로그를 한 번 남기고, 발행자가 재시작할 때까지 그 입력을 뺀다.
- adapter에 `std_msgs/Header header`가 있어야 한다. header를 읽지 못하거나 stamp가 음수인 프레임은 넘기지 않고 `unreadable()`로 센다. `forwarded()`는 넘긴 수다.
- 한 synchronizer의 입력은 한 스레드에서 돌아야 한다. `flux::Executor`에서는 늘 그렇다. `PartitionedExecutor`에서는 입력을 같은 그룹에 넣고 `add_sync_group()`으로 알린다.

## 오류

생성자와 설정 함수는 `flux::Error`(`std::runtime_error`)를 던진다. `status()`가 이유를 알려 준다.

| `status()` | 뜻 |
| --- | --- |
| `FLUX_INVALID_ARGUMENT` | 잘못된 인자(이름, QoS, 슬롯 수 등) |
| `FLUX_MISMATCH` | 채널이 이 요청과 맞지 않는다. 불러온 `libflux_ffi`의 ABI 버전이 헤더와 다를 때도 이 값이다 |
| `FLUX_NOT_READY` | 아직 발행자가 없다 |
| `FLUX_PUBLISHER_EXISTS` | 같은 채널의 발행자가 이미 있다 |
| `FLUX_SUBSCRIBERS_FULL` | 구독 프로세스 자리가 모두 찼다 |
| `FLUX_UNSUPPORTED` | 이 호스트에서 할 수 없다(GPU 경로 없음 등) |
| `FLUX_IO` | 시스템 호출이 실패했다 |
| `FLUX_IN_USE` | 다른 스레드가 같은 객체를 쓰는 중이다 |

## 채널 목록

```cpp doc:cpp_enumerate
for (const flux::Topic & t : flux::enumerate_topics()) {
  const flux::ChannelStats s = flux::read_channel_stats(t.signpost);
  std::cout << t.key << ": " << (s.live ? "up" : "down") << ", " << t.endpoints.size()
            << " endpoints\n";
}
```

목록에는 현재 사용자가 열 수 있는 채널만 나온다. 이 함수들은 읽기만 하므로 대상 프로세스에 영향이 없다. 같은 정보를 `flux` 명령이 보여 준다([cli.md](cli.md)).
