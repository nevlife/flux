# ROS 2 도구

flux 채널은 DDS의 ROS 그래프에 없으므로 기본 ROS 2 도구(`ros2 topic`, `ros2 bag`, rqt, rviz2)로는 확인 불가능하다. `ros2/` 아래의 세 패키지가 flux 채널을 이 도구들과 연결한다.

| 패키지 | 하는 일 | 복사 |
| --- | --- | --- |
| `flux_bridge` | flux 채널을 DDS 토픽으로 다시 발행한다 | 프레임마다 1, 그리고 CDR 쓰기와 DDS 전달 |
| `flux_bag` | flux 채널을 rosbag2 bag에 녹화하고 flux로 재생한다 | 프레임마다 1. DDS를 거치지 않는다 |
| `flux_tools` | rviz2 디스플레이. 공유 메모리에서 바로 읽는다 | GPU 업로드 1 |

bridge 없이 발행자가 직접 ROS 2 토픽도 발행 가능하다([발행자의 ROS 2 mirror](#발행자의-ros-2-mirror)).

## flux_bridge

```bash
ros2 run flux_bridge bridge [--poll SEC]
```

bridge는 노드 하나(`flux_bridge_<pid>`)를 생성하고, `--poll` 간격(기본 1 s)마다 채널 목록을 읽고, 아래 조건이 모두 맞는 채널만 중계한다. 그래서 구독자가 없는 채널에는 비용이 없다.

- 발행자가 살아 있고 이름이 정확히 알려진 채널이다.
- 채널의 fingerprint가 설치된 `.msg` 타입(`AMENT_PREFIX_PATH`)과 맞거나, 그 타입의 GPU 메모리 채널([gpu.md](gpu.md#gpu-메모리-채널devicehandle))이다. 어댑터를 설치하지 않은 타입도 된다.
- bridge 외에 그 이름의 ROS 2 구독자가 있다.
- bridge 외에 그 이름의 ROS 2 발행자가 없다(발행자의 mirror 포함). 둘 다 발행하면 ROS 2 구독자가 같은 프레임을 두 번 받는다.

| 항목 | 값 |
| --- | --- |
| 프레임 경로 | 슬롯에서 프레임을 복사하고 view를 반납한 뒤, 그 사본에서 CDR bytes를 써서 `GenericPublisher`로 발행한다. 메시지 객체를 거치지 않는다. 필드를 arena에 저장한 프레임([concepts.md](concepts.md#arena))은 view를 반납하기 전에 그 필드까지 사본에 복사한다. GPU 메모리 채널은 aux와 payload의 호스트 사본을 한 메시지로 합친다 |
| DDS QoS | depth 1, best-effort, volatile |
| flux QoS | depth 1, `max_borrow` 1 |
| 같은 이름의 채널 둘 | fingerprint가 다른 채널이 둘이면 signpost 순서로 첫 채널을 고른다 |
| 하지 않는 것 | DDS -> flux 방향, 서비스, 액션, 파라미터 |

### ros2 topic 하위 명령

```bash
ros2 topic echo_flux /camera/image --no-arr
ros2 topic hz_flux /camera/image
```

기본 `echo`, `hz`는 bridge의 다음 poll 시점까지 토픽을 찾지 못한다. 그래서 두 하위 명령은 콜백이 비어 있는 구독을 먼저 생성하고, 그 토픽의 발행자(bridge나 발행자의 mirror)가 ROS 그래프에 나타나는 시점까지 기다린 뒤 기본 하위 명령을 실행한다.

- 인자는 기본 하위 명령의 인자와 `--bridge-timeout SEC`(기본 10)다.
- 메시지 타입은 flux가 채널의 fingerprint로 찾으므로 DDS discovery를 기다리지 않는다.
- 두 하위 명령을 사용하려면 bridge가 실행 중이거나 발행자가 mirror를 활성화해야 한다.

## 발행자의 ROS 2 mirror

```cpp
auto pub = flux::create_publisher(*node, "image", flux::PublisherOptions(), Image::kFingerprint);
pub->mirror_to_ros();
```

```python
pub = flux.create_publisher(
    node, "image", fingerprint=Image.FINGERPRINT__, mirror_to_ros=True
)
```

mirror를 활성화하면 발행자가 같은 이름의 ROS 2 토픽도 발행한다. 일반 ROS 2 노드와 기본 도구가 그 토픽을 구독 가능하고, flux 구독자의 경로는 바뀌지 않는다.

| 항목 | 값 |
| --- | --- |
| 프레임 경로 | 발행자 프로세스 안의 스레드가 자기 채널을 읽는다. 이후는 bridge와 같다. 필드를 arena에 저장한 프레임도 발행한다 |
| 발행 경로 | `publish()`와 commit은 변환을 기다리지 않는다 |
| ROS 2 구독자가 없는 경우 | flux 구독을 해제하고 100 ms마다 구독자 수만 확인한다. 구독자 번호(`max_subscribers`)를 차지하지 않는다 |
| ROS 2 구독자가 있는 경우 | flux 구독자 하나로 연결되므로 구독자 번호 하나를 사용한다. 발행자의 `subscriber_count`에도 1로 들어간다 |
| DDS QoS | depth 1, best-effort, volatile |
| flux QoS | depth 1, `max_borrow` 1 |
| 생성 비용 | 활성화 시 설치된 `.msg`를 모두 읽는다 |
| 거절하는 것 | 타입이 없는 발행자(`kNoSchema`, `NO_SCHEMA`), fingerprint가 설치된 `.msg`에 없는 발행자, 설치된 타입의 GPU 메모리 채널이 아닌 GPU 메모리 발행자. C++은 `flux::Error`, Python은 `flux.Error`(`INVALID_ARGUMENT`)다 |
| Python | 스레드는 `threading.Thread`이고 rclpy 발행자에 `bytes`로 발행한다. 발행자 객체가 사라지면 스레드도 종료된다 |

FHD 이미지처럼 큰 메시지를 여러 토픽으로 보내면 DDS의 UDP 수신 버퍼가 넘쳐 best-effort 메시지가 버려질 가능성이 있다. bridge도 같다. `netstat -su`의 `receive buffer errors`가 늘어나면 rmw 설정에서 소켓 버퍼를 늘린다(Cyclone DDS는 `CYCLONEDDS_URI`의 `SocketReceiveBufferSize`, 상한은 `net.core.rmem_max`).

## flux_wrapper

드라이버가 flux 없이도 빌드되고 실행되게 하는 발행, 구독 API다. 드라이버의 `package.xml`에는 `flux_wrapper`만 넣는다. `flux_wrapper`가 `ENABLE_FLUX == 1` 조건으로 flux에 의존한다.

| 스위치 | 시점 | 뜻 |
| --- | --- | --- |
| 빌드 | `flux_wrapper` 빌드 시 `ENABLE_FLUX=1` | flux 경로를 컴파일하고 `USE_FLUX_ENABLED`를 정의한다. flux를 찾았는지로 정하지 않는다. 이 값이 없으면 `colcon build --packages-up-to <드라이버>`가 flux를 빌드하지 않는다 |
| 실행 | 프로세스의 환경에 `ENABLE_FLUX=1` | 그 프로세스의 발행자와 구독을 모두 flux로 보낸다. flux 구독자는 ROS 경로의 발행자에게서 받지 못하므로 launch 전체에 같은 값을 준다 |

```cpp
#include <flux_wrapper/flux_wrapper.hpp>

flux_wrapper::Publisher<sensor_msgs::msg::Image> pub(*node, "image", rclcpp::QoS(1));
auto loan = pub.loan();
loan.set_header(header);
loan.set_height(h);
loan.set_width(w);
loan.set_encoding("rgb8");
loan.set_step(w * 3);
auto data = loan.alloc_data(h * w * 3);  // flux 슬롯이나 ROS 메시지에 바로 쓴다
pub.publish(std::move(loan));

flux_wrapper::Subscription<sensor_msgs::msg::Image> sub(
  *node, "image", rclcpp::QoS(1), [](const flux_wrapper::View<sensor_msgs::msg::Image> & v) {
    // v.data()는 flux 경로에서 슬롯을 가리킨다. 콜백 밖에서 사용할 값은 v.to_msg()로 복사한다
  });
```

### 메시지 타입

flux 어댑터가 있는 메시지 타입은 모두 사용 가능하다. 표준 타입([messages.md](messages.md#표준-메시지))의 어댑터는 wrapper가 include한다. 다른 패키지의 메시지는 그 패키지가 `flux_generate_adapters()`로 어댑터를 생성하고([messages.md](messages.md#flux-없이도-빌드되는-메시지-패키지)), 드라이버가 그 ROS bridge 헤더를 include한다. flux 없이 빌드하면 이 헤더가 없으므로 `USE_FLUX_ENABLED` 아래에 둔다.

```cpp
#include <flux_wrapper/flux_wrapper.hpp>
#include <my_msgs/msg/foo.hpp>
#ifdef USE_FLUX_ENABLED
#include <my_msgs/flux/foo_ros.hpp>
#endif

flux_wrapper::Publisher<my_msgs::msg::Foo> pub(*node, "foo", rclcpp::QoS(1));
auto loan = pub.loan();
loan.set_from(fields);           // data를 뺀 필드를 쓴다
auto data = loan.alloc_data(n);  // data는 flux 슬롯이나 ROS 메시지에 바로 쓴다
pub.publish(std::move(loan));

flux_wrapper::Subscription<my_msgs::msg::Foo> sub(
  *node, "foo", rclcpp::QoS(1), [](const flux_wrapper::View<my_msgs::msg::Foo> & v) {
    const my_msgs::msg::Foo f = v.without_data();  // data를 읽지 않고 나머지 필드를 복사한다
    // v.data()는 flux 경로에서 슬롯을 가리킨다
  });
```

| 타입 | Loan | View |
| --- | --- | --- |
| `sensor_msgs/Image`, `sensor_msgs/PointCloud2` | 필드마다 setter, `set_from(msg)` | 필드마다 접근자, `without_data()`, `to_msg()` |
| 그 밖의 타입 | `set_from(msg)` | `without_data()`, `to_msg()` |
| 최상위에 `std_msgs/Header header`가 있는 타입 | `set_header(h)` | `stamp()`, `frame_id()` |
| 최상위에 `uint8[] data`나 `byte[] data`가 있는 타입 | `alloc_data(n)`, `alloc_buffer(n)` | `data()`, `size()`, `on_device()`, `device_ptr()`, `stream()`, `after()` |

- 이 문서에서 `data`는 최상위의 `uint8[] data`나 `byte[] data`다. 다른 이름이나 다른 원소 타입의 배열, 중첩된 메시지 안의 `data`는 다른 필드와 같다.
- `set_from(msg)`는 `data`를 뺀 필드를 한 번에 쓰고 `data`는 그대로 둔다. `data`가 없는 타입은 모든 필드를 쓴다. flux 경로에서는 바로 슬롯에 쓰므로, 슬롯에 들어가지 않는 필드가 있으면 loan이 `false`가 된다. `alloc_data(n)`과 호출 순서는 상관없다.
- loan이 `false`면 `alloc_data(n)`은 빈 span을, `alloc_buffer(n)`은 `false`인 `Buffer`를 반환한다. 빈 슬롯이 없을 때, 앞서 쓴 필드나 `data`가 슬롯이나 aux에 들어가지 않았을 때, 발행자 없이 만든 loan일 때다. publish가 보내지 않을 `data`를 쓰지 않게 하기 위해서다. Python은 둘 다 `None`이다.
- `without_data()`는 `data`를 비운 메시지를 반환한다. 두 경로 모두 `data`를 읽거나 복사하지 않는다. ROS 경로에서는 메시지의 introspection type support로 필드를 복사한다. `data`가 없는 타입은 `to_msg()`와 같다.
- 타입에 해당 필드가 없으면 그 멤버는 컴파일되지 않는다. 예로 `data`가 없는 타입의 `alloc_data()`와 header가 없는 타입의 `stamp()`다.
- flux로 빌드한 wrapper에서 어댑터를 include하지 않은 타입을 쓰면, `static_assert`가 include할 헤더(`<pkg>/flux/<snake>_ros.hpp`)를 알려 준다.

### 발행

- `publish(std::unique_ptr<Msg>)`와 `publish(const Msg &)`는 이미 만든 ROS 메시지를 보낸다. flux 경로에서는 슬롯과 arena로 한 번 복사한다. 메시지를 계속 사용하는 쪽은 `const Msg &`를 사용한다. ROS 경로에서 복사본을 만들지 않는다.
- `publish()`는 `flux_wrapper::PublishResult`를 반환한다. 메시지가 나갔으면 참이고(`if (!pub.publish(m))`), 아니면 `status()`가 이유(`Published::Backpressure`는 빈 슬롯 없음, `TooLarge`는 필드가 슬롯이나 aux에 들어가지 않음, `DeviceError`는 CUDA 복사 실패나 이미지의 step과 슬롯 pitch 불일치 등)다. 결과를 버리면 컴파일러가 경고한다. Python은 `PublishResult`(참과 거짓, `status`는 `"backpressure"` 같은 문자열)다.
- mirror는 발행자에 준 QoS로 ROS 2 토픽을 발행한다. ROS 경로와 같으므로, 기본 QoS(reliable)로 만든 일반 ROS 노드도 받는다. QoS의 depth만큼 mirror가 변환이 밀린 프레임을 기다린다.
- `get_subscription_count()`는 ROS 경로에서 rclcpp 값을, flux 경로에서 `subscriber_count()`를 반환한다.
- `PublisherOptions::alloc`(Python `alloc="arena"|"slot"`)은 flux 경로의 `Cpu` 발행자에만 적용된다. 기본 `Arena`는 `data`를 발행 프로세스의 arena에 저장한다. `Slot`은 `data`까지 모두 슬롯 안에 두고, 슬롯(기본 4 MiB)에 들어가지 않는 메시지는 보내지 않는다. Jetson 같은 통합 GPU에서 `Device::Cuda` 구독은 두 발행자의 `data`를 모두 복사 없이 GPU에서 읽는다. `Arena` 발행자의 `data`는 구독 프로세스가 arena를 GPU에 등록해서 읽는다([gpu.md](gpu.md)).
- `flux_wrapper::create_publisher<Msg>(node, topic, qos, options)`와 `flux_wrapper::create_subscription<Msg>(node, topic, qos, callback, options)`는 `rclcpp`처럼 `shared_ptr`를 반환한다. 생성자도 그대로 쓸 수 있다.
- `PublisherOptions`의 `slot_size`, `slot_count`, `max_slot_size`, `max_slot_count`, `growth`(C++ `flux_wrapper::Growth::PowerOfTwo`, `BestFit`, `Static`, Python `"power_of_two"`, `"best_fit"`, `"static"`), `mirror_to_ros`는 flux 경로에만 적용되고, flux 발행자에게 그대로 넘어간다([concepts.md](concepts.md)). 0이면 flux의 기본값이다. `Arena` 발행자의 슬롯 크기는 타입에서 정하고, `data`는 크기와 상관없이 발행 프로세스의 arena에 저장한다. `Cuda` 발행자는 슬롯에 `data`를 담으므로 `slot_size`가 `data`의 최대 크기다(기본 4 MiB, 최대 `max_slot_size`). `alloc_on_subscriber`는 `Slot`이나 `Cuda` 발행자가 구독자가 처음 붙은 뒤에 슬롯을 잡게 하고, 그 전의 `publish()`는 `false`다. `image_slots`(C++ `ImageSlots::Auto`, `Off`, `On`, Python `"auto"`, `"off"`, `"on"`), `image_encoding`(`"bgra8"`, `"nv12"` 등 sensor_msgs/Image 인코딩), `image_width`, `image_height`는 flux 발행자의 이미지 슬롯 설정으로 그대로 넘어간다([gpu.md](gpu.md)). 형식은 `Cuda` 발행자만 받는다. 호스트 메시지는 헤더가 슬롯 앞에 오므로 이미지가 배치가 말하는 자리에 있지 않다. `step`은 배치의 행 간격(256바이트 배수)이어야 한다. 다르면 `publish()`가 `false`를 반환하고, 처음 한 번 두 값을 오류 로그로 남긴다. 메시지 자신의 문자열(`header.frame_id`, `encoding`)이 슬롯을 넘으면 publish가 `false`를 반환하고, 발행자마다 처음 한 번 그 필드와 크기를 오류 로그로 남긴다. `slot_size`가 0인 `Slot`이나 `Cuda` 발행자는 그 뒤 `growth`대로 슬롯을 키우고, 다음 메시지부터 큰 슬롯으로 보낸다. `Image`와 `PointCloud2`가 아닌 타입은 `set_from()`이 필드를 한 번에 쓰므로 필드 이름은 남기지 않고, 슬롯(`Cuda`면 aux)의 크기와 그 앞까지 쓴 크기를 남긴다. mirror는 기본 비활성화이고, 그 상태에서는 flux 경로의 토픽을 rviz2나 `ros2 bag`이 받지 못한다.

### 구독

- 구독은 `SubscriptionOptions`로 callback group을 받는다. flux 경로의 구독은 그 그룹을 실행하는 flux executor가 처리한다. `flux_wrapper::Executor`는 flux로 빌드했으면 `flux::Executor`, 아니면 `rclcpp::executors::SingleThreadedExecutor`다. `flux::Executor`의 `spin_some()`과 `spin_once()`는 예외를 던지므로 `spin()`을 쓴다. 그룹마다 스레드를 두는 `flux::PartitionedExecutor`는 flux 빌드에만 있다. rclcpp 실행기와 API가 달라 wrapper에 별칭을 두지 않는다.
- `View`는 복사할 수 없다. 복사본이 슬롯보다 오래 남지 않게 하기 위해서다. 콜백 뒤에도 쓰려면 아래의 `HeldCallback`을 쓴다.
- 콜백이 `std::shared_ptr<const View<Msg>>`를 받으면(`HeldCallback`) 콜백 뒤에도 View를 보유 가능하다. 포인터가 살아 있는 동안 View가 읽는 메시지가 남고, flux 경로에서는 슬롯도 남는다. 마지막 포인터가 사라지는 시점에 반납되며, 반납하는 스레드는 무관하다.
- `SubscriptionOptions::max_borrow`(기본 1)는 한 구독에서 동시에 살아 있는 View의 최대 수다. 콜백 안의 View와 보유한 View를 합한다. 0이면 생성자가 `std::invalid_argument`를 던진다.
- flux 경로에서는 View를 `max_borrow`개 보유한 동안 다음 프레임이 발행자의 링에서 기다린다. 슬롯이 `max_borrow`개 이하인 발행자는 거절한다. 생성자에서 연결 시에는 `flux::Error`를 던지고, executor 아래에서는 로그를 한 번 남기고 발행자 재시작 시까지 그 구독을 제외한다.
- 보유된 슬롯에는 발행자가 쓰지 않는다. 프레임이 버려지지 않으려면 발행자의 `slot_count`가 같은 토픽의 모든 구독이 동시에 점유하는 수보다 1 이상 커야 한다.
- ROS 경로에서 `max_borrow`는 `Cuda` 구독의 GPU 버퍼 수만 정한다. 버퍼가 모두 점유되어 있으면 그 메시지를 건너뛰고 오류 로그를 한 번 남긴다. Reentrant 그룹에서 콜백이 `max_borrow`개보다 많이 겹치는 경우도 같다. 호스트 구독에는 제한이 없다.

### message_filters

`flux_wrapper::message_filters::Subscriber<Msg>`(`flux_wrapper/message_filters.hpp`)는 토픽 하나를 `message_filters`의 synchronizer나 `Cache`에 넣는다. flux 경로인지 ROS 경로인지는 `Subscription`과 같이 정해지므로, 같은 코드가 flux를 넣은 빌드와 뺀 빌드에서 모두 동작한다. 최상위 `std_msgs/Header header`가 있는 타입만 받는다.

```cpp
#include <flux_wrapper/message_filters.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using flux_wrapper::message_filters::Stamped;
using Policy = message_filters::sync_policies::ApproximateTime<Stamped<Image>, Stamped<Image>>;
using Ptr = std::shared_ptr<const Stamped<Image>>;

flux_wrapper::message_filters::Subscriber<Image> left(node, "left", rclcpp::QoS(10), 10);
flux_wrapper::message_filters::Subscriber<Image> right(node, "right", rclcpp::QoS(10), 10);
message_filters::Synchronizer<Policy> sync(Policy(10), left, right);
sync.registerCallback(std::function<void(const Ptr &, const Ptr &)>(
  [](const Ptr & l, const Ptr & r) { use(l->view->data(), r->view->data()); }));
```

- synchronizer가 받는 메시지는 `Stamped<Msg>`다. `header`는 메시지 header의 사본이고, `view`는 `std::shared_ptr<const View<Msg>>`다(`HeldCallback`과 같다). flux 경로에서는 `view`가 살아 있는 동안 슬롯을 보유한다.
- 네 번째 인자는 synchronizer나 `Cache`의 큐 크기다. 큐에 든 메시지가 View를 보유하고 필터는 버리기 전에 새 것을 넣으므로, `max_borrow`를 큐 크기 + 1로 올린다. flux 경로에서는 슬롯이 그보다 많지 않은 발행자를 거절한다.
- stamp가 음수인 메시지는 ROS 시각이 아니어서 필터가 예외를 던진다. 그래서 넘기지 않고 `unreadable()`로 센다. 넘긴 수는 `forwarded()`다.
- 일반 ROS 구독(`message_filters::Subscriber<Imu>` 등)과 같은 synchronizer에 함께 넣을 수 있다.
- Python은 `flux_wrapper.message_filters.Subscriber(node, msg_type, topic, qos, queue_size)`다. synchronizer가 받는 메시지는 `header`와 `view`를 가진 `Stamped`이고, 구독은 `flux_wrapper.Executor`에 `add()`한다.

### GPU(C++)

- `PublisherOptions::device = flux_wrapper::Device::Cuda`면 메시지의 `data`를 GPU 메모리째 보낸다. `data`가 있는 타입만 지원한다. 없는 타입은 발행자와 구독 모두 토픽 이름과 함께 경고를 남기고 `Cpu`가 된다. `loan.alloc_buffer(n)`이 데이터 자리를 할당하고, `on_device()`가 GPU 메모리인지 알려 준다. `copy_from(src, n)`은 원본이 호스트 메모리든 GPU 메모리든 받는다. 커널이 직접 쓰는 경우에는 `device_ptr()`와 `stream()`을 사용하고, publish가 그 스트림의 작업이 완료된 뒤 발행한다.
- flux 경로에서는 `data`를 payload로, 나머지 필드를 aux로 보내는 GPU 메모리 채널([gpu.md](gpu.md#gpu-메모리-채널devicehandle))로 보낸다. ROS 경로에서는 wrapper가 GPU 버퍼를 하나 생성하고, publish 시 호스트로 복사해 ROS 메시지로 보낸다.
- `data` 밖의 필드는 aux 4096 B에 들어가야 한다. 넘으면 loan이 `false`가 되고 publish가 `false`를 반환한다. 발행자마다 처음 한 번, 들어가지 않은 첫 필드와 그 크기, 그 앞까지 쓴 크기, 메시지의 `header.stamp`를 오류 로그로 남긴다.

- `Cuda`를 사용 불가능하면(CUDA 없이 빌드했거나, GPU 경로나 GPU가 없으면) 그 발행자나 구독이 토픽 이름과 함께 경고를 남기고 `Cpu`로 바뀐다. 발행자와 구독의 `uses_device()`가 실제로 사용하는 쪽을 알려 준다. `mirror_to_ros`를 켜면 mirror가 ROS 2 구독자가 있는 동안 `data`를 호스트로 복사해 발행한다.
- 구독은 flux 경로에서 `Cuda` 발행자의 프레임도 받는다. `SubscriptionOptions::device`가 `Cpu`(기본)면 `data()`가 호스트 메모리다(외장 GPU에서는 take 시 한 번 복사한다). aux가 잘못된 프레임은 건너뛰고 `rejected()`로 센다.
- 구독의 `device`가 `Cuda`면 발행자가 어느 쪽이든 `data`가 GPU 메모리에 있다. `on_device()`, `device_ptr()`, `size()`, `stream()`으로 읽고, GPU 작업은 `stream()`에 올린다. `Cpu` 발행자의 프레임은 외장 GPU에서 take 시 GPU로 한 번 복사하고, 통합 GPU에서는 복사하지 않는다. 호스트 메모리에도 있는 프레임은 `data()`로도 읽힌다.
- ROS 경로의 `Cuda` 구독은 구독마다 GPU 버퍼를 `max_borrow`개 생성하고, 메시지 수신마다 빈 버퍼로 복사한다. 버퍼마다 스트림이 따로 있다.
- GPU 메모리 프레임을 `stream()`이 아닌 스트림 `s`에서 읽으면, 반납 전에 `View::after(s)`를 호출한다. `stream()`이 `s`에 지금까지 올린 작업 뒤에 오게 해서, 슬롯이나 GPU 버퍼가 그 작업이 완료된 뒤 반납된다. 호출하지 않으면 그 작업이 완료되기 전에 발행자가 슬롯에 새 프레임을 쓸 가능성이 있다.
- GPU 기능은 CMake가 CUDAToolkit을 찾은 경우에 컴파일된다. 찾지 못해도 `Device::Cuda`, `stream()`, `after()`는 있으므로 코드는 그대로 빌드된다. 그 빌드에서 `stream()`은 null이고 `after()`는 참이다.
- 이미지 형식을 준 `Cuda` 발행자가 `Cuda`를 쓰지 못하면, 경고를 남기고 형식을 버린 `Cpu` 발행자가 된다. `Cpu`를 요청하면서 형식을 주면 생성자가 `flux::Error`를 던진다.

### Python

- Python 모듈 `flux_wrapper`도 같은 구조다. 다만 `HeldCallback`은 없다. `Subscription(..., max_borrow=n)`은 C++의 `SubscriptionOptions::max_borrow`와 같다. `create_publisher(node, msg_type, topic, qos, **options)`, `create_subscription(node, msg_type, topic, qos, callback, **options)`는 C++과 같은 순서이고, 클래스 생성자 `Subscription(node, msg_type, topic, callback, qos, ...)`는 rclpy 순서다. `ENABLE_FLUX=1`이고, 패키지를 flux와 함께 빌드했고(C++와 같은 조건), `flux`를 import 가능하면 flux 경로다. 구독은 `flux_wrapper.Executor`에 `add()`하고, spin은 `cancel()`로 끝낸다.
- flux 경로에서는 View와 그 `data`가 참조되는 동안 슬롯을 잡는다. 기본 `max_borrow` 1에서는 콜백 뒤에 View를 남기면 그것이 사라질 때까지 다음 프레임이 발행자의 링에서 기다린다. 남길 값은 `to_msg()`로 복사한다. ROS 경로의 View는 메시지를 잡을 뿐이라 구독이 멈추지 않는다.
- flux 경로에서는 메시지 타입의 rosidl 모듈 이름(`my_msgs.msg._foo`)으로 어댑터 모듈 `my_msgs_flux.foo`와 `my_msgs_flux.foo_ros`를 찾고, `foo_ros.MESSAGE`가 그 타입인지 확인한다. 찾지 못하면 찾은 모듈 이름과 함께 `TypeError`를 던진다. include는 필요 없다.
- loan에 쓰는 방법은 C++와 같다. `Image`와 `PointCloud2`는 필드를 ROS 메시지의 이름으로 대입하고, `loan.set_from(msg)`로 `data`를 뺀 필드를 한 번에 쓴다. 그 밖의 타입은 `loan.set_from(msg)`로 쓰고, 최상위 header가 있으면 `loan.header`만 따로 대입한다(C++ `set_header()`). 다른 필드를 대입하면 `AttributeError`다.
- flux 경로에서는 쓰는 즉시 슬롯에 쓰므로, 슬롯에 들어가지 않는 필드가 있으면 그 자리에서 loan이 `False`가 되고 `publish()`가 거짓인 `PublishResult`(`status`는 `"too_large"`)를 반환한다.
- loan마다 새 메시지에서 시작하므로, 쓰지 않은 필드는 직전 loan의 값이 아니다. ROS 경로에서는 새 메시지의 값이고, flux 경로에서는 0, 빈 문자열, 빈 배열이다. C++도 같다.
- View에서는 필드를 ROS 메시지의 이름의 속성으로 읽는다. `Image`와 `PointCloud2`가 아닌 타입의 View는 필드를 읽을 때 View마다 한 번 `without_data()`를 만든다. `view.without_data()`는 C++와 같다.
- ROS 경로의 `byte[] data`는 rclpy가 1바이트 `bytes`의 list로 받으므로 `View.data`가 복사본이다.
- `Publisher`와 `Subscription`은 `device="cpu"|"cuda"`를 받는다. GPU 메모리 채널의 형식이 C++와 같아서, C++ 발행자의 프레임을 Python 구독이 받고 그 반대도 된다.
- `loan.alloc_buffer(n)`은 `uint8[n]`의 `flux.Loan`을 반환하고, 그 loan은 `__dlpack__`이나 `__cuda_array_interface__`로 GPU에서 쓴다. C++와 같이 `device="cuda"` 발행자도 `loan()`에서 슬롯을 할당한다. 빈 슬롯이 없으면 loan이 그 자리에서 `False`다. loan이 `False`면 이유와 상관없이 `alloc_buffer()`와 `alloc_data()`는 `None`을 반환한다.
- 받은 `View`는 `on_device`가 참이고 `device_data`로 읽는다. GPU 작업은 `View.stream`에 올린다.

  | 발행자 | `device_data` |
  | --- | --- |
  | `device="cuda"` | `flux.Frame` |
  | `"cpu"` | `flux.wire.DeviceSpan`(`__dlpack__`, `__cuda_array_interface__`, `nbytes`). `data`와 `to_msg()`도 호스트 메모리에서 읽힌다. 외장 GPU에서는 구독자가 GPU로 한 번 복사한다 |

- `"cuda"`를 사용 불가능하면 경고를 한 번 남기고 `"cpu"`로 바뀐다(발행자와 구독의 `uses_device`로 안다).

- `device="cuda"` 발행자의 `publish(msg)`는 메시지의 `data`를 슬롯으로 복사해 보낸다. GPU 메모리에 있는 프레임의 `to_msg()`는 `data`를 호스트로 복사하고, 그 복사가 끝날 때까지 기다린다.

아래 경우는 C++와 다르다.

| 경우 | Python에서 |
| --- | --- |
| ROS 경로의 `device="cuda"` | `"cpu"`로 바뀐다 |
| GPU 메모리에 있는 프레임의 `data` | `None`이다. 통합 GPU에서도 같다 |

## flux_bag

```bash
ros2 bag record_flux -a                         # 모든 flux 채널과 ROS 토픽, 서비스
ros2 bag record_flux --topics /camera/image /imu   # 이름으로. flux와 ROS 이름은 한 목록이다
ros2 bag record_flux -a -s mcap --storage-preset-profile zstd_fast   # 무손실 압축
ros2 bag play_flux rosbag2_2026_10_23-10_00_00
ros2 bag convert_flux rosbag2_2026_10_23-10_00_00 -o converted
```

`record_flux`와 `play_flux`는 `ros2 bag record`, `ros2 bag play`의 옵션을 모두 받는다. 인자 정의, 검사, rosbag2 옵션으로의 변환은 설치된 `ros2bag`의 것을 그대로 쓴다. 그래서 ROS 토픽은 `ros2 bag record`, `ros2 bag play`와 똑같이 녹화, 재생되고, flux 채널은 같은 bag에 들어간다. 옵션 목록은 `-h`로 본다. 아래 표는 각 옵션이 flux 채널에 어떻게 적용되는지다.

### 녹화

```text
ros2 bag record_flux [ros2 bag record의 옵션] [-x REGEX] [--poll SEC]
```

flux 채널 `NAME`의 프레임은 토픽 `NAME/flux`에 `flux_msgs/msg/FluxFrame`으로 저장된다. ROS 토픽은 같은 bag에 그대로 저장된다.

```text
std_msgs/Header header   # commit 시각
string flux_topic        # 채널 이름
uint64 fingerprint       # 채널의 fingerprint
string codec             # "raw"
uint8[] meta             # raw: slot_size u32, slot_count u32, dtype u8, ndim u8, shape u64[ndim]
                         # aux가 있는 채널: 위 meta 뒤에 version u32(1), aux_size u32,
                         #   aux_len u32, aux u8[aux_len]
uint8[] data             # raw: 슬롯에 있던 프레임 bytes 그대로
uint32 format_version    # 이 메시지의 배치, 지금 1. 이 필드가 없는 예전 bag은 0으로 읽는다
string ros_type          # fingerprint가 가리키는 설치된 타입(sensor_msgs/msg/Image 등), 없으면 ""
```

- 새 필드는 끝에 더한다. 예전 `flux_bag`은 모르는 끝 필드를 무시하고 읽는다. `play_flux`와 `convert_flux`는 자기보다 나중 `format_version`의 bag을 거절한다.

| 옵션 | flux 채널에서 |
| --- | --- |
| `--topics`(위치 인자도), `-a`, `--all-topics`, `-e`, `--exclude-regex`, `--exclude-topics`, `--include-hidden-topics` | 채널 이름에 ROS 토픽 이름과 같은 규칙으로 적용한다. `-x`는 `--exclude-regex`와 같다 |
| `--topic-types`, `--exclude-topic-types` | 채널의 fingerprint가 가리키는 설치된 ROS 타입(`sensor_msgs/msg/Image` 등)으로 비교한다. 타입이 설치되어 있지 않은 채널은 `flux_msgs/msg/FluxFrame`으로 본다 |
| `--no-discovery` | 시작할 때 있던 채널만 녹화한다 |
| `-o`, `-s`, `--storage-config-file`, `--storage-preset-profile`, `-b`, `-d`, `--custom-data` | bag 전체에 적용된다 |
| `--max-cache-size`, `--snapshot-mode` | rosbag2 writer의 cache다. flux 프레임도 ROS 메시지처럼 이 cache를 거친다(아래) |
| `--compression-mode`, `--compression-format`, `--compression-queue-size`, `--compression-threads`, `--compression-threads-priority` | flux 프레임도 압축된다. `message` 모드는 flux 프레임 하나하나를 압축한다 |
| `--start-paused`, `--disable-keyboard-controls`, `--node-name`, `--log-level` | 녹화기 전체에 적용된다. pause 동안의 프레임은 받지만 쓰지 않는다 |
| `--use-sim-time` | flux 프레임의 시각도 녹화기의 시계(`/clock`)로 기록한다. 첫 `/clock` 전의 프레임은 쓰지 않는다 |
| `--services`, `--all-services`, `--exclude-services`, `--include-unpublished-topics`, `--ignore-leaf-topics`, `--qos-profile-overrides-path`, `-p`, `-f` | ROS 토픽과 서비스에만 적용된다. flux 채널은 발행자가 있을 때만 목록에 있고, QoS와 직렬화 형식이 없다 |
| `--poll` | flux 전용. 채널 목록을 다시 읽는 간격(기본 0.1 s)이다. 10 s 넘게 내려간 채널은 해제한다 |

| 항목 | 값 |
| --- | --- |
| 복사 | 채널 스레드가 프레임을 자기 버퍼로 복사하고 슬롯을 바로 반납한 뒤, rosbag2가 ROS 메시지를 쓰듯 writer에 쓴다 |
| 쓰기 대기 | flux_bag은 flux 프레임을 따로 모아 두지 않는다. writer는 `ros2 bag record`처럼 `--max-cache-size`(기본 100 MiB, 버퍼 두 개)의 cache에 메시지를 모아 쓴다. cache가 차면 rosbag2가 메시지를 버리고 토픽별로 센다. 녹화기는 녹화하는 동안 토픽이 처음 메시지를 잃을 때와 그 뒤 최대 5 s마다 잃은 수를 로그로 남기고(`rosbag2's cache is full: lost N on TOPIC since the last report`), rosbag2는 bag 파일을 닫을 때 합계를 남긴다. flux 프레임의 토픽은 `NAME/flux`다. flux 프레임과 ROS 메시지는 한 cache를 나눠 쓰므로, 큰 프레임이 cache를 채우면 ROS 메시지도 버려진다. `--max-cache-size 0`이면 writer가 바로 storage에 쓴다. 디스크가 느린 동안 flux 프레임은 채널의 슬롯에서 기다리고, 녹화기가 받기 전에 채널이 덮어쓴 프레임은 채널별 `lost`로 센다. ROS 메시지도 구독 콜백에서 바로 쓰이므로, writer가 느리면 ROS 메시지가 구독의 QoS depth를 넘어 DDS에서 버려질 수 있고, 그 수는 어디에도 남지 않는다 |
| 압축 큐 | `--compression-mode message`에서 압축이 따라가지 못하면 rosbag2가 압축 큐(`--compression-queue-size`, 기본 1)를 넘는 메시지를 버린다. flux 프레임도 같다 |
| 시각 | send 시각과 header stamp는 commit 시각, recv 시각은 take 시각이다 |
| 일시정지 | `~/pause`, `~/resume` 서비스와 키보드(rosbag2와 같다). 노드 이름은 `--node-name`(기본 `rosbag2_recorder`)이다 |
| 저장 형식 | `-s`가 없으면 rosbag2 기본값이다(Jazzy는 mcap, Humble은 sqlite3) |
| 무손실 압축 | `-s mcap --storage-preset-profile zstd_fast`는 mcap chunk를 zstd로 압축한다. `--compression-mode file`은 파일을 닫을 때 zstd 등으로 압축한다. 둘 다 flux 채널과 ROS 토픽에 모두 적용된다. 읽는 쪽(`play_flux`, `convert_flux`, `ros2 bag play`)은 옵션 없이 읽는다 |
| 이미지 압축 | 녹화기는 프레임을 바꾸지 않고 슬롯의 bytes 그대로 저장한다. 압축한 이미지를 녹화하려면 압축한 결과를 따로 flux 채널로 발행하고 그 채널을 녹화한다 |
| aux | 채널을 `aux_size`와 함께 생성했으면 프레임마다 meta 끝에 채널의 `aux_size`와 그 프레임의 aux bytes를 담는다. aux를 쓰지 않은 프레임도 `aux_size`는 담는다 |
| GPU 메모리 채널 | `flux_wrapper`의 `Device::Cuda` Image, PointCloud2 채널 같은 GPU 메모리 채널도 녹화한다. 녹화기는 CPU 구독자라서 take마다 자기 호스트 버퍼로 한 번 복사하고, 그 bytes를 `data`에 담는다. 헤더는 aux로 함께 담긴다 |
| arena | 필드를 arena에 저장한 프레임([concepts.md](concepts.md#arena))은 arena의 필드를 프레임 안으로 복사한 사본으로 녹화한다. 프레임에는 arena 안의 위치만 있어 그대로는 재생이 불가능하기 때문이다. 재생과 변환은 이 사본을 다른 프레임과 똑같이 읽고, 재생은 사본이 들어가는 슬롯으로 발행자를 생성한다. 사본을 만들려면 그 타입이 설치되어 있어야 한다. 타입이 설치되어 있지 않거나 필드가 GPU 메모리에 있는 프레임은 녹화하지 않고, 처음 한 번 경고를 남긴다 |
| 종료 시 | 채널마다 rosbag2에 넘긴 수, 받기 전에 채널이 덮어쓴 수(`lost`), 위 이유로 녹화하지 않은 arena 프레임 수를 로그로 남긴다. 넘긴 프레임 중 cache가 버린 수는 rosbag2의 종료 로그에 있다 |
| 다른 녹화기 | 라이브러리 `flux_bag::flux_bag_frames`의 `flux_bag::FrameEncoder`(`flux_bag/frame_encoder.hpp`)가 위의 meta, aux, arena 사본을 같은 방식으로 만든다. 채널마다 하나를 두고, `encode()`가 채운 필드를 `flux_bag::FluxFrameCdr::write()`로 직렬화한다. 그 bag도 `play_flux`와 `convert_flux`로 읽는다 |

### 재생

```text
ros2 bag play_flux [ros2 bag play의 옵션]
```

`FluxFrame` 토픽은 bag에 기록된 fingerprint, `slot_size`, `slot_count`로 원래 채널에 발행한다. 나머지 토픽은 rosbag2 player가 재생한다.

| 옵션 | flux 채널에서 |
| --- | --- |
| `-r`, `-p`, `--disable-keyboard-controls`, `--log-level` | ROS 토픽과 함께 적용된다. `-r`은 유한한 양수다 |
| `--topics`, `-e`, `-x`, `--exclude-topics` | 채널 이름(`NAME/flux`가 아니라 `NAME`)에 rosbag2의 storage plugin과 같은 규칙으로 적용한다. 목록에 있거나 정규식이 이름 전체에 맞으면 재생하고, 제외 목록에 있거나 제외 정규식이 전체에 맞으면 뺀다 |
| `-l` | ROS 토픽과 함께 반복한다. 한 회차는 둘 다 마지막 메시지를 재생하면 끝나고, 둘이 함께 처음부터 다시 시작한다 |
| `--remap` | 채널 이름에도 player 노드의 remap 규칙을 적용한다 |
| `--start-offset`, `-d`, `--playback-duration`, `--playback-until-sec`, `--playback-until-nsec` | ROS 토픽과 같은 시작 시각과 끝 시각을 쓴다. `-d`는 첫 회차 앞에서만 기다린다 |
| bag 경로, `-i`, `-s`, `--storage-config-file` | bag 하나만 받는다. `-i`를 두 번 이상 주면 거절한다 |
| `--clock` | player의 시계를 주어진 주기로 `/clock`에 발행한다. 이 시계는 flux 채널의 시계와 같은 시점에 시작하므로, `/clock`은 flux 프레임이 나가는 시각과 한 주기 안에서 맞는다. flux 채널만 있는 bag에서도 재생하는 동안 발행한다 |
| `--read-ahead-queue-size`, `--qos-profile-overrides-path`, `--clock-topics`, `--clock-topics-all`, `--wait-for-all-acked`, `--disable-loan-message`, `--services`, `--exclude-services`, `--publish-service-requests`, `--service-requests-source` | ROS 토픽과 서비스에만 적용된다. `--clock-topics`, `--clock-topics-all`은 ROS 메시지를 재생할 때만 `/clock`을 발행하고, 미리 읽기는 아래의 256 MiB다 |

flux 채널은 자기 시계를 따른다. player의 pause, resume, rate(서비스와 키보드)는 이 시계도 함께 조정한다. play_next와 burst는 ROS 토픽을 진행시키고, flux 채널은 그 뒤를 따라간다. ROS 메시지가 모두 소진되면 play_next와 burst는 flux 프레임을 진행시킨다. 서비스는 player 노드(`rosbag2_player`, `ros2 bag play`와 같다) 아래에 있다.

- 재생은 발행자를 생성한 뒤 200 ms 기다리고 시작한다. 먼저 시작된 구독자가 연결될 시간을 주기 위해서다.
- 재생할 flux 프레임은 별도 스레드가 최대 256 MiB 미리 읽는다.
- 프레임의 `slot_size`나 `slot_count`가 바뀌면 발행자를 다시 생성한다. dtype x shape가 데이터 크기와 다른 프레임은 건너뛴다.
- codec이 `raw`가 아닌 프레임은 건너뛰고, 처음 한 번 로그를 남긴다.
- aux를 담은 프레임은 녹화된 `aux_size`로 발행자를 생성하고, aux bytes를 프레임과 함께 commit한다. `aux_size`가 바뀌면 발행자를 다시 생성한다.
- 재생은 GPU 메모리 채널도 호스트 메모리 채널로 발행한다. CPU 구독자는 슬롯을 그대로 읽는다. GPU 구독자(`flux_wrapper`의 `SubscriptionOptions::device = Cuda` 등)는 외장 GPU에서 take마다 GPU로 한 번 복사해 받고, 통합 GPU에서는 복사 없이 받는다. aux는 둘 다 그대로 받는다.
- seek는 ROS 토픽만 진행시킨다.

### 변환

```text
ros2 bag convert_flux BAG -o OUT
```

bag의 flux 프레임을 ROS 메시지로 변환해 새 bag에 저장한다.

| 대상 | 결과 |
| --- | --- |
| fingerprint가 설치된 타입을 가리키는 `FluxFrame` | ROS 메시지로 바꿔 채널 이름의 토픽에 넣는다. 녹화 시각은 유지한다 |
| 타입을 찾지 못했거나, 같은 이름의 ROS 토픽이 다른 타입인 `FluxFrame` | `FluxFrame`으로 남긴다 |
| `flux_wrapper`의 `Device::Cuda` Image, PointCloud2 채널 | 설치된 타입이 아니므로 `FluxFrame`으로 남는다 |
| 나머지 토픽 | 그대로 복사한다 |

- 변환한 토픽은 flux 없이 읽기 가능하다. 남은 `FluxFrame`을 읽으려면 `flux_msgs`가 필요하다.
- 스키마와 맞지 않는 프레임이나 codec이 `raw`가 아닌 프레임이 있으면 변환이 exit code 1로 멈춘다.
- `--compression-mode file`이나 `message`로 압축한 bag도 읽는다. 출력은 압축하지 않은 bag이고, 저장 형식은 rosbag2 기본값이다.
- ROS 메시지에는 aux를 담을 자리가 없으므로 aux는 버린다. `FluxFrame`으로 남는 프레임은 aux를 그대로 가진다.

### aux를 담은 bag의 호환

- aux tail이 없는 bag(aux 이전에 녹화한 bag, aux가 없는 채널)은 그대로 읽는다. meta가 codec의 meta 크기와 같으면 aux가 없다.
- aux tail의 version이나 길이가 맞지 않는 프레임은 잘못된 meta와 같이 다룬다. 재생은 그 프레임을 건너뛰고, 변환은 exit code 1로 멈춘다.
- aux를 담은 bag을 aux 이전의 flux_bag으로 읽으면 meta 크기가 맞지 않는다. 재생은 그 채널의 프레임을 모두 건너뛰고, 변환은 exit code 1로 멈춘다.

## flux_tools(rviz2)

rviz2에서 `Add` -> `flux_tools`를 고른다. 세 디스플레이 모두 rviz2 프로세스 안에서 flux 구독을 생성하고, 렌더 주기마다 최신 프레임 하나를 읽는다(depth 1, `max_borrow` 1). `Topic`에는 같은 타입의 살아 있는 채널이 나오고, 이름을 직접 입력해도 된다. 연결에 실패하면 Topic 상태에 이유를 표시한다.

| 디스플레이 | 타입 | 하는 일 |
| --- | --- | --- |
| `flux_tools/Image` | `sensor_msgs/Image` | 슬롯의 bytes를 텍스처로 바로 올린다. `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `8UC1`, `mono16`, `16UC1`, `32FC1`, `nv12` |
| `flux_tools/PointCloud2` | `sensor_msgs/PointCloud2` | 점을 메시지로 한 번 복사해 rviz2 기본 point cloud renderer에 넘긴다. `x`, `y`, `z`가 `FLOAT32`여야 한다. 점이 arena에 있는 프레임도 읽는다 |
| `flux_tools/DepthCloud` | depth `sensor_msgs/Image` | depth 이미지(`16UC1`, `mono16`은 mm, `32FC1`은 m)를 GPU에서 3D 점으로 변환해 그린다. 같은 카메라의 `CameraInfo`는 DDS에서 받는다 |

- Image: 16-bit와 `32FC1` 프레임에서는 `Normalize Range`(기본 활성화)가 프레임마다 최솟값과 최댓값을 검정과 흰색에 맞춘다. 비활성화하면 `Min Value`, `Max Value`를 사용한다. 픽셀 변환은 shader가 하고, CPU에서 픽셀을 바꾸지 않는다.
- DepthCloud: `Camera Info Topic`은 채널 이름에서 image_transport 규칙으로 정한다(`/cam/depth/image_raw` -> `/cam/depth/camera_info`). 직접 고치면 그 값을 사용한다. `Min Range`, `Max Range`(0은 상한 없음), `Point Size (Pixels)`, `Alpha`도 설정 가능하다.
- PointCloud2와 DepthCloud는 `header.frame_id`를 TF로 fixed frame에 맞춘다.
- rviz2는 업로드나 복사를 한 번 하는 동안 슬롯을 점유한다. 그리기가 느리면 발행자의 `Backpressure`가 늘어날 가능성이 있다.
