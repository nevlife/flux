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
- 채널의 fingerprint가 설치된 `.msg` 타입(`AMENT_PREFIX_PATH`)과 맞는다. 어댑터를 설치하지 않은 타입도 된다.
- bridge 외에 그 이름의 ROS 2 구독자가 있다.
- bridge 외에 그 이름의 ROS 2 발행자가 없다(발행자의 mirror 포함). 둘 다 발행하면 ROS 2 구독자가 같은 프레임을 두 번 받는다.

| 항목 | 값 |
| --- | --- |
| 프레임 경로 | 슬롯에서 프레임을 복사하고 view를 반납한 뒤, 그 사본에서 CDR bytes를 써서 `GenericPublisher`로 발행한다. 메시지 객체를 거치지 않는다. 필드를 arena에 저장한 프레임([concepts.md](concepts.md#arena))은 view를 반납하기 전에 그 필드까지 사본에 복사한다 |
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
flux::Publisher pub(*node, "image", Image::kFingerprint);
pub.mirror_to_ros();
```

```python
pub = flux.Publisher(node, "image", fingerprint=Image.FINGERPRINT__, mirror_to_ros=True)
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
| 거절하는 것 | 타입이 없는 발행자(`kNoSchema`, `NO_SCHEMA`), fingerprint가 설치된 `.msg`에 없는 발행자, GPU 메모리 발행자. C++은 `flux::Error`, Python은 `flux.Error`(`INVALID_ARGUMENT`)다 |
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

### 발행

- 타입은 `sensor_msgs/Image`와 `sensor_msgs/PointCloud2`다.
- `publish(std::unique_ptr<Msg>)`와 `publish(const Msg &)`는 이미 만든 ROS 메시지를 보낸다. flux 경로에서는 슬롯으로 한 번 복사한다. 메시지를 계속 사용하는 쪽은 `const Msg &`를 사용한다. ROS 경로에서 복사본을 만들지 않는다.
- `get_subscription_count()`는 ROS 경로에서 rclcpp 값을, flux 경로에서 `subscriber_count()`를 반환한다.
- `PublisherOptions`의 `max_message_size`, `slot_count`, `mirror_to_ros`는 flux 경로에만 적용된다. mirror는 기본 비활성화이고, 그 상태에서는 flux 경로의 토픽을 rviz2나 `ros2 bag`이 받지 못한다.

### 구독

- 구독은 `SubscriptionOptions`로 callback group을 받는다. flux 경로의 구독은 그 그룹을 실행하는 flux executor가 처리한다. `flux_wrapper::Executor`는 flux로 빌드했으면 `flux::Executor`, 아니면 `rclcpp::executors::SingleThreadedExecutor`다.
- 콜백이 `std::shared_ptr<const View<Msg>>`를 받으면(`HeldCallback`) 콜백 뒤에도 View를 보유 가능하다. 포인터가 살아 있는 동안 View가 읽는 메시지가 남고, flux 경로에서는 슬롯도 남는다. 마지막 포인터가 사라지는 시점에 반납되며, 반납하는 스레드는 무관하다.
- `SubscriptionOptions::max_borrow`(기본 1)는 한 구독에서 동시에 살아 있는 View의 최대 수다. 콜백 안의 View와 보유한 View를 합한다. 0이면 생성자가 `std::invalid_argument`를 던진다.
- flux 경로에서는 View를 `max_borrow`개 보유한 동안 다음 프레임이 발행자의 링에서 기다린다. 슬롯이 `max_borrow`개 이하인 발행자는 거절한다. 생성자에서 연결 시에는 `flux::Error`를 던지고, executor 아래에서는 로그를 한 번 남기고 발행자 재시작 시까지 그 구독을 제외한다.
- 보유된 슬롯에는 발행자가 쓰지 않는다. 프레임이 버려지지 않으려면 발행자의 `slot_count`가 같은 토픽의 모든 구독이 동시에 점유하는 수보다 1 이상 커야 한다.
- ROS 경로에서 `max_borrow`는 `Cuda` 구독의 GPU 버퍼 수만 정한다. 버퍼가 모두 점유되어 있으면 그 메시지를 건너뛰고 오류 로그를 한 번 남긴다. Reentrant 그룹에서 콜백이 `max_borrow`개보다 많이 겹치는 경우도 같다. 호스트 구독에는 제한이 없다.

### GPU(C++)

- `PublisherOptions::device = flux_wrapper::Device::Cuda`면 메시지의 `data`를 GPU 메모리째 보낸다. 두 타입 모두 지원한다. `loan.alloc_buffer(n)`이 데이터 자리를 할당하고, `on_device()`가 GPU 메모리인지 알려 준다. `copy_from(src, n)`은 원본이 호스트 메모리든 GPU 메모리든 받는다. 커널이 직접 쓰는 경우에는 `device_ptr()`와 `stream()`을 사용하고, publish가 그 스트림의 작업이 완료된 뒤 발행한다.
- flux 경로에서는 타입마다 따로 있는 GPU 채널로 보내고, `data` 밖의 필드를 같은 프레임에 담는다. ROS 경로에서는 wrapper가 GPU 버퍼를 하나 생성하고, publish 시 호스트로 복사해 ROS 메시지로 보낸다.
- GPU 채널의 필드에는 상한이 있다. 넘으면 loan이 `false`가 되고 publish가 `false`를 반환한다.

  | 타입 | 상한 |
  | --- | --- |
  | `Image` | `encoding` 32 B, `frame_id` 256 B |
  | `PointCloud2` | `frame_id` 256 B, `fields` 32개, 필드 이름 32 B |

- `Cuda`를 사용 불가능하면(CUDA 없이 빌드했거나, GPU 경로나 GPU가 없으면) 그 발행자나 구독이 토픽 이름과 함께 경고를 남기고 `Host`로 바뀐다. `uses_device()`가 실제로 사용하는 쪽을 알려 준다. GPU 채널에서는 `mirror_to_ros`를 무시한다.
- 구독은 flux 경로에서 `Cuda` 발행자의 프레임도 받는다. `SubscriptionOptions::device`가 `Host`(기본)면 `data()`가 호스트 메모리다(외장 GPU에서는 take 시 한 번 복사한다). 헤더가 잘못된 프레임은 건너뛰고 `rejected()`로 센다.
- 구독의 `device`가 `Cuda`면 발행자가 어느 쪽이든 `data`가 GPU 메모리에 있다. `on_device()`, `device_ptr()`, `size()`, `stream()`으로 읽고, GPU 작업은 `stream()`에 올린다. `Host` 발행자의 프레임은 외장 GPU에서 take 시 GPU로 한 번 복사하고, 통합 GPU에서는 복사하지 않는다. 호스트 메모리에도 있는 프레임은 `data()`로도 읽힌다.
- ROS 경로의 `Cuda` 구독은 구독마다 GPU 버퍼를 `max_borrow`개 생성하고, 메시지 수신마다 빈 버퍼로 복사한다. 버퍼마다 스트림이 따로 있다.
- GPU 메모리 프레임을 `stream()`이 아닌 스트림 `s`에서 읽으면, 반납 전에 `View::after(s)`를 호출한다. `stream()`이 `s`에 지금까지 올린 작업 뒤에 오게 해서, 슬롯이나 GPU 버퍼가 그 작업이 완료된 뒤 반납된다. 호출하지 않으면 그 작업이 완료되기 전에 발행자가 슬롯에 새 프레임을 쓸 가능성이 있다.
- GPU 기능은 CMake가 CUDAToolkit을 찾은 경우에 컴파일된다. 찾지 못해도 `Device::Cuda`는 있으므로 코드는 그대로 빌드된다.

### Python

- Python 모듈 `flux_wrapper`도 같은 구조다. 다만 `max_borrow`와 `HeldCallback`은 없다. `flux`를 import 가능하고 `ENABLE_FLUX=1`이면 flux 경로다. 구독은 `flux_wrapper.Executor`에 `add()`한다.
- `Publisher`와 `Subscription`은 `device="cpu"|"cuda"`를 받는다. GPU 채널과 그 필드 배치가 C++와 같아서, C++ 발행자의 프레임을 Python 구독이 받고 그 반대도 된다.
- `loan.alloc_buffer(n)`은 `uint8[n]`의 `flux.Loan`을 반환하고, 그 loan은 `__dlpack__`이나 `__cuda_array_interface__`로 GPU에서 쓴다. `device="cuda"` 발행자의 loan은 `alloc_buffer()`나 `alloc_data()` 호출 시 슬롯을 할당한다. 빈 슬롯이 없으면 그 결과가 `None`이고, 그 전까지 loan은 참이다.
- 받은 `View`는 `on_device`가 참이고 `device_data`로 읽는다. GPU 작업은 `View.stream`에 올린다.

  | 발행자 | `device_data` |
  | --- | --- |
  | `device="cuda"` | `flux.Frame` |
  | `"cpu"` | `flux.wire.DeviceSpan`(`__dlpack__`, `__cuda_array_interface__`, `nbytes`). `data`와 `to_msg()`도 호스트 메모리에서 읽힌다. 외장 GPU에서는 구독자가 GPU로 한 번 복사한다 |

- 통합 GPU에서는 슬롯을 넘어 arena에 저장된 `data`에 GPU 주소가 없어 `on_device`가 거짓이다.
- `"cuda"`를 사용 불가능하면 경고를 한 번 남기고 `"cpu"`로 바뀐다(`uses_device`).

flux Python 모듈에는 호스트 메모리와 GPU 메모리 사이의 복사가 없으므로, 아래 경우가 C++와 다르다.

| 경우 | Python에서 |
| --- | --- |
| ROS 경로의 `device="cuda"` | `"cpu"`로 바뀐다 |
| 외장 GPU에서 `device="cuda"` 발행자의 `publish(msg)` | `False`를 반환하고 오류 로그를 한 번 남긴다. 데이터는 `loan.alloc_buffer()`로 쓴다. 통합 GPU에서는 슬롯이 호스트 메모리이기도 해서 그대로 보낸다 |
| GPU 메모리에 있는 프레임의 `data` | `None`이다. 통합 GPU에서도 같다 |
| GPU 메모리에 있는 프레임의 `to_msg()` | `RuntimeError`다 |

## flux_bag

```bash
ros2 bag record_flux -a                         # 모든 flux 채널과 ROS 토픽
ros2 bag record_flux /camera/image /imu         # 이름으로. flux와 ROS 이름은 한 목록이다
ros2 bag record_flux -a -s mcap --storage-preset-profile zstd_fast --image-codec jpeg   # 압축
ros2 bag play_flux rosbag2_2026_10_23-10_00_00
ros2 bag convert_flux rosbag2_2026_10_23-10_00_00 -o converted
```

### 녹화

```text
ros2 bag record_flux [-a] [TOPIC ...] [-e REGEX] [-x REGEX] [--exclude-topics TOPIC ...]
                     [-o OUT] [--poll SEC] [--max-cache-size BYTES]
                     [-s STORAGE] [--storage-preset-profile PROFILE]
                     [--image-codec raw|jpeg] [--jpeg-quality Q]
```

flux 채널 `NAME`의 프레임은 토픽 `NAME/flux`에 `flux_msgs/msg/FluxFrame`으로 저장된다. ROS 토픽은 같은 bag에 그대로 저장된다.

```text
std_msgs/Header header   # commit 시각
string flux_topic        # 채널 이름
uint64 fingerprint       # 채널의 fingerprint
string codec             # "raw" 또는 "jpeg"
uint8[] meta             # raw: slot_size u32, slot_count u32, dtype u8, ndim u8, shape u64[ndim]
                         # jpeg: raw의 meta 뒤에 픽셀 구간의 offset u64, 길이 u64
                         # aux가 있는 채널: 위 meta 뒤에 version u32(1), aux_size u32,
                         #   aux_len u32, aux u8[aux_len]
uint8[] data             # raw: 슬롯에 있던 프레임 bytes 그대로
                         # jpeg: 픽셀 구간을 뺀 프레임 bytes 뒤에 JPEG bytes
```

| 항목 | 값 |
| --- | --- |
| 선택 | `ros2 bag record`와 같은 규칙을 flux 채널 이름과 ROS 토픽 이름에 적용한다 |
| 채널 찾기 | `--poll` 간격(기본 0.1 s)마다 채널 목록을 다시 읽는다. 10 s 넘게 내려간 채널은 해제한다 |
| 복사 | 채널 스레드가 프레임을 자기 버퍼로 복사하고 슬롯을 바로 반납한다. jpeg로 저장하는 프레임은 인코딩이 끝난 뒤 반납한다. bag 쓰기는 스레드 하나가 한다 |
| 쓰기 대기 | 디스크가 느린 동안 프레임은 메모리에서 기다린다. 상한은 `--max-cache-size`(기본 1 GiB)이고, 넘는 프레임은 버리고 알린다. 발행자는 기다리지 않는다 |
| 시각 | send 시각과 header stamp는 commit 시각, recv 시각은 take 시각이다 |
| 일시정지 | `~/pause`, `~/resume` 서비스(rosbag2와 같다). 멈춘 동안의 프레임은 받지만 쓰지 않는다 |
| 저장 형식 | `-s`가 없으면 rosbag2 기본값(mcap)이다. `-s`와 `--storage-preset-profile`은 `ros2 bag record`의 같은 옵션처럼 rosbag2에 넘긴다 |
| 무손실 압축 | `-s mcap --storage-preset-profile zstd_fast`는 mcap chunk를 zstd로 압축한다. flux 채널과 ROS 토픽에 모두 적용된다. 읽는 쪽(`play_flux`, `convert_flux`, `ros2 bag play`)은 옵션 없이 읽는다 |
| 이미지 압축 | `--image-codec jpeg`는 `sensor_msgs/Image` 채널의 `rgb8`, `bgr8`, `mono8` 프레임의 픽셀을 JPEG(품질 `--jpeg-quality`, 기본 90)로 저장한다. 손실 압축이다. 픽셀 밖의 bytes(header, encoding, step 등)는 그대로 유지한다. 다른 encoding과 다른 타입의 프레임은 raw로 저장한다. 기본은 `raw`다 |
| aux | 채널을 `aux_size`와 함께 생성했으면 프레임마다 meta 끝에 채널의 `aux_size`와 그 프레임의 aux bytes를 담는다. aux를 쓰지 않은 프레임도 `aux_size`는 담는다 |
| GPU 메모리 채널 | `flux_wrapper`의 `Device::Cuda` Image, PointCloud2 채널 같은 GPU 메모리 채널도 녹화한다. 녹화기는 CPU 구독자라서 take마다 자기 호스트 버퍼로 한 번 복사하고, 그 bytes를 `data`에 담는다. 헤더는 aux로 함께 담긴다 |
| arena | 필드를 arena에 저장한 프레임([concepts.md](concepts.md#arena))은 arena의 필드를 프레임 안으로 복사한 사본으로 녹화한다. 프레임에는 arena 안의 위치만 있어 그대로는 재생이 불가능하기 때문이다. 재생과 변환은 이 사본을 다른 프레임과 똑같이 읽고, 재생은 사본이 들어가는 슬롯으로 발행자를 생성한다. 사본을 만들려면 그 타입이 설치되어 있어야 한다. 타입이 설치되어 있지 않거나 필드가 GPU 메모리에 있는 프레임은 녹화하지 않고, 처음 한 번 경고를 남긴다 |
| 종료 시 | 채널마다 큐에 넣은 수, 캐시가 차서 버린 수, `lost`, 위 이유로 녹화하지 않은 arena 프레임 수를 로그로 남긴다 |

### 재생

```text
ros2 bag play_flux BAG [-r RATE]
```

`FluxFrame` 토픽은 bag에 기록된 fingerprint, `slot_size`, `slot_count`로 원래 채널에 발행한다. 나머지 토픽은 rosbag2 player가 재생한다.

flux 채널은 자기 시계를 따른다. player의 pause, resume, rate(서비스와 키보드)는 이 시계도 함께 조정한다. play_next와 burst는 ROS 토픽을 진행시키고, flux 채널은 그 뒤를 따라간다. ROS 메시지가 모두 소진되면 play_next와 burst는 flux 프레임을 진행시킨다.

- 재생은 발행자를 생성한 뒤 200 ms 기다리고 시작한다. 먼저 시작된 구독자가 연결될 시간을 주기 위해서다.
- 프레임의 `slot_size`나 `slot_count`가 바뀌면 발행자를 다시 생성한다. dtype x shape가 데이터 크기와 다른 프레임은 건너뛴다.
- jpeg 프레임은 픽셀을 디코딩해 원래 크기의 프레임으로 발행한다. 픽셀 값은 원본과 다르다. 행 끝의 padding(`step`이 `width` x 채널 수보다 긴 부분)은 0이다.
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
| jpeg 프레임 | `sensor_msgs/msg/CompressedImage`로 바꿔 `<채널 이름>/compressed`에 넣는다. `format`은 compressed_image_transport와 같은 형식(`rgb8; jpeg compressed bgr8`)이고, `data`는 녹화한 JPEG bytes 그대로다 |
| `flux_wrapper`의 `Device::Cuda` Image, PointCloud2 채널 | 설치된 타입이 아니므로 `FluxFrame`으로 남는다 |
| 나머지 토픽 | 그대로 복사한다 |

- 변환한 토픽은 flux 없이 읽기 가능하다. 남은 `FluxFrame`을 읽으려면 `flux_msgs`가 필요하다.
- 스키마와 맞지 않는 프레임이 있으면 변환이 exit code 1로 멈춘다.
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
