# ROS 2 도구

flux 채널은 DDS graph에 없으므로 기본 ROS 2 도구(`ros2 topic`, `rqt`, `ros2 bag`, rviz2)로는 그대로 볼 수 없다. `ros2/` 아래의 세 패키지가 flux 채널을 이 도구들과 연결한다.

| 패키지 | 하는 일 | 복사 |
| --- | --- | --- |
| `flux_bridge` | flux 채널을 DDS 토픽으로 다시 발행한다 | 프레임마다 1, 그리고 CDR 쓰기와 DDS 전달 |
| `flux_bag` | flux 채널을 rosbag2 bag에 녹화하고 flux로 재생한다 | 프레임마다 1. DDS를 거치지 않는다 |
| `flux_tools` | rviz2 display. 공유메모리에서 바로 읽는다 | GPU 업로드 1 |

bridge를 띄우지 않고 발행자가 직접 ROS 2 토픽을 함께 낼 수도 있다([발행자의 ROS 2 mirror](#발행자의-ros-2-mirror)).

## flux_bridge

```bash
ros2 run flux_bridge bridge [--poll SEC]
```

bridge는 노드 하나(`flux_bridge_<pid>`)를 띄우고, `--poll`초(기본 1)마다 채널 목록을 읽는다. 그리고 채널마다 아래 조건이 모두 맞는 동안에만 relay를 돌린다. 그래서 아무도 구독하지 않는 채널에는 비용이 들지 않는다.

- 발행자가 살아 있고 이름이 정확히 알려진 채널이다.
- 채널의 fingerprint가 설치된 `.msg` 타입(`AMENT_PREFIX_PATH`)과 맞는다. adapter를 설치하지 않은 타입도 된다.
- bridge 말고 그 이름의 ROS 2 구독자가 있다.
- bridge 말고 그 이름의 ROS 2 발행자가 없다. 발행자가 mirror를 켰으면 bridge는 그 토픽을 relay하지 않는다. 둘 다 내면 ROS 2 구독자가 같은 프레임을 두 번 받기 때문이다.

| 항목 | 값 |
| --- | --- |
| 프레임 경로 | 슬롯에서 프레임을 복사하고 view를 놓은 뒤, 그 사본에서 메시지의 CDR 바이트를 바로 써서 `GenericPublisher`로 낸다. 메시지 객체를 거치지 않는다 |
| DDS QoS | depth 1, best effort, volatile |
| flux QoS | depth 1, `max_borrow` 1 |
| 같은 이름의 채널 둘 | fingerprint가 다른 채널이 둘이면 signpost 순서로 첫 것을 고른다 |
| 하지 않는 것 | DDS -> flux 방향, 서비스, 액션, 파라미터 |

### ros2 topic verb

```bash
ros2 topic echo_flux /camera/image --no-arr
ros2 topic hz_flux /camera/image
```

기본 `echo`·`hz`를 바로 띄우면, bridge가 다음에 poll할 때까지 토픽이 없다. 그래서 두 verb는 먼저 아무 일도 하지 않는 구독을 만든다. 그리고 그 토픽의 발행자(bridge나 발행자의 mirror)가 graph에 나타날 때까지 기다린 뒤 기본 verb를 그대로 돌린다. 인자는 기본 verb의 인자에 `--bridge-timeout SEC`(기본 10)가 더해진 것이다. 메시지 타입은 flux가 채널의 fingerprint로 찾으므로 DDS 발견을 기다리지 않는다. 두 verb를 쓰려면 bridge가 떠 있거나 발행자가 mirror를 켜야 한다.

## 발행자의 ROS 2 mirror

```cpp
flux::Publisher pub(*node, "image", Image::kFingerprint);
pub.mirror_to_ros();
```

```python
pub = flux.Publisher(node, "image", fingerprint=Image.FINGERPRINT__, mirror_to_ros=True)
```

켜면 발행자가 같은 이름의 ROS 2 토픽도 낸다. 일반 ROS 2 노드와 기본 도구가 그 토픽을 바로 구독할 수 있다. flux 구독자가 받는 경로는 바뀌지 않는다.

| 항목 | 값 |
| --- | --- |
| 프레임 경로 | 발행자 프로세스 안의 스레드가 자기 채널을 읽는다. 슬롯에서 프레임을 복사하고 view를 놓은 뒤, 그 사본에서 메시지의 CDR을 써서 낸다. bridge와 같은 경로다 |
| 발행 경로 | `publish()`와 commit은 변환을 기다리지 않는다 |
| ROS 2 구독자가 없을 때 | flux 구독을 놓고 100 ms마다 구독자 수만 본다. 구독자 번호(`max_subscribers`)를 차지하지 않는다 |
| ROS 2 구독자가 있을 때 | flux 구독자 하나로 붙으므로 구독자 번호 하나를 쓴다. 발행자의 `subscriber_count`에도 1로 들어간다 |
| DDS QoS | depth 1, best effort, volatile |
| flux QoS | depth 1, `max_borrow` 1 |
| 생성 비용 | 켤 때 설치된 `.msg`를 모두 읽는다 |
| 거절하는 것 | 타입이 없는 발행자(`kNoSchema`, `NO_SCHEMA`)와 fingerprint가 설치된 `.msg`에 없는 발행자, 장치 메모리 발행자. C++은 `flux::Error`, Python은 `flux.Error`(`INVALID_ARGUMENT`)다 |
| Python | 스레드는 `threading.Thread`이고 rclpy 발행자에 `bytes`로 낸다. 발행자 객체가 사라지면 스레드도 끝난다 |

FHD 이미지처럼 큰 메시지를 여러 토픽으로 보내면 DDS의 UDP 수신 버퍼가 넘쳐 best effort 메시지가 버려질 수 있다. bridge도 같다. `netstat -su`의 `receive buffer errors`가 늘면 rmw 설정에서 소켓 버퍼를 늘린다(Cyclone DDS는 `CYCLONEDDS_URI`의 `SocketReceiveBufferSize`, 상한은 `net.core.rmem_max`다).

## flux_wrapper

드라이버가 flux 없이도 빌드되고 돌게 하는 발행·구독 API다. 드라이버의 `package.xml`에는 `flux_wrapper`만 넣는다. `flux_wrapper`가 flux를 `ENABLE_FLUX == 1` 조건으로 의존한다.

| 스위치 | 언제 | 뜻 |
| --- | --- | --- |
| 빌드 | `flux_wrapper`를 빌드할 때 `ENABLE_FLUX=1` | flux 경로를 컴파일하고 `USE_FLUX_ENABLED`를 정의한다. flux를 찾았는지로 정하지 않는다. 이 값이 없으면 `colcon build --packages-up-to <드라이버>`가 flux를 빌드하지 않는다 |
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
    // v.data()는 flux 경로에서 슬롯을 가리킨다. 콜백 밖에서 쓸 값은 v.to_msg()로 복사한다
  });
```

- 타입은 `sensor_msgs/Image`와 `sensor_msgs/PointCloud2`다.
- `publish(std::unique_ptr<Msg>)`와 `publish(const Msg &)`는 이미 만든 ROS 메시지를 보낸다. flux 경로에서는 슬롯으로 한 번 복사한다. 메시지를 계속 쓰는 쪽은 `const Msg &`를 쓴다. ROS 경로에서 복사본을 만들지 않는다.
- `get_subscription_count()`는 ROS 경로에서 rclcpp 값을, flux 경로에서 `subscriber_count()`를 돌려준다.
- `PublisherOptions`의 `max_message_size`, `slot_count`, `mirror_to_ros`는 flux 경로에만 쓰인다. mirror는 기본으로 꺼져 있다. 켜지 않으면 flux 경로의 토픽을 rviz2나 `ros2 bag`이 받지 못한다.
- 구독은 `SubscriptionOptions`로 callback group을 받는다. flux 경로의 구독은 그 그룹을 돌리는 flux executor가 돌린다. `flux_wrapper::Executor`는 flux로 빌드했으면 `flux::Executor`, 아니면 `rclcpp::executors::SingleThreadedExecutor`다.
- `PublisherOptions::device = flux_wrapper::Device::Cuda`면 메시지의 `data`를 GPU 메모리째 보낸다. 두 타입 모두 된다. `loan.alloc_buffer(n)`이 데이터 자리를 주고, `on_device()`가 GPU 메모리인지 알려 준다. `copy_from(src, n)`은 원본이 호스트든 장치든 받는다. kernel이 직접 쓸 때는 `device_ptr()`와 `stream()`을 쓰고, publish가 그 stream의 작업이 끝난 뒤 발행한다.
- flux 경로에서는 타입마다 따로 있는 GPU 채널로 보내고 `data` 밖의 필드를 같은 프레임에 담는다. ROS 경로에서는 wrapper가 GPU 버퍼를 하나 두고, publish할 때 호스트로 복사해 ROS 메시지로 보낸다. GPU 채널의 필드에는 상한이 있다. `Image`는 `encoding` 32바이트, `frame_id` 256바이트까지다. `PointCloud2`는 `frame_id` 256바이트, `fields` 32개, 필드 이름 32바이트까지다. 넘으면 loan이 `false`가 되고 publish가 `false`다.
- `Cuda`를 쓸 수 없으면(CUDA 없이 빌드했거나, GPU 경로나 장치가 없으면) 경고를 한 번 남기고 `Host`로 간다. `uses_device()`가 실제로 쓰는 쪽을 알려 준다. GPU 채널에서는 `mirror_to_ros`를 무시한다.
- 구독은 flux 경로에서 `Cuda` 발행자의 프레임도 받는다. `SubscriptionOptions::device`가 `Host`(기본)면 `data()`가 호스트 메모리다(외장 GPU에서는 take할 때 한 번 복사한다). 헤더가 잘못된 프레임은 건너뛰고 `rejected()`로 센다.
- 구독의 `device`가 `Cuda`면 발행자가 어느 쪽이든 `data`가 장치 메모리에 있다. `on_device()`, `device_ptr()`, `size()`, `stream()`으로 읽고, 장치 작업은 `stream()`에 올린다. `Host` 발행자의 프레임은 외장 GPU에서 take할 때 장치로 한 번 복사하고, 통합 GPU에서는 복사하지 않는다. ROS 경로에서는 구독마다 장치 버퍼를 하나 두고, 메시지를 받을 때마다 그 버퍼로 복사한다. 콜백들이 이 버퍼를 함께 쓰므로 그 구독을 Reentrant 그룹에 넣지 않는다. 호스트 메모리에도 있는 프레임은 `data()`로도 읽힌다.
- GPU 기능은 CMake가 CUDAToolkit을 찾을 때 컴파일된다. 찾지 못해도 `Device::Cuda`는 있으므로 코드는 그대로 빌드된다.
- Python 모듈 `flux_wrapper`도 같은 구조다. `flux`를 import할 수 있고 `ENABLE_FLUX=1`이면 flux 경로다. 구독은 `flux_wrapper.Executor`에 `add()`한다.
- Python의 `Publisher`와 `Subscription`은 `device="cpu"|"cuda"`를 받는다. GPU 채널과 그 필드 배치가 C++와 같아서, C++ 발행자의 프레임을 Python 구독이 받고 그 반대도 된다. `loan.alloc_buffer(n)`은 `uint8[n]`의 `flux.Loan`을 주고, 그 loan은 `__dlpack__`이나 `__cuda_array_interface__`로 장치에서 쓴다. `device="cuda"` 발행자의 loan은 `alloc_buffer()`나 `alloc_data()`를 부를 때 슬롯을 잡는다. 빈 슬롯이 없으면 그 결과가 `None`이고, 그 전까지 loan은 참이다. 받은 `View`는 `on_device`가 참이고 `device_data`(`flux.Frame`)로 읽는다. `"cuda"`를 쓸 수 없으면 경고를 한 번 남기고 `"cpu"`로 간다(`uses_device`).

flux Python 모듈에는 호스트 메모리와 장치 메모리 사이의 복사가 없다. 그래서 Python에서는 아래가 C++와 다르다.

| 경우 | Python에서 |
| --- | --- |
| ROS 경로의 `device="cuda"` | `"cpu"`로 간다 |
| 외장 GPU에서 `device="cuda"` 발행자의 `publish(msg)` | `False`를 돌려주고 오류 로그를 한 번 남긴다. 데이터는 `loan.alloc_buffer()`로 쓴다. 통합 GPU에서는 슬롯이 호스트 메모리이기도 해서 그대로 보낸다 |
| `device="cuda"` 구독이 `"cpu"` 발행자의 프레임을 받을 때 | 호스트 메모리로 받고(`on_device`가 거짓), 경고를 한 번 남긴다. 필드를 읽으려면 프레임의 호스트 바이트가 필요한데, flux Python 모듈은 장치 구독의 프레임에서 그것을 주지 않는다 |
| 장치 메모리에 있는 프레임의 `data` | `None`이다. 통합 GPU에서도 같다 |
| 장치 메모리에 있는 프레임의 `to_msg()` | `RuntimeError`다 |

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

flux 채널 `NAME`의 프레임은 토픽 `NAME/flux`에 `flux_msgs/msg/FluxFrame`으로 들어간다. ROS 토픽은 같은 bag에 그대로 들어간다.

```text
std_msgs/Header header   # commit 시각
string flux_topic        # 채널 이름
uint64 fingerprint       # 채널의 스키마 지문
string codec             # "raw" 또는 "jpeg"
uint8[] meta             # raw: slot_size u32, slot_count u32, dtype u8, ndim u8, shape u64[ndim]
                         # jpeg: raw의 meta 뒤에 픽셀 구간의 offset u64, 길이 u64
uint8[] data             # raw: 슬롯에 있던 프레임 바이트 그대로
                         # jpeg: 픽셀 구간을 뺀 프레임 바이트 뒤에 JPEG 바이트
```

| 항목 | 값 |
| --- | --- |
| 선택 | `ros2 bag record`와 같은 규칙을 flux 채널 이름과 ROS 토픽 이름에 똑같이 적용한다. |
| 채널 찾기 | `--poll`초(기본 0.1)마다 채널 목록을 다시 읽는다. 10초 넘게 내려간 채널은 놓는다 |
| 복사 | 채널 스레드가 프레임을 자기 버퍼로 복사하고 슬롯을 바로 돌려준다. jpeg로 저장하는 프레임은 인코딩이 끝난 뒤 돌려준다. bag 쓰기는 스레드 하나가 한다 |
| 쓰기 대기 | 디스크가 느린 동안 프레임은 메모리에서 기다린다. 상한은 `--max-cache-size`(기본 1 GiB)이고, 넘는 프레임은 버리고 알린다. 발행자는 기다리지 않는다 |
| 시각 | send 시각과 header stamp는 commit 시각, recv 시각은 take 시각이다 |
| 일시정지 | `~/pause`·`~/resume` 서비스(rosbag2와 같다). 멈춘 동안의 프레임은 받되 쓰지 않는다 |
| 저장 형식 | `-s`가 없으면 rosbag2 기본값(mcap)이다. `-s`와 `--storage-preset-profile`은 `ros2 bag record`의 같은 옵션처럼 rosbag2에 넘긴다 |
| 무손실 압축 | `-s mcap --storage-preset-profile zstd_fast`는 mcap chunk를 zstd로 압축한다. flux 채널과 ROS 토픽에 모두 적용된다. 읽는 쪽(`play_flux`, `convert_flux`, `ros2 bag play`)은 옵션 없이 읽는다 |
| 이미지 압축 | `--image-codec jpeg`는 `sensor_msgs/Image` 채널의 `rgb8`, `bgr8`, `mono8` 프레임의 픽셀을 JPEG(품질 `--jpeg-quality`, 기본 90)로 저장한다. 손실 압축이다. 픽셀 밖의 바이트(header, encoding, step 등)는 그대로 둔다. 다른 encoding과 다른 타입의 프레임은 raw로 저장한다. 기본은 `raw`다 |
| 끝날 때 | 채널마다 큐에 넣은 수, 캐시가 차서 버린 수, `lost`를 로그로 남긴다 |

### 재생

```text
ros2 bag play_flux BAG [-r RATE]
```

`FluxFrame` 토픽은 파일에서 읽어 바로 원래 채널에 발행한다. 이때 bag에 기록된 fingerprint, `slot_size`, `slot_count`를 쓴다. 나머지 토픽은 rosbag2 player가 재생한다. flux 채널은 자기 시계를 따른다. player의 pause, resume, rate(서비스와 키보드)는 이 시계도 함께 움직인다. play_next와 burst는 ROS 토픽을 옮기고, flux 채널은 그 뒤를 따라간다. ROS 메시지가 다 떨어지면 play_next와 burst는 flux 프레임을 옮긴다.

- 재생은 발행자를 만든 뒤 200 ms 기다리고 시작한다. 먼저 떠 있던 구독자가 붙을 시간을 주기 위해서다.
- 프레임의 `slot_size`나 `slot_count`가 바뀌면 발행자를 다시 만든다. dtype x shape가 데이터 크기와 다른 프레임은 건너뛴다.
- jpeg 프레임은 픽셀을 디코딩해 원래 크기의 프레임으로 발행한다. 픽셀 값은 원본과 다르다. 행 끝의 padding(`step`이 `width` x 채널 수보다 긴 부분)은 0이다.
- seek는 ROS 토픽만 옮긴다.

### 변환

```text
ros2 bag convert_flux BAG -o OUT
```

bag을 새 bag으로 옮긴다. `FluxFrame`의 fingerprint가 설치된 타입을 가리키면, 그 프레임을 ROS 메시지로 바꿔 채널 이름의 토픽에 넣는다. 녹화 시각은 그대로 둔다. 타입을 못 찾거나 같은 이름의 ROS 토픽이 다른 타입이면 `FluxFrame`으로 남긴다. jpeg 프레임은 `sensor_msgs/msg/CompressedImage`로 바꿔 `채널 이름/compressed`에 넣는다. `format`은 compressed_image_transport와 같은 꼴(`rgb8; jpeg compressed bgr8`)이고, `data`는 녹화한 JPEG 바이트 그대로다. 나머지 토픽은 그대로 복사한다. 변환한 토픽은 flux 없이 읽을 수 있다. 남은 `FluxFrame`을 읽으려면 `flux_msgs`가 있어야 한다. 스키마와 맞지 않는 프레임이 있으면 변환이 exit code 1로 멈춘다.

## flux_tools(rviz2)

rviz2에서 `Add` -> `flux_tools`를 고른다. 세 display 모두 rviz2 프로세스 안에서 flux 구독을 열고 render tick마다 최신 프레임 하나를 읽는다(depth 1, `max_borrow` 1). `Topic`에는 같은 타입의 살아 있는 채널이 나오고, 이름을 직접 적어도 된다. 붙지 못하면 Topic 상태에 이유를 보인다.

| display | 타입 | 하는 일 |
| --- | --- | --- |
| `flux_tools/Image` | `sensor_msgs/Image` | 슬롯의 바이트를 텍스처로 바로 올린다. `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `8UC1`, `mono16`, `16UC1`, `32FC1`, `nv12` |
| `flux_tools/PointCloud2` | `sensor_msgs/PointCloud2` | 점을 메시지로 한 번 복사해 rviz2 기본 point cloud renderer에 넘긴다. `x`, `y`, `z`가 `FLOAT32`여야 한다 |
| `flux_tools/DepthCloud` | depth `sensor_msgs/Image` | depth 이미지(`16UC1`·`mono16` mm, `32FC1` m)를 GPU에서 3D 점으로 펴서 그린다. 같은 카메라의 `CameraInfo`는 DDS에서 받는다 |

- Image: 16비트와 `32FC1` 프레임에서는 `Normalize Range`(기본 켬)가 프레임마다 최솟값과 최댓값을 검정과 흰색에 맞춘다. 끄면 `Min Value`, `Max Value`를 쓴다. 픽셀 변환은 shader가 하고 CPU에서 픽셀을 바꾸지 않는다.
- DepthCloud: `Camera Info Topic`은 채널 이름에서 image_transport 규칙으로 정한다(`/cam/depth/image_raw` -> `/cam/depth/camera_info`). 직접 고치면 그 값을 쓴다. `Min Range`, `Max Range`(0은 상한 없음), `Point Size (Pixels)`, `Alpha`도 정할 수 있다.
- PointCloud2와 DepthCloud는 `header.frame_id`를 TF로 fixed frame에 맞춘다.
- rviz2는 업로드나 복사를 한 번 하는 동안 슬롯을 쥔다. 그리기가 느리면 그만큼 발행자의 `Backpressure`가 늘 수 있다.
