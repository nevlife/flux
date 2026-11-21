# flux

flux는 같은 호스트의 ROS 2 노드 사이에서 이미지, 포인트 클라우드, 텐서 같은 큰 메시지를 복사 없이 전달하는 라이브러리다. 발행자와 구독자는 C++, Python, CPU, GPU 어느 조합으로도 연결 가능하다.

기존 `rclcpp`, `rclpy` 노드에서 flux로 보낼 토픽만 지정한다. 나머지 토픽은 기존처럼 DDS로 전달된다. rmw는 바꾸지 않는다.

## 특징

### `.msg` 메시지의 필드 단위 접근

`flux_gen`이 `.msg`에서 C++와 Python 어댑터를 생성한다. 구독자는 `View`로 발행자가 쓴 공유 메모리를 필드 이름으로 읽는다. 발행자는 `Builder`로 슬롯에 필드를 직접 쓴다. 직렬화와 역직렬화는 없다.

```python doc:py_adapter_read
img = Image.View(frame)
print(img.width, img.height, img.encoding, img.data.shape)
```

- Python에서도 배열 필드는 슬롯을 가리키는 numpy 배열이다. 콜백 전에 메시지를 Python 객체로 변환하지 않는다.
- 가변 길이 배열과 중첩 메시지를 지원한다. 슬롯에 들어가지 않는 배열은 발행 프로세스의 arena에 저장하므로, 슬롯 크기를 메시지의 최대 크기에 맞출 필요가 없다.
- 다른 라이브러리가 할당한 버퍼도 복사 없이 필드로 지정한다(`adopt__<필드>`).

자세한 내용은 [messages.md](docs/messages.md)에 있다.

### GPU 메모리

CUDA 스트림을 선언한 채널은 GPU에서 쓴 데이터를 다른 프로세스의 GPU로 복사 없이 전달한다. Python 구독자는 `torch.from_dlpack(frame)`으로 텐서를 얻는다.

- 한 메시지 안에서 숫자 배열 필드는 GPU에, 헤더와 문자열은 CPU 슬롯에 저장한다(`alloc_device__<필드>`, `<필드>__device()`). 이미지 데이터와 헤더가 채널 하나로 전달된다.
- GPU 발행자와 CPU 구독자, CPU 발행자와 GPU 구독자도 연결 가능하다. 통합 GPU에서는 어느 조합이든 복사가 없다. 외장 GPU에서는 CPU와 GPU 사이의 전달에서만 구독자가 한 번 복사한다. CPU 메모리와 GPU 메모리가 물리적으로 분리되어 있기 때문이다.

자세한 내용은 [gpu.md](docs/gpu.md)에 있다.

### Jetson

Jetson은 CPU와 GPU가 메모리를 공유한다. flux는 공유 메모리 슬롯을 GPU에 등록하므로, GPU가 슬롯을 복사 없이 읽는다. Orin에서 GPU가 등록한 공유 메모리를 읽는 속도는 `cudaMalloc`으로 할당한 메모리와 같았다(9.2 MB 합산 커널이 둘 다 59~60 μs).

이미지 슬롯은 슬롯마다 dmabuf로 할당한다. VIC, NVJPG, NVENC 같은 하드웨어 엔진에 슬롯을 복사 없이 넘긴다. 엔진 API에 넘길 `NvBufSurface`는 `flux::NvBufSurfaces::surface()`로 얻는다([gpu.md](docs/gpu.md#이미지-슬롯통합-gpu)).

### 그 밖

발행자가 재시작하면 구독은 새 발행자에 재연결된다. 구독 프로세스가 죽으면 발행자는 그 프로세스가 점유하던 슬롯을 회수한다([concepts.md](docs/concepts.md)). rosbag 녹화와 재생, DDS로 재발행하는 bridge, rviz2 디스플레이를 지원한다([ros2_tools.md](docs/ros2_tools.md)).

## 지원 환경

- Linux, ROS 2 Jazzy
- C++17, Python 3(numpy 1.x, 2.x)
- colcon, CMake 3.16 이상, rustup
- GPU(선택): Jetson Orin, Thor(JetPack 7.2), VMM과 POSIX fd를 지원하는 외장 GPU

## 빌드

colcon 워크스페이스의 `src/` 아래에 submodule(nanobind)과 함께 받아 빌드한다.

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules <flux 저장소 주소> flux
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 빠른 시작

C++ 노드가 슬롯을 loan해 이미지를 직접 쓰고 발행한다.

```cpp doc:readme_loan
auto pub = flux::create_publisher(*node, "image");

constexpr std::size_t kHeight = 480;
constexpr std::size_t kWidth = 640;
flux::Loan slot = pub->loan({kHeight, kWidth, 3}, flux::DType::U8);
if (slot) {
  auto * pixels = static_cast<std::uint8_t *>(slot.data());
  std::fill_n(pixels, kHeight * kWidth * 3, std::uint8_t{0});
  if (flux::faulted(slot.commit())) {
    RCLCPP_ERROR(node->get_logger(), "commit failed");
  }
}
```

이미 만들어진 배열은 `pub->publish(pixels.data(), flux::DType::U8, {kHeight, kWidth, 3})`로 보낸다. 이 경우 슬롯으로 한 번 복사한다([복사 모델](docs/copy_model.md)).

Python 노드가 같은 프레임을 받는다. `frame.array`가 numpy 배열이다.

```python doc:py_subscribe
def on_frame(frame):
    print(frame.shape, frame.dtype)

sub = flux.create_subscription(node, "image", flux.QoS(), on_frame)

ex = flux.Executor()
ex.add(sub)
ex.add_node(node)
ex.spin()
```

`frame`은 공유 메모리 슬롯을 가리키는 읽기 전용 배열이다. `frame`이 있는 동안 발행자는 그 슬롯에 쓰지 않는다. 오래 보관할 값은 복사한다. 사용법은 [C++](docs/cpp.md)과 [Python](docs/python.md) 문서에 있다.

## 패키지

- `flux_core`: Rust 엔진. 공유 메모리 링, discovery, 생존 확인, executor
- `flux_ffi`: C ABI(`flux.h`, `libflux_ffi.so`)
- `flux_cpp`: C++ API와 rclcpp 통합(`flux::Publisher`, `flux::Subscription`, `flux::Executor`)
- `flux_py`: Python 모듈 `flux`(nanobind, numpy)
- `flux_gen`: `.msg`에서 C++, Python 어댑터를 생성하는 생성기
- `flux_std_adapters`: `sensor_msgs/Image` 같은 표준 메시지의 어댑터
- `flux_msgs`: bag에 프레임을 담는 `FluxFrame` 메시지
- `flux_cli`: `flux` 명령
- `flux_components`: 컴포넌트 컨테이너. 로드한 노드의 콜백 그룹마다 스레드 하나를 생성하고, 컴포넌트의 flux 구독도 실행한다
- `flux_heaphook`: `LD_PRELOAD` 라이브러리. scope 안의 `malloc`을 발행 프로세스의 arena로 보낸다([messages.md](docs/messages.md))
- `ros2/flux_bridge`: flux 채널을 DDS 토픽으로 다시 발행한다. `ros2 topic echo_flux`, `hz_flux`
- `ros2/flux_bag`: rosbag2 녹화, 재생. `ros2 bag record_flux`, `play_flux`, `convert_flux`
- `ros2/flux_tools`: rviz2 디스플레이(Image, PointCloud2, DepthCloud)
- `ros2/flux_wrapper`: flux 없이도 빌드되는 드라이버용 발행, 구독 API. `ENABLE_FLUX=1`이면 flux로 보낸다([ros2_tools.md](docs/ros2_tools.md))

## 문서

- [docs/concepts.md](docs/concepts.md): 채널, 슬롯, fingerprint, 전달 규칙, 재시작, 스레드 규칙
- [docs/cpp.md](docs/cpp.md): C++ 사용법
- [docs/python.md](docs/python.md): Python 사용법
- [docs/qos.md](docs/qos.md): `depth`, `replay`, `max_borrow`와 빈 결과의 이유
- [docs/copy_model.md](docs/copy_model.md): 복사가 발생하는 시점
- [docs/messages.md](docs/messages.md): `.msg`에서 어댑터 만들기와 메시지 규칙
- [docs/gpu.md](docs/gpu.md): CUDA 프레임
- [docs/cli.md](docs/cli.md): `flux` 명령
- [docs/ros2_tools.md](docs/ros2_tools.md): bridge, bag, rviz2
- [docs/api.md](docs/api.md): API 층과 생성 레퍼런스
- [docs/release_notes/unreleased.md](docs/release_notes/unreleased.md): 다음 릴리스의 변경

## 라이선스

MIT. [LICENSE](LICENSE)
