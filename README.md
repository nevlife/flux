# flux

flux는 같은 호스트의 ROS 2 노드끼리 큰 데이터(이미지, 포인트 클라우드, 텐서)를 복사 없이 주고받게 하는 C++·Python 라이브러리다. 새 미들웨어가 아니다. rmw를 바꾸지 않고 데몬도 없다. 기존 `rclcpp`·`rclpy` 노드 안에서 flux로 보낼 토픽만 고른다. 나머지 토픽은 지금처럼 DDS로 오간다.

```text
발행자 노드 (C++ 또는 Python)                구독자 노드 (C++ 또는 Python)
  loan()으로 받은 슬롯에 직접 쓴다  ->  /dev/shm 링  ->  콜백이 같은 슬롯을 읽기 전용 view로 받는다
```

구독자는 직렬화도 복사도 없이 발행자가 쓴 슬롯을 그대로 읽는다. 구독자가 받는 것은 C++에서는 `FrameView`, Python에서는 읽기 전용 numpy 배열이다. CUDA GPU에서 만든 프레임도 같은 API로 보낸다([GPU](docs/gpu.md)).

## 지원 환경

| 항목 | 지원 |
| --- | --- |
| OS | Linux. 커널 5.15(Ubuntu 22.04, JetPack 6)부터 7.0까지 시험했다. 5.16 이상에서는 `futex_waitv` 호출 하나로 기다리고, 그보다 낮으면 구독마다 대기 스레드를 둔다 |
| ROS 2 | Humble, Jazzy, Lyrical. Kilted는 지원하지 않는다 |
| 언어 | C++17, Python 3(numpy 1.x·2.x) |
| 빌드 도구 | colcon, CMake 3.16 이상, rustup(Rust 버전은 `rust-toolchain.toml`이 고정한다) |
| GPU(선택) | CUDA 드라이버. 통합 GPU(Jetson Orin, Thor)와 VMM·POSIX fd를 지원하는 외장 GPU. flux는 `libcuda.so.1`을 실행 시점에 연다. 그래서 GPU가 없는 호스트에서도 같은 바이너리가 돈다 |

## 빌드

colcon 워크스페이스의 `src/` 아래에 두고 빌드한다. nanobind가 submodule로 들어 있으므로 submodule도 함께 받는다.

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules <flux 저장소 주소> flux
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 빠른 시작

C++ 노드가 이미지 배열을 발행한다.

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

Python 노드가 같은 프레임을 numpy 배열로 받는다.

```python doc:py_subscribe
def on_frame(frame):
    print(frame.shape, frame.dtype)

sub = flux.Subscription(node, "image", callback=on_frame)

ex = flux.Executor()
ex.add(sub)
ex.add_node(node)
ex.spin()
```

`frame`은 공유메모리 슬롯을 가리키는 읽기 전용 배열이다. 배열이 살아 있는 동안 발행자는 그 슬롯을 다시 쓰지 못한다. 오래 둘 값은 복사한다. 자세한 사용법은 [C++](docs/cpp.md)과 [Python](docs/python.md) 문서에 있다.

## 패키지

| 패키지 | 내용 |
| --- | --- |
| `flux_core` | Rust 엔진. 공유메모리 링, 발견, 생존 확인, executor |
| `flux_ffi` | C ABI(`flux.h`, `libflux_ffi.so`) |
| `flux_cpp` | C++ API와 rclcpp 통합(`flux::Publisher`, `flux::Subscription`, `flux::Executor`) |
| `flux_py` | Python 모듈 `flux`(nanobind, numpy) |
| `flux_gen` | `.msg`에서 C++·Python adapter를 만드는 생성기 |
| `flux_std_adapters` | `sensor_msgs/Image` 같은 표준 메시지의 adapter |
| `flux_msgs` | bag에 프레임을 담는 `FluxFrame` 메시지 |
| `flux_cli` | `flux` 명령 |
| `flux_components` | 컴포넌트 컨테이너. 로드한 노드의 콜백 그룹마다 스레드 하나를 두고, 컴포넌트의 flux 구독도 돌린다 |
| `ros2/flux_bridge` | flux 채널을 DDS 토픽으로 다시 발행한다. `ros2 topic echo_flux`, `hz_flux` |
| `ros2/flux_bag` | rosbag2 녹화·재생. `ros2 bag record_flux`, `play_flux`, `convert_flux` |
| `ros2/flux_tools` | rviz2 display(Image, PointCloud2, DepthCloud) |

## 문서

| 문서 | 내용 |
| --- | --- |
| [docs/concepts.md](docs/concepts.md) | 채널, 슬롯, fingerprint, 전달 규칙, 재시작, 스레드 규칙 |
| [docs/cpp.md](docs/cpp.md) | C++ 사용법 |
| [docs/python.md](docs/python.md) | Python 사용법 |
| [docs/qos.md](docs/qos.md) | `depth`, `replay`, `max_borrow`와 빈 결과의 이유 |
| [docs/copy_model.md](docs/copy_model.md) | 복사가 언제 일어나는가 |
| [docs/messages.md](docs/messages.md) | `.msg`에서 adapter 만들기와 메시지 규칙 |
| [docs/gpu.md](docs/gpu.md) | CUDA 프레임 |
| [docs/cli.md](docs/cli.md) | `flux` 명령 |
| [docs/ros2_tools.md](docs/ros2_tools.md) | bridge, bag, rviz2 |
| [docs/api.md](docs/api.md) | API 층과 생성 레퍼런스 |
| [docs/release_notes/unreleased.md](docs/release_notes/unreleased.md) | 다음 릴리스의 변경 |

## 라이선스

MIT. [LICENSE](LICENSE)를 본다.
