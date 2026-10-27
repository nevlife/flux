# ROS 2 도구

flux 채널은 DDS graph에 없으므로 기본 ROS 2 도구(`ros2 topic`, `rqt`, `ros2 bag`, rviz2)로는 그대로 볼 수 없다. `ros2/` 아래의 세 패키지가 flux 채널을 이 도구들과 연결한다.

| 패키지 | 하는 일 | 복사 |
| --- | --- | --- |
| `flux_bridge` | flux 채널을 DDS 토픽으로 다시 발행한다 | 프레임마다 1, 그리고 CDR 쓰기와 DDS 전달 |
| `flux_bag` | flux 채널을 rosbag2 bag에 녹화하고 flux로 재생한다 | 프레임마다 1. DDS를 거치지 않는다 |
| `flux_tools` | rviz2 display. 공유메모리에서 바로 읽는다 | GPU 업로드 1 |

## flux_bridge

```bash
ros2 run flux_bridge bridge [--poll SEC]
```

bridge는 노드 하나(`flux_bridge_<pid>`)를 띄우고, `--poll`초(기본 1)마다 채널 목록을 읽는다. 그리고 채널마다 아래 조건이 모두 맞는 동안에만 relay를 돌린다. 그래서 아무도 구독하지 않는 채널에는 비용이 들지 않는다.

- 발행자가 살아 있고 이름이 정확히 알려진 채널이다.
- 채널의 fingerprint가 설치된 `.msg` 타입(`AMENT_PREFIX_PATH`)과 맞는다. adapter를 설치하지 않은 타입도 된다.
- bridge 말고 그 이름의 ROS 2 구독자가 있다.

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

기본 `echo`·`hz`를 바로 띄우면, bridge가 다음에 poll할 때까지 토픽이 없다. 그래서 두 verb는 먼저 아무 일도 하지 않는 구독을 만든다. 그리고 bridge의 발행자가 graph에 나타날 때까지 기다린 뒤 기본 verb를 그대로 돌린다. 인자는 기본 verb의 인자에 `--bridge-timeout SEC`(기본 10)가 더해진 것이다. 메시지 타입은 flux가 채널의 fingerprint로 찾으므로 DDS 발견을 기다리지 않는다. 두 verb를 쓰려면 bridge가 떠 있어야 한다.

## flux_bag

```bash
ros2 bag record_flux -a                         # 모든 flux 채널과 ROS 토픽
ros2 bag record_flux /camera/image /imu         # 이름으로. flux와 ROS 이름은 한 목록이다
ros2 bag play_flux rosbag2_2026_10_23-10_00_00
ros2 bag convert_flux rosbag2_2026_10_23-10_00_00 -o converted
```

### 녹화

```text
ros2 bag record_flux [-a] [TOPIC ...] [-e REGEX] [-x REGEX] [--exclude-topics TOPIC ...]
                     [-o OUT] [--poll SEC] [--max-cache-size BYTES]
```

flux 채널 `NAME`의 프레임은 토픽 `NAME/flux`에 `flux_msgs/msg/FluxFrame`으로 들어간다. ROS 토픽은 같은 bag에 그대로 들어간다.

```text
std_msgs/Header header   # commit 시각
string flux_topic        # 채널 이름
uint64 fingerprint       # 채널의 스키마 지문
string codec             # "raw"
uint8[] meta             # raw: slot_size u32, slot_count u32, dtype u8, ndim u8, shape u64[ndim]
uint8[] data             # 슬롯에 있던 프레임 바이트 그대로
```

| 항목 | 값 |
| --- | --- |
| 선택 | `ros2 bag record`와 같은 규칙을 flux 채널 이름과 ROS 토픽 이름에 똑같이 적용한다. `--exclude-topics`는 rosbag2 0.26(Jazzy) 이상이 필요하다 |
| 채널 찾기 | `/dev/shm`의 변화를 inotify로 바로 본다. 그 밖에는 `--poll`초(기본 1)마다 다시 읽는다. 10초 넘게 내려간 채널은 놓는다 |
| 복사 | 채널 스레드가 프레임을 자기 버퍼로 복사하고 슬롯을 바로 돌려준다. bag 쓰기는 스레드 하나가 한다 |
| 쓰기 대기 | 디스크가 느린 동안 프레임은 메모리에서 기다린다. 상한은 `--max-cache-size`(기본 1 GiB)이고, 넘는 프레임은 버리고 알린다. 발행자는 기다리지 않는다 |
| 시각 | send 시각과 header stamp는 commit 시각, recv 시각은 take 시각이다 |
| 일시정지 | `~/pause`·`~/resume` 서비스(rosbag2와 같다). 멈춘 동안의 프레임은 받되 쓰지 않는다 |
| 저장 형식 | rosbag2 기본값(Jazzy는 mcap) |
| 끝날 때 | 채널마다 큐에 넣은 수, 캐시가 차서 버린 수, `lost`를 로그로 남긴다 |

### 재생

```text
ros2 bag play_flux BAG [-r RATE]
```

`FluxFrame` 토픽은 파일에서 읽어 바로 원래 채널에 발행한다. 이때 bag에 기록된 fingerprint, `slot_size`, `slot_count`를 쓴다. 나머지 토픽은 rosbag2 player가 재생한다. flux 채널은 자기 시계를 따른다. player의 pause, resume, rate(서비스와 키보드)는 이 시계도 함께 움직인다. play_next와 burst는 ROS 토픽을 옮기고, flux 채널은 그 뒤를 따라간다. ROS 메시지가 다 떨어지면 play_next와 burst는 flux 프레임을 옮긴다. Humble(rosbag2 0.15)에서는 ROS 토픽만 옮기고 flux 채널은 그대로다.

- 재생은 발행자를 만든 뒤 200 ms 기다리고 시작한다. 먼저 떠 있던 구독자가 붙을 시간을 주기 위해서다.
- 프레임의 `slot_size`나 `slot_count`가 바뀌면 발행자를 다시 만든다. dtype x shape가 데이터 크기와 다른 프레임은 건너뛴다.
- seek는 ROS 토픽만 옮긴다.

### 변환

```text
ros2 bag convert_flux BAG -o OUT
```

bag을 새 bag으로 옮긴다. `FluxFrame`의 fingerprint가 설치된 타입을 가리키면, 그 프레임을 ROS 메시지로 바꿔 채널 이름의 토픽에 넣는다. 녹화 시각은 그대로 둔다. 타입을 못 찾거나 같은 이름의 ROS 토픽이 다른 타입이면 `FluxFrame`으로 남긴다. 나머지 토픽은 그대로 복사한다. 변환한 토픽은 flux 없이 읽을 수 있다. 남은 `FluxFrame`을 읽으려면 `flux_msgs`가 있어야 한다. 스키마와 맞지 않는 프레임이 있으면 변환이 exit code 1로 멈춘다.

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
