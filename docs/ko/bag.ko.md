# flux bag

`flux_bag`은 flux 채널을 rosbag2 bag에 녹화하고 flux로 다시 재생한다. 프레임은 양방향 모두 DDS를 거치지 않는다. 같은 bag의 ROS 토픽은 stock rosbag2 recorder와 player가 맡으므로 bag 하나에 둘이 같이 들어간다.

```bash
ros2 run flux_bag record -a                       # 모든 flux 채널과 모든 ROS 토픽
ros2 run flux_bag record /cam/left /cam/right     # 이름으로. flux와 ROS 이름은 한 목록이다
ros2 run flux_bag play rosbag2_2026_09_17-16_35_31
```

## bag에 무엇이 들어가나

flux 채널은 wire에 ROS 타입이 없다. 바이트와 fingerprint뿐이다. bag은 프레임마다 `flux_msgs/msg/FluxFrame` 하나를 토픽 `<채널>/flux`에 넣는다.

```text
std_msgs/Header header      # 받은 시각
string  flux_topic          # 채널 이름, "/cam/left"
uint64  fingerprint         # 채널의 스키마 fingerprint
string  codec               # "raw"
uint8[] meta                # raw: slot_size, slot_count, dtype, ndim, shape
uint8[] data                # raw: slot에 있던 프레임 바이트 그대로
```

`header.stamp`는 recorder가 프레임을 가져간 시각이다. `.msg`에 stamp가 있다면 그 프레임 자체의 stamp는 `data` 안에 있고 재생 때 그대로 돌아온다. bag 토픽을 채널 이름과 다르게 두는 이유는, 녹화 중 bridge relay가 `/cam/left`에 `sensor_msgs/Image`를 내고 있어도 같은 bag 토픽에 섞이지 않게 하기 위해서다.

`ros2 bag info`, `ros2 bag play`, rosbag2를 읽는 모든 도구가 이 bag을 연다. flux 밖에서 프레임을 읽으려면 `FluxFrame`을 역직렬화하고 `data`를 어댑터의 `View`에 넘긴다(`sensor_msgs_flux.image.Image.View(np.frombuffer(m.data, np.uint8))`).

## record

```bash
ros2 run flux_bag record [-a] [TOPIC ...] [-e REGEX] [-x REGEX] [--exclude-topics TOPIC ...] [-o OUT] [--poll SEC]
```

| 항목 | 값 |
| --- | --- |
| 선택 | `ros2 bag record`와 같은 규칙. flux 채널 이름과 ROS 토픽 이름에 똑같이 적용한다 |
| 탐색 | `--poll`초(기본 1)마다 flux 채널을 열거한다. 이 도메인에 살아 있고 선택에 맞는 채널은 그때부터 스레드 하나를 받는다 |
| 복사 | flux slot -> 직렬화 버퍼 한 번. 그 뒤 borrow를 놓는다. 파일 쓰기는 writer가 한다 |
| QoS | `depth=1`, `max_borrow=2`. 스레드가 제때 가져가지 못한 프레임은 종료 때 찍는 채널별 `lost`에 들어간다 |
| 저장 | rosbag2 기본값. mcap, chunk 압축 없음, write cache 없음. `-o`는 `ros2 bag record`와 같은 의미다 |
| ROS 토픽 | 같은 writer 위의 `rosbag2_transport::Recorder`, 같은 옵션 |

Orin에서 1920x1200 bgr8 채널 하나 30 Hz(프레임당 6.9 MB)로 측정: 207 MB/s 기록, 71초 동안 lost 0.

## play

```bash
ros2 run flux_bag play BAG [-r RATE]
```

`FluxFrame` 토픽은 파일에서 읽어 `flux_topic`의 채널에 기록된 fingerprint, `slot_size`, `slot_count` 그대로 발행한다. flux로 쓴 구독자는 라이브 채널을 받던 것과 똑같이 재생을 받는다. 나머지 토픽은 `FluxFrame` 토픽을 제외한 `rosbag2_transport::Player`가 맡으므로 프레임은 DDS에 닿지 않는다. 양쪽 다 bag 시작 시각 기준으로 같은 clock에서 페이싱한다.

| 항목 | 값 |
| --- | --- |
| publisher | 토픽마다 첫 프레임에서 만든다. 먼저 기다리고 있던 구독자는 자기 executor의 다음 tick에 붙으므로 30 Hz의 첫 몇 장이 앞설 수 있다 |
| 복사 | 파일 -> slot 한 번(`loan`, `memcpy`, `commit`) |
| 미지원 | `--loop`, 일시정지, 키보드 조작. `-r`만 |

## 이 문서가 아닌 것

살아 있는 flux 채널을 stock 도구로 보는 것은 `flux_bridge`다([bridge.md](bridge.ko.md)). 보는 쪽마다 DDS 복사 하나가 붙고, 녹화 경로가 아니다.
