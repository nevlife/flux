# flux bridge

`flux_bridge`는 flux 채널을 DDS 토픽으로 다시 발행한다. 공유 메모리에만 있는 채널을 stock ROS 2 도구(`rqt`, `ros2 topic`)가 읽게 하는 옆길이다. 녹화는 이 경로가 아니다. `flux_bag`이 채널을 공유 메모리에서 바로 bag에 쓴다([bag.md](bag.ko.md)). 데이터 경로에는 관여하지 않는다. 채널은 그 이름의 ROS 2 구독자가 있는 동안에만 중계되므로, 아무도 보지 않는 채널에는 비용이 붙지 않는다.

```bash
ros2 run flux_bridge bridge
```

노드 하나, `flux_bridge_<pid>`, 사용자가 직접 띄운다. 앱이 fork하지 않고 데몬도 없다. 도메인당 하나를 띄운다. 도메인은 프로세스가 결정하는 것(`FLUX_DOMAIN`, 없으면 `ROS_DOMAIN_ID`, 없으면 `0`)이고 노드와 같은 규칙이다.

## 하는 일

매초(`--poll`) 한 번 지나간다.

1. `flux.enumerate_topics()`. 자기 도메인이고, 발행자가 살아 있고, 키가 정확한 채널만 남긴다.
2. 채널의 fingerprint로 설치된 어댑터를 찾는다. 어댑터가 없는 채널은 한 번 로그를 남기고 건너뛴다. 설치돼 있지만 import가 실패하는 어댑터 모듈(예: 옛 `flux_gen`으로 생성한 것)은 에러를 담은 `flux: skipped adapter module` 한 줄을 stderr에 남기고 건너뛴다.
3. ROS graph에 `get_subscriptions_info_by_topic()`을 묻는다. bridge 자신의 노드는 뺀다. 외부 구독자가 하나라도 있으면 그 채널은 필요한 채널이다.
4. 필요한데 relay가 없는 채널은 relay를 시작하고, 더는 필요하지 않거나 발행자가 죽은 채널의 relay는 멈춘다.

relay는 스레드 하나다. `flux.Subscription(depth=1, max_borrow=1)`에 `take_blocking()`, `frame_to_msg()`로 ROS 메시지 객체를 만들고, 같은 이름의 rclpy publisher로 `publish()`한다. 뷰는 publish 전에 놓는다. 그래서 프레임 경로는 flux 슬롯 -> 복사 1회 -> CDR 직렬화 -> DDS이고, 이것이 flux 채널에 stock 도구를 쓰는 대가다. 타입은 `flux_gen`이 생성한 어댑터(프레임은 `<pkg>_flux.<name>`, 메시지 객체는 `<pkg>_flux.<name>_ros`)에서 오므로, bridge가 도는 곳에 어댑터 패키지가 설치돼 있어야 한다. `frame_to_msg()`가 스키마대로 읽지 못하는 프레임은 건너뛰고 세며, relay는 다음 프레임을 이어서 중계한다. relay가 멈출 때 남기는 로그 한 줄이 그 개수를 알려 준다.

| 항목 | 값 |
| --- | --- |
| DDS QoS | `depth=1`, best effort, volatile. flux 쪽의 최신 프레임 성질과 같다. 늦게 붙는 도구는 다음 프레임을 받지 오래된 프레임을 받지 않는다 |
| 노드 | `flux_bridge_<pid>`, 네임스페이스 `/`. `rqt_graph`에는 이 노드가 발행자로 보인다. flux 발행자는 DDS graph에 없다 |
| 활성화 지연 | 구독자가 나타난 뒤 최대 poll 한 주기 |
| 타임스탬프 | `header.stamp`는 프레임이 실은 값 그대로다. rosbag2는 bridge에서 받은 시각을 메시지마다 찍는다 |
| 어댑터 탐색 | `sys.path`의 모든 `<pkg>_flux` 패키지, 시작할 때 한 번. bridge를 띄우기 전에 어댑터를 설치한다 |

## verb

stock verb를 그냥 띄우면 첫 poll을 놓치고, `hz`는 발행자 없는 토픽을 기다린다. 이 패키지는 verb 둘을 등록한다. 아무 일도 하지 않는 구독을 하나 들고, bridge의 publisher가 graph에 오를 때까지 기다린 뒤, stock verb를 그대로 부른다.

```bash
ros2 topic echo_flux /cam/left --no-arr
ros2 topic hz_flux /cam/left
```

둘 다 stock verb의 모든 인자에 `--bridge-timeout`(기본 10초)을 더한 것이다. 채널 이름은 flux 쪽에서 찾으므로 살아 있는 flux 채널이어야 한다.

## 비용

Orin, ZED wrapper의 1280x720 NV12 프레임(3.4 MB) 30 Hz, 기본 rmw에서 측정.

| 단계 | 프레임당 |
| --- | --- |
| `frame_to_msg` | 1.5 ms |
| CDR 직렬화 | 1.5 ms |
| 3.4 MB를 로컬 구독자까지 DDS로 전달 | 나머지. 구독자가 있는 relay 하나가 약 20 Hz |

relay는 스레드 하나씩이고 변환은 GIL을 잡으므로, relay 여럿이 Python 한 코어를 나눠 쓴다. 카메라 셋과 55 MB mosaic을 동시에 중계했을 때 카메라당 7 Hz가 나왔다. 전체 속도로 빠짐없이 받아야 하는 스트림은 직접 구독 플러그인(`flux_tools`)을 쓰거나 `flux_bag`으로 녹화하거나 소비자를 flux로 쓴다.

## 하지 않는 것

- DDS -> flux. ROS 토픽의 것을 flux 채널에 쓰지 않는다.
- 서비스, 액션, 파라미터.
- 어댑터가 설치되지 않은 채널. `flux topic list`에는 보이고 bridge는 한 번 로그를 남긴다.
- 자기 죽음의 뒤처리. bridge가 끝나면 publisher가 graph에서 사라지고 도구는 토픽이 사라진 것을 본다. 다시 띄운다.
