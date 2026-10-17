# flux 실노드 테스트 경우의 수

실제 rclcpp·rclpy 노드를 프로세스로 띄워 확인할 조합이다. 기준 메시지는 두 가지다.

| 이름 | 타입 | 주기 | 크기 |
| --- | --- | --- | --- |
| IMU | `sensor_msgs/Imu` | 100 Hz | 약 300 B |
| IMG | `sensor_msgs/Image` 1920x1080 rgb8 | 30 Hz | 약 6.2 MB |

모든 경우에 공통으로 확인하는 것: 손상 프레임 0, 같은 발행자 안에서 seq 역행 0, 발행자가 마지막으로 보낸 프레임까지 받았는지, 빠진 프레임이 모두 flux `lost`에 잡혔는지, 빠진 프레임 수, 지연(p50·p99·max), 발행자 backpressure 수, 프로세스 종료 코드.

## 1. 축

| 축 | 값 |
| --- | --- |
| 발행 언어 | C++, Python |
| 구독 언어 | C++, Python |
| 프로세스 배치 | 발행·구독 다른 프로세스, 같은 프로세스, 같은 노드 |
| 메시지 | IMU, IMG, `NO_SCHEMA` 바이트, 타입 있는 numpy 배열(dtype·shape) |
| 받는 방식 | 콜백(executor), `take()`, `take_blocking()`, message_filters `Subscriber` |
| executor | `flux::Executor`, Python `flux.Executor`, `PartitionedExecutor`(C++·Python) |
| rclpy 쪽 executor | `SingleThreadedExecutor`, `MultiThreadedExecutor`, `EventsExecutor` |
| QoS depth | 1, 8, 슬롯 수와 같음 |
| QoS replay | 0, 1, depth와 같음 |
| QoS max_borrow | 1, 2, 슬롯 수 근처 |
| 구독자 수 | 1, 2, 8, 상한, 상한+1 |
| 배포판 | Humble(numpy 1.21, Qt5), Jazzy(numpy 1.26, Qt5), Lyrical(numpy 2.x, Qt6) |
| 실행 환경 | 호스트, Docker 기본 seccomp(`/dev/shm` 64 MB) |
| executor 대기 | merged(`futex_waitv`), parker 스레드 fallback(5.16 미만 커널, `FLUX_DISABLE_FUTEX_WAITV=1`) |

## 2. 언어 조합

| 발행 | 구독 | IMU | IMG |
| --- | --- | --- | --- |
| C++ | C++ | O | O |
| C++ | Python | O | O |
| Python | C++ | O | O |
| Python | Python | O | O |
| C++ | C++·Python 동시 | O | O |
| Python | C++·Python 동시 | O | O |

## 3. ROS 토픽과 함께 구독

| 경우 | 구성 |
| --- | --- |
| 같은 executor | 한 노드에 flux 구독(IMG) + ROS 구독(IMU, DDS) + 타이머 |
| 같은 콜백 그룹 | flux 구독과 ROS 구독이 한 MutuallyExclusive 그룹 |
| 다른 콜백 그룹 | 그룹을 나누고 `PartitionedExecutor`로 스레드 분리 |
| 서비스·액션 | flux 구독 중 서비스 호출, 액션 goal 처리 |
| 느린 ROS 콜백 | ROS 콜백이 50 ms 막을 때 flux 콜백 지연 |
| 느린 flux 콜백 | flux 콜백이 50 ms 막을 때 ROS 콜백 지연 |
| message_filters | flux IMG + ROS CameraInfo `TimeSynchronizer`, `ApproximateTime` |
| message_filters 둘 다 flux | IMG 두 채널 동기화 |
| 같은 이름 | 같은 토픽 이름으로 ROS 토픽과 flux 채널이 동시에 존재 |
| 노드 추가 시점 | spin 전에 추가, spin 중 노드·콜백 그룹·엔티티 생성 |

## 4. executor

| executor | 경우 |
| --- | --- |
| `flux::Executor` | flux만, flux+ROS, 노드 여러 개, `spin_until_future_complete` |
| `flux::Executor` | 다른 스레드에서 `cancel()`, spin 중 `rclcpp::shutdown()` |
| `flux::Executor` | 채널 127개(상한), 128개째 거절 |
| `flux::Executor` | `spin_once`·`spin_some`·`spin_all` 호출 시 예외 |
| Python `flux.Executor` | rclpy executor 세 종류 각각, `stop()`, 컨텍스트 shutdown |
| `PartitionedExecutor` | 그룹마다 스레드, 한 그룹 막힘이 다른 그룹에 영향 없음 |
| `PartitionedExecutor` | `on_thread_start` hook, 한 스레드 예외가 전체 spin으로 전파 |
| `PartitionedExecutor` | spin 중 새 그룹 생성, Reentrant 그룹 거절 |
| `PartitionedExecutor` Python | flux 전용 그룹에 ROS 엔티티가 들어오면 거절 |
| 섞기 | 한 프로세스에 `flux::Executor`와 `PartitionedExecutor` 동시 |
| 콜백 예외 | C++ throw, Python raise가 spin 호출자에게 도달 |

## 5. QoS

| 경우 | 구성 |
| --- | --- |
| 최신만 | depth 1, 느린 구독자(작업 100 ms) |
| 버퍼 | depth 8, 순간적인 지연 뒤 따라잡기 |
| replay | 발행자가 N개 보낸 뒤 구독 시작, replay=k면 마지막 k개 수신 |
| replay 없음 | 늦게 붙은 구독자는 붙은 뒤 프레임만 수신 |
| view 오래 쥠 | max_borrow만큼 view를 쥔 채 유지, 발행자 backpressure |
| QoS 섞기 | 한 채널에 depth·replay·max_borrow가 다른 구독자 여럿 |
| 거절 | depth 0, replay > depth, max_borrow 0, 슬롯 1, 슬롯 수를 넘는 borrow 합 |
| 크기 | 프레임 > 슬롯이면 `TOO_LARGE`, 슬롯 크기 경계값 |

## 6. fan-out

| 경우 | 구성 |
| --- | --- |
| 1 → 2, 1 → 8 | IMG, C++·Python 섞어서 |
| 상한 | 구독자 상한까지 붙고, 상한+1은 attach 거절 |
| 느린 구독자 격리 | 8개 중 1개만 느릴 때 나머지 빠진 프레임 0 |
| 같은 프로세스 여러 구독 | 한 노드에 같은 채널 구독 여러 개 |
| 다른 채널 동시 | IMU·IMG를 한 executor에서 동시 |

## 7. 발행자 껐다 켜기

| 경우 | 구성 |
| --- | --- |
| 정상 종료 후 재시작 | SIGINT, 1 s 뒤 같은 언어로 재시작 |
| 강제 종료 후 재시작 | SIGKILL, 1 s 뒤 재시작 |
| 언어 바꿔 재시작 | C++ → Python, Python → C++ |
| geometry 바꿔 재시작 | 슬롯 크기·슬롯 수가 다른 발행자로 재시작 |
| fingerprint 바꿔 재시작 | 다른 타입으로 같은 이름 발행 |
| 구독자보다 늦게 시작 | 구독자가 먼저 떠 있고 발행자가 나중에 시작 |
| 두 번째 발행자 | 살아 있는 발행자가 있을 때 `PUBLISHER_EXISTS`, 죽은 뒤 인계 |
| 빠른 반복 | 0.2 s 간격으로 10회 재시작 |
| 쓰는 도중 종료 | loan 중 SIGKILL |

확인: 구독자 재접속 시간, 재시작 뒤 수신 재개, 이전 run 프레임과 섞이지 않음, `/dev/shm` 잔여 세그먼트.

## 8. 구독자 껐다 켜기

| 경우 | 구성 |
| --- | --- |
| 정상 종료 후 재시작 | SIGINT, 재시작 |
| 강제 종료 | SIGKILL, view를 쥐지 않은 상태 |
| view 쥔 채 강제 종료 | 콜백 안에서 sleep 중 SIGKILL, 발행자가 슬롯을 회수하는지 |
| 빠른 반복 | 0.2 s 간격으로 10회 |
| 상한 근처 반복 | 구독자 상한까지 채운 뒤 하나씩 죽이고 새로 붙이기 |
| 노드만 파괴 | 프로세스는 살아 있고 노드·구독 객체만 파괴 |

확인: 발행자 publish가 멈추지 않음, 다른 구독자 빠진 프레임 0, 새 구독자 수신 시작.

## 9. 이름과 타입

| 경우 | 구성 |
| --- | --- |
| namespace·remap | 상대 이름, `--ros-args -r`, namespace가 다른 노드 |
| fingerprint 불일치 | IMU 구독자가 IMG 채널 이름에 붙음 |
| 같은 이름 다른 타입 | 같은 key, fingerprint가 다른 두 채널 공존 |
| `NO_SCHEMA` | 타입 없는 발행과 타입 있는 구독 |

## 10. 자원

| 경우 | 구성 |
| --- | --- |
| `/dev/shm` 부족 | Docker 64 MB에서 IMG 기본 슬롯(8 MiB x 16) 생성, ENOSPC |
| 메모리 정책 | `MemoryPolicy` 각 값, pages locked |
| 장시간 | IMG 30 Hz, 구독자 8개, 1시간, RSS·`/dev/shm`·fd 수 변화 |
| CPU | 구독자 수별 발행자·구독자 CPU |

## 11. 도구와 함께

| 경우 | 구성 |
| --- | --- |
| bridge | 구독 중 `ros2 topic echo_flux`·`hz_flux` |
| bag | 구독 중 `record_flux`, 재생한 채널을 실노드가 구독 |
| rviz | flux_tools display가 추가 구독자로 붙음 |
| CLI | `flux topic` 목록에 발행자·구독자 노드 표시 |
