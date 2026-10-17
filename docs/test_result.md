# flux2 실노드 테스트 결과

[test.md](test.md)의 경우의 수 가운데 `flux_ws/src/flux2_test_pkg`의 하네스가 만드는 것을 실제 rclcpp·rclpy 노드 프로세스로 돌린 결과다. 아래 수치는 모두 2026-10-06에 lyrical 호스트에서 다시 돌린 것이다. 그날 호스트에서는 다른 빌드가 같이 돌았다(load average 30~60, 12코어). 지연의 p99·max와 장시간의 빠짐에는 그 부하가 섞여 있다.

- 노드: `cpp_pub`·`py_pub`(IMU 100 Hz, FHD rgb8 이미지 30 Hz, f32 배열), `cpp_sub`·`py_sub`(모든 프레임의 내용을 검사하고 발행자별 seq, 빠짐, `lost`, 지연을 기록)
- 러너: `test/scenarios.py`(60개 케이스), `test/longrun.py`(장시간)
- 실행: 각 실행에 자기 `/dev/shm`을 준다. `flux2` CLI 케이스는 cargo로 빌드한 `flux2`의 경로를 `FLUX2_CLI`로 받는다.

```bash
cd flux_ws/src/flux2_test_pkg/test
bwrap --dev-bind / / --tmpfs /dev/shm -- python3 scenarios.py OUT_DIR [-k SUBSTRING ...]
bwrap --dev-bind / / --size 67108864 --tmpfs /dev/shm -- python3 scenarios.py OUT_DIR -k "small /dev/shm"
bwrap --dev-bind / / --tmpfs /dev/shm -- python3 longrun.py OUT_DIR MINUTES
```

## 판정

구독자가 받은 것을 그 프레임을 보낸 발행자의 최종 통계와 대조한다. 발행자는 마지막으로 commit에 성공한 seq(`last_ok`)를 남긴다.

| 검사 | 기준 |
| --- | --- |
| 손상·역행 | 내용 검사 실패 0, 같은 발행자 안에서 seq 역행 0 |
| 끝까지 받음 | 발행자를 먼저 멈추고 0.5초 뒤 구독자를 멈춘다. 구독자가 받은 마지막 seq가 `last_ok`와 같아야 한다 |
| 빠진 프레임 | 첫 프레임 뒤에 빠진 seq 수가 같은 구간에서 그 구독의 `lost`가 늘어난 수와 같아야 한다. `lost`가 세지 않은 빠짐은 하나도 허용하지 않는다 |
| `lost`로 센 빠짐 | 구독자가 느려 depth 밖으로 밀려난 프레임이다. 빨리 받는 구독자는 1% 이하, 장시간은 0.1% 이하 |
| 종료 코드 | SIGINT에 모든 노드가 0으로 끝난다. 거절은 종료 코드 1과 이유를 담은 메시지여야 하고, abort나 다른 신호로 끝나면 실패다 |
| message_filters | 짝이 올 때마다 큐에 든 view를 다시 읽어, 그 stamp의 프레임 내용인지와 두 입력이 같은 프레임인지 본다 |
| depth | 100 ms씩 걸리는 depth 1 구독자가 받은 프레임의 지연 p50은 100 ms 미만, 같은 조건의 depth 8 구독자는 150 ms 이상 |

지연은 첫 1초(DDS discovery, attach)를 빼고 잰다. `lost`는 프레임마다 읽어 둔다. `lost`는 구독의 수명 전체에 걸쳐 누적하므로, 판정은 한 구간의 처음과 끝 값의 차이로 한다.

### 판정이 잡는 변형

판정이 실제로 실패할 수 있는지 코드를 일부러 망가뜨려 확인했다. 아래 변형은 모두 이전 하네스에서 PASS했고, 지금 하네스에서는 해당 케이스가 FAIL한다.

| 변형 | FAIL하는 케이스 | 걸린 검사 |
| --- | --- | --- |
| Python 구독자가 첫 프레임 뒤 세지 않음 | 2절 7개, 1 → 8 | 마지막 seq(예: 522 중 1) |
| Python take 스레드가 예외로 죽음, 또는 조용히 끝남 | take()·take_blocking() | 마지막 seq, 종료 코드, 스레드 오류 |
| Python 구독자가 110개 중 1개를 버림 | cpp → cpp+py, 1 → 8, 8개 중 1개만 느림 | 빠진 프레임 1, `lost` 0 |
| 장시간에서 Python 구독자 5개가 200개 뒤 SIGKILL | `longrun.py` | died, 종료 코드 -9, 마지막 seq |
| message_filters 큐가 첫 프레임만 들고 있음(C++, Python 각각) | message_filters 3개 | 짝의 내용 불일치 |
| depth 1이 depth 8처럼 동작 | depth 1 느린 구독자 | p50 254 ms |
| core가 `lost`를 세지 않음 | depth 1 느린 구독자, 200 ms 멈춤, 8개 중 1개만 느림 | 빠진 프레임 수와 `lost` 불일치 |
| core가 depth 창 안에서 덮인 프레임을 `lost`에 넣지 않음 | 200 ms 멈춤(링 16, depth 32) | 빠진 프레임 수와 `lost` 불일치 |
| QoS 거절, 두 번째 발행자, 17번째 구독자를 abort로 바꿈 | 각 거절 케이스 | 종료 코드 -6 |

## 요약

| 환경 | PASS | INFO | FAIL | 비고 |
| --- | --- | --- | --- | --- |
| lyrical 호스트, `scenarios.py` 전체 | 57 | 5 | 1 | FAIL 1은 하네스의 시간 문제였다. 고친 뒤 그 케이스만 다시 돌려 PASS |
| lyrical 호스트, `scenarios.py` 전체(2026-10-07, 2차 수정 뒤) | 58 | 5 | 0 | |
| 작은 `/dev/shm`(64 MiB), lyrical | 1 | | 0 | 위 두 번째 명령 |
| 장시간 15분 두 번, lyrical | | | 2 | 손상, 죽은 프로세스, `lost` 밖의 빠짐은 없었다. `lost`로 센 빠짐이 0.1%를 넘었다(아래) |
| 장시간 15분, lyrical, 부하 없음(2026-10-07) | 1 | | | 빠짐 0, `lost` 0(아래) |
| Jazzy(Docker, numpy 1.26, Qt5) | | | | 다시 돌리지 않았다. 그 환경을 만드는 하네스가 저장소에 없다 |
| rviz2 Image display(Xvfb) | | | | 다시 돌리지 않았다. 하네스가 저장소에 없다 |

INFO 5개는 판정선이 없는 측정이다. 호스트의 `/dev/shm`이 커서 전체 실행에서는 작은 `/dev/shm` 케이스도 INFO로 끝난다.

전체 실행의 FAIL은 슬롯 크기 경계 케이스였다. 부하 때문에 Python 발행자가 고정 2초 안에 통계를 한 번도 쓰지 못했다. 이제 발행자가 17개를 보낼 때까지 기다린다.

## test.md 가운데 하네스가 만들지 않는 경우

| test.md | 상태 |
| --- | --- |
| 1 축: Jazzy, Docker 기본 seccomp | 하네스 없음 |
| 3 노드 추가 시점: spin 중 노드 생성 | 콜백 그룹과 구독 생성만 한다 |
| 4 `flux2::Executor` 노드 여러 개 | 없음. 섞기 케이스는 노드 둘을 executor 둘에 나눠 넣는다 |
| 5 거절: 슬롯 수를 넘는 borrow 합 | flux2가 거절하지 않는다. INFO로 측정만 한다 |
| 10 장시간 1시간 | `longrun.py OUT_DIR 60`으로 돌릴 수 있다. 이번에는 15분 |
| 11 `echo_flux2`, rviz display | 없음. bridge는 `hz_flux2`만 돈다 |

## 찾아서 고친 flux2 버그

이전 실행에서 찾은 것이다. 넷 다 수정을 되돌리면 실패하는 단위 테스트가 있다.

| 버그 | 증상(실노드) | 원인 | 수정 |
| --- | --- | --- | --- |
| C++ `flux2::Executor`에서 ROS가 flux를 굶김 | ROS Imu 100 Hz에 50 ms 작업을 걸면 FHD가 157개 중 3개만 옴 | rclcpp 작업을 flux를 보지 않고 한 번에 64개까지 돌림 | flux에 일이 있으면 1 ms 뒤 넘김(`Executor::ready()`, `flux2_executor_ready`) |
| C++에서 flux가 ROS를 밀어냄(위 수정의 첫 시도에서 나옴) | FHD에 50 ms 작업이면 ROS p50 314 ms | executable 하나마다 넘기면 가벼운 ROS callback이 50 ms마다 하나씩만 돎 | 1 ms 동안은 ROS를 이어서 돎(`kTurnNs`) |
| Python `flux2.Executor`에서 flux가 ROS를 굶김 | FHD에 50 ms 작업을 걸면 ROS Imu가 196개 중 3개만 옴 | rclpy는 준비된 task가 있으면 wait set을 보지 않음. dispatch task가 늘 준비돼 있었음 | dispatch가 1 ms 뒤 멈추고(`_TURN_NS`), 남은 프레임이 있으면 바로 다음 task |
| view를 쥔 구독자가 재시작한 발행자를 따라가지 못함 | message_filters 입력이 늦은 CameraInfo를 기다리는 사이 발행자가 재시작하면 영원히 멈춤 | 남은 view가 하나라도 있으면 새 매핑으로 바꾸지 않았음. view는 `Arc`로 옛 매핑을 스스로 붙잡으므로 기다릴 이유가 없음 | 조건 삭제 |

## flux2 밖의 한계

| 항목 | 관찰(2026-10-06) | 근거 |
| --- | --- | --- |
| rclpy `MultiThreadedExecutor` | `flux2.Executor`에 넣으면 FHD 128개 중 101개만 받고(27개는 `lost`) p99 58 ms | flux2 없이 rclpy만으로 재현된다(`test/mte_plain.py`). Single·Events executor는 빠짐 0 |
| DDS로 FHD 보내기(bridge) | relay는 30 Hz로 내보내는데 `hz_flux2`는 0.9 Hz | 기본 RMW로 6 MB 메시지를 보낼 때의 손실이다. IMU는 100 Hz 그대로 |
| `mlock` | `lock=True`인 FHD 발행자는 생성이 거절됨 | `RLIMIT_MEMLOCK`이 세그먼트를 덮지 못한다. 메시지가 그렇게 알려 준다 |

Jazzy rclpy가 SIGINT 때 드물게 `RCLError: failed to initialize wait set`을 낸다는 이전 관찰은 이번에 다시 보지 않았다(Jazzy를 돌리지 않음). `py_sub`는 컨텍스트가 닫힌 뒤의 이 오류만 정상 종료로 본다.

## 주요 수치(lyrical, 2026-10-06)

| 경우 | 결과 |
| --- | --- |
| IMU, C++ → C++ | 지연 p50 0.01 ms, p99 1.00 ms, 발행자의 마지막 프레임까지 빠짐 0 |
| IMU, C++ → Python | p50 0.09 ms, p99 1.94 ms |
| FHD, C++ → C++(6.2 MB 전부 검사 포함) | p50 1.08 ms, p99 4.30 ms |
| FHD, C++ → Python | p50 0.87 ms, p99 2.42 ms |
| FHD 1 → 8(C++ 4, Python 4) | 8개 모두 빠짐 0, 마지막 프레임까지 받음, 최악 p99 13.8 ms, 발행자 backpressure 0 |
| 8개 중 1개만 느림(200 ms) | 느린 구독자는 240개 중 200개를 건너뜀(모두 `lost`), 나머지 7개 빠짐 0 |
| depth 1 느린 구독자(100 ms) | 받은 프레임 지연 p50 17 ms(C++), 21 ms(Python). 같은 조건의 depth 8은 255 ms |
| 같은 노드에서 ROS 50 ms 작업, PartitionedExecutor | flux p99 4.9 ms(C++), 1.0 ms(Python), 빠짐 0 |
| 같은 노드에서 flux 50 ms 작업, PartitionedExecutor | ROS p99 0.4 ms(C++), 4.2 ms(Python) |
| 같은 노드에서 위 둘, `flux2::Executor` 한 스레드 | 서로 번갈아 돈다. ROS 50 ms 작업이면 flux p50 18 ms(54개 `lost`), flux 50 ms 작업이면 ROS p50 25 ms(C++) |
| 서비스·액션 | Trigger p50 0.1~0.3 ms, 최대 15 ms, 이미지 빠짐 0 |
| 발행자 SIGINT·SIGKILL 재시작 | 재접속 0.25~0.85 s, 두 run이 섞이지 않음, 각 run을 마지막 프레임까지 받음, 손상 0 |
| 구독자가 먼저 뜸 | 발행자 시작 0.28 s 뒤 첫 프레임 |
| 4개 슬롯을 모두 쥔 구독자 SIGKILL | 발행자가 슬롯을 회수해 곧바로 100 Hz 복귀 |
| 구독자 상한 16 | 17번째는 생성에서 `no free subscriber number`로 거절(종료 코드 1). 하나를 죽이면 새 구독자가 그 자리를 씀 |
| executor | `spin_until_future_complete`가 SUCCESS로 돌아옴. 다른 스레드의 `cancel()`·`stop()`·shutdown이 spin을 끝냄. 소스 127개까지 받고 128번째 거절. `spin_once`·`spin_some`·`spin_all`은 예외. 콜백 예외가 spin 호출자에게 감. `on_thread_start`가 그 그룹 스레드에서 돎. Reentrant 그룹 거절 |
| bag 녹화 → 재생 | 녹화 115개, 재생한 것을 C++·Python 구독자가 113개씩 받아 내용 검사 통과 |
| `/dev/shm` | 발행자가 끝나면 남는 항목 없음 |
| 작은 `/dev/shm`(64 MiB) | 기본 슬롯(8 MiB × 16)은 `fallocate (/dev/shm too small?)`로 생성 실패(종료 코드 1), 8 MiB × 4는 빠짐 0 |
| CPU(FHD 30 Hz) | 발행자 1%, 구독자 3~9%(6.2 MB 전수 검사 포함) |

## 장시간

2026-10-07, 다른 작업이 없는 lyrical 호스트에서 15분 한 번(2차 수정 뒤 트리). 판정은 PASS다.

| 항목 | 값 |
| --- | --- |
| 발행자 | FHD 27,017개, IMU 90,041개, backpressure 0, 실패 0 |
| FHD 구독자 8개(C++ 4, Python 4) | 모두 마지막 프레임까지 받음. 빠짐 0, `lost` 0, 손상 0. p99 1.8~2.2 ms, 최대 3.4 ms |
| IMU 구독자 2개 | 90,041개 모두 받음. 빠짐 0, `lost` 0. p99 C++ 1.2 ms, Python 6.1 ms |
| fd, `/dev/shm` | 프로세스마다 41~42개, `/dev/shm` 4개로 변하지 않음 |
| RSS | C++ 구독자 129.5 → 129.9 MiB, Python FHD 구독자 181.8 → 185.5 MiB, Python IMU 구독자 81.6 → 88.3 MiB |

아래는 2026-10-06, 부하가 큰 호스트에서 돌린 두 번이다.

lyrical 호스트, 15분씩 두 번. FHD 30 Hz를 구독자 8개(C++ 4, Python 4)가, IMU 100 Hz를 구독자 2개가 받았다. 두 번 다 판정은 FAIL이다. 모든 구독자가 발행자의 마지막 프레임까지 받았고, 손상, 죽은 프로세스, `lost` 밖의 빠짐은 0이었다. 그러나 `lost`로 센 빠짐이 0.1%를 넘은 구독자가 있었다. 빠짐은 호스트의 load average가 50을 넘은 몇 분에 몰렸다(첫 번째는 6~11분, 두 번째는 2~5분).

| 항목 | 첫 번째 | 두 번째 |
| --- | --- | --- |
| 발행자 | FHD 26,807개, IMU 84,950개, backpressure 0 | FHD 27,022개, IMU 88,217개, backpressure 0 |
| FHD 구독자, C++ | 빠짐 35~43(0.13~0.16%), p99 19 ms, 최대 441 ms | 빠짐 2~3(0.01%), p99 12 ms, 최대 33 ms |
| FHD 구독자, Python | 빠짐 344~397(1.3~1.5%), p99 28~32 ms | 빠짐 11~15(0.04~0.06%), p99 13 ms |
| IMU 구독자, C++ | 빠짐 1,130(1.3%) | 빠짐 76(0.09%) |
| IMU 구독자, Python | 빠짐 8,152(9.6%) | 빠짐 704(0.8%), 이 구독자만 0.1%를 넘음 |
| 손상, `lost` 밖의 빠짐, 죽은 프로세스 | 0, 0, 없음 | 0, 0, 없음 |
| fd, `/dev/shm` | 프로세스마다 45~47개, `/dev/shm` 5개로 변하지 않음 | 41~43개, 5개로 변하지 않음 |
| RSS | 7분까지 C++ 구독자 129 MiB, Python 구독자 182 MiB. 8분부터 호스트 메모리 회수로 줄었다(101 MiB, 120 MiB) | 2분까지 129 MiB, 181 MiB. 3분에 회수로 줄고(106 MiB, 126 MiB) Python 구독자는 그 뒤 150 MiB까지 다시 늘었다 |

빠짐은 모두 flux2가 `lost`로 센 것이다. 구독자가 depth 1을 제때 비우지 못한 것으로, 부하가 낮은 구간에서는 거의 늘지 않았다. 이전 문서의 "15분 빠짐 0"은 받은 개수를 보낸 개수와 비교하지 않은 판정이어서 근거가 되지 못한다. 부하가 없는 호스트에서 다시 돌린 위 결과는 빠짐이 0이었다.

## INFO 항목

| 케이스 | 결과 |
| --- | --- |
| 슬롯 수를 넘는 borrow 합 | 거절하지 않는다. slot 4에 `max_borrow` 3인 구독자 3개가 모두 붙고 받는다. 슬롯을 모두 쥐면 발행자가 backpressure를 받는다(별도 케이스에서 확인) |
| MemoryPolicy | `precommit`은 동작, `lock`은 위 `mlock` 한계 |
| 작은 `/dev/shm` | 호스트 `/dev/shm`(15 GiB)에서는 해당 없음. 위 두 번째 명령으로 판정한다 |
| CPU | 위 표 |
| `/dev/shm` 잔여 | 위 표 |
