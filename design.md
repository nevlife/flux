# flux Rust 이전 초기 설계

## 구조

```text
flux2_core (Rust)    세그먼트, discovery, liveness, channel, executor, PartitionedExecutor
   |
flux2_ffi (C ABI)    cbindgen 헤더. 최소 표면
   |            \
flux2_cpp            flux2_py
C ABI 위 헤더 래퍼    nanobind로 C ABI 직결
+ rclcpp 통합         + rclpy 통합

flux2_gen (Rust)     .msg -> C++/Python adapter 생성기, 설치된 .msg의 layout과 CDR 변환
flux2_std_adapters   표준 메시지 adapter를 설치된 .msg에서 한 벌 생성
flux2_bridge         flux2 채널 -> DDS 토픽 relay(C++ 노드), ros2 topic echo_flux2·hz_flux2
flux2_bag            flux2 채널을 rosbag2에 녹화·재생·변환, ros2 bag record_flux2·play_flux2·convert_flux2
flux2_msgs           bag이 프레임을 담는 FluxFrame
flux2_tools          rviz2 Image·PointCloud2·DepthCloud display
flux2_cli            `flux2` 명령(cargo가 빌드하고 colcon이 `bin`에 설치)
```

ROS 2 도구와 맞물리는 앱인 flux2_bridge, flux2_bag, flux2_tools는 `ros2/` 아래에 두고, 나머지 패키지는 저장소 루트에 둔다.

- loan·view 핸들은 호출자 메모리에 두는 storage 방식. loan 경로 힙 할당 없음
- `panic = "abort"`(dev·release 모두)
- wire 런타임은 `flux2/wire.hpp` 하나다. Python adapter는 이를 nanobind로 묶은 `flux2.wire`를 쓴다. Rust의 읽기 쪽 사본(`flux2_gen/src/wire.rs`)은 CDR 변환이 쓴다
- 언어: Rust로 쓴다. C++·Python은 rclcpp·rclpy·ros2cli·rviz가 요구하는 층(래퍼, 바인딩, ROS 노드, verb, display)에만 둔다. 빌드할 때만 도는 도구도 같다

## 가져갈 것

- 세그먼트 ABI 원칙: POD, 고정폭, bitfield 금지, hot atomic 별도 cache line, `#[repr(C)]` + 컴파일 타임 크기·offset assert
- 토픽별 고정 링(같은 크기 슬롯 N개, N >= 2), payload page 정렬. 슬롯이 1개면 구독자가 view 하나만 쥐어도 발행자가 막히므로, 공개 생성 함수가 `INVALID_ARGUMENT`로 거절한다. 세그먼트 자체는 1개로도 동작하며 내부 동시성 테스트가 이를 쓴다
- 슬롯 상태는 `commit_ticket` 하나다. `TICKET_EMPTY`(0) = 빈 슬롯, `TICKET_CLAIMED` = 발행자가 쓰는 중, 그 밖의 값 = 그 프레임의 ticket. 슬롯별 seqlock은 두지 않는다
- atomic refcount(슬롯 안 구독자 번호별 칸), claim과 borrow의 Dekker(seq_cst). 발행자: ticket을 CAS로 `TICKET_CLAIMED`로 바꾼 뒤 칸 확인, 칸이 0이 아니면 원래 ticket으로 되돌린다(아직 payload를 쓰지 않았다). 구독자: 자기 칸 +1 후 ticket 재확인
- claim은 구독자가 잡지 않은 슬롯 중 ticket이 가장 작은(가장 오래된) 슬롯을 고른다. 다음에 쓸 위치는 공유 헤더에 두지 않는다. 발행자는 자기 슬롯을 commit 순서(= ticket 순서)로 프로세스 안 큐에 들고, 앞에서부터 구독자가 잡지 않은 첫 슬롯을 claim한다. 그래서 비용이 슬롯 수가 아니라 잡힌 슬롯 수에 비례한다. loan은 commit하면 자기 ticket 자리로, abort하면 맨 앞으로 큐에 돌아간다
- `latest` 워드(마지막 commit의 ticket) + 전역 티켓
- futex wake + `waiters` gate(대기자 없으면 syscall 없음). 대기자는 `take_blocking`으로 기다리는 스레드와 executor에 붙은 구독이다. executor 구독은 붙어 있는 동안 계속 대기자로 세므로, 그런 구독이 있으면 publish마다 futex wake가 한 번 나간다
  - 대기 등록은 store 하나다. 구독자 번호가 있으면 그 번호의 칸에, 없으면(발행자 자신의 프로세스, heap 채널) 헤더 칸에 센다. 발행자는 헤더 칸과, 지금까지 쓰인 가장 큰 번호(`readers_high`)까지의 번호 칸을 읽어 하나라도 0보다 크면 깨운다
  - 두 store로 나눠 세면 그 사이에서 죽은 프로세스가 합계를 영구히 올려 둔다. 그래서 하나로 했다
  - 회수는 죽은 번호의 칸을 통째로 0으로 한다. 발행자는 대기자가 있는 동안 1초에 한 번 죽은 번호를 찾는다
- `take`(티켓 커서). 공유 ticket 색인(`index[t % L] = slot`, L은 슬롯 수 × 2 이상인 2의 거듭제곱, waiter 칸과 슬롯 헤더 사이)으로 `cursor+1 ..= latest`를 따라간다. commit은 ticket을 쓰기 전에 색인을 쓴다. 색인은 힌트라서 슬롯의 ticket이 맞는지 확인한 뒤 borrow와 재확인을 그대로 한다. 위조된 색인은 프레임을 숨길 수는 있어도 범위 밖을 읽게 하지 못한다. 커서가 색인 범위보다 뒤처지면 모든 슬롯을 훑는다
  - 슬롯 1024개에서 publish+take 1.1 us → 71 ns, 65535개에서 88 us → 72 ns. 대신 슬롯 4개 프로세스 간 spin ping-pong 단방향이 약 175 → 190 ns다
- QoS: `depth`, `durability`, `max_borrow`
- 빈 view 사유 카운터(`Refused`), `lost`, `dropped`. `lost`는 구독의 수명 전체에 걸쳐 누적한다. 재attach나 발행자 종료로 0이 되지 않는다
- `take_blocking`은 100 ms 조각으로 기다리며 조각마다 멈춤 조건을 본다. C++·Python 모두 노드의 context가 끝나면 빈 결과로 돌아오고(rclcpp `wait_for_message`처럼, 다만 100 ms 안에), `max_borrow`만큼 view를 쥐고 있으면 기다리지 않고 바로 빈 결과다. 조각 사이에도 한 스레드 규칙의 guard를 놓지 않는다
- signpost + 유일 이름 세그먼트, epoch 회전, 재attach. epoch는 wrap할 때 0(발행자 없음)을 건너뛴다. `max_borrow`에 막힌 take도 epoch 회전을 확인하므로, view를 다 쥔 구독자도 재시작한 발행자에 다시 붙는다. 재attach하면 새 발행자의 첫 프레임부터 읽어, depth 안의 것은 받고 밀려난 것은 `lost`로 센다. 재attach가 `NOT_READY` 말고 다른 이유로 거절되면 core가 그 에러를 epoch가 바뀔 때까지 들고(`attach_error`, `flux2_channel_attach_status`), 래퍼는 이를 실패 격리 규칙으로 넘긴다. 옛 세그먼트의 view는 borrow를 옛 세그먼트에 돌려준다
- 발행자 생존 확인(orphan probe, syscall 3개)은 빈 take `PROBE_EVERY`(8)번마다, 그리고 100 ms에 한 번까지만 한다. executor는 깰 때마다 조용한 구독도 take하므로, 횟수만으로 세면 빠른 토픽 하나에 조용한 토픽 15개인 executor에서 초당 2만 번이 된다. epoch 회전 확인은 load 하나라 빈 take마다 한다
- 이름 규칙(layout version, topic, fingerprint). layout version은 지금 6이다(4: 대기 등록을 store 하나로, 5: `readers_high`, 6: `CudaIpc` storage와 장치 UUID). key는 `[A-Za-z0-9_/]`만 쓴다. 다른 문자는 평탄화하면 `.`가 되어 `cam.left`, `cam/left`, `cam-left`가 한 채널로 합쳐지므로 이름을 만들 때 `InvalidArgument`로 거절한다. ROS 이름은 해당하지 않는다
- 이 버전의 signpost가 없는데 같은 key와 fingerprint의 다른 layout version signpost가 있으면 open이 두 버전을 적은 `Mismatch`를 낸다. /dev/shm 목록은 프로세스 안에서 100 ms에 한 번까지만 읽는다
- schema fingerprint
- OFD 락 liveness(발행자: 세그먼트 바이트 0, 구독자 번호 `i`: 바이트 `1+i`). 락은 생존 확인에만 쓴다. 슬롯 접근은 atomic만으로 하고, 공유 헤더에 lock 워드는 없다
- 구독자 payload 매핑 `PROT_READ`. 한 프로세스의 구독들은 세그먼트 매핑 하나를 같이 쓴다(프로세스 전역 표, `Weak` 참조). `outstanding`, `dropped` 같은 구독별 상태는 구독마다 따로 둔다. `MemoryPolicy`는 요청들의 합집합이고 한번 켠 것은 끄지 않는다
- attach 시 `layout_config_ok`, `st_size` 대조, meta 경계 검사
- fork 시 liveness fd 정리(D-102). fork한 자식이 물려받은 채널은 공유 카운트를 건드리지 않는다. view·loan 해제와 `waiters` 증감을 건너뛰고, 새 borrow와 발행은 거절한다(fork 세대 번호). 자식이 새로 연 채널은 자기 구독자 번호를 받는다. 물려받은 구독의 take는 C++·Python에서 `std::logic_error`(`RuntimeError`)로 거절하고, core `take_blocking`은 기다리지 않고 돌아온다. 자식에게는 borrow를 주지 않으므로 기다림이 끝나지 않기 때문이다. 물려받은 view·loan의 데이터 접근은 막지 않는다. iceoryx2와 zenoh는 fork를 다루지 않는다. agnocast는 heaphook의 `pthread_atfork`로 자식의 할당을 일반 힙으로 돌릴 뿐, 자식이 공유메모리를 쓰는 경우는 지원하지 않는다. 구독자 번호 lease 표와 매핑 표는 `fork()`가 기다리는 lock 아래에서만 고친다. 자식은 자기 첫 구독에서 다시 sweep한다
- `MemoryPolicy`(사전 커밋, mlock)
  - `MemoryPolicy`와 상관없이 세그먼트의 제어 영역(헤더, 대기 칸, ticket 색인, 슬롯 헤더와 구독자 칸, 곧 payload 앞까지)은 attach할 때 mlock한다. 기본 세그먼트에서 한 페이지다. `fallocate`는 공간만 잡을 뿐 상주를 보장하지 않는다. Jetson은 보통 zram swap이 켜져 있어 오래 쓰지 않은 페이지가 압축되고, 그 페이지가 제어 영역이면 take에서 major fault가 나고, wake word가 있는 페이지면 `futex_wake` 안에서도 난다(공유 futex key는 그 페이지를 찾는다)
  - 이 mlock은 실패해도 무시한다. `RLIMIT_MEMLOCK`에 들어가므로 한도가 작으면 잠기지 않을 뿐이다. 커널은 이미 잠긴 페이지를 다시 세지 않으므로 그 뒤 `MemoryPolicy::lock`이 세그먼트 전체를 잠글 때 이중으로 세지 않는다. 다만 다른 채널의 제어 영역도 같은 한도에 들므로, 한도가 세그먼트 크기와 거의 같으면 전체 mlock이 전보다 일찍 실패할 수 있다. 전체 mlock의 실패는 전처럼 오류다
  - mlock 대신 발행자와 구독자의 systemd 유닛에 `MemorySwapMax=0`(cgroup v2 `memory.swap.max=0`, zswap도 끄려면 `memory.zswap.max=0`)을 둘 수 있다. `RLIMIT_MEMLOCK`이 필요 없다. 세그먼트 전체를 잠그려면(`MemoryPolicy::lock`) systemd는 `LimitMEMLOCK=`, docker는 `--ulimit memlock=-1:-1`로 한도를 올린다. `mlockall(MCL_FUTURE)` 아래에서는 한도를 넘는 mmap 자체가 EAGAIN이다. tmpfs `noswap`(6.4)은 JetPack 6(5.15)에 없다. 컨테이너가 `memory.high`에 닿으면 그 cgroup의 shmem부터 swap된다
  - Docker의 `/dev/shm`은 기본 64 MiB라 C++ 기본(8 MiB × 16 슬롯 = 128 MiB) 채널 하나도 만들지 못하고 `fallocate`가 ENOSPC다. `--shm-size`로 채널들의 합보다 크게 둔다(`--ipc=host`면 호스트의 `/dev/shm`을 쓴다). 세그먼트 페이지는 `fallocate`한 발행자의 memory cgroup에 과금된다
- 한 번의 대기로 여러 채널과 제어 신호를 기다리는 merged wait(`futex_waitv`). 5.16 미만 커널은 parker 스레드 fallback
- executor 단일 구현(core), priority, 같은 priority 교대, pass 예산
- `on_thread_start` hook
- GPU 경로 셋(`ShmDirect`, `ShmRegistered`, `DeviceHandle`)과 두 seam 동기 fence. 아래 "GPU" 절
- Python numpy view, bf16 `.bits`. GPU 채널은 `Frame`과 `Loan`이 `__cuda_array_interface__`, `__dlpack__`를 낸다
- message_filters: 언어별 upstream policy + flux Subscriber 어댑터. 어댑터는 filter의 큐 크기를 받아 `max_borrow`를 큐 + 1 이상으로 올린다. filter는 새 프레임을 넣은 뒤 오래된 것을 지우므로, 그보다 작으면 새 프레임을 못 받아 입력이 멈춘다. 발행자의 슬롯이 큐 + 1보다 적으면 `INVALID_ARGUMENT`로 거절한다(executor 안에서 붙을 때면 아래 규칙대로 그 입력만 빠진다). 큐가 슬롯을 모두 쥐면 발행자가 다음 프레임을 쓰지 못하기 때문이다. 큐에 넣는 header는 stamp와 `frame_id`를 담는다(adapter의 `header__frame_id()`). 음수 stamp는 두 언어 모두 `unreadable`로 세고 넘기지 않는다. Python은 `std_msgs/Header`인 `header`가 없는 adapter를 생성 시점에 거절한다
- `flux_model_checking` 하니스(알고리즘 모델), fuzz(wire, 손상 세그먼트). C++ 층은 `-DFLUX2_SANITIZER=thread`(또는 `address`)로 따로 빌드해 시험한다. 이 빌드는 Rust core를 계측하지 않으므로 core의 atomic이 순서를 정하는 경쟁은 잡지 않는다. 그쪽은 Miri와 `scripts/sanitize.sh`(아래 "검증")가 맡는다. Jazzy에서 TSan 보고는 모두 CycloneDDS(`libddsc`, 계측 안 됨) 안에서 나온다
- 기존 gtest 시나리오를 C++ 층 적합성 테스트로 이식
- `flux` CLI(열거)
- docker 지원(아직 없다. Docker 기본 seccomp와 `--ipc` private·host에서 도는 것은 확인했다)
- 슬롯 발행 시각 `publish_ns`(u64, `CLOCK_MONOTONIC`). 발행자가 claim한 동안 쓰고 구독자는 borrow한 뒤 읽는다

## executor

대기는 `futex_waitv` 한 번이다. 붙어 있는 채널의 wake word(공유 futex)와 제어 word(프로세스 내부 futex)를 pass 시작 때 샘플한 값으로 넘긴다. 샘플 뒤 프레임이 오면 즉시 돌아온다. stop·interrupt·wake와 ROS hook은 제어 word를 올리고 깨운다.

`futex_waitv`는 Linux 5.16부터 있다. 그 아래(`ENOSYS`: Ubuntu 22.04 GA 커널 5.15, Jetson JetPack 6의 L4T 36 커널 5.15)에서는 `Executor::new`가 parker 스레드 fallback을 고른다. flux의 io_uring fallback과 같은 구조다. seccomp가 막은 곳(`EPERM`)에서는 fallback으로 가지 않고 `Unsupported`(C ABI `FLUX2_UNSUPPORTED`)로 거절한다. fallback은 호출이 없는 커널을 위한 것이고, 막힌 호출은 profile에서 허용하는 것이 맞기 때문이다. 단위 테스트는 seccomp filter로 두 errno를 만들어 각 경로를 확인한다. 환경 변수 `FLUX2_DISABLE_FUTEX_WAITV`가 있으면 5.16 이상에서도 fallback을 고른다. 테스트는 이것으로 두 경로를 모두 돈다(Rust `tests/executor.rs`는 자기 자신을 한 번 더 실행하고, C ABI `executor.c`, C++ `test_executor`·`test_ros`·`test_partitioned`, Python `test_executor`·`test_partitioned`는 `_parked` 이름으로 한 번 더 돈다).

```text
merged:   executor ── futex_waitv([제어 word, wake word...,  watch word...])
fallback: parker(word) ── FUTEX_WAIT(word, seen) ── 값이 바뀜 ── 제어 word를 올리고 깨움
          executor ── FUTEX_WAIT(제어 word, 마지막으로 들은 값)
```

- parker는 source마다 하나다. `add`가 만들고 executor가 사라질 때 멈춘다. 기다리는 word는 merged wait가 넘겼을 word다(붙은 채널은 wake word, 붙지 않은 source는 watch word). 그런 word가 없는 동안은 자기 겨냥 word에서 잔다.
- word가 바뀌면(attach, epoch 변경, 발행자 재시작) `wait_for_work`는 스레드를 띄우거나 멈추지 않는다. 그 parker의 겨냥(word, 시작 값)만 바꾸고, parker가 기다리던 word를 futex wake한다. executor 스레드는 cyra가 SCHED_FIFO로 올리는 RT 스레드이고, 스레드를 만들면 스택을 mmap하고(mmap_lock 쓰기, 할당) 끝내면 munmap하기 때문이다. 스레드 생성과 join은 `add`와 executor drop에만 있다. 예외는 fork한 자식이다. 부모의 parker 스레드는 자식에 없으므로 자식의 첫 대기가 새로 띄우고, 자식의 drop은 그 스레드를 join하지 않는다.
- 겨냥은 executor 하나가 쓰는 seqlock이다. 바뀐 word의 mapping은 parker가 그 뒤의 겨냥을 읽었다고 표시할 때까지 executor가 쥔다. 그 전에는 parker가 옛 word를 아직 만질 수 있기 때문이다.
- parker는 겨냥이 바뀔 때마다 merged wait가 넘겼을 값(pass가 샘플한 seq, watch의 armed epoch)에서 시작한다. 그래서 샘플 뒤에 온 프레임은 곧바로 제어 word를 울린다. executor는 제어 word를 마지막으로 들은 값으로 기다리므로, 그 뒤에 울린 것은 대기를 바로 끝낸다. 잃는 wake가 없다.
- waiter gate, priority, pass 예산, stop·interrupt·tick, source 127개 상한, trace 이벤트(`flux2_trace_wake`의 위치와 값)는 두 경로가 같다. C ABI 위의 층은 어느 경로인지 모른다.
- parker를 멈추거나 겨냥을 바꿀 때는 표시를 한 뒤 그 word를 futex wake한다. 같은 word를 기다리는 다른 대기는 이를 stray wake로 보고 다시 확인한다. 표시와 parker의 대기 사이에 끼면 parker의 대기 상한(50 ms) 안에 본다. 겨냥을 바꿀 때 이 틈에 끼면 새 word의 프레임은 잃지 않지만 최대 50 ms 늦는다. 옛 word의 futex 대기는 겨냥 word를 함께 비교할 수 없어서 이 틈을 닫지 못한다. 겨냥 word에서 자는 parker에는 겨냥의 틈이 없다. stop의 틈은 어느 word에서든 있다. stop 표시는 futex가 비교하는 word가 아니기 때문이다. 그래서 executor drop은 최대 50 ms 늦는다. 두 틈 모두 의도한 예외이고, flux_model_checking `wake`의 parked 설정은 이 틈에서만 timeout을 허용한다.
- parker, GPU accept, `flux2-reaper`(아래 "정리 순서") 스레드는 모든 신호를 막은 채 시작한다. 프로세스로 보낸 신호는 막지 않은 아무 스레드로 가므로, 그렇지 않으면 rclcpp가 건 SIGINT 처리기가 이 스레드에서 돌고 그 대기가 EINTR로 끝난다
- 신호는 대기를 끝내지 않는다. `SA_RESTART` 없이 건 처리기는 `FUTEX_WAIT`, `futex_waitv`, `poll`을 EINTR로 끝내고, `SA_RESTART`도 timeout 있는 `FUTEX_WAIT`와 `poll`은 다시 시작하지 않는다. 그래서 futex 대기 둘과 fd 교환의 `poll`은 EINTR이면 남은 deadline으로 다시 부른다(`futex_waitv`는 deadline이 절대 시각이라 같은 값으로 다시 부른다). `fallocate`는 1 MiB 단위로 다시 부른다. 대기가 신호로 일찍 끝나지 않으므로 trace `WAKE_SIGNAL`은 더 이상 나오지 않는다(계약 번호는 그대로 둔다). 신호 처리기에서 spin을 멈추려면 `stop`·`cancel`을 부른다. futex wake만 하므로 처리기에서 불러도 된다. `close()`는 EINTR이어도 다시 부르지 않는다(Linux는 이미 fd를 풀었다). non-blocking `connect`·`recvmsg`와 helper 스레드(신호를 막았다)의 `poll`·`accept4`는 EINTR을 받지 않거나 루프가 이미 다시 부른다
  - 시험 `storm`은 처리기를 `SA_RESTART` 없이 건 SIGUSR1(대상 스레드에 200 us마다)과 `setitimer` SIGALRM 아래에서 생성, `Channel::wait`, `PublisherWatch::wait`, `wait_for_work`(merged와 parker 각각), 느린 발행자와의 fd 교환을 돈다. 타이머가 프로세스 전체에 걸리므로 시험 바이너리를 자식으로 다시 띄워 그 시험만 돌린다. 고치기 전에는 대기가 수십 us 만에 끝나고 fd 교환이 `NotReady`였다
- 대가: source마다 스레드 하나(아직 붙지 않은 source도), wake마다 문맥 전환 한 번이 더 든다. 아래 표는 이 대가를 잰 것이다. fallback의 겨냥, 재겨냥, retired word, 두 틈은 flux_model_checking `wake`의 parked 설정이 다룬다.

publish에서 콜백까지의 지연이다(Jetson AGX Orin, Linux 6.8 RT, SCHED_OTHER, 채널을 돌아가며 1 ms마다 한 프레임, 5000개). 두 번 잰 p99는 3 us 안에서 같았고, max는 두 경로 모두 실행마다 0.05~1.9 ms로 흔들려 싣지 않는다.

| 채널 | merged p50 | merged p99 | merged p99.9 | fallback p50 | fallback p99 | fallback p99.9 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 4.5 us | 9.1 us | 12.3 us | 8.7 us | 19.0 us | 26.9 us |
| 8 | 5.1 us | 11.4 us | 17.0 us | 13.1 us | 23.4 us | 37.8 us |
| 32 | 7.7 us | 16.7 us | 25.6 us | 15.2 us | 25.1 us | 37.7 us |
| 127 | 15.8 us | 32.3 us | 54.2 us | 16.8 us | 30.2 us | 45.3 us |

채널이 적을 때 fallback은 문맥 전환 한 번만큼(p50 4~8 us, p99 8~12 us) 느리다. 127개에서는 merged wait가 호출마다 word 128개를 등록하는 비용이 커서 둘이 비슷하다.

flux는 io_uring(`FUTEX_WAIT` + eventfd `POLL_ADD`)을 썼다. ROS 알림과 제어 신호를 eventfd로 받아서 futex와 fd를 함께 기다려야 했기 때문이다. 둘 다 같은 프로세스 안의 일이라 futex로 받을 수 있고, 그러면 기다릴 것이 모두 futex라 `futex_waitv`로 충분하다. ROS hook이 futex를 깨우는 것은 Jazzy·lyrical, Fast DDS·CycloneDDS·Zenoh, 같은 프로세스·다른 프로세스 publisher에서 확인했다. 비교는 다음과 같다(Ryzen 5 9600X, Linux 7.0, 대기 계층만 측정).

| 채널 | io_uring 지연 중앙값 | `futex_waitv` 지연 중앙값 | io_uring CPU/wake | `futex_waitv` CPU/wake |
| --- | --- | --- | --- | --- |
| 1 | 2.27 us | 2.02 us | 1.23 us | 0.97 us |
| 8 | 2.25 us | 2.14 us | 1.24 us | 1.60 us |
| 32 | 2.24 us | 2.46 us | 1.24 us | 3.70 us |
| 127 | 2.24 us | 3.86 us | 1.24 us | 11.92 us |

- 대가: 호출마다 모든 word를 등록하므로 비용이 채널 수에 비례하고, 한 executor는 source 127개까지다(커널 한도 128, 제어 word 1개). 128번째 `add`는 `InvalidArgument`다. 더 많으면 `PartitionedExecutor`로 나눈다.
- 얻는 것: Docker 기본 seccomp가 막는 io_uring을 쓰지 않고, 추가 스레드가 없고, 걸어 둔 대기의 cancel·누수 추적이 사라진다.

`PartitionedExecutor`도 core에 둔다. 파티션마다 스레드 하나를 쓰고, 파티션은 `Executor`이거나 외부 루프(`run`, `stop`)다. 호출 스레드는 callback을 돌리지 않고 기다리기만 하며, stop과 interrupt를 자식에게 전달한다. 어느 파티션이든 오류가 나면 전체를 멈추고 그 첫 오류를 돌려준다.

붙기가 `NOT_READY` 말고 다른 이유로 거절된 구독(구독자 자리 부족, fd·메모리 부족, 다른 사용자의 파일, 깨진 파일, message_filters 슬롯 부족)은 이유를 로그로 한 번 남기고 executor에서 빠진다. 나머지 source는 계속 돈다. 발행자가 재시작하면(signpost의 epoch가 바뀌면) 다시 시도한다. iceoryx2가 발행자 목록이 바뀔 때 연결을 다시 맺는 것과 같다. 빠진 구독은 watch(아래)의 epoch를 load 하나로 본다. Subscription은 announce로 signpost를 쥐고 있어 발행자가 내려가도 epoch가 이어진다.

발행자보다 먼저 만든 구독은 signpost의 epoch 워드를 watch로 쥔다. 발행자는 새 epoch를 쓴 뒤 그 워드에 futex wake를 하고, executor는 붙지 않은 source의 watch 워드를 `futex_waitv`에 넣는다(`Source::watch`). `take_blocking`도 붙기 전에는 이 워드에서 기다린다. 그래서 발행자가 뜨자마자 붙는다. 처음 읽는 위치는 그대로다. 발행자 재시작은 아직 다음 tick이나 take에서야 알아챈다. executor가 옛 세그먼트의 wake 워드에 묶여 있기 때문이고, source마다 워드를 하나 더 두면 source 한도가 반으로 준다. 구독을 만드는 자리에서 난 거절은 호출한 쪽에 예외로 간다. bridge와 bag 녹화도 그 채널만 빼고, 발행자가 재시작하면 다시 시도한다. bag 재생은 rosbag2의 player처럼 발행자를 만들지 못한 채널을 무시하고 나머지를 재생한다. 사용자 콜백의 예외는 이 규칙에 들지 않고 spin 밖으로 나간다.

`max_borrow`에 막혀 take가 거절된 source는 view가 풀릴 때 깬다. executor가 채널에 깨울 곳을 걸어 두고, borrow를 돌려주는 쪽이 그것을 보면 제어 futex를 깨운다. 둘 다 SeqCst라 한쪽은 반드시 다른 쪽을 본다. 전에는 다음 tick(100 ms)까지 기다렸다.

`spin`이 `&mut self`를 받으므로 재진입과 spin 중 `add`는 컴파일 시점에 막힌다. 다른 스레드에서 stop·interrupt·wake를 하려면 `ExecutorHandle`을 쓴다.

층별 배분:

| 층 | 맡는 일 |
| --- | --- |
| core | `Executor`(대기, priority, 같은 priority 교대, pass 예산), `PartitionedExecutor`(스레드, `on_thread_start`, stop·interrupt 전파, 오류 전파), `Source` trait |
| C ABI | 핸들 생성·해제, 함수 포인터 + ctx로 받는 `Source`·hook, 반환 코드로 전달하는 callback 실패 |
| C++ | ROS 전용. 공개는 `Publisher`, `Subscription`, `Executor`, `FrameView`, `WriteSlot`, `QoS`이고, C ABI RAII 래퍼는 `flux2::detail`에 둔다. `PartitionedExecutor`는 callback group마다 스레드 하나와 `flux2::Executor` 하나를 둔다. flux 구독이 있든 ROS만 있든 같은 경로라 stop이 tick을 기다리지 않는다. message_filters 입력은 `flux2::message_filters::Subscriber<Adapter>`이고, `add_sync_group`은 한 synchronizer의 입력이 다른 group에 흩어지면 spin을 거절한다. `add()`나 `on_thread_start()`에 넘긴 group은 SharedPtr로 붙잡고, 그 밖의 group은 사라지면 그 스레드를 끝낸다. `cancel()`은 lock을 잡지 않고 futex wake만 하므로 신호 처리기에서 불러도 된다 |
| Python | ROS 전용. nanobind 모듈이 flux2_cpp의 `flux2::detail` 층(C ABI RAII 래퍼, `Subscriber`)을 그대로 쓴다. 공개는 `Publisher`, `Subscription`, `Executor`, `Loan`, `QoS`이고 프레임은 읽기 전용 numpy view다. bf16은 `uint16` 비트로 읽는다. 거절은 C++과 같은 값으로 돌려준다. 실패한 commit은 loan을 끝내고 `TOO_LARGE`를 돌려주며(iceoryx2, zenoh, agnocast도 실패한 송신이 loan을 소모한다), 8차원을 넘는 publish도 `TOO_LARGE`, 지원하지 않는 dtype은 `TypeError`다. 생성된 Builder의 setter가 예외를 내면 writer가 잠겨 commit이 거절된다. 정수 필드는 범위를 넘으면 numpy 버전과 상관없이 `OverflowError`, 문자열 필드는 rclpy처럼 `str`만 받는다. 콜백은 GIL을 잡고 돈다. flux 구독만 도는 루프(ROS 노드 없는 `Executor`, `PartitionedExecutor`의 flux 스레드)는 core의 `spin`을 GIL 없이 돌리고 콜백과 tick마다 GIL을 잡는다(`PyEval_SaveThread`·`RestoreThread`). tick에서 Ctrl-C와 구독 노드의 context 종료를 본다. rclpy의 SIGTERM 처리는 Python `on_shutdown`을 부르지 않아서, 알림을 기다리지 않고 tick마다 확인한다. SIGTERM이나 `rclpy.shutdown()`으로 context가 끝나면 spin은 정상 반환한다. rclpy가 이때 wait set을 만들다 내는 `RCLError`는 종료로 본다. 종료 때는 `atexit`에서 아직 도는 spin을 멈추고 1초까지 기다린 뒤, GIL을 다시 잡으러 가는 중인 스레드를 모두 지나가게 하고 그 뒤에 오는 스레드는 세워 둔다. Python 3.14 미만은 finalization 중에 GIL을 다시 잡는 스레드를 `pthread_exit`로 끝내는데, 이것이 `noexcept` 프레임을 지나며 abort하기 때문이다(Jazzy, Python 3.12). numpy 배열과 scalar는 numpy C API로 만든다. `nb::ndarray`는 C++에서 `numpy.array()`를 불러 넘겨서 배열 하나에 약 500 ns가 들고, C API로는 약 50 ns다. 그래서 빌드에 numpy 헤더가 필요하고 numpy 1.x(Jazzy)와 2.x(lyrical)를 각 환경에서 빌드해 시험한다. rclpy executor는 group 단위로 엔티티를 받지 못하므로 `PartitionedExecutor`는 flux 구독이 있는 group마다 flux 전용 스레드, 노드마다 rclpy 스레드를 둔다. 그래서 flux group에 ROS 엔티티가 있으면 spin을 거절하고, 한 synchronizer에 flux와 ROS 입력이 섞이면 `Executor`를 쓰게 한다. 노드 스레드는 `spin_once`마다, flux 스레드가 GIL을 기다리는 동안 GIL을 내준다(`_step_aside`). CPython은 5 ms 동안 전환이 없어야 GIL을 넘기게 하는데, 바쁜 rclpy 스레드는 그보다 자주 GIL을 놓았다 다시 잡아 flux 스레드가 굶었다(300개 중 8개) |

ROS 2 executor 통합은 C++ flux와 같다.

- C++: `flux2::Executor`가 `rclcpp::Executor`를 상속해 대체한다. 한 pass는 `wait_for_work` → `dispatch` → rclcpp 작업 순서이고, ROS 엔티티의 준비 hook이 `wake`를 부른다. ROS timer는 hook이 없으므로 가장 가까운 deadline으로 대기를 줄인다. ROS time이 켜진 timer는 `rcl_wait`처럼 그 deadline에서 빼고, clock jump callback이 due가 되거나 시간원이 바뀔 때 executor를 깨운다.
- hook을 걸 엔티티는 tick마다 rclcpp의 callback group 목록(`get_all_callback_groups`)을 다시 읽어 정한다. `add_node` 같은 등록 함수는 override하지 않는다. 그 시그니처가 rclcpp 버전마다 달라서다(Jazzy는 값, 이후는 const 참조).
- `spin()`과 `spin_until_future_complete()`가 같은 루프를 돈다. 후자는 `spin_until_future_complete_impl`을 직접 구현한 것이라 `spin_once_impl`을 거치지 않는다. timeout은 pass를 한 번 돈 뒤에 본다. rclcpp처럼 timeout 0도 준비된 일을 처리한다. `spin_once`·`spin_some`·`spin_all`은 예외를 던진다. `spin_some`·`spin_all`은 rclcpp wait set 기준 예산이라 flux를 건너뛰고, `spin_once`는 쓰지 않기로 했다.
- Python: rclpy executor를 그대로 쓴다. 노드가 있거나 rclpy executor를 넘기면 이 방식이다. 스레드는 셋이다.
  - bridge 스레드: GIL 없이 프레임을 기다린다.
  - post 스레드: GIL을 잡고 pass 하나를 담은 rclpy task만 낸다. task는 세대 번호를 들고 있어, spin이 끝난 뒤 남은 task는 아무것도 하지 않는다.
  - spin 스레드: 그 task를 돌고, rclpy callback 사이에 아직 아무 task도 가져가지 않은 pass를 직접 돈다(`dispatch_pending`). ROS 쪽이 바빠 post 스레드가 GIL을 얻지 못해도 전달이 멈추지 않는다(300개 중 16개에서 300개).
  - core에는 한 번에 한 스레드만 들어간다(mutex 하나로 claim).
  - `stop()`은 rclpy를 건드리지 않고 post 스레드가 깨우므로 신호 처리기에서 불러도 된다.
- 한쪽이 밀려도 다른 쪽이 굶지 않는다. 실노드 시험에서 둘 다 굶었다(아래 표).
  - C++: flux `dispatch`는 ROS 작업이 준비되면 다음 callback 전에 멈추는데, rclcpp 작업은 flux를 보지 않고 한 pass에 64개까지 돌았다. 이제 executable 하나를 돌릴 때마다 `ready()`(syscall 없이 wake word와 샘플을 비교)를 보고, flux에 일이 있고 1 ms(`kTurnNs`)가 지났으면 넘긴다. 1 ms를 두는 이유는 반대쪽이다. executable 하나마다 넘기면 50 ms짜리 flux callback 사이에 가벼운 ROS callback이 하나씩만 돌아 100 Hz Imu가 밀렸다(p50 314 ms).
  - Python: rclpy는 준비된 task가 있으면 wait set을 보지 않고 task부터 돈다. 프레임이 계속 오면 `dispatch` task가 매번 준비돼 있어 ROS callback이 돌지 못했다. 이제 `dispatch`는 1 ms(`_TURN_NS`)가 지나면 다음 callback 전에 멈추고, bridge는 `has_more`면 기다리지 않고 바로 다음 task를 낸다. 기다리면 끊긴 pass에 남은 구독이 다음 프레임까지 밀려 그 프레임에 덮였다. guard condition으로 wait set에 넣는 방법도 시험했지만 EventsExecutor가 guard condition을 받지 않고, 시간 예산만으로 세 rclpy executor 모두 굶지 않았다.
  - rclpy `MultiThreadedExecutor`는 바쁜 구독 옆에서 다른 스레드가 낸 task를 늦게(최대 수백 ms) 돌리거나 놓친다. flux2 없이 rclpy만으로도 재현된다(`flux2_test_pkg/test/mte_plain.py`). `flux2.Executor`와는 Single 또는 Events executor를 쓰거나 `PartitionedExecutor`를 쓴다.

| 과부하 | 고치기 전 | 고친 뒤 |
| --- | --- | --- |
| C++, ROS callback 5 ms / 1 ms timer, flux 프레임 20개 | 1개 받음 | 18개 이상 |
| C++, flux callback 20 ms, 10 ms timer 3개, 1초 | tick 49 | 100 이상 |
| C++ 실노드, ROS Imu 100 Hz에 50 ms 작업, FHD 30 Hz | flux 157개 중 3개 | 171개 중 117개 |
| C++ 실노드, FHD에 50 ms 작업, ROS Imu 100 Hz | (`ready()`만 넣었을 때) ROS p50 314 ms | ROS 157개, 빠짐 0, p50 27 ms |
| Python, flux callback 5 ms / 2 ms 프레임, ROS timer 100회 | Single·Events 3회 | 50회 이상 |
| Python, 한 채널에 구독 3개, callback 0.8 ms | (`has_more` 없이) 30개 중 19개 | 30개 |
| Python 실노드, FHD에 50 ms 작업, ROS Imu 100 Hz | ROS 196개 중 3개 | 163개 중 162개 |

예외는 C ABI 경계를 넘지 않는다. `panic = "abort"`이므로 C++·Python 예외는 경계에서 잡아 반환 코드로 바꾼다.

## GPU

정본은 flux의 `flux_docs/design/gpu_platform.md`와 `gpu_copy.md`(D-040~D-052)다. 여기에는 flux2에서 달라진 것과 구현 위치만 적는다.

| 층 | 맡는 일 |
| --- | --- |
| core `gpu` | `libcuda.so.1`을 실행 시점에 연다(링크하지 않는다). 경로 판정 `probe`, `Stream`, 호스트 등록, VMM 할당·export·import, fd를 넘기는 소켓, 두 seam의 fence와 통계 |
| C ABI | `flux2_stream`(NULL은 호스트 채널), `flux2_gpu_probe`, `*_device_ptr`, `*_stream`, `flux2_channel_fence_failed`·`fence_wait`, `FLUX2_PUBLISHED_WRONG_DEVICE`·`FENCE_FAILED`. ABI 버전 2에서 들어왔다. `flux2_channel_set_fence_timeout`, `flux2_channel_set_release_mode`, `flux2_view_after`, `flux2_gpu_wedged`는 뒤에 더한 함수이고, `FLUX2_PUBLISHED_GPU_WEDGED`는 뒤에 더한 값이다. 구조체 배치가 그대로라 ABI 버전은 올리지 않았고, 이 함수들을 쓰는 바이너리는 옛 라이브러리에서 symbol을 못 찾아 로드 때 실패한다 |
| C++ | `Device`, `gpu::Stream`, Publisher·Subscription의 `Device`·`gpu::Stream` 생성자, `FrameView`·`WriteSlot`의 `device_ptr()`·`stream()` |
| Python | `device="cuda"`, `Frame`(`with` 스코프나 callback 안에서만 메모리를 낸다), `Loan`·`Frame`의 `__cuda_array_interface__`·`__dlpack__`, `probe()` |

- 경로는 장치마다 위에서부터 처음 맞는 줄이다. `integrated && host_register`이면 `ShmRegistered`(매핑을 `cuMemHostRegister`), `integrated && pageable`이면 `ShmDirect`(매핑 포인터 그대로), `vmm && posix_fd`이면 `DeviceHandle`(payload 전체를 VMM 할당 하나로), 그 밖은 이유를 붙인 `None`이다. stream을 선언한 채널은 `None`에서 생성이 거절된다
- flux와 순서가 다르다. flux는 `ShmDirect`를 먼저 고른다. Thor(`pageable = 1`)에서 등록하지 않은 슬롯은 CUDA runtime이 pageable host 메모리로 보아, 슬롯으로 가는 `cudaMemcpyAsync`가 동기 pageable 경로를 탄다. 30 Hz 9.2 MB 복사에서 p50은 약 240 μs로 같지만 20%가 500 μs를 넘고 p99가 5 ms였다. 등록하면 같은 복사의 p99가 255 μs다(2026-10-14)
- 구독자의 `PROT_READ` 매핑은 장치가 `READ_ONLY_HOST_REGISTER_SUPPORTED`일 때만 `READ_ONLY`로 등록한다. Orin은 이 플래그 없이는 거절하고, Thor는 이 플래그를 지원하지 않아 플래그 없이 등록한다
- 등록과 import는 세그먼트 매핑마다 한 번이다. 한 프로세스의 구독들이 매핑을 같이 쓰고, 드라이버는 같은 메모리의 두 번째 등록을 거절하기 때문이다
- 두 seam은 동기로 닫는다. commit은 stream을 기다린 뒤 발행하고, view 해제는 기다린 뒤 borrow를 돌려준다. 기다림은 blocking-sync event다(`cuEventRecord` + `cuEventSynchronize`). `cuStreamSynchronize`는 context 설정에 따라 기다리는 동안 코어를 돌린다
- fence가 실패하면 그 슬롯을 놓지 않는다. commit 쪽은 claim을 쥔 채 `FenceFailed`이고, 해제 쪽은 borrow를 쥔 채 남는다. 샌 borrow가 `max_borrow`를 채우면 그 뒤 take는 `refused.fence`로 센다(trace 사유 `REFUSED_FENCE`)
- 연산 하나의 오류와 sticky 오류를 나눈다. cuda.h가 "프로세스를 다시 띄워야만 풀린다"고 적은 오류(226, 700, 702, 710, 714~719, 721, 810, 911)를 어느 드라이버 호출에서든 처음 받으면, 프로세스의 GPU 상태를 그 오류로 한 번 wedged로 표시한다. 드라이버는 그 뒤 모든 호출에 같은 오류를 돌려준다
  - 그 뒤 모든 GPU 채널은 드라이버를 부르기 전에 같은 사유로 바로 거절한다. take는 `refused.fence`(trace 사유 `REFUSED_GPU_WEDGED`), loan은 빈 결과(`CLAIM_GPU_WEDGED`), commit과 publish는 `GpuWedged`(`LOAN_GPU_WEDGED`, `CLAIM_GPU_WEDGED`), `take_blocking`은 기다리지 않고 빈 결과다. 새 stream과 GPU 채널은 오류 번호를 밝힌 `Unsupported`다. 전에는 해제마다 borrow가 새어 `max_borrow`를 채운 뒤에야 거절이 드러났고, loan은 commit마다 슬롯을 하나씩 잃었다
  - 오류 번호는 core `gpu::wedged()`, C ABI `flux2_gpu_wedged()`, C++ `gpu::wedged()`, Python `flux2.gpu_wedged()`로 읽고, 표시하는 순간 trace `gpu_wedged` 이벤트가 한 번 나간다. supervisor는 이것을 보고 프로세스를 다시 띄운다. 호스트 채널은 영향을 받지 않는다
  - 검사는 GPU 채널에서만 하는 atomic load 하나이고, 오류 분류는 실패한 호출에서만 돈다
- 실패한 fence는 채널도 프로세스가 끝날 때까지 쥔다. 채널이 사라지면 매핑이 풀리고 구독자 번호가 풀려, 다음 구독자가 그 번호의 칸을 0으로 지우기 때문이다. 프로세스가 죽으면 그 kernel도 같이 끝나므로 그때 회수된다
- fence 대기에는 채널마다 상한을 둘 수 있다. core `Channel::set_fence_timeout`, C ABI `flux2_channel_set_fence_timeout`, C++ `set_fence_timeout`, Python `fence_timeout`(초)이고, 기본은 상한 없음이다. GPU가 멈추면 상한 없는 대기는 executor 스레드도 멈추게 하기 때문이다
  - `cuEventSynchronize`에는 상한을 줄 수 없다. 그래서 상한이 있으면 그 작업만 표시하는 event를 기록해 stream의 대기 스레드(아래)에 넘기고, 호출한 스레드는 그 event의 상태 word를 상한을 두고 futex로 기다린다. 대기 스레드가 device의 신호로 깨어 word를 올리므로 끝난 뒤 곧 돌아온다. 처음에는 `cuEventQuery`를 늘어나는 간격(최대 1 ms)으로 물었는데, 끝난 뒤에도 다음 질문까지 늦게 알았다. 상한이 없으면 지금처럼 blocking-sync event로 기다린다
  - 상한을 넘긴 대기는 fence 실패와 같다. `fence_failed`로 세고, commit은 `FenceFailed`이고, 슬롯은 쥔 채 둔다. 그 작업만 표시하는 event를 따로 기록해 두고, 그 채널의 다음 take나 claim이 끝났는지 다시 묻는다. 끝났으면 해제 쪽은 borrow를 돌려주고, commit 쪽은 발행하지 않은 claim을 놓는다. 커널도 멈춘 job이 쓰던 메모리는 리셋 전까지 풀지 않는다
  - 재attach하거나 채널이 사라질 때 남은 늦은 작업은 아래의 orphan 규칙을 따른다
  - Python이 consumer stream(`__dlpack__(stream=)`)을 기다리는 대기에는 상한이 없다
- 구독은 해제 방식을 고른다. core `Channel::set_release_mode(ReleaseMode::Deferred)`, C ABI `flux2_channel_set_release_mode(FLUX2_RELEASE_DEFERRED)`, C++ `set_release_mode(gpu::ReleaseMode::Deferred)`, Python `release="deferred"`이고, 기본은 `Wait`(위의 동기 해제)다. C++·Python 구독은 재attach한 채널에도 다시 건다
  - `Wait`는 해제가 stream을 기다린 뒤 borrow를 돌려준다. callback이 결과를 꺼내며(`.cpu()` 등) 이미 GPU를 기다렸다면 이 대기는 거의 없다
  - `Deferred`는 해제가 그 순간까지의 작업을 표시하는 event를 기록하고 곧바로 돌아간다. callback이 GPU 작업을 걸어 둔 채 끝나는 노드(consumer 버퍼로 비동기 복사, 파이프라인)에서 executor 스레드가 그 GPU 시간 동안 막히지 않는다
  - borrow는 device가 그 슬롯을 다 쓴 뒤에만 돌아간다. 돌려주는 자리는 채널 자기 스레드의 다음 take, `can_borrow`, executor pass다. 공유메모리를 만지는 스레드를 늘리지 않기 위해서다(D-044)
  - 그때까지 borrow는 `max_borrow`에 든다. 그 작업이 실패하면 실패한 fence처럼 borrow를 쥔 채 `fence_failed`로 센다. 해제가 기다리지 않으므로 `fence_wait`의 release 값은 늘지 않는다. trace `release` 이벤트는 해제 시점에 `held` 0으로 남는다
  - 표시 event를 기록하지 못하면(forked child 등) 그 해제는 `Wait`로 한다. commit은 두 방식 모두 기다린다
  - 늦은 작업은 채널을 쥔다. 채널이 사라지거나 재attach로 바뀌면, 이미 끝난 것은 그 자리에서 돌려주고 남은 것은 그 작업의 waiter 스레드에 넘긴다(orphan). waiter는 작업이 끝나면 그 항목을 놓기만 하고(마지막 unmap은 `flux2-reaper`), 실패하면 실패한 fence처럼 계속 쥔다. waiter는 공유메모리 카운터를 만지지 않는다(D-044). 그래서 그 borrow는 프로세스의 구독자 번호가 풀린 뒤 다음에 그 번호를 받는 쪽이 지울 때까지 남는다. 죽은 구독자와 같다. 채널이 사라진 뒤 view가 늦게 해제해도 같은 길로 간다. 처음에는 이 경우 채널과 import가 프로세스가 끝날 때까지 남았다(compute-sanitizer가 C++ `test_gpu`에서 2 MiB로 보고). 시험은 `a_deferred_release_outliving_its_subscription_lets_the_mapping_go_once_done`이다
  - Python `Frame`은 `__dlpack__(stream=s)`로 받은 consumer stream을 기다리지 않고, 채널 stream이 device에서 s 뒤에 오게 한다(`cuStreamWaitEvent`, C ABI `flux2_view_after`). 그 뒤의 표시 event가 s의 작업까지 덮는다
- 대기 스레드 `flux2-gpu-wait`는 상한이나 `Deferred`를 쓰는 stream마다 하나다. 그 설정을 하는 호출이 만들므로 executor 스레드에서 생기지 않는다
  - 표시 event를 들어온 순서대로 `cuEventSynchronize`(blocking-sync)로 기다린다. 한 stream의 event는 순서대로 끝나므로, stream마다 두면 다른 채널의 긴 kernel 뒤에서 기다리지 않는다. 프로세스에 하나만 두면 그 kernel이 다른 채널의 상한까지 넘기게 한다
  - 끝나면 event의 상태 word를 올려 futex로 깨우고, 그 채널을 맡은 executor의 제어 word를 울린다. executor는 원래 제어 word를 함께 기다리므로(merged wait와 parker 경로 둘 다) 새 프레임이 없어도 깨어 borrow를 돌려준다. 채널은 executor pass마다 자기 executor를 등록한다
  - 이 스레드는 공유메모리를 만지지 않는다. 채널이 사라져도 남은 event를 다 기다린 뒤 끝나고, join하지 않는다. 멈춘 kernel이 join을 영영 막기 때문이다
  - 깨움 경로 위의 스레드다. RT 체인에서는 cyra로 이 이름을 선언한다. 모든 신호를 막은 채 시작한다
- blocking-sync event에서 자는 스레드(`Wait` 해제·commit의 호출 스레드, `flux2-gpu-wait`)를 깨우는 것은 GPU 드라이버의 IRQ 처리다. 그래서 fence의 깨움 경로에는 flux2 밖의 스레드가 하나 더 있다
  - PREEMPT_RT에서는 IRQ 처리가 스레드(`irq/<n>-nvidia`, Orin은 GPU 드라이버 이름)로 돌고 기본 우선순위가 `SCHED_FIFO` 50이다. 같은 CPU에서 그보다 높은 stage가 돌면 GPU가 끝났어도 깨움이 그 stage 뒤로 밀린다
  - cyra 체인에서는 이 IRQ 스레드를 `match: {irq: nvidia}`(Orin은 GPU 드라이버의 handler 이름) stage로 선언하고, fence를 기다리는 stage보다 낮지 않게 둔다. cyra 데몬 `cyrad`가 그 스레드를 찾아 건다
  - fence 대기가 길게 재지면 GPU 시간과 함께 이 IRQ 스레드가 언제 깨어 돌았는지를 본다
  - 돌려주는 쪽은 깨어 있는 김에 event를 한 번 묻기도 한다(`cuEventQuery`). device가 끝났는데 대기 스레드가 아직 word를 올리지 않은 순간을 놓치지 않기 위해서다. 기다리는 수단으로 묻지는 않는다
- legacy default stream(NULL)은 `Stream::from_native`가 거절한다. 스레드마다 다른 stream을 가리키기 때문이다
- `DeviceHandle` 세그먼트는 control plane만 shm에 두고(`StorageKind::CudaIpc`), 헤더에 할당한 장치의 UUID를 적는다. 구독자는 ordinal이 아니라 UUID로 자기 장치를 찾는다. `CUDA_VISIBLE_DEVICES`가 다르면 ordinal이 달라지기 때문이다. 구독자 stream의 장치가 그 장치와 다르면 attach에서 `Mismatch`다. import는 할당한 장치에만 READ 권한을 주므로 다른 GPU의 kernel이 읽으면 illegal address로 context가 망가진다
- VMM 할당과 import, 호스트 등록은 그 장치의 primary context에서 만들고 해제한다. primary context는 retain한 채 놓지 않는다. 호출자가 current로 둔 자기 context를 먼저 파괴해도 해제가 그 context를 쓰지 않게 하기 위해서다. 호스트 등록은 그 context와 함께 사라지기도 해서 `PORTABLE`로 등록한다. 그래야 그 장치의 다른 context에 있는 stream도 등록한 메모리를 pinned로 본다
- fd는 채널마다 accept 스레드 하나가 소켓 파일 `/dev/shm/flux2.gpu.<dev>.<ino>`(0600)로 `SCM_RIGHTS`에 실어 넘긴다. 데이터와 같은 `/dev/shm`에 두므로 세그먼트가 보이는 곳에서는 endpoint도 보인다. 처음에 쓴 abstract 이름은 network namespace를 따라서, `--ipc=host`이면서 network가 다른 컨테이너에서는 연결되지 않았고 `--net=host` 컨테이너에는 그대로 노출됐다
  - 발행자는 자기 pid를 붙인 이름에 bind하고 listen한 뒤 rename으로 제자리에 옮긴다. 그래서 그 이름은 듣고 있는 소켓만 가리키고, 같은 inode를 다시 받은 세그먼트는 죽은 발행자가 남긴 파일을 덮어쓴다
  - 발행자는 끝날 때 파일이 아직 자기 것(inode)이면 지운다. sweep은 듣는 쪽이 없는 endpoint와 10초 넘게 남은 bind 중간 이름을 지운다. 중간 이름의 pid로 판정하지 않는 것은 다른 pid namespace의 것일 수 있기 때문이다
  - 양쪽 모두 `SO_PEERCRED`로 상대가 이 euid인지도 확인한다. flux에는 이 확인이 없었다. 받는 쪽이 open file 한도에 닿아 커널이 fd를 버리면(`MSG_CTRUNC`) 그 사유로 `NotReady`다
- 구독자의 import 매핑은 읽기 전용(`cuMemSetAccess` READ)이다. export한 fd에는 `FD_CLOEXEC`를 붙인다. 드라이버가 지금 붙여 주지만 문서로 보장하지 않고, exec한 자식이 이 fd를 물려받으면 할당을 쓰기 가능한 채로 살려 두기 때문이다
- 받는 쪽은 non-blocking으로 connect하고 fd를 최대 1초 기다린다. 응답이 없으면 `NotReady`다. 멈춘 발행자(SIGSTOP, debugger)의 backlog가 차면 blocking connect는 돌아오지 않기 때문이다. backlog가 찬 뒤에는 기다리지 않고 바로 `NotReady`다
- `DeviceHandle` 채널은 GPU 전용이다. CPU 구독은 attach에서 `Mismatch`, `publish()`는 `WrongDevice`, loan의 host 포인터는 NULL이다. 호스트 경로를 가진 장치는 `CudaIpc` 세그먼트를 import하지 않는다
- fork한 자식은 물려받은 GPU 객체를 건드리지 않는다. stream·등록·VMM 해제와 fd 서버 join을 건너뛰고, fence는 하지 않는다
- 부모가 `cuInit`한 뒤 fork한 자식은 새 GPU 객체도 만들지 않는다. 경로 판정, stream 생성, 할당·import가 그 이유를 밝힌 `Unsupported`로 끝난다. 드라이버는 이때 다른 원인을 가리키는 오류(`cuDeviceGetCount` 실패 등)만 낸다
- Python `Frame`의 `__cuda_array_interface__`는 읽기 전용이다(구독자 매핑이 실제로 그렇다). torch는 읽기 전용 CAI를 받지 않으므로 torch에서는 `torch.from_dlpack`을 쓴다. DLPack capsule은 read-only flag 없이 낸다(torch가 거절한다). bf16은 CAI typestr가 없어 `__dlpack__`만 받는다
- Python `Frame`의 메모리는 `with`나 callback 안에서만 내준다. 해제는 그 범위가 끝날 때지만, 내준 메모리가 아직 쓰이면 늦춘다

  | 내준 방법 | 해제 시점 |
  | --- | --- |
  | 없음 | 범위 끝 |
  | `__dlpack__` | 범위 끝과 그 capsule에서 만든 tensor가 모두 사라진 때 |
  | `__cuda_array_interface__` | `Frame` 객체가 사라진 때. CAI consumer는 `Frame` 자체를 쥐기 때문이다 |
  | tensor 메시지 View(아래 "tensor 메시지") | 범위 끝과 그 View, 그 View에서 낸 numpy view와 tensor가 모두 사라진 때 |

  stream 대기와 fence는 모두 GIL을 놓고 한다. tensor나 `Frame`이 사라지며 일어나는 늦춘 해제도 같다. 그 stream에 GIL이 필요한 host callback(`cuLaunchHostFunc`의 Python 함수)이 걸려 있으면, GIL을 쥔 대기는 끝나지 않기 때문이다. 인터프리터가 끝나는 중에는 GIL을 쥔 채 한다. 그때 GIL을 다시 잡으려는 스레드는 영영 멈추기 때문이다
- `__dlpack__(stream=s)`로 받은 consumer stream은 해제 전에 기다린다. `Loan`의 `__dlpack__`으로 받은 stream은 commit과 abort 전에, 그리고 commit하지 않은 loan이 사라질 때 기다린다. 채널 stream만 기다리면 torch가 자기 stream에서 읽거나 쓰는 동안 슬롯이 넘어가기 때문이다. `None`·`-1`은 기다리지 않는다. legacy default stream(`1`)은 채널 stream이 blocking stream이라 CUDA가 이미 순서를 맞춘다. 이 기다림이 실패하면 view나 loan을 쥔 채 둔다(fence 실패와 같다)
- C++·Python 구독의 `fence_failed`·`fence_wait`는 마지막 take 시점의 값이다. 해제는 구독을 거치지 않는 호출이기 때문이다

이 Orin(L4T R39.2.1, CUDA 13.2)에서의 검증:

| 경로 | 방법 |
| --- | --- |
| `ShmRegistered` | 실제 경로. GPU가 쓴 loan을 CPU와 GPU 구독이 읽는다(core, C++, Python torch) |
| `DeviceHandle` | core는 경로 판정을 시험용으로 바꿔 실제 VMM으로, Python은 `integrated`·`pageable`을 0으로 답하는 libcuda shim을 child 프로세스에 걸어 끝까지 돈다. 밑은 여전히 통합 GPU라 emulated다 |
| `ShmDirect` | pageable access가 없어 돌릴 수 없다. `device_ptr`가 host 포인터와 같음만 확인한다 |
| fence 실패 | core에서 실패를 강제해 commit·해제 양쪽의 슬롯 보존과 `refused.fence`를 확인한다 |

x86 RTX 4070 SUPER(driver 580, CUDA 13)에서의 검증:

| 대상 | 방법 |
| --- | --- |
| `DeviceHandle` | 실제 경로. core, Python에서 부모가 torch로 쓰고 자식 프로세스가 `torch.from_dlpack`으로 읽는다 |
| 장치 불일치 | GPU가 한 장이라 core에서 UUID가 찾은 ordinal을 시험용으로 바꿔 `Mismatch`를 확인한다 |
| 사용자 context 파괴 | core에서 `cuCtxCreate`한 context를 current로 두고 할당·import한 뒤 파괴하고, 해제가 메모리를 돌려주는지 확인한다. 호스트 등록은 파괴 뒤에도 primary context에서 등록돼 있는지(`cuMemHostGetFlags`), 해제 뒤 같은 범위를 다시 등록할 수 있는지 본다. 이 장비는 pageable access가 있어 등록하지 않은 메모리도 GPU가 읽으므로, GPU로 읽어 보는 것으로는 구분되지 않는다 |
| Python 수명과 stream | `with` 밖으로 나간 tensor의 값, torch 자기 stream의 읽기·쓰기가 해제·commit 전에 끝나는지 확인한다 |
| sticky 오류 | PTX로 만든 kernel이 구독자의 READ import에 쓰게 해서 실제 illegal address(700)를 낸다. 커널 로그에 Xid 31(`FAULT_RO_VIOLATION`)이 남는다. 그 뒤 take, loan, stream 생성이 바로 거절되는지 본다. 일부러 GPU fault를 내므로 `#[ignore]`이고 `--include-ignored`로 따로 돌린다 |

GPU가 없어도 도는 시험은 대역 드라이버(`flux2_core/src/gpu/fake.rs`, `cfg(test)`)로 돈다. 시험 바이너리가 `FLUX2_FAKE_CUDA`를 걸고 자기 자신을 그 시험 하나만 도는 자식 프로세스로 띄우면, `Driver`가 `libcuda.so.1` 대신 이 대역을 쓴다. 통합 GPU 하나(`integrated`·`pageable`·`host_register`가 모두 1이라 판정은 `ShmRegistered`)를 흉내 내고 VMM은 없다. stream의 작업은 시험이 넣고(`submit`), 끝내고(`finish`), 실패시키고(`fail`), 끝내지 않는다. sticky 오류는 `wedge`로 내며, 그 뒤 드라이버 호출은 이미 기다리는 것까지 그 오류를 돌려준다. 사용자 context를 만들고 파괴할 수 있고, 파괴한 context를 쓰거나 객체를 두 번 해제하면 crash 대신 기록한다. 시험은 `gpu::tests::stand_in`에 있다

| 규칙 | 시험 |
| --- | --- |
| commit은 stream의 작업이 끝난 뒤 발행한다 | 끝내기 전에는 commit이 돌아오지 않고 구독자가 받지 못한다 |
| 해제는 작업이 끝난 뒤 슬롯을 돌려준다 | 끝내기 전에는 해제가 돌아오지 않는다 |
| 상한을 넘긴 commit은 claim을 쥐고, 끝나면 놓는다 | 발행하지 않았는지도 본다 |
| `Deferred` 해제는 곧바로 돌아가고 끝날 때까지 borrow를 쥔다 | |
| 끝나지 않는 작업은 슬롯을 쥐고, 호출자는 상한만큼만 기다린다 | |
| 연산 하나의 실패는 그 fence만 실패시킨다 | wedged가 아니고 다른 채널은 발행한다 |
| sticky 오류는 모든 GPU 채널을 바로 거절하게 한다 | 반복한 take가 모두 `refused.fence`이고 `max_borrow`는 0이다. publish, loan, 쥐고 있던 loan의 commit, stream·채널 생성, trace 사유, 호스트 채널이 그대로인지 본다 |
| fork한 자식은 드라이버를 부르지 않는다 | 자식에서 view·채널·stream을 버려도 드라이버 호출 수가 그대로이고, stream 생성은 fork를 밝힌 오류다 |
| 호스트 등록은 사용자 context를 파괴해도 남고, 그 context를 쓰지 않고 해제된다 | |

debug 빌드(`debug_assertions`)에서는 GPU 경로가 쥔 것을 장부(`flux2_core/src/gpu/ledger.rs`)에 적는다. 커널의 `DMA_API_DEBUG`에 해당한다. release 빌드에서는 장부 함수가 비어 있어 hot path 비용이 없다.

| 적는 것 | 바로 실패(panic)로 보는 것 | 누수로 보고하는 때 |
| --- | --- | --- |
| 호스트 등록, export, import, export한 fd와 크기 | 쥐지 않은 것의 해제(이중 해제), 만든 크기와 다른 해제 | 프로세스 종료(`atexit`) |
| GPU 채널의 borrow와 그것을 놓게 한 fence | borrow하지 않은 슬롯의 반환, 끝난 fence 없는 반환 | 채널이 사라질 때 |
| 실패한 fence가 일부러 쥔 borrow | | 따로 센다 |

- 장부는 GPU 채널에서만 쓴다. 호스트 채널의 take·해제에 lock을 더하면 Miri의 `race` 시험이 볼 data race를 그 lock이 가리기 때문이다
- READ import에 쓰는 kernel은 호스트에서 보이지 않는다. 장치가 fault를 내고(실제로 illegal address 700과 Xid 31 `FAULT_RO_VIOLATION`), 그것은 sticky 오류 처리와 아래 커널 로그 판정이 잡는다

compute-sanitizer로도 돌린다. 자식 프로세스로 도는 시험의 보고는 부모가 받아 두므로, 오류가 있으면 자식이 99로 끝나 부모의 시험이 실패한다.

```bash
cargo test -p flux2_core --no-run   # 시험 바이너리 경로를 찍는다
compute-sanitizer --tool memcheck --leak-check full --target-processes all --error-exitcode 99 \
  target/debug/deps/flux2_core-<hash> gpu:: --test-threads=1
compute-sanitizer --tool memcheck --leak-check full --target-processes all --error-exitcode 99 \
  build/flux2_cpp/test_gpu
```

이 RTX 4070 SUPER에서 core는 37개 통과(1개 ignored), 오류 0, 누수 0이다. C++ `test_gpu`는 위의 늦은 해제 때문에 import 하나(2 MiB)를 누수로 보고한다.

GPU 시험은 커널 로그로도 판정한다. 등록 해제 뒤 접근이나 READ import에 쓰기는 CUDA 오류 없이 IOMMU나 GPU의 fault 로그에만 남을 수 있기 때문이다. `scripts/kernel_log.sh`가 시험 앞뒤의 kernel 로그(journal, 읽을 수 없으면 dmesg)를 비교해 NVRM Xid, DMAR fault, AMD-Vi `IO_PAGE_FAULT`, SMMU fault·event, nvgpu MMU fault가 새로 나오면 실패로 본다.

| 시험 | 판정 방법 |
| --- | --- |
| C++ `test_gpu` | gtest 전역 environment가 시작과 끝에 표시하고 비교한다 |
| Python `test_gpu.py` | module fixture가 같은 일을 한다 |
| core `cargo test` | `scripts/kernel_log.sh run cargo test -p flux2_core gpu::`로 감싸 돈다 |

- root 없이 로그를 읽을 수 없으면(journal은 `adm`·`systemd-journal` 그룹, dmesg는 `kernel.dmesg_restrict=0`) 판정하지 않는다고 알리고 시험만 돈다
- 로그는 호스트 전체의 것이라 같은 시각 다른 프로세스의 fault도 실패로 잡는다. 일부러 Xid를 내는 `#[ignore]` sticky 시험은 이 판정 밖에서 따로 돌린다

## tensor 메시지

DLPack `DLTensor`의 필드만 가진 메시지는 flux2_gen이 필드 이름과 타입으로 알아본다. `tensor_msgs/ExperimentalTensor`(Lyrical)가 그 모양이다: `uint8 dtype_code`, `uint8 dtype_bits`, `uint16 dtype_lanes`, `int64[] shape`, `int64[] strides`, `uint64 byte_offset`, `uint8[] data`. 패키지 이름은 보지 않으므로 사용자가 복사한 메시지도 같다. 필드 하나가 이름, 타입, 중첩에서 다르거나 필드가 더 있으면 보통 메시지다. 판정은 `Layout::tensor_at`이고, 그 밖의 메시지는 생성물이 한 글자도 바뀌지 않는다(설치된 .msg와 시험 .msg 262개 메시지의 생성물을 바이트 단위로 대조했다).

생성 adapter는 dtype, shape, strides, byte_offset을 해석해 슬롯 위의 view를 낸다. 복사는 없다.

| 언어 | View | Builder |
| --- | --- | --- |
| Python | `as_tensor__()`는 numpy view. numpy에 이름이 없는 dtype(bf16, fp8)은 `flux2.wire.Tensor`(`.bits`, `__dlpack__`, `.array`는 `TypeError`). View 자체도 `__dlpack__`, `__dlpack_device__`를 낸다 | `alloc_tensor__(dtype, shape)`. dtype은 numpy dtype이나 DLPack 이름(`"bfloat16"`, `"float8_e4m3fn"` 등). 쓰기 가능한 view를 같은 규칙으로 돌려준다 |
| C++ | `as_tensor__()`는 `flux2::wire::Tensor`(첫 원소 포인터, `DlType`, shape, strides). `as<T>()`는 dtype이 `T`일 때만 포인터 | `alloc_tensor__(DlType, shape)`는 `void *`, `alloc_tensor__<T>(shape)`는 `Span<T>` |

- 프레임은 다른 프로세스가 쓴 것이라 필드를 믿지 않는다. `byte_offset`과 shape, strides, 원소 크기가 닿는 모든 원소가 `data` 안이어야 하고 첫 원소는 원소 크기 경계에 있어야 한다. 어기면 Python은 `WireError`, C++은 빈 결과와 `!ok__()`다. 복사로 대신하지 않는다
- 검사는 `flux2::wire::tensor_fault` 하나를 두 언어가 쓴다. Python은 shape와 strides를 슬롯에서 복사한 뒤 그 복사본을 검사하고 view도 복사본으로 만든다. 검사한 뒤 발행자가 슬롯을 고쳐도 검사하지 않은 값으로 view를 만들지 않기 위해서다. C++ `Tensor`의 shape, strides는 슬롯을 가리키는 span이다
- strides는 원소 단위이고 비어 있으면 C-contiguous다. 음수 stride도 범위 안이면 받는다(numpy는 읽고 torch는 거절한다). 크기가 0이나 1인 축의 stride는 밟지 않으므로 범위를 묻지 않는다
- 받는 dtype: Int·UInt 8~64비트, Float 16·32·64, Complex 64·128, Bool 8, BFloat 16, Float8 계열(코드 7~14). lanes가 1이 아니거나 opaque handle, sub-byte float(코드 15~17), DLPack에 없는 코드는 거절한다
- bf16과 fp8은 D-048대로 `.bits`(같은 폭 unsigned)와 `__dlpack__`로만 낸다. 다른 float로 위장하지 않는다
- 구독자 쪽 view는 읽기 전용이고 Builder 쪽은 쓰기 가능하다. CPU DLPack capsule도 GPU `Frame`처럼 read-only flag 없이 낸다. 구독자 매핑이 `PROT_READ`이므로 그 torch tensor에 쓰면 SIGSEGV다
- Builder는 `data`를 64바이트 경계에 잡는다(`flux2::wire::kTensorAlign`). 이를 위해 `Writer::alloc<T>(off, n, align)`을 더했다. 슬롯 시작이 page 경계라서 frame offset의 정렬이 주소의 정렬이다
- GPU: `device="cuda"` 구독의 `Frame`을 View에 넘기면 필드는 호스트 매핑(`Frame._host()`)으로 읽고, `__dlpack__`은 장치 tensor를 낸다. 주소는 등록한 장치 주소에 호스트에서 잰 `data` offset을 더한 것이다(`Frame._tensor_dlpack`). core와 C ABI는 바뀌지 않았다. 호스트 매핑이 없는 `DeviceHandle`은 이유를 붙여 거절한다
  - 해제는 `Frame` 규칙을 따른다. View는 `with`나 callback 안에서만 만든다. View의 호스트 view와 장치 tensor는 DLPack tensor처럼 frame을 쥔다. consumer stream(`__dlpack__(stream=)`)은 `Frame.__dlpack__`과 같이 해제 전에 기다리거나 `Deferred`에서는 순서만 건다
  - CPU 발행자와 `device="cuda"` 구독자의 조합은 이 Orin(`ShmRegistered`)에서 같은 프로세스와 다른 프로세스 모두 돈다
- 시험: `flux2_py/test/test_tensor.py`(실제 채널로 C++과 Python 양방향, strides·byte_offset, bf16·fp8, 범위·정렬 위반, 슬롯 안의 주소인지), `test_gpu.py`의 tensor 메시지 셋(장치 주소, 슬롯 유지, strided bf16, 위반), `flux2_gen/tests/emit.rs`의 판정, 시험 메시지 `flux2_gen/test/msg/DlTensor.msg`

## 구독자 crash

- 슬롯마다 구독자 번호별 칸(지금 읽는 수). 공용 `refcount` 없음
- 발행자는 칸이 전부 0인 슬롯에만 쓴다
- 구독자 번호 `i` = 세그먼트 fd의 바이트 `1+i` OFD 락. 살아 있는 동안 쥔다
- 락은 커널 상태라 파일 공간을 쓰지 않는다
- 새 구독자는 번호를 받으면 자기 칸을 먼저 0으로
- 회수는 빈 슬롯이 없을 때만
  1. 0이 아닌 칸의 번호를 모은다
  2. 번호마다 바이트 락 시도. 실패 = 살아 있음, 그대로 둔다
  3. 성공 = 죽음. 모든 슬롯의 그 번호 칸을 0으로, 락을 푼다
  4. 빈 슬롯을 다시 찾는다. 없으면 drop
- 평소 syscall 없음. 부족할 때만 번호당 락 시도 1회
- 구독 프로세스 수 상한은 발행자가 생성할 때 `max_subscribers`로 정한다(기본 16, 최대 4096). 한 프로세스의 구독들은 번호 하나를 같이 쓴다. 번호마다 슬롯별 칸과 대기 칸이 있어 claim과 publish가 번호 수만큼 읽는다. 다만 claim, 대기 확인, 회수는 지금까지 쓰인 가장 큰 번호(헤더의 `readers_high`)까지만 읽는다. 구독자는 번호를 받을 때 borrow나 대기보다 먼저 이 값을 `fetch_max`로 올린다. 발행자의 claim CAS가 이 값 읽기보다 먼저이므로, 발행자가 놓친 번호의 구독자는 ticket 재확인에서 claim을 본다. 그래서 비용이 한도가 아니라 실제로 쓰인 번호 수에 비례한다(구독자 둘이면 한도 16부터 4096까지 publish+take 약 68 ns). 넘으면 attach 거절. 번호가 모두 잠겨 있으면 약 10 ms(100 us 간격 100번) 다시 찾은 뒤 거절한다. 발행자가 죽은 번호를 회수하는 동안 그 번호의 lock을 잠깐 쥐기 때문이다

## C ABI

`flux2_ffi`가 core 위에 C 함수를 내보낸다. 헤더 `flux2_ffi/include/flux2.h`는 cbindgen이 Rust 소스에서 생성하고, 생성본을 커밋한다. 빌드는 cbindgen에 의존하지 않는다. pre-commit(`scripts/gen_c_header.sh --check`)이 다시 생성해 보고, 차이가 있으면 실패한다.

| 항목 | 방식 |
| --- | --- |
| 핸들 | 불투명 포인터. `_free`는 NULL 허용 |
| view·loan | 호출자 메모리의 고정 크기 저장소(`flux2_view` 128바이트, `flux2_loan` 128바이트). 0으로 초기화하면 빈 상태. 첫 바이트가 태그다. 값으로 복사하지 않는다. 옮기는 것은 바이트를 복사한 뒤 원본을 0으로 만드는 것뿐이다(iceoryx2, zenoh-c도 복사를 감지하지 않는다) |
| 오류 | `flux2_status` 반환과 스레드별 `flux2_last_error()` |
| 버전 | soname `libflux2_ffi.so.<FLUX2_ABI_VERSION>`, `flux2_abi_version()`. 지금 3이다(trace 계약 2의 `stamp_offset` 인자와 `flux2_source.endpoint`). C++ 래퍼는 처음 쓸 때 헤더의 값과 비교해 다르면 `Mismatch`를 던진다. 옛 헤더로 빌드한 바이너리가 view·loan 저장소 밖을 덮어쓰는 것을 막는다 |
| callback | 함수 포인터 + ctx. `attach`·`deliver_one`은 음수, hook은 0이 아닌 값이 실패다. 실패하면 spin이 `FLUX2_CALLBACK`으로 끝난다 |
| 상수 | 리터럴로 적고 core 값과 같은지 컴파일 시점에 assert |

예외는 경계를 넘지 않는다. C++ 래퍼는 callback 안에서 잡은 예외를 ctx에 보관했다가, 그 callback을 부른 C 호출이 끝나면 반환 코드와 상관없이 다시 던진다. `flux2_executor_add`는 실패해도 source를 소비한다(`drop`이 한 번 돈다).

ABI 테스트는 C 프로그램(`flux2_ffi/tests/c/`)이다. `cargo test`가 커밋된 헤더와 staticlib으로 이들을 빌드해 돌리고, 헤더를 C++로도 컴파일해 본다. 라이브러리가 내보낸 `flux2_*` 심볼이 모두 헤더에 있는지도 확인한다. cbindgen은 매크로가 만든 함수를 보지 못하기 때문이다.

## trace

trace 이벤트는 core의 `flux2_trace_*` 함수다(`flux2_core/src/trace.rs`). 함수 자체는 아무것도 하지 않고, 수집기(armo의 uprobe, LD_PRELOAD, LTTng)가 이름으로 붙는 지점이다. 이 함수들은 `libflux2_ffi.so`가 내보내므로, C++·Python이 공유 라이브러리로 링크하면 armo가 프로세스의 maps에서 찾는다. 이벤트 이름, 인자, 상수는 계약이고 `flux2_trace_contract_2`가 그 버전이다. 이름이나 인자를 바꾸면 번호를 올리고 옛 심볼은 지운다. 이벤트나 상수를 더하기만 하면 번호를 그대로 둔다(`gpu_wedged`, `*_GPU_WEDGED`). 정본은 `flux_docs/armo/design.md` 11절이고, C 선언과 상수는 설치 헤더 `flux2/trace.h`다.

| 이벤트 | 인자 | 부르는 곳 |
| --- | --- | --- |
| `channel_init` | channel, dev, ino, segment, publisher, slot_count, slot_size, stamp_offset | 채널 생성과 재attach |
| `claim` | channel, slot, result(`CLAIM_*`), loan | publish와 loan의 슬롯 획득, claim 전 거절 |
| `loan`, `loan_end` | channel, slot(, result `LOAN_*`) | loan 시작, commit·abort마다 한 번 |
| `commit` | channel, ticket, slot, nbytes, woke, stamp | 발행 |
| `take` | channel, ticket, slot, lost, stamp | 받음 |
| `release` | channel, ticket, held | view 해제(fence 결과 뒤) |
| `refused` | channel, reason(`REFUSED_*`) | borrow 거절 |
| `gpu_wedged` | result(`CUresult`) | sticky CUDA 오류로 프로세스의 GPU를 wedged로 표시할 때, 프로세스에 한 번 |
| `wait`, `wake` | executor, sources, timeout_ns / executor, result(`WAKE_*`), entry | `wait_for_work`, 두 대기 경로 같음 |
| `callback_start`, `callback_end` | executor, entry, endpoint / executor, entry, delivered, channel, segment | dispatch |
| `endpoint_init` | endpoint, node, key, signpost, fingerprint, role, lang | flux2_cpp·flux2_py의 Publisher·Subscription 생성자 |
| `py_gil`, `py_ready` | phase / channel, ticket | flux2_py의 GIL 재획득, 콜백 직전 |
| `record_drop` | channel, ticket, count, reason(`RECORD_*`) | flux2_bag 녹화기가 bag에 쓰지 못한 프레임 |

- channel 식별자: `ChannelShared`의 주소. 재attach 전까지 고정이다. 바인딩은 `flux2_view_trace_channel`로 얻는다
- executor 식별자: 제어 블록의 주소. executor를 옮겨도 고정이다
- endpoint 식별자: 바인딩의 Publisher·Subscription 객체 주소. executor source는 `flux2_source.endpoint`(core `Source::endpoint`)로 같은 주소를 넘긴다
- `callback_end`의 segment: owner 파일이 없으므로, 늦게 붙은 도구도 이 인자로 callback의 채널을 안다
- roster label은 노드의 fully qualified name이다. armo는 `/`로 시작하는 label을 노드로 읽는다
- stamp: 최상위 `std_msgs/Header`의 stamp 8 B를 little-endian u64로 읽은 값(sec 아래, nanosec 위)이다. 위치(`stamp_offset`)는 flux2_gen이 정하고, 생성 코드가 실릴 때 fingerprint별로 `libflux2_ffi`에 등록하며(`flux2_stamp_offset_register`), flux2_cpp가 채널을 만들 때 찾아 C ABI 인자로 넘긴다. Header가 없는 타입, raw API, device payload는 0이다

함수마다 asm 문자열이 다르다. 같으면 LLVM이 같은 시그니처의 빈 함수를 한 주소로 합쳐서, 한 이벤트의 uprobe가 다른 이벤트에도 걸린다. 모든 인자는 asm 입력 operand로 읽는다. 읽지 않으면 `codegen-units=1`이나 fat LTO 빌드에서 호출 지점이 인자 레지스터를 채우지 않아 수집기가 쓰레기 값을 받는다. `flux2_ffi/tests/trace_args.rs`가 그 빌드의 cdylib와 staticlib에서 C 프로그램의 이벤트 값을 기록기로 확인한다. `flux2_ffi`의 ABI 테스트는 18개 함수가 서로 다른 주소로 export되고 `flux2/trace.h`에 선언돼 있으며 상수가 core와 같은지 본다. LTTng·LD_PRELOAD 같은 수집 방식은 armo가 맡는다.

끈 상태(아무것도 붙지 않음) 비용은 이벤트당 0.7~2.6 ns다(Orin, CPU 고정, 9회 중앙값, `flux2_core/benches/trace_cost.rs`). 호출 지점은 `trace::event!` 매크로이고, `RUSTFLAGS="--cfg flux2_trace_off"` 빌드가 호출 지점을 인자 계산과 함께 지워 비교 기준이 된다. 이 cfg는 측정용이고 기능이 아니다.

| 루프 | 반복당 이벤트 | 켬 | 끔 | 이벤트당 |
| --- | --- | --- | --- | --- |
| heap 채널 publish, take, 해제 | 4 | 196.7 ns | 193.9 ns | 0.7 ns |
| executor dispatch(콜백 안에서 publish, take) | 6 | 573.4 ns | 558.0 ns | 2.6 ns |
| `wait_for_work(0)` | 2 | 1218.3 ns | 1216.6 ns | 0.9 ns(syscall 잡음 안) |

## 열거

열거용 manifest는 signpost 안의 roster에 둔다. flux2에는 owner 파일이 없어서, 같은 OFD lock 방식을 signpost에 한 번 더 쓴다. 칸 `j`의 주인은 signpost 바이트 `2+j`에 write lock을 쥔 프로세스다. 프로세스가 죽으면 커널이 lock을 풀고, 열거는 lock이 걸린 칸만 읽는다(`F_OFD_GETLK`). 열거는 아무것도 바꾸지 않는다.

- signpost 크기: 16 KiB
- endpoint 칸: 62개
- 칸 내용: pid, 발행자 여부, label. 주인이 seqlock으로 쓴다.
  - 쓰는 쪽: `seq`를 홀수로 store한 뒤 Release fence, 필드들(모두 atomic, Relaxed), 짝수 `seq` Release store다. 홀수 store만 Release로 하면 뒤의 쓰기가 그 앞으로 넘어올 수 있다(ARM에서만 드러난다)
  - 쓰다 죽은 주인은 `seq`를 홀수로 남긴다. 새 주인은 그 값에서 시작한다(`seq | 1`). lock이 풀린 칸은 열거가 읽지 않으므로 남은 홀수는 새 주인이 쓰기 전까지만 보인다
  - 읽는 쪽: 짝수 `seq`(Acquire), 필드, label 전체(240 B)를 고정 버퍼로 복사, Acquire fence, `seq` 재확인이다. 64번 안에 맞지 않으면 그 칸은 "busy"다. `Topic::busy`로 세고 `flux2 topic info`가 표시한다. C ABI의 열거 구조체는 배치를 바꾸지 않으려고 이 값을 싣지 않는다
  - label 길이는 재확인을 통과한 뒤에야 쓴다. 버퍼보다 길면 쓰는 쪽이 만들 수 없는 값이므로 그 칸을 목록에서 뺀다
  - 비용 모델: 읽기 재시도는 쓰기 빈도 × 읽기 시간에 비례한다. 칸은 announce와 종료 때만 쓰므로 재시도는 드물다
- 정확한 key: 처음 announce한 쪽이 한 번만 쓴다. 열거는 key를 평탄화한 값이 signpost 이름의 key 부분과 같을 때만 정확한 key로 낸다. 다르면 이름에서 되살린 key를 `key_exact = false`로 낸다
- 62칸이 차면 그 뒤의 announce는 roster에 오르지 않는다. 프레임 수신과는 상관없고 목록에만 빠진다. `max_subscribers`를 따라 늘리지 않는다. signpost는 발행자보다 먼저 생길 수 있고 재시작에도 남아서, 만들 때 그 값을 모르기 때문이다. `flux2 topic info`는 목록이 찼으면 그렇다고 표시한다

roster가 signpost에 있으므로 발행자가 없어도 announce한 구독자가 보이고, 발행자가 재시작해도 등록이 유지된다. API는 C++ flux와 같다(`ManifestEntry`, `Announcement`, `Topic`, `Endpoint`, `enumerate_topics`, `ChannelStats`, `read_channel_stats`). 다만 `Endpoint`는 `(pid, starttime)` 대신 pid만 갖는다.

## signpost 수명

signpost를 연 프로세스(발행자, 구독자, announce한 쪽)는 signpost 바이트 64(`SIGNPOST_USER_BYTE`)에 read lock을 쥔다. read lock은 여럿이 같이 쥘 수 있다. 마지막으로 닫는 쪽은 이 lock을 write lock으로 바꿀 수 있으므로, lock을 쥔 채로 unlink한다. zenoh가 세그먼트를 다루는 방식과 같다.

| 상황 | 결과 |
| --- | --- |
| 발행자가 재시작하고 구독자가 기다리는 중 | 구독자가 lock을 쥐고 있어 signpost와 epoch가 유지된다 |
| 모든 사용자가 정상 종료 | 마지막 사용자가 지운다. 마지막 둘이 동시에 나가면 서로의 read lock 때문에 둘 다 write lock으로 못 바꾼다. 그래서 각자 자기 read lock을 놓고 한 번 더 write lock을 시도하고, 성공한 쪽이 이름이 아직 그 객체를 가리키는지 확인한 뒤 지운다 |
| 사용자가 crash로 죽음 | 커널이 lock을 푼다. `sweep_dead`가 지운다. sweep은 프로세스마다 한 번(첫 발행자, 첫 구독, fork한 자식의 첫 구독)이고, 발행자는 생성할 때 자기 signpost의 이전 epoch 세그먼트를 따로 확인해 지운다. 전체 sweep을 막는 전역 lock 파일은 없다. 지우는 쪽은 모두 그 객체의 생존 바이트를 write lock으로 쥐거나 직접 쥔 채 지우므로, sweep끼리 겹쳐도 같은 이름을 잘못 지우지 않는다 |

- 열 때는 lock을 잡은 뒤 이름이 아직 그 객체를 가리키는지 확인한다(`name_still_bound`). 그 사이 다른 쪽이 지웠으면 다시 연다
- 열거와 CLI는 lock을 쥐지 않는다. 읽기만 하므로 지워져도 해가 없다
- sweep은 이 빌드의 layout version 이름만 본다. 생존은 lock 바이트로 판정하는데, 다른 버전은 그 바이트 배치가 다를 수 있어 살아 있는 객체를 지울 수 있기 때문이다. 다른 버전이 남긴 것은 그 버전의 프로세스가 지운다

## 정리 순서

세그먼트가 사라질 때의 순서다(`Segment::drop`, 그 뒤 필드 순서).

```text
발행자 이름 unlink(공개 목록에서 내림, 구독자의 생존 확인은 이름이 없으면 죽음으로 본다)
 -> 장치 payload 해제: fd 서버 스레드 정지·join, export fd 닫기, VMM 해제 / 호스트 등록 해제 / import 해제
 -> munmap(마지막 Mapped가 갈 때. commit한 loan의 배열이 쥐고 있으면 그때까지 늦다)
 -> 발행자 생존 lock fd 닫기, signpost(발행자 바이트, 사용자 lock) 놓기
```

- 비동기 실행원 중 GPU 대기 스레드는 공유메모리를 만지지 않고 join하지 않는다. 늦춘 해제와 시간을 넘긴 fence는 `ChannelShared`를 쥐므로 그 일이 끝나기 전에는 이 순서가 시작되지 않는다
- 전에는 `map` 필드가 `device` 필드보다 먼저 drop되어 호스트 등록 해제가 munmap 뒤에 돌았다. 시험 `a_registration_is_undone_before_its_mapping_goes`가 해제 시점에 그 범위가 아직 매핑돼 있는지 `mincore`로 본다
- 발행자가 사라진 세그먼트의 마지막 매핑을 놓는 스레드가 그 tmpfs 페이지를 모두 해제한다(마지막 파일 참조가 그 munmap이나 구독자 번호 fd다). 128 MiB에서 7.3 ms, 16 MiB에서 0.8 ms다(Ryzen 5 9600X). 재attach는 executor 스레드의 take 안에서 일어나므로, 옛 `ChannelShared`를 놓는 세 곳(채널의 재attach, executor의 대기 핸들 교체, parker가 다 쓴 겨냥)은 직접 drop하지 않고 `flux2-reaper` 스레드에 넘긴다. 128 MiB 세그먼트의 재attach take가 5.1 ms에서 0.04 ms가 됐다
  - reaper는 프로세스에 하나이고 첫 구독을 여는 호출(사용자 스레드)이 만든다. 모든 신호를 막은 채 시작하고, 시작하자마자 SCHED_OTHER, nice 0으로 내려간다. 아무도 기다리지 않는 정리 스레드라서 일부러 보통 우선순위로 돈다. 만든 스레드의 RT 정책을 물려받으면 페이지 해제가 정작 비켜 주려던 RT 스레드를 밀어내기 때문이다. 양수 nice를 0으로 낮추려면 권한(`RLIMIT_NICE`)이 있어야 하므로, 양수 nice 스레드에서 만들면 그 값으로 남는다. CPU affinity는 건드리지 않는다(cyra가 정할 수 있다). 시험은 SCHED_FIFO가 허용되면 FIFO 스레드에서, 아니면 SCHED_BATCH 스레드에서 만들어 reaper가 SCHED_OTHER인지 본다
  - fork한 자식에는 reaper가 없으므로 그 자리에서 drop한다
  - 옛 세그먼트의 view를 콜백 밖까지 쥐다 놓는 경우는 그 view를 놓는 스레드가 값을 치른다

## CLI

`flux2_cli`가 바이너리 `flux2`를 만든다. colcon이 `bin`에 설치하므로 워크스페이스를 source하면 PATH에 있다. core 바로 위에 있어서 Python이나 ROS 없이 돈다. /dev/shm을 읽기만 하고 lock이나 borrow를 잡지 않으므로, 대상 프로세스는 이를 알아채지 못한다. 명령과 출력 형식은 C++ flux CLI를 따른다.

| 명령 | 하는 일 |
| --- | --- |
| `flux2 topic list` | 채널, 세그먼트 up/down, endpoint 요약 |
| `flux2 topic info <channel>` | fingerprint, signpost, epoch, 슬롯, storage, 발행 프레임 수, wake gate 대기자, endpoint |
| `flux2 topic hz <channel> [--window SEC]` | 발행 속도 |

- 호스트의 채널 중 이 사용자가 열 수 있는 것을 다룬다. 열 수 없는 signpost는 열거에서 빠진다
- `--window`는 1 ms 이상이다. key, label, signpost의 제어 문자는 escape해서 찍는다. 크기는 단위로 나눠 떨어지지 않으면 소수 한 자리로 찍는다
- `hz`는 세그먼트의 commit 카운터(`publish_seq`)로 잰다. 받은 쪽과 무관하게 발행자가 commit한 수다. epoch가 바뀌면 카운터가 새로 시작하므로 그 구간은 표시만 한다
- 아무도 announce하지 않은 채널은 이름에서 되살린 key에 `(from name)`을 붙이고, 그 이름으로도 찾을 수 있다. announce된 채널은 자기 key로만 찾는다

## 메모리 순서

공유 word마다 쓰는 쪽, 읽는 쪽, `Ordering`, 짝, 모양, 선형화 지점, 공개 전·후를 적는다. 모양은 litmus 이름이다.

| 모양 | 뜻 |
| --- | --- |
| MP | 데이터를 쓰고 깃발을 `Release`로 쓴다. 깃발을 `Acquire`로 보고 데이터를 읽는다 |
| SB | 두 쪽이 각자 자기 word를 쓰고 상대 word를 읽는다(Dekker). 둘 다 옛 값을 읽으면 안 된다 |
| LB | 데이터를 읽은 뒤 `Release`로 내놓고, 상대는 그것을 `Acquire`로 보고 덮어쓴다 |

규칙은 셋이다.

- SB 짝은 두 쪽 모두 `SeqCst`이거나 사이에 `SeqCst` fence가 있다. 한 자리라도 acquire/release면 둘 다 옛 값을 읽을 수 있다
- 깃발을 보고 필드를 읽는 쪽은 깃발 load가 `Acquire` 이상이다. 분기(control dependency)는 C11에서 순서를 주지 않는다
- 해제는 LB다. 데이터를 읽은 뒤의 감소는 `Release` 이상, 0을 보고 덮어쓰는 쪽의 읽기는 `Acquire` 이상이다

공유메모리의 word(프로세스 사이):

| word | 쓰는 쪽 | 읽는 쪽 | 짝, 모양 | 선형화 지점 | 공개 전·후 |
| --- | --- | --- | --- | --- | --- |
| 슬롯 `commit_ticket` | 발행자. claim CAS `SeqCst`(`try_claim`), 되돌림 `Release`, commit `Release`(`finish_commit`), abort `SeqCst` | 구독자. borrow 확인 `SeqCst`(`take`), 후보 고르기 `Relaxed`(`next_frame`). 발행자의 claim 전 읽기 `Relaxed` | SB: claim CAS와 구독자 +1(Dekker 1). MP: meta·payload·`publish_ns` → commit·되돌림 → borrow 확인 | claim: CAS 뒤 칸 재확인이 모두 0. borrow: +1 뒤 ticket 확인이 같은 값 | `CLAIMED` 동안 meta·payload를 쓴다. ticket store 뒤에는 다음 claim까지 쓰지 않는다 |
| 구독자 칸 `readers[s][i]` | 구독자 +1 `SeqCst`, -1 `Release`(`end_borrow`, take의 실패 경로). -1 전에 슬롯 ticket을 `Relaxed`로 읽어 오해제를 거르고(해제는 이미 쥔 borrow라 순서를 더 얻을 것이 없다), 이미 0이던 칸의 되돌림은 `Relaxed`다. 회수 `store(0)` `Release`(`clear_reader`) | 발행자. claim 전 훑기 `Acquire`(힌트), 재확인 `SeqCst`(`readers_free`), 회수 훑기 `Acquire` | SB: +1(Dekker 1). LB: payload 읽기 → -1 → 재확인 → 덮어쓰기 | 해제: -1 | 번호를 받을 때 자기 칸을 0으로 한다 |
| `readers_high` | 구독자 `fetch_max` `SeqCst`(`raise_readers`), 첫 borrow·대기 전 | 발행자. claim 재확인과 wake gate `SeqCst`, 훑기·회수 `Acquire` | 두 SB의 셋째 word. 구독자는 올린 뒤 칸을 쓰고, 발행자는 CAS·bump 뒤 이 값을 읽는다 | - | - |
| `wakeup`(futex word) | 발행자 bump `SeqCst`(`finish_commit`) | 대기 `SeqCst`(`wait`의 빠른 확인)와 커널 비교. executor 샘플 `Acquire`(`wake_seq`), `ready` `Acquire` | SB: bump → waiters 읽기, waiters 올림 → word 비교(Dekker 2). MP: `latest` → bump → 샘플 → take의 `latest` | 대기: 커널 비교. 깨움: 발행자의 waiters 읽기 | u32 카운터라 2^32번 bump 뒤 같은 값이면 샘플과 같아 보인다(ABA). 의도한 무시 |
| `waiters`, `waiters[i]` | 대기자 `SeqCst`(`add_waiter`, `remove_waiter`). 회수 `store(0)` `Release` | 발행자 gate `SeqCst`, 회수 훑기 `Acquire` | SB(Dekker 2). 대기 쪽은 futex 호출 안 커널 full barrier(`futex_wait`의 `smp_mb`)가 이미 순서를 준다. 필요한 것은 발행자 쪽 `SeqCst`다 | gate: waiters 읽기 | - |
| `latest` | 발행자 `Release`(`finish_commit`) | 구독자 `Acquire`(`take`, `ready`, 커서 자리 잡기) | MP: 색인 칸·슬롯 ticket·payload → `latest` | take가 보는 commit | 색인 칸과 ticket을 먼저 쓴다 |
| 색인 칸 `index[t % L]` | 발행자 `Relaxed` | 구독자 `Relaxed`(`next_frame`) | `latest` MP의 데이터. 힌트라 슬롯 ticket과 borrow 확인을 다시 한다 | - | `latest` 전 |
| meta, payload, `publish_ns` | 발행자 plain(`publish_ns`는 `Relaxed`) | 구독자 plain, borrow 확인 뒤 | 위 MP·LB의 데이터 | - | claim 동안만 쓴다 |
| 세그먼트 `init_state` | 발행자 `Release`(`init_header`) | `Acquire`(`map_validate`, `read_channel_stats`) | MP: 헤더 plain 필드 → `INIT_READY` | 세그먼트 완성 | 헤더는 그 전에 쓰고, 뒤에는 atomic만 바뀐다 |
| signpost `epoch` | 발행자 `Release`(`open_publisher_segment`, 세그먼트 완성 뒤)와 futex wake | `Acquire`(`SignpostMap::epoch`, `SignpostView::epoch`). watch와 parker의 futex 비교 | MP: 세그먼트(`INIT_READY` 포함) → epoch. 새 epoch를 본 구독자는 준비 안 된 세그먼트를 보지 않는다 | 발행자 공개 | - |
| signpost `init_state` | bootstrap `Release`(OFD lock 아래) | `Acquire` | MP: magic·version·epoch 0 → `INIT_READY` | - | - |
| roster 칸 `seq`와 필드 | 주인(OFD lock). 홀수 `Relaxed` → fence `Release` → 필드 `Relaxed` → 짝수 `Release`(`write_entry`) | 열거. `Acquire` → 필드 `Relaxed` → fence `Acquire` → `Relaxed` 재확인(`read_entry`) | seqlock | 쓰기: 짝수 store. 읽기: 재확인 | 쓰다 죽은 주인은 홀수를 남기고, 다음 주인이 그 값에서 잇는다. 읽기는 64번 시도 뒤 빈 칸이다 |
| key 기록 `state` | 첫 announce. CAS `Acquire`, `KEY_READY` `Release` | `Acquire`(`read_key`) | MP | - | 한 번만 쓴다 |

프로세스 안의 word(스레드 사이):

| word | 쓰는 쪽 | 읽는 쪽 | 짝, 모양 | 공개 전·후 |
| --- | --- | --- | --- | --- |
| `outstanding`, `release_armed` | 해제: `outstanding` -1 `SeqCst` → `release_armed` 읽기 `SeqCst`, swap `SeqCst`(`end_borrow`). executor: `release_armed` `SeqCst`(`arm_release`) → `outstanding` 읽기 `SeqCst`(`can_borrow`) | 같다 | SB(Dekker 3). swap은 깨움을 한 번으로 하고 순서에는 쓰이지 않는다 | - |
| `Control::bell` | poke `fetch_add` `SeqCst`와 futex wake(`poke`, `ring`) | executor의 futex 비교(`heard`), `drain` `Acquire` | 커널 비교가 순서를 정한다. `rang`은 bell의 RMW에 실린다 | - |
| `Aim`의 generation·word·seen | executor만. 홀수 `Relaxed` → fence `Release` → 필드 `Relaxed` → 짝수 `Release`(`post`) | parker. `Acquire` → 필드 `Relaxed` → fence `Acquire` → `Relaxed` 재확인(`read`), 변경 확인 `Acquire` | seqlock(roster와 같은 모양) | - |
| `Aim::taken` | parker `Release`(새 겨냥을 읽은 뒤) | executor `Acquire`(`reclaim`) | MP: 옛 word의 마지막 접촉 → taken → 옛 주인 해제 | retired 주인은 taken이 그 post에 닿을 때까지 쥔다 |
| `Aim::stop` | executor drop `Release` | parker `Acquire` | stop 뒤 빈 겨냥 post와 wake. 사이에 낀 parker는 `PARK_NS`(50 ms) 안에 본다 | - |
| `PiMutex` word | 잠금 CAS 0 → tid `Acquire`, 풀기 CAS tid → 0 `Release`. 경합하면 `FUTEX_LOCK_PI`·`FUTEX_UNLOCK_PI`(커널이 순서를 정한다) | 같다 | mutex(acquire/release) | - |
| `Mark::state`(늦은 GPU 작업) | 대기 스레드. GPU 완료를 본 뒤 `DONE`·`FAILED` `Release`와 futex wake | 채널 스레드의 회수와 상한 대기 `Acquire` | MP: GPU 완료 → state → 슬롯 반환(반환은 채널 스레드만 한다). 채널이 먼저 사라지면 넘김(`hand_off`)과 대기 스레드가 `orphan` mutex로 만난다. state는 그 lock 전에 쓰므로 lock을 둘째로 잡는 쪽이 최종 state를 본다 | - |
| `WEDGED` | 첫 sticky 오류 CAS `Relaxed` | 채널마다 `Relaxed` | 거절 깃발이다. 그 값(CUDA 오류 코드) 말고는 넘기는 데이터가 없다 | - |

### 순서 검사

Miri 테스트(`scripts/miri.sh`):

| 테스트 | 보는 것 |
| --- | --- |
| `channel::race::a_held_view_is_never_rewritten` | 구독자 둘(번호 1, 2), 슬롯 셋. claim·borrow, commit, 해제 |
| `channel::race::a_claim_and_a_borrow_of_one_slot_exclude_each_other` | 슬롯 하나라 모든 프레임에서 Dekker 1이 만난다. depth가 색인보다 커서 뒤처진 take가 슬롯을 훑고, 읽은 `latest`보다 새 ticket을 찾는다 |
| `channel::race::a_waiting_subscriber_sees_the_last_frame` | wake gate. 번호 칸 대기와 헤더 칸 대기, 시간 상한 없음. 잃은 wake는 Miri의 deadlock이다 |
| `channel::race::a_release_wakes_an_executor_at_max_borrow` | Dekker 3, deadlock |
| `channel::race::a_take_finds_every_frame_through_the_index` | 색인 → `latest`. 건너뛴 프레임은 assert |
| `manifest::race::a_read_entry_is_never_torn` | roster seqlock |
| `executor::race::a_retired_word_outlives_the_parkers_last_touch` | 겨냥 seqlock과 `taken`. 일찍 놓은 word는 use after free나 race다 |

flux_model_checking의 `tools/mutate_orderings.py rust`로 `channel.rs`·`segment.rs`·`executor.rs`·`manifest.rs`·`discovery.rs`의 자리 83개를 하나씩 낮춰 이 테스트를 seed 64개로 돌렸다(2026-10-11). `SeqCst`는 acquire/release 형태로, 나머지는 `Relaxed`로 낮춘다.

| 결과 | 자리 |
| --- | --- |
| race | claim CAS, claim 재확인, 구독자 +1과 ticket 확인(Dekker 1 네 자리 모두), commit의 ticket `Release`, `end_borrow`의 -1 `Release`, 겨냥 짝수 store `Release`, 겨냥 읽기 `Acquire`, `taken` store `Release` |
| deadlock | bump `SeqCst`, 번호 칸 gate 읽기 `SeqCst`, `end_borrow`의 `release_armed` 읽기 `SeqCst`, `wake_seq` `Acquire` |
| assert | `latest`의 `Release`와 take의 `Acquire`, roster의 fence 둘과 짝수 store, 첫 읽기 |

잡히지 않은 자리와 이유:

| 자리 | 이유 |
| --- | --- |
| 헤더 칸 gate 읽기, gate의 `readers_high` 읽기, `readers_high` 올림, Dekker 3의 나머지 세 자리 | SB다. 한 자리만 낮추면 Miri가 그 결과를 드물게 만든다(Dekker 3의 executor 쪽 둘을 함께 낮추면 seed 64개 중 1개). `litmus/sb_wake.c`, `sb_claim.c`, `sb_release.c`가 RC11에서 본다 |
| 대기 쪽 `add_waiter`·`remove_waiter`, `wait`의 빠른 확인 | futex 호출 안의 full barrier가 이미 순서를 준다(Miri도 futex를 `SeqCst` fence로 다룬다). 필요 이상이고 대기마다 한 번이라 그대로 둔다 |
| 되돌림 `Release` | 뒤처진 take가 되돌린 ticket을 읽는 경로에서만 쓰인다. 테스트가 거의 닿지 않는다. `sb_claim.c`가 본다 |
| claim 전 훑기 `Acquire`, take 실패 경로의 -1 `Release` | 힌트이거나, ticket만 읽고 슬롯을 읽지 않았다. 필요 이상이다 |
| bad frame 경로의 -1 `Release` | 테스트 프레임이 깨지지 않아 닿지 않는다. meta를 읽은 뒤라 필요하다 |
| abort의 `SeqCst` | EMPTY를 보고 슬롯을 읽는 쪽이 없다. 필요 이상이다 |
| `latest`의 커서 자리 잡기·`ready` 읽기, `outstanding`의 증감, executor `ready`의 word 읽기 | 그 값을 보고 공유 데이터를 읽지 않는다. 판단에만 쓴다 |
| parker 루프의 stop·겨냥 변경·word 읽기 `Acquire`, stop `Release`, swap `SeqCst` | 테스트가 이 순서의 위반을 관찰하지 못한다. stop의 늦음은 50 ms 틈과 같다 |
| 회수(`clear_reader`, `reclaim_dead`), `readers_in_use` `Acquire` | heap 테스트에 죽은 번호가 없다. `readers/` 모델이 알고리즘을 본다 |
| 겨냥 seqlock의 fence 둘, `reclaim`의 `taken` `Acquire` | 테스트가 잡지 못한다. Miri는 futex 호출의 `SeqCst` fence로 parker에서 executor로 가는 다른 happens-before를 만드는 것으로 보인다. `litmus/seqlock.c`, `mp_taken.c`가 본다 |
| `Control`(bell, interrupt, spinning, stop), fence 대기 카운터(`late_count`, `leases_leaked`) | 이 테스트가 다루지 않는다. bell은 커널 비교가 순서를 정한다 |
| `init_state`, `epoch`, signpost bootstrap, key 기록, 통계 | Miri가 shm 경로를 돌지 않는다. `litmus/mp_epoch.c`가 본다 |

리뷰는 해제 쪽 감소를 `Relaxed`로 낮춘 버그를 Miri가 놓친다고 했지만 잡는다. Miri는 LB 결과를 만들지 않지만, race 검출이 happens-before(vector clock)로 하기 때문이다. 감소와 덮어쓰기 사이에 happens-before가 없으면 재배치가 실제로 일어나지 않아도 payload 읽기와 쓰기를 race로 보고한다.

GenMC litmus(flux_model_checking의 `litmus/`)는 GenMC가 없는 곳에서 썼고 아직 RC11에서 돌려 보지 않았다. 대신 `make native`로 C로서 유효한지만 본다.

## 검증

fuzz 대상은 `flux2_core/src/fuzz.rs`의 `frame`, `attach`, `roster` 셋이다. `attach`는 첫 바이트가 홀수면 파일을 헤더가 말하는 크기(16 MiB까지)로 맞춰, 헤더만 있는 입력도 attach 검사까지 간다. `cargo test`가 이 함수들을 고정 corpus로 돌리고, `fuzz/`(cargo-fuzz, 고정 nightly)가 같은 함수를 fuzzing한다.

model checking 하니스는 비공개 저장소 flux_model_checking에 둔다. 손으로 쓴 C 모델이라 이 저장소의 소스를 include하지 않는다. `wake`의 CRASH 설정은 어느 단계에서든 죽을 수 있는 대기자와 발행자의 sweep을 넣고, 두 store로 세는 옛 방식(`BUG_TWO_STORES`)을 누수로 잡는다. `readers`의 LOAN 설정은 loan abort를 넣는다. fork 가드와 1초 주기는 모델에 없다.

실제 Rust 코드의 `Ordering`은 Miri로 검사한다. 대상은 `::race::` 테스트이고, 무엇을 보는지와 어느 자리를 잡는지는 아래 "메모리 순서" 절에 있다. 구독자는 프로세스처럼 각자 `ChannelShared`와 구독자 번호를 갖는다. Miri가 mmap·fcntl을 지원하지 않아 heap 경로만 돈다. claim이 0번 번호의 칸만 읽는 변형, 구독자가 borrow한 뒤 ticket을 다시 확인하지 않는 변형도 data race로 잡힌다. 툴체인은 fuzz crate가 고정한 nightly다.

```bash
scripts/miri.sh    # MIRI_SEEDS 기본 0..64, 약 25초
```

모델과 litmus는 `colcon test`(패키지 `flux_model_checking`)와, 덮는 파일이 바뀐 커밋의 pre-commit(flux_devel `.pre-commit-workspace.yaml`)에서 돈다. Miri는 Rust 소스와 함께 이 저장소에 두고 `colcon test`(패키지 `flux2_ffi`)와 pre-commit에서 돈다.

전역 lock 없는 sweep은 flux_model_checking의 `sweep/`이 검사한다. 이름(signpost, 세그먼트 epoch), inode, 프로세스마다 open file description과 OFD lock을 모델로 두고, sweeper·발행자·구독자 중 누구든 아무 단계에서나 죽을 수 있다. 살아 있는 객체를 가리키는 이름을 지우지 않는지, 쓰고 있는 이름이 다른 객체로 바뀌지 않는지, 다 끝나면 남는 이름이 없는지를 본다(설정마다 수백만 상태). 이름 확인이나 lock을 빼거나 순서를 바꾼 변형은 모두 잡힌다. `flux2_core/tests/sweep_stress.rs`는 프로세스 12개를 계속 만들고 SIGKILL·SIGSTOP하며 같은 성질을 실제 /dev/shm에서 본다.

loom은 쓰지 않는다. loom은 `SeqCst`를 AcqRel로 다뤄서, 올바른 `SeqCst` Dekker도 위반으로 보고한다.

### 다중 프로세스 시험

`flux2_core/tests/`의 셋이다. 시험 바이너리 자신을 역할마다 다시 실행해 워커 프로세스로 쓴다(`tests/common`). 프레임은 머리에 자기 ticket, 발행자 nonce, 길이, dtype, shape, 해시를 담는다. 구독자는 take할 때와 해제 직전에 이를 확인하므로, 찢어진 프레임, 쥔 동안 다시 쓰인 슬롯, 이전 프레임의 길이나 shape를 물려받은 재사용 슬롯이 그 자리에서 드러난다. 기본 길이는 `cargo test`에서 2초이고 환경 변수로 늘린다. 공유메모리를 직접 읽는 oracle(`fuzz::borrows`, `fuzz::lag`)은 `fuzz` feature 뒤에 있고, flux2_core가 자기 자신을 그 feature와 함께 dev-dependency로 받아 시험에서만 켠다.

| 시험 | 하는 일 | 길이 |
| --- | --- | --- |
| `op_fuzz` | 발행자 하나와 구독자 둘이 coordinator가 하나씩 보내는 연산 열을 돈다. 연산은 publish, loan, commit, 잘못된 commit, abort, 발행자 재시작, take, release, SIGKILL, SIGSTOP, SIGCONT다. 한 번에 하나라서 순차 모델이 모든 응답을 정확히 예측한다. publish의 ticket이나 backpressure, take의 ticket·`lost`·`refused.max_borrow`, 연산마다 공유메모리의 구독자 번호별 borrow 합을 모델과 대조한다. 죽인 구독자는 다음 연산 때 다시 띄워, 그 사이 claim이 주인 없는 번호의 borrow를 회수하게 한다. 실패한 열은 줄여서 `tests/op_fuzz/`에 저장하고, `fixed_sequences_match_the_model`이 그 파일을 모두 다시 돈다 | `FLUX2_OPFUZZ_SECS`, 첫 seed `FLUX2_OPFUZZ_SEED`(16진) |
| `torture` | 같은 세 역할이 쉬지 않고 돈다. coordinator는 무작위로 죽이고 다시 띄우고, 하나를 1~20 ms 멈추고, 주기마다 모두를 10 ms 멈추고(stutter), CPU affinity를 무작위 부분집합으로 바꾼다(shuffle). 해제 때 (그 세그먼트의 `latest` - view의 ticket)을 히스토그램으로 남긴다. 슬롯 수 이상이면 borrow가 없었을 때 다시 쓰였을 슬롯이고, 그 해제의 해시 확인이 덮어쓰기 판정이다. 죽이지 않는 keeper 구독자는 끝에 fd 수와 매핑한 flux2 객체가 open 전과 같은지 보고, 도는 동안 매핑한 객체가 현재 세그먼트, signpost, view가 쥔 옛 세그먼트를 넘지 않는지 본다. 마지막 sweep 뒤 /dev/shm에 채널 이름이 남으면 실패다 | `FLUX2_TORTURE_SECS`, `FLUX2_TORTURE_STUTTER_MS`(500), `FLUX2_TORTURE_SHUFFLE_MS`(1000) |
| `gpu_soak` | GPU 채널의 kill soak이다. 발행자는 loan을 `cuMemsetD8Async`로 ticket이 정하는 바이트와 길이로 채우고, 구독자는 `cuMemcpyDtoH`로 읽어 확인한다. keeper는 자기 프로세스의 device 메모리(nvidia-smi)가 open 전으로 돌아오는지 본다. 발행자가 연 device endpoint는 마지막 sweep 뒤 모두 없어야 한다. 길이를 지정한 실행은 살아 있는 어느 프로세스에도 잡히지 않는 device 메모리가 처음과 32 MiB 안인지도 본다. 다른 GPU 사용자가 이 값을 흔들어서 기본 실행에서는 출력만 한다. CUDA 장치가 없으면 바로 돌아온다 | `FLUX2_GPU_SOAK_SECS` |

oracle은 일부러 넣은 버그로 확인했다. `op_fuzz`는 넷을 모두 잡아 3~8 연산으로 줄였고, 그 열이 `tests/op_fuzz/`의 고정 시험이다.

| 넣은 버그 | 잡은 곳 |
| --- | --- |
| commit이 슬롯의 meta를 처음 쓸 때만 쓴다 | 재사용 슬롯의 프레임 확인(`stale_meta.ops`) |
| claim이 0번 번호의 칸만 읽는다 | 모델과 다른 publish 결과(`one_number_claim.ops`). `torture`에서는 "rewritten while held", TSan에서는 meta의 data race |
| 회수가 죽은 번호의 lock만 풀고 칸을 지우지 않는다 | 모델과 다른 backpressure(`dead_reader_reclaim.ops`) |
| 해제가 0번 슬롯의 칸을 내리지 않는다 | 번호별 borrow 합(`leaked_borrow.ops`) |
| 재attach가 옛 채널을 놓지 않는다 | `torture` keeper의 매핑 수, `gpu_soak` keeper의 device 메모리(182 → 662 MiB) |

`the_frame_check_finds_a_wrong_ticket_shape_length_or_byte`는 프레임 확인 자체가 틀린 ticket, 다른 shape, 잘린 길이, 뒤집힌 바이트를 잡는지 본다.

60초 실행 결과(i7-14700K, RTX 4070 SUPER, Linux 7.0, 2026-10-11)다.

| 시험 | 결과 |
| --- | --- |
| `op_fuzz` | 3709열, 연산 443398개, 받은 프레임 35185, backpressure 23324, 회수 582, 재attach 9804, kill 17670, 모두 모델과 같다 |
| `torture` | kill 573, stop 1630, stutter 119, shuffle 59, take 432197, 해제 때 lag가 슬롯 수 이상 17806, 위반 0 |
| `gpu_soak` | kill 432, endpoint 2100개 모두 정리, keeper device 메모리 182 → 최대 246 → 182 MiB, 잡히지 않는 device 메모리 928 → 937 MiB |

### 검출기 음성 시험

검출기마다 규칙을 일부러 어기고 검출을 기대하는 시험이다.

| 검출기 | 시험 |
| --- | --- |
| commit·해제 fence 실패, `refused.fence` | `gpu::tests`의 `a_failed_commit_fence_keeps_the_slot_and_is_counted`, `failed_release_fences_leak_their_borrows_until_the_take_is_refused_for_it` |
| `refused.max_borrow` | `tests/heap.rs` |
| `refused.bad_frame` | `channel::header`의 셋 |
| `refused.contended` | `a_take_out_of_retries_counts_contended`. 건너뛴 프레임도 재시도 하나를 쓰므로, 예산만큼 이어진 손상 프레임이 take를 contended로 끝낸다 |
| `refused.no_reader` | `an_inherited_subscription_takes_nothing_and_counts_no_reader`(fork한 자식의 take와 `take_blocking`) |
| `MSG_CTRUNC` | `a_descriptor_dropped_at_the_open_file_limit_is_named`. 커널은 fd를 하나도 넣지 못하면 control message를 쓰지 않는다. 그래서 fd를 꺼내기 전에 `MSG_CTRUNC`를 본다. 이 시험 전에는 그 경우가 "sent no descriptor"로 나갔다 |
| 죽은 endpoint·세그먼트·signpost sweep | `a_sweep_removes_only_an_endpoint_nothing_listens_on`, `tests/shm.rs`의 sweep 시험 |
| 같은 view의 두 번 해제(`misreleased`) | `channel::release_twice`의 둘. 이미 0인 칸의 감소와 다른 프레임이 든 슬롯의 해제를 거절하고 센다 |
| debug 검사 | `the_debug_checks_are_built_into_the_test_profile`. 슬롯 번호의 `debug_assert!`가 시험 빌드에서 panic하는지 본다. release 시험 빌드에서는 ignored로 보인다 |

### sanitizer

`scripts/sanitize.sh address` 또는 `thread`가 flux2_core의 프로세스 안 시험(lib, `heap`, `executor`)을 nightly(`nightly-2026-10-05`)의 `-Zsanitizer`로 돈다. std도 같은 방식으로 다시 빌드하고(`-Zbuild-std`), 빌드는 `target/<kind>`에 따로 둔다. ASan(LeakSanitizer 포함)과 TSan은 커널의 KASAN·KCSAN처럼 따로 돈다.

- 빼는 것: GPU 시험(드라이버가 계측되지 않고 ASan의 shadow 자리에 매핑한다), 한 주기의 시간을 재는 `publish_and_take_do_not_scan_every_slot`(계측이 상한을 넘긴다)
- `allocator_may_return_null=1`: 할당할 수 없을 만큼 큰 heap 채널을 일부러 요청하는 시험이 있다
- TSan은 단독 fence를 모델링하지 않는다. roster seqlock(`manifest.rs`)과 parker 겨냥 seqlock(`executor.rs`)은 `fence(Release)`·`fence(Acquire)`로 순서를 정하므로, 이 둘에서 나온 보고는 거짓이다. 이유와 함께 `scripts/sanitize/tsan.supp`에 넣는다. 지금은 보고가 없어 비어 있다
- TSan은 프로세스 사이 공유메모리를 보지 못한다. 그쪽은 위의 다중 프로세스 시험이 프레임 해시로 본다

결과(2026-10-11, i7-14700K): ASan과 TSan 모두 lib 31, `executor` 40, `heap` 15개가 통과했고 보고는 0이다. 음성 대조로 claim이 0번 번호의 칸만 읽게 하면 TSan이 `channel::race`에서 meta의 data race를 보고한다. take의 ticket 재확인만 빼면 보고가 나오지 않았다. 그 경쟁은 claim이 실제로 끼어들어야 생기고, 200 프레임 실행에서는 끼어들지 않았다.

### coverage

`cargo llvm-cov`로 퍼저가 실제로 도는 분기를 본다. root 없이 설치된다(`rustup component add llvm-tools-preview`, `cargo install cargo-llvm-cov`). 워커는 같은 시험 바이너리라 정상 종료한 워커의 profile도 합쳐진다. SIGKILL로 죽은 워커의 것은 빠진다.

```bash
FLUX2_OPFUZZ_SECS=60 FLUX2_TORTURE_SECS=60 cargo llvm-cov --no-report -p flux2_core --test op_fuzz --test torture
cargo llvm-cov report --text --output-path cov.txt
```

| 분기 | 60초 실행에서 지난 횟수 |
| --- | --- |
| `reclaim_dead`가 죽은 번호를 회수 | 281 |
| `maybe_reattach`의 재attach(epoch 변경) | 4960 |
| take의 ticket 재확인 실패 | 39 |
| `next_frame`의 전체 훑기(커서가 색인보다 뒤처짐) | 1800 |
| `try_claim`이 claim 뒤 borrow를 보고 되돌림(Dekker) | 1 |
| 재attach가 `NotReady`나 다른 오류로 거절됨 | 0 |

마지막 줄은 발행자가 새 epoch를 쓴 뒤 세그먼트가 열리지 않는 순간이 있어야 지난다. 지금의 시험은 그 순간을 만들지 않는다.

## 사후 분석

`panic = "abort"`라 core가 crash의 유일한 기록이다.

- 기본 `coredump_filter`(0x33)는 공유 파일 매핑(bit 3)을 빼서, core에 세그먼트 헤더와 슬롯 상태가 없다. debug 빌드(`debug_assertions`)는 첫 세그먼트를 매핑할 때 프로세스의 `coredump_filter`에 bit 3을 켠다. release 빌드는 바꾸지 않는다. 운영에서 같은 증거가 필요하면 프로세스를 띄우기 전에 `echo 0x3b > /proc/<pid>/coredump_filter`나 부모 셸에서 같은 값을 둔다(자식이 물려받는다). 커널은 이 값을 base 0으로 읽으므로 hex에는 `0x`가 필요하다
- payload는 어느 빌드에서든 `MADV_DONTDUMP`로 core에서 뺀다. 덤프 크기의 대부분이고 원인을 보여 주는 일은 드물다. 그 대신 bit 3을 켜도 core는 제어 영역만큼만 커진다
- 빌드마다 build-id와 debuginfo를 보관한다. Ubuntu의 링커는 기본으로 `.note.gnu.build-id`를 넣는다(`readelf -n libflux2_ffi.so`). debuginfod나 빌드 산출물 보관으로 core의 build-id에서 심볼을 찾는다
- core를 받는 곳은 `/proc/sys/kernel/core_pattern`(Ubuntu는 apport, systemd 배포는 systemd-coredump)과 `ulimit -c`가 정한다. RT 배포는 systemd-coredump `ProcessSizeMax`로 크기를 제한한다. 덤프가 끝날 때까지 OFD lock이 풀리지 않으므로 발행자 생존 판정이 그만큼 늦는다(CUDA 프로세스는 수 초)

## 발행자

- 토픽과 fingerprint 조합당 발행자 하나. 두 번째 발행자는 생성 시 거절한다. fingerprint가 다르면 이름이 다른 별개 채널이라, 같은 토픽에 타입 없는 발행자와 타입 있는 발행자가 공존하고 구독은 자기 fingerprint의 채널만 받는다. ROS 2(DDS)도 같은 이름에 타입이 다른 발행자가 공존하고 타입이 맞는 쪽끼리만 연결된다
- 발행자 재시작은 새 epoch 세그먼트를 만든다. 쓰다 죽은 슬롯의 ticket이 `TICKET_CLAIMED`로 남으므로 이전 세그먼트를 이어 쓰지 않는다

## 스레드

- Publisher나 Subscription 하나를 두 스레드가 동시에 쓰면 거절한다
  - C ABI 핸들: publish·loan·take·set_qos가 겹치면 `FLUX2_PUBLISHED_IN_USE`, `FLUX2_IN_USE`, 빈 결과를 돌려준다
  - C++·Python 구독: `std::logic_error`(Python은 `RuntimeError`)를 던진다. 재attach가 다른 스레드가 쓰는 매핑을 해제하기 때문이다
- 차례로 쓰는 것은 허용한다. rclcpp와 rclpy의 MultiThreadedExecutor는 콜백을 매번 다른 스레드에서 돌린다
- executor가 serve하는 구독은 사용자가 take할 수 없다. 둘이 프레임을 나눠 갖기 때문이다
- 여러 스레드가 모든 프레임을 받으려면 스레드마다 구독을 만든다
- 막지 않는 것
  - loan의 commit. 한 Publisher의 loan 둘을 두 스레드가 동시에 commit하면 `latest`가 잠깐 뒤로 갈 수 있다. commit이 부르는 죽은 번호 회수는 `ChannelShared`의 mutex로 한 번에 하나만 돈다. OFD lock은 같은 open file description을 쓰는 스레드끼리 막지 않아서, 회수 둘이 같은 번호를 잡으면 한쪽의 unlock 뒤에 그 번호를 받은 새 구독자의 칸을 다른 쪽이 0으로 지운다
  - C++·Python 구독의 카운터 읽기(`lost`, `refused`, `attach_generation`, `attached`). take하는 스레드가 매번 atomic 사본을 남기고 getter는 그것만 읽으므로, 재attach가 채널을 바꾸는 중에도 어느 스레드에서든 읽을 수 있다. C ABI getter는 검사하지 않으므로 그 채널의 다른 호출과 겹치지 않게 부른다

## lock

데이터 경로의 lock과 그 순서다. 순서 규칙은 하나다. GIL을 쥔 채 flux lock을 잡을 수는 있지만, flux lock을 쥔 채 GIL을 기다리거나 GPU 작업(`cuEventSynchronize`, stream 대기)을 기다리지 않는다. CUDA 동기 대기와 blocking OFD lock도 lock으로 친다. 이런 교착은 TSan도 lockdep류 검사도 보지 못하므로 아래 표가 정본이다.

| lock | 종류 | 지키는 것 | 잡는 스레드 | 쥔 채 하는 일 |
| --- | --- | --- | --- | --- |
| fork lock(`sys::REGISTRY`) | PI futex(`sys::PiMutex`) | 프로세스 전역 표. `fork()`가 prepare에서 잡고 fork가 끝날 때까지 쥔다 | `subscribe`, 재attach(executor 스레드의 take), `other_layout`, fork하는 스레드 | 아래 둘, `open`·`fstat`·`mmap`, 구독자 번호 찾기(다 잠겼으면 100 us씩 최대 10 ms) |
| 구독자 번호·매핑 표(`reader::REGISTRY`), `/dev/shm` 목록(`LISTING`) | `std::sync::Mutex` | lease, 매핑 `Weak`, 목록 | fork lock 아래에서만. 그래서 경합하지 않는다 | 위와 같다 |
| live fd(`sys::LIVE`) | PI futex | 생존 lock fd 목록. fork prepare가 fork lock 다음에 잡는다 | fd를 열고 닫는 모든 곳 | `open`·`close` 한 번 |
| `Segment::device_init` | `std::sync::Mutex` | 매핑마다 한 번 하는 호스트 등록·import | 그 매핑의 첫 GPU attach | `cuMemHostRegister`(256 MB에 수십 ms), fd 교환(최대 1 s), VMM import. GPU 작업 대기는 아니다 |
| `ChannelShared::reclaiming` | PI futex | 죽은 번호 회수 | 발행자(commit이 1초에 한 번, claim이 굶을 때) | 번호마다 OFD lock 시도 |
| `ChannelShared::late` | PI futex | 늦춘 해제와 시간을 넘긴 fence | 해제하는 아무 스레드(push), 채널 스레드(꺼내고 되돌림) | `Vec` 조작만. 끝났는지 묻는 `cuEventQuery`는 lock을 놓고 한다 |
| `ChannelShared::release_waker`, `gpu_waker` | PI futex | 깨울 executor의 `Weak` | 해제하는 스레드, executor | `Weak` 복사 |
| GPU 대기 스레드 큐(`Waiter::queue`) | `std::sync::Mutex` + `Condvar` | 표시 event 큐 | 표시하는 스레드, `flux2-gpu-wait` | push·pop. `cuEventSynchronize`는 lock 밖이다 |
| stamp offset 표(`flux2_ffi`) | `std::sync::Mutex` | fingerprint별 offset | 생성 코드 적재, 채널 생성 | 표 조회 |
| `PartitionedExecutor` 실패(core), C++ `PartitionedExecutor::m_` | `std::sync::Mutex`, `std::mutex` | 첫 오류, 등록 상태 | 파티션 스레드, 호출 스레드 | 값 기록 |
| Python `Executor::m_`(claim), `Turnstile::m_` | `std::mutex` + `condition_variable` | 어느 스레드가 pass를 도는지, GIL 넘겨주기 | bridge·post·spin 스레드, `serve()` 스레드 | 깃발 조작. 기다리는 동안은 `condition_variable`이 놓는다. GIL은 이 mutex를 놓은 뒤에 다시 잡는다 |
| GIL | CPython | 인터프리터 | Python 층 | 콜백. 대기(`take_blocking`, `spin`, stream·fence)는 GIL을 놓고 한다 |

- 겹쳐 잡는 것은 "fork lock → 구독자 표 또는 `LISTING` → live fd" 한 줄이고, `fork_prepare`가 같은 순서로 잡는다. 나머지 flux lock은 쥔 채 다른 lock을 잡지 않는다. 그래서 debug 빌드의 lock 순위 검사는 두지 않았다. 검사할 순서쌍이 하나뿐인데 lock을 잡는 자리 열 곳 남짓을 감싸야 하고, 정작 위험한 GIL 순서는 Rust 쪽 검사가 볼 수 없기 때문이다
- 확인한 것: flux lock을 쥔 채 GIL이나 GPU 작업을 기다리는 곳은 없다. 마지막으로 남았던 `late`를 쥔 `cuEventQuery`도 lock 밖으로 옮겼다. `device_init`은 CUDA API(등록, import)와 fd 교환을 쥔 채 하지만, attach마다 한 번이고 GPU 작업을 기다리지 않는다
- Rust std `Mutex`, `RwLock`, `Condvar`, glibc `pthread_mutex_t`(기본 속성)와 condvar에는 우선순위 상속(PI)이 없다. 낮은 우선순위 스레드가 쥔 lock을 RT 스레드가 기다리면 그 사이 중간 우선순위 스레드가 RT 스레드를 무한히 밀어낼 수 있다. 그래서 RT executor 스레드가 기다릴 수 있는 lock은 PI로 둔다
  - fork lock과 live fd: 재attach가 executor 스레드의 take 안에서 잡는다. 다른 SCHED_OTHER 스레드가 구독을 열며 번호를 찾는 동안(최대 10 ms) 쥘 수 있다
  - `late`, `release_waker`, `gpu_waker`: 해제는 어느 스레드에서든 하므로(Python 늦춘 해제, 다른 스레드로 옮긴 view) 쥐는 쪽이 RT가 아닐 수 있다
  - `reclaiming`: commit을 여러 스레드가 할 때
- `sys::PiMutex`는 PI futex를 직접 쓴다. word는 0이거나 쥔 스레드의 tid이고, 경합하면 `FUTEX_LOCK_PI`가 기다리는 쪽을 우선순위 순으로 줄 세우고 쥔 쪽을 올린다. 풀 때 `FUTEX_WAITERS`가 있으면 `FUTEX_UNLOCK_PI`가 가장 높은 대기자에게 넘긴다. 비경합 lock과 unlock은 compare-exchange 하나씩이다(tid는 스레드마다 한 번 `gettid`로 읽어 둔다). lock+unlock 11 ns로 std `Mutex`와 같고, heap 채널 publish+take+release도 바꾸기 전과 차이가 없다(부하가 걸린 Ryzen에서 8회 교대로 잰 중앙값 112.5 → 108.5 ns, 이 lock들은 그 경로에 없다)
  - pthread `PTHREAD_PRIO_INHERIT`가 아닌 이유: 정적 초기화 값이 없고, fork한 자식에서 prepare가 잡은 PI mutex는 tid가 달라 풀 수 없다. `fork_child`는 자기 스레드의 tid 사본을 지우고 fork lock과 live fd의 word를 0으로 되돌린다. 자식에는 그 스레드 하나뿐이라 기다리는 쪽이 없다
  - poison이 없다. `panic = "abort"`라 의미가 없다
  - guard는 `Send`가 아니다. 커널이 기록한 주인 스레드만 풀 수 있기 때문이다
  - 시험: 경합한 word에 쥔 스레드의 tid와 `FUTEX_WAITERS`가 들어 있는지 본다(권한 없이 돈다). SCHED_FIFO가 허용되면 FIFO 3 스레드가 기다리는 동안 FIFO 1인 쥔 쪽의 실효 우선순위가 3이 되는지도 본다. 이 x86 호스트는 `RLIMIT_RTPRIO` 0이라 그 시험은 건너뛴다

## 신뢰

- 같은 uid의 프로세스는 믿는다. 공유메모리를 일부러 고치는 피어는 막지 않는다. iceoryx2, zenoh, agnocast도 같은 전제다
- signpost와 세그먼트는 이 euid가 소유한 것만 연다(mmap 전에 `fstat`의 `st_uid`). 열거와 통계는 다른 사용자의 것을 건너뛴다. 연 뒤에 소유자가 파일을 줄여 생기는 SIGBUS는 막지 않는다. `sweep_dead`는 이 uid의 객체만 지운다(iceoryx2, zenoh도 자기 uid만). 세그먼트 이름이 이미 있으면 다음 epoch를 무작위로 뽑아 선점을 피한다. signpost 이름은 그대로 예측할 수 있다
- 다만 버그 있는 피어나 손상된 프레임 때문에 받는 쪽이 죽거나 프레임 밖을 읽지 않게 한다
  - `take`는 슬롯 헤더를 복사해 검사한다. dtype이 알려진 값인지, `itemsize`가 dtype 크기인지, shape 곱이 `nbytes`인지, `nbytes`가 슬롯 안인지 본다. 어긋나면 `refused.bad_frame`으로 세고 그 프레임을 건너뛴다. 커서는 넘어가므로 같은 프레임을 다시 세지 않는다
  - `FrameView`는 검사한 복사본을 들고 다닌다. `flux2_view_meta`는 view 저장소 안을 가리키므로 view를 옮기면 함께 옮겨진다
  - descriptor로 읽은 바이트의 합이 프레임 크기를 넘으면 거절한다. writer는 같은 바이트를 두 번 가리키지 않으므로 디코딩 크기가 프레임 크기에 묶인다. CDR 변환(bridge), 생성된 C++·Python `frame_to_msg`(프레임 하나에 예산 하나), Python `Reader.strings`, rviz PointCloud2의 필드 이름이 이렇게 한다. 원소를 만드는 복사는 원소 안 고정 배열(`string[N]`, `T[N]`)이 요구하는 바이트까지 먼저 확인한다. 그 배열은 프레임에서 descriptor 하나지만 만들 때는 N개가 생겨서, stride만 세면 짧은 프레임이 프레임보다 훨씬 큰 메시지를 만들게 한다
  - rviz PointCloud2는 width × height × point_step을 곱하지 않고 나눗셈으로 비교해 wrap을 막는다
  - bool 바이트는 0이 아니면 true다(C++, Python, bridge 모두)
- 공유메모리에서 읽는 값은 상대가 만든 입력으로 다룬다
  - attach는 헤더의 일반 필드(magic, version, 슬롯 크기·수, `max_subscribers`, storage kind, fingerprint, 장치 UUID)를 한 번씩 volatile로 읽어 `HeaderConfig`에 복사하고, 검사와 layout 계산과 그 뒤의 모든 사용(`host_payload`, 공유 매핑의 fingerprint 재확인, UUID로 장치 찾기, `read_channel_stats`)이 그 복사본을 쓴다. 전에는 범위를 검사한 필드를 layout을 만들 때 다시 읽었고, storage kind는 view가 데이터를 낼 때마다 다시 읽었다. 장치 payload 세그먼트(제어 영역만 매핑한다)에서 상대가 storage kind를 host로 바꾸면 view가 매핑 밖을 가리켰다(시험 `a_storage_kind_rewritten_after_attach_is_not_believed`)
  - offset은 `layout_config_ok`로 범위를 묶은 값에서만 나온다. 가장 넓은 layout의 크기가 2^50 아래임을 `layout.rs`가 const assert로 고정하므로 그 범위 안의 산술은 wrap하지 않는다. 그래서 checked 산술을 따로 두지 않는다
  - 상대가 쓰는 `latest`에서 나오는 `lost` 증가는 포화 덧셈이다. 재attach를 거치며 쌓이면 debug 빌드의 overflow 검사가 abort하기 때문이다. `readers_high`는 `max_subscribers`로, roster의 길이 값은 버퍼 크기로 자른다. key 길이가 레코드보다 길면 key를 버린다
  - 세그먼트 매핑의 시작이 cache line에 맞는지 attach에서 본다. 공유 구조체의 atomic은 모두 그 시작에서 자기 크기의 배수 offset에 있다(`#[repr(C)]` 필드와 cache line에 맞춘 파생 영역, const assert). 정렬이 어긋난 atomic은 aarch64에서 SIGBUS, x86에서 split lock이다. mmap은 페이지 단위라 실제로는 실패하지 않는다
  - 공유메모리를 읽는 모듈(`channel`, `segment`, `discovery`, `manifest`, `reader`, `layout`)은 clippy `unwrap_used`, `expect_used`, `panic`, `indexing_slicing`을 deny로 둔다(시험 코드는 제외). 구독자 번호와 슬롯 번호로 칸을 찾는 것은 slice 색인 대신 `Segment::reader`·`waiter`가 한다. 두 값은 이 프로세스가 정한 것이고 debug assert로 확인한다
  - 헤더를 아직 `&ControlHeader`로 본다. 일반 필드는 위 복사본으로만 읽고 참조로 읽지 않는다
  - bridge와 bag 녹화는 ROS 토픽 이름이 아닌 key를 경고 한 번과 함께 건너뛴다. 로그에는 검사를 통과한 key만 찍는다
- view 해제가 프로토콜이 만들 수 없는 감소이면 거절한다. C 호출자가 `flux2_view` 저장소를 복사해 두 번 해제하는 경우다. 커널 kref처럼 use-after-free 대신 누수를 고른다
  - 감소 전에 슬롯의 ticket을 view의 ticket과 비교한다(Relaxed load 하나). borrow가 서 있는 동안 claim은 성공하지 못하므로 다른 ticket이면 그 borrow는 이미 돌아간 것이다. 같은 구독자가 슬롯을 다시 borrow한 뒤 옛 view를 해제하면 0 검사로는 못 잡고 새 borrow가 일찍 풀리는데, 이것을 잡는다. claim이 실패하기 직전의 `TICKET_CLAIMED`는 정상으로 본다. epoch는 따로 담지 않는다. view가 자기 세그먼트를 `Arc`로 쥐고 그 세그먼트에 해제하기 때문이다
  - 감소가 돌려준 이전 값에 비교 하나를 더한다. 0이었거나 2^31 이상이면 감소를 되돌린다. 0이었으면 1로 남겨 그 슬롯을 세그먼트가 사라질 때까지(발행자 재시작이나 이 프로세스의 종료와 회수) 쥔다. 2^31 이상은 이미 claim되지 않는다
  - 상한을 `max_borrow`로 두지 않는다. 한 프로세스의 구독들이 번호 하나를 같이 쓰므로 칸 값은 그 구독들의 `max_borrow` 합까지 정상이다
  - 거절한 해제는 `outstanding`도 바꾸지 않는다. `Channel::misreleased()`(C ABI `flux2_channel_misreleased`, 구조체 배치가 그대로라 ABI 버전은 그대로다)로 세고, 프로세스에서 처음 한 번만 stderr에 슬롯·ticket·이유를 남긴다
  - 비용: heap 채널 publish+take+release 66.2 → 66.8 ns(Ryzen 5 9600X, CPU 고정, 9회 중앙값을 세 번), 잡음 안이다
  - 바이트 복사한 view 둘이 같은 `ChannelShared`의 `Arc`를 두 번 놓는 것은 막지 못한다. 칸 검사는 그 `Arc`가 아직 살아 있을 때만 돈다
- 남은 것: C++ `bool[N]` View 접근자(원시 바이트를 `bool`로 본다)

## loan 수명

- commit 뒤 loan과 Builder는 슬롯에 닿지 않는다. Python `loan.array`는 읽기 전용이 되고, Builder의 쓰기는 버려지고, 두 번째 commit은 실패한다
- commit 전에 꺼낸 조각(numpy slice, memoryview, `alloc__` 배열, C++ `data()` 포인터)은 막지 않는다. iceoryx2, zenoh, agnocast와 같은 수준이다
- loan 저장소는 commit 뒤에도 매핑(`Mapped`)을 쥐고, abort하거나 버릴 때 놓는다. Python 배열은 Loan을 붙잡으므로 발행자를 지운 뒤에도 읽을 수 있다
- 매핑과 발행자 신원(발행자 락, 이름 unlink)은 따로 산다. 배열이 남아도 발행자를 지우면 신원은 정리되고, 같은 토픽에 새 발행자를 만들 수 있다

## 알려진 한계

고치지 않기로 한 것이다. 고치는 대가가 얻는 것보다 크거나, 다른 구현도 같은 수준이다.

| 한계 | 이유 |
| --- | --- |
| executor 아래에서 발행자 재시작을 다음 tick(100 ms)에 알아채, 그 사이 프레임(30 Hz 영상이면 약 3개, 고속이면 수십 개)을 잃는다 | source마다 대기 워드를 하나 더 두면 executor 하나의 source 한도가 127에서 63으로 준다. iceoryx2도 새 발행자를 다음 `receive`에서 안다 |
| `take_blocking`은 ROS 종료를 최대 100 ms 뒤에 알아챈다 | 바로 깨우려면 막힌 take를 깨우는 C ABI가 더 필요하다. 끝나는 순간의 지연이라 영향이 없다 |
| 잘못된 Builder의 commit은 `TOO_LARGE`를 돌려주고 loan을 유지한다. Python `loan()`은 슬롯보다 크면 예외, C++은 빈 `WriteSlot`이다 | 두 언어의 Builder는 같다. `loan()`을 맞추려면 C++ API가 바뀐다 |
| 생성된 Python setter는 정수만 범위를 검사한다. 실수 필드에 `None`을 넣으면 nan이 된다 | 입력 실수이고 프레임이나 받는 쪽을 깨지 않는다 |
| signpost 이름은 예측할 수 있다. 다른 사용자가 먼저 만들면 발행자 생성이 소유자를 밝힌 에러로 실패한다 | iceoryx2, zenoh도 서비스 이름을 예측할 수 있다. 세그먼트 이름은 선점되면 무작위 epoch로 피한다 |
| roster는 62칸이다. 넘는 endpoint는 목록에만 빠진다 | signpost는 발행자보다 먼저 생기고 재시작에도 남아, 만들 때 `max_subscribers`를 모른다 |

## Humble

Humble(Ubuntu 22.04, rclcpp 16, rosbag2 0.15, rviz_common 11, Python 3.10, numpy 1.21)에서 빌드하고 시험한다. 차이는 배포판 이름이 아니라 버전으로 고른다(`RCLCPP_VERSION_MAJOR`, rosbag2_transport의 minor 버전인 `FLUX2_ROSBAG2_MINOR`, `rviz_common_VERSION`, 헤더 유무). Jazzy·Lyrical의 동작은 그대로다.

| Humble의 차이 | flux2가 하는 것 |
| --- | --- |
| `rclcpp::Executor::cancel()`이 virtual이 아니고 `spin_until_future_complete_impl`이 없다(rclcpp 28 미만) | `flux2::Executor`가 `cancel()`과 `spin_until_future_complete()`를 자기 타입에 둔다. `rclcpp::Executor &`로 부르면 rclcpp의 것이 돈다. 그때 `spin_until_future_complete()`는 `spin_once` 미지원 예외를 던진다. context shutdown은 hook으로 어느 쪽이든 멈춘다 |
| `TimerBase::get_clock()`이 없다 | protected `clock_`을 멤버 포인터로 읽는다 |
| bag 메시지의 시각이 `time_stamp` 하나다 | 녹화는 recv 시각(take 시각)만 bag에 쓴다. commit 시각은 FluxFrame header stamp에 그대로 있다. 재생·변환은 그 하나를 쓴다 |
| `TopicFilter`를 설치하지 않는다 | 녹화가 0.15의 선택 규칙(토픽 목록, `-e`, `-x`, 숨은 토픽)을 그대로 따른다. `--exclude-topics`는 0.26부터라 거절한다 |
| 녹화기에 서비스가 없다 | `~/pause`·`~/resume`으로는 멈출 수 없다. 녹화기가 멈춘 동안(`is_paused()`)의 프레임을 쓰지 않는 규칙은 같다 |
| `Player::play()`가 bag 끝까지 막고, 재생 hook, `stop()`, `exclude_topics_to_filter`가 없다 | `play()`를 자기 스레드에서 돌리고 시작은 protected 준비 상태로 기다린다. ROS 토픽은 포함 목록으로 고르고, ROS 토픽이 없으면 어느 토픽과도 맞지 않는 이름 하나를 넘긴다. 재생 루프가 `rclcpp::ok()`만 보므로 멈출 때 context를 shutdown한다 |
| 재생한 메시지를 알려 주지 않는다 | `play_next`·`burst`가 ROS 토픽을 옮겨도 flux2 시계는 따라가지 않는다. ROS 메시지가 다 떨어진 뒤에 flux2 프레임을 하나씩 옮긴다. pause·resume·rate는 Jazzy와 같다 |
| rclcpp·rclpy `EventsExecutor`, gtest `GTEST_FLAG_SET`이 없다 | 그 executor를 쓰는 시험 조합만 빠진다 |

## 뺄 것

- 다중 발행자
  - writer 신원 각인(`writer_pid`, `writer_starttime`, `writer_stamp`)
  - 죽은 writer claim 회수(`recover_stuck_writes`)
  - `latest` CAS-max -> store
  - signpost write 락 bootstrap 경쟁, join 경로, `try_join`
  - `multiwriter` 하니스
- 예약 필드: `epoch`(ControlHeader), `kInitCleaning`, `StorageKind::DmaBuf`, `flags`
- `peek`(필요해지면 추가)
- owner 파일, `(pid, starttime)` 신원, holder 테이블, 공용 `refcount`

## 결정

| 항목 | 결정 |
| --- | --- |
| 연구 범위 | flux와 같다 |
| 커널 | 5.16 이상은 merged wait(`futex_waitv`), 그 아래는 parker 스레드 fallback이다. 그래서 Humble의 Ubuntu 22.04는 GA 커널(5.15)과 HWE 커널(6.8) 모두에서 돌고, Jetson JetPack 6(L4T 36, 5.15)도 fallback으로 돈다. 처음에는 io_uring `FUTEX_WAIT`(6.7)였다 |
| ROS 배포판 | Humble, Jazzy, Lyrical. Kilted는 지원하지 않는다(rviz_common 15.0의 `Display::update`가 Lyrical과 달라 flux2_tools가 빌드되지 않는다). Humble 차이는 아래 "Humble" 절 |
| `flux_bag`, `flux_bridge`, `flux_tools`, `flux_std_adapters` | flux2용으로 다시 만들었다(`flux2_bag`, `flux2_bridge`, `flux2_tools`, `flux2_std_adapters`). 채널 열거는 C++·Python 모두 `enumerate_topics`·`read_channel_stats`로 한다 |
| tracing(LTTng) | armo가 맡는다. flux2는 trace 함수만 노출한다 |
| C++ | `flux2::` 네임스페이스, C++17(Jazzy 기준). flux(`flux::`)와 한 프로세스에 같이 링크할 수 있다 |
| 콜백 예외 | C++ 콜백의 예외는 C ABI를 넘지 않는다. 래퍼가 잡아 두고 그 콜백을 부른 호출(`dispatch`, `spin`)에서 다시 던진다 |
| flux2_gen | flux_gen을 옮겼고 Rust로 다시 썼다. cargo가 빌드하고 `lib/flux2_gen/flux2_gen`에 설치한다. fingerprint와 layout(scalar 크기, offset, stride, align)은 Python 판과 같음을 설치된 .msg 214종과 시험 .msg로 대조했다. 생성 코드의 글자는 이후 고친 것들로 달라졌다. 생성물은 `<pkg>/flux2/<msg>.hpp`(`<pkg>::flux2_msg`)와 `<pkg>_flux2` 모듈이라 flux 어댑터와 한 워크스페이스에 공존한다. Python 모듈은 `flux2.wire`(flux2_py)를 import한다. flux2_gen은 의존이 없으므로 flux2_cpp·flux2_py의 테스트 의존이 순환하지 않는다. 생성된 코드의 시험(`test_emit`)은 런타임이 있는 flux2_py에 둔다. 해석 규칙은 rosidl을 따른다. 파싱되지 않는 .msg는 그것을 읽는 메시지만 실패시키고, 상수는 rosidl의 literal·범위·따옴표 규칙을 따르고, `@optional`과 `T[N>65536]`은 그 메시지만 거절한다. 줄은 Python `splitlines()`처럼 나눈다. rosidl처럼 탭은 공백으로 바꾸고, 줄은 첫 `#`에서 끊는다. 따옴표 안의 `#`도 마찬가지라 상수 값이 거기서 끝난다. 이름만 쓴 `Header`·`Time`은 builtin 타입으로 풀릴 때만 frame stamp다. 문자열은 첫 NUL에서 끊는다(rclpy와 같다). C++ 이름은 끝에 `_`를 붙여 피하고 Python은 Python 키워드만 피한다. 키워드 이름의 메시지 모듈은 `<snake>_.py`다. 메시지 이름 `View`, `Builder`, 원소 클래스 이름은 거절한다. `flux2_generate_adapters()`는 `<pkg>_flux2_adapters` INTERFACE 라이브러리를 만들고, 같은 패키지의 target은 이를 링크해 자기 adapter를 include한다. ROS bridge 쌍은 그보다 먼저 `rosidl_generate_interfaces()`에 넘긴 메시지에만 만든다. 두 메시지가 같은 파일을 쓰면 configure 에러다 |
| flux2_bridge | flux_bridge를 옮겼다. relay는 C++ 노드다. adapter를 찾지 않는다. 채널의 fingerprint로 설치된 .msg에서 타입과 layout을 찾고(`flux2_schemas_*`), 프레임을 그 메시지의 CDR로 바로 써서 `GenericPublisher`로 낸다. 그래서 adapter를 설치하지 않은 타입도 relay되고, 메시지 객체를 거치지 않는다. CDR은 rmw가 같은 메시지에서 쓰는 바이트와 같음을 표준 9종으로 시험한다(padding은 0으로 쓴다). domain이 없으니 key가 정확하고 발행자가 살아 있는 채널만 relay한다. 한 이름에 설치된 타입의 채널이 둘이면 signpost 순서로 첫 것을 고른다(`echo_flux2`도 같다). 같은 이름이 다른 fingerprint로 돌아오면 relay를 다시 만든다. 프레임은 복사한 뒤 view를 놓고 CDR 크기 계산과 쓰기를 복사본으로 한다. `--poll`은 유한한 양수다(`--poll=SEC`, `-h` 지원). verb는 flux와 겹치지 않게 `echo_flux2`·`hz_flux2`다 |
| flux2_bag | flux_bag을 옮겼다. 프레임은 `<channel>/flux2` 토픽에 `flux2_msgs/msg/FluxFrame`으로 담는다. 필드와 CDR 인코딩은 flux의 `FluxFrame`과 같고 타입 이름만 다르다. 메시지를 flux2_bag 안에서 생성하지 않고 flux2_msgs로 나눈 것은 rosidl이 만드는 Python 패키지 `flux2_bag`이 verb 패키지와 이름이 겹치기 때문이다. 녹화는 signpost마다 하고 생존은 세그먼트로 본다. 10초 넘게 내려간 채널은 놓는다. 채널 목록은 `/dev/shm`에서 flux2 이름이 생기거나 크기가 바뀌거나 지워지면 inotify로 곧바로 다시 읽고, 발행자는 마지막 사건 뒤에 epoch와 key를 쓰므로 그 뒤 500 ms 동안은 10 ms마다 다시 읽는다. `--poll`은 그 밖의 주기이고, 10초는 채널이 내려간 것을 처음 본 목록부터 잰다. `~/pause` 동안의 프레임은 rosbag2처럼 take하되 bag에 쓰지 않는다. 채널 스레드는 프레임을 자기 버퍼로 복사해 슬롯을 곧바로 돌려주고 쓰기 큐에 넣는다. bag은 스레드 하나가 쓴다. 디스크가 느린 동안 프레임은 큐에서 기다리고, 큐는 `--max-cache-size`(기본 1 GiB)를 넘지 않는다. 넘는 프레임은 버리고 `record_drop`의 `RECORD_CACHE_FULL`로 남긴다. 받기 전에 채널이 덮어쓴 프레임은 `RECORD_OVERRUN`이고, 쓰기에 실패한 프레임은 `RECORD_WRITE_FAILED`다. 다 쓴 버퍼는 다음 프레임에 다시 쓴다. 프레임 크기의 새 버퍼는 페이지마다 page fault가 나서 프레임이 오는 속도보다 느리기 때문이다. 측정(Orin, NVMe 약 0.7 GB/s, full HD 6.2 MB 프레임 100개를 5 ms 간격, 슬롯 8개): 큐 전 8~21% 손실, 큐 뒤 0. 같은 프레임을 메모리 복사 속도(0.5 ms 간격)로 연달아 내면 받는 쪽도 프레임마다 복사를 하므로 큐로도 다 받지 못한다. 슬롯 크기와 수는 attach 세대마다 다시 읽는다. send 시각과 header stamp는 commit 시각, recv 시각은 take 시각이다. 재생은 시작 전에 발행자를 만들고 2 tick(200 ms) 기다린다. 재생할 FluxFrame은 별도 스레드가 최대 256 MiB 미리 읽고, ROS 토픽과 flux2 채널은 함께 시작하는 두 시계를 따르며 플레이어의 pause·resume·set_rate·play_next·burst(서비스와 키보드)가 두 시계를 같이 움직인다. Jazzy의 seek는 virtual이 아니라서 ROS 토픽만 옮긴다. 프레임의 geometry가 바뀌면 발행자를 다시 만들고, dtype × shape가 데이터 크기와 다른 프레임은 건너뛴다. `-r`은 유한한 양수다. 채널이 구독이나 발행자를 거절하면 그 채널만 빠진다. writer 같은 나머지 예외는 bag을 닫은 뒤 exit 1로 끝낸다. 변환(`convert_flux2 BAG -o OUT`)은 오프라인으로 bag을 새 bag에 옮긴다. FluxFrame은 fingerprint가 가리키는 설치된 타입이 있으면 bridge와 같은 스키마 CDR 변환으로 그 ROS 메시지가 되어 채널 자체 토픽에 녹화 시각 그대로 들어가고, 없거나 같은 이름의 ROS 토픽이 다른 타입이면 FluxFrame으로 남는다. 나머지 토픽은 그대로 복사한다. 저장 형식은 rosbag2 기본(Jazzy는 mcap, CDR)이다. 변환, inotify 감지, 재생 미리 읽기, 키보드 조작은 사용자가 유지하기로 확인했다(2026-10-09) |
| flux2_tools | flux_tools를 옮겼다. adapter는 직접 생성하지 않고 flux2_std_adapters 헤더를 쓴다. rviz_common 15(Lyrical)는 Qt6이고 `Display::update`가 `std::chrono::nanoseconds`를 받는다. 14(Jazzy)와 11(Humble)은 Qt5이고 float 초를 받는다. CMake가 찾은 버전으로 둘을 고른다. Qt는 직접 찾지 않고 rviz_common이 빌드된 Qt를 쓴다. display 셋은 `ChannelDisplay`를 공유하고, attach 실패는 Topic 상태에 띄운다 |
| 노드 단위 적용 | 나중에 한다. 지금은 기존 rclcpp/rclpy 노드에서 토픽 단위로 opt-in한다. DDS participant 비용까지 없애는 flux2 전용 노드(agnocast의 `agnocast::Node`에 해당)는 그때 설계한다 |
| domain id | 없다. 이름에 넣지 않고 `FLUX_DOMAIN`·`ROS_DOMAIN_ID`도 읽지 않는다. 한 호스트의 같은 key·fingerprint는 모두 한 채널이다 |
| shm 크기 | 세그먼트와 signpost는 `fallocate`로 만든다. `/dev/shm`이 모자라면 생성이 ENOSPC로 실패하고, 쓰다가 SIGBUS로 죽지 않는다. 대신 publisher가 슬롯 전체 메모리를 생성 시점에 차지한다. C++ 기본 슬롯은 8 MiB × 16 |
| 외부 이벤트 루프용 fd | 노출하지 않는다. rclpy wait set이 임의 fd를 받지 않아 fd가 있어도 bridge 스레드가 필요하다. asyncio 같은 사용처가 생기면 추가한다 |
