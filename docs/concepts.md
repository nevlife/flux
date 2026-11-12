# flux 개념

C++와 Python API에 공통인 규칙이다. 언어별 사용법은 [cpp.md](cpp.md)와 [python.md](python.md)에 있다.

## 채널

채널은 한 호스트 안에서 토픽 이름과 fingerprint로 정해진다. 발행자와 구독자는 두 값이 모두 같아야 연결된다.

| 요소 | 뜻 |
| --- | --- |
| 토픽 이름 | 노드의 namespace와 remap을 적용한 이름. `/robot1` namespace에서 `Publisher(node, "image")`는 `/robot1/image`다 |
| fingerprint | 메시지 스키마의 64-bit 해시. `flux_gen`이 만든 어댑터의 `kFingerprint`(Python은 `FINGERPRINT__`)를 넘긴다. 타입 없이 배열을 보내면 `kNoSchema`(0)다 |

- ROS 2처럼 한 이름에 fingerprint가 다른 채널이 공존 가능하다.
- flux는 `ROS_DOMAIN_ID`를 읽지 않는다. domain이 달라도 같은 채널이다.
- 이름은 185자까지이고 `A-Z a-z 0-9 _ /`만 사용 가능하다. 다른 문자가 있으면 생성자가 `INVALID_ARGUMENT`를 낸다.

채널은 `/dev/shm`에 파일 두 종류를 만든다. 이름의 `/`는 `.`으로 바뀐다. 죽은 프로세스가 남긴 파일은 다음에 시작하는 flux 프로세스가 지운다.

| 파일 | 이름 | 개수 |
| --- | --- | --- |
| signpost | `flux.v<버전>.<이름>.<fingerprint>`, 16 KiB | 채널마다 하나 |
| 세그먼트 | `<signpost>.e<epoch>` | 발행자 시작마다 하나 |

## 발행자와 슬롯

채널마다 발행자는 호스트에 하나다. 두 번째 발행자는 생성자에서 `PUBLISHER_EXISTS`를 받는다.

발행자는 생성 시 같은 크기의 슬롯 `slot_count`개로 링을 만든다.

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `slot_size` | 8 MiB | 슬롯 하나의 크기(bytes). 생성된 어댑터의 가변 필드는 넘치면 arena에 들어간다. 그 밖의 프레임은 이 크기를 초과 불가능하다 |
| `slot_count` | 16 | 링의 슬롯 수. 2 이상 |
| `max_subscribers` | 16 | 구독 가능한 프로세스 수. 최대 4096 |
| `aux_size` | 0 | 슬롯마다 있는 aux의 크기(bytes). 최대 32 KiB |

- 링 전체(`slot_size` x `slot_count`, 기본 128 MiB)는 생성 시 `/dev/shm`에 예약한다. 공간이 모자라면 생성이 실패하고, 쓰기 도중에는 SIGBUS가 발생하지 않는다.
- 한 프로세스의 구독들은 `max_subscribers` 자리 하나를 공유한다. 자리가 모자라면 구독이 `SUBSCRIBERS_FULL`을 받는다.
- aux는 페이로드와 별도인 호스트 메모리이고, stamp나 크기 같은 작은 헤더를 담는다. GPU 메모리 채널에서도 호스트 메모리이므로 CPU 구독자와 GPU 구독자가 모두 읽는다. 쓴 길이가 `aux_size`를 넘으면 commit이 `TOO_LARGE`를 낸다.
- 프레임은 dtype, shape(8차원까지), bytes로 이루어진다. dtype은 `u8 i8 u16 i16 u32 i32 u64 i64 f16 f32 f64 bf16` 중 하나다. 생성된 어댑터의 프레임은 `u8` 배열이고, 스키마는 fingerprint로 구별한다.

### arena

arena는 발행 프로세스마다 하나 있는 공유 메모리다. 생성된 어댑터의 `Builder`는 슬롯에 들어가지 않는 가변 배열과 문자열을 arena에 저장하고, 프레임에는 그 위치만 적는다. 그래서 `slot_size`를 메시지의 최대 크기에 맞출 필요가 없다.

| 항목 | 동작 |
| --- | --- |
| 생성 시점 | 슬롯을 넘는 첫 프레임 |
| 크기 | 환경 변수 `FLUX_ARENA_BYTES`. 기본값이자 최대값은 1 GiB이고, 0이면 arena를 비활성화한다 |
| 공간이 모자라면 | commit이 `TOO_LARGE`를 낸다. SIGBUS는 발생하지 않는다 |
| arena 비활성화 시 | 슬롯보다 큰 프레임의 commit이 `TOO_LARGE`를 낸다 |
| 삭제 시점 | 발행 프로세스의 정상 종료. 죽은 프로세스의 arena는 다음 flux 프로세스가 지운다 |

- 블록은 그 슬롯에 다음 프레임을 쓰는 시점에 해제된다. 따라서 채널 하나가 arena에 남기는 프레임은 슬롯 수만큼이다. 점유된 프레임의 블록은 반납 시까지 남는다.
- 발행자를 지우면 점유되지 않은 블록은 즉시 해제되고, 점유된 블록은 발행 프로세스 종료 시까지 남는다.
- arena가 활성화된 `Builder`는 슬롯의 마지막 71 B를 가변 필드에 쓰지 않는다.
- 구독 측의 생성된 `View`는 arena를 읽기 전용으로 매핑하고, 구독과 `View`가 모두 사라지면 해제한다.
- `MemoryPolicy`의 `precommit`과 `lock`은 arena 페이지에도 적용된다. arena는 프로세스에 하나이므로 한 발행자의 요청이 arena 전체에 적용된다.
- 원시 loan(`loan(dtype, shape)`), GPU 메모리 채널, arena를 만든 부모에게서 fork한 자식은 arena를 사용하지 않는다.
- GPU에서 사용하는 필드는 GPU마다 하나 있는 GPU arena에 저장한다([gpu.md](gpu.md#생성-메시지의-gpu-필드)).

## 전달

전달은 best-effort다. 발행자는 느린 구독자를 기다리지 않는다.

```text
발행 -> [링: slot_count] -> take()가 커서를 옮긴다 -> view가 슬롯을 점유한다 -> view 반납
```

- 발행자는 점유되지 않은 슬롯 중 가장 오래된 것에 쓴다. 모든 슬롯이 점유되어 있으면 프레임을 버리고 `Backpressure`를 반환하며, `dropped`가 1 늘어난다.
- 구독자는 슬롯을 복사하지 않고 view로 읽는다. view가 있는 동안 발행자는 그 슬롯에 쓰지 않는다. 한 구독이 동시에 보유 가능한 view 수는 QoS `max_borrow`(기본 1)다.
- 받지 못한 프레임은 그 구독의 `lost`에 더해진다. 수신 규칙은 [qos.md](qos.md)에 있다.

## 시작 순서와 재시작

구독은 발행자보다 먼저 생성 가능하다. 연결 시점은 다음과 같다.

| 경우 | 연결되는 시점 |
| --- | --- |
| 발행자가 이미 있다 | 구독 생성자에서 |
| executor에 넣은 구독 | 발행자가 시작되는 즉시 |
| executor 없이 사용하는 구독 | 다음 `take()`에서 |

- 기본(volatile) 구독은 연결 후 발행된 프레임부터 받는다. 연결 전 링에 있던 프레임도 받으려면 QoS `replay`(transient local)를 사용한다.
- 발행자가 재시작하면 새 세그먼트가 생긴다. 구독은 다음 `take()`나 executor 주기(100 ms)에 새 세그먼트로 재연결된다.
- 프레임을 받은 `take()`는 새 세그먼트를 확인하지 않으므로, 재시작과 재연결 사이에 발행된 프레임은 받지 못할 가능성이 있다.
- 이전 세그먼트의 view는 반납 시까지 유효하다.
- 재연결되면 `attach_generation`이 바뀐다. `lost`는 재시작과 관계없이 계속 센다.

## 프로세스가 죽는 경우

- 구독 프로세스가 죽으면 발행자가 그 프로세스가 점유하던 슬롯을 회수한다.
- 발행자가 죽으면 구독은 새 발행자를 기다린다. 점유하던 view는 계속 읽기 가능하다.

## 스레드

- 두 스레드가 한 `Publisher`나 한 `Subscription`을 동시에 사용하면 거절된다. 발행자는 `InUse`나 빈 loan을 반환하고, 구독은 예외를 던진다(C++ `std::logic_error`, Python `RuntimeError`).
- 순차 사용은 호출마다 스레드가 달라도 가능하다(rclcpp, rclpy의 MultiThreadedExecutor).
- executor가 처리하는 구독에서 직접 `take()`를 호출하면 같은 예외가 발생한다. executor와 사용자가 프레임을 나눠 받게 되기 때문이다.
- 여러 스레드가 각자 모든 프레임을 받으려면 스레드마다 구독을 만든다.
- commit은 동시 호출을 거절하지 않지만, 한 발행자의 loan 둘을 두 스레드에서 동시에 commit하지 않는다.

## fork

fork한 자식 프로세스에서는 상속받은 채널이 사용 불가능하다. 구독의 `take()`는 예외(C++ `std::logic_error`, Python `RuntimeError`)를 던지고, 발행은 거절된다. 자식이 새로 만든 발행자와 구독은 정상 동작한다.

## 신뢰와 권한

- flux는 같은 uid의 프로세스 사이에서 사용한다. 프로세스는 자기 euid가 소유한 signpost와 세그먼트만 매핑한다. 다른 사용자의 채널은 목록에도 나오지 않는다.
- 같은 uid의 프로세스가 공유 메모리를 고의로 변경하는 것은 막지 않는다. 구독자는 프레임을 검사해 깨진 프레임을 건너뛰고(`refused.bad_frame`), 범위 밖을 읽지 않는다.

## 메모리와 컨테이너

| 상황 | 할 일 |
| --- | --- |
| Docker의 `/dev/shm`은 기본 64 MiB라서 기본 채널(128 MiB) 하나도 생성 불가능하다 | `--shm-size`를 채널들의 합보다 크게 설정하거나 `--ipc=host`를 사용한다 |
| 오래 사용하지 않은 슬롯 페이지가 스왑 아웃되어(Jetson의 zram 등) 첫 접근이 느려진다 | `MemoryPolicy`의 `lock`으로 세그먼트를 잠근다(`RLIMIT_MEMLOCK` 필요). 또는 systemd 유닛에 `MemorySwapMax=0`을 설정한다 |
| 첫 발행과 첫 take에서 페이지 폴트가 발생한다 | `MemoryPolicy`의 `precommit`으로 연결 시 페이지를 미리 채운다(Linux 5.14 이상) |

- `MemoryPolicy`의 `precommit`과 `lock`이 요청한 대로 되지 않으면 생성이 실패한다.
- Docker 기본 seccomp와 `--ipc` private, host 설정에서 동작을 확인했다. 컨테이너 사이의 채널 공유는 아직 지원하지 않는다.
