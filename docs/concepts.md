# flux 개념

C++·Python API가 공통으로 따르는 규칙이다. 언어별 사용법은 [cpp.md](cpp.md)와 [python.md](python.md)에 있다.

## 채널

채널은 한 호스트 안에서 토픽 이름과 fingerprint로 정해진다.

| 요소 | 뜻 |
| --- | --- |
| 토픽 이름 | 노드의 namespace와 remap을 적용한 이름이다. `Publisher(node, "image")`는 `/robot1` namespace에서 `/robot1/image`가 된다 |
| fingerprint | 메시지 스키마의 64비트 해시다. `flux_gen`이 만든 adapter의 `kFingerprint`(Python `FINGERPRINT__`)를 넘긴다. 타입 없이 배열을 보낼 때는 `kNoSchema`(0)다 |

- 발행자와 구독자는 두 값이 모두 같아야 연결된다. 이름이 같아도 fingerprint가 다르면 다른 채널이다. ROS 2에서 같은 이름에 타입이 다른 발행자가 공존하는 것과 같다.
- flux 채널에는 domain이 없다. flux는 `ROS_DOMAIN_ID`를 읽지 않는다. 그래서 한 호스트에서 이름과 fingerprint가 같으면 domain이 달라도 한 채널이다.
- 이름은 185자까지이고 `A-Z a-z 0-9 _ /`만 쓴다. ROS 토픽 이름은 이 규칙에 맞는다. 다른 문자가 있으면 생성자가 `INVALID_ARGUMENT`로 거절한다.
- `/dev/shm`에 채널마다 signpost 파일(`flux.v<버전>.<이름>.<fingerprint>`, 16 KiB)이 하나 있다. 이름의 `/`는 `.`으로 바뀐다. 발행자가 뜰 때마다 세그먼트 파일(`<signpost>.e<epoch>`)이 하나 생긴다. 죽은 프로세스가 남긴 파일은 다음에 뜨는 flux 프로세스가 지운다.

## 발행자와 슬롯

채널마다 발행자는 호스트에 하나다. 두 번째 발행자는 생성자에서 `PUBLISHER_EXISTS`로 거절된다.

발행자는 생성할 때 같은 크기의 슬롯 `slot_count`개로 된 링을 만든다.

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `slot_size` | 8 MiB | 슬롯 하나의 바이트 수. 생성된 adapter의 가변 필드는 넘치면 arena에 들어가고(아래), 그 밖의 프레임은 이것이 상한이다 |
| `slot_count` | 16 | 링의 슬롯 수. 2 이상이어야 한다 |
| `max_subscribers` | 16 | 이 채널을 구독할 수 있는 프로세스 수. 최대 4096 |
| `aux_size` | 0 | 슬롯마다 프레임과 함께 commit되는 호스트 메모리 바이트 수(aux). 최대 32 KiB |

- 발행자는 생성할 때 링 전체(`slot_size` x `slot_count`)를 `/dev/shm`에서 미리 잡는다(`fallocate`). 공간이 모자라면 생성이 실패한다. 그래서 쓰는 도중에 SIGBUS로 죽지 않는다. 기본값이면 채널 하나에 128 MiB다.
- `max_subscribers`는 구독 수가 아니라 구독 프로세스 수다. 한 프로세스의 구독들은 자리 하나를 같이 쓴다. 자리가 모자라면 그 구독은 `SUBSCRIBERS_FULL`로 거절된다.
- aux는 payload와 따로 있는 호스트 메모리다. payload가 장치 메모리인 채널에서도 aux는 호스트 메모리라서 CPU 구독자와 GPU 구독자가 모두 읽는다. 프레임에 붙일 작은 값(stamp, 크기 같은 헤더)을 쓴다. 프레임마다 쓴 길이를 함께 담고, `aux_size`를 넘는 길이의 commit은 `TOO_LARGE`다.
- 프레임은 dtype, shape(8차원까지), 바이트로 이루어진다. dtype은 `u8 i8 u16 i16 u32 i32 u64 i64 f16 f32 f64 bf16`이다. 생성된 adapter가 쓰는 프레임은 `u8` 바이트열이고, 스키마는 fingerprint로 구별한다.

### arena

생성된 adapter의 `Builder`는 슬롯에 들어가지 않는 가변 배열과 문자열을 발행 프로세스마다 하나인 공유메모리 arena에 둔다. 프레임에는 arena 안의 위치만 적는다. 그래서 `slot_size`를 메시지의 최대 크기에 맞추지 않아도 된다.

- arena는 슬롯을 넘는 프레임이 처음 나올 때 만든다. 크기는 환경 변수 `FLUX_ARENA_BYTES`(기본이자 최대 1 GiB)이고, `/dev/shm`의 페이지는 블록이 닿는 만큼 4 MiB씩 잡는다(`fallocate`). 공간이 모자라면 그 프레임의 commit이 `TOO_LARGE`이고, SIGBUS로 죽지 않는다.
- `FLUX_ARENA_BYTES=0`이면 끈다. 그때 슬롯보다 큰 프레임은 `TOO_LARGE`다.
- 블록은 그 프레임의 슬롯을 다시 쓸 때 푼다. 그래서 채널 하나가 arena에 남기는 것은 슬롯 수만큼의 프레임이고, 구독자가 쥔 프레임의 블록은 놓을 때까지 남는다. 발행자를 지우면 구독자가 쥐지 않은 블록을 바로 풀고, 쥔 블록은 그 프로세스가 끝날 때까지 남는다.
- arena를 쓴 프레임은 슬롯 끝 64바이트에 arena 이름과 블록 목록의 위치를 담는다. 목록은 블록마다 8바이트이고, 슬롯에 자리가 남으면 그 앞에, 아니면 arena에 둔다. 이름 자리 때문에 arena가 켜진 `Builder`는 슬롯의 마지막 71바이트를 가변 필드에 쓰지 않는다.
- 받는 쪽의 생성된 `View`(C++, Python)가 arena를 읽기 전용으로 매핑하고, 그 구독과 `View`가 모두 사라지면 푼다.
- `MemoryPolicy`의 `precommit`과 `lock`은 arena의 페이지에도 적용된다. 한 프로세스의 요청을 합친 것이다.
- arena는 정상 종료할 때 지운다. 죽은 프로세스가 남긴 arena는 다음 flux 프로세스가 지운다.
- arena를 쓰지 않는 것: 원시 loan(`loan(dtype, shape)`), 장치 메모리 채널, arena를 만든 부모에게서 fork한 자식.
- GPU에서 쓰는 필드는 장치마다 있는 GPU arena에 둔다([gpu.md](gpu.md#생성-메시지의-장치-필드)).

## 전달

전달은 best-effort다. 발행자는 느린 구독자를 기다리지 않는다.

```text
발행 -> [링: slot_count] -> take()가 커서를 옮긴다 -> view가 슬롯을 쥔다 -> view 해제
```

- 발행자는 구독자가 쥐지 않은 슬롯 중 가장 오래된 것에 쓴다. 모든 슬롯을 구독자가 쥐고 있으면 발행자는 그 프레임을 버리고 `Backpressure`를 돌려준다. 이때 `dropped`가 1 오른다.
- 구독자는 슬롯을 복사하지 않고 view로 읽는다. view가 살아 있는 동안 발행자는 그 슬롯을 다시 쓰지 못한다. 한 구독이 동시에 쥘 수 있는 view 수는 QoS `max_borrow`(기본 1)다.
- 구독자가 받지 못하고 지나간 프레임은 그 구독의 `lost`에 쌓인다. 구독자가 무엇을 언제 받는지는 [qos.md](qos.md)에 있다.

## 시작 순서와 재시작

- 구독은 발행자보다 먼저 만들어도 된다. 발행자가 뜨면 붙는다. executor에 넣은 구독은 발행자가 뜨는 순간 깨어 붙고, executor 없이 쓰는 구독은 다음 `take()`에서 붙는다.
- 기본(volatile) 구독은 붙은 뒤에 발행된 프레임부터 받는다. 발행자가 이미 떠 있으면 구독 생성자에서 붙는다. 붙기 전에 링에 있던 프레임까지 받으려면 QoS `replay`(transient local)를 쓴다.
- 발행자가 재시작하면 새 세그먼트가 생긴다. 구독은 그 뒤의 `take()`나 다음 executor tick(100 ms)에서 새 세그먼트에 다시 붙는다. 다만 프레임을 받은 `take()`는 새 세그먼트를 확인하지 않는다. 그 사이에 발행된 프레임은 받지 못할 수 있다. 이전 세그먼트의 view는 해제될 때까지 유효하다.
- 다시 붙을 때 `attach_generation`이 바뀐다. `lost`는 재시작을 넘어 계속 센다.

## 프로세스가 죽을 때

- 구독 프로세스가 죽으면 그 프로세스가 쥐던 슬롯을 발행자가 회수한다. 발행자는 빈 슬롯을 찾지 못할 때 회수한다. 기다리는 구독자가 있는 동안에는 commit할 때도 1초에 한 번까지 회수한다.
- 발행자가 죽으면 구독은 새 발행자를 기다린다. 쥐고 있던 view는 그대로 읽을 수 있다.
- flux는 프로세스가 살아 있는지를 OFD lock으로 판정한다. 프로세스가 끝나면 커널이 이 lock을 함께 푼다. 공유메모리에 남은 숫자로는 판정하지 않는다.

## 스레드

- 두 스레드가 `Publisher`나 `Subscription` 하나를 동시에 쓰면 flux가 거절한다. 발행자는 `InUse`나 빈 loan을 돌려주고, 구독은 C++에서 `std::logic_error`, Python에서 `RuntimeError`를 던진다. 차례로 쓰는 것은 된다. rclcpp·rclpy의 MultiThreadedExecutor처럼 매번 다른 스레드에서 부르는 것도 된다.
- executor가 서비스하는 구독은 직접 `take()`할 수 없다(C++ `std::logic_error`, Python `RuntimeError`). executor와 사용자가 프레임을 나눠 갖게 되기 때문이다.
- 여러 스레드가 각자 모든 프레임을 받으려면 스레드마다 구독을 만든다.
- loan의 commit은 동시에 불러도 거절하지 않는다. 그래도 한 발행자의 loan 둘을 두 스레드가 동시에 commit하는 것은 피한다.

## fork

fork한 자식은 부모에게서 물려받은 채널을 쓰지 않는다. 물려받은 구독의 `take()`는 C++에서 `std::logic_error`, Python에서 `RuntimeError`로 거절되고, 물려받은 발행자의 발행도 거절된다. 자식이 새로 만든 발행자와 구독은 정상으로 동작한다.

## 신뢰와 권한

- flux는 같은 uid의 프로세스끼리 쓴다. 프로세스는 자기 euid가 소유한 signpost와 세그먼트만 연다. 다른 사용자의 채널은 목록에도 나오지 않는다.
- 같은 uid의 피어가 공유메모리를 일부러 고치는 것은 막지 않는다. 다만 받는 쪽이 프레임을 검사해 깨진 프레임을 건너뛴다(`refused.bad_frame`). 그래서 범위 밖을 읽지 않는다.

## 메모리와 컨테이너

| 상황 | 할 일 |
| --- | --- |
| Docker의 `/dev/shm`은 기본 64 MiB다. 기본 채널(128 MiB) 하나도 만들지 못한다 | `--shm-size`를 채널들의 합보다 크게 두거나 `--ipc=host`를 쓴다 |
| 스왑(Jetson의 zram 등)이 오래 안 쓴 슬롯 페이지를 내보내 첫 접근이 느려진다 | `MemoryPolicy`의 `lock`으로 세그먼트를 잠근다(`RLIMIT_MEMLOCK` 필요). 또는 systemd 유닛에 `MemorySwapMax=0`을 둔다 |
| 첫 발행과 첫 take에서 페이지 fault가 난다 | `MemoryPolicy`의 `precommit`으로 붙을 때 미리 채운다(Linux 5.14 이상) |

- flux는 세그먼트의 제어 영역(슬롯 데이터 앞 부분, 기본 세그먼트에서 한 페이지)을 `MemoryPolicy`와 상관없이 붙을 때 mlock한다. mlock 한도가 모자라면 오류 없이 넘어간다.
- `MemoryPolicy`의 `precommit`과 `lock`은 요청한 대로 되지 않으면 생성이 실패한다.
- 컨테이너 안에서는 Docker 기본 seccomp와 `--ipc` private·host 설정에서 도는 것을 확인했다. 컨테이너 사이에서 채널을 공유하는 것은 아직 지원하지 않는다.
