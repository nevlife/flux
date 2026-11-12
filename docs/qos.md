# QoS

구독자가 고르는 값은 셋이다. 전달은 언제나 best-effort이고, 발행자는 구독자를 기다리지 않는다.

| 값 | 기본값 | ROS 2에서 가까운 것 | 뜻 |
| --- | --- | --- | --- |
| `depth` | 8 | History KEEP_LAST(depth) | 최신 프레임에서 몇 개까지 뒤처져도 되는가 |
| `replay` | 0 | Durability TRANSIENT_LOCAL | 연결 시 이미 링에 있던 프레임을 몇 개 먼저 받는가. 0은 volatile이다 |
| `max_borrow` | 1 | 없음 | 동시에 보유 가능한 view 수 |

```cpp doc:cpp_qos
const flux::QoS qos = flux::QoS(4).transient_local(2).max_borrow(2);
flux::Subscription sub(*node, "cmd", flux::kNoSchema, qos);
```

```python doc:py_qos
qos = flux.QoS(depth=4, replay=2, max_borrow=2)
sub = flux.Subscription(node, "cmd", qos=qos)
```

## 세 깊이

```text
발행 -> [링: slot_count개] -> [depth] -> take() -> [max_borrow] -> 반납
         프레임이 있는 곳       뒤처짐 상한            점유한 view
```

- `slot_count`는 발행자가 생성 시 정한다. 프레임은 발행자의 링에만 있으므로 `slot_count`가 아래 두 값의 실제 상한이다.
- `depth`가 링보다 커도 오류는 아니지만, 링에 남은 프레임만 받는다.
- 점유된 슬롯에는 발행자가 쓰지 않으므로, 모든 구독자가 점유한 view의 합이 `slot_count`에 이르면 발행자가 `Backpressure`를 받는다.

## depth

`take()`는 아직 받지 않은 가장 오래된 프레임을 반환한다. 커서가 최신 프레임보다 `depth`를 넘게 뒤처져 있으면 `최신 - depth` 뒤로 건너뛰고, 건너뛴 수를 `lost`에 더한다. 발행자가 이미 덮어쓴 프레임도 `lost`에 센다.

- `depth = 1`이면 언제나 최신 프레임만 받는다.
- `depth = N`이면 N개까지 밀린 뒤 발행 순서대로 따라잡는다.
- executor가 한 번에 실행하는 콜백 수는 `depth`와 관계없이 pass 예산(64)으로 정해진다.

## replay

volatile(`replay = 0`) 구독은 연결 후 발행된 프레임부터 받는다. `replay = n`이면 연결 시점에 링에 남아 있는 최신 n개를 먼저 받고, 이어서 새 프레임을 받는다.

- 발행자가 이미 있으면 구독은 생성자에서 연결된다. 발행자가 없으면 발행자가 시작된 뒤, executor가 깨어나는 시점이나 다음 `take()` 호출 시 연결된다([concepts.md](concepts.md#시작-순서와-재시작)).
- 발행자가 재시작하면 연결된 구독은 새 발행자의 첫 프레임부터 읽는다. 그 프레임은 모두 구독의 합류 후 발행된 것이기 때문이다. `depth`보다 오래된 프레임은 `lost`로 센다.

## max_borrow

구독이 view를 `max_borrow`개 점유하고 있으면, `take()`는 프레임이 있어도 빈 결과를 반환하고 `refused.max_borrow`를 센다. `take_blocking()`도 즉시 반환한다. view는 호출자만 반납하므로 기다려도 반납되지 않는다. executor에 넣은 구독은 view가 반납되면 깨어나 다시 받는다.

Python에서는 배열에 대한 참조가 모두 사라져야 view가 반납된다. 앞 프레임을 변수에 남긴 채 다음 `take()`를 호출하면 `max_borrow = 1`에서 빈 결과가 나온다([python.md](python.md#executor-없이-읽기)).

## 거절하는 값

아래 값은 다른 값으로 바꾸지 않고, 구독 생성자가 예외를 던진다(C++ `flux::Error`, Python `flux.Error`(`Status.INVALID_ARGUMENT`)).

| 값 | 이유 |
| --- | --- |
| `depth = 0` | 최신 프레임만 받으려면 `depth = 1`을 사용한다 |
| `max_borrow = 0` | view를 하나도 보유하지 못하면 take가 성립하지 않는다 |
| `replay > depth` | 다시 받는 프레임도 같은 뒤처짐 범위에 들어온다 |

## 빈 결과의 이유

빈 결과(C++의 빈 `FrameView`, Python의 `None`)는 받을 프레임이 아직 없거나, 프레임과 관계없는 이유로 take가 거절됐다는 뜻이다. 거절 횟수는 `refused()`(Python `refused`)에 이유별로 누적된다.

| 필드 | 발생 조건 | 할 일 |
| --- | --- | --- |
| `max_borrow` | view를 `max_borrow`개 점유한 채 다시 호출했다 | view를 반납한다 |
| `no_reader` | core와 C ABI 채널에서 구독자 번호를 얻지 못했거나, fork 전에 생성한 채널을 자식 프로세스에서 사용했다. C++와 Python 구독은 이 경우를 연결 시 `SUBSCRIBERS_FULL`로, fork한 자식의 take에서는 예외로 알린다 | 자리가 나면 다시 시도한다. fork한 자식은 새 구독을 생성한다 |
| `contended` | 발행자와 경쟁하다가 재시도 예산을 다 썼다 | 다시 시도한다 |
| `bad_frame` | 프레임의 메타데이터가 슬롯 범위를 벗어났다. 그 프레임은 건너뛴다 | 발행 쪽을 확인한다 |
| `fence` | GPU 해제의 완료 신호 대기가 실패해 점유된 view가 `max_borrow`를 채웠거나, 프로세스의 GPU가 멈췄다([gpu.md](gpu.md)) | 완료 신호 대기 상한이 있으면 GPU 작업이 완료된 뒤 view가 반납된다. 상한이 없거나 GPU가 멈췄으면 프로세스를 다시 시작한다 |

## 세는 값

| 값 | 쪽 | 뜻 |
| --- | --- | --- |
| `dropped` | 발행자 | 모든 슬롯이 점유되어 버린 프레임 수 |
| `subscriber_count` | 발행자 | 현재 구독 중인 살아 있는 프로세스 수. 한 프로세스의 구독 여럿은 1로 센다 |
| `lost` | 구독 | 받지 못하고 지나간 프레임 수. DDS의 sample lost와 같은 뜻이다. 재시작과 관계없이 누적된다 |
| `refused` | 구독 | 위 표의 이유별 거절 수 |

## 없는 것

| 옵션 | 이유 |
| --- | --- |
| reliability(손실 없는 전달) | 발행자가 가장 느린 구독자를 기다려야 하고, 구독자마다 읽은 위치를 공유 메모리에 기록해야 한다. best-effort를 넘는 전달 모델이라 지금은 목표가 아니다 |
| 링이 가득 찬 경우 발행자를 막는 정책 | 위와 같은 구독자별 위치 기록이 필요하다 |
| 구독자별 수신 큐 | 프레임은 발행자의 링에만 있다. 밀린 프레임은 `depth` 범위 안에서 받는다 |
