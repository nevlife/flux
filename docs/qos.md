# QoS

구독자가 고르는 값은 셋이다. 전달은 언제나 best-effort다. 발행자는 구독자를 기다리지 않는다.

| 값 | 기본 | ROS 2에서 가까운 것 | 뜻 |
| --- | --- | --- | --- |
| `depth` | 8 | History KEEP_LAST(depth) | 최신 프레임에서 몇 개 뒤처져도 되는가 |
| `replay` | 0 | Durability TRANSIENT_LOCAL | 붙을 때 이미 링에 있던 프레임 몇 개를 먼저 받는가. 0은 volatile이다 |
| `max_borrow` | 1 | 없음 | 동시에 쥘 수 있는 view 수 |

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
발행 -> [링: slot_count개] -> [depth] -> take() -> [max_borrow] -> 해제
         프레임이 있는 곳       뒤처짐 상한            손에 쥔 view
```

- `slot_count`는 발행자가 생성할 때 정한다. 프레임이 실제로 있는 곳은 발행자의 링 하나뿐이고, 구독자는 프레임을 따로 보관하지 않는다. 그래서 `slot_count`가 아래 둘의 실제 상한이다.
- `depth`는 구독자의 뒤처짐 상한이다. 링보다 큰 `depth`도 오류는 아니다. 링에 남은 만큼만 받는다.
- `max_borrow`는 구독자가 동시에 쥐는 view 수다. 쥔 view는 슬롯을 막으므로, 모든 구독자가 쥔 view의 합이 `slot_count`에 닿으면 발행자는 `Backpressure`를 받는다.

## depth

`take()`는 아직 받지 않은 가장 오래된 프레임을 준다. 커서가 최신에서 `depth`보다 더 뒤처져 있으면 `최신 - depth` 뒤로 건너뛰고, 건너뛴 수를 `lost`에 더한다. 발행자가 이미 덮어써 없어진 프레임도 `lost`에 든다.

- `depth = 1`이면 늘 최신 프레임만 받는다.
- `depth = N`이면 N개까지 밀렸다가 발행 순서대로 따라잡는다.
- executor가 한 번에 돌리는 콜백 수는 `depth`가 아니라 pass 예산(64)이 정한다.

## replay

volatile(`replay = 0`) 구독은 붙은 뒤에 발행된 프레임부터 받는다. `replay = n`이면 붙는 순간 링에 남아 있는 최신 n개를 먼저 받고 이어서 새 프레임을 받는다.

- 붙는 시점은 발행자가 이미 있으면 구독 생성자다. 발행자가 없으면 발행자가 뜬 뒤 executor가 깨어나거나 다음 `take()`가 불릴 때다([concepts.md](concepts.md#시작-순서와-재시작)).
- 한 번 붙은 구독은 발행자가 재시작하면 새 발행자의 첫 프레임부터 읽는다. 그 프레임은 모두 구독이 합류한 뒤에 나왔기 때문이다. `depth`보다 오래된 것은 `lost`로 센다.
- `replay`는 `depth` 이하여야 한다. 다시 받는 프레임도 같은 뒤처짐 범위로 들어오기 때문이다.

## max_borrow

`max_borrow`만큼 view를 쥔 구독의 `take()`는 프레임이 있어도 빈 결과를 주고 `refused.max_borrow`를 센다. `take_blocking()`도 기다리지 않고 바로 돌아온다. view를 놓는 것은 호출자뿐이라 기다려도 풀리지 않기 때문이다. executor에 넣은 구독은 view가 풀리면 깨어 다시 받는다.

Python에서는 배열에 대한 참조가 모두 사라져야 view가 풀린다. 앞 프레임을 이름에 남긴 채 다음 `take()`를 부르면 `max_borrow = 1`에서 빈 결과가 나온다([python.md](python.md#executor-없이-읽기)).

## 거절하는 값

조용히 다른 값으로 바꾸지 않는다. 구독 생성자가 C++에서 `flux::Error`, Python에서 `flux.Error`(`Status.INVALID_ARGUMENT`)를 던진다.

| 값 | 이유 |
| --- | --- |
| `depth = 0` | 최신만 받는 것은 `depth = 1`이다 |
| `max_borrow = 0` | view를 하나도 쥘 수 없으면 take가 성립하지 않는다 |
| `replay > depth` | 다시 받는 프레임도 같은 뒤처짐 범위로 들어온다 |

## 빈 결과의 이유

빈 결과(C++의 빈 `FrameView`, Python의 `None`)는 받을 프레임이 아직 없다는 뜻이거나, 프레임과 무관한 이유로 거절됐다는 뜻이다. 거절은 `refused()`(Python `refused`)가 이유별로 센다. 값은 누적이다.

| 필드 | 언제 | 할 일 |
| --- | --- | --- |
| `max_borrow` | `max_borrow`만큼 view를 쥔 채 또 불렀다 | view를 놓는다 |
| `no_reader` | core와 C ABI 채널에서 구독자 번호를 얻지 못했거나, fork 전에 연 채널을 자식에서 썼다. C++·Python 구독은 이를 다르게 알린다. 붙을 때 `SUBSCRIBERS_FULL`이고, fork한 자식의 take는 예외다 | 자리가 나면 다시 시도한다. fork한 자식은 새 구독을 만든다 |
| `contended` | 발행자와의 경쟁에서 재시도 예산을 다 썼다 | 다시 시도한다 |
| `bad_frame` | 프레임의 메타데이터가 슬롯 범위를 벗어났다. 그 프레임은 건너뛴다 | 발행 쪽을 확인한다 |
| `fence` | GPU 해제 fence가 실패해 쥔 view가 `max_borrow`를 채웠거나, 프로세스의 GPU가 멈췄다([gpu.md](gpu.md)) | fence 상한이 있으면 GPU 작업이 끝난 뒤 view가 돌아온다. 상한이 없거나 GPU가 멈췄으면 프로세스를 다시 띄운다 |

## 세는 값

| 값 | 쪽 | 뜻 |
| --- | --- | --- |
| `dropped` | 발행자 | 모든 슬롯이 쥐여 있어 버린 프레임 수 |
| `lost` | 구독 | 받지 못하고 지나간 프레임 수. DDS의 sample lost와 같은 뜻이다. 재시작을 넘어 누적한다 |
| `refused` | 구독 | 위 표의 이유별 거절 수 |

## 없는 것

| 옵션 | 이유 |
| --- | --- |
| reliability(손실 없는 전달) | 발행자가 가장 느린 구독자를 기다려야 한다. 구독자마다 읽은 위치를 공유메모리에 적어야 하고, best-effort를 넘는 전달 모델이다. 지금은 목표가 아니다 |
| 가득 찰 때 발행자를 막는 정책 | 위와 같은 구독자별 위치 기록이 필요하다 |
| 구독자별 수신 큐 | 프레임은 발행자의 링에만 있다. 밀린 프레임을 받는 동작은 `depth`가 맡는다 |
