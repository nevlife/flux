# flux QoS

전달은 best-effort로 고정이다(drop-on-full). 소비자가 고르는 것은 셋 — `depth`·`durability`·`max_borrow`다. 앞의 둘은 이름과 뜻을 ROS 2 rmw QoS에서 가져오고 `max_borrow`는 대응이 없다(2절). 셋 다 구현됐다.

## 1. 두 연산 — peek / take

QoS를 읽기 전에 연산 두 개를 구분한다. DDS의 `read()` / `take()`와 같은 구분이다.

| | 커서 | 빈 것을 주는 때 | 쓰는 곳 |
| --- | --- | --- | --- |
| `peek()` | 안 건드린다 | 발행이 한 번도 없을 때 | 지금 상태를 자기 주기에 읽는다 |
| `take()` | 전진한다 | 위 + 따라잡았을 때 | 프레임을 빠짐없이 처리한다 |

`peek()`은 새 프레임이 없으면 같은 프레임을 다시 준다. 상태 읽기라서 그렇다. `take()`는 한 프레임을 한 번만 준다. `take_blocking(ns)`는 `take()`가 빈 것을 줄 때 futex에 park한다. 단 `max_borrow`가 소진돼 빈 것이면 park하지 않고 바로 돌려준다 — 리스를 풀 수 있는 건 호출자뿐이라 기다려도 안 풀린다. `peek`에는 blocking이 없다 — 기다릴 것이 없다.

둘 다 프레임 유무와 무관하게 빈 것을 주는 경우가 더 있다. `Channel::refused()`가 이유별로 센다 — `lost`에도 `dropped`에도 안 잡히는 것들이라 여기 말고는 드러날 데가 없다. 누적값이다.

| 왜 | 카운터 | 언제 |
| --- | --- | --- |
| `max_borrow` 소진 | `max_borrow` | 쥔 view를 안 놓고 또 부를 때 |
| slot holder 테이블 만원 | `holder_table` | 한 슬롯을 `kMaxHolders`(10) 프로세스가 이미 쥐고 있을 때 |
| 세그먼트가 아직 ready 아님 | `not_ready` | `init_state != ready` — 생성자가 bootstrap 중 |
| 경쟁에 져 borrow 획득 실패 | `contended` | 재시도 상한(64회)까지 seqlock 검증이 계속 실패할 때 |
| meta가 슬롯 범위를 벗어남 | `bad_frame` | 손상된 프레임을 view로 안 내준다 |
| 슬롯에 프레임이 없음 | `bad_frame` | `commit_ticket == 0` — abort된 loan이나 회수된 claim이 남긴 상태. `latest`가 아직 그 슬롯을 가리켜도 내줄 프레임이 없다 |
| owner 파일을 못 잡음 | `no_owner_file` | fd 고갈 등으로 이 프로세스가 자기 owner 파일을 못 열었을 때 |
| 리스가 fence 실패로 다 샘 | `fence` | 실패한 release fence가 리스를 `max_borrow`개 새게 했을 때. 영구 상태다 |

"아직 발행이 없다"와 "따라잡았다"는 안 센다. 그 둘은 빈 view가 제 일을 한 것이다.

`max_borrow` 소진만 대응이 다르다. 재시도해도 안 풀리고 `take_blocking`이 park조차 안 하므로, 그 자리에서 도는 루프는 CPU만 태운다. 미리 물으려면 `can_borrow()`를 쓴다. 나머지는 재시도하면 된다.

`fence`는 대응이 아예 없다. `max_borrow`처럼 리스가 없어서 거절하는 것이지만, 새어 나간 리스는 호출자가 돌려줄 수 없다. 채널을 버리는 것이 유일한 답이다.

해제 쪽에도 카운터가 하나 있다. `Channel::fence_failed()`는 GPU stream fence가 실패해 view를 놓지 못한 횟수다. fence를 확인 못 했는데 refcount를 줄이면 발행자가 아직 읽히는 중인 슬롯을 덮으므로, 그 슬롯은 잠긴 채 남는다. `Device::Cpu` 채널에서는 항상 0이다 — CUDA를 부르지 않으므로 실패할 것이 없다.

`Channel::fence_wait()`는 같은 자리에서 시간을 준다 -- 두 seam이 각각 합·횟수·최댓값이다(`api.ko.md` GPU 절). `fence_failed()`가 실패를 세는 동안 이쪽은 성공한 기다림이 얼마나 걸렸는지를 센다. `Device::Cpu` 채널에서는 시계조차 안 읽는다.

이 값은 `max_borrow`에서 멈춘다. 리스가 그만큼 새면 borrow 자체가 안 나가서 fence할 것도 없기 때문이다. 비율이 아니라 상태로 읽는다 — 멈춘 값이 곧 그 소비자가 끝났다는 뜻이고, 그 뒤의 호출은 `refused().fence`가 센다.

콜백 경로(`flux::ros::Subscription`, `Executor`)는 `take()`로 배달한다. `peek()`은 직접 당겨 쓰는 쪽 전용이다.

## 2. 값 셋

```text
depth        뒤처짐 상한. take()가 최신에서 몇 프레임까지 밀려도 되는가.
durability   붙기 전 발행분을 받는가. Volatile() 또는 TransientLocal(n).
max_borrow   동시에 쥐는 view 수 상한. ROS 2에 대응이 없다.
```

| 설정 | ROS 2 | 결과 |
| --- | --- | --- |
| `depth = 1` (기본) | History KEEP_LAST(1) | 항상 최신만. 중간 프레임은 건너뛰고 `lost`로 센다 |
| `depth = N` | History KEEP_LAST(N) | N개까지 밀렸다가 발행 순서대로 따라잡는다 |
| `Volatile()` (기본) | Durability VOLATILE | 이 소비자가 스트림에 합류한 뒤 발행분만 |
| `TransientLocal(n)` | Durability TRANSIENT_LOCAL | 합류 시점에 ring에 남아 있던 것 중 n개를 먼저 재생하고 live로 |

매 `take()`마다 커서를 `newest - depth`로 당긴다. `depth = 1`이면 커서가 늘 `newest - 1`이라 최신이 나온다 — 별도 모드가 아니라 같은 식의 끝값이다.

`depth`는 executor가 한 pass에 돌리는 콜백 수를 안 정한다. 콜백이 도는 동안 도착한 프레임에 대해 다음 `take()`가 다시 성공하므로 `depth = 1`에서도 한 pass에 수십 개가 돈다. 그 수를 묶는 것은 `pass_budget`이다.

합류 시점은 첫 take가 아니라 attach다. 구독을 만들 때 세그먼트가 이미 있으면 거기서 붙고, 없으면 뜬 뒤 첫 시도에서 붙는다. 그래서 `Volatile()`은 "내가 붙은 뒤"이지 "내가 처음 읽은 뒤"가 아니다.

`max_borrow`(기본 2)는 ROS 2에 대응이 없다. 동시에 쥐는 view 수 상한이다 — 쥔 view는 슬롯을 byte-lock으로 잡아두므로 발행자가 그 슬롯을 못 쓴다.

`reliability`는 `best_effort`만 받는다. `reliable`을 주면 거절한다.

## 3. 세 깊이 구분 (헷갈림 주의)

```text
발행 -> [ring: slot_count] -> [depth] -> take() -> [max_borrow] -> release
         프레임이 실재하는 곳    뒤처짐 상한          손에 쥔 것
```

- `slot_count` — 발행자가 생성 시 정한다. ROS 2의 발행자 History depth에 해당한다. 프레임이 실제로 사는 유일한 곳이라 아래 둘의 하드 캡이다.
- `depth` — 구독자가 정한다. 뒤처짐 상한.
- `max_borrow` — 구독자가 정한다. 동시 보유 상한.

구독자는 아무것도 보관하지 않는다. 바이트는 발행자 ring에 한 벌만 있고 구독자는 커서만 옮긴다. `depth`가 큐 깊이처럼 보이는 것은 밖에서 관측되는 동작이 같아서다.

## 4. 거절하는 조합

조용히 다른 값으로 바꾸지 않는다. `QoS::validate()`가 던진다.

| 조합 | 왜 |
| --- | --- |
| `TransientLocal(n)`에서 `n > depth` | 재생분도 같은 뒤처짐 창으로 들어온다. 밀릴 수 있는 것보다 많이 받을 수 없다 |
| `depth == 0` | 최신만은 `depth = 1`이다 |
| `max_borrow == 0` | view를 하나도 못 쥐면 take가 성립하지 않는다 |
| `reliability = reliable` | 미구현 (§6) |

ring보다 깊은 QoS는 에러가 아니다. ring은 아직 없을 수도 있는 발행자의 것이다. 보관된 만큼으로 잘리고, 못 받은 수는 `lost`에 잡힌다.

## 5. 쓰는 법

```cpp
flux::QoS q;                                        // depth 1, volatile
q.depth = 10;
q.durability = flux::Durability::TransientLocal(5);
ch.qos(q);
while (flux::FrameView v = ch.take()) { use(v); }
```

```python
sub = flux.Subscription("/img")                     # depth 1, volatile
sub.peek()                                          # 지금 최신
sub.take_blocking(-1)                               # 새 것 올 때까지

qos = flux.QoS(depth=10, durability=flux.TransientLocal(5))
sub = flux.Subscription("/cmd", qos=qos)
while (v := sub.take()) is not None:
    handle(v)
```

`lost`는 합류 이후 못 받은 프레임의 누적 수다. DDS의 sample-lost status와 같은 뜻이라 차분을 내면 비율이 된다.

## 6. 미구현

| 옵션 | 뜻 | 필요한 것 |
| --- | --- | --- |
| sub_buffer | 도착했지만 아직 안 꺼낸 것의 큐 깊이 | 절반은 `depth`가 이미 한다(3절). 나머지 절반은 구독자별 커서 테이블 |
| overflow 정책 | 가득 찰 때 오래된 것을 덮을지 발행자를 막을지 | 발행자 정책 플래그 |
| reliability | 전달 보장 | 공유 커서 테이블 + backpressure |

reliability가 제일 크다. 발행자가 제일 느린 구독자를 기다리려면 공유메모리에 구독자별 커서 테이블을 두고 ring이 미소비 프레임으로 차면 block/drop-newest해야 한다. best-effort를 넘어서는 delivery 모델 전환이다.

reliability는 우선순위 아니다. 무손실은 non-goal로 정해져 있고, 그 결정을 뒤집을 수요가 확인되면 별도 설계로 착수한다.

overflow 정책도 우선순위 아니다. flux에는 "미소비"라는 개념 자체가 없다 — 발행자는 `latest` 다음 자리부터 찾아 처음 만나는 borrow되지 않은 슬롯을 덮고, 아무도 어디까지 읽었는지 기록하지 않는다. 덮지 않는 쪽을 고르게 하려면 그 기록이 필요하고, 그것이 곧 reliability가 요구하는 구독자별 커서 테이블이다. 플래그만 먼저 낼 수 있는 항목이 아니다.

sub_buffer는 둘로 갈린다. "N개까지 밀렸다가 발행 순서대로 따라잡는다"는 관측 동작은 `depth`가 그대로 한다(3절) — ROS 2 구독의 KEEP_LAST(N)를 flux 용어로 옮기면 그 이름이다. 그 절반은 이름을 맞추는 일이 맞고, 이미 맞춰져 있다.

남는 절반은 "그 N개가 나를 위해 잡혀 있다"이고, 이쪽은 없다. `depth`는 발행자 ring 위의 뒤처짐 상한일 뿐이라 아무것도 예약하지 않는다 — 내가 아직 안 가져간 프레임을 발행자가 덮으면 커서가 `newest - depth`로 당겨지고 그 차이가 `lost`로 잡힌다. 잡아 두게 하려면 구독자마다 어디까지 안 가져갔는지를 공유메모리에 적어야 하고, 그것이 overflow·reliability가 요구하는 바로 그 커서 테이블이다.

그래서 이 표의 세 항목은 크기가 다른 셋이 아니라 같은 막힘 하나다. 구독자별 커서 테이블을 두는 순간 셋이 같이 열리고, 두지 않으면 셋 다 안 열린다. 그 테이블은 best-effort를 넘어서는 delivery 모델 전환이고, 무손실은 non-goal이다.
