# flux borrow 수명 (view 해제 규약)

구독자가 받는 view는 공유메모리 슬롯을 가리키는 빌린 참조다. 복사가 아니다. 이 문서는 그 borrow가 언제 잡히고 언제 놓이는지를 못박는다. 정본은 이 문서와 코드(`flux_core`의 FrameView, `flux_cpp`의 Subscription, `flux_py` 바인딩)다.

## borrow란

- 구독자가 `peek()`/`take()`로 view를 받으면 그 슬롯의 refcount가 +1 된다(borrow).
- borrow를 쥔 슬롯은 byte-lock된다 — 발행자가 그 슬롯을 못 덮는다(active => byte-locked).
- view가 놓일 때 refcount가 -1 된다.
- 한 구독자가 동시에 쥐는 view 수는 `max_borrow`(기본 2)로 제한된다(`qos.ko.md`). 초과하면 빈 view가 온다 — 쥔 걸 놓아야 풀린다.

view는 슬롯을 가리키는 참조이므로 쥐고 있는 동안만 유효하다. 오래 유지할 데이터는 복사한다.

## 두 배달 경로

### take / peek (직접 당김) — 수명은 호출자 소유

호출자가 view를 받아 들고 있으므로 해제는 그 view를 놓는 시점이다. `peek()`과 `take()`는 borrow를 쥐는 방식이 같다(고르는 슬롯·커서만 다르다).

C++ — FrameView는 move-only, RAII다. 소멸(스코프 종료) 또는 `release()`에서 해제한다. 결정적이다.

```cpp
{ flux::FrameView v = ch.take(); use(v); }            // 블록 끝 = 소멸 = 해제
flux::FrameView v = ch.take(); use(v); v.release();   // 명시 해제 (이후 v는 빈 view)
```

Python — numpy view는 refcount(GC) 수명이다. numpy 객체(및 그걸 base로 하는 슬라이스)의 참조가 0이 되어 GC될 때 해제한다. host 경로에는 Python에 노출된 명시 release가 없다. host bf16 프레임은 `flux.Frame`으로 오지만 수명은 같다. 해제가 감소 하나뿐이라 GC 시점에 풀린다. 스코프가 필수인 것은 stream을 선언한 GPU 채널뿐이다(아래 GPU 절).

```python
v = sub.take(); use(v); v = None   # (del / 재대입 / 스코프 종료) 참조 0 -> 해제
```

### 콜백 (executor 배달) — 콜백 동안 프레임워크 소유

executor가 view를 만들어 콜백에 넘기고, 콜백이 끝나면 자기 참조를 버린다.

C++ — 콜백은 `void(const FrameView &)`로 받는다. FrameView는 move-only·복사 불가라 콜백 밖으로 빼돌릴 수 없다. 콜백이 리턴하면 그 view가 소멸해 **항상 결정적으로 해제**된다. 유지하려면 바이트를 복사한다.

```cpp
// flux::ros::Subscription 콜백
[](const flux::FrameView & v) { use(v); };   // 리턴 = 해제. 못 빼돌림 -> 유지하려면 복사
```

Python — 콜백에 numpy view를 넘긴다. 콜백이 리턴하고 참조를 안 남겼으면 그 자리에서 GC돼 해제된다. 참조를 남기면(예: `self.last = v`) 그게 GC될 때까지 borrow가 유지된다(`max_borrow` 범위 내에서 허용). 콜백 후 강제 무효화는 하지 않는다.

```python
def on_msg(v):
    use(v)            # 안 남기면 콜백 끝에 해제
    # self.last = v   # 남기면 유지 (max_borrow 압박, GC까지)
```

## GPU 채널 (`Device::Cuda`) — 해제가 곧 stream fence다

선언된 stream이 있으면 해제 자리에 기다림이 하나 붙는다. `refcount`를 줄이기 전에 stream을 기다린다 — 안 그러면 소비 커널이 읽는 중인 슬롯을 발행자가 정당하게 가져가 덮는다. 그래서 "언제 놓느냐"가 곧 "언제 GPU를 기다리느냐"다.

C++는 규약이 안 바뀐다. `FrameView`가 이미 RAII라 해제 시점이 결정적이고, 그 자리에 fence가 들어갈 뿐이다. 콜백 경로는 콜백 리턴이 그 시점이다.

Python은 바뀐다. stream을 선언한 구독의 `take()`/`peek()`은 numpy view가 아니라 `flux.Frame`을 준다. `Frame`이 오는 조건은 둘이다. stream 선언이 하나이고 dtype이 bf16인 것이 다른 하나다. `with`가 필요한 것은 앞의 하나뿐이다. `with` 스코프가 borrow를 묶고, 스코프를 나갈 때 stream을 기다린 뒤 해제한다. 스코프 밖에서 `__cuda_array_interface__`를 부르면 거절한다 — GC 수명으로 돌아가면 임의의 스레드가 임의의 시점에 stream을 기다리게 되고, 그것이 이 스코프가 막는 것이다.

```python
with sub.take() as v:
    use(cp.asarray(v), v.stream)
# 스코프 종료: stream 대기 후 해제
```

fence가 실패하면 borrow를 놓지 않는다. 확인하지 못한 완료를 완료로 치는 대신 슬롯 하나를 잠근다. 그 슬롯은 다시 안 쓰이고 `max_borrow` 리스도 같이 잠긴 채 남는다 — `Channel::fence_failed()`가 세고, 리스가 `max_borrow`개 새면 그 소비자는 borrow를 아예 못 하게 되어 거기서 끝난다(`qos.ko.md` 1). 프로세스가 죽으면 아래 crash 회수가 슬롯을 되돌린다.

fence의 단위는 view가 아니라 stream이다. view를 오래 쥐면 그 해제가 그동안 줄에 쌓인 일까지 문다. 그래서 GPU 채널에서는 받아 쓰고 바로 놓는 것이 유일한 최적해다. `max_borrow`를 올려도 파이프라인이 깊어지지 않는다.

## 해제됨 / 해제 안 됨

| 경로 · 언어 | 해제됨 | 해제 안 됨 (유지) |
| --- | --- | --- |
| take C++ | 스코프 종료 · `release()` · 재대입 | 멤버·컨테이너 보관 · move로 소유권 이동 |
| take Python | 참조 0 -> GC | 멤버·리스트 보관 · 슬라이스 `v[a:b]` · 무복사 래핑 · 순환참조 |
| 콜백 C++ | 콜백 리턴 (항상, 결정적) | 불가 -- const ref·move-only라 못 빼돌림 |
| 콜백 Python | 콜백 리턴 (참조 안 남기면) | `self.x = v` 등 참조 남기면 GC까지 |
| take Python (cuda) | `with` 스코프 종료 (stream 대기 후, 결정적) | 스코프 밖으로 뺀 `flux.Frame` — 쓰려 하면 거절이다 |

함정은 둘이다. Python GC가 비결정적이라는 것, 그리고 슬라이스와 무복사 래핑이 view를 계속 붙잡는다는 것.

복사는 borrow를 쥐지 않는다. `np.array(v)`는 독립 메모리다. 원본 `v`를 놓아야 해제되고, 복사본은 그 뒤에도 안전하다.

## crash 시 (놓기 전에 죽으면)

구독자가 view를 쥔 채 프로세스가 죽으면 refcount가 남는다. 발행자가 슬롯이 굶을 때 죽은 borrower를 OFD 락으로 판정해 그 refcount를 되돌린다(`reclaim_dead`). 영원히 새지 않는다. 단 정상 경로에서는 빨리 놓는 것이 원칙이다.

## 규칙 (요약)

1. 콜백: 콜백 안에서 처리하고 밖으로 안 빼돌린다 -> 자동 해제. C++은 강제, Python은 참조를 안 남기면.
2. take: 스코프를 좁게. C++은 블록, Python은 짧은 함수·`del v`.
3. 유지할 데이터는 복사한다(`np.array(v)` / memcpy) -> borrow 즉시 자유, 복사본은 마음대로.

view는 빌린 참조다. 쥐고 있는 동안만 유효하고, 남길 데이터는 복사한다.

## 언어별 결정성 차이

- C++: FrameView RAII로 결정적. 콜백은 const ref·move-only라 애초에 못 빼돌려 항상 콜백 끝에 해제된다.
- Python(host): numpy GC 수명이라 참조가 남으면 해제가 비결정적이다. 콜백 후 view를 무효화하는 규약(유지하려면 복사 강제)은 현재 미채택 -- 채택하면 host 경로도 콜백이 C++처럼 결정적이 된다. 열린 결정.
- Python(cuda): 결정적이다. `flux.Frame`의 `with` 스코프가 그것을 강제한다 -- 해제 시점이 stream 동기화 시점이라 비결정성을 남길 수 없었다(위 GPU 절). host 경로의 열린 결정을 닫으면 두 경로가 같아진다.
