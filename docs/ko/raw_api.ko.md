# flux 원시 경로 API

`.msg` 없이 쓰는 표면이다. 기본 경로는 [api.md](api.ko.md)이고 이 문서는 그것으로 안 되는 경우만 다룬다. ROS 없이 엔진만 쓰는 표면은 [core_api.md](core_api.ko.md).

## 1. 언제 이걸 쓰나

`.msg`를 붙이면 생성기가 배치와 fingerprint를 양쪽에 박고, `Builder`와 `View`가 프레임을 대신 채우고 읽는다. 그 경로에서는 이 문서의 이름이 하나도 안 나온다. 원시 경로는 그 계약을 만들 재료가 없을 때만 쓴다.

| 상황 | 왜 `.msg`로 안 되나 |
| --- | --- |
| dtype이나 ndim이 프레임마다 바뀐다 | fingerprint는 채널 수명 동안 고정이다 |
| 스키마가 ROS IDL로 표현이 안 된다 | `bool[]`, `T[<=N]`, `wstring`, `bfloat16`은 생성기가 거절한다 |
| 발행 쪽이 이미 만들어진 버퍼를 들고 있다 | 슬롯 밖 데이터라 어차피 1복사다 |
| dGPU payload | flat wire는 host 배치라 device 슬롯에서 못 쓴다 |

원시 경로에는 타입 계약이 없다. `fingerprint`를 `flux::kNoSchema`(Python `flux.NO_SCHEMA`)로 두면 토픽 이름이 같은 무엇이든 붙는다. 직접 상수를 정해 넣을 수는 있지만 그건 스스로 규약을 만들고 스스로 지키는 것이다. 스키마가 바뀌어도 숫자가 안 따라 움직이므로 구버전 peer를 못 막는다.

## 2. FrameMeta와 Published

`FrameMeta`는 프레임마다 슬롯에 함께 실리는 서술자다. 무엇이 담겼는지가 아니라 얼마나 담겼는지를 말한다. 소비자가 `FrameView::meta()`로 읽는다.

```cpp doc:frame_meta
std::uint8_t ndim = m.ndim;
flux::DType dtype = m.dtype;
std::uint32_t itemsize = m.itemsize;
std::uint64_t nbytes = m.nbytes;
std::uint64_t rows = m.shape[0];
```

발행 API는 `FrameMeta`를 받지 않는다. 발행자가 말하는 것은 `dtype`과 `shape` 둘이고 나머지는 유도된다.

| 칸 | 어디서 나오나 |
| --- | --- |
| `dtype` · `shape` | 발행자가 `publish`나 `loan`에 넘긴 값 |
| `itemsize` | `dtype_size(dtype)` |
| `ndim` | `shape`의 길이 |
| `nbytes` | `shape`의 곱 × `itemsize` |

`DType`은 열둘이다 -- `U8` `I8` `U16` `I16` `U32` `I32` `U64` `I64` `F16` `F32` `F64` `BF16`. `shape`은 여덟 칸이고 `ndim`이 그보다 크면 프레임을 만들 수 없다.

서로 어긋날 수 있는 칸이 인자에 없으므로 자기모순인 서술자를 만들 수 없다. `itemsize`가 `dtype`과 다른 프레임, `shape`이 `nbytes`와 안 맞는 프레임은 표현 자체가 안 된다.

발행 결과는 `flux::Published`다.

| 값 | 뜻 | 카운터 |
| --- | --- | --- |
| `Ok` | 커밋됐다 | -- |
| `Backpressure` | 모든 슬롯이 borrow 중이다. 일시적이다 | `dropped`++ |
| `TooLarge` | payload가 `slot_size`를 넘거나 rank가 8을 넘는다. 이미 쓴 핸들을 또 commit해도 이 값이다 | -- |
| `WrongDevice` | device 슬롯에 host 발행을 했다 | -- |
| `FenceFailed` | 선언한 stream을 기다리지 못했다 | `fence_failed`++ |

`Backpressure`만 기다리면 풀린다. 나머지는 배선 실수거나 fault이므로 세지 않고 그 자리에서 알린다. `dropped`가 오르면 그것은 backpressure 하나를 뜻한다.

다섯을 매번 나눠 볼 일은 드물다. 갈라야 하는 선은 하나다 -- 떨어뜨린 프레임인가, 스스로 안 풀리는 fault인가.

```cpp
if (const flux::Published p = w.commit(); flux::faulted(p)) {
  report(flux::to_string(p));
}
```

```python
p = loan.commit()
if flux.faulted(p):
    log(f"publish refused: {p}")
```

`faulted`는 `Ok`와 `Backpressure`에 false, 나머지 셋에 true다. 예제 publisher 전부가 이 모양을 쓴다.

Python도 같은 `flux.Published`를 돌려준다. bool로 접지 않는 이유는 `Backpressure`(best-effort에서 떨어진 프레임)와 `FenceFailed`(슬롯이 영구히 새는 fault)를 호출자가 구분할 수 있어야 하기 때문이다. `Ok`에만 truthy라 `if not loan.commit():`도 뜻대로 읽힌다.

인자를 보고 바로 알 수 있는 것은 Python에서 예외다. 크기 초과·device 배열·상대 topic 이름은 채널에 닿기 전에 `ValueError`가 난다 -- 메시지가 되는 경로를 이름 댈 수 있는 자리이기 때문이다(`contracts.ko.md` 3).

`DType::BF16`은 엔진이 계산하지 않고 나르기만 하는 타입이다. 슬롯은 그대로 바이트 구간이고 `dtype_size`가 2를 돌려주는 것이 전부다. C++17에 bf16 스칼라 타입이 없으므로 소비자가 바이트를 자기 타입으로 해석한다.

## 3. C++

### 1복사 발행

```cpp doc:raw_publish
flux::ros::Publisher pub(*node, "img", flux::kNoSchema, slot_size, slot_count);

if (const flux::Published p = pub.publish(data, flux::DType::U8, {480, 640, 3});
    flux::faulted(p)) {
  report(flux::to_string(p));
}
```

`publish`는 크기를 초과하면 잘라 담지 않고 `TooLarge`로 거절한다. rank가 8을 넘는 것도 같은 값이다. 소비자가 읽기 단계에서 거부하면 커서가 안 움직여 그 슬롯에서 `take()`가 멎기 때문이다.

바이트 수는 인자가 아니라 `shape`의 곱이다. `shape`이 `nbytes`보다 큰 프레임은 만들 수 없으므로, 소비자가 그 shape으로 view를 만들다 터지는 경우도 없다.

평평한 바이트열은 `publish(data, nbytes)`를 쓴다. `u8[nbytes]`로 찍힌다. 생성된 어댑터가 쓰는 것이 이 형태다 -- 스키마가 fingerprint에 있으므로 발행자가 서술할 것이 없다.

### 0복사 발행

```cpp doc:write_slot
flux::WriteSlot w = pub.loan(flux::DType::U8, {480, 640, 3});
if (w) {
  render_into(w.data(), w.capacity());
  w.commit();
}
```

```cpp doc:write_slot_api
bool held = static_cast<bool>(w);
void * buffer = w.data();
std::size_t capacity = w.capacity();
flux::Published published = w.commit(nbytes);
w.abort();
```

찜한 슬롯은 commit·abort 전까지 ring에서 빠진다. 오래 쥐고 있으면 free 슬롯이 줄어 `dropped`가 오른다. `FrameView`와 마찬가지로 move-only다.

`loan()`이 주는 슬롯은 빈 칸이 아니다. 아직 아무도 안 가져간 프레임이 들어 있을 수 있고 `data()`는 그 바이트를 그대로 가리킨다. 거기 쓰는 순간 그 프레임은 사라지므로 `abort`는 되살리지 못한다 -- 발행을 안 할 뿐이다. 사라진 프레임은 뒤처진 소비자의 `lost`에 잡힌다.

`loan`에서 dtype과 shape을 한 번 말했으므로 `commit()`은 인자를 받지 않는다. `commit(nbytes)`는 1차원 loan의 앞 nbytes만 발행하고 `shape[0]`을 다시 계산한다 -- shape을 두 번 말하는 자리는 없다. 1차원이 아닌 loan에 넘기거나 loan보다 크면 `TooLarge`다.

`commit`은 `publish`와 같은 검사를 하고, 걸리면 claim을 되돌린다. 다만 `dropped`는 안 오른다 -- 이미 슬롯을 받은 뒤라 backpressure가 아니기 때문이다. 그 슬롯이 들고 있던 이전 프레임은 함께 사라진다.

### 소비

```cpp doc:raw_subscribe
flux::ros::Subscription sub(
  *node, "img", flux::kNoSchema,
  [](const flux::FrameView & v) {
    const flux::FrameMeta & m = v.meta();
    if (m.ndim == 3 && m.dtype == flux::DType::U8) {
      use(v.data(), v.size());
    }
  },
  flux::QoS{});
```

`meta()`가 돌려주는 것은 다른 프로세스가 쓴 값이다. 같은 버전의 flux가 쓴 것이라면 `shape`의 곱은 `nbytes`와 같다 -- 발행 경로가 그렇게 유도한다. 다른 구현이나 손상된 세그먼트를 상대할 때는 그 값으로 view의 크기를 정하기 전에 검사한다.

## 4. Python

Python에는 `FrameMeta`가 노출되지 않는다. 배열이 `dtype`과 `shape`을 함께 말하므로 둘이 어긋날 자리가 없다.

### 1복사 발행

```python doc:raw_py_publish
pub = flux.ros.Publisher(node, "img", fingerprint=FP, slot_size=16 << 20, slot_count=16)

pub.publish(arr)
```

`arr`은 C-contiguous여야 한다. 슬롯을 초과하면 `ValueError`다.

### 0복사 발행

```python doc:raw_py_loan
loan = pub.loan((480, 640, 3), dtype="uint8")
if loan:
    loan.array[:] = frame
    loan.commit()
```

```python doc:raw_py_loan_api
loan = pub.loan(pub.slot_size, dtype="uint8")
held = loan.valid
writable = loan.host_addressable
published = loan.commit(nbytes=1024)
loan.abort()
```

`pub.loan(...)`이 돌려주는 것은 `flux.Loan`이다. C++의 `flux::WriteSlot`에 대응하고, 슬롯을 쥔 채로 살아 있다 -- `commit()`이나 `abort()`가 그것을 놓는다. 빈 슬롯이 없으면 falsy한 것을 돌려주므로 `if loan:`이 그 판정이다.

`commit(nbytes=n)`은 1차원 loan의 앞 n바이트만 발행한다. `shape[0]`은 `n / itemsize`로 다시 계산되므로 shape을 다시 말하지 않는다. 생성된 어댑터가 이걸 쓴다 -- 슬롯 전체를 빌리고 실제로 채운 만큼만 커밋한다.

### 소비

```python doc:raw_py_view
v = sub.take()
if v is not None:
    _sink(v.shape, v.dtype, v.nbytes)
    v = None
```

돌려주는 것은 읽기 전용 numpy view다. 데이터는 공유메모리를 그대로 가리킨다. 마지막 참조가 사라질 때 borrow가 풀리므로 `v = None`으로 놓거나 스코프를 벗어나게 한다.

Python은 받은 meta를 직접 검증한다. `shape`을 곱한 것이 `nbytes`와 같지 않거나 `itemsize`가 `dtype`과 어긋나면 view를 만들지 않고 던진다. C++ 소비자에는 그 검사가 없다.

### bfloat16

flux가 나르는 dtype 중 numpy에 이름이 없는 것은 bf16 하나다. 그래서 이 타입만 경로가 다르다.

```python
loan = pub.loan((4, 8), dtype="bfloat16")     # 문자열. np.dtype()에 안 넘긴다
torch.from_dlpack(loan)[:] = tensor           # 또는 loan.bits로 직접 채운다
loan.commit()

v = sub.take()                                # numpy 배열이 아니라 flux.Frame
t = torch.from_dlpack(v)                      # 진짜 bf16으로 온다
```

`ml_dtypes`로 해석하는 쪽은 `.bits`를 쓴다.

```python doc:raw_py_bits
loan = pub.loan(1024, dtype="uint16")
if loan:
    loan.bits.view(ml_dtypes.bfloat16)[:] = x
    loan.commit()
```

| | |
| --- | --- |
| `pub.loan(shape, dtype=...)` | `"bfloat16"` 문자열을 받는다. `ml_dtypes.bfloat16` 객체도 받는다(설치돼 있으면) |
| `publish(array)` | DLPack을 내는 배열이면 dtype이 bf16이어도 받는다. numpy로는 만들 수 없는 배열이다 |
| `loan.array` · `take()`의 numpy 배열 | 안 나온다. `.array`는 numpy를 약속하므로 이유를 붙여 거절한다 |
| `__dlpack__` | 양쪽 다 있다. torch·jax가 여기로 집어간다 |
| `.bits` | 같은 바이트의 unsigned numpy view(bf16이면 `uint16`). `ml_dtypes` 쓰는 쪽은 `.view(ml_dtypes.bfloat16)` |
| `__cuda_array_interface__` | bf16에서는 `RuntimeError`. CAI의 typestr은 numpy 것이라 bf16을 못 적는다 |
| `frame.dtype` | `"bfloat16"`. 다른 타입은 numpy typestr 그대로다 |

`take()`가 `flux.Frame`을 주는 것은 `device="cuda"`일 때와 같은 이유다 -- `__dlpack__`을 다는 자리가 객체이기 때문이다. 다만 host bf16 프레임에는 `with`가 필요 없다. 스코프는 해제가 stream 동기화일 때만 필요하고, host borrow의 해제는 감소 하나라 numpy view처럼 GC 시점에 풀린다.

`ml_dtypes`는 flux의 의존성이 아니다. 쓰고 싶은 쪽이 설치해서 `.bits`를 해석하면 되고, 설치하지 않아도 DLPack 경로는 그대로 돈다.

## 5. 영구 오류와 backpressure

양쪽 다 둘을 나눈다. 표현이 다를 뿐이다.

| | Python | C++ |
| --- | --- | --- |
| 잘못된 타입 | `TypeError` | (표현 불가) |
| slot_size 초과 | `ValueError` | `TooLarge` |
| GPU 슬롯에 host 발행 | `ValueError` | `WrongDevice` |
| stream 기다리기 실패 | `Published.FenceFailed` | `FenceFailed`, `fence_failed`++ |
| meta 불량 | (표현 불가) | (표현 불가) |
| 슬롯 없음 | `Published.Backpressure`. `loan()`은 `None` | `Backpressure`, `dropped`++ |

Python 열에 `False`는 없다. `publish()`와 `commit()`이 돌려주는 것은 C++과 같은 `flux.Published`이고, 빈 슬롯이 없을 때 `loan()`만 `None`을 돌려준다(2절).

성공할 수 없는 발행은 backpressure가 아니라 배선 실수다. Python은 그것을 던지고 C++은 반환값으로 갈라 놓는다. 어느 쪽이든 `dropped`가 오르는 것은 슬롯이 없었던 경우 하나뿐이다.
