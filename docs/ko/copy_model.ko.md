# flux 복사 모델 (0복사 vs 1복사)

flux에서 복사가 언제 일어나는지다. 읽기는 항상 0복사고, 복사 여부는 발행(쓰기) 쪽에서 갈린다. 형태별 매핑은 [message_shapes.md](message_shapes.ko.md), GPU는 `gpu_copy.md`.

## 읽기 (수신) -- 항상 0복사

수신자는 슬롯을 그대로 가리키는 view를 받는다. 복사가 붙는 조건은 없다. 그 view를 담는 객체만 경우에 따라 갈린다.

```text
C++    : view.data()          -> 슬롯을 alias하는 span
Python : sub.take()           -> 읽기전용 numpy view
         (stream 선언 또는 bf16) -> flux.Frame (__dlpack__ / __cuda_array_interface__)
```

`flux.Frame`은 GPU 전용이 아니다. `device="cuda"`로 stream을 선언한 구독과 dtype이 bf16인 프레임 둘 다 `Frame`으로 온다. numpy가 bf16을 이름 댈 수 없어 `__dlpack__`을 다는 자리가 객체여야 하기 때문이다. host bf16 `Frame`은 `with`가 필요 없다(`borrow_lifetime.ko.md`). 어느 쪽이든 바이트는 슬롯 그대로라 0복사다.

jagged 원소를 잘라 주는 접근자도 같은 성질이다. 생성된 adapter의 `items[i]`가 원소 블록을 그대로 가리킨다(`message_shapes.ko.md` 4).

## 발행 (쓰기) -- 두 경로

| 경로 | 무엇 | 복사 |
| --- | --- | --- |
| loan | 빈 슬롯을 먼저 받아 거기 직접 쓴다 | 0 |
| publish | 이미 딴 데 있는 데이터를 슬롯으로 memcpy | 1 |

```python
# loan (0복사)
loan = pub.loan(shape, dtype)    # 슬롯을 찜하고 핸들을 받는다. 빈 슬롯 없으면 None
compute_into(loan.array)         # 슬롯을 가리키는 쓰기 가능 numpy. 아래 셋에서는 던진다
loan.commit()                    # 여기서 발행. 안 하고 버리면 발행하지 않는다

# publish (1복사)
arr = lib.process(...)           # 딴 메모리(힙)에 있음
pub.publish(arr)                 # 슬롯으로 memcpy 1회
```

`loan.array`는 항상 numpy가 아니다. 셋에서 던진다. `commit()`·`abort()` 뒤에는 슬롯이 이미 남의 것이고, 슬롯이 device 메모리인 dGPU 채널에는 host 주소가 없고, bf16은 numpy가 이름 댈 수 없다. 뒤의 둘은 `loan.bits`·`__dlpack__`·`__cuda_array_interface__`가 대신 받는다.

```cpp
// C++도 같은 짝이다.
flux::WriteSlot w = pub.loan(flux::DType::F32, {rows, cols});
if (w) {
  compute_into(w.data(), w.capacity());
  w.commit();
}
```

## 0복사 되는 경우

| 경우 | 왜 |
| --- | --- |
| 큰 균일 버퍼를 슬롯에 직접 생성 (카메라->이미지, 라이다->클라우드) | 데이터가 슬롯 안에서 만들어진다 |
| 계산 결과를 loan 버퍼에 씀 | 위와 같음 |
| jagged를 builder로 원소마다 슬롯 bulk에 순차 append | 원소가 슬롯 안에서 만들어진다 |

## 무조건 1복사인 경우

| 경우 | 왜 |
| --- | --- |
| `string` / `string[]` | str 객체가 자기 버퍼를 소유한다. tail로 복사해야 하고 alias할 수 없다 |
| 이미 만들어진 데이터 (라이브러리가 자기 버퍼로 반환) | 슬롯 밖에서 만들어진다. 슬롯으로 모아 담는다 |
| 객체 하나씩 만들어 append (`Detection3D[]` 관행) | 흩어진 객체를 슬롯으로 모아 담는다 |

복사 여부는 데이터가 어디서 만들어지는지로 갈린다. 슬롯 안이면 0복사, 밖이면 1복사다. C++인지 Python인지는 영향이 없다.

string은 예외 없이 1복사다. 고정타입은 메모리 구조와 wire 구조가 같아 그대로 놓인다. string은 길이가 가변이고 바이트를 객체가 소유하므로 그 성질이 없다.

## 1복사여도 빠른 이유

```text
plain ROS: 직렬화 + 커널복사 + 구독자마다 복사 + 역직렬화
flux 1복사: memcpy 한 번 (직렬화 없음, 구독자 수와 무관)
flux 0복사: 아무것도 안 함
```

## 현재 상태

| | 상태 |
| --- | --- |
| 읽기 0복사 | 구현됨 (C++ · Python) |
| 발행 1복사 (`publish`) | 구현됨 (C++ · Python) |
| 발행 0복사 (`loan`) | 구현됨 (C++ · Python) |
| `.msg` -> 필드별 배치 adapter | 구현됨 (`flux_gen`, `message_shapes.ko.md`) |
| jagged builder (원소마다 슬롯에 씀) | 구현됨 |

타입을 안 붙이면 API가 다루는 단위는 프레임 하나의 바이트열(+ shape/dtype)이고, 필드별 배치는 호출자가 직접 정한다(`raw_api.ko.md`). `.msg`를 opt-in하면 생성된 `Builder`/`View`가 그 배치를 대신 잡는다(`api.ko.md` 2).

`loan`은 슬롯을 먼저 claim해 열어두므로 커밋 전까지 그 슬롯이 링에서 빠진다. 오래 쥐고 있으면 free 슬롯이 줄어 drop이 는다.
