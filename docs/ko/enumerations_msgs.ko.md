# flux .msg 규칙 (array / enum / 이름)

flux는 데이터의 모양(dtype·배열 종류)과 **최상위 타입 이름**을 본다. 필드 이름과 중첩 타입 이름은 안 본다. ROS 2 메시지 컨벤션과 겹치거나 무관해서 그대로 써도 된다. flatten·fingerprint 규약은 `message_shapes.ko.md`.

## 배열

| 형태 | flux |
| --- | --- |
| `T[]` (unbounded) | 됨 — column(0복사) 또는 jagged(원소 블록 배열) |
| `T[N]` (fixed) | 됨 — 고정 scalar 블록의 count-N 필드 |
| `T[<=N]` (bounded) | 거부 -> 생성 에러 |

권장은 unbounded `T[]`. `T[]`가 다른 `T[]`(중첩 dynamic)를 들면 jagged다.

## enum (정수 상수)

ROS2는 enum이 없다. 정수 상수로 표현한다.

```text
uint16 ERROR_UNSAFE = 1
uint16 ERROR_GNSS   = 3
uint16 type              # 실제 필드
```

- `NAME = value` 줄은 wire 데이터가 아니다. 배치에도 fingerprint에도 안 들어간다.
- 실제 필드(`type`)만 flatten된다 -- 여기선 fixed u16.
- 생성된 adapter는 상수를 named 심볼로 다시 낸다. C++은 `Status::ERROR_GNSS`(`static constexpr`), Python은 `Status.ERROR_GNSS`다.
- 상수 이름은 rosidl 규칙을 따른다: 대문자 시작, `UPPER_CASE`, underscore는 문자·숫자 사이 하나씩. 위반과 중복은 ParseError다.
- adapter가 스스로 정의하는 상수는 언어마다 철자가 다르다. C++은 `kFingerprint`·`kTypeName`·`kScalarBytes`(`static constexpr`), Python은 `FINGERPRINT__`·`TYPE_NAME__`·`SCALAR_BYTES__`다. 어느 쪽도 상수 이름 규칙이 만들 수 없다. C++ 쪽은 소문자 `k`로 시작하고 Python 쪽은 `__`로 끝나는데, 규칙은 대문자로 시작하는 `UPPER_CASE`만 허용한다. 그래서 `.msg`가 이 이름들에 닿지 못한다. 남는 거부는 하나다 -- 상수 이름이 메시지 자신의 클래스명과 같으면 생성이 거부된다.
- string 상수 값은 rosidl과 같다: 주석(`#`)을 떼고, 양끝이 짝 맞는 따옴표면 벗긴다. 따옴표 안의 `#`는 값이다. 같은 `.msg`가 ROS 쪽과 flux 쪽에서 같은 상수 값을 내야 한다.

## 필드 이름

- 필드 이름에 예약어가 없다. adapter는 자기 멤버와 파생 접근자에 doubled underscore를 단다. 필드 이름 규칙이 doubled underscore를 만들 수 없으므로 `.msg`가 이 이름들에 닿지 못한다. `std_msgs/MultiArrayDimension`의 `size`가 최상위든 중첩이든 통과하는 이유다.
- 두 어댑터의 표면은 같지 않다. 아래가 필드 `x`에 대한 대응이다.

| | C++ | Python |
| --- | --- | --- |
| 어댑터 자기 멤버 | `ok__()` · `size__()` · `commit__()` · `build__()` | `size__` · `commit__()` · `build__()` |
| 스칼라 읽기 | `x()` | `x` property |
| 스칼라 쓰기 | `set__x(v)` | `x` property setter |
| 가변 길이 | `x__size()` (string 배열 · struct 배열) · `x().size()` (column) | `len()` |
| 가변 길이 할당 | `alloc__x(n)` | `alloc__x(n)` |
| stamp 읽기 | `x__sec()` · `x__nanosec()` | `x__stamp` -> `(sec, nanosec)` |
| stamp 쓰기 | `set__x__stamp(sec, nanosec)` | `set__x__stamp(sec, nanosec)` |
| frame_id | `x__frame_id()` · `set__x__frame_id(s)` | `x__frame_id` property |

Python에는 `ok__`가 없다. 어긋난 descriptor를 래치 대신 `WireError`로 던지기 때문이다(`message_shapes.ko.md` 9).

- 언어 예약어와 부딪히는 필드 이름은 뒤에 `_`를 붙여 뺀다. 파생 접근자도 그 철자를 쓴다. C++은 C++ 키워드에 적용하고(`double x` -> `double_()`·`set__double_()`), Python은 Python 키워드와 `np`·`self`에 적용한다(`class x` -> `class_`). 두 집합이 다르므로 한쪽에서만 바뀌는 이름이 있다. `double`은 C++에서만 바뀌고 `class`는 양쪽에서 바뀐다. 두 어댑터의 접근자 철자가 갈리는 자리는 여기 하나다.
- 구조체 배열 두 개의 경로가 camel-case로 같은 element 클래스 이름이 되면(`foo_bar`와 `foo.bar` -> 둘 다 `FooBarElem`) 생성이 거부된다. 에러가 두 경로를 명시한다. 필드 하나를 개명하면 풀린다.

상수와 default value의 구분은 이름 바로 뒤에 `=`가 오는지로 한다. "타입 뒤 어딘가에 `=`가 있으면 상수"로 판정하면 `string label "a=b"` 같은 default가 상수로 오인돼 필드가 통째로 사라진다 -- wire 레이아웃과 fingerprint가 같이 바뀌고, 양쪽 어댑터가 같은 오답에 합의하므로 아무 데서도 안 걸린다.

## 파서가 거부하는 것

flux_gen은 애매한 `.msg`를 추측하지 않는다. 파싱 결과가 곧 wire 레이아웃이라 조용히 틀리면 양쪽이 같은 오답에 합의할 뿐 아무도 못 잡는다. 그래서 아래는 파일·줄 번호가 붙은 에러다.

| 입력 | 이유 |
| --- | --- |
| `float64[abc] x` · `float64[-1] x` · `float64[1_0] x` | 배열 크기가 10진 정수가 아니다 |
| `float64[0] x` | 길이 0 고정배열. 동적이면 `T[]` |
| `float64[3 x` · `float64[ 3] x` | 괄호가 안 닫혔다(공백은 토큰을 쪼갠다) |
| `float64[70000] x` | 고정배열 상한(65536) 초과. 큰 데이터는 `T[]` |
| `int32<=4 x` | `<=` 바운드는 string/wstring에만 |
| 같은 이름의 필드 둘 | 중복 필드 |
| 이름 없는 타입, 잘못된 식별자 | 형태 불명 |

flatten 단계에는 별도 상한이 있다: 고정배열의 중첩은 곱해지므로(`A[64]` 안에 `B[64]` 안에 `float64[64]` = 26만 leaf) 총 leaf 수 4096을 넘으면 reject다. reject는 생성 에러이고 plain ROS로 자동 폴백하지 않는다(`message_shapes.ko.md` 7). 실제 ROS jazzy 메시지는 두 자릿수 leaf에 머물러 이 상한과 거리가 멀다.

## 이름 · 단위 접미사

- fingerprint는 **최상위 타입 이름 + flat 레이아웃**이다. 필드 이름과 중첩 타입 이름은 제외한다.
- 그래서 타입 이름을 바꾸거나 패키지를 옮기면 fingerprint가 바뀌고 기존 피어와 안 붙는다. ROS 2도 같다 — DDS가 타입 이름으로 짝을 맞추므로 이름이 다르면 애초에 안 만난다.
- 중첩 구조를 바꾸는 것(필드를 struct로 묶기)은 fingerprint를 안 바꾼다. wire 바이트가 같기 때문이다.
- 필드 이름은 여전히 안 본다. 그래서 flux는 단위 접미사 불일치(`velocity` vs `velocity_kmph`)를 잡지 못한다. 이름 규칙은 사람용이다.

ROS 2가 장치 둘로 하는 일을 fingerprint 하나가 한다.

| ROS 2 | 잡는 것 | flux |
| --- | --- | --- |
| 타입 이름 (DDS 매칭) | 다른 타입끼리 붙는 것 | 최상위 타입 이름이 fingerprint에 |
| 타입 해시 (RIHS01, Iron+) | 같은 타입인데 정의가 어긋난 것 | flat 레이아웃이 fingerprint에 |

`geometry_msgs/Point`와 `geometry_msgs/Vector3`는 둘 다 `float64` 셋이다. 레이아웃만 보면 같은 세그먼트를 쓰게 되고, 위치를 방향으로 읽어도 아무도 안 막는다. rclpy로 확인했다 — 한 토픽에 `Point`를 발행하고 `Vector3`로 구독하면 ROS는 하나도 안 준다. fingerprint에 타입 이름이 들어가면서 flux도 같은 판단을 한다.
