# flux 메시지 형태 (.msg -> wire)

flux는 `.msg`를 flat wire로 편다. 각 멤버가 형태에 따라 슬롯의 어느 영역으로 가는지, 그리고 그 배치를 읽고 쓰는 adapter가 어떻게 나오는지를 아래에 보인다. `.msg` 작성 규칙은 [enumerations_msgs.md](enumerations_msgs.ko.md), 복사 여부는 [copy_model.md](copy_model.ko.md), 코어 동시성은 `design.md`, 세그먼트 이름·fingerprint는 `discovery.md`.

`flux_gen`이 build time에 네 단계를 돈다. 각 단계가 다음 단계의 입력이다.

```text
parse      .msg -> Message (필드·상수)
flatten    Message -> leaf 목록 (중첩 평탄화, tier 판정, 거부)
layout     leaf 목록 -> byte offset
emit       offset -> C++ 헤더 + Python 모듈 (+ ROS 메시지 다리)
```

fingerprint는 flatten 결과에서 나오고 양쪽 산출물에 상수로 박힌다. 두 언어가 같은 트리에서 나오므로 배치가 어긋날 수 없다.

## 1. 프레임 구조

프레임 하나는 포인터 없는 연속 바이트다. 주소가 아니라 프레임 시작점 기준 offset으로만 안쪽을 가리킨다.

```text
프레임 = [ scalar block (크기 고정, offset은 build time에 확정) ][ var region (프레임마다 bump 할당) ]
```

scalar block이 담는 것:

| 멤버 | scalar block에 | var region에 |
| --- | --- | --- |
| 고정 스칼라 `int32 x` | 값 그대로 | -- |
| 고정 배열 `float64[9] m` | 9x8B 연속 | -- |
| 가변 멤버 전부 | descriptor 8B `{uint32 off, uint32 len}` | 실제 데이터 |

descriptor 하나가 "어디에 몇 개"를 기록한다. 읽기는 descriptor를 읽고 그 자리를 view하는 것이고, 쓰기는 cursor를 밀고 descriptor를 적는 것이다. 할당 순서는 자유다 -- 어디에 놓였는지는 descriptor가 기록한다.

offset이 uint32라 생성된 스키마의 프레임 하나는 slot_size와 무관하게 4 GiB가 상한이다.

## 2. 바로 얹히는 것 (0복사)

고정 스칼라 -- `int32 x; float64 y`:

```text
scalar: [ x(4) | pad(4) | y(8) ]        C struct와 같은 자연 정렬
```

고정 배열 `T[N]` -- `float64[9] matrix`:

```text
scalar: [ matrix[0] ... matrix[8] ]     (9x8B 연속, 그 자리에서 읽고 쓴다)
```

동적 배열 `T[]` -- `float32[] data`, N=3:

```text
scalar: [ data.desc = {off, 3} ]
var   : [ data0 data1 data2 ]           (0복사)
```

평행 같은길이 배열 -- `float32[] x, y, z`, N=3:

```text
scalar: [ x.desc | y.desc | z.desc ]
var   : [ x0 x1 x2 ][ y0 y1 y2 ][ z0 z1 z2 ]     (열마다 0복사, 각자 정렬)
```

## 3. 펴서 얹히는 것 (중첩)

중첩은 wire에서 이름 prefix일 뿐이다. 안의 멤버가 각자 형태대로 제 영역으로 간다.

고정 중첩 struct -- `geometry_msgs/PoseStamped`의 `pose`:

```text
scalar: [ ... pose_position_x(8) pose_position_y(8) pose_position_z(8)
              pose_orientation_x(8) ... pose_orientation_w(8) ]
```

접근자 이름은 경로를 `_`로 이은 것이다 (`pose.position.x` -> `pose_position_x`). 두 필드가 같은 이름으로 합쳐지면 (`a.b`와 `a_b`) 생성이 거부된다. 두 경로가 같은 element 클래스 이름을 만들 때도 거부다. adapter가 자기 몫으로 쓰는 이름은 doubled underscore를 달아 `.msg`가 닿지 못하게 한다([enumerations_msgs.md](enumerations_msgs.ko.md)).

가변 멤버 든 단일 중첩 (hoisting) -- `Polygon polygon` (안에 `Point32[] points`):

```text
polygon이 사라지고 안의 points가 위로:
scalar: [ polygon_points.desc = {off, N} ]
var   : [ x0 y0 z0 ][ x1 y1 z1 ] ...
```

중첩을 재구성해도 (필드를 struct로 묶거나 풀어도) 배치가 안 바뀌고 fingerprint도 안 바뀐다. 두 성질은 같이 가야 한다 -- fingerprint가 같은데 배치가 다르면 fingerprint가 못 잡는 유일한 어긋남이 된다.

## 4. 원소 배열 (record · jagged)

원소가 여럿인 배열은 record든 jagged든 한 가지 방법으로 얹는다. var region에 원소 블록 배열을 놓는다. 블록 크기(stride)가 고정이라 `items[i]`가 O(1)이다.

원소가 전부 고정이면 record column이다 -- `Point32[] points`, N=2:

```text
scalar: [ points.desc = {off, 2} ]
var   : [ x0 y0 z0 ][ x1 y1 z1 ]        (원소 12B 고정, 0복사)
```

원소 안에 가변 멤버가 있으면 jagged다. 달라지는 건 하나뿐이다 -- 원소 블록이 자기 descriptor를 들고, 그게 var region 바깥쪽을 가리킨다.

`sensor_msgs/PointCloud2`의 `PointField[] fields` (원소 = `string name; uint32 offset; uint8 datatype; uint32 count`):

```text
scalar: [ fields.desc = {off, 3} ]
var   : [ 원소0: name.desc offset datatype count ]   원소 블록 20B
        [ 원소1: name.desc offset datatype count ]
        [ 원소2: name.desc offset datatype count ]
        [ "x" ][ "y" ][ "z" ]                        원소들의 string 바이트
```

깊이는 재귀로 그대로 들어간다. 설치된 ROS jazzy 메시지에서 jagged 중첩은 최대 3단이다.

## 5. string (그 멤버만 1복사)

string 스칼라 -- `string label = "hello"`:

```text
scalar: [ label.desc = {off, 5} ]
var   : [ "hello" ]                     (1복사, 작음)
```

string 배열 -- `string[] names = ["ab","cde"]`:

```text
scalar: [ names.desc = {off, 2} ]
var   : [ desc0 | desc1 ][ "ab" ][ "cde" ]
```

숫자 column은 0복사를 유지하고 string 바이트만 복사된다.

## 6. Header / bare Time

`std_msgs/Header`와 bare `builtin_interfaces/Time`은 필드로 펴지 않고 정해진 모양으로 얹는다.

```text
Header: [ sec(int32) | nanosec(uint32) | frame_id.desc(8) ]     16B
Time  : [ sec(int32) | nanosec(uint32) ]                         8B
```

payload 안에 있다. `flux_core`는 ROS를 모르므로 프레임 메타(`FrameMeta`)에는 stamp도 frame_id도 없다 -- 그 자리를 만들면 엔진이 ROS 의미를 지게 된다. 수신자는 어차피 프레임 전체를 0복사로 들고 있으므로 payload에서 읽는 비용과 메타에서 읽는 비용이 같다.

중첩 안의 Header는 특별하지 않다 — 보통 struct처럼 평탄화된다. 프레임 stamp는 하나, 최상위 것이다.

## 7. 안 되는 것 (거부 -> 생성 에러)

| 형태 | 예 | 왜 |
| --- | --- | --- |
| bounded 배열 `T[<=N]` | `float64[<=10] vals` | 용량 상한은 dynamic wire에 무의미 |
| `bool[]` | `bool[] flags` | `std::vector<bool>`가 비트팩이라 memcpy 불가 |
| `wstring` | `wstring s` | 와이드 문자열 인코딩 |
| 필드 없음 | `std_msgs/Empty` | 실을 payload가 없다 |
| 재귀 타입 | `A`가 `A`를 품음 | flat wire로 못 편다 |
| leaf 4096개 초과 | `A[64] a`가 `B[64] b`를 품고... | 고정 중첩이 곱으로 터진다 |

거부는 에러다. 자동 폴백 경로는 없다. `flux_gen` CLI는 `SystemExit`로 던지고(`flux_gen/flux_gen/cli.py`), `emit_cpp`·`emit_py`·`emit_ros`는 `ValueError`로 던진다. `flux_generate_adapters()`를 쓴 빌드는 그 자리에서 실패한다. 그 메시지를 plain ROS로 보내려면 호출자가 그것을 `flux_generate_adapters()`에 넣지 않는다.

## 8. 한눈에 -- 결정 트리

```text
멤버가...
├─ 고정 스칼라 / T[N]                        -> scalar block (0복사)
├─ 고정타입 T[] / 고정 struct T[]             -> descriptor + var region (0복사)
├─ 중첩 struct                               -> 펴서 (평탄화 / hoisting)
├─ string / string[]                         -> descriptor + var region (그 부분만 1복사)
├─ 가변 원소의 배열 (jagged)                  -> descriptor + 원소 블록 배열
├─ Header / bare Time                        -> 고정 위치 (16B / 8B)
└─ T[<=N] / bool[] / wstring / 필드없음       -> 거부 -> 생성 에러
```

거부되는 것은 마지막 줄뿐이고 나머지는 전부 얹힌다. 어느 메시지가 거부되는지는 설치된 패키지 집합에 좌우되므로 비율도 개수도 여기 적지 않는다 — `flux_gen`으로 그 기계에서 직접 센다.

## 9. 생성물

`flux_generate_adapters()`가 메시지마다 어댑터 한 쌍과 ROS 다리 한 쌍을 낸다. 쓰는 법은 [api.md](api.ko.md) 2절.

```text
include/<pkg>/flux/<snake>.hpp       <pkg>::flux_msg::<Msg>   -- kFingerprint, View, Builder
<pkg>_flux/<snake>.py                <pkg>_flux.<snake>.<Msg> -- FINGERPRINT__, View, Builder
include/<pkg>/flux/<snake>_ros.hpp   msg_to_frame · frame_to_msg  (ROS 메시지 객체 다리)
<pkg>_flux/<snake>_ros.py            msg_to_frame · frame_to_msg
```

기본 어댑터(`.hpp`/`.py`)는 얇다. 접근자 하나가 상수 offset으로 런타임([`flux/wire.hpp`](../../flux_core/include/flux/wire.hpp) · `flux_gen/flux_gen/wire.py`)을 한 번 호출한다. 메시지별 런타임은 없고, 맞춰야 할 것은 offset뿐이다.

`_ros` 다리는 따로 나온다. `msg_to_frame`은 ROS 메시지 객체의 모든 필드를 슬롯에 복사하고 `frame_to_msg`는 반대로 한다 -- 둘 다 1복사 이상이라 `Builder`/`View`를 직접 쓰는 0복사 경로와 다르다(`copy_model.ko.md`, `api.ko.md` 2). rosidl 메시지 타입을 include하는 부분이 여기로 격리돼 있어, 안 쓰는 패키지는 rclcpp·rclpy를 안 본다.

adapter는 host payload 전용이다. `View`·`Builder`가 `FrameView::data()`·`WriteSlot::data()`를 그대로 `wire` 런타임에 넘기고 그 위에서 host load·store를 하므로, 슬롯이 device 할당인 dGPU 채널에서는 쓸 수 없다. 그 채널에서 `data()`는 `nullptr`이라 `View`·`Builder`가 `ok__() == false`가 되고 `commit()`이 거부한다 -- 조용히 GPU 메모리에 host store를 하는 대신이다. dGPU에서 발행하고 받는 길은 `device_ptr()`과 `stream()`이다(`api.ko.md` GPU 절). iGPU(`ShmDirect`)는 슬롯이 host 메모리이기도 해서 adapter가 그대로 돈다.

수신 쪽은 프레임을 다른 프로세스가 썼다고 보고 descriptor를 하나도 안 믿는다. 범위와 정렬을 다 검사하고, 어긋나면 C++은 `ok__()`가 false로 래치되고 Python은 `WireError`를 던진다. 프레임 밖을 가리키는 포인터를 내주는 경로는 없다.

`frame_to_msg`에서 프레임이 든 배열 길이가 스키마의 `T[N]`과 다르면 두 언어 모두 명시 에러다 — C++은 `std::length_error`, Python은 `ValueError`. 길이는 프레임이 들고 온 데이터고 N은 스키마다. 잘라 맞춘 복사는 조용히 틀린 메시지가 된다.

쓰기 쪽도 같은 길이를 강제한다. `Builder`에 `T[N]` 필드를 N 아닌 길이로 쓰면 Python은 `ValueError`를 던지고 C++은 writer를 poison해 `ok__()`가 false로 래치되어 `commit()`이 거부한다 — 틀린 프레임은 만들어지지 않는다. `frame_to_msg`의 검사는 남의(외부·구버전) writer가 만든 프레임을 막는 마지막 방어선이다.
