# 복사 모델

flux에서 복사가 언제 일어나는지 적는다. 읽기는 언제나 복사가 없고, 복사 여부는 발행 쪽에서 갈린다.

## 읽기: 복사 없음

구독자는 슬롯을 그대로 가리키는 view를 받는다. 직렬화도 역직렬화도 없다.

| 받는 쪽 | 받는 것 |
| --- | --- |
| C++ | `FrameView::data()`. 슬롯 안 주소다 |
| C++ adapter | `View`의 접근자. 배열 필드는 슬롯을 가리키는 `Span`이다 |
| Python | 읽기 전용 numpy 배열 |
| Python adapter | `View`의 속성. 배열 필드는 읽기 전용 numpy 배열이다 |
| GPU 구독 | 장치 주소(`device_ptr()`), Python은 `Frame`의 `__dlpack__`·`__cuda_array_interface__`([gpu.md](gpu.md)) |

구독자가 몇이든 프레임은 슬롯에 한 벌뿐이다.

## 발행: 두 경로

| 경로 | 무엇 | 복사 |
| --- | --- | --- |
| loan | 빈 슬롯을 먼저 빌려 데이터를 거기서 만든다 | 0 |
| publish | 다른 곳에 있는 데이터를 슬롯으로 복사한다 | 1 |

```cpp doc:cpp_loan
constexpr std::size_t kPoints = 1024;
flux::WriteSlot slot = pub.loan(flux::DType::F32, {kPoints, 3});
if (slot) {
  auto * xyz = static_cast<float *>(slot.data());
  std::fill_n(xyz, kPoints * 3, 0.0F);
  if (flux::faulted(slot.commit())) {
    RCLCPP_ERROR(node->get_logger(), "commit failed");
  }
}
```

```python doc:py_loan
loan = pub.loan((1024, 3), "float32")
if loan is not None:
    loan.array[:] = 0.0
    loan.commit()
```

복사 여부는 데이터가 어디서 만들어지는지로 정해진다. 슬롯 안에서 만들어지면 0, 밖에서 만들어지면 1이다. C++인지 Python인지와는 상관없다.

| 0 복사가 되는 경우 | 이유 |
| --- | --- |
| 카메라·라이다 드라이버가 받은 버퍼를 슬롯에 바로 채운다 | 데이터가 슬롯 안에서 만들어진다 |
| 계산 결과를 loan 배열이나 adapter의 `alloc__*` 배열에 쓴다 | 같다 |
| GPU kernel이 loan의 장치 주소에 쓴다 | 같다 |

| 1 복사인 경우 | 이유 |
| --- | --- |
| 라이브러리가 자기 버퍼로 돌려준 결과를 `publish()`한다 | 슬롯 밖에서 만들어졌다 |
| `string`, `string[]` 필드 | 문자열 객체가 자기 버퍼를 가진다. 슬롯으로 옮겨 적는다 |
| ROS 메시지 객체를 `msg_to_frame()`으로 옮긴다 | 메시지가 자기 버퍼를 가진다 |

## 1 복사여도 빠른 이유

```text
DDS        : 직렬화 + 커널 복사 + 구독자마다 복사 + 역직렬화
flux 1 복사 : memcpy 한 번(직렬화 없음, 구독자 수와 무관)
flux 0 복사 : 없음
```

## 복사가 붙는 도구

| 경로 | 복사 |
| --- | --- |
| `frame_to_msg()`로 ROS 메시지를 만든다 | 받는 쪽마다 1. 기존 함수가 메시지 객체를 요구할 때만 쓴다 |
| `flux_bridge`로 DDS에 다시 발행한다 | 프레임마다 1 + CDR 쓰기 + DDS 전달 |
| `flux_bag`으로 녹화한다 | 프레임마다 1(슬롯 -> 녹화 버퍼). 슬롯은 바로 돌려주고 파일 쓰기는 따로 한다 |
| rviz2 Image display | GPU 텍스처 업로드 1. `Normalize Range`를 켜면 CPU가 16비트와 `32FC1` 프레임을 한 번 더 읽는다. step이 픽셀 크기의 배수가 아닌 행은 CPU에서 먼저 모은다 |

## loan의 비용

loan은 commit이나 abort까지 슬롯을 링에서 뺀다. 오래 쥐면 빈 슬롯이 줄어 `Backpressure`가 늘어난다. 데이터가 이미 다른 버퍼에 있으면 `publish()`가 간단하다. 데이터를 만들 곳을 정할 수 있으면 loan으로 복사를 없앤다.
