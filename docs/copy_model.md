# 복사 모델

flux에서 복사가 발생하는 경우다. 읽기에는 복사가 없고, 예외는 외장 GPU에서 CPU와 GPU 사이로 받는 경우 하나다. 그 밖의 복사는 발행 쪽에서 정해진다.

## 읽기: 복사 없음

구독자는 슬롯을 그대로 가리키는 view를 받는다. 직렬화와 역직렬화가 없다.

| 받는 쪽 | 받는 것 |
| --- | --- |
| C++ | `FrameView::data()`. 슬롯 안의 주소다 |
| C++ 어댑터 | `View`의 접근자. 배열 필드는 슬롯을 가리키는 `Span`이다 |
| Python | 읽기 전용 NumPy 배열 |
| Python 어댑터 | `View`의 속성. 배열 필드는 읽기 전용 NumPy 배열이다 |
| GPU 구독 | GPU 주소(`device_ptr()`). Python은 `Frame`의 `__dlpack__`, `__cuda_array_interface__`([gpu.md](gpu.md)) |

구독자 수와 관계없이 프레임은 슬롯에 한 벌뿐이다.

외장 GPU에서는 CPU 메모리와 GPU 메모리가 분리되어 있으므로, 메모리 반대편의 구독자는 take마다 자기 버퍼로 한 번 복사한다. GPU 메모리 채널의 CPU 구독자와 호스트 메모리 채널의 GPU 구독자가 해당한다. 같은 쪽 구독자와 통합 GPU에는 복사가 없다([gpu.md](gpu.md)).

## 발행: 두 경로

| 경로 | 방식 | 복사 |
| --- | --- | --- |
| loan | 빈 슬롯을 먼저 빌려 그 안에서 데이터를 만든다 | 0 |
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

복사 횟수는 데이터를 만드는 위치로 정해진다. 슬롯 안이면 0, 밖이면 1이고, C++와 Python에 차이가 없다.

| 복사가 0인 경우 | 이유 |
| --- | --- |
| 카메라나 라이다 드라이버가 받은 버퍼를 슬롯에 바로 채운다 | 데이터가 슬롯 안에서 만들어진다 |
| 계산 결과를 loan 배열이나 어댑터의 `alloc__*` 배열에 쓴다 | 위와 같다 |
| GPU 커널이 loan의 GPU 주소에 쓴다 | 위와 같다 |

| 복사가 1인 경우 | 이유 |
| --- | --- |
| 라이브러리가 자기 버퍼로 반환한 결과를 `publish()`한다 | 슬롯 밖에서 만들어졌다 |
| `string`, `string[]` 필드 | 문자열 객체의 버퍼에서 슬롯으로 복사한다 |
| ROS 메시지 객체를 `msg_to_frame()`으로 복사한다 | 메시지가 자기 버퍼를 가진다 |

## 복사가 1이어도 빠른 이유

```text
DDS          : 직렬화 + 커널 복사 + 구독자마다 복사 + 역직렬화
flux 복사 1  : memcpy 한 번(직렬화 없음, 구독자 수와 무관)
flux 복사 0  : 없음
```

## 복사가 추가되는 도구

| 경로 | 복사 |
| --- | --- |
| `frame_to_msg()`로 ROS 메시지를 만든다 | 받는 쪽마다 1. 기존 함수가 메시지 객체를 요구하는 경우에만 사용한다 |
| `flux_bridge`로 DDS에 다시 발행한다 | 프레임마다 1 + CDR 쓰기 + DDS 전달 |
| 발행자의 mirror(`mirror_to_ros`)로 DDS에도 발행한다 | ROS 2 구독자가 있는 동안 프레임마다 1 + CDR 쓰기 + DDS 전달. Python은 CDR을 `bytes`로 한 번 더 복사한다 |
| `flux_bag`으로 녹화한다 | 프레임마다 1(슬롯 -> 녹화 버퍼). 슬롯은 바로 반납하고, 파일 쓰기는 다른 스레드가 한다 |
| rviz2 Image 디스플레이 | GPU 텍스처 업로드 1. `Normalize Range`를 활성화하면 CPU가 16-bit와 `32FC1` 프레임을 한 번 더 읽는다. step이 픽셀 크기의 배수가 아닌 행은 CPU에서 먼저 모은다 |

## loan의 비용

loan한 슬롯은 commit이나 abort 시까지 링에서 제외되므로, 오래 유지하면 빈 슬롯이 줄어 `Backpressure`가 늘어난다. 데이터가 이미 다른 버퍼에 있으면 `publish()`가 간단하다. 데이터를 만들 위치를 선택 가능하면 loan으로 복사를 없앤다.
