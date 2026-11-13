# GPU(CUDA)

GPU에서 만들고 GPU에서 읽는 프레임도 같은 채널 API로 보낸다. 채널 생성 시 CUDA 스트림을 선언하면 GPU 채널, 선언하지 않으면 호스트 채널이다. 이 문서는 GPU 채널에만 적용된다.

## 경로

flux는 `libcuda.so.1`을 실행 시점에 로드하고, GPU마다 아래 표에서 조건이 처음 맞는 경로를 고른다.

| 경로 | 조건 | GPU 예 | 슬롯 데이터 |
| --- | --- | --- | --- |
| `ShmRegistered` | 통합 GPU이고 호스트 메모리 등록을 지원한다 | Jetson Orin, Thor | 공유 메모리 매핑을 `cuMemHostRegister`로 등록한다 |
| `ShmDirect` | 통합 GPU이고 pageable memory에 접근한다 | | 공유 메모리 매핑 주소를 그대로 사용한다 |
| `DeviceHandle` | VMM과 POSIX fd export를 지원한다 | 외장 GPU | GPU 메모리를 조각으로 잡아 조각마다 fd로 넘긴다 |
| `None` | 그 밖 | GPU 없음 | GPU 채널 생성이 불가능하다 |

`probe(device = 0)`는 GPU 하나의 경로를 예외 없이 반환한다. 경로가 `None`이면 `reason`에 이유가 담긴다.

```cpp
const flux::gpu::Platform p = flux::gpu::probe();   // p.route, p.reason
```

```python
p = flux.probe()                                     # p.route, p.reason
```

확인한 장비는 Jetson AGX Orin(JetPack 7.2, `ShmRegistered`)과 RTX 4070 SUPER(`DeviceHandle`)다.

## C++

```cpp doc:cpp_gpu_publish
flux::Publisher pub(*node, "depth", flux::kNoSchema, 4U << 20U, 4, flux::Device::Cuda);

flux::WriteSlot slot = pub.loan(flux::DType::F32, {480, 640});
if (slot) {
  launch_my_kernel(slot.device_ptr(), slot.capacity(), slot.stream());
  if (flux::faulted(slot.commit())) {
    RCLCPP_ERROR(node->get_logger(), "commit: the stream could not be waited for");
  }
}
```

```cpp
flux::Subscription sub(*node, "depth", flux::kNoSchema, flux::QoS{}, callback, flux::Device::Cuda);
```

```cpp doc:cpp_gpu_read
if (flux::FrameView frame = sub.take()) {
  read_on_device(frame.device_ptr(), frame.size(), frame.stream());
}
```

`launch_my_kernel`과 `read_on_device`는 사용자의 커널 호출 자리다.

- `flux::Device::Cuda`를 넘기면 flux가 스트림을 생성해 선언한다. 직접 만든 스트림은 `flux::gpu::Stream::from_native(cudaStream_t)`로 넘기고, 채널과 view보다 오래 유지한다. legacy default 스트림(`nullptr`)은 스레드마다 다른 스트림이므로 거절한다.
- 주소는 `device_ptr()`로 얻고, 커널은 `stream()`의 채널 스트림에 올린다. 다른 스트림의 작업은 flux가 기다리지 않는다.
- `commit()`은 스트림의 작업이 끝난 뒤 발행하므로, 구독자는 받은 프레임을 CPU 채널처럼 바로 읽어도 된다.
- view를 해제하면 스트림의 작업이 끝난 뒤 슬롯이 반납된다. 구독의 GPU 매핑은 읽기 전용이다(`ShmRegistered`에서는 GPU가 읽기 전용 등록을 지원하는 경우).

## Python

```python doc:py_gpu_publish
pub = flux.Publisher(node, "depth", slot_size=4 << 20, slot_count=4, device="cuda")

loan = pub.loan((480, 640), "float32")
if loan is not None:
    depth = torch.from_dlpack(loan)
    depth.fill_(1.5)
    loan.commit()
```

```python
sub = flux.Subscription(node, "depth", device="cuda")
```

```python doc:py_gpu_read
frame = sub.take()
if frame is not None:
    with frame:
        depth = torch.from_dlpack(frame)
        print(depth.shape, float(depth.mean()))
```

- `device="cuda"` 구독은 numpy 배열 대신 `flux.Frame`을 반환한다. 메모리는 `with frame:` 블록이나 콜백 안에서만 제공하고, `shape`, `dtype`, `nbytes`, `stream`은 언제든 읽기 가능하다.
- `Frame`과 `Loan`은 `__dlpack__`과 `__cuda_array_interface__`를 제공한다. torch는 읽기 전용인 `__cuda_array_interface__`를 받지 않으므로 `torch.from_dlpack()`을 사용한다. bf16은 `__dlpack__`으로만 받는다.
- flux는 `__dlpack__(stream=s)`로 받은 소비자 측 스트림(torch의 현재 스트림)도 해제나 commit 전에 상한 없이 기다린다. 그래서 torch가 자기 스트림에서 읽고 쓰는 동안 슬롯이 넘어가지 않는다.
- 이 대기가 실패하면 view나 loan은 `fence_failed`에 세지 않고 점유를 유지하며, 그 뒤의 take는 `refused.max_borrow`로 거절된다.
- GPU 배열을 `publish()`에 넘기면 `TypeError`다. GPU 데이터는 loan에 쓴다.

`Frame`의 메모리는 범위가 끝나는 시점에 반납하고, 제공한 메모리가 아직 사용 중이면 늦춘다.

| 제공한 방법 | 슬롯 반납 시점 |
| --- | --- |
| 없음 | `with`나 콜백 종료 시 |
| `__dlpack__` | 범위가 끝나고 그 캡슐로 만든 텐서가 모두 사라진 시점 |
| `__cuda_array_interface__` | `Frame` 객체가 사라진 시점 |

## 생성 메시지의 GPU 필드

생성된 어댑터 메시지의 숫자 배열 필드는 GPU에서 쓰고 읽는다. 나머지 필드(헤더, 문자열, 작은 값)는 호스트 슬롯에 저장한다.

```cpp
using Image = sensor_msgs::flux_msg::Image;
flux::Publisher pub(*node, "image", Image::kFingerprint, 1U << 16U, 4, flux::Device::Cuda);

flux::WriteSlot slot = pub.loan();
Image::Builder b(slot);
b.set__encoding("rgb8");
const flux::wire::DeviceSpan<std::uint8_t> data = b.alloc_device__data(bytes);
launch_my_kernel(data.data(), data.size(), slot.stream());
b.commit__();
```

```cpp
const Image::View m(frame);
const flux::wire::DeviceSpan<const std::uint8_t> data = m.data__device();
read_on_device(data.data(), data.size(), frame.stream());
```

- 생성 타입의 fingerprint로 스트림을 선언한 발행자는 외장 GPU에서도 슬롯이 호스트 메모리다(`device_backed()`가 `false`, loan의 `device_ptr()`이 `0`).
- `alloc_device__<필드>(n)`은 그 필드를 발행 프로세스의 GPU arena에 할당한다. 통합 GPU에서는 CPU와 GPU가 함께 읽는 호스트 메모리, 외장 GPU에서는 GPU 메모리다. 이 필드는 채널 스트림에서 쓰고, commit이 그 스트림을 기다린다.
- GPU arena는 프로세스에 GPU마다 하나다. 크기는 호스트 arena와 같은 `FLUX_ARENA_BYTES`(기본값이자 최대값 16 GiB)이고, 실제 GPU 메모리는 블록이 사용하는 만큼 32 MiB 단위로 차지한다. 발행 프로세스는 GPU arena를 1 GiB 사용할 때마다 fd를 32개 사용한다. 블록은 그 프레임의 슬롯에 다음 프레임을 쓰는 시점에 해제된다.
- 받는 쪽의 `<필드>__device()`는 GPU 주소와 개수를 반환한다. 구독 스트림이 발행자와 다른 GPU(UUID)에 있으면 빈 값이다.
- arena가 활성화된 `Builder`는 슬롯의 마지막 71 B(스트림을 선언한 발행자는 135 B)를 가변 필드에 쓰지 않는다.

CPU와 GPU가 서로의 필드를 읽는 경우는 다음과 같다.

| 쓰는 쪽 → 읽는 쪽 | 통합 GPU | 외장 GPU |
| --- | --- | --- |
| `alloc_device__` → CPU 구독자의 일반 접근자 | 복사 없음 | 그 프레임의 첫 `View` 생성 시 GPU 필드만 호스트 버퍼로 복사하고 완료를 기다린다. 같은 프레임의 다음 `View`는 그 복사본을 읽는다 |
| `alloc__`(슬롯 안) → GPU 구독자의 `__device()` | 복사 없음 | take가 만든 프레임 복사본에 있다 |
| `alloc__`(arena) → GPU 구독자의 `__device()` | 빈 값 | 그 프레임의 첫 `View` 생성 시 GPU 버퍼로 복사한다. 복사는 구독 스트림에 걸리고 기다리지 않는다 |

Python에서는 같은 필드가 DLPack 객체로 오간다. GPU 필드는 loan의 스트림에서 쓴다.

```python
loan = pub.loan(dtype="uint8")
b = image.Builder(loan.array, loan)
b.encoding = "rgb8"
with torch.cuda.stream(torch.cuda.ExternalStream(loan.stream)):
    torch.from_dlpack(b.alloc_device__data(nbytes)).copy_(pixels)
b.commit__()
```

```python
with sub.take() as frame, torch.cuda.stream(torch.cuda.ExternalStream(frame.stream)):
    data = torch.from_dlpack(image.View(frame).data__device)
```

- 생성 모듈은 import 시 fingerprint를 등록하므로, `device="cuda"` 발행자도 C++처럼 슬롯이 호스트 메모리다.
- `<필드>__device`와 `alloc_device__<필드>`는 `flux.wire.DeviceSpan`을 반환한다. `DeviceSpan`은 `__dlpack__`, `__cuda_array_interface__`, `nbytes`, `dtype`을 갖고, 받는 쪽 것은 읽기 전용이다. `<필드>__device`는 GPU 주소가 없으면 None이다.
- 생성된 `View`는 `device="cuda"` 구독의 `Frame`도 받는다. 이 `View`는 그 `with` 블록 안에서만 생성한다.
- GPU를 선언하지 않은 발행자의 `alloc_device__<필드>`는 `WireError`이고, 그 `Builder`의 `commit__()`은 `TOO_LARGE`다.
- data가 arena에 있는 tensor 메시지의 `__dlpack__`은 외장 GPU에서 위의 복사본을 반환한다. 통합 GPU에서는 `RuntimeError`다.

## GPU 메모리 채널(`DeviceHandle`)

외장 GPU에서 GPU 발행자의 채널은 데이터가 GPU 메모리에만 있다(생성 메시지의 발행자는 예외, 위 절). CPU 메모리와 GPU 메모리가 물리적으로 분리되어 있으므로, 메모리의 반대편에 있는 쪽은 프레임마다 한 번 복사한다.

| 발행 → 구독 | 통합 GPU | 외장 GPU |
| --- | --- | --- |
| CPU → CPU | 복사 없음 | 복사 없음 |
| GPU → GPU | 복사 없음 | 복사 없음 |
| CPU → GPU | 복사 없음 | 구독자가 take마다 자기 GPU 버퍼로 복사한다. 복사는 구독 스트림에서 실행된다 |
| GPU → CPU | 복사 없음 | 구독자가 take마다 자기 호스트 버퍼로 복사하고, take 안에서 완료를 기다린다 |
| `publish(호스트 배열)` → GPU 메모리 채널 | 해당 없음 | 발행자가 슬롯으로 복사한 뒤 commit한다 |

| GPU 메모리 채널에서 불가능한 것 | 대신 |
| --- | --- |
| loan의 `data()`, Python `loan.array` | 각각 `nullptr`, 예외다. `device_ptr()`이나 `__dlpack__`을 사용한다 |
| 어댑터의 `Builder`, GPU 구독자의 `View` | 프레임 배치가 호스트 메모리를 전제하므로 사용 불가능하다. CPU 구독자의 `View`는 자기 복사본을 읽는다 |

- GPU 메모리 채널의 CPU 구독자가 받는 view는 `data()`가 자기 복사본이고, `device_ptr()`은 `0`이다.
- GPU 메모리 채널에서도 aux는 호스트 메모리다. stamp나 크기 같은 헤더는 발행자를 `aux_size`와 함께 생성해 aux에 쓴다([concepts.md](concepts.md#발행자와-슬롯)). CPU 구독과 GPU 구독 모두 aux를 복사 없이 읽는다.
- 생성 메시지의 `data` 필드를 payload로, `data`를 비운 그 메시지의 프레임을 aux로 보내는 채널은 fingerprint를 `flux::gpu_fingerprint(타입의 fingerprint)`(Python `flux.gpu_fingerprint`, C `flux_gpu_fingerprint`)로 만든다. `data`가 `uint8[]`인 타입에 사용 가능하다. `flux_bridge`, ROS 2 mirror, `convert_flux`, `flux_tools`의 rviz2 디스플레이는 이 채널의 aux와 payload를 그 타입의 메시지 하나로 합쳐 읽는다. 외장 GPU에서는 payload를 호스트로 한 번 복사한다. `flux_wrapper`의 `Cuda` 발행자가 이 형식이다.
- 복사가 실패한 take는 빈 결과이고 `refused.fence`로 센다. 그 슬롯은 반납하지 않는다.
- 구독자는 메모리를 할당한 GPU를 번호가 아니라 UUID로 찾으므로, `CUDA_VISIBLE_DEVICES`가 달라도 같은 GPU를 찾는다. 구독 스트림의 GPU가 그 GPU와 다르면 `MISMATCH`다.
- GPU 메모리 fd는 `/dev/shm/flux.gpu.<dev>.<ino>` 소켓으로 넘긴다. 세그먼트에 접근 가능한 곳에서는 이 소켓에도 접근 가능하다.

통합 GPU(`ShmRegistered`, `ShmDirect`)에서는 슬롯이 호스트 메모리이기도 하므로 복사 없이 한 채널에서 CPU 구독과 GPU 구독이 함께 동작한다.

## 대기 상한과 해제 방식

| 설정 | C++ | Python | 기본값 |
| --- | --- | --- | --- |
| 완료 신호 대기 상한 | `set_fence_timeout(ns)` | `fence_timeout=초` | 상한 없음 |
| 해제 방식 | `set_release_mode(gpu::ReleaseMode::Deferred)` | `release="deferred"` | `Wait` |

- 상한이 없으면 GPU가 멈춘 경우 완료 신호를 기다리는 executor 스레드도 멈춘다. 상한을 넘긴 대기는 실패로 세고, commit은 `FenceFailed`를 반환하며, 슬롯은 작업 완료 시까지 점유를 유지한다.
- `Wait`에서는 해제가 스트림을 기다린 뒤 반환된다. `Deferred`에서는 해제가 즉시 반환되고, 슬롯은 GPU 사용이 끝난 뒤 다음 take나 executor pass에 반납된다. 반납 시까지 그 view는 `max_borrow`에 포함된다. `Deferred`는 콜백이 GPU 작업을 건 채 끝나는 노드에서 사용한다.
- 완료 신호 대기가 실패한 슬롯은 반납하지 않는다. 다시 쓰면 다른 쪽이 아직 사용 중인 메모리를 덮기 때문이다. 해제가 실패한 view가 `max_borrow`를 채우면, 그 구독의 take는 `refused.fence`로 센다.

| 값 | 뜻 |
| --- | --- |
| `fence_failed()` / `fence_failed` | 대기에 실패한 완료 신호 수. 0이 아니면 고장이다 |
| `fence_wait()` / `fence_wait` | 실제 대기 시간. commit과 해제 각각의 합(`*_ns`), 횟수(`*_waits`), 최댓값(`*_max_ns`) |

## 복구 불가능한 CUDA 오류

프로세스 재시작으로만 복구된다고 CUDA가 정한 오류(illegal address 등)를 받으면, flux는 프로세스의 GPU를 멈춤 상태로 표시한다. 그 뒤의 동작은 다음과 같다.

| 대상 | 동작 |
| --- | --- |
| 모든 GPU 채널의 loan과 take | 드라이버를 호출하지 않고 바로 거절된다(take는 `refused.fence`) |
| publish와 commit | `GpuWedged`를 반환한다 |
| 새 GPU 채널 | `UNSUPPORTED`로 실패한다 |
| 호스트 채널 | 영향 없음 |

`flux::gpu::wedged()`(Python `flux.gpu_wedged()`)가 그 오류 번호를 반환한다. GPU가 멈추지 않았으면 0이다. supervisor는 이 값을 보고 프로세스를 다시 시작한다.

## 그 밖의 규칙

- 부모가 CUDA를 초기화한 뒤 fork한 자식은 GPU 객체 생성이 불가능하다(`UNSUPPORTED`).
- 완료 신호 대기 상한이나 `Deferred`를 사용하는 스트림은 스트림마다 하나인 `flux-gpu-wait` 스레드가 기다린다. RT 체인에서는 이 스레드와 GPU 드라이버 IRQ 스레드의 우선순위도 설정한다.
- 완료 신호는 스트림 단위다. 채널 스트림에 프레임과 무관한 작업을 올리면 flux는 그 작업도 기다린다.
