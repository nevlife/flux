# GPU(CUDA)

GPU에서 만들고 GPU에서 읽는 프레임도 같은 채널 API로 보낸다. 채널을 만들 때 CUDA stream을 선언하면 GPU 채널이 된다. 선언하지 않으면 호스트 채널이 되고, 이 문서의 내용은 적용되지 않는다.

## 경로

flux는 `libcuda.so.1`을 실행 시점에 연다. 그리고 장치마다 아래 표에서 조건이 처음 맞는 줄의 경로를 고른다.

| 경로 | 조건 | 장치 예 | 슬롯 데이터 |
| --- | --- | --- | --- |
| `ShmRegistered` | 통합 GPU이고 호스트 메모리 등록을 지원한다 | Jetson Orin, Thor | 공유메모리 매핑을 `cuMemHostRegister`로 등록한다 |
| `ShmDirect` | 통합 GPU이고 pageable 메모리에 접근한다 | | 공유메모리 매핑 주소를 그대로 쓴다 |
| `DeviceHandle` | VMM과 POSIX fd export를 지원한다 | 외장 GPU | 장치 메모리 할당 하나를 fd로 넘긴다 |
| `None` | 그 밖 | GPU 없음 | GPU 채널을 만들 수 없다 |

`probe(device = 0)`는 장치 하나의 경로를 알려 주고, 예외를 던지지 않는다. 경로가 `None`이면 `reason`에 이유가 담긴다.

```cpp
const flux::gpu::Platform p = flux::gpu::probe();   // p.route, p.reason
```

```python
p = flux.probe()                                     # p.route, p.reason
```

확인한 장비는 Jetson AGX Orin(JetPack 7.2, `ShmRegistered`)과 RTX 4070 SUPER(`DeviceHandle`)다. `ShmDirect`는 실제 장비에서 돌려 보지 않았다.

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

`launch_my_kernel`과 `read_on_device`는 사용자의 kernel 호출 자리다.

- `flux::Device::Cuda`를 넘기면 flux가 stream을 만들어 채널에 선언한다. 직접 만든 stream을 쓰려면 `flux::gpu::Stream::from_native(cudaStream_t)`를 넘긴다. 그 stream은 채널과 view보다 오래 살아야 한다. legacy default stream(`nullptr`)은 스레드마다 다른 stream이라 거절한다.
- 주소는 `device_ptr()`로 얻고, kernel은 `stream()`이 돌려주는 채널 stream에 올린다. 다른 stream에 올리면 flux가 그 작업을 기다리지 않는다.
- `commit()`은 stream에 쌓인 작업이 끝난 뒤 발행한다. 그래서 구독자가 받은 프레임은 CPU 채널처럼 바로 읽어도 된다.
- view를 해제하면 stream의 작업이 끝난 뒤에 슬롯이 돌아간다. 구독의 장치 매핑은 읽기 전용이다(`ShmRegistered`에서는 장치가 읽기 전용 등록을 지원할 때).

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

- `device="cuda"` 구독은 numpy 배열 대신 `flux.Frame`을 준다. `Frame`은 `with frame:` 블록이나 콜백 안에서만 메모리를 내준다. `shape`, `dtype`, `nbytes`, `stream`은 언제든 읽을 수 있다.
- `Frame`과 `Loan`은 `__dlpack__`과 `__cuda_array_interface__`를 제공한다. torch는 읽기 전용인 `__cuda_array_interface__`를 받지 않으므로 `torch.from_dlpack()`을 쓴다. bf16은 `__dlpack__`으로만 받는다.
- flux는 `__dlpack__(stream=s)`로 받은 consumer stream(torch의 현재 stream)도 해제나 commit 전에 기다린다. 그래서 torch가 자기 stream에서 읽고 쓰는 동안에는 슬롯이 넘어가지 않는다. 이 기다림에는 상한이 없다. 기다리기에 실패하면 flux는 view나 loan을 `fence_failed`에 세지 않고 쥔 채로 둔다. 그 뒤의 take는 `refused.max_borrow`로 거절된다.
- GPU 배열을 `publish()`에 넘기면 `TypeError`다. GPU 데이터는 loan에 쓴다.

`Frame`의 메모리는 범위가 끝날 때 돌려준다. 다만 내준 메모리가 아직 쓰이고 있으면 돌려주는 때를 늦춘다.

| 내준 방법 | 슬롯을 돌려주는 때 |
| --- | --- |
| 없음 | `with`나 콜백이 끝날 때 |
| `__dlpack__` | 범위가 끝나고 그 capsule로 만든 tensor가 모두 사라질 때 |
| `__cuda_array_interface__` | `Frame` 객체가 사라질 때 |

## 생성 메시지의 장치 필드

생성된 adapter 메시지의 숫자 배열 필드는 GPU에서 쓰고 GPU에서 읽는다. 나머지 필드(header, 문자열, 작은 값)는 호스트 슬롯에 그대로 둔다.

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

- 생성 타입의 fingerprint로 stream을 선언한 발행자는 외장 GPU에서도 슬롯이 호스트 메모리다(`device_backed()`가 false). 그 발행자의 loan은 `device_ptr()`이 0이다.
- `alloc_device__<필드>(n)`은 그 필드를 발행 프로세스의 GPU arena에 잡는다. 통합 GPU에서는 CPU와 GPU가 함께 읽는 호스트 메모리이고, 외장 GPU에서는 장치 메모리다. 채널 stream에 쓴다. commit이 그 stream을 기다린다.
- GPU arena는 프로세스에 장치마다 하나이고 32 MiB 단위로 커진다. 크기는 `FLUX_ARENA_BYTES`(기본이자 최대 1 GiB)이고 호스트 arena도 같은 값을 쓴다. 블록은 그 프레임의 슬롯을 다시 쓸 때 푼다.
- 받는 쪽의 `<필드>__device()`가 장치 주소와 개수를 준다. 구독 stream이 발행자와 같은 장치(UUID)에 있어야 하고, 아니면 빈 값이다.
- CPU 구독자도 그 필드를 보통 접근자로 읽는다. 통합 GPU에서는 복사가 없다. 외장 GPU에서는 그 프레임으로 처음 `View`를 만들 때 프레임의 장치 필드만 자기 호스트 버퍼로 복사하고 끝을 기다린다. 같은 프레임의 다음 `View`는 그 복사본을 읽는다.
- CPU 발행자가 `alloc__`로 쓴 필드는 `__device()`가 빈 값이다.
- 장치 필드를 쓴 프레임은 슬롯 끝에 두 arena의 이름(128바이트)과 장치 필드마다 8바이트를 담는다. 그래서 stream을 선언한 발행자의 `Builder`는 슬롯의 마지막 135바이트와 장치 필드마다 8바이트를 가변 필드에 쓰지 않는다.
- Python에는 아직 없다. 확인한 장비는 RTX 4070 SUPER이고, Jetson에서는 돌려 보지 않았다.

## 장치 메모리 채널(`DeviceHandle`)

외장 GPU에서 GPU 발행자의 채널은 데이터가 장치 메모리에만 있다. 생성 메시지의 발행자는 예외다(위 절). CPU 메모리와 장치 메모리가 떨어져 있으므로, 메모리의 반대편에 있는 쪽은 프레임마다 한 번 복사한다.

| 발행 → 구독 | 통합 GPU | 외장 GPU |
| --- | --- | --- |
| CPU → CPU | 복사 없음 | 복사 없음 |
| GPU → GPU | 복사 없음 | 복사 없음 |
| CPU → GPU | 복사 없음 | 구독자가 take마다 자기 장치 버퍼로 복사한다. 복사는 구독 stream에 걸린다 |
| GPU → CPU | 복사 없음 | 구독자가 take마다 자기 호스트 버퍼로 복사하고, take 안에서 끝을 기다린다 |
| `publish(호스트 배열)` → 장치 메모리 채널 | 해당 없음 | 발행자가 슬롯으로 복사한 뒤 commit한다 |

| 장치 메모리 채널에서 하지 못하는 것 | 대신 |
| --- | --- |
| loan의 `data()`, Python `loan.array` | `nullptr`, 예외. `device_ptr()`이나 `__dlpack__`을 쓴다 |
| adapter의 `Builder`, GPU 구독자의 `View` | 프레임 배치가 호스트 메모리를 전제하므로 쓰지 못한다. CPU 구독자의 `View`는 자기 복사본을 읽는다 |

- 장치 메모리 채널의 CPU 구독자가 받는 view는 `data()`가 자기 복사본이고, `device_ptr()`은 0이다.
- 장치 메모리 채널에서도 aux는 호스트 메모리다. 프레임의 stamp나 크기 같은 헤더는 발행자를 `aux_size`와 함께 만들고 aux에 쓴다([concepts.md](concepts.md#발행자와-슬롯)). 받는 쪽은 CPU 구독이든 GPU 구독이든 aux를 복사 없이 읽는다.
- 복사가 실패한 take는 빈 결과이고 `refused.fence`로 센다. 그 슬롯은 돌려주지 않는다.

- 구독자는 메모리를 할당한 장치를 번호가 아니라 UUID로 찾는다. 그래서 `CUDA_VISIBLE_DEVICES`가 달라도 같은 장치를 찾는다. 구독 stream의 장치가 그 장치와 다르면 `MISMATCH`다.
- 장치 메모리 fd는 `/dev/shm/flux.gpu.<dev>.<ino>` 소켓으로 넘긴다. 세그먼트가 보이는 곳에서는 이 소켓도 보인다.

통합 GPU(`ShmRegistered`, `ShmDirect`)에서는 슬롯이 호스트 메모리이기도 하므로 복사가 없다. 한 채널에서 CPU 구독과 GPU 구독이 같이 돈다.

## 기다림의 상한과 해제 방식

| 설정 | C++ | Python | 기본 |
| --- | --- | --- | --- |
| fence 기다림 상한 | `set_fence_timeout(ns)` | `fence_timeout=초` | 상한 없음 |
| 해제 방식 | `set_release_mode(gpu::ReleaseMode::Deferred)` | `release="deferred"` | `Wait` |

- 상한이 없으면, GPU가 멈췄을 때 fence를 기다리는 executor 스레드도 함께 멈춘다. 상한을 넘긴 기다림은 실패로 센다. 이때 commit은 `FenceFailed`를 돌려주고, 슬롯은 작업이 끝날 때까지 쥐어 둔다.
- `Wait`에서는 해제가 stream을 기다린 뒤 돌아온다. `Deferred`에서는 해제가 바로 돌아온다. 슬롯은 장치가 다 쓴 뒤 다음 `take()`나 executor pass에서 돌려준다. 그때까지 그 view는 `max_borrow`에 포함된다. `Deferred`는 콜백이 GPU 작업을 걸어 둔 채 끝나는 노드에서 쓴다.
- fence를 기다리지 못하면 flux는 그 슬롯을 놓지 않는다. 슬롯을 다시 쓰면 다른 쪽이 아직 쓰는 메모리를 덮기 때문이다. 해제할 때 실패한 view가 `max_borrow`를 채우면, 그 구독의 `take()`는 `refused.fence`로 센다.

| 값 | 뜻 |
| --- | --- |
| `fence_failed()` / `fence_failed` | 기다리지 못한 fence 수. 0이 아니면 고장이다 |
| `fence_wait()` / `fence_wait` | 실제로 기다린 시간. commit과 해제 각각의 합(`*_ns`), 횟수(`*_waits`), 최댓값(`*_max_ns`) |

## 복구할 수 없는 CUDA 오류

CUDA가 프로세스를 다시 띄워야만 풀린다고 정한 오류(illegal address 등)를 받으면, flux는 프로세스의 GPU를 멈춤 상태로 표시한다. 그 뒤로 모든 GPU 채널의 loan과 take는 드라이버를 부르지 않고 바로 거절된다(take는 `refused.fence`). publish와 commit은 `GpuWedged`를 돌려주고, 새 GPU 채널은 `UNSUPPORTED`로 실패한다. 호스트 채널은 영향을 받지 않는다.

`flux::gpu::wedged()`(Python `flux.gpu_wedged()`)가 그 오류 번호를 준다. GPU가 멈추지 않았으면 0이다. supervisor는 이 값을 보고 프로세스를 다시 띄운다.

## 그 밖의 규칙

- 부모가 CUDA를 초기화한 뒤 fork한 자식은 GPU 객체를 새로 만들지 못한다(`UNSUPPORTED`).
- fence 상한이나 `Deferred`를 쓰는 stream은 `flux-gpu-wait` 스레드가 기다린다. 이 스레드는 stream마다 하나다. RT 체인에서는 이 스레드와 GPU 드라이버 IRQ 스레드의 우선순위도 정해 둔다.
- fence는 stream 단위다. 채널 stream에 프레임과 무관한 작업을 올리면 flux는 그 작업도 기다린다.
