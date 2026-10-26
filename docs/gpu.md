# GPU(CUDA)

GPU에서 만들고 GPU에서 읽는 프레임도 같은 채널 API로 보낸다. 채널을 만들 때 CUDA stream을 선언하면 GPU 채널이 된다. 선언하지 않으면 호스트 채널이고 이 문서의 내용은 해당하지 않는다.

## 경로

flux는 `libcuda.so.1`을 실행 시점에 연다. 장치마다 아래 표에서 처음 맞는 줄을 고른다.

| 경로 | 조건 | 장치 예 | 슬롯 데이터 |
| --- | --- | --- | --- |
| `ShmRegistered` | 통합 GPU이고 호스트 메모리 등록을 지원한다 | Jetson Orin, Thor | 공유메모리 매핑을 `cuMemHostRegister`로 등록한다 |
| `ShmDirect` | 통합 GPU이고 pageable 메모리에 접근한다 | | 공유메모리 매핑 주소를 그대로 쓴다 |
| `DeviceHandle` | VMM과 POSIX fd export를 지원한다 | 외장 GPU | 장치 메모리 할당 하나를 fd로 넘긴다 |
| `None` | 그 밖 | GPU 없음 | GPU 채널을 만들 수 없다 |

`probe(device = 0)`가 장치 하나의 경로를 알려 준다. 던지지 않는다. `None`이면 `reason`에 이유가 있다.

```cpp
const flux::gpu::Platform p = flux::gpu::probe();   // p.route, p.reason
```

```python
p = flux.probe()                                     # p.route, p.reason
```

확인한 장비는 Jetson AGX Orin(`ShmRegistered`)과 RTX 4070 SUPER(`DeviceHandle`)다. `ShmDirect`는 장비에서 돌려 보지 않았다.

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

- `flux::Device::Cuda`는 flux가 만드는 stream을 채널에 선언한다. 자기 stream을 쓰려면 `flux::gpu::Stream::from_native(cudaStream_t)`를 넘긴다. 그 stream은 채널과 view보다 오래 살아야 한다. legacy default stream(`nullptr`)은 스레드마다 다른 stream이라 거절한다.
- 주소는 `device_ptr()`로 얻고, kernel은 `stream()`이 돌려주는 채널 stream에 올린다. 다른 stream에 올리면 flux가 그 작업을 기다리지 않는다.
- `commit()`은 stream에 쌓인 작업이 끝난 뒤 발행한다. 그래서 구독자가 받은 프레임은 CPU 채널처럼 바로 읽어도 된다.
- view 해제는 stream의 작업이 끝난 뒤 슬롯을 돌려준다. 구독의 장치 매핑은 읽기 전용이다(`ShmRegistered`에서는 장치가 읽기 전용 등록을 지원할 때).

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

- `device="cuda"` 구독은 numpy 배열 대신 `flux.Frame`을 준다. 메모리는 `with frame:` 안이나 콜백 안에서만 내준다. `shape`, `dtype`, `nbytes`, `stream`은 언제든 읽는다.
- `Frame`과 `Loan`은 `__dlpack__`과 `__cuda_array_interface__`를 낸다. torch는 읽기 전용 CUDA array interface를 받지 않으므로 `torch.from_dlpack()`을 쓴다. bf16은 `__dlpack__`으로만 받는다.
- `__dlpack__(stream=s)`로 받은 consumer stream(torch의 현재 stream)도 해제나 commit 전에 기다린다. torch가 자기 stream에서 읽고 쓰는 동안 슬롯이 넘어가지 않는다. 이 기다림에는 상한이 없고, 실패하면 view나 loan을 `fence_failed`에 세지 않고 쥔 채 둔다. 그 뒤의 take는 `refused.max_borrow`로 거절된다.
- GPU 배열을 `publish()`에 넘기면 `TypeError`다. loan에 쓴다.

`Frame`의 메모리는 범위가 끝날 때 돌려주지만, 내준 메모리가 아직 쓰이면 늦춘다.

| 내준 방법 | 슬롯을 돌려주는 때 |
| --- | --- |
| 없음 | `with`나 콜백이 끝날 때 |
| `__dlpack__` | 범위가 끝나고 그 capsule로 만든 tensor가 모두 사라질 때 |
| `__cuda_array_interface__` | `Frame` 객체가 사라질 때 |

## 장치 메모리 채널(`DeviceHandle`)

외장 GPU의 채널은 데이터가 장치 메모리에만 있다.

| 하지 못하는 것 | 대신 |
| --- | --- |
| `publish(호스트 배열)` | `WrongDevice`. loan하고 장치에서 쓴다 |
| CPU 구독(`Device::Cpu`) | 붙을 때 `MISMATCH`로 거절한다. GPU 구독만 붙는다 |
| `data()`, Python `loan.array` | `nullptr`, 예외. `device_ptr()`이나 `__dlpack__`을 쓴다 |
| adapter의 `View`·`Builder` | 프레임 배치가 호스트 메모리를 전제하므로 쓰지 못한다 |

- 구독자는 할당한 장치를 번호가 아니라 UUID로 찾는다. `CUDA_VISIBLE_DEVICES`가 달라도 같은 장치를 찾는다. 구독 stream의 장치가 다르면 `MISMATCH`다.
- 경로가 `DeviceHandle`인 호스트에서는 호스트 메모리 채널의 GPU 구독을 `INVALID_ARGUMENT`로 거절한다.
- 장치 메모리 fd는 `/dev/shm/flux.gpu.<dev>.<ino>` 소켓으로 넘긴다. 세그먼트가 보이는 곳에서는 이 소켓도 보인다.

통합 GPU(`ShmRegistered`, `ShmDirect`)는 슬롯이 호스트 메모리이기도 해서 위 제한이 없다. 한 채널에서 CPU 구독과 GPU 구독이 같이 돈다.

## 기다림의 상한과 해제 방식

| 설정 | C++ | Python | 기본 |
| --- | --- | --- | --- |
| fence 기다림 상한 | `set_fence_timeout(ns)` | `fence_timeout=초` | 상한 없음 |
| 해제 방식 | `set_release_mode(gpu::ReleaseMode::Deferred)` | `release="deferred"` | `Wait` |

- GPU가 멈추면 상한 없는 기다림은 executor 스레드도 멈추게 한다. 상한을 넘긴 기다림은 실패로 센다. commit은 `FenceFailed`이고 슬롯은 작업이 끝날 때까지 쥐어 둔다.
- `Wait`는 해제가 stream을 기다린 뒤 돌아온다. `Deferred`는 해제가 바로 돌아오고, 장치가 슬롯을 다 쓴 뒤 다음 `take()`나 executor pass에서 슬롯을 돌려준다. 그때까지 그 view는 `max_borrow`에 든다. 콜백이 GPU 작업을 걸어 둔 채 끝나는 노드에서 쓴다.
- 기다리지 못한 fence는 그 슬롯을 놓지 않는다. 다시 쓰면 다른 쪽이 아직 쓰는 메모리를 덮기 때문이다. 해제 쪽 실패가 `max_borrow`를 채우면 그 구독의 `take()`는 `refused.fence`로 센다.

| 값 | 뜻 |
| --- | --- |
| `fence_failed()` / `fence_failed` | 기다리지 못한 fence 수. 0이 아니면 고장이다 |
| `fence_wait()` / `fence_wait` | 실제로 기다린 시간. commit과 해제 각각의 합(`*_ns`), 횟수(`*_waits`), 최댓값(`*_max_ns`) |

## 복구할 수 없는 CUDA 오류

CUDA가 프로세스를 다시 띄워야만 풀린다고 정한 오류(illegal address 등)를 받으면, flux는 프로세스의 GPU를 멈춤 상태로 표시한다. 그 뒤 모든 GPU 채널의 loan과 take는 드라이버를 부르지 않고 바로 거절하고(take는 `refused.fence`), publish와 commit은 `GpuWedged`를 돌려주며, 새 GPU 채널은 `UNSUPPORTED`다. 호스트 채널은 영향이 없다.

`flux::gpu::wedged()`(Python `flux.gpu_wedged()`)가 그 오류 번호를 주고, 멈추지 않았으면 0이다. supervisor는 이 값을 보고 프로세스를 다시 띄운다.

## 그 밖의 규칙

- 부모가 CUDA를 초기화한 뒤 fork한 자식은 GPU 객체를 새로 만들지 못한다(`UNSUPPORTED`).
- 상한이나 `Deferred`를 쓰는 stream은 `flux-gpu-wait` 스레드가 기다린다. stream마다 하나다. RT 체인에서는 이 스레드와 GPU 드라이버의 IRQ 스레드의 우선순위도 정한다.
- fence는 stream 단위다. 채널 stream에 프레임과 무관한 작업을 올리면 그 작업도 기다린다.
