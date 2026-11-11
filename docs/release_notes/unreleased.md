# flux 미출시

다음 릴리스에 들어갈 변경이다. 양식은 [template.md](template.md)를 따른다.

## 미출시

### 기능

<!--
    새 항목은 issue 번호 순으로 넣는다. 병합 충돌을 줄이기 위해서다.
    issue가 없는 항목은 번호 없이 맨 뒤에 둔다.
-->

* C++ `Publisher`, `Subscription`, `message_filters::Subscriber`가 `rclcpp::Node`뿐 아니라 노드 interface getter를 가진 노드(`rclcpp_lifecycle::LifecycleNode` 등)를 받는다
* 컴포넌트 컨테이너 `flux_components`(`component_container`). 로드한 노드의 콜백 그룹마다 스레드 하나를 두고, 컴포넌트의 flux 구독도 돌린다
* C++ flux 구독을 콜백 그룹에 넣으면, 그 그룹을 돌리는 flux executor가 구독도 돌린다. 노드 코드에서 executor를 알 필요가 없다
* C++ `PartitionedExecutor`에 spin 중에도 노드를 넣고 뺄 수 있고, 모든 그룹 스레드에서 도는 hook을 줄 수 있다. 다른 executor가 이미 돌리는 그룹은 건드리지 않는다
* `flux` 명령의 bash Tab 자동완성. 하위 명령, `--window`, 이 호스트에 보이는 채널 이름을 채운다
* 외장 GPU에서 CPU와 GPU의 네 조합을 모두 받는다. 장치 메모리 채널의 CPU 구독과 호스트 메모리 채널의 GPU 구독은 구독자가 take마다 한 번 복사한다. 장치 메모리 채널의 `publish(호스트 배열)`은 슬롯으로 복사해 발행한다. 전에는 각각 `MISMATCH`, `INVALID_ARGUMENT`, `WrongDevice`였다
* C ABI가 기본값 상수 `FLUX_DEFAULT_SLOT_SIZE`, `FLUX_DEFAULT_SLOT_COUNT`, `FLUX_DEFAULT_TURN_NS`를 내보낸다
* `record_flux`가 `-s`, `--storage-preset-profile`을 rosbag2에 넘긴다. `zstd_fast`로 mcap bag을 무손실 압축한다
* `record_flux --image-codec jpeg`가 `sensor_msgs/Image`(`rgb8`, `bgr8`, `mono8`)의 픽셀을 JPEG로 저장한다. `play_flux`는 디코딩해 원래 채널에 발행하고, `convert_flux`는 `sensor_msgs/CompressedImage`로 바꾼다
* 발행자가 같은 이름의 ROS 2 토픽도 낼 수 있다(C++ `mirror_to_ros()`, Python `mirror_to_ros=True`). ROS 2 구독자가 있는 동안만 변환한다. `flux_bridge`는 다른 ROS 2 발행자가 있는 토픽을 relay하지 않고, `echo_flux`·`hz_flux`는 mirror만 있어도 동작한다
* 발행자가 지금 구독하는 프로세스 수를 센다(C++ `subscriber_count()`, Python `subscriber_count`, C `flux_channel_subscriber_count`). 죽은 구독 프로세스는 세지 않는다
* `ros2/flux_wrapper`: flux가 있든 없든 빌드되는 발행·구독 API. 빌드할 때와 실행할 때 `ENABLE_FLUX=1`이면 flux로, 아니면 ROS 2로 보낸다. `sensor_msgs/Image`와 `PointCloud2`를 다룬다
* 슬롯마다 프레임과 함께 commit되는 호스트 메모리(aux)를 둘 수 있다. 장치 메모리 채널에서도 호스트 메모리라서, GPU 프레임에 stamp 같은 헤더를 같은 채널로 붙인다
* `flux_wrapper`가 `sensor_msgs/Image`와 `PointCloud2`를 GPU 메모리째 보내고 받는다(C++ `Device::Cuda`, Python `device="cuda"`). flux가 없으면 호스트로 복사해 ROS 메시지로 보낸다. `Cuda` 구독은 CPU 발행자와 ROS 경로의 메시지도 장치 메모리로 받는다(C++)
* `record_flux`가 프레임의 aux와 채널의 `aux_size`를 `FluxFrame.meta` 끝에 담고, `play_flux`가 같은 `aux_size`로 발행자를 만들어 aux와 함께 발행한다. `flux_wrapper`의 GPU 이미지 채널도 헤더와 함께 녹화하고 재생한다. aux를 담은 bag은 이전 `flux_bag`이 읽지 못한다
* 구독이 콜백 뒤에도 프레임을 복사 없이 들고 있을 수 있다. C++ `flux::Subscription`의 콜백은 `FrameView`를 move해서 들고 있고, `flux_wrapper`의 콜백은 `std::shared_ptr<const View<Msg>>`를 받아 들고 있는다. 들고 있을 수는 `max_borrow`이고, 슬롯이 그보다 적은 발행자는 거절한다
* 생성된 adapter의 가변 필드가 슬롯에 들어가지 않으면 발행 프로세스의 공유메모리 arena에 들어간다. `slot_size`를 메시지의 최대 크기에 맞추지 않아도 된다. 크기는 `FLUX_ARENA_BYTES`(기본이자 최대 1 GiB, 페이지는 쓰는 만큼 잡는다)로 정한다
* 생성된 adapter 메시지의 숫자 배열 필드를 GPU에서 쓰고 GPU에서 읽는다(C++ `alloc_device__<name>`, `<name>__device()`). 나머지 필드는 호스트 슬롯에 남는다. 외장 GPU에서도 그 발행자의 슬롯은 호스트 메모리이고, 필드는 프로세스마다 장치마다 하나인 GPU arena에 둔다. 외장 GPU의 CPU 구독자는 그 프레임을 처음 읽을 때 장치 필드를 복사한다. 반대로 CPU 발행자가 쓴 필드도 GPU 구독자가 `__device()`로 읽고, 외장 GPU에서 arena에 간 필드는 그 구독자가 처음 읽을 때 복사한다(통합 GPU에서는 슬롯 안의 필드만). Python에서도 같다(생성된 `alloc_device__<name>`, `<name>__device`, DLPack)
* 라이브러리가 만든 버퍼를 복사 없이 필드로 넘긴다(`LD_PRELOAD=libflux_heaphook.so`, C++ `flux::ArenaScope`, Python `flux.arena_scope()`, 생성된 `adopt__<name>`)

### 버그 수정

<!--
    새 항목은 issue 번호 순으로 넣는다.
-->

* `record_flux`가 inotify 감시 한도가 다 찬 호스트에서 시작하지 못했다. inotify를 빼고 채널 목록을 `--poll`초(기본 0.1)마다 읽는다
* `PartitionedExecutor`의 `on_thread_start` 훅이 정한 스레드 이름이 기본 이름 `flux-g<N>`으로 덮어써질 수 있었다.

### 리팩터링

<!--
    새 항목은 issue 번호 순으로 넣는다.
-->

* `flux_bridge`, `flux_bag`, `flux_tools`를 `ros2/` 아래로 옮겼다
* 동시성 프로토콜을 모델로 검사하는 하니스를 저장소 밖으로 옮겼다
* 기본값을 core 한 곳에 두고 C++과 Python이 그 값을 쓴다. 값은 바뀌지 않았다

### 작업 흐름

<!--
    빌드, 시험, 검사, 문서 생성의 변경. 새 항목은 issue 번호 순으로 넣는다.
-->

* 사용자 문서(`README.md`, `docs/`)를 새로 썼다
* 지원 환경을 시험한 범위(ROS 2 Jazzy, JetPack 7.2)로 좁혀 적었다
* `scripts/build_docs.sh`가 rustdoc, Doxygen, pdoc 레퍼런스를 만든다
* 문서의 코드 예제를 시험(`test_doc_examples`)으로 돌린다. pre-commit은 문서의 예제가 시험의 예제와 같은지 확인한다(`scripts/check_doc_examples.py`)

### 새 API

<!--
    새 항목은 issue 번호 순으로 넣는다.
-->

* `flux::NodeRef`
* `flux::Subscription::add_to_callback_group`, `flux::message_filters::Subscriber::add_to_callback_group`
* `flux::PartitionedExecutor::remove_node`, `flux::PartitionedExecutor::on_thread_start(ThreadStart)`
* `flux::ComponentManager`, `flux::run_component_container`
* `flux::Publisher::mirror_to_ros`, Python `Publisher(mirror_to_ros=...)`
* aux: C++ `Publisher`의 `aux_size` 인자와 `aux_size()`, `WriteSlot::aux`, `WriteSlot::set_aux_len`, `FrameView::aux`. Python `Publisher(aux_size=...)`, `Loan.aux`, `Loan.aux_len`, `flux.aux`. C `flux_loan_aux`, `flux_loan_set_aux_len`, `flux_view_aux`, `flux_channel_aux_size`, `FLUX_MAX_AUX_SIZE`
* `flux_wrapper`: `Device`, `PublisherOptions::device`, `SubscriptionOptions::device`, `Buffer`, `Loan<Image>`와 `Loan<PointCloud2>`의 `alloc_buffer`, `View<Image>`와 `View<PointCloud2>`의 `on_device`·`device_ptr`·`size`·`stream`, `Publisher::uses_device`, `Subscription::rejected`. Python `Publisher(device=...)`, `Publisher.uses_device`, `Subscription(device=...)`, `Subscription.rejected`, `Loan.alloc_buffer`, `View.on_device`, `View.device_data`
* `flux::Subscription::require_publisher_slots`
* `flux_wrapper`: `SubscriptionOptions::max_borrow`, `Subscription::HeldCallback`, `View::after`
* arena: C++ `flux::ArenaScope`, `WriteSlot::heap`, `FrameView::heap`, `wire::Heap`, 생성된 `adopt__<name>`, `View::budget__`. Python `flux.arena_scope`, 생성된 `adopt__<name>`, `flux.wire.Reader.budget`. C `flux_heap`, `flux_loan_heap`, `flux_heap_alloc`, `flux_heap_adopt`, `flux_view_heap`, `flux_arena_acquire`, `flux_arena_release`, `flux_arena_scope_enter`, `flux_arena_scope_leave`, `FLUX_HEAP_BASE`. 라이브러리 `libflux_heaphook.so`
* GPU arena: C++ `wire::DeviceSpan`, `wire::GpuHeap`, `wire::GpuBlock`, `Writer::alloc_device`, `Reader::device`, `FrameView::gpu_heap`, 생성된 `alloc_device__<name>`와 `<name>__device`. C `flux_heap.gpu`, `flux_heap_alloc_device`, `flux_gpu_heap`, `flux_gpu_block`, `flux_view_gpu_heap`, `flux_layout_register`, `FLUX_GPU_HEAP_BASE`. Python 생성된 `alloc_device__<name>`와 `<name>__device`, `flux.wire.Reader.device`, `flux.wire.Writer.alloc_device`

### 호환이 깨지는 변경

<!--
    사용자가 고쳐야 하는 것마다 번호를 붙이고, 고치기 전과 후를 코드로 보인다.
-->

1. 공유메모리 배치 버전이 6에서 7로, C ABI 버전이 3에서 4로 올랐다. 슬롯에 aux 영역이 생겼기 때문이다. 버전이 다른 프로세스는 서로의 채널을 찾지 못하므로, 한 호스트의 flux 프로그램을 모두 다시 빌드한다. `libflux_ffi.so.3`에 링크한 바이너리도 다시 빌드한다.
2. C `flux_channel_create`에 `max_subscribers` 다음 인자로 `aux_size`가 생겼다. aux가 필요 없으면 0을 준다. `flux_frame_meta.reserved0[2]`는 `uint16_t aux_len`이 되었다.

   ```c
   // 전
   flux_channel_create(name, slot_size, slot_count, max_subscribers, fingerprint, mem, stream,
                       stamp_offset, &ch);
   // 후
   flux_channel_create(name, slot_size, slot_count, max_subscribers, 0, fingerprint, mem, stream,
                       stamp_offset, &ch);
   ```

3. C++ `flux::Subscription::Callback`이 `std::function<void(FrameView &&)>`가 되었다. `const flux::FrameView &`로 받는 콜백은 그대로 된다. const 없는 `flux::FrameView &`로 받는 콜백은 고친다. `libflux_cpp`에 링크한 바이너리는 다시 빌드한다.

   ```cpp
   // 전
   flux::Subscription sub(*node, "image", flux::kNoSchema, flux::QoS{}, [](flux::FrameView & f) {});
   // 후
   flux::Subscription sub(*node, "image", flux::kNoSchema, flux::QoS{}, [](flux::FrameView && f) {});
   ```

4. arena가 기본으로 켜진다. 생성된 adapter의 가변 필드가 슬롯에 들어가지 않으면 전에는 `commit__()`이 `TooLarge`였고, 이제는 arena에 넣어 발행한다. arena가 켜진 `Builder`는 슬롯의 마지막 71바이트를 가변 필드에 쓰지 않으므로, 슬롯을 프레임에 꼭 맞게 잡은 발행자의 프레임도 arena로 넘칠 수 있다. `flux_bridge`, ROS 2 mirror, `record_flux`는 필드를 arena에 둔 프레임을 건너뛴다. 전처럼 동작하게 하려면 arena를 끈다.

   ```bash
   # 전: 설정 없음
   # 후: 슬롯보다 큰 프레임을 전처럼 TooLarge로
   export FLUX_ARENA_BYTES=0
   ```

5. C++에서 생성된 adapter 헤더를 포함한 프로세스가 그 타입의 fingerprint로 `Device::Cuda`(또는 `gpu::Stream`)를 선언해 발행자를 만들면, 외장 GPU에서도 슬롯이 호스트 메모리다. 전에는 장치 메모리 채널이었고, `Builder`는 쓰지 못했지만 원시 loan의 `device_ptr()`은 장치 주소였다. 이제 그 loan의 `device_ptr()`은 0이다. 원시 장치 loan에는 `kNoSchema`나 생성 타입이 아닌 fingerprint를 쓴다. Python도 생성 모듈을 import한 프로세스에서 같다. 그 타입의 `device="cuda"` 발행자는 `device_backed`가 false이고 `mirror_to_ros=True`를 받는다. 전에는 `flux.Error`였다.
