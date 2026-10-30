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

### 버그 수정

<!--
    새 항목은 issue 번호 순으로 넣는다.
-->

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

### 호환이 깨지는 변경

<!--
    사용자가 고쳐야 하는 것마다 번호를 붙이고, 고치기 전과 후를 코드로 보인다.
-->
