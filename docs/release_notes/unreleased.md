# flux 미출시

다음 릴리스에 들어갈 변경이다. 양식은 [template.md](template.md)를 따른다.

## [미출시](https://github.com/nevlife/flux/tree/devel)

### 기능

<!--
    새 항목은 issue 번호 순으로 넣는다. 병합 충돌을 줄이기 위해서다.
    issue가 없는 항목은 번호 없이 맨 뒤에 둔다.
-->

* C++ `Publisher`, `Subscription`, `message_filters::Subscriber`가 `rclcpp::Node`뿐 아니라 노드 interface getter를 가진 노드(`rclcpp_lifecycle::LifecycleNode` 등)를 받는다

### 버그 수정

<!--
    새 항목은 issue 번호 순으로 넣는다.
-->

### 리팩터링

<!--
    새 항목은 issue 번호 순으로 넣는다.
-->

* `flux_bridge`, `flux_bag`, `flux_tools`를 `ros2/` 아래로 옮겼다
* 동시성 프로토콜 모델 검사 하니스를 저장소 밖으로 옮겼다

### 작업 흐름

<!--
    빌드, 시험, 검사, 문서 생성의 변경. 새 항목은 issue 번호 순으로 넣는다.
-->

* 사용자 문서(`README.md`, `docs/`)를 새로 썼다
* `scripts/build_docs.sh`가 rustdoc, Doxygen, pdoc 레퍼런스를 만든다
* 문서의 코드 예제를 시험으로 돌리고(`test_doc_examples`), pre-commit이 문서와 시험의 예제가 같은지 본다(`scripts/check_doc_examples.py`)

### 새 API

<!--
    새 항목은 issue 번호 순으로 넣는다.
-->

* `flux::NodeRef`

### 호환이 깨지는 변경

<!--
    사용자가 고쳐야 하는 것마다 번호를 붙이고, 고치기 전과 후를 코드로 보인다.
-->
