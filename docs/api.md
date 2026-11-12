# API 개요

flux의 API는 네 층으로 나뉜다. 이 문서는 개요만 다루고, 함수별 레퍼런스는 소스에서 생성한다.

```text
flux_core (Rust)     공유 메모리 링, discovery, 생존 확인, executor
   |
flux_ffi (C ABI)     flux.h, libflux_ffi.so
   |            \
flux_cpp           flux_py
C++ 래퍼            nanobind 모듈 flux
+ rclcpp 통합        + rclpy 통합
```

| 층 | 사용하는 쪽 | 레퍼런스 |
| --- | --- | --- |
| `flux_cpp` | rclcpp 노드 | Doxygen(`cpp/html`). 사용법은 [cpp.md](cpp.md) |
| `flux_py` | rclpy 노드 | pdoc(`python/flux.html`). 사용법은 [python.md](python.md) |
| `flux_ffi` | 다른 언어 바인딩, C 프로그램 | Doxygen(`cpp/html`의 `flux.h`), rustdoc(`rust/doc/flux_ffi`) |
| `flux_core` | Rust 프로그램, flux 내부 | rustdoc(`rust/doc/flux_core`) |

ROS 노드는 `flux_cpp`나 `flux_py`를 사용한다. 두 층이 노드의 이름 해석, context 종료, executor 통합을 담당한다.

## 레퍼런스 생성

```bash
source install/setup.bash          # pdoc이 flux 모듈을 import한다
src/flux/scripts/build_docs.sh     # 출력: src/flux/target/docs (git이 무시한다)
```

| 출력 | 도구 | 내용 |
| --- | --- | --- |
| `rust/doc/flux_core/index.html` | `cargo doc` | Rust core |
| `rust/doc/flux_ffi/index.html` | `cargo doc` | C ABI의 Rust 쪽 |
| `cpp/html/index.html` | Doxygen | `flux_cpp/include`의 공개 헤더와 `flux.h`. `flux::detail`은 제외한다 |
| `python/flux.html` | pdoc | `flux`. `flux.wire`와 `flux.message_filters`는 `python/flux/` 아래에 있다 |

- 스크립트의 첫 인자로 출력 디렉터리를 지정한다.
- 필요한 도구는 rustup의 cargo, doxygen, `PYTHON`(기본 `python3`)으로 import 가능한 `pdoc`이다. 다른 실행 파일은 `DOXYGEN`과 `PYTHON`으로 지정한다. 하나라도 없으면 빠진 도구를 출력하고 실패한다.
- pdoc은 rclpy, numpy와 같은 인터프리터에 설치한다. 시스템 패키지가 보이는 가상환경을 사용한다.

```bash
python3 -m venv --system-site-packages ~/venvs/flux-docs
~/venvs/flux-docs/bin/pip install pdoc
PYTHON=~/venvs/flux-docs/bin/python src/flux/scripts/build_docs.sh
```

## C ABI

- 헤더 `flux.h`는 cbindgen이 Rust 소스에서 생성해 저장소에 커밋한다. 빌드에는 cbindgen이 필요하지 않다.
- 공유 라이브러리의 soname은 `libflux_ffi.so.<FLUX_ABI_VERSION>`이고 현재 버전은 3이다. `flux_abi_version()`은 실행 중인 라이브러리의 버전을 반환한다. `flux_cpp`는 처음 사용 시 이 값을 헤더와 비교하고, 다르면 `flux::Error`(`FLUX_MISMATCH`)를 던진다.
- 핸들은 불투명 포인터이고, `_free` 함수에 `NULL`을 넘겨도 된다.
- view와 loan은 호출자가 마련한 128 B 저장소에 담긴다. 0으로 채운 저장소는 빈 상태이고, 값으로 복사하지 않는다.
- 오류는 `flux_status` 반환값과 스레드별 `flux_last_error()`로 알린다. 예외와 panic은 C ABI 경계를 넘지 않는다.
- 채널 하나에서 발행, loan, take, QoS 설정은 한 번에 한 호출만 실행된다. 겹친 호출은 `*_IN_USE`나 빈 결과를 받는다. commit은 겹쳐도 된다.
- 접근자는 이 검사를 하지 않으므로, 같은 채널의 다른 호출과 겹치지 않게 호출한다.

## trace

`libflux_ffi.so`는 수집기(uprobe, `LD_PRELOAD`, LTTng)가 이름으로 연결하는 빈 함수 `flux_trace_*`를 export한다. 연결된 수집기가 없으면 비용은 이벤트당 0.7~2.6 ns다(Jetson AGX Orin). 이벤트 이름, 인자, 상수는 설치 헤더 `flux/trace.h`에 선언되어 있고, 계약 버전은 `flux_trace_contract_2`다.

| 이벤트 | 시점 |
| --- | --- |
| `channel_init` | 채널 생성과 재연결 |
| `claim`, `loan`, `loan_end`, `commit` | 발행 쪽의 슬롯 획득과 발행 |
| `take`, `release`, `refused` | 구독 쪽의 수신, 반납, 거절 |
| `wait`, `wake`, `callback_start`, `callback_end` | executor |
| `endpoint_init` | C++와 Python의 `Publisher`, `Subscription` 생성 |
| `py_gil`, `py_ready` | Python 바인딩의 GIL 재획득과 콜백 직전 |
| `record_drop` | `flux_bag` 녹화기가 bag에 기록하지 못한 프레임 |
| `gpu_wedged` | 복구 불가능한 CUDA 오류로 프로세스의 GPU를 정지 상태로 표시하는 시점 |
