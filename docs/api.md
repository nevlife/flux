# API 개요

flux의 API는 네 층으로 나뉜다. 손으로 쓴 문서는 개요와 개념만 다루고, 함수 하나하나의 레퍼런스는 소스에서 생성한다.

```text
flux_core (Rust)     공유메모리 링, 발견, 생존 확인, executor
   |
flux_ffi (C ABI)     flux.h, libflux_ffi.so
   |            \
flux_cpp           flux_py
C++ 래퍼            nanobind 모듈 flux
+ rclcpp 통합        + rclpy 통합
```

| 층 | 쓰는 쪽 | 레퍼런스 |
| --- | --- | --- |
| `flux_cpp` | rclcpp 노드 | Doxygen(`cpp/html`). 사용법은 [cpp.md](cpp.md) |
| `flux_py` | rclpy 노드 | pdoc(`python/flux.html`). 사용법은 [python.md](python.md) |
| `flux_ffi` | 다른 언어 바인딩, C 프로그램 | Doxygen(`cpp/html`의 `flux.h`), rustdoc(`rust/doc/flux_ffi`) |
| `flux_core` | Rust 프로그램, flux 자신 | rustdoc(`rust/doc/flux_core`) |

ROS 노드에서는 `flux_cpp`나 `flux_py`를 쓴다. 두 층이 노드의 이름 풀이, context 종료, executor 통합을 맡는다.

## 레퍼런스 생성

```bash
source install/setup.bash          # pdoc이 flux 모듈을 import한다
src/flux/scripts/build_docs.sh     # 출력: src/flux/target/docs (git이 무시한다)
```

| 출력 | 도구 | 내용 |
| --- | --- | --- |
| `rust/doc/flux_core/index.html` | `cargo doc` | Rust core |
| `rust/doc/flux_ffi/index.html` | `cargo doc` | C ABI의 Rust 쪽 |
| `cpp/html/index.html` | Doxygen | `flux_cpp/include`의 공개 헤더와 `flux.h`. `flux::detail`은 뺀다 |
| `python/flux.html` | pdoc | `flux`. `flux.wire`와 `flux.message_filters`는 `python/flux/` 아래에 있다 |

- 스크립트의 첫 인자로 출력 디렉터리를 바꿀 수 있다.
- 스크립트에는 rustup의 cargo, doxygen, 그리고 `PYTHON`(기본 `python3`)으로 import할 수 있는 `pdoc`이 필요하다. 환경 변수 `DOXYGEN`과 `PYTHON`으로 다른 실행 파일을 고른다. 하나라도 빠지면 무엇이 없는지 출력하고 실패한다.
- pdoc은 ROS의 Python 패키지(rclpy, numpy)와 같은 인터프리터에 설치해야 한다. 시스템 패키지를 볼 수 있는 가상환경에 설치한다.

```bash
python3 -m venv --system-site-packages ~/venvs/flux-docs
~/venvs/flux-docs/bin/pip install pdoc
PYTHON=~/venvs/flux-docs/bin/python src/flux/scripts/build_docs.sh
```

## C ABI

- 헤더 `flux.h`는 cbindgen이 Rust 소스에서 생성하고, 생성된 파일을 저장소에 커밋한다. 그래서 빌드에는 cbindgen이 필요 없다.
- 공유 라이브러리의 soname은 `libflux_ffi.so.<FLUX_ABI_VERSION>`이고 지금 버전은 3이다. `flux_abi_version()`이 실행 중인 라이브러리의 버전을 준다. `flux_cpp`는 처음 쓰일 때 이 값을 헤더의 값과 비교하고, 다르면 `flux::Error`(`FLUX_MISMATCH`)를 던진다.
- 핸들은 불투명 포인터이고, `_free` 함수에는 `NULL`을 넘겨도 된다. view와 loan은 호출자가 마련한 고정 크기 저장소(각 128바이트)에 담긴다. 이 저장소는 0으로 채우면 빈 상태다. 값으로 복사하지 않는다.
- 오류는 `flux_status` 반환값과 스레드별 `flux_last_error()`로 알린다. 예외와 panic은 C ABI 경계를 넘지 않는다.
- 채널 하나에서 발행, loan, take, QoS 설정은 한 번에 한 호출만 실행된다. 겹친 호출은 `*_IN_USE`나 빈 결과를 받는다. getter는 이 검사를 하지 않는다. 그러므로 getter는 그 채널의 다른 호출과 겹치지 않게 부른다. commit은 겹쳐도 된다.

## trace

`libflux_ffi.so`는 `flux_trace_*` 함수를 내보낸다. 이 함수들은 아무 일도 하지 않는다. 수집기(uprobe, `LD_PRELOAD`, LTTng)가 함수 이름으로 붙을 수 있게 둔 자리다. 아무것도 붙지 않았을 때 비용은 이벤트당 0.7~2.6 ns다(Jetson AGX Orin). 이벤트 이름, 인자, 상수는 설치 헤더 `flux/trace.h`에 선언돼 있고, 계약 버전은 `flux_trace_contract_2`다.

| 이벤트 | 때 |
| --- | --- |
| `channel_init` | 채널 생성과 재attach |
| `claim`, `loan`, `loan_end`, `commit` | 발행 쪽 슬롯 획득과 발행 |
| `take`, `release`, `refused` | 구독 쪽 수신, 해제, 거절 |
| `wait`, `wake`, `callback_start`, `callback_end` | executor |
| `endpoint_init` | C++·Python의 Publisher·Subscription 생성 |
| `py_gil`, `py_ready` | Python 바인딩의 GIL 재획득과 콜백 직전 |
| `record_drop` | `flux_bag` 녹화기가 bag에 쓰지 못한 프레임 |
| `gpu_wedged` | 복구할 수 없는 CUDA 오류로 프로세스의 GPU를 멈춤 상태로 표시할 때 |
