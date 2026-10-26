# API 개요

flux의 API는 네 층이다. 손으로 쓴 문서는 개요와 개념만 다루고, 함수 하나하나의 레퍼런스는 소스에서 생성한다.

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

- 첫 인자로 출력 디렉터리를 바꾼다.
- 필요한 것은 rustup의 cargo, doxygen, `PYTHON`(기본 `python3`)에서 import되는 `pdoc`이다. 환경 변수 `DOXYGEN`과 `PYTHON`으로 다른 실행 파일을 고른다. 빠진 것이 있으면 무엇이 없는지 적고 실패한다.
- pdoc은 ROS의 Python 패키지(rclpy, numpy)와 같은 인터프리터에 있어야 한다. 시스템 패키지를 보는 가상환경에 설치한다.

```bash
python3 -m venv --system-site-packages ~/venvs/flux-docs
~/venvs/flux-docs/bin/pip install pdoc
PYTHON=~/venvs/flux-docs/bin/python src/flux/scripts/build_docs.sh
```

## C ABI

- 헤더 `flux.h`는 cbindgen이 Rust 소스에서 생성하고 생성본을 커밋한다. 빌드에는 cbindgen이 필요 없다.
- 공유 라이브러리의 soname은 `libflux_ffi.so.<FLUX_ABI_VERSION>`이고 지금 버전은 3이다. `flux_abi_version()`이 실행 중인 라이브러리의 버전을 준다. `flux_cpp`는 처음 쓸 때 헤더의 값과 비교해 다르면 `flux::Error`(`FLUX_MISMATCH`)를 던진다.
- 핸들은 불투명 포인터이고 `_free`는 `NULL`을 받는다. view와 loan은 호출자 메모리의 고정 크기 저장소(각 128바이트)다. 0으로 채우면 빈 상태이고, 값으로 복사하지 않는다.
- 오류는 `flux_status` 반환값과 스레드별 `flux_last_error()`로 알린다. 예외와 panic은 경계를 넘지 않는다.
- 발행, loan, take, QoS 설정은 채널 하나에 한 번에 한 호출만 들어간다. 겹친 호출은 `*_IN_USE`나 빈 결과를 받는다. getter는 검사하지 않으므로 그 채널의 다른 호출과 겹치지 않게 부른다. commit은 겹쳐도 된다.

## trace

`libflux_ffi.so`는 `flux_trace_*` 함수를 내보낸다. 함수 자체는 아무것도 하지 않고, 수집기(uprobe, `LD_PRELOAD`, LTTng)가 이름으로 붙는 자리다. 붙은 것이 없을 때 비용은 이벤트당 0.7~2.6 ns다(Jetson AGX Orin). 이벤트 이름, 인자, 상수의 선언은 설치 헤더 `flux/trace.h`에 있고, 계약 버전은 `flux_trace_contract_2`다.

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

## 문서 예제

이 문서들의 코드 블록 가운데 언어 표시 뒤에 `doc:<id>`가 붙은 것은 시험 코드와 같다. C++ 예제는 `flux_cpp/test/test_doc_examples.cpp`, Python 예제는 `flux_py/test/test_doc_examples.py`에서 실제로 돌고, `scripts/check_doc_examples.py`(pre-commit)가 문서의 코드 블록과 시험의 `[doc:<id>]` 구간이 같은지 본다.
