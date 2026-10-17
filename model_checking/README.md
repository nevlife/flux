# flux2 model checking

flux2 동시성 프로토콜의 형식 검증 하니스다. flux의 `flux_model_checking`과 별개로 flux2 프로토콜만 다룬다. 탐색기(`explore.h`)는 그쪽에서 복사해 와서, 이 디렉터리만으로 빌드된다.

| 하니스 | 대상 |
| --- | --- |
| `readers/` | 구독자 번호별 reader 칸, 단일 발행자 claim, crash, 재가입, lock 기반 회수, loan abort |
| `wake/` | 발행자 wake gate, `wait()`의 park, executor의 샘플·arm·cancel·재gate, 번호별 waiter와 죽은 대기자의 sweep, parker fallback의 겨냥·재겨냥·retired word |
| `sweep/` | /dev/shm 이름과 OFD lock. 차례 lock 없는 sweeper, 발행자 생성·종료, 구독자 attach, signpost의 마지막 사용자, 어느 단계에서든 죽는 프로세스 |
| `litmus/` | 공유 word마다의 ordering을 RC11(GenMC)에서 보는 2~3 스레드 litmus |
| `tools/mutate_orderings.py` | ordering을 한 자리씩 낮춰 Miri나 GenMC가 잡는지 본다 |

C 코드는 루트 `.clang-format`으로 포맷하고, Makefile이 `-Wall -Wextra -Werror`로 빌드한다. clang-tidy hook은 compile commands가 없는 이 디렉터리를 제외한다.

`readers/`, `wake/`, `sweep/`의 검증 범위는 SC(sequential consistency) 인터리빙에서의 알고리즘 safety다. `Ordering` 지정은 둘이 본다. 실제 Rust 코드는 Miri가 돌리고(`flux2_core`의 `::race::` 테스트, `scripts/miri.sh`), 모양마다 떼어 낸 C11 litmus는 GenMC가 RC11에서 돈다(`litmus/`). 공유 word와 그 짝의 표는 `design.md`의 "메모리 순서" 절이다.

```bash
make -C readers all     # check, bug, vacuous, configs
make -C wake all
make -C sweep all
make -C litmus all      # native, check, mutate. GenMC가 없으면 native만 돈다
../scripts/miri.sh      # 고정 nightly의 Miri, seed 64개
```

## 자동 실행

| 언제 | 무엇 |
| --- | --- |
| `colcon test` | 이 디렉터리가 패키지 `flux2_model_checking`이다. 세 모델의 check·bug·vacuous·configs, litmus native, Miri(고정 nightly에 Miri가 있을 때), GenMC litmus(GenMC가 있을 때). 없는 도구는 configure 때 경고로 밝힌다. 다 해서 약 1분이다 |
| pre-commit | 저장소 `.pre-commit-config.yaml`이 덮는 파일이 바뀔 때만 돈다. `channel.rs`·`reader.rs`·`segment.rs`는 readers, `channel.rs`·`executor.rs`·`segment.rs`는 wake, `discovery.rs`·`segment.rs`는 sweep, `channel.rs`·`executor.rs`·`segment.rs`·`reader.rs`·`manifest.rs`는 Miri, `litmus/`는 litmus. Miri가 없으면 설치 명령을 찍고 실패한다 |

## ordering 변이

`tools/mutate_orderings.py`는 flux의 같은 이름 도구를 옮겨 Rust `Ordering`에도 쓰게 한 것이다. 자리 하나를 낮춘 사본마다 검사를 돌려 결과를 찍는다. `SeqCst`는 그 연산의 acquire/release 형태로 낮춘다. 그래서 잡히면 그 자리가 acquire/release로는 모자라고 단일 전순서(SB 짝)가 필요하다는 뜻이다. `Acquire`·`Release`·`AcqRel`은 `Relaxed`로, fence는 지운다.

```bash
TMPDIR=... model_checking/tools/mutate_orderings.py rust [--only REGEX] [channel.rs ...]
model_checking/tools/mutate_orderings.py genmc --strict litmus/sb_claim.c
```

Rust 사본은 저장소 밖(`TMPDIR`)에 만들고 사본마다 target을 새로 만든다. 같은 target을 다시 쓰면 cargo-miri가 바뀐 소스를 다시 빌드하지 않는다. 잡히지 않은 자리는 다른 자리가 이미 그 순서를 주거나, 테스트가 보지 못하는 경로다. 결과와 그 이유는 `design.md` "메모리 순서" 절의 표에 있다.
