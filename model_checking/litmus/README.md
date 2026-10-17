# litmus: ordering을 RC11에서

공유 word의 짝마다 2~3 스레드 C11 litmus를 둔다. GenMC가 RC11 모델의 모든 실행을 돌아 assert와 data race를 본다. Rust atomic은 C11 모델이므로 LKMM cat이 아니라 RC11로 본다. 표는 `design.md`의 "메모리 순서" 절이다.

| 파일 | 모양 | 대상 |
| --- | --- | --- |
| `sb_claim.c` | SB, MP, LB | claim CAS와 구독자 +1의 Dekker, `readers_high`, commit과 되돌림의 Release, 해제 쪽 감소 |
| `lb_release.c` | LB | payload 읽기 → borrow 감소(Release), 칸 읽기(Acquire) → 덮어쓰기 |
| `sb_wake.c` | SB | wake gate. bump → waiters 읽기, `readers_high`·waiters 올림 → word 읽기 |
| `sb_release.c` | SB | `end_borrow`와 `arm_release` |
| `mp_index.c` | MP | 색인 칸과 슬롯 ticket → `latest` |
| `mp_epoch.c` | MP | 세그먼트 헤더 → `INIT_READY` → signpost epoch. 열거 경로 포함 |
| `seqlock.c` | seqlock | roster 칸과 parker 겨냥 |
| `mp_taken.c` | MP | parker의 마지막 word 접촉 → `taken` → retired 해제 |

- payload, 헤더처럼 실제 코드에서 plain인 것은 plain 변수로 둔다. 순서가 빠지면 data race로 드러난다
- RC11은 LB 결과(po와 rf의 순환)를 relaxed에서도 금지한다. 그래서 `lb_release.c`는 결과가 아니라 plain payload의 race로 감소의 Release를 확인한다
- 그 litmus가 다루지 않는 모양에 쓰이는 ordering은 litmus에서 `relaxed`로 둔다(`mp_index.c`의 ticket store). 다른 litmus가 그 자리를 본다
- 다른 짝이 이미 순서를 주는 자리는 줄 끝에 `// redundant:`와 이유를 단다. `make mutate`는 그 자리를 낮춰도 litmus가 통과해야 하고, 나머지 자리는 낮추면 실패해야 한다

```bash
make native   # cc로 한 번씩 빌드하고 실행한다. GenMC가 없는 곳에서도 C로 유효한지 본다
make check    # GenMC, 위반이 없어야 한다
make mutate   # 자리마다 낮춰 기대한 결과인지 본다
```

GenMC는 `PATH`나 `~/.local/bin/genmc`에서 찾는다. 없으면 `check`와 `mutate`는 `SKIP`을 찍고 통과한다. GenMC v0.19.0(LLVM 19)으로 처음 돌렸을 때 `check`는 8개 모두 위반이 없었고, `mutate --strict`는 각 자리의 결과가 표의 기대와 모두 같았다. 낮춰도 위반이 나지 않는 자리는 `sb_wake`의 대기 쪽 SeqCst 둘(futex 호출의 커널 배리어가 순서를 준다)과 `mp_epoch`의 `init_state` 재확인 load 하나이고, 셋 다 표에 중복으로 적힌 자리다. herd7의 C11 cat은 쓰지 않는다. CAS와 plain 접근의 race를 GenMC가 그대로 다루기 때문이다.
