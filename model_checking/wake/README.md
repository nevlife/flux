# wake: 깨우기 프로토콜

발행자의 wake gate, 구독자의 park, executor의 대기 루프를 모델로 고정하고 모든 인터리빙에서 검사한다. 대상 코드는 `channel.rs`의 `finish_commit`(wake 꼬리)과 `ChannelShared::wait`·`add_waiter`·`reclaim_dead`, `segment.rs`의 `clear_reader`, `executor.rs`의 `dispatch`·`wait_for_work`·`park`, `Control`, `Aim`, `Parker`, 그리고 `Channel::maybe_reattach`다. flux의 wake 하니스를 옮긴 것이고, 재attach 시점만 flux2에 맞췄다.

flux2는 latest-wins라서 "보낸 것은 모두 배달된다"는 성립하지 않는다. 대신 다음 성질을 검사한다. 새 프레임이 있는 한, 대기 프로토콜에 들어간 구독자는 반드시 깨어난다. tick 없는 config에서는 timeout 없이 wake 프로토콜만으로 이 성질이 성립해야 한다.

## 모델

발행자는 commit 뒤 다음 순서로 진행한다. waiters가 0이면 wake syscall을 건너뛴다. waiters는 구독자 번호별 칸의 합이다(번호 없는 대기는 헤더 칸). 대기자는 자기 번호의 칸 하나만 store 한 번으로 올리고 내린다.

```text
commit -> wakeup +1 -> waiters 읽기 -> (waiters > 0이면) wake
```

구독자는 두 방식을 따로 검사한다.

- executor(기본): take 전에 wake word를 샘플하고, take 뒤 그 값으로 `futex_waitv`에 들어간다. 붙어 있는 동안 waiter gate를 쥔다. 들어갈 때 word가 이미 움직였으면 즉시 돌아온다(`-EAGAIN`). 대기는 호출 하나로 끝나며 커널에 남지 않는다.
- bare(`STYLE_BARE`): `take_blocking`이 부르는 `wait()`다. waiters를 올리고 `FUTEX_WAIT`로 잠든 뒤 내린다.
- parked(`STYLE_PARKED`): `futex_waitv`가 없는 커널(5.16 미만, JetPack 6)의 executor다. 아래 절.

crash config(`CRASH`)에서는 번호 1을 쓰는 bare 대기자가 하나 더 있다. 한 번 기다리고 끝나며, 그 전 어느 스텝에서든 죽을 수 있다. 죽으면 커널이 park을 풀고 칸은 남는다. 발행자의 sweep은 죽은 번호의 lock을 잡을 수 있으므로 그 번호의 칸을 0으로 만든다(`clear_reader`). 모델은 sweep을 죽은 뒤 언제든 일어날 수 있는 스텝으로 둔다. 실제 코드는 대기자가 있는 동안 1초에 한 번 sweep한다.

재시작 config(`RESTART_AFTER`)에서는 발행자가 새 세그먼트로 옮긴다. 직전 take가 비었으면 다음 take가 signpost epoch를 확인하고 재attach한다. 이때 executor는 gate를 새 세그먼트로 옮긴 뒤 다시 샘플한다. 옛 word에 남은 대기는 없다. 옛 word는 다시 움직이지 않으므로 회복은 tick이 몬다. tick은 교체가 일어난 뒤에만 켠다.

## parker fallback

`STYLE_PARKED`에서 executor는 wake word를 직접 기다리지 않는다. source의 parker 스레드가 그 word에서 자고, word가 움직이면 제어 word(`Control::bell`)를 올려 깨운다. executor는 제어 word를 마지막으로 들은 값(`heard`)으로 기다린다.

```text
executor: sample -> take -> park(): reclaim, (word가 바뀌었으면) post + 옛 word wake -> bell 대기 -> drain
parker:   stop 확인 -> 겨냥 읽기(바뀌었으면 taken 기록) -> word 대기(seen과 비교) -> 움직였으면 ring
drop:     stop -> 빈 겨냥 post + 옛 word wake -> join
```

- 겨냥(`Aim`)은 executor만 쓰는 seqlock이다. 모델은 post 하나를 한 스텝으로 두고, parker는 그 스냅샷을 통째로 읽는다. seqlock 자체의 ordering은 `litmus/seqlock.c`와 Miri가 본다
- 바뀐 word의 주인(mapping)은 retired로 옮겨 그 post의 세대와 함께 쥔다. `park()`는 parker가 읽었다고 기록한 세대(`taken`)가 그 세대에 닿은 것만 놓는다. word는 채널, parker의 현재 주인, retired 가운데 하나가 쥐는 동안만 mapping이 있다
- parker가 word를 만지는 곳(futex 대기의 비교, 깬 뒤의 load)에서 mapping이 없으면 `V_UNMAPPED`다
- 재시작 config에서는 재attach가 parker를 새 세그먼트 word로 다시 겨냥한다. 첫 겨냥은 빈 겨냥에서 옮기는 것이라 parker가 자던 `Aim::generation`을 깨운다

의도한 예외: parker의 futex 대기는 word 하나만 비교한다. stop 표시도, 채널 word에서 자는 동안의 겨냥 변화도 비교하지 않는다. 그래서 parker가 stop을 확인했거나 겨냥을 읽은 뒤, 잠들기 전에 stop이나 새 post가 끼면 그 뒤의 wake는 놓친다. parker는 대기 상한 `PARK_NS`(50 ms)에 깨어 다시 본다. 모델은 이 경우에만 timeout 스텝(`park_timeout`)을 허용한다. 잠들 때 이미 stop이 서 있었거나, 채널 word에서 잠들 때 읽은 것보다 새 post가 있었던 경우다. 그 밖의 대기가 timeout으로만 풀린다면 그것은 잃은 wake이고 `V_LOST_WAKE`로 잡힌다. 늦어지는 것은 프레임이 아니라 깨움이다. 새 겨냥은 pass가 샘플한 seq에서 시작하므로 그 사이 온 프레임은 timeout 뒤 곧바로 ring된다. stop의 늦음은 join이 최대 50 ms 기다리는 것이다.

## 위반

| 위반 | 뜻 |
| --- | --- |
| `V_LOST_WAKE` | 배달 안 된 최신 프레임을 두고 구독자가 영원히 잠든다 |
| `V_STALE_WAIT` | 교체된 세그먼트에 붙은 채 회복하지 못한다 |
| `V_SKIP_PARKED` | 잠든 waiter가 있는데 발행자가 wake를 건너뛴다 |
| `V_GATE` | executor가 waiter gate 없이 기다린다 |
| `V_LEAK` | 죽은 대기자를 sweep한 뒤에도 그 몫이 gate에 남는다. 그 뒤 publish마다 wake syscall을 낸다 |
| `V_UNMAPPED` | parker가 mapping이 풀린 word를 만진다 |

## teeth

| 주입 | 바꾸는 것 | 위반 |
| --- | --- | --- |
| `BUG_SAMPLE_AFTER` | take 뒤에 샘플 | `V_LOST_WAKE` |
| `BUG_NO_RESYNC` | epoch 변화 무시 | `V_STALE_WAIT` |
| `BUG_NO_REGATE` | 재attach 때 새 세그먼트 `add_waiter` 생략 | `V_GATE` |
| `BUG_GATE_EARLY` | 발행자가 bump 전에 waiters 읽기 | `V_SKIP_PARKED` |
| `BUG_NO_ANNOUNCE` | waiters를 올리지 않고 park | `V_SKIP_PARKED` |
| `BUG_PARK_NOCHECK` | 커널의 word 확인 없이 잠들기 | `V_LOST_WAKE` |
| `BUG_TWO_STORES` | 이전 프로토콜. 헤더 합계와 번호 칸을 store 두 번으로 올린다 | `V_LEAK` |
| `BUG_GATE_ONE_NUMBER` | 발행자가 번호 0의 칸만 읽는다 | `V_SKIP_PARKED` |
| `BUG_REAIM_NO_WAKE` | post 뒤 옛 word를 깨우지 않는다(parked) | `V_LOST_WAKE` |
| `BUG_RECLAIM_EARLY` | retired를 `taken`과 상관없이 놓는다(parked, 재시작) | `V_UNMAPPED` |
| `BUG_SEEN_AT_POST` | 겨냥의 시작 값을 pass의 샘플이 아니라 post 때의 word로 둔다(parked) | `V_LOST_WAKE` |
| `BUG_HEARD_AT_SLEEP` | executor가 마지막으로 들은 값이 아니라 잠들 때의 bell 값으로 기다린다(parked) | `V_LOST_WAKE` |

## 커버리지

필요한 카운터가 0이면 `VACUOUS`(exit 3)로 실패한다. 공통으로 `deliver`, `park`, `wake`, `wake_void`, `skip_idle`을 요구한다. executor는 `eagain`, bare는 `dodge`(커널 확인이 잠들기를 거부한 경우)를 더 요구한다. 재시작 config는 `reattach`, `tick`, `deliver_b`(교체된 세그먼트에서의 배달)도 요구한다. crash config는 `crash_counted`(칸을 남기고 죽음)와 `swept`(sweep이 칸을 지움)를 요구한다. parked는 `ring`(parker가 제어 word를 울림)과 `window`(stop 틈의 timeout)를, 재시작과 함께면 `reaim`(옛 word에서 옮김), `window_aim`(겨냥 틈의 timeout), `reclaimed`(retired를 놓음)도 요구한다.

`make vacuous`는 교체 뒤 프레임이 없는 config를 돌린다. 이 config는 회복만 하고 새 세그먼트에서의 배달은 보이지 못하므로 거절되어야 한다.

## 범위

parked는 source 하나와 그 parker 하나다. fork한 자식이 parker를 새로 띄우는 경로는 모델에 없다. crash config를 빼면 구독자는 하나다. `FUTEX_WAKE(INT_MAX)`는 잠든 waiter를 각각 깨우므로 구독자별 결론이 여러 구독자로 일반화된다. take와 borrow의 내부는 "최신 ticket 읽기"로 추상화한다. 그 부분은 `readers/`가 맡는다.
