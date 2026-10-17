# readers: reader 칸과 회수

flux2의 구독 슬롯 보호를 모델로 고정하고, bounded config의 모든 인터리빙에서 safety를 검사한다. 대상 코드는 `flux2_core/src/channel.rs`의 `claim`·`take`·`reclaim_dead`·`abort`와 `reader.rs`의 `lease_in`이다. 이 코드를 고치면 여기부터 다시 돌린다.

## 모델

슬롯의 ticket이 상태를 겸한다. 0은 빈 슬롯, `CLAIMED`는 발행자가 쓰는 중, 그 밖의 값은 그 프레임의 ticket이다.

발행자는 하나다. 칸이 모두 0이고 `CLAIMED`가 아닌 슬롯 중 ticket이 가장 작은 슬롯을 고른다. ticket을 `CLAIMED`로 바꾼 뒤 번호마다 칸을 하나씩 다시 읽는다(Dekker 재확인). 0이 아닌 칸을 만나면 ticket을 원래 값으로 되돌려 claim을 버린다. 아직 payload를 쓰지 않았으므로 원래 프레임은 그대로다. 통과하면 다음 순서로 쓴다.

```text
payload -> ticket -> latest
```

`LOAN_ABORTS` config에서는 발행자가 payload를 쓴 뒤 commit 대신 loan을 버릴 수 있다(`WriteSlot` drop). 이미 payload가 원래 프레임을 덮었으므로 ticket은 빈 슬롯(0)이 된다.

빈 슬롯이 없으면 한 번 회수한다. 칸이 0이 아닌 번호마다 lock을 시도하고, 잡히면 주인이 죽은 것이므로 그 번호의 칸을 모두 0으로 만든 뒤 lock을 푼다.

구독 프로세스는 빈 번호의 lock을 잡고 자기 칸을 0으로 만들고, high-water mark(지금까지 받은 가장 큰 번호 + 1)를 올린 뒤 활성화된다. 발행자의 claim 재확인과 회수는 그 아래 번호만 읽는다. claim은 `CLAIMED` 표시 뒤 mark를 한 번 읽는다. 한 프로세스의 스레드들은 번호 하나를 같이 쓴다. 스레드의 take는 QoS depth로 cursor 하한을 올리고, 가장 오래된 새 프레임을 골라 다음 순서로 검증한다. ticket이 고를 때와 다르면 칸을 1 내리고 다시 시도한다.

```text
칸 +1 -> ticket 재확인
```

crash는 프로세스 단위다. 그 프로세스의 스레드가 모두 멈추고 칸은 그대로 남으며, 커널이 lock을 푼다.

## 불변식

| 위반 | 뜻 |
| --- | --- |
| `INV_BYTELOCK` | 산 스레드가 검증을 마치고 쥔 슬롯의 payload를 발행자가 쓴다 |
| `INV_COHERENT` | 쥔 view의 payload가 검증한 ticket의 프레임이 아니다 |
| `INV_UNDERFLOW` | 칸이 0에서 내려간다 |
| `INV_FLOOR` | 산 프로세스 번호의 칸이 그 스레드들이 아직 지닌 +1보다 작다. 회수나 재가입이 산 기여를 지운 것이다 |
| `INV_LEAK` | 끝난 상태에서 산 프로세스 번호의 칸이 지닌 +1과 다르다. 죽은 주인의 잔여가 남은 것이다 |

## teeth

결함을 하나씩 넣으면 표의 위반이 반드시 나와야 한다. `make bug`가 확인한다.

| 주입 | 바꾸는 것 | 위반 |
| --- | --- | --- |
| `BUG_NO_DEKKER` | `CLAIMED` 표시 뒤 칸 재확인 생략 | `INV_BYTELOCK` |
| `BUG_INC_LAST` | ticket 재확인 뒤에 +1 | `INV_BYTELOCK` |
| `BUG_NO_CHECK` | +1 뒤 ticket 재확인 생략 | `INV_BYTELOCK` |
| `BUG_RECLAIM_NO_LOCK` | lock 없이 칸 지우기 | `INV_FLOOR` |
| `BUG_UNLOCK_EARLY` | 칸을 지우기 전에 lock 풀기 | `INV_FLOOR` |
| `BUG_JOIN_NO_ZERO` | 재가입 때 자기 칸 0으로 만들기 생략 | `INV_LEAK` |
| `BUG_ABORT_RESTORE` | 버린 loan의 슬롯에 원래 ticket을 되돌림 | `INV_COHERENT` |
| `BUG_RAISE_LATE` | 활성화한 뒤에 high-water mark를 올림 | `INV_BYTELOCK` |

`BUG_UNLOCK_EARLY`의 반례가 회수 중 lock을 쥐고 있어야 하는 이유다. 발행자가 lock을 일찍 풀면, 그 사이 새 프로세스가 같은 번호를 받아 +1 하고, 발행자가 그 +1을 지운다.

## 커버리지

config가 경쟁에 닿지 않으면 PASS는 아무것도 증명하지 않는다. 그래서 도달한 상황을 세고, config가 요구하는 카운터가 0이면 `VACUOUS`(exit 3)로 실패한다.

| 카운터 | 0이면 증명 못 하는 것 |
| --- | --- |
| `abort` | 발행자의 claim이 borrow에 진 경우 |
| `retry` | 구독자 검증이 실패한 경우 |
| `shared_hold` | 한 프로세스의 두 스레드가 같은 슬롯을 함께 쥔 경우(`N_THREADS > N_PROCS`일 때만 요구) |
| `cross_hold` | 두 프로세스가 같은 슬롯을 함께 쥔 경우 |
| `crash_holding` | 칸을 남기고 죽은 프로세스 |
| `reclaimed` | 회수가 칸을 지운 경우 |
| `refused` | 회수가 산 번호의 lock에 거절된 경우 |
| `rejoin` | 죽은 프로세스의 번호를 새 프로세스가 받은 경우 |
| `residue` | 재가입이 죽은 주인의 잔여를 지운 경우 |
| `loan_abort` | 버린 loan이 commit된 프레임을 덮은 경우 |
| `below_high` | claim이 세그먼트의 번호보다 적은 번호만 읽은 경우 |

`crash_holding`부터 `residue`까지 다섯은 `MAX_CRASHES > 0`일 때, `loan_abort`는 `LOAN_ABORTS > 0`일 때만 요구한다. `make vacuous`는 게이트 자체를 확인한다. 스레드가 하나뿐이면 슬롯 하나만 잡으므로 발행자가 굶지 않고, 회수에 도달하지 못해 거절되어야 한다.

## config

공유 번호, crash, loan abort를 한 config에 다 넣으면 상태 풀(800만)을 넘는다. 그래서 `make check`는 config 셋을 돌린다.

| config | 대상 | 상태 수 |
| --- | --- | --- |
| 기본 | 공유 번호(프로세스 2, 스레드 3), crash 없음 | 787,206 |
| `RECLAIM` | crash 1회와 재가입(프로세스 3, 번호 2), 프로세스당 스레드 1 | 2,405,582 |
| `LOAN` | 쓴 뒤 버린 loan 1회(프레임 4, 프로세스당 스레드 1) | 456,540 |

## 범위

SC 인터리빙에서 알고리즘만 검증한다. 다음은 다루지 않는다.

- `SeqCst`·`Acquire`·`Release` 지정의 정합
- OFD lock syscall 자체
- 회수 스캔에서 칸을 하나씩 읽는 동안의 비원자성. 모델은 번호 하나가 "0이 아닌 칸을 가졌는가"를 한 스텝으로 읽는다.
- claim이 같은 pass에서 이미 실패한 슬롯을 건너뛰는 것. 모델은 매번 가장 오래된 빈 슬롯을 다시 고른다. safety는 어느 빈 슬롯을 고르는지에 달려 있지 않다.
- fork. fork한 자식은 물려받은 view·loan의 해제를 건너뛰고(fork 세대 번호) 자기 번호를 새로 받는다. 모델에는 프로세스 복제가 없다. `flux2_core/tests/shm.rs`의 fork 테스트가 실제 프로세스로 확인한다.

프로세스 crash 뒤의 회수는 `flux2_core/tests/shm.rs`의 `dead_subscriber_slot_is_reclaimed`가 실제 프로세스로 확인한다.
