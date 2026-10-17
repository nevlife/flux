# sweep: 이름과 sweep

`/dev/shm`의 이름, 객체, lock을 모델로 고정하고 모든 인터리빙에서 safety를 검사한다. 대상 코드는 `flux2_core/src/discovery.rs`의 `SignpostMap::open`·`leave`(drop), `open_publisher_segment`, `remove_if_dead`, `sweep_dead`와 `segment.rs`의 `Segment::drop`이다. 이 코드를 고치면 여기부터 다시 돌린다.

sweeper끼리 차례를 정하는 lock(`/flux2.sweep`)은 없다. 객체마다 자기 lock과 이름 재확인만으로 안전하다는 것을 이 모델이 확인한다.

## 모델

이름은 signpost 하나와 세그먼트 `.e1`~`.eN`이다. 이름은 객체(inode) 하나를 가리키거나 비어 있다. 프로세스는 fd(open file description)를 셋까지 쥐고, OFD lock은 fd에 붙는다. 다른 fd끼리는 read lock 둘만 함께 설 수 있고, fd가 닫히면 그 lock이 풀린다. 바이트는 둘이다. 생존 바이트(signpost의 `SIGNPOST_USER_BYTE`, 세그먼트의 `PUBLISHER_LOCK_BYTE`)와 signpost의 `SIGNPOST_PUBLISHER_BYTE`다.

| 행위자 | 단계 |
| --- | --- |
| signpost 열기 | `open(O_CREAT)` -> 생존 바이트 read lock -> 이름이 아직 그 객체인지 확인. 실패하면 닫고 다시 |
| 발행자 | signpost 열기 -> 발행자 바이트 write lock(실패하면 `PublisherExists`) -> epoch 읽기 -> 앞 epoch 세그먼트 `remove_if_dead` -> `O_EXCL` 생성(있으면 `remove_if_dead`, 못 지우면 다음 epoch) -> read lock -> 이름 재확인 -> epoch 저장. 끝낼 때 세그먼트 unlink, 닫기, signpost 떠나기 |
| 구독자 | signpost 열기 -> epoch 읽기 -> 세그먼트 열기 -> 발행자 read lock 확인. 끝낼 때 세그먼트 닫기, signpost 떠나기 |
| signpost 떠나기 | read lock을 write lock으로 올리고, 되면 unlink. 안 되면 자기 read lock을 풀고 write lock을 한 번 더 시도해, 되면 이름을 재확인한 뒤 unlink. 그리고 닫기 |
| sweeper | 이름마다 `remove_if_dead`: 열기 -> 생존 바이트 write lock -> 이름 재확인 -> unlink -> 닫기 |

시작 상태에는 앞서 죽은 프로세스가 남긴 signpost(epoch 1)와 `.e1`이 있다(`LEFTOVER`). 프로세스는 어느 단계에서든 죽을 수 있다(`MAX_CRASHES`). 죽으면 fd가 모두 닫힌다.

## 불변식

| 위반 | 뜻 |
| --- | --- |
| `INV_UNLINK_LIVE` | 다른 프로세스가 생존 바이트에 read lock을 쥔 객체, 곧 살아 있는 객체를 가리키는 이름을 누군가 unlink한다. 자기 세그먼트를 지우는 발행자는 제외 |
| `INV_REBOUND` | 프로세스가 기대는 이름(연 signpost, 살아 있는 발행자의 세그먼트)이 그 객체가 아닌 것을 가리킨다 |
| `INV_LEAK` | 아무도 죽지 않고 모두 끝났는데 이름이 남아 있다 |

## teeth

| 주입 | 바꾸는 것 | 위반 |
| --- | --- | --- |
| `BUG_RM_NO_CHECK` | `remove_if_dead`가 이름 재확인을 건너뜀 | `INV_UNLINK_LIVE` |
| `BUG_LOCK_AFTER_CHECK` | 재확인한 뒤에 lock | `INV_UNLINK_LIVE` |
| `BUG_RM_NO_LOCK` | lock 없이 재확인만 하고 unlink | `INV_UNLINK_LIVE` |
| `BUG_DROP_NO_LOCK` | signpost를 떠날 때 lock 없이 unlink | `INV_UNLINK_LIVE` |
| `BUG_OPEN_NO_CHECK` | signpost를 열 때 이름 재확인을 건너뜀 | `INV_REBOUND` |
| `BUG_CREATE_NO_CHECK` | 세그먼트를 만든 뒤 이름 재확인을 건너뜀 | `INV_REBOUND` |
| `BUG_DROP_NO_RETRY` | 올리기가 실패하면 그냥 닫음(전의 코드) | `INV_LEAK` |
| `BUG_DROP_NO_RECHECK` | 다시 잡은 write lock 뒤 이름 재확인을 건너뜀 | `INV_UNLINK_LIVE` |

`BUG_DROP_NO_RETRY`는 이 모델이 찾은 실제 결함이다. 마지막 사용자 둘이 함께 떠나면 둘 다 상대의 read lock 때문에 올리기에 실패해 signpost가 남았다. 다음 프로세스의 sweep이 지우기 전까지다. `BUG_DROP_NO_RECHECK`는 고친 코드에 재확인이 왜 필요한지 보인다. read lock을 푼 사이 sweeper가 이름을 지우고 다른 프로세스가 새로 만들 수 있다.

## 커버리지

| 카운터 | 0이면 증명 못 하는 것 |
| --- | --- |
| `swept` | 무엇이든 지운 경우 |
| `refused` | 지우려는 쪽이 잡힌 lock을 만난 경우 |
| `rebound` | 지우려는 쪽이 다른 객체로 바뀐 이름을 만난 경우 |
| `open_retry` | signpost 열기가 다시 시도한 경우 |
| `create_retry` | 새 세그먼트가 lock 전에 지워진 경우 |
| `live` | 발행자가 살아난 경우 |
| `attached` | 구독자가 붙은 경우 |
| `exists` | 두 번째 발행자가 첫째를 만난 경우(발행자 둘일 때) |
| `drop_retry` | 두 사용자가 함께 signpost를 떠난 경우(사용자 둘 이상일 때) |
| `dead_holder` | 생존 lock을 쥔 채 죽은 경우(`MAX_CRASHES > 0`일 때) |

`make vacuous`는 게이트 자체를 확인한다. 남은 것도 sweeper도 없으면 지울 것이 없어 `VACUOUS`(exit 3)여야 한다.

## config

| config | 대상 | 상태 수 |
| --- | --- | --- |
| 기본 | sweeper 둘, 발행자, 구독자, 죽음 1회 | 5,701,368 |
| `RESTART` | 발행자 둘(재시작이나 거절), 구독자, sweeper, 죽음 1회 | 3,773,447 |
| `USERS` | 발행자, 구독자 둘, sweeper, 죽음 1회 | 1,796,459 |

## 범위

SC 인터리빙에서 알고리즘만 검증한다. 다음은 다루지 않는다.

- fstat의 소유자 확인. 모델의 프로세스는 모두 같은 사용자다
- signpost bootstrap(초기화 내용), payload, 구독자 번호 lock. `readers/`가 번호를 다룬다
- 무작위 epoch. 못 지운 이름 다음은 모델에서 다음 번호다
- inode 번호 재사용. 열린 fd가 있는 동안 커널은 번호를 다시 쓰지 않고, 모델은 번호를 다시 쓰지 않는다

실제 프로세스로는 `flux2_core/tests/sweep_stress.rs`가 확인한다.
