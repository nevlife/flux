# flux 명령

`flux_cli`가 내는 `flux` 명령. 이 호스트의 flux 채널을 들여다본다.

읽기만 한다. 열거는 락을 안 잡고, 통계 경로는 락도 borrow도 안 잡는다. 그래서 도는 시스템에 붙여도 교란하지 않고, 대상 프로세스는 이 명령이 돌았다는 것을 알 수 없다.

ROS도 DDS도 안 쓴다. 데몬리스다. 데이터가 `/dev/shm`에 있으므로 ROS가 안 깔린 배포에서도 그대로 돈다.

## 명령

```bash
flux topic list                      # 채널 목록
flux topic info /cam/left            # 채널 하나 상세
flux topic hz /cam/left              # 발행 주기
flux domain list                      # 이 호스트의 domain과 내 것
```

`--domain`으로 볼 domain을 고르고, `--all-domains`으로 전부 본다. 기본값은 노드가 쓰는 것과 같은 규칙으로 정한다(`FLUX_DOMAIN`, 없으면 `ROS_DOMAIN_ID`, 없으면 `0`). 두 플래그는 verb에 붙는다 — `flux topic list --all-domains`이 사람이 타이핑하는 자리다.

## list

```text
CHANNEL        DOMAIN  SEGMENT  ENDPOINTS
-------------  -----  -------  ------------
/cam/left      lab    up       1 pub, 1 sub
/lidar/points  lab    down     1 sub
```

`SEGMENT`가 `down`인 것은 고장이 아니다. signpost는 영구라 발행자가 없어도 이름은 남는다.

채널 이름 뒤에 `(from name)`이 붙으면 그 이름은 산 참가자가 아니라 `/dev/shm` 이름에서 되읽은 것이다. 이름은 alnum이 아닌 문자를 전부 `.`으로 바꾸므로 `/a/b`와 `.a.b`가 한 이름이다. 표시가 없으면 진짜 key다.

조회는 둘 다 받는다. `info`와 `hz`에 `/cam/left`를 줘도 `.cam.left`를 줘도 같은 채널을 찾는다. 발행자가 죽은 채널은 되읽은 철자로만 보이는데, 채널을 조회하는 상황이 대개 그 상황이다.

## info

```text
channel      /cam/left
domain        lab
fingerprint  0x0000000000000abc
signpost     /flux.v7.slab..cam.left.0000000000000abc
segment      up, epoch 1
slots        8 x 1 MiB
storage      host
published    1594 frames on this segment
parked       0 subscriber(s) on the wake gate
endpoints
  ROLE  PID      LABEL
  ----  -------  -----
  pub   1108498  -
  sub   1108498  -
```

`LABEL`은 경계층이 announce한 값이다. `flux_cpp`는 노드의 fully qualified name을 넣고, `flux_py`는 노드가 없어 비운다.

같은 key에 fingerprint가 다른 채널이 둘 있으면 이름으로 못 가르므로 둘 다 보여주고 거절한다. 그 둘은 별개 채널이고 서로 통신하지 않는다.

## hz

```text
sampling /cam/left every 1s (Ctrl-C to stop)
     29.99 Hz   (30 frames in 1.00s)
     29.99 Hz   (30 frames in 1.00s)
```

세그먼트 헤더의 `publish_seq`를 두 번 읽어 차를 낸다. 그 값은 monotone한 티켓 발행기이므로 결과가 추정이 아니라 그 구간의 정확한 프레임 수다. 구독자로 붙지 않으므로 `max_borrow`도 holder 테이블도 차지하지 않고, `flux topic info`로 본 endpoint 수가 `hz`가 도는 동안에도 안 변한다.

`epoch`가 바뀌면 발행자 그룹이 재시작한 것이다. 카운터가 새것이라 그 경계를 넘는 차는 프레임 수가 아니므로 세지 않고 알린다.

## domain list

```text
DOMAIN      CHANNELS  UP  ENDPOINTS
---------  --------  --  ---------
0          96        0   0
10 (here)  2         1   2

more than one domain here: processes in different domains never meet.
domain comes from FLUX_DOMAIN, else ROS_DOMAIN_ID, else '0'.
```

`(here)`가 이 프로세스가 해석한 domain다. `--domain`을 안 받는다 — 이 명령이 답하는 것은 "환경이 나를 어디에 두는가"이고, 덮어쓰면 묻지 않은 질문에 답하게 된다.

노드가 보는 값과 같은 규칙, 같은 시점으로 정한다(`process_domain()`). 그래서 이 명령의 `(here)`와 같은 셸에서 띄운 노드의 `domain`이 어긋날 수 없다.

해석한 domain은 비어 있어도 행으로 남는다. 없는 것이 곧 확인하러 온 값이라서다. 행이 둘 이상이면 그 사실을 문장으로 말한다.

이것이 답하는 상황은 하나다. 구독이 프레임을 못 받는데 `refused`가 전부 0인 경우다. 발행자가 아직 없는 것과, 발행자가 다른 domain에서 도는 것이 겉이 같다. 후자면 그 발행자가 다른 행에 나타난다. 노드 쪽에서 같은 것을 묻는 것은 `Subscription`의 `domain`과 `attached`다(`api.ko.md`).

## 없는 것

`echo`는 없다. 프레임을 보려면 진짜 구독자로 붙어야 하고, 그러면 `max_borrow`와 holder 테이블을 차지한다. 관측이 대상을 교란하기 시작하는 지점이라 위 셋과 같은 묶음에 넣지 않았다.
