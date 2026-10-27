# flux 명령

`flux_cli`가 `flux` 명령을 설치한다. 워크스페이스를 source하면 PATH에 있다. 이 명령은 Python이나 ROS 없이 돈다. `/dev/shm`을 읽기만 하고 lock이나 슬롯을 잡지 않는다. 그래서 대상 프로세스의 동작에 영향을 주지 않는다.

```text
flux topic list
flux topic info <channel>
flux topic hz <channel> [--window SEC]
```

명령을 실행한 사용자가 열 수 있는 채널만 다룬다. 다른 사용자의 채널은 보이지 않는다.

## topic list

채널마다 한 줄이다.

```text
CHANNEL         SEGMENT  ENDPOINTS
--------------  -------  ------------
/clidemo/image  up       1 pub, 1 sub
```

| 열 | 뜻 |
| --- | --- |
| `CHANNEL` | 채널 이름. 이름을 알리는 endpoint가 없으면 `/dev/shm` 파일 이름에서 이름을 복원하고 뒤에 `(from name)`을 붙인다 |
| `SEGMENT` | `up`이면 발행자가 떠 있다. `down`이면 구독만 남아 발행자를 기다린다 |
| `ENDPOINTS` | 발행자와 구독자 수 |

## topic info

```text
channel      /clidemo/image
fingerprint  0x0000000000000000
signpost     /flux.v6..clidemo.image.0000000000000000
segment      up, epoch 1
slots        4 x 1 MiB
subscribers  up to 16 processes
storage      host
published    134 frames on this segment
parked       1 subscriber(s) on the wake gate
endpoints
  ROLE  PID      LABEL
  ----  -------  ----------------------
  pub   1587957  /clidemo/camera_driver
  sub   1587957  /clidemo/detector
```

- `epoch`는 발행자가 재시작할 때마다 바뀐다. `published`는 이 세그먼트에서 commit된 프레임 수다.
- `parked`는 프레임을 기다리며 잠든 구독자 수다. 0보다 크면 발행자는 발행할 때마다 futex wake를 한 번 보낸다.
- `storage`는 슬롯 데이터가 있는 곳이다. `host`는 공유메모리, `cuda-ipc`는 장치 메모리(`DeviceHandle`, [gpu.md](gpu.md))다.
- `LABEL`은 노드의 전체 이름이다.
- endpoint 목록에는 62개까지 들어간다. 그보다 많으면 나머지는 표시하지 않고, 생략했다는 줄을 하나 덧붙인다. 이 제한은 프레임 전달과 상관없다.

## topic hz

```text
$ flux topic hz /clidemo/image
sampling /clidemo/image every 1s (Ctrl-C to stop)
     98.99 Hz   (99 frames in 1.00s)
```

`--window`로 준 초(기본 1, 최소 0.001)마다 발행 속도를 출력한다. 속도는 발행자가 commit한 프레임 수로 잰다. 그래서 어느 구독자가 받았는지와 상관없다. 발행자가 재시작하면 그 구간은 표시만 하고 다음 구간부터 다시 잰다. Ctrl-C로 멈춘다.

## 종료 코드

| 코드 | 뜻 |
| --- | --- |
| 0 | 성공 |
| 1 | 채널이 없거나, 이름이 fingerprint가 다른 채널 여럿과 맞거나, `hz`의 채널에 발행자가 없다 |
| 2 | 인자가 틀렸다 |
