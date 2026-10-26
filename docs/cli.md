# flux 명령

`flux_cli`가 `flux` 명령을 설치한다. 워크스페이스를 source하면 PATH에 있다. Python이나 ROS 없이 돈다. `/dev/shm`을 읽기만 하고 lock이나 슬롯을 잡지 않으므로 대상 프로세스는 이를 알아채지 못한다.

```text
flux topic list
flux topic info <channel>
flux topic hz <channel> [--window SEC]
```

이 사용자가 열 수 있는 채널만 다룬다. 다른 사용자의 채널은 보이지 않는다.

## topic list

채널마다 한 줄이다.

```text
CHANNEL         SEGMENT  ENDPOINTS
--------------  -------  ------------
/clidemo/image  up       1 pub, 1 sub
```

| 열 | 뜻 |
| --- | --- |
| `CHANNEL` | 채널 이름. 이름을 알리는 endpoint가 없는 채널은 `/dev/shm` 파일 이름에서 되살린 이름에 `(from name)`을 붙인다 |
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
- `parked`는 프레임을 기다리며 잠든 구독자 수다. 0보다 크면 발행마다 futex wake가 한 번 나간다.
- `storage`는 슬롯 데이터가 있는 곳이다. `host`는 공유메모리, `cuda-ipc`는 장치 메모리(`DeviceHandle`, [gpu.md](gpu.md))다.
- `LABEL`은 노드의 전체 이름이다.
- 목록은 62칸까지다. 넘는 endpoint는 표시되지 않고, 그렇다고 한 줄을 덧붙인다. 프레임 전달과는 상관없다.

## topic hz

```text
$ flux topic hz /clidemo/image
sampling /clidemo/image every 1s (Ctrl-C to stop)
     98.99 Hz   (99 frames in 1.00s)
```

`--window` 초(기본 1, 최소 0.001)마다 발행 속도를 찍는다. 발행자가 commit한 프레임 수로 재므로, 어느 구독자가 받았는지와 무관하다. 발행자가 재시작하면 그 구간은 표시만 하고 다음 구간부터 다시 잰다. Ctrl-C로 멈춘다.

## 종료 코드

| 코드 | 뜻 |
| --- | --- |
| 0 | 성공 |
| 1 | 채널이 없거나, 이름이 fingerprint가 다른 채널 여럿과 맞거나, `hz`의 채널에 발행자가 없다 |
| 2 | 인자가 틀렸다 |
