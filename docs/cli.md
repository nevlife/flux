# flux 명령

`flux_cli`가 설치하는 `flux` 명령은 워크스페이스를 source하면 PATH에 들어가고, Python과 ROS 없이 실행된다. `/dev/shm`을 읽기만 하고 lock을 획득하거나 슬롯을 점유하지 않으므로 대상 프로세스에 영향을 주지 않는다.

```text
flux topic list
flux topic info <channel>
flux topic hz <channel> [--window SEC]
```

- 실행한 사용자의 채널만 보인다.
- bash에서는 워크스페이스를 source하면 Tab 자동완성이 활성화된다. 하위 명령, `--window`, 이 호스트의 채널 이름을 완성한다.

## topic list

채널마다 한 줄을 출력한다.

```text
CHANNEL         SEGMENT  ENDPOINTS
--------------  -------  ------------
/clidemo/image  up       1 pub, 1 sub
```

| 열 | 뜻 |
| --- | --- |
| `CHANNEL` | 채널 이름. 이름을 알리는 엔드포인트가 없으면 `/dev/shm` 파일 이름에서 복원하고 `(from name)`을 추가한다 |
| `SEGMENT` | `up`은 발행자가 실행 중, `down`은 구독만 남아 발행자를 기다리는 중 |
| `ENDPOINTS` | 발행자 수와 구독자 수 |

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

| 줄 | 뜻 |
| --- | --- |
| `epoch` | 발행자 재시작마다 바뀌는 번호 |
| `published` | 이 세그먼트에서 commit된 프레임 수 |
| `parked` | 프레임을 기다리며 잠든 구독자 수. 0보다 크면 발행자는 발행마다 futex wake를 한 번 보낸다 |
| `storage` | 슬롯 데이터의 위치. `host`는 공유 메모리, `cuda-ipc`는 GPU 메모리(`DeviceHandle`, [gpu.md](gpu.md)) |
| `LABEL` | 노드의 전체 이름 |

엔드포인트 목록은 62개까지 표시하고, 초과분은 생략했다는 줄 하나로 대신한다. 이 제한은 프레임 전달과 관계가 없다.

## topic hz

```text
$ flux topic hz /clidemo/image
sampling /clidemo/image every 1s (Ctrl-C to stop)
     98.99 Hz   (99 frames in 1.00s)
```

- `--window`로 지정한 초(기본 1, 최소 0.001)마다 발행 속도를 출력한다.
- 속도는 발행자가 commit한 프레임 수로 측정하므로 구독자의 수신과 관계가 없다.
- 발행자가 재시작한 구간은 표시만 하고 다음 구간부터 다시 측정한다.
- Ctrl-C로 종료한다.

## 종료 코드

| 코드 | 뜻 |
| --- | --- |
| 0 | 성공 |
| 1 | 채널이 없거나, 이름이 fingerprint가 다른 채널 여럿과 일치하거나, `hz`의 채널에 발행자가 없다 |
| 2 | 인자가 잘못되었다 |
