# flux_core API (ROS 없이)

`flux_core`는 ROS를 모른다. rclcpp도 rclpy도 안 부르고, 단독 cmake로 빌드된다. 이 문서는 ROS 노드 밖에서 flux를 쓰는 표면이다. ROS 노드 안이면 [api.md](api.ko.md)가 맞고, `.msg` 없이 쓰는 얘기는 [raw_api.md](raw_api.ko.md)다.

## 1. 무엇이 다른가

`flux::ros::Publisher`와 `flux::ros::Subscription`이 하는 일은 셋이다. 그 셋을 직접 하면 `flux::Channel`이 남는다.

| ROS 래퍼가 하는 일 | 직접 할 때 |
| --- | --- |
| 토픽 이름을 노드 네임스페이스·remap으로 푼다 | 절대 이름을 직접 준다 |
| 이름·fingerprint·domain에서 signpost를 만든다 | `flux::signpost_name()`을 부른다 |
| 콜백을 executor에 물린다 | `take()`를 직접 부르거나 `wait()`한다 |

`Channel` 자체는 양쪽이 같은 객체다. 래퍼는 `Channel` 하나를 들고 있을 뿐이라 동시성 규약·복사 모델·QoS가 전부 그대로 성립한다.

## 2. 빌드

```cmake
find_package(flux_core REQUIRED)
target_link_libraries(mytool PRIVATE flux::core)
```

ament 워크스페이스 밖이면 `flux_core`만 따로 cmake로 세운다. 의존은 pthread뿐이다. `flux_cpp`·`flux_py`·`flux_gen`은 필요 없다.

## 3. 이름 만들기

rendezvous 키는 이름과 fingerprint와 domain다. 셋이 같아야 붙는다. ROS 래퍼가 안 끼므로 이름을 직접 절대 경로로 준다.

```cpp doc:core_name
std::string domain = flux::process_domain();
std::string name = flux::signpost_name("/img", /*fingerprint=*/0, domain);
```

`process_domain()`는 `FLUX_DOMAIN`을 먼저 보고 없으면 `ROS_DOMAIN_ID`를 본다. 둘 다 없으면 `0`이다. ROS를 안 쓰더라도 같은 호스트의 ROS 노드와 붙으려면 그 노드가 쓰는 domain과 같아야 한다.

첫 호출에서 한 번 정하고 프로세스가 끝날 때까지 안 바뀐다. rcl이 `ROS_DOMAIN_ID`를 context init에서 래치하는 것과 같다 — 그래야 한 프로세스의 ROS 쪽과 flux 쪽이 다른 구획에 앉지 않는다. 환경을 그때그때 읽어보는 것은 `resolve_domain(var)`이고, 그쪽은 질의라 이름을 만드는 데 쓰지 않는다.

ROS 래퍼와 붙일 때는 토픽 이름이 remap 이후의 것이어야 한다. 노드가 `img`를 `/robot1/img`로 풀었으면 여기에도 `/robot1/img`를 준다.

## 4. 발행

```cpp doc:core_publish
flux::Channel ch = flux::Channel::create(
  flux::signpost_name("/img", flux::kNoSchema), /*slot_size=*/16 << 20, /*slot_count=*/16,
  flux::kNoSchema);

flux::WriteSlot w = ch.loan(flux::DType::U8, {height, width, 3});
if (w) {
  render_into(w.data(), w.capacity());
  if (w.commit() != flux::Published::Ok) sink(ch.dropped());
}
```

`create`가 세그먼트를 만든다. 이미 산 발행자가 있고 `slot_size`나 `slot_count`가 어긋나면 `flux::SegmentMismatch`를 던진다. 재시도로 안 고쳐지므로 잡지 않는다.

`loan`·`commit`·`publish`의 의미는 ROS 래퍼와 같다. dtype과 shape은 `loan`에서 한 번 말하고 `commit`은 인자를 받지 않는다. 반환값 해석은 [raw_api.md](raw_api.ko.md) 2절이고, `.msg` 어댑터를 쓰면 `Builder`가 프레임을 대신 채운다 -- 어댑터는 `WriteSlot` 위에 얹히므로 ROS 없이도 그대로 쓴다.

### 페이지 사전 커밋

`create`·`open`의 마지막 인자가 `flux::MemoryPolicy`다. 이 프로세스의 매핑에 대한 페이지 residency이고 기본은 꺼짐이다.

```cpp doc:core_memory_policy
flux::MemoryPolicy mem;
mem.precommit = true;  // attach 때 전 페이지 fault-in
mem.lock = true;       // mlock까지. RLIMIT_MEMLOCK이 세그먼트를 덮어야 한다

flux::Channel committed = flux::Channel::create(
  flux::signpost_name("/img", flux::kNoSchema), 4096, 8, flux::kNoSchema, {}, mem);
sink(committed.pages_committed(), committed.pages_locked());
```

세부는 [api.md](api.ko.md) 3절의 `MemoryPolicy` 절에 있다. 요점 셋이다. 프로세스마다 따로 선언한다. 재attach마다 다시 적용된다. 거부는 조용히 낮추지 않고 `std::system_error`로 던진다.

Python은 `flux.MemoryPolicy`가 같은 것이다(아래 7절).

## 5. 소비

```cpp doc:core_subscribe
flux::Channel ch = flux::Channel::open(flux::signpost_name("/img", 0), /*fingerprint=*/0);
ch.qos(flux::QoS{});

flux::FrameView v = ch.take();
if (v) {
  use(v.data(), v.size());
  v.release();
}
sink(ch.lost(), ch.refused().total());
```

`open`은 발행자가 아직 없으면 던진다. 래퍼가 하던 지연 attach를 직접 해야 한다 -- 잡아서 재시도한다. fingerprint나 layout version이 어긋나는 경우는 `SegmentMismatch`이고 이건 재시도가 무의미하다.

`qos()`는 생성 뒤에 준다. `depth`·`durability`·`max_borrow`의 뜻은 [qos.md](qos.ko.md)와 같다.

`peek()`은 최신 프레임을 소비하지 않고 준다. `take()`는 발행 순서로 다음 것을 소비한다. `take_blocking(timeout_ns)`은 올 때까지 기다린다.

## 6. 기다리기

콜백을 물릴 executor가 없으므로 깨어나는 것을 직접 한다.

```cpp doc:core_wait
ch.add_waiter();
const std::uint32_t seen = ch.wake_seq();
if (!ch.take()) {
  ch.wait(seen, /*timeout_ns=*/-1);
}
ch.remove_waiter();
```

`add_waiter()`가 없으면 발행자가 깨우는 syscall을 아예 안 낸다. 등록하지 않고 `wait()`하면 timeout까지 안 깨어난다.

`wake_seq()`를 `take()`보다 먼저 읽는 순서가 중요하다. 반대로 하면 그 사이에 들어온 발행을 놓치고 park한다. 읽어 둔 값을 `wait()`에 넘기면 그 사이 발행이 있었는지 커널이 판정한다.

`orphaned()`가 true면 발행자 그룹이 죽었고 rotation도 안 온다. 그 채널은 다시 안 살아난다.

### 여러 채널을 한 번에

채널이 여럿이면 `flux::Executor`가 전부를 io_uring 하나로 기다린다. ROS를 모르는 클래스이고, `flux::ros::Executor`와 `flux.Executor`가 둘 다 이것 위에 있다 -- 대기 순서를 정하는 코드가 언어마다 갈리지 않게 한 곳에 둔 것이다.

무엇을 기다릴지는 `flux::Source`로 말한다. 채널 하나와, 그 프레임으로 무엇을 할지다.

```cpp doc:core_source
class Sink : public flux::Source
{
public:
  explicit Sink(flux::Channel && ch) : ch_(std::move(ch)) {}

  bool attach() override { return true; }  // already open; a lazy one would open here

  int deliver_one() override
  {
    flux::FrameView v = ch_.take();
    if (!v) return 0;
    handle(v);
    return 1;
  }

  flux::Channel * channel() noexcept override { return &ch_; }

private:
  flux::Channel ch_;
};
```

`attach()`는 멱등이고 발행자 세그먼트가 아직 없으면 false를 돌려준다. `deliver_one()`은 프레임 하나만 처리하고 1을, 없으면 0을 돌려준다. 한 번에 하나인 이유는 executor가 콜백 사이마다 다음에 어느 채널을 돌릴지 다시 고르기 때문이다. 여러 개를 소유자가 직접 빼고 싶으면 `deliver(max)`를 쓴다. 이건 virtual이 아니고 executor가 쓰는 경로도 아니다.

```cpp doc:core_executor
flux::Executor ex(32);
ex.add(a);
ex.add(b, 10);  // priority: b goes first whenever it has a frame
ex.set_pass_budget(16);  // callbacks one dispatch() may run over all channels together
ex.spin(100'000'000);  // stop()까지
ex.stop();
```

등록은 spin 전에만 한다. `add`의 둘째 인자는 우선순위다. 큰 것이 먼저 가고 같으면 등록순이다. 기본값은 0이고 음수도 쓴다. 선택은 콜백마다 다시 한다 -- 낮은 채널의 콜백이 도는 중에 높은 채널로 프레임이 오면 낮은 채널의 다음 프레임보다 그것이 먼저 돈다. 선점 우선순위는 아니다. 이미 도는 콜백은 밀리지 않는다.

`set_pass_budget()`은 `dispatch()` 한 번이 돌릴 콜백 수의 상한이다. 채널마다가 아니라 채널 전체에 대한 하나의 예산이고 기본값은 `flux::kMaxDrain`(64)이다. spin 전에만 부른다. 처리율이 아니라 pass 길이를 묶는다 -- 예산이 자른 나머지는 다음 pass가 블록 없이 이어 받는다. 낮추면 높은 우선순위 채널이 낮은 채널 뒤에서 기다리는 시간이 줄고, 올리면 pass당 arm 비용이 더 많은 프레임에 나뉜다. `uses_io_uring()`이 false면 리눅스 6.7 미만이라 채널당 parker 스레드 폴백으로 돈다 -- 기다리는 스레드는 여전히 한 번만 블록한다.

남의 이벤트 루프에 얹을 때는 둘로 나눠 쓴다. 같은 ring을 쓰므로 동시에 돌면 안 된다.

```cpp doc:core_executor_split
ex.wait_for_work(ex.has_more() ? 0 : -1);  // 예산이 자른 나머지가 있으면 블록하지 않는다
const int delivered = ex.dispatch();       // 전달만 한다. 블록하지 않는다
```

`has_more()`는 직전 `dispatch()`가 프레임이 남은 채로 예산에서 멈췄는지다. 이미 와 있던 프레임에 대해 아무도 ring을 다시 두드리지 않으므로, 이 상태에서 블록하면 tick 하나를 자기가 들고 있는 일감 위에서 잔다. `spin()`은 `dispatch()`의 반환값으로 같은 판단을 하므로 이 플래그를 안 쓴다.

Python은 `flux.Executor`가 같은 클래스를 감싼 것이다(아래 8절).

## 7. Python (ROS 없이)

`flux`만 import하면 ROS가 안 끼어든다. `flux.ros`가 하는 일은 이름 해석과 rclpy executor 연결뿐이다.

```python doc:core_py_publisher
pub = flux.Publisher("/img", fingerprint=FP, slot_size=16 << 20, slot_count=16)
pub.publish(arr)
```

```python doc:core_py_subscription
sub = flux.Subscription("/img", fingerprint=FP, qos=flux.QoS())
v = sub.take()
if v is not None:
    _sink(v.shape)
```

이름은 절대 경로여야 한다. 상대 이름을 주면 던진다 -- 풀어 줄 노드가 없기 때문이다.

`memory` 인자가 페이지 사전 커밋이다. C++의 `flux::MemoryPolicy`와 같은 것이고 필드 이름도 같다.

```python doc:core_py_memory_policy
mem = flux.MemoryPolicy(precommit=True, lock=True)
pub = flux.Publisher("/img", fingerprint=FP, slot_size=4096, slot_count=8, memory=mem)
sub = flux.Subscription("/img", fingerprint=FP, memory=mem)
```

`precommit`은 attach 때 전 페이지를 fault-in 한다. `lock`은 `mlock`까지 하고 `precommit`을 포함한다. 거부는 `OSError`다 -- 선언한 상한을 못 받은 채로 도는 채널을 만들지 않는다. `flux.ros.Publisher`·`flux.ros.Subscription`도 같은 인자를 그대로 받는다.

## 8. Python executor

이 클래스는 6절의 `flux::Executor`를 그대로 감싼 것이다. 이름과 인자가 C++과 같고, 대기 순서를 정하는 코드도 같은 하나다.

```python doc:py_core_executor
ex = flux.Executor(max_channels=32, poll_tick_ns=2_000_000)
ex.add(sub, callback, priority=0)
ex.spin_once(timeout_ns=-1)
ex.spin(tick_ns=100_000_000)
ex.stop()
ex.interrupt()
merged = ex.uses_io_uring
busy = ex.is_spinning
```

`uses_io_uring`이 false면 리눅스 6.7 미만이라 채널당 스레드 폴백으로 돈다.

`priority`는 C++과 같은 값이다. pass 안 방문 순서를 정하고, 큰 것이 먼저 가고 같으면 등록순이다.

`stop()`은 요청이고 `is_spinning`은 상태다. 둘을 한 플래그로 합치지 않았으므로 `spin()`보다 먼저 온 `stop()`이 사라지지 않는다 -- 스레드를 띄우자마자 멈추는 코드가 그 창에 걸리지 않는다. 요청은 그것을 본 `spin()`이 나가면서 지우므로 같은 executor를 다시 spin할 수 있다. 이미 도는 executor에 `spin()`을 또 부르면 던진다.

`spin_once(timeout_ns)`는 준비된 것을 처리하고 없으면 대기한다. 처리한 개수를 돌려준다. `interrupt()`는 루프를 두고 지금 대기만 깨며, 한 번에 호출 하나만 끝낸다.

다른 이벤트 루프에 끼워 넣으려면 둘로 나눠 쓴다. 둘이 동시에 돌면 안 된다.

```python doc:py_split_wait
ex.wait_for_work(timeout_ns=-1)
delivered = ex.dispatch()
```

## 9. 열거 (도구용)

지금 이 호스트에 무슨 flux 채널이 있고 누가 붙어 있는지 읽는다. `flux.enumerate_topics()`가 `flux.Topic` 목록을 주고, 각 topic이 `flux.Endpoint` 목록을 든다. 데몬리스이고 레지스트리도 없다 -- `/dev/shm` 이름과, owner 락을 아직 쥐고 있는 프로세스의 manifest를 그 자리에서 읽어 만든 스냅샷이다. `flux_cli`가 이 표면 위에 있다([cli.md](cli.ko.md)).

```python doc:py_enumerate
for topic in flux.enumerate_topics():
    name = topic.key if topic.key_exact else topic.signpost
    for ep in topic.endpoints:
        role = "pub" if ep.publisher else "sub"
        _sink(name, topic.domain, topic.fingerprint, role, ep.pid, ep.starttime, ep.label)
```

`key_exact`가 False면 `key`는 산 참가자가 아니라 이름에서 되읽은 것이다. 이름은 alnum이 아닌 문자를 전부 `.`으로 바꾸므로 `/a/b`와 `.a.b`가 한 이름이다. 그때 보이는 것은 peer가 실제로 합의한 key가 아니라 이름의 철자다. 참가자가 하나도 없는 채널이 그 경우다(signpost는 영구라 이름만 남는다).

`flux.flatten_key(key)`가 그 변환이다. 사용자가 친 이름을 열거 결과와 맞춰야 하는 도구는 이것을 통해 비교한다. 규칙을 도구가 다시 적으면 포맷이 조용히 갈라진다. `flux topic info`가 발행자가 죽은 채널을 원래 이름으로 찾는 것이 이것이다.

`label`은 경계층이 announce한 값이다. `flux_cpp`는 노드의 fully qualified name을 넣고, `flux_py`는 노드가 없으므로 비운다. core는 이 문자열을 해석하지 않는다.

죽은 참가자는 안 나온다. owner 파일의 OFD 락이 풀려 있으면 그 manifest는 아무것도 서술하지 않으므로 건너뛴다. 건너뛸 뿐 지우지는 않는다 -- 지우는 것은 sweep의 일이다.

이름 하나당 open 하나가 든다. 도구에서 쓰고 루프에서 쓰지 않는다.

채널 하나를 붙지 않고 들여다볼 때는 `flux.read_channel_stats()`가 `flux.ChannelStats`를 준다. 락도 borrow도 안 잡는다.

```python doc:py_channel_stats
domain = flux.process_domain()

stats = flux.read_channel_stats(signpost)
if stats.live:
    _sink(stats.slot_count, stats.slot_size, stats.storage_kind, stats.fingerprint)
    _sink(stats.publish_seq, stats.epoch, stats.waiters)
```

`publish_seq`는 monotone한 티켓 발행기다. 두 번 재서 차를 내면 그 구간의 정확한 프레임 수다. 추정이 아니다. `epoch`가 바뀌면 발행자 그룹이 재시작한 것이고 카운터가 새것이므로 그 경계를 넘는 차는 프레임 수가 아니다. `live`가 False면 지금 세그먼트를 올린 발행자가 없다는 뜻이고, signpost는 영구라 그것이 채널의 정상적인 휴지 상태다.

`process_domain()`는 노드가 쓰는 것과 같은 규칙으로, 같은 시점 규약으로 domain을 정한다. 규칙이 core에 한 벌만 있으므로 도구가 같은 답을 두 번 유도하지 않는다. 환경을 지금 다시 읽어보려면 `flux.resolve_domain("ROS_DOMAIN_ID")`이고, domain 변수를 아예 안 보려면 `flux.resolve_domain(None)`이다.

`read_channel_stats`는 구독자로 붙지 않는다. 헤더만 읽으므로 `max_borrow`도 holder 테이블도 안 건드린다. 관측이 대상을 교란하지 않는다.
