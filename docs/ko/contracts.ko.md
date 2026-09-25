# 바인딩 간 계약

C++과 Python은 같은 엔진(`flux_core`)을 부르지만 서로 다른 코드가 부른다. 한쪽에서만 성립하는 동작은 "flux가 이렇게 한다"고 쓸 수 없다. 이 문서는 두 바인딩이 함께 지켜야 하는 규약을 한 곳에 모으고, 각 규약을 어느 테스트가 잡는지 양쪽 다 적는다.

`scripts/check_contracts.py`가 이 표를 읽는다. 표가 지목한 테스트가 실제로 없으면 실패한다 -- 테스트를 지우거나 이름을 바꾸면 여기서 걸린다. 규약을 새로 세우면 행을 먼저 추가하고, 양쪽에 테스트가 생길 때까지 §2에 둔다.

## 1. 양쪽이 지키는 것

| id | 규약 | C++ | Python |
| --- | --- | --- | --- |
| X-001 | `peek`는 같은 프레임을 반복해 주고 `take`는 소비한다 | `Channel.PeekRepeatsAndTakeConsumes` | `test_peek_repeats_and_take_consumes` |
| X-002 | `depth=1`은 최신 프레임만 준다. 건너뛴 수는 `lost`에 잡힌다 | `Channel.DepthOneDeliversNewestOnly` | `test_depth_one_delivers_newest_only` |
| X-003 | `depth=n`은 밀린 프레임을 발행 순서대로 따라잡는다 | `Channel.DepthNCatchesUpInOrder` | `test_depth_n_catches_up_in_order` |
| X-004 | `Volatile`은 붙기 전 발행분을 안 준다 | `Channel.VolatileSkipsBacklog` | `test_volatile_skips_the_backlog_and_transient_local_replays_it` |
| X-005 | `TransientLocal(n)`은 ring에 남은 것 중 n개를 재생한다 | `Channel.TransientLocalReplaysBacklog` | `test_volatile_skips_the_backlog_and_transient_local_replays_it` |
| X-006 | 지킬 수 없는 QoS 조합은 거절한다 | `Channel.QosRejectsUnhonourableCombinations` | `test_unhonourable_qos_is_rejected` |
| X-007 | 슬롯보다 큰 publish는 거절한다 | `Channel.OversizedPublishRejected` | `test_oversized_publish_rejected` |
| X-008 | 새 프레임이 없으면 `take`는 빈 값을 준다 | `Channel.PeekBeforePublishIsEmpty` | `test_take_empty_returns_none` |
| X-009 | view는 그것을 발급한 구독보다 오래 살아도 된다 | `Channel.ViewOutlivesTheChannelThatIssuedIt` | `test_view_outlives_subscription` |
| X-010 | 쥔 view는 슬롯 재사용을 막고, 놓으면 풀린다 | `Channel.HeldFrameNotOverwritten` | `test_borrow_blocks_publish_then_releases_on_gc` |
| X-011 | `max_borrow`가 동시에 쥐는 view 수를 캡한다 | `Channel.MaxBorrowCapsConcurrentViews` | `test_take_blocking_does_not_park_when_max_borrow_is_held` |
| X-012 | 커밋하지 않은 loan은 발행되지 않는다 | `Channel.LoanInProgressNotVisibleUntilCommit` | `test_uncommitted_loan_is_safe_to_drop` |
| X-013 | loan에 직접 써서 commit하면 복사 없이 왕복한다 | `Channel.LoanCommitZeroCopyRoundTrip` | `test_zero_copy_roundtrip` |
| X-014 | `take_blocking`은 다음 프레임이 올 때까지 기다린다 | `Wake.PublishWakesBlockedSubscriber` | `test_take_blocking_waits_for_the_next_frame` |
| X-015 | 발행자가 재시작해도 구독은 계속 받는다 | `FluxExecutor.KeepsDeliveringAcrossPublisherRestart` | `test_take_blocking_recovers_from_a_publisher_restart` |
| X-016 | 죽은 발행자의 매핑을 놓는다 | `RosCoexist.SubscriptionDropsADeadPublisherMapping` | `test_subscription_drops_a_dead_publisher_mapping` |
| X-017 | `max_borrow` 소진으로 빈 것이 나가면 그것을 센다 | `Channel.ExhaustedMaxBorrowIsCountedNotSilent` | `test_an_exhausted_lease_is_visible_and_counted` |
| X-018 | 프레임이 없어서 빈 것은 거절로 세지 않는다 | `Channel.AnEmptyStreamIsNotARefusal` | `test_an_idle_stream_is_not_counted_as_refused` |
| X-019 | 산 발행자와 config가 어긋난 attach는 아직 없는 것과 다른 타입으로 던진다 | `Shm.ConfigMismatchIsDistinctFromAnAbsentSegment` | `test_config_mismatch_is_distinct_from_an_absent_segment` |
| X-020 | 양 끝이 자기 domain을 보고하고, 그 값이 core가 정하는 것과 같다 | `RosDomain.EndpointsReportTheDomainTheyResolvedAndCoreAgrees` | `test_endpoints_report_the_domain_they_resolved_and_core_agrees` |
| X-021 | flux 토픽 둘이 header stamp로 한 synchronizer에서 짝을 맺는다 | `MessageFilters.SynchronizesTwoFluxTopics` | `test_two_flux_inputs_pair_on_the_header_stamp` |
| X-022 | flux 토픽과 DDS 토픽이 한 synchronizer에서 짝을 맺는다 | `MessageFilters.SynchronizesAFluxTopicWithARosTopic` | `test_flux_and_dds_inputs_pair_in_one_synchronizer` |
| X-023 | 한 synchronizer의 입력이 여러 스레드에 걸리면 spin이 거절한다 | `SyncGroup.InputsInDifferentGroupsAreRefused` | `test_partitioned_refuses_sync_inputs_split_across_groups` |
| X-024 | 어느 스레드도 안 돌리는 synchronizer 입력은 거절한다 | `SyncGroup.AnUnassignedFluxInputIsRefused` | `test_partitioned_refuses_an_unassigned_flux_input` |
| X-025 | 자리를 정할 수 없는 입력은 판정하지 않고 센다 | `SyncGroup.AnUnplaceableInputIsCountedNotJudged` | `test_an_unplaceable_input_is_counted_not_judged` |
| X-026 | 구독의 `MemoryPolicy`가 거부되면 attach 안 된 상태로 두지 않고 던진다 | `PullSurface.ARefusedMemoryPolicyThrowsInsteadOfLookingUnattached` | `test_a_refused_lock_on_a_subscription_raises_rather_than_looking_unattached` |
| X-027 | ROS executor의 `spin`이 도는 동안 `spin_once`는 거부되고 spin은 계속 돈다 | `FluxExecutor.ASpinOnceDuringASpinIsRefusedAndLeavesTheSpinRunning` | `test_spin_once_rejected_while_spinning` |
| X-028 | ROS executor의 `spin` 전에 온 `stop`은 그 spin을 끝내고, 끝날 때 지워진다 | `FluxExecutor.AStopBeforeSpinEndsItAndIsClearedOnTheWayOut` | `test_a_stop_before_spin_ends_it_and_is_cleared_on_the_way_out` |
| X-029 | `PartitionedExecutor`의 `spin` 전에 온 `stop`은 그 spin을 끝내고, 끝날 때 지워진다 | `PartitionedExecutor.AStopBeforeSpinEndsItAndIsClearedOnTheWayOut` | `test_a_stop_before_a_partitioned_spin_ends_it_and_is_cleared_on_the_way_out` |
| X-030 | ROS executor의 `spin_once`는 두 전송 중 먼저 온 일에 반환한다 | `FluxExecutor.ASpinOnceReturnsOnTheFirstWorkOfEitherTransport` | `test_spin_once_returns_on_the_first_work_of_either_transport` |
| X-031 | domain은 앞자리 0 없이 렌더한 정수이고, 그 밖의 값은 거절한다 | `DomainTest.CanonicalRendersTheParsedInteger` | `test_a_domain_is_a_canonical_integer` |
| X-032 | `PartitionedExecutor`에 넘긴 reentrant 그룹은 넘기는 호출에서 거절한다 | `PartitionedExecutor.AReentrantGroupIsRefusedAtAdd` | `test_reentrant_group_is_refused` |
| X-033 | unit이 없는 `on_thread_start` hook은 그 호출에서 거절한다 | `PartitionedExecutor.OnThreadStartRejects` | `test_a_hook_for_no_unit_is_refused_at_the_call` |
| X-034 | 산 발행자의 세그먼트를 열 수 없으면 attach 안 된 채 두지 않고 던진다 | `PullSurface.ASegmentThatCannotBeOpenedWhileItsPublisherLivesThrows` | `test_a_segment_that_cannot_be_opened_while_its_publisher_lives_raises` |
| X-035 | 있지만 읽을 수 없는 signpost는 발행자 없음으로 읽지 않고 던진다 | `PullSurface.ASignpostThatCannotBeReadThrows` | `test_a_signpost_that_cannot_be_read_raises` |
| X-036 | 185자를 넘는 토픽은 자르지 않고 생성 때 거절한다 | `PullSurface.ATopicPastTheNameLimitIsRefused` | `test_a_key_past_the_name_limit_is_refused` |
| X-037 | 0차원 배열은 원소 하나다 | `Channel.AZeroDimensionalFrameCarriesOneElement` | `test_a_zero_dimensional_array_round_trips` |

## 2. 한쪽만 지키는 것

여기 있는 항목은 규약이 아니다. 양쪽이 갈린 지점이고, 붙이거나 갈린 채로 두기로 정할 때까지 남는다.

지금은 비어 있다.

## 3. 갈린 채로 두기로 한 것

§2와 다르다. 아래는 미결이 아니라 결정이다. 두 바인딩의 층이 달라 같은 검사를 같은 자리에 둘 수 없는 지점이다.

행은 갈린 지점의 이름과 양쪽 테스트만 든다. 갈린 이유는 표 아래 문단이 들고 행에는 다시 쓰지 않는다 -- 행이 동작 서술을 들면 그 서술이 코드와 갈라지고, 실제로 갈라졌다. S-001의 옛 행은 C++이 `false`를 돌려주고 `dropped`를 올린다고 적고 있었는데, 반환형은 `Published`로 바뀌었고 `dropped`는 `Backpressure`만 센다. 그 행을 읽고 쓴 테스트도 같이 틀려 있었다(`dropped() == 1`을 단언했고, DeviceHandle 러너 밖에서는 skip이라 아무도 못 봤다).

`scripts/check_contracts.py`가 이 표도 읽는다. 이름 붙은 테스트가 실제로 있어야 하고, 그 쪽에 테스트를 둘 자리가 없으면 `-`로 적는다. 한 행이 양쪽 다 `-`일 수는 없다.

| id | 갈린 것 | C++ | Python |
| --- | --- | --- | --- |
| S-001 | device 채널에 host 바이트를 `publish` | `GpuVmmChannel.PublishOfAHostBufferIsRefused` | `test_publish_of_a_host_array_into_a_device_channel_is_refused` |
| S-002 | 상대 토픽 이름 | `-` | `test_relative_topic_rejected` |
| S-003 | `PartitionedExecutor`에서 flux 입력과 DDS 입력을 섞은 synchronizer | `SyncGroup.AMixedGraphInOneGroupIsAccepted` | `test_partitioned_refuses_a_mixed_flux_and_dds_synchronizer` |
| S-004 | `PartitionedExecutor` `on_thread_start` hook의 단위 | `PartitionedExecutor.OnThreadStartRunsOnTheChildBeforeItsFirstCallback` | `test_a_node_hook_runs_on_the_node_thread` |
| S-005 | QoS를 만드는 방식, enum 표기, 구독이 QoS를 받는 자리 | `Channel.QosSetterRejectsAValueWrongOnItsOwn` | `test_unhonourable_qos_is_rejected` |
| S-006 | `flux.ros.Executor.close()` | `-` | `test_stop_is_idempotent_and_close_detaches` |
| S-007 | 인자만 보고 틀린 발행: 크기 초과, rank 8 초과, `commit(nbytes)` 오용 | `Channel.OversizedPublishRejected` | `test_oversized_publish_rejected` |
| S-008 | 이미 넣은 노드로 `add_ros_node` | `FluxExecutor.AddingTheSameNodeTwiceThrowsAsRclcppDoes` | `test_add_ros_node_is_idempotent` |
| S-009 | 콜백 없는 구독을 넣을 때의 에러 종류 | `FluxExecutor.AddingASubscriptionWithNoCallbackIsRefused` | `test_adding_a_subscription_with_no_callback_is_refused` |
| S-010 | `flux.ros.Executor`가 두 transport를 합치는 방식과, 그래서 없는 직접 루프 호출(`dispatch`, `wait_for_work`, `pump_ros`, 예산) | `FluxExecutor.TheInheritedSpinOnceServicesRosEntities` | `test_spin_once_returns_on_the_first_work_of_either_transport` |
| S-011 | message_filters `Subscriber`를 나중에 붙이기(기본 생성, `subscribe`, `unsubscribe`) | `MessageFilters.SubscribesLate` | `-` |

S-001은 dGPU 슬롯이 VRAM인 데서 온다. host 배열을 받으면 H2D 복사가 필요한데 `flux_core`는 복사 primitive를 두지 않으므로 양쪽 다 거절한다. 갈린 것은 거절의 형태다. C++은 `Published`로 돌려주고 Python은 던진다.

S-002는 core가 채널 키를 ROS 토픽으로 해석하지 않는 데서 온다. `flux_cpp`는 pub·sub 양쪽이 `resolve_topic_name()`을 거쳐 절대 이름만 넘기므로 거절할 자리가 없고, 노드가 없어 resolve를 못 하는 `flux_py`는 `require_absolute`로 거절한다.

S-003은 rclpy가 `add_callback_group`을 안 내주는 데서 온다. C++은 그 그룹에 flux 구독과 ROS 구독을 같이 담아 한 스레드에 놓을 수 있고, Python은 ROS entity를 그룹째 옮길 수단이 없어 DDS 입력이 노드 스레드에 남는다. 그래서 같은 그래프가 C++에서는 통과하고 Python에서는 거절된다. Python에서 그 그래프를 돌리는 자리는 `flux.ros.Executor`다 -- 거기서는 X-022가 성립한다.

S-004는 hook의 단위가 갈린다. hook을 그 단위의 자식에서 첫 콜백 전에 실행하는 것, hook의 예외를 `spin()`의 오류로 올리는 것, 한 단위에 두 번째 hook과 자식이 없는 단위의 hook을 거절하는 것은 양쪽이 같다. C++은 콜백 그룹을 받는다. Python은 노드도 받는데, S-003과 같은 분할(flux는 그룹, ROS는 노드)에서 온다.

S-005는 ROS를 따른다. ROS도 두 언어가 대칭이 아니다. 각 쪽은 그 언어에서 ROS를 쓰는 방식대로 쓴다. C++은 `rclcpp::QoS`처럼 `flux::QoS`에 setter를 잇고, rclcpp처럼 enum 값을 `Cpu`처럼 CamelCase로 쓰고, `create_subscription(topic, qos, callback)`처럼 QoS를 콜백 앞에서 받는다. Python은 `QoSProfile`처럼 QoS를 키워드로 받고, rclpy처럼 enum 값을 `CPU`처럼 대문자로 쓰고, `create_subscription(type, topic, callback, qos)`처럼 콜백을 키워드 앞에서 받는다. setter를 이으면 두 필드를 한 번에 검사할 수 없으므로 C++은 `n > depth`를 QoS를 쓰는 자리에서 검사한다. Python은 `flux.QoS(...)`에서 검사한다.

S-006은 GC에서 온다. `flux.ros.Executor`는 노드를 rclpy executor에 붙이고, 노드는 한 번에 rclpy executor 하나에만 붙는다. 그래서 `close()`가 객체가 수거되는 시점이 아니라 호출자가 고른 시점에 노드를 뗀다. C++은 같은 일을 시점이 정해진 소멸자에서 하므로 `close()`가 없다. `PartitionedExecutor`는 `spin()`이 돌아온 뒤 풀 것이 없어 두 언어 모두 `close()`가 없다.

S-007은 호출자 실수를 알리는 각 언어의 관례를 따른다. C++ 발행 경로는 `noexcept`라(D-065) `Published::TooLarge`를 돌려주고, `Published`가 `[[nodiscard]]`라 결과를 버리는 호출은 컴파일 경고다. Python은 무엇이 틀렸고 어느 경로가 되는지를 메시지에 담은 `ValueError`를 던진다. C++ 리터럴 모양이 8개를 넘으면 컴파일 에러다. 실수가 아닌 `Backpressure`는 두 언어 모두 반환값이다.

S-008은 각 언어의 ROS executor를 따른다. rclcpp는 이미 executor에 넣은 노드에 `std::runtime_error`를 던지고, rclpy `add_node`는 이미 든 노드면 조용히 돌아온다.

S-009는 같은 거절을 각 언어의 타입으로 낸다. C++은 `std::invalid_argument`, Python은 호출 가능한 것이 빠졌을 때의 관례대로 `TypeError`다.

S-010은 rclpy에서 온다. C++이 io_uring에 거는 on-new-message 콜백을 rclpy가 내주지 않아, `flux.ros.Executor`는 ROS 준비 상태를 flux 링에 넣을 수 없다. 브릿지 스레드가 flux 쪽을 기다리다 `create_task`로 dispatch를 rclpy spin 스레드에 넘긴다. 링이 하나가 아니므로 호출자가 손으로 돌릴 pass가 없고, 직접 루프 호출은 C++에만 있다. flux만 도는 루프라면 Python core `flux.Executor`에 `dispatch`와 `wait_for_work`가 있다.

S-011은 upstream message_filters를 따른다. upstream C++ `Subscriber`는 나중에 붙일 수 있고 Python `Subscriber`는 그렇지 않다. 안쪽 구독의 이름도 각 언어의 upstream대로 C++은 `getSubscriber()`, Python은 `.sub`다.

S-002의 C++이 `-`인 것은 검사를 빠뜨린 것이 아니다. `Channel::create`/`open`의 인자는 ROS 토픽이 아니라 채널 키이고 core는 ROS 없이 선다. 거절은 `flux::ros::Publisher`/`Subscription`이 노드로 resolve하는 자리에 있다.
