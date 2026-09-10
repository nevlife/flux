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
| S-004 | `PartitionedExecutor` 자식 스레드 스케줄링의 이름과 인자 | `PartitionedExecutor.ScheduleReachesTheChildThread` | `test_a_group_child_runs_where_it_was_declared` |

S-001은 dGPU 슬롯이 VRAM인 데서 온다. host 배열을 받으면 H2D 복사가 필요한데 `flux_core`는 복사 primitive를 두지 않으므로 양쪽 다 거절한다. 갈린 것은 거절의 형태다. C++은 `Published`로 돌려주고 Python은 던진다.

S-002는 core가 채널 키를 ROS 토픽으로 해석하지 않는 데서 온다. `flux_cpp`는 pub·sub 양쪽이 `resolve_topic_name()`을 거쳐 절대 이름만 넘기므로 거절할 자리가 없고, 노드가 없어 resolve를 못 하는 `flux_py`는 `require_absolute`로 거절한다.

S-003은 rclpy가 `add_callback_group`을 안 내주는 데서 온다. C++은 그 그룹에 flux 구독과 ROS 구독을 같이 담아 한 스레드에 놓을 수 있고, Python은 ROS entity를 그룹째 옮길 수단이 없어 DDS 입력이 노드 스레드에 남는다. 그래서 같은 그래프가 C++에서는 통과하고 Python에서는 거절된다. Python에서 그 그래프를 돌리는 자리는 `flux.ros.Executor`다 -- 거기서는 X-022가 성립한다.

S-004는 이름과 인자만 갈린다. 자식이 첫 콜백 전에 자기에게 적용하는 것, 거절이 `spin()`의 예외가 되는 것, 적용될 리 없는 선언을 거절하는 것은 양쪽이 같다. 갈린 것은 C++ `schedule(group, opts, strict, control_priority)`이 `Strictness`와 control loop 우선순위를 받고 `RtStage`로 체인 선언에 붙는다는 점이고, Python `set_thread_scheduling(unit, policy=, priority=, cpus=)`은 그 셋을 안 든다. Python이 선언 단위로 노드도 받는 것은 S-003과 같은 분할(flux는 그룹, ROS는 노드)에서 온다.

S-002의 C++이 `-`인 것은 검사를 빠뜨린 것이 아니다. `Channel::create`/`open`의 인자는 ROS 토픽이 아니라 채널 키이고 core는 ROS 없이 선다. 거절은 `flux::ros::Publisher`/`Subscription`이 노드로 resolve하는 자리에 있다.
