# Contracts between the bindings

C++ and Python call the same engine (`flux_core`), but different code calls it. A behavior that holds on only one side cannot be written as "flux does this". This document gathers in one place the contracts both bindings must keep, and records for each contract which test catches it on both sides.

`scripts/check_contracts.py` reads this table. If a test the table names does not actually exist, it fails. Deleting or renaming a test is caught here. When a new contract is established, add the row first and keep it in §2 until tests exist on both sides.

## 1. What both sides keep

| id | Contract | C++ | Python |
| --- | --- | --- | --- |
| X-001 | `peek` returns the same frame repeatedly and `take` consumes it | `Channel.PeekRepeatsAndTakeConsumes` | `test_peek_repeats_and_take_consumes` |
| X-002 | `depth=1` delivers only the newest frame. The number skipped is captured in `lost` | `Channel.DepthOneDeliversNewestOnly` | `test_depth_one_delivers_newest_only` |
| X-003 | `depth=n` catches up on backlogged frames in publish order | `Channel.DepthNCatchesUpInOrder` | `test_depth_n_catches_up_in_order` |
| X-004 | `Volatile` does not deliver what was published before attaching | `Channel.VolatileSkipsBacklog` | `test_volatile_skips_the_backlog_and_transient_local_replays_it` |
| X-005 | `TransientLocal(n)` replays n of what remains in the ring | `Channel.TransientLocalReplaysBacklog` | `test_volatile_skips_the_backlog_and_transient_local_replays_it` |
| X-006 | A QoS combination that cannot be honored is refused | `Channel.QosRejectsUnhonourableCombinations` | `test_unhonourable_qos_is_rejected` |
| X-007 | A publish larger than the slot is refused | `Channel.OversizedPublishRejected` | `test_oversized_publish_rejected` |
| X-008 | With no new frame, `take` returns empty | `Channel.PeekBeforePublishIsEmpty` | `test_take_empty_returns_none` |
| X-009 | A view may outlive the subscription that issued it | `Channel.ViewOutlivesTheChannelThatIssuedIt` | `test_view_outlives_subscription` |
| X-010 | A held view blocks slot reuse, and releasing it unblocks | `Channel.HeldFrameNotOverwritten` | `test_borrow_blocks_publish_then_releases_on_gc` |
| X-011 | `max_borrow` caps the number of views held concurrently | `Channel.MaxBorrowCapsConcurrentViews` | `test_take_blocking_does_not_park_when_max_borrow_is_held` |
| X-012 | A loan that is not committed is not published | `Channel.LoanInProgressNotVisibleUntilCommit` | `test_uncommitted_loan_is_safe_to_drop` |
| X-013 | Writing directly into a loan and committing round-trips without a copy | `Channel.LoanCommitZeroCopyRoundTrip` | `test_zero_copy_roundtrip` |
| X-014 | `take_blocking` waits until the next frame arrives | `Wake.PublishWakesBlockedSubscriber` | `test_take_blocking_waits_for_the_next_frame` |
| X-015 | The subscription keeps receiving across a publisher restart | `FluxExecutor.KeepsDeliveringAcrossPublisherRestart` | `test_take_blocking_recovers_from_a_publisher_restart` |
| X-016 | The mapping of a dead publisher is released | `RosCoexist.SubscriptionDropsADeadPublisherMapping` | `test_subscription_drops_a_dead_publisher_mapping` |
| X-017 | An empty result caused by `max_borrow` exhaustion is counted | `Channel.ExhaustedMaxBorrowIsCountedNotSilent` | `test_an_exhausted_lease_is_visible_and_counted` |
| X-018 | An empty result caused by no frame is not counted as a refusal | `Channel.AnEmptyStreamIsNotARefusal` | `test_an_idle_stream_is_not_counted_as_refused` |
| X-019 | An attach whose config mismatches a live publisher throws a different type from one that does not exist yet | `Shm.ConfigMismatchIsDistinctFromAnAbsentSegment` | `test_config_mismatch_is_distinct_from_an_absent_segment` |
| X-020 | Both ends report their domain, and the value equals what core decides | `RosDomain.EndpointsReportTheDomainTheyResolvedAndCoreAgrees` | `test_endpoints_report_the_domain_they_resolved_and_core_agrees` |
| X-021 | Two flux topics pair in one synchronizer on the header stamp | `MessageFilters.SynchronizesTwoFluxTopics` | `test_two_flux_inputs_pair_on_the_header_stamp` |
| X-022 | A flux topic and a DDS topic pair in one synchronizer | `MessageFilters.SynchronizesAFluxTopicWithARosTopic` | `test_flux_and_dds_inputs_pair_in_one_synchronizer` |
| X-023 | spin refuses when the inputs of one synchronizer span several threads | `SyncGroup.InputsInDifferentGroupsAreRefused` | `test_partitioned_refuses_sync_inputs_split_across_groups` |
| X-024 | A synchronizer input that no thread runs is refused | `SyncGroup.AnUnassignedFluxInputIsRefused` | `test_partitioned_refuses_an_unassigned_flux_input` |
| X-025 | An input whose placement cannot be determined is counted, not judged | `SyncGroup.AnUnplaceableInputIsCountedNotJudged` | `test_an_unplaceable_input_is_counted_not_judged` |

## 2. What only one side keeps

Entries here are not contracts. They are points where the two sides diverge, and they stay until it is decided to align them or leave them diverged.

Currently empty.

## 3. What is deliberately left diverged

This differs from §2. The entries below are decisions, not open items. They are points where the layers of the two bindings differ, so the same check cannot be placed at the same spot.

A row holds only the name of the divergence and the tests on both sides. The reason for the divergence is in the paragraph below the table and is not repeated in the row. If a row held a behavior description, that description would diverge from the code, and it actually did. The old S-001 row said C++ returns `false` and raises `dropped`, but the return type changed to `Published` and `dropped` counts only `Backpressure`. The test written from that row was wrong along with it (it asserted `dropped() == 1`, and outside the DeviceHandle runner it was skipped, so nobody saw it).

`scripts/check_contracts.py` reads this table too. The named tests must actually exist, and if there is no place for a test on one side, write `-`. One row cannot be `-` on both sides.

| id | Divergence | C++ | Python |
| --- | --- | --- | --- |
| S-001 | `publish` of host bytes into a device channel | `GpuVmmChannel.PublishOfAHostBufferIsRefused` | `test_publish_of_a_host_array_into_a_device_channel_is_refused` |
| S-002 | Relative topic names | `-` | `test_relative_topic_rejected` |
| S-003 | A synchronizer mixing flux inputs and DDS inputs under `PartitionedExecutor` | `SyncGroup.AMixedGraphInOneGroupIsAccepted` | `test_partitioned_refuses_a_mixed_flux_and_dds_synchronizer` |
| S-004 | The name and arguments of `PartitionedExecutor` child thread scheduling | `PartitionedExecutor.ScheduleReachesTheChildThread` | `test_a_group_child_runs_where_it_was_declared` |

S-001 comes from dGPU slots being VRAM. Accepting a host array would require an H2D copy, and `flux_core` has no copy primitive, so both sides refuse. What diverges is the form of the refusal. C++ returns `Published` and Python raises.

S-002 comes from core not interpreting the channel key as a ROS topic. `flux_cpp` passes only absolute names, since both pub and sub go through `resolve_topic_name()`, so there is no place to refuse. `flux_py`, which has no node and cannot resolve, refuses with `require_absolute`.

S-003 comes from rclpy not exposing `add_callback_group`. C++ can put flux subscriptions and ROS subscriptions together in that group and place it on one thread. Python has no means to move ROS entities as a group, so DDS inputs stay on the node thread. So the same graph passes in C++ and is refused in Python. The place to run that graph in Python is `flux.ros.Executor`. There, X-022 holds.

S-004 diverges only in name and arguments. Both sides agree that the child applies it to itself before its first callback, that a refusal becomes an exception from `spin()`, and that a declaration that can never apply is refused. What diverges is that C++ `schedule(group, opts, strict, control_priority)` takes `Strictness` and a control loop priority and attaches to a chain declaration via `RtStage`, while Python `set_thread_scheduling(unit, policy=, priority=, cpus=)` takes none of those three. Python also accepting a node as the declaration unit comes from the same partition as S-003 (flux by group, ROS by node).

The C++ `-` in S-002 is not a missing check. The argument of `Channel::create`/`open` is a channel key, not a ROS topic, and core stands without ROS. The refusal lives where `flux::ros::Publisher`/`Subscription` resolve through the node.
