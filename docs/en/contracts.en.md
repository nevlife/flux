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
| X-026 | A refused `MemoryPolicy` on a subscription throws instead of leaving it unattached | `PullSurface.ARefusedMemoryPolicyThrowsInsteadOfLookingUnattached` | `test_a_refused_lock_on_a_subscription_raises_rather_than_looking_unattached` |
| X-027 | `spin_once` while the ROS executor's `spin` runs is refused, and the spin keeps running | `FluxExecutor.ASpinOnceDuringASpinIsRefusedAndLeavesTheSpinRunning` | `test_spin_once_rejected_while_spinning` |
| X-028 | A `stop` before the ROS executor's `spin` ends it and is cleared on the way out | `FluxExecutor.AStopBeforeSpinEndsItAndIsClearedOnTheWayOut` | `test_a_stop_before_spin_ends_it_and_is_cleared_on_the_way_out` |
| X-029 | A `stop` before `PartitionedExecutor`'s `spin` ends it and is cleared on the way out | `PartitionedExecutor.AStopBeforeSpinEndsItAndIsClearedOnTheWayOut` | `test_a_stop_before_a_partitioned_spin_ends_it_and_is_cleared_on_the_way_out` |
| X-030 | The ROS executor's `spin_once` returns on the first work of either transport | `FluxExecutor.ASpinOnceReturnsOnTheFirstWorkOfEitherTransport` | `test_spin_once_returns_on_the_first_work_of_either_transport` |
| X-031 | A domain is a plain integer rendered without leading zeros, and anything else is refused | `DomainTest.CanonicalRendersTheParsedInteger` | `test_a_domain_is_a_canonical_integer` |
| X-032 | A Reentrant group handed to `PartitionedExecutor` is refused at the call that hands it over | `PartitionedExecutor.AReentrantGroupIsRefusedAtAdd` | `test_reentrant_group_is_refused` |
| X-033 | An `on_thread_start` hook for no unit is refused at the call | `PartitionedExecutor.OnThreadStartRejects` | `test_a_hook_for_no_unit_is_refused_at_the_call` |
| X-034 | A live publisher's segment that cannot be opened throws instead of leaving the subscription unattached | `PullSurface.ASegmentThatCannotBeOpenedWhileItsPublisherLivesThrows` | `test_a_segment_that_cannot_be_opened_while_its_publisher_lives_raises` |
| X-035 | A signpost that exists but cannot be read throws instead of reading as no publisher | `PullSurface.ASignpostThatCannotBeReadThrows` | `test_a_signpost_that_cannot_be_read_raises` |
| X-036 | A topic past 185 characters is refused at construction instead of cut | `PullSurface.ATopicPastTheNameLimitIsRefused` | `test_a_key_past_the_name_limit_is_refused` |
| X-037 | A 0-d array is one element | `Channel.AZeroDimensionalFrameCarriesOneElement` | `test_a_zero_dimensional_array_round_trips` |
| X-038 | Within one pass a higher `priority` is visited first (`Executor`) | `FluxExecutor.PriorityReachesTheCore` | `test_priority_reaches_the_core` |
| X-039 | Within one `PartitionedExecutor` group a higher `priority` is visited first | `PartitionedExecutor.PriorityOrdersWithinAGroup` | `test_priority_orders_within_a_group` |
| X-040 | A stopped executor spins again and delivers | `FluxExecutor.AStoppedExecutorSpinsAgain` | `test_a_stopped_executor_spins_again` |
| X-041 | A callback exception ends `spin`, and the next `spin` delivers | `FluxExecutor.ACallbackExceptionEndsSpinAndTheNextSpinDelivers` | `test_a_callback_exception_ends_spin_and_the_next_spin_delivers` |
| X-042 | A second `spin` is refused and the running one keeps running | `FluxExecutor.ASecondSpinIsRefused` | `test_a_second_spin_is_refused` |
| X-043 | A subscription handed to two `PartitionedExecutor` groups is refused at the call | `PartitionedExecutor.RejectsASubscriptionAssignedTwice` | `test_rejects_a_subscription_assigned_twice` |
| X-044 | With no free slot a loan fails and `dropped` counts it | `Channel.LoanDropsWhenAllSlotsBorrowed` | `test_a_loan_with_no_free_slot_is_none` |
| X-045 | An aborted loan is not published, and the frame it overwrote is skipped | `Channel.LoanAbortDoesNotPublishOrResurrectTheOverwrittenFrame` | `test_an_aborted_loan_is_not_delivered` |
| X-046 | A loan may outlive the publisher that issued it | `Channel.LoanOutlivesTheChannelThatIssuedIt` | `test_a_loan_outlives_its_publisher` |
| X-047 | A reader behind `depth` is pulled forward and the skipped frames go to `lost` | `Channel.DepthPullsCursorForwardAndReportsLost` | `test_depth_pulls_the_cursor_forward_and_counts_lost` |
| X-048 | The ring caps `depth`, and the shortfall shows in `lost` | `Channel.RingCapsDepth` | `test_the_ring_caps_depth` |
| X-049 | A message_filters frame that does not read as the schema is counted in `unreadable` and not forwarded | `MessageFilters.AFrameThatDoesNotReadIsCountedNotForwarded` | `test_a_frame_that_does_not_read_is_counted_not_forwarded` |
| X-050 | A queued `StampedFrame` stays readable after the callback that delivered it | `MessageFilters.AQueuedFrameOutlivesTheCallbackThatDeliveredIt` | `test_a_queued_frame_outlives_the_callback_that_delivered_it` |
| X-051 | `enumerate_topics` lists a live publisher with its key, fingerprint and owner | `Enumerate.AnnouncedEndpointsComeBackWithTheirKeyAndLabel` | `test_enumerate_topics_lists_a_live_publisher` |
| X-052 | `read_channel_stats` reports a live channel's shape and counts publishes exactly | `ChannelStats.ALiveChannelReportsItsShapeAndCountsPublishes` | `test_read_channel_stats_reports_shape_and_counts_publishes` |

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
| S-004 | The unit of a `PartitionedExecutor` `on_thread_start` hook | `PartitionedExecutor.OnThreadStartRunsOnTheChildBeforeItsFirstCallback` | `test_a_node_hook_runs_on_the_node_thread` |
| S-005 | How a QoS is built, enum spelling, and where a subscription takes its QoS | `Channel.QosSetterRejectsAValueWrongOnItsOwn` | `test_unhonourable_qos_is_rejected` |
| S-006 | `flux.ros.Executor.shutdown()` | `-` | `test_stop_is_idempotent_and_shutdown_detaches` |
| S-007 | A publish argument wrong on its own: oversize, rank above 8, `commit(nbytes)` misuse | `Channel.OversizedPublishRejected` | `test_oversized_publish_rejected` |
| S-008 | `add_ros_node` with a node already added | `FluxExecutor.AddingTheSameNodeTwiceThrowsAsRclcppDoes` | `test_add_ros_node_is_idempotent` |
| S-009 | The error type for a subscription with no callback | `FluxExecutor.AddingASubscriptionWithNoCallbackIsRefused` | `test_adding_a_subscription_with_no_callback_is_refused` |
| S-010 | How `flux.ros.Executor` merges the two transports, and the direct-loop calls (`dispatch`, `wait_for_work`, `pump_ros`, budgets) it therefore lacks | `FluxExecutor.TheInheritedSpinOnceServicesRosEntities` | `test_spin_once_returns_on_the_first_work_of_either_transport` |
| S-011 | Subscribing a message_filters `Subscriber` late (default construction, `subscribe`, `unsubscribe`) | `MessageFilters.SubscribesLate` | `-` |
| S-012 | `flux.Frame` and the `.bits` views (`Loan.bits`, `Frame.bits`) | `-` | `test_a_device_subscription_hands_back_a_scoped_frame` |
| S-013 | String bytes that are not UTF-8 | `-` | `test_a_string_that_is_not_utf8_reads_as_rclpy_reads_it` |

S-001 comes from dGPU slots being VRAM. Accepting a host array would require an H2D copy, and `flux_core` has no copy primitive, so both sides refuse. What diverges is the form of the refusal. C++ returns `Published` and Python raises.

S-002 comes from core not interpreting the channel key as a ROS topic. `flux_cpp` passes only absolute names, since both pub and sub go through `resolve_topic_name()`, so there is no place to refuse. `flux_py`, which has no node and cannot resolve, refuses with `require_absolute`.

S-003 comes from rclpy not exposing `add_callback_group`. C++ can put flux subscriptions and ROS subscriptions together in that group and place it on one thread. Python has no means to move ROS entities as a group, so DDS inputs stay on the node thread. So the same graph passes in C++ and is refused in Python. The place to run that graph in Python is `flux.ros.Executor`. There, X-022 holds.

S-004 diverges in the hook's unit. Both sides run the hook on the unit's child before its first callback, turn an exception from it into the `spin()` error, and refuse a second hook for one unit and a hook for a unit no child serves. C++ takes a callback group. Python also takes a node, from the same partition as S-003 (flux by group, ROS by node).

S-005 follows ROS, which is not symmetric either. Each side is written the way ROS is written in that language. C++ chains setters on `flux::QoS` as on `rclcpp::QoS`, spells enum values `Cpu` in CamelCase as rclcpp does, and takes the QoS before the callback as `create_subscription(topic, qos, callback)` does. Python takes QoS keywords as `QoSProfile` does, spells enum values `CPU` in upper case as rclpy does, and takes the callback before keywords as `create_subscription(type, topic, callback, qos)` does. Chained setters cannot check two fields at once, so C++ checks `n > depth` where the QoS is used. Python checks it in `flux.QoS(...)`.

S-006 comes from garbage collection. `flux.ros.Executor` adds its nodes to an rclpy executor, and a node can belong to one rclpy executor at a time, so `shutdown()` removes them at a point the caller chooses instead of whenever the object is collected. The name is rclpy's `Executor.shutdown()`. C++ does the same in the destructor, which runs at a known point, as rclcpp's executor does, so it has no `shutdown()`. `PartitionedExecutor` has nothing to release after `spin()` returns and has no `shutdown()` in either language.

S-007 follows each language's convention for a caller's mistake. The C++ publish path is `noexcept` (D-065), so it returns `Published::TooLarge`, and `Published` is `[[nodiscard]]`: a call that drops the result is a compile warning. Python raises `ValueError`, whose message names what was wrong and the path that works. A literal C++ shape longer than 8 is a compile error instead. `Backpressure`, which is not a mistake, is a returned value in both.

S-008 follows each language's ROS executor. rclcpp throws `std::runtime_error` for a node already added to an executor, and rclpy's `add_node` returns quietly for a node it already holds.

S-009 is the same refusal in each language's type. C++ throws `std::invalid_argument`. Python raises `TypeError`, which is what Python raises for a missing callable.

S-010 comes from rclpy. rclpy does not expose the on-new-message callback that C++ hooks into the io_uring, so `flux.ros.Executor` cannot put ROS readiness in the flux ring. A bridge thread waits on the flux side and hands the dispatch to the rclpy spin thread with `create_task`. With no single ring there is no pass a caller could drive by hand, so the direct-loop calls exist only in C++. Python's core `flux.Executor` still has `dispatch` and `wait_for_work` for a flux-only loop.

S-011 follows upstream message_filters, whose C++ `Subscriber` subscribes late and whose Python `Subscriber` does not. The inner subscription is named the upstream way in each language too: `getSubscriber()` in C++, `.sub` in Python.

S-012 comes from numpy. A numpy array cannot hold GPU memory or bf16. So a device frame arrives in Python as a `flux.Frame`, which opens as a GPU array only inside `with`, and a bf16 payload is read and written through `.bits`, an unsigned view of the same bytes (`test_bfloat16_roundtrips_through_bits`). C++ reads both straight from `FrameView` (`device_ptr()`, `data()`), so it has neither. ROS has no counterpart in either language.

S-013 follows each language's ROS. rclcpp keeps a string field as the bytes in a `std::string`, and a flux C++ View returns them unchecked as a `std::string_view` (`test_cpp_hands_over_string_bytes_that_are_not_utf8_unchanged`). rclpy decodes a string field with `errors="replace"`, and flux Python does the same: a byte that is not UTF-8 becomes U+FFFD and the rest of the frame reads.

The C++ `-` in S-002 is not a missing check. The argument of `Channel::create`/`open` is a channel key, not a ROS topic, and core stands without ROS. The refusal lives where `flux::ros::Publisher`/`Subscription` resolve through the node.
