# flux_example_executor

What runs a subscription. Five nodes show five arrangements.

| Node | Arrangement | Threads | Mutual exclusion |
| --- | --- | --- | --- |
| `merged_sub` | flux + ROS in one flux executor | 1 | Yes |
| `partitioned_sub` | A child executor per callback group | Number of groups | Within a group |
| `split_sub` | A flux executor and an rclcpp executor separately | 2 | No. Lock it yourself |
| `service_sub` | flux frames + a ROS service in one executor | 1 | Yes |
| `multi_sub` | ROS on `MultiThreadedExecutor`, flux separately | 1 + pool | No. Lock it yourself |

```bash
ros2 run flux_example_executor merged_sub     # ros2 run flux_example_executor merged_sub.py
ros2 run flux_example_executor partitioned_sub   # ros2 run flux_example_executor partitioned_sub.py
ros2 run flux_example_executor split_sub      # ros2 run flux_example_executor split_sub.py
ros2 run flux_example_executor service_sub    # ros2 run flux_example_executor service_sub.py
ros2 run flux_example_executor multi_sub      # C++ only
```

A publisher is needed. `ros2 run flux_example_loan image_pub` fills the flux half of `merged_sub`. The ROS half is filled by a plain ROS 2 publisher that publishes `sensor_msgs/msg/Image` on `image`, and that example package is not in this repository.

## Why they can be split

A flux subscription is not an rclcpp entity. It does not enter a callback group, so there is no contention over the group. This is why `split_sub` and `multi_sub` work.

What is contended is the node. Giving one node to two executors makes C++ throw and Python quietly take it over. So those two give the node only to the ROS side executor and hand the flux executor only the subscription.

## merged_sub, one thread waits for both

The futex of the flux channel and the readiness eventfd of the ROS subscription are armed on one io_uring. The wait is one syscall. flux does not take the ROS subscription. When the eventfd fires it is handed to rclcpp, because rclcpp already has serialized subscriptions and the intra-process branch.

```cpp
flux::ros::Executor ex(4);
ex.add(node->flux_subscription());
ex.add_ros_node(node);
ex.spin();                // ex.stop() ends it
```

`uses_io_uring()` reports which path is in use. Below Linux 6.7 it falls back to a thread per channel.

Registration happens only before spin. The inherited `spin()`, `cancel()`, and `spin_once(timeout)` are overridden for this executor, and the rest that take a duration budget (`spin_some`, `spin_all`) throw.

Python is different. rclpy does not expose rcl's on-new-message callback to Python, so there is no way to arm the ROS side on the same ring. `flux.ros.Executor` has a bridge thread wait on io_uring and hand the dispatch to the rclpy spin thread with `create_task`. Callbacks still run on one thread.

## partitioned_sub, one thread per group

The callback group is the partition unit, and each group gets a child executor and a thread. No group's callback delays another group.

`on_slow` sleeps 80 ms, yet the count of `fast` does not drop. That is the reason this executor exists.

```cpp
flux::ros::PartitionedExecutor ex;
ex.add_ros_node(node);
ex.add(node->fast(), node->fast_group());
ex.add(node->slow(), node->slow_group());
ex.spin();
```

Reentrant groups are refused. There is one thread per group, so that group's callbacks run serially, and a group that declared concurrent execution is not quietly serialized.

## split_sub, two executors

```cpp
flux::ros::Executor fex;
fex.add(node->flux_subscription());   // subscription only. the node is not given

std::thread ros_thread([node]() { rclcpp::spin(node); });
fex.spin();
```

Callbacks run concurrently on two threads. The mutual exclusion `merged_sub` gave for free is gone, so state both touch is locked by hand. The `std::mutex` in this example is that cost.

Python has the same shape. Only `add_flux` on `flux.ros.Executor`, no `add_ros_node`. `rclpy.spin` already owns that node. An executor that received no node runs flux only.

## service_sub, services also wake on arrival

This is what `merged_sub` does not show. The readiness of services, clients, and waitables is armed on the same eventfd as subscriptions. An action server and an action client are one waitable each, so goal, cancel, and result are included here.

The long 3 second tick is the entire claim. If the bridge were not armed, `ros2 service call` would wait several seconds.

```bash
ros2 run flux_example_executor service_sub
ros2 service call /frame_count std_srvs/srv/Trigger
```

The Python side claim is weaker. rclpy does not expose that hook, so the service is run by the wrapped rclpy executor. What is the same is only that the two transports share one thread.

## multi_sub, ROS on MultiThreaded

A combination `PartitionedExecutor` cannot do. PartitionedExecutor claims every callback group of the node it receives, so that node cannot be given to a `MultiThreadedExecutor`.

```cpp
rclcpp::executors::MultiThreadedExecutor mex(rclcpp::ExecutorOptions(), 2);
mex.add_node(node);

flux::ros::Executor fex;
fex.add(node->flux_subscription());
```

The parallelism is the same as PartitionedExecutor, the arrangement differs. `MultiThreadedExecutor` picks a free thread from the pool, PartitionedExecutor pins a thread per group. Only the latter can take an RT priority. Which callback runs on which thread must be fixed before a policy can be applied to that thread.

In Python the partition unit is not one. rclpy has no `add_callback_group`, so there is no way to hand a callback group to a child. The unit is the group for flux subscriptions and the node for ROS callbacks. The group passed to `add_flux` must hold no ROS entity at all. rclpy cannot move it to this thread, so the group's mutual exclusivity quietly breaks.
