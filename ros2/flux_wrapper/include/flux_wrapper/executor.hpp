#ifndef FLUX_WRAPPER_EXECUTOR_HPP
#define FLUX_WRAPPER_EXECUTOR_HPP

#ifdef USE_FLUX_ENABLED
#include <flux/executor.hpp>
#include <flux/partitioned_executor.hpp>
#else
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#endif

namespace flux_wrapper
{

// An rclcpp executor that also serves flux subscriptions. A build with flux uses the flux
// executors on both paths, since they run ROS entities too; spin_some() and spin_once() throw
// there, so use spin().
#ifdef USE_FLUX_ENABLED
using Executor = flux::Executor;
using PartitionedExecutor = flux::PartitionedExecutor;
#else
using Executor = rclcpp::executors::SingleThreadedExecutor;
using PartitionedExecutor = rclcpp::executors::MultiThreadedExecutor;
#endif

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_EXECUTOR_HPP
