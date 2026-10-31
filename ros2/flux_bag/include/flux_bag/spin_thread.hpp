#ifndef FLUX_BAG_SPIN_THREAD_HPP
#define FLUX_BAG_SPIN_THREAD_HPP

#include <rclcpp/executor.hpp>

#include <thread>

namespace flux_bag
{

// Spins an executor on its own thread, and cancels and joins it on the way out, an exception
// included: a std::thread still joinable at destruction ends the process.
class SpinThread
{
public:
  explicit SpinThread(rclcpp::Executor & exec) : exec_(exec), thread_([&exec] { exec.spin(); }) {}
  SpinThread(const SpinThread &) = delete;
  auto operator=(const SpinThread &) -> SpinThread & = delete;
  SpinThread(SpinThread &&) = delete;
  auto operator=(SpinThread &&) -> SpinThread & = delete;
  ~SpinThread()
  {
    exec_.cancel();
    thread_.join();
  }

private:
  rclcpp::Executor & exec_;
  std::thread thread_;
};

}  // namespace flux_bag

#endif  // FLUX_BAG_SPIN_THREAD_HPP
