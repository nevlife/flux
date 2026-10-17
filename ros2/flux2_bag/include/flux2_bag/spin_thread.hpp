#ifndef FLUX2_BAG_SPIN_THREAD_HPP
#define FLUX2_BAG_SPIN_THREAD_HPP

#include <rclcpp/executor.hpp>

#include <thread>

namespace flux2_bag
{

// Spins an executor on its own thread, and cancels and joins it on the way out, an exception
// included: a std::thread still joinable at destruction ends the process.
class SpinThread
{
public:
  explicit SpinThread(rclcpp::Executor & exec) : exec_(exec), thread_([&exec] { exec.spin(); }) {}
  SpinThread(const SpinThread &) = delete;
  SpinThread & operator=(const SpinThread &) = delete;
  SpinThread(SpinThread &&) = delete;
  SpinThread & operator=(SpinThread &&) = delete;
  ~SpinThread()
  {
    exec_.cancel();
    thread_.join();
  }

private:
  rclcpp::Executor & exec_;
  std::thread thread_;
};

}  // namespace flux2_bag

#endif  // FLUX2_BAG_SPIN_THREAD_HPP
