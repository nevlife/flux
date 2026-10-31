#ifndef FLUX_ON_EXIT_HPP
#define FLUX_ON_EXIT_HPP

#include <utility>

namespace flux::detail
{

template <class F>
class OnExit
{
public:
  explicit OnExit(F f) : f_(std::move(f)) {}
  ~OnExit() { f_(); }
  OnExit(const OnExit &) = delete;
  auto operator=(const OnExit &) -> OnExit & = delete;
  OnExit(OnExit &&) = delete;
  auto operator=(OnExit &&) -> OnExit & = delete;

private:
  F f_;
};

}  // namespace flux::detail

#endif  // FLUX_ON_EXIT_HPP
