#ifndef FLUX2_ON_EXIT_HPP
#define FLUX2_ON_EXIT_HPP

#include <utility>

namespace flux2::detail
{

template <class F>
class OnExit
{
public:
  explicit OnExit(F f) : f_(std::move(f)) {}
  ~OnExit() { f_(); }
  OnExit(const OnExit &) = delete;
  OnExit & operator=(const OnExit &) = delete;
  OnExit(OnExit &&) = delete;
  OnExit & operator=(OnExit &&) = delete;

private:
  F f_;
};

}  // namespace flux2::detail

#endif  // FLUX2_ON_EXIT_HPP
