#include "flux/spin_control.hpp"

#include <sys/eventfd.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace flux
{

SpinControl::SpinControl()
{
  const int fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error(std::string("flux: eventfd: ") + std::strerror(errno));
  }
  ev_ = std::make_shared<WakeFd>(fd);
}

SpinControl::WakeFd::~WakeFd()
{
  if (fd >= 0) ::close(fd);
}

// Drop ownership rather than close: a foreign hook mid-poke still holds the fd through its weak
// reference, and the close happens when it lets go.
SpinControl::~SpinControl()
{
  ev_.reset();
}

void SpinControl::drain() noexcept
{
  std::uint64_t v;
  while (::read(ev_->fd, &v, sizeof(v)) == static_cast<ssize_t>(sizeof(v))) {
  }
}

void SpinControl::interrupt() noexcept
{
  const std::uint64_t one = 1;
  [[maybe_unused]] ssize_t r = ::write(ev_->fd, &one, sizeof(one));
}

void SpinControl::stop() noexcept
{
  request_stop();
  interrupt();
}

std::function<void()> SpinControl::waker() const
{
  return [w = std::weak_ptr<WakeFd>(ev_)] {
    if (const auto e = w.lock()) {
      const std::uint64_t one = 1;
      [[maybe_unused]] ssize_t r = ::write(e->fd, &one, sizeof(one));
    }
  };
}

SpinControl::Session::Session(SpinControl & ctl) : ctl_(ctl)
{
  if (ctl_.spinning_.exchange(true, std::memory_order_acq_rel)) {
    throw std::logic_error("flux: spin() called while already spinning");
  }
}

SpinControl::Session::~Session()
{
  ctl_.stop_requested_.store(false, std::memory_order_relaxed);
  ctl_.drain();
  ctl_.spinning_.store(false, std::memory_order_release);
}

}  // namespace flux
