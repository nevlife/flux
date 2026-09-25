#ifndef FLUX_QOS_HPP
#define FLUX_QOS_HPP

#include <cstdint>
#include <stdexcept>
#include <string>

// Consumer QoS. Names follow ROS 2 rmw QoS where flux has the same knob (docs/en/qos.en.md).

namespace flux
{

struct Durability
{
  std::uint32_t replay = 0;  // 0 = volatile, N = transient_local(N)

  static constexpr Durability Volatile() { return Durability{0}; }
  static constexpr Durability TransientLocal(std::uint32_t n) { return Durability{n}; }
  constexpr bool is_volatile() const { return replay == 0; }
};

// Built like rclcpp::QoS. A setter refuses a value wrong on its own; validate() is the one check
// that needs two fields, so it runs where the QoS is used.
class QoS
{
public:
  QoS() = default;
  QoS(std::uint32_t depth) { keep_last(depth); }  // NOLINT(runtime/explicit): QoS(10) as rclcpp

  // Frames this consumer may fall behind the newest.
  QoS & keep_last(std::uint32_t depth)
  {
    if (depth == 0) fail("depth must be >= 1; depth=1 means the newest frame only");
    depth_ = depth;
    return *this;
  }
  QoS & durability_volatile()
  {
    durability_ = Durability::Volatile();
    return *this;
  }
  QoS & transient_local(std::uint32_t n)
  {
    durability_ = Durability::TransientLocal(n);
    return *this;
  }
  // Views held at once; a held view keeps its slot byte-locked.
  QoS & max_borrow(std::uint32_t n)
  {
    if (n == 0)
      fail("max_borrow must be >= 1: a consumer that may hold no view can never take one");
    max_borrow_ = n;
    return *this;
  }

  std::uint32_t depth() const noexcept { return depth_; }
  Durability durability() const noexcept { return durability_; }
  std::uint32_t max_borrow() const noexcept { return max_borrow_; }

  void validate() const
  {
    if (durability_.replay > depth_) {
      fail(
        "durability=transient_local(n) needs n <= depth: replayed frames arrive through the same "
        "lag window, so a consumer cannot receive more of them than it may fall behind");
    }
  }

private:
  [[noreturn]] static void fail(const char * why)
  {
    throw std::invalid_argument(std::string("flux: ") + why);
  }

  std::uint32_t depth_ = 1;
  Durability durability_ = Durability::Volatile();
  std::uint32_t max_borrow_ = 2;
};

}  // namespace flux

#endif  // FLUX_QOS_HPP
