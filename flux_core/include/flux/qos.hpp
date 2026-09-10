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

enum class Reliability : std::uint8_t {
  BestEffort = 0,
  Reliable = 1,  // not implemented (docs/en/qos.en.md 2)
};

struct QoS
{
  std::uint32_t depth = 1;  // frames this consumer may fall behind the newest
  Durability durability = Durability::Volatile();
  std::uint32_t max_borrow = 2;  // views held at once; a held view keeps its slot byte-locked
  Reliability reliability = Reliability::BestEffort;

  // Rejects instead of reinterpreting. Silently redefining a value is how one integer ended up
  // meaning both start position and delivery mode.
  void validate() const
  {
    if (reliability == Reliability::Reliable) {
      fail(
        "reliability=reliable is not implemented; delivery is best-effort (docs/en/qos.en.md 2)");
    }
    if (depth == 0) {
      fail("depth must be >= 1; depth=1 means the newest frame only");
    }
    if (max_borrow == 0) {
      fail("max_borrow must be >= 1: a consumer that may hold no view can never take one");
    }
    if (durability.replay > depth) {
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
};

}  // namespace flux

#endif  // FLUX_QOS_HPP
