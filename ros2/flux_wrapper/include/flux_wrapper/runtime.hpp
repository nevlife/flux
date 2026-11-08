#ifndef FLUX_WRAPPER_RUNTIME_HPP
#define FLUX_WRAPPER_RUNTIME_HPP

#include <cstdlib>
#include <string_view>

namespace flux_wrapper
{

// True when ENABLE_FLUX is "1" in the environment, read once per process.
inline auto flux_requested() -> bool
{
  static const bool requested = [] {
    const char * v = std::getenv("ENABLE_FLUX");
    return v != nullptr && std::string_view(v) == "1";
  }();
  return requested;
}

// Per process: a flux subscriber gets nothing from a ROS-path publisher, so a launch sets
// ENABLE_FLUX for every node. Always false when this package was built without flux.
inline auto use_flux() -> bool
{
#ifdef USE_FLUX_ENABLED
  return flux_requested();
#else
  return false;
#endif
}

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_RUNTIME_HPP
