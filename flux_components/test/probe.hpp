#ifndef FLUX_COMPONENTS_TEST_PROBE_HPP
#define FLUX_COMPONENTS_TEST_PROBE_HPP

#include <sys/types.h>

#include <cstdint>
#include <string>
#include <vector>

// One record for the test binary and the components it loads: both link this library.
namespace probe
{

struct Event
{
  std::string what;
  const void * key;
  pid_t tid;
  std::uint64_t seq;
  double value;
};

void record(const std::string & what, const void * key = nullptr, double value = 0);
std::vector<Event> events(const std::string & what);

}  // namespace probe

#endif  // FLUX_COMPONENTS_TEST_PROBE_HPP
