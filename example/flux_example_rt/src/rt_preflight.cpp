#include "flux/rt.hpp"

#include <cstdio>
#include <cstdlib>

// Ask whether this host can hold the schedule before anything is applied. Every verdict comes
// from a kernel-readable source, so this answers on the machine it runs on rather than in
// general.
int main()
{
  flux::rt::Options opts;
  opts.policy = flux::rt::Policy::Fifo;
  opts.priority = 80;
  opts.cpus = {2};

  const int control_priority = 90;
  const flux::rt::Report report = flux::rt::preflight(opts, control_priority);
  std::printf("%s\n", report.to_string().c_str());
  return report.ok() ? EXIT_SUCCESS : EXIT_FAILURE;
}
