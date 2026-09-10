#ifndef FLUX_MEMORY_HPP
#define FLUX_MEMORY_HPP

// Page residency for a segment mapping. Deliberately separate from QoS and
// rt::Options: QoS says what a consumer receives, rt says how a thread runs, and this says what
// the kernel has already done for the pages that thread will touch. Everything here is opt-in and
// a default-constructed MemoryPolicy is a full no-op.

namespace flux
{

// flux maps a segment whose payload is a sparse tmpfs hole: slot_size is a ceiling, not an
// allocation, so an idle channel with a 16 MiB slot costs almost no RAM. The price is a page
// fault on every page the first time it is touched, and those faults land on the publish and the
// take, not on init. This moves them to attach.
struct MemoryPolicy
{
  // Fault in every page of the mapping at attach: header, slot table, holder tables and payload.
  // All of it or none -- a partial commit sets no bound, and a bound is the entire point.
  bool precommit = false;

  // mlock the mapping, so the kernel may not swap it back out. A commit alone is undone by memory
  // pressure. Needs RLIMIT_MEMLOCK to cover the segment; a refusal is reported, never downgraded.
  // Implies the commit -- mlock populates what it locks.
  bool lock = false;

  bool none() const noexcept { return !precommit && !lock; }
};

}  // namespace flux

#endif  // FLUX_MEMORY_HPP
