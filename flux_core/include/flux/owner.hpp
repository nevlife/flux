#ifndef FLUX_OWNER_HPP
#define FLUX_OWNER_HPP

#include <cstdint>
#include <string>
#include <vector>

// Participant liveness via an OFD-locked owner file. Each process
// holds one owner file `/flux.owner.<pid>.<starttime>` write-locked with F_OFD_SETLK for
// its whole lifetime. The kernel releases the lock when the process dies, so a reclaimer
// that try-locks the file and succeeds knows the owner is dead. OFD (not classic F_SETLK)
// so the lock binds to the open file description, immune to the "close any fd drops the
// lock" footgun that a traditional record lock carries.

namespace flux
{

// Identity of a participant, stable across pid reuse. Matches the SlotHolder fields in the
// segment (segment_layout.hpp); the owner file name is derived from it.
struct OwnerId
{
  std::uint32_t pid = 0;        // 0 = none/empty
  std::uint32_t pad = 0;        // keep 8-byte alignment explicit
  std::uint64_t starttime = 0;  // /proc/self/stat field 22 (clock ticks since boot)

  bool valid() const noexcept { return pid != 0; }
  bool operator==(const OwnerId & o) const noexcept
  {
    return pid == o.pid && starttime == o.starttime;
  }
  bool operator!=(const OwnerId & o) const noexcept { return !(*this == o); }
};

// The shm owner-file name for an identity: `/flux.owner.<pid>.<starttime>`.
std::string owner_file_name(const OwnerId & id);

// One endpoint a process announced in its own owner file. Enumeration
// only: crash reclaim reads the OFD lock and the slot owner-ids, never this, so a manifest that
// is missing, stale or half-written costs visibility and nothing else.
struct ManifestEntry
{
  std::string signpost;  // the channel's fixed name; carries domain, key and fingerprint
  std::string key;       // the un-sanitized channel key, which the name cannot give back
  std::string label;     // opaque to core; flux_cpp puts the ROS node's fully qualified name here
  bool publisher = false;
};

// Liveness verdict for a probed owner. Dead also covers a missing
// owner file (torn self-reported starttime, or already cleaned up) -- in every such case no
// live participant holds the file under this identity, so the holder is reclaimable.
enum class Liveness : std::uint8_t {
  Alive,
  Dead,
};

// Process-wide owner file. First use creates `/flux.owner.<pid>.<starttime>` and OFD
// write-locks it; the lock is held until the process exits (clean exit unlinks the file).
// This is the "lock first, mark later" anchor: a participant must hold its owner lock
// before it registers as a slot holder.
class OwnerFile
{
public:
  // The calling process's identity, cached after first read of /proc/self/stat.
  static const OwnerId & self();

  // Create and OFD-write-lock this process's owner file (idempotent, once per process). On
  // return the lock is held on an object the name still resolves to, so probes of this process
  // find it. Throws std::runtime_error if that cannot be reached. Call before the first borrow.
  static void ensure();

  // Reclaimer probe: is the participant identified by `id` still alive? Opens its owner
  // file and try-locks it (F_OFD_SETLK): lock acquired -> Dead (the probe lock is dropped
  // immediately and the stale file is unlinked, since a crashed owner skips its atexit
  // cleanup); lock refused -> Alive; file missing -> Dead. Any other open failure (fd
  // exhaustion) is Alive: it is no evidence of death, and a wrong Dead breaks the byte lock.
  // Never blocks.
  static Liveness probe(const OwnerId & id);

  // Append one endpoint to this process's manifest, calling ensure() first so a manifest can
  // never exist without the lock that says who wrote it. Best-effort and never throws: the
  // caller is opening a channel, and losing a line of metadata must not fail that. Tabs and
  // newlines in `key` or `label` become spaces, so one entry is always one line.
  static void announce(const ManifestEntry & entry) noexcept;

  // Read the manifest out of the owner file `name`, which is another process's in every
  // interesting case. Empty when there is none. A trailing line with no newline is dropped: the
  // writer may have died mid-append. Fields past the fourth are ignored, so the format can grow
  // without this reader changing.
  static std::vector<ManifestEntry> read_manifest(const std::string & name) noexcept;

  OwnerFile(const OwnerFile &) = delete;
  OwnerFile & operator=(const OwnerFile &) = delete;

private:
  OwnerFile() = default;
};

}  // namespace flux

#endif  // FLUX_OWNER_HPP
