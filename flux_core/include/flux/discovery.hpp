#ifndef FLUX_DISCOVERY_HPP
#define FLUX_DISCOVERY_HPP

#include "flux/gpu_platform.hpp"
#include "flux/owner.hpp"
#include "flux/segment.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

// Owns the /dev/shm name namespace: deriving names, telling which object a
// name currently binds to, removing names. Above Segment (a mapping), below Channel (frames).

namespace flux
{

// Longest domain label a name may carry. Bounds the name prefix so a long key still gets room.
inline constexpr std::size_t kMaxDomainLen = 32;

// Domain used when nothing selects one. Canonical under canonical_domain(), and what an unset
// inherited variable resolves to, so a plain core process and a ROS process with no domain set
// land in one domain rather than two.
inline constexpr const char * kDefaultDomain = "0";

// Environment variable process_domain() reads for a host domain id when FLUX_DOMAIN is unset.
// Defined once, here, because every process on a host has to agree on it: a boundary that named
// its own string and a core caller that took the default used to resolve to different domains and
// then fail to meet, with no error on either side.
inline constexpr const char * kDefaultDomainEnv = "ROS_DOMAIN_ID";

// Canonical form of a domain label. Alphanumeric, 1..kMaxDomainLen characters; anything else
// throws std::invalid_argument. Every domain reaches a name through here, so one input cannot
// canonicalize two ways.
std::string canonical_domain(const std::string & raw);

// Resolve a domain from the environment, now:
//   FLUX_DOMAIN set and non-empty  -> canonical_domain(FLUX_DOMAIN), an opaque label
//   else domain_env set and non-empty -> its integer value, rendered canonically so "007" and
//                                           "7" cannot name two domains
//   else                          -> kDefaultDomain
//
// A query, not the answer names are built from -- it reads getenv on every call, so two calls
// straddling a setenv disagree. process_domain() is what a name uses. `domain_env` names the
// variable the host domain id comes from; nullptr opts out and takes FLUX_DOMAIN or
// kDefaultDomain only. Core does not interpret the value it reads -- it renders an integer and
// stops. A value that cannot be canonicalized throws rather than falling back: a typo must not
// silently share the default domain with everyone else.
std::string resolve_domain(const char * domain_env);

// This process's domain. Resolved from kDefaultDomainEnv on first use and fixed for the life of
// the process, so every name this process builds carries the same domain no matter when it is
// built.
//
// Latched rather than read per name because rcl latches the same variable: it reads ROS_DOMAIN_ID
// once at context init and a node created later keeps that domain. Reading it per endpoint let
// one process sit in two domains at once -- measured, by setting the variable between two
// constructions -- while its ROS half stayed in one domain. That is the split core-side
// resolution exists to close, relocated from between layers to inside a process.
//
// A throwing first call is not latched: the initialization is retried, so a corrected environment
// resolves on the next call rather than pinning the failure. fork keeps the value, which is right
// -- the child inherits the environment it was resolved from, and exec replaces the process.
const std::string & process_domain();

// Valid POSIX shm name for a channel key + schema fingerprint: single leading '/', no other '/',
// bounded length. Distinct fingerprints never share a name.
//
// `key` is an opaque channel key, not a ROS topic: core neither resolves nor validates it, and
// two keys that differ at all (`a/b` vs `/a/b`) name two segments. Peers agree on the key above
// core -- flux_cpp resolves it through the node, flux_py has no node and so demands an already
// absolute name (docs/en/contracts.en.md 3).
//
// `domain` is the third compatibility axis, beside the layout version and the fingerprint: two
// domains never share a name, and nothing crosses between them. It is always present in the name,
// never elided when it is the default -- an optional component would give one state two spellings
// and let "unset" and "explicitly default" fail to meet.
std::string segment_name(
  const std::string & key, std::uint64_t fingerprint,
  const std::string & domain = process_domain());

// The key as a name spells it: every non-alnum character mapped to '.', so `/a/b` and `.a.b`
// flatten alike. What a reader with no live participant to ask gets back (TopicView::key_exact),
// so a tool matching a user-typed name against one must compare through this rather than restate
// the rule.
std::string flatten_key(const std::string & key);

// Fixed-name rendezvous object for a channel key. Never changes across
// publisher restarts; points at the current unique-named segment.
std::string signpost_name(
  const std::string & key, std::uint64_t fingerprint,
  const std::string & domain = process_domain());

// Segment name for one publisher-group instance: `<signpost>.<pid>.<starttime>`, never reused
// across restarts. If the result would exceed NAME_MAX the signpost portion is truncated, never
// the owner-id suffix -- a subscriber must be able to rebuild the exact name from the signpost.
std::string unique_segment_name(const std::string & signpost, const OwnerId & creator);

// Identity of the object `name` currently binds to. False if the name does not exist.
bool stat_segment(const std::string & name, SegmentId & out) noexcept;

// One live process's endpoint on a topic, as enumeration reports it.
struct EndpointView
{
  OwnerId owner;
  std::string label;  // what the boundary announced; empty when it announced nothing
  bool publisher = false;
};

// One channel, as enumeration reports it. Built from the /dev/shm name namespace plus the
// manifests of the processes currently holding owner locks -- there is no registry and no daemon,
// so this is a snapshot taken by reading, not a subscription to anything.
struct TopicView
{
  std::string signpost;  // the fixed name; the identity everything else joins on
  std::string domain;    // parsed back out of the name
  std::string key;       // the channel key
  std::uint64_t fingerprint = 0;
  // False when `key` came from the name rather than from a live participant's manifest. The name
  // maps every non-alnum key character to '.', so `/a/b` and `.a.b` are one name: what is
  // reported then is the name's spelling, not the key the peers actually agreed on.
  bool key_exact = false;
  std::vector<EndpointView> endpoints;
};

// What a channel's current segment reports about itself, read without attaching to it.
// No lock and no borrow: every field here is either write-once config
// or a publisher-written atomic, so reading them perturbs nothing and cannot be seen by anyone.
// `publish_seq` is the global ticket dispenser and is monotone, so sampling it twice and taking
// the difference is an exact frame count over that interval rather than an estimate.
struct ChannelStats
{
  bool live = false;  // false when the signpost advertises no readable segment right now
  std::uint32_t slot_size = 0;
  std::uint32_t slot_count = 0;
  std::uint32_t storage_kind = 0;
  std::uint32_t epoch = 0;  // signpost epoch; a change means the publisher group restarted
  std::uint64_t fingerprint = 0;
  std::uint64_t publish_seq = 0;
  std::int32_t waiters = 0;
};

// Read `signpost`'s current segment. `live` is false, and every other field zero, when no
// publisher has one up. Never throws and never writes: a tool may call this in a loop.
ChannelStats read_channel_stats(const std::string & signpost) noexcept;

// Every flux channel this process can see, newest state at the moment of the call. Reads the
// `/dev/shm` names and the manifest of every owner file whose OFD lock is still held; unlocked
// owner files are dead participants and are skipped, not unlinked (that is sweep_dead's job, and
// enumeration must not mutate what it reports). Signposts are persistent, so a channel with no
// endpoints is a name whose participants have all gone, not an error.
//
// Never throws: a directory that cannot be read, or a name that vanishes mid-scan, yields fewer
// rows rather than a failure. Costs one open per name, so it belongs in tooling, not a hot path.
std::vector<TopicView> enumerate_topics() noexcept;

// Unlink crashed orphans (segments and owner files with no live holder), preserving signposts.
// Best-effort, never throws, serialized against concurrent sweeps: nothing with a live holder
// is ever unlinked.
void sweep_dead() noexcept;

// sweep_dead() at most once in this process. A publisher sweeps on every create; this is what
// gives a subscriber-only process the same reclamation.
void sweep_dead_once() noexcept;

// A segment exists but can never serve this participant: wrong magic, layout version,
// fingerprint, or publisher config. Distinct from the transient failures (name absent, creator
// still initializing) so a caller retrying an attach can tell "wait" from "give up and report".
class SegmentMismatch : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// ---- rendezvous ----

// Publisher side: join the segment the signpost `name` advertises, or create a fresh one and
// advertise it. The returned Segment holds this publisher's liveness
// lock for its lifetime. Joiners must match the advertised config. Throws on failure;
// SegmentMismatch when the existing state is permanently incompatible.
//
// `device` says where this publisher's payloads live. On a host that takes the DeviceHandle route
// Device::Cuda makes the payload a device allocation and the shm object control-plane only;
// everywhere else it changes nothing. A joiner adopts whatever the creator chose.
Segment open_publisher_segment(
  const std::string & name, std::uint32_t slot_size, std::uint32_t slot_count,
  std::uint64_t fingerprint, Device device = Device::Cpu);

// Subscriber side: attach to the segment the signpost `name` advertises, validating
// magic/version/fingerprint. Holds no lock. Throws while there is nothing live to attach to
// (transient: retry later). `out_epoch`, if given, receives the signpost epoch attached at, for
// detecting a later rotation.
//
// Throws SegmentMismatch when the advertised segment's payload lives somewhere this subscriber
// cannot read it: a device-backed channel opened with Device::Cpu, or one on another GPU.
// Permanent, not transient -- retrying never fixes either.
Segment open_subscriber_segment(
  const std::string & name, std::uint64_t fingerprint, std::uint32_t * out_epoch = nullptr,
  Device device = Device::Cpu);

// Epoch the signpost currently advertises; 0 when absent or no current segment. Read-only,
// never creates the signpost. Costs an open+map+unmap per call; a
// consumer that polls for rotations should hold a SignpostView instead.
std::uint32_t signpost_epoch(const std::string & signpost) noexcept;

// A held read-only mapping of one signpost. A subscriber keeps this for its lifetime so
// watching for a publisher rotation is a seqlock read of a page it already has, rather than
// reopening the object -- cheap enough to check on every take instead of behind a threshold.
// The signpost is persistent and rotations update it in place, so the
// mapping stays valid across restarts. Move-only.
class SignpostView
{
public:
  SignpostView() = default;
  // Maps `name` if it exists and is bootstrapped; invalid otherwise. Never throws or creates.
  explicit SignpostView(const std::string & name) noexcept;
  ~SignpostView();
  SignpostView(SignpostView && other) noexcept;
  SignpostView & operator=(SignpostView && other) noexcept;
  SignpostView(const SignpostView &) = delete;
  SignpostView & operator=(const SignpostView &) = delete;

  bool valid() const noexcept { return sp_ != nullptr; }

  // Advertised epoch (0 = no current segment). False on no mapping or a torn read, which the
  // caller treats as "look again later" rather than as a change.
  bool epoch(std::uint32_t & out) const noexcept;

private:
  const Signpost * sp_ = nullptr;
  void * base_ = nullptr;
};

// True when no live publisher holds `segment`. The probe never mutates
// the segment.
bool segment_publishers_dead(const std::string & segment) noexcept;

}  // namespace flux

#endif  // FLUX_DISCOVERY_HPP
