#ifndef FLUX_ROS_PUBLISHER_HPP
#define FLUX_ROS_PUBLISHER_HPP

#include "flux/channel.hpp"
#include "flux/discovery.hpp"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <string>

namespace rclcpp
{
class Node;
}

namespace flux::ros
{

// Publisher side of a flux channel, named after a ROS topic. Creates the shm segment
// and publishes zero-copy frames into it. The owning node keeps using its
// normal rclcpp publishers alongside this -- flux is opt-in per topic.
class Publisher
{
public:
  // Defaults match flux_py so both languages size a channel the same way. slot_size is an upper
  // bound, not an allocation: the payload region is never touched at init, so tmpfs keeps it
  // sparse and idle RAM stays near zero however large it is. slot_count is the ring depth --
  // it bounds concurrent borrows and the retained history (ROS 2: the publisher's History
  // depth), so it caps every consumer's depth and replay.
  static constexpr std::uint32_t kDefaultSlotSize = 16u << 20;  // 16 MiB
  static constexpr std::uint32_t kDefaultSlotCount = 16u;

  // `device` declares what this publisher's frames are produced on.
  // Device::Cuda makes flux create the stream commit() fences on; a caller that already has one
  // passes it to Channel directly instead. Throws when this host cannot serve the declaration --
  // a publisher that believes it is on the GPU must not quietly run off it.
  //
  // `mem` is opt-in page residency for this publisher's mapping. It covers this
  // process only: a subscriber declares its own.
  Publisher(
    rclcpp::Node & node, const std::string & topic, std::uint64_t fingerprint = kNoSchema,
    std::uint32_t slot_size = kDefaultSlotSize, std::uint32_t slot_count = kDefaultSlotCount,
    Device device = Device::Cpu, const MemoryPolicy & mem = {});

  Publisher(const Publisher &) = delete;
  Publisher & operator=(const Publisher &) = delete;

  // Copy nbytes into the next free slot scanning forward from the newest and
  // wake subscribers as one u8 run.
  // Backpressure when every slot is currently borrowed (frame dropped; delivery is best-effort).
  // An adapter-built frame goes out this way: the schema is in the fingerprint, so there is
  // nothing about it left for the caller to describe.
  Published publish(const void * data, std::size_t nbytes) noexcept;

  // Same, for a frame that is not a flat byte run. The byte count follows from the shape and the
  // dtype, so the descriptor a consumer sizes its view from cannot disagree with the payload.
  Published publish(
    const void * data, DType dt, std::initializer_list<std::uint64_t> shape) noexcept;

  // 0-copy publish (docs/en/copy_model.en.md): write the frame straight into WriteSlot::data(),
  // then commit(). The slot is out of the ring until the handle is committed or destroyed. Returns
  // an invalid handle (and counts a drop) when every slot is borrowed.
  WriteSlot loan() noexcept;
  WriteSlot loan(DType dt, std::initializer_list<std::uint64_t> shape) noexcept;

  std::uint64_t dropped() const noexcept;

  // Bytes one slot holds. A generated adapter loans this much and commits the prefix it filled.
  std::uint32_t slot_size() const noexcept { return ch_.slot_size(); }

  // The stream a producing kernel must be launched on -- the one commit() waits for. Undeclared
  // on a host publisher.
  const gpu::Stream & stream() const noexcept { return ch_.stream(); }

  // Commits refused because the declared stream could not be waited on (docs/en/qos.en.md 1). Each
  // one costs a slot that is never reused, so a nonzero value is a fault, not a rate. Always 0 with
  // Device::Cpu.
  std::uint64_t fence_failed() const noexcept { return ch_.fence_failed(); }

  // False when this channel's slots are GPU memory (a discrete GPU). publish() copies from host
  // memory and so always fails there; loan() plus a kernel is the publish path. Asked
  // rather than thrown, because publish() is noexcept for the hard-RT path -- flux_py, which has
  // no such constraint and no header to read, raises instead (docs/en/contracts.en.md 3).
  bool host_addressable() const noexcept { return ch_.host_addressable(); }

  // How long commit blocked waiting on the declared stream. The
  // release fields stay 0 here: a publisher holds no views.
  Channel::FenceWait fence_wait() const noexcept { return ch_.fence_wait(); }

  // What the declared MemoryPolicy got for this mapping. Both false with the default policy; a
  // policy that asked and was refused threw at construction rather than reporting false here.
  bool pages_committed() const noexcept { return ch_.pages_committed(); }
  bool pages_locked() const noexcept { return ch_.pages_locked(); }
  // The fixed rendezvous name derived from the resolved topic + fingerprint -- the signpost,
  // not the segment. The segment behind it carries a per-instance
  // owner-id suffix and changes on every publisher restart.
  const std::string & segment_name() const noexcept { return seg_name_; }

  // This process's domain, the one every name it builds carries.
  // Reported because a domain this node did not expect is invisible otherwise: peers in another
  // domain simply never appear, which reads exactly like a peer that has not started yet.
  const std::string & domain() const noexcept { return flux::process_domain(); }

private:
  std::string seg_name_;
  Channel ch_;
};

}  // namespace flux::ros

#endif  // FLUX_ROS_PUBLISHER_HPP
