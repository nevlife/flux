#include "flux_bag/frame_encoder.hpp"

#include "flux/schemas.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace flux_bag
{

FrameEncoder::FrameEncoder(std::string flux_topic, std::uint64_t fingerprint)
: flux_topic_(std::move(flux_topic)),
  fingerprint_(fingerprint),
  meta_bytes_(RawMeta::kMaxSize + AuxTail::kMaxSize)
{
}

auto FrameEncoder::encode(
  const flux::FrameView & v, const flux::Subscription & sub, std::int64_t sent_ns,
  FluxFrameFields & f) -> const char *
{
  // A frame with fields in its publisher's arena holds only offsets there, which a bag cannot
  // replay: the bag gets a copy that holds the fields itself. That needs the installed type.
  const void * data = v.data();
  std::size_t size = v.size();
  if (flux::in_arena(v)) {
    // Every .msg on AMENT_PREFIX_PATH, read once by the first channel that needs it.
    static const flux::Schemas schemas;
    if (!schemas.contains(fingerprint_)) {
      return "its type is not installed";
    }
    try {
      schemas.at(fingerprint_).inline_arena(v, inlined_);
    } catch (const flux::Error &) {
      return "it does not match its type";
    }
    data = inlined_.data();
    size = inlined_.size();
  }
  // A publisher restarted with other slots is a new attach: replay recreates it the same way.
  if (sub.attach_generation() != generation_) {
    generation_ = sub.attach_generation();
    meta_.slot_size = sub.channel()->slot_size();
    meta_.slot_count = sub.channel()->slot_count();
    aux_size_ = sub.channel()->aux_size();
  }
  const flux::FrameMeta & m = v.meta();
  meta_.dtype = static_cast<flux::DType>(m.dtype);
  meta_.ndim = m.ndim;
  std::memcpy(meta_.shape.data(), m.shape, sizeof(meta_.shape));
  if (data != v.data()) {
    // The copy is a byte frame of its own size, and replay needs a slot that holds it.
    meta_.dtype = flux::DType::U8;
    meta_.ndim = 1;
    meta_.shape.fill(0);
    meta_.shape[0] = size;
    std::uint64_t slot = 4096;
    while (slot < size) {
      slot *= 2;
    }
    inlined_slot_ = std::max(inlined_slot_, slot);
    meta_.slot_size = std::max<std::uint32_t>(
      meta_.slot_size,
      static_cast<std::uint32_t>(std::min<std::uint64_t>(inlined_slot_, 1u << 31)));
  }

  f.sec = static_cast<std::int32_t>(sent_ns / 1'000'000'000);
  f.nanosec = static_cast<std::uint32_t>(sent_ns % 1'000'000'000);
  f.flux_topic = flux_topic_;
  f.fingerprint = fingerprint_;
  f.codec = "raw";
  f.meta = meta_bytes_.data();
  f.meta_size = meta_.write(meta_bytes_.data());
  f.data = data;
  f.data_size = size;
  if (aux_size_ != 0) {
    const auto aux = v.aux();
    AuxTail tail;
    tail.aux_size = aux_size_;
    tail.aux = aux.data();
    tail.aux_len = static_cast<std::uint32_t>(aux.size());
    f.meta_size += tail.write(meta_bytes_.data() + f.meta_size);
  }
  return nullptr;
}

}  // namespace flux_bag
