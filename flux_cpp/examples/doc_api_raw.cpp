// Compile-only companion to docs/en/raw_api.en.md -- the path a node takes when it has no .msg.
// Every region between [doc:id] and [doc:/id] is the code the fence tagged `doc:id` in that
// document shows, so a renamed or re-signatured API breaks this build instead of silently
// outdating the document. Nothing here is meant to run.
//
// Split from doc_api.cpp because the two documents are separately paired in
// scripts/check_doc_examples.py: api.md carries the .msg path, this file the raw one.

#include "flux/ros/publisher.hpp"
#include "flux/ros/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace flux::doc_raw_examples
{

void render_into(void *, std::size_t)
{
}
void use(const void *, std::size_t)
{
}
void report(const char *)
{
}

template <typename... Ts>
void sink(const Ts &...)
{
}

void doc_raw_publish(
  const rclcpp::Node::SharedPtr & node, std::uint32_t slot_size, std::uint32_t slot_count,
  const void * data)
{
  // [doc:raw_publish]
  flux::ros::Publisher pub(*node, "img", flux::kNoSchema, slot_size, slot_count);

  if (const flux::Published p = pub.publish(data, flux::DType::U8, {480, 640, 3});
      flux::faulted(p)) {
    report(flux::to_string(p));
  }
  // [doc:/raw_publish]
}

void doc_write_slot(flux::ros::Publisher & pub)
{
  // [doc:write_slot]
  flux::WriteSlot w = pub.loan(flux::DType::U8, {480, 640, 3});
  if (w) {
    render_into(w.data(), w.capacity());
    w.commit();
  }
  // [doc:/write_slot]
}

void doc_write_slot_api(flux::WriteSlot & w, std::size_t nbytes)
{
  // [doc:write_slot_api]
  bool held = static_cast<bool>(w);
  void * buffer = w.data();
  std::size_t capacity = w.capacity();
  flux::Published published = w.commit(nbytes);
  w.abort();
  // [doc:/write_slot_api]
  sink(held, buffer, capacity, published);
}

void doc_raw_subscribe(const rclcpp::Node::SharedPtr & node)
{
  // [doc:raw_subscribe]
  flux::ros::Subscription sub(
    *node, "img", flux::kNoSchema,
    [](const flux::FrameView & v) {
      const flux::FrameMeta & m = v.meta();
      if (m.ndim == 3 && m.dtype == flux::DType::U8) {
        use(v.data(), v.size());
      }
    },
    flux::QoS{});
  // [doc:/raw_subscribe]
  sink(sub.attached());
}

void doc_frame_meta(const flux::FrameMeta & m)
{
  // [doc:frame_meta]
  std::uint8_t ndim = m.ndim;
  flux::DType dtype = m.dtype;
  std::uint32_t itemsize = m.itemsize;
  std::uint64_t nbytes = m.nbytes;
  std::uint64_t rows = m.shape[0];
  // [doc:/frame_meta]
  sink(ndim, dtype, itemsize, nbytes, rows);
}

}  // namespace flux::doc_raw_examples
