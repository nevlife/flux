// Compile-only companion to docs/en/api.en.md. Every region between [doc:id] and [doc:/id] is the
// code the fence tagged `doc:id` in that document shows, so a renamed or re-signatured API
// breaks this build instead of silently outdating the document. Nothing here is meant to run:
// the functions are never called and take everything they need as parameters.
//
// scripts/check_doc_examples.py compares the two sides, ignoring comments so the document can
// keep its Korean ones. Change the code here first, then the fence.

#include "flux/ros/executor.hpp"
#include "flux/ros/partitioned_executor.hpp"
#include "flux/ros/publisher.hpp"
#include "flux/ros/subscription.hpp"
#include "flux/rt.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

// External linkage on purpose: an anonymous namespace would make every function here an
// unused-function warning, since nothing calls them.
namespace flux::doc_examples
{

void render_into(void *, std::size_t)
{
}
void render_into(void *, std::size_t, flux::gpu::Stream)
{
}
void use(const void *, std::size_t)
{
}
void handle(const flux::FrameView &)
{
}
void use(const void *, std::size_t, flux::gpu::Stream)
{
}
void log(const std::string &, const std::string &)
{
}

template <typename... Ts>
void sink(const Ts &...)
{
}

void doc_publisher(
  const rclcpp::Node::SharedPtr & node, std::uint32_t slot_size, std::uint32_t slot_count,
  std::uint64_t fingerprint)
{
  // [doc:publisher]
  flux::ros::Publisher pub(*node, "img", fingerprint, slot_size, slot_count);

  pub.loan();
  pub.dropped();
  pub.slot_size();
  pub.segment_name();
  // [doc:/publisher]
}

void doc_gpu_publisher(
  const rclcpp::Node::SharedPtr & node, std::uint32_t slot_size, std::uint32_t slot_count,
  std::uint64_t fingerprint)
{
  // [doc:gpu_publisher]
  flux::ros::Publisher pub(*node, "img", fingerprint, slot_size, slot_count, flux::Device::Cuda);

  flux::WriteSlot w = pub.loan(flux::DType::U8, {480, 640, 3});
  if (w) {
    render_into(w.device_ptr(), w.capacity(), w.stream());
    w.commit();
  }
  pub.fence_failed();
  pub.fence_wait();
  // [doc:/gpu_publisher]
}

void doc_gpu_subscription(const rclcpp::Node::SharedPtr & node, std::uint64_t fingerprint)
{
  // [doc:gpu_subscription]
  flux::ros::Subscription sub(
    *node, "img", fingerprint,
    [](const flux::FrameView & v) { use(v.device_ptr(), v.size(), v.stream()); }, flux::QoS{},
    flux::Device::Cuda);

  sub.fence_failed();
  sub.fence_wait();
  // [doc:/gpu_subscription]
}

void doc_subscription(const rclcpp::Node::SharedPtr & node, std::uint64_t fingerprint)
{
  // [doc:subscription]
  flux::ros::Subscription sub(
    *node, "img", fingerprint, [](const flux::FrameView & v) { handle(v); }, flux::QoS{});
  // [doc:/subscription]
  sink(sub.attached());
}

void doc_subscription_api(flux::ros::Subscription & sub)
{
  // [doc:subscription_api]
  bool attached = sub.attached();
  bool driven = sub.has_callback();
  const flux::QoS & qos = sub.qos();
  std::uint64_t lost = sub.lost();
  bool can_borrow = sub.can_borrow();
  flux::Channel::Refused refused = sub.refused();
  const std::string & domain = sub.domain();
  // [doc:/subscription_api]
  sink(attached, driven, qos.depth, lost, can_borrow, refused.total(), domain);
}

void doc_subscription_pull(const rclcpp::Node::SharedPtr & node, std::uint64_t fingerprint)
{
  // [doc:subscription_pull]
  flux::ros::Subscription sub(*node, "img", fingerprint);  // no callback: pull with peek/take below

  flux::FrameView newest = sub.peek();              // newest frame; does not consume
  flux::FrameView next = sub.take();                // next frame in order; invalid when caught up
  flux::FrameView blocked = sub.take_blocking(-1);  // sleeps on the futex until one arrives
  // [doc:/subscription_pull]
  sink(static_cast<bool>(newest), static_cast<bool>(next), static_cast<bool>(blocked));
}

void doc_frame_view_api(flux::FrameView & v)
{
  // [doc:frame_view_api]
  bool valid = static_cast<bool>(v);
  const void * data = v.data();
  std::size_t size = v.size();
  v.release();
  // [doc:/frame_view_api]
  sink(valid, data, size);
}

void doc_memory_policy(const rclcpp::Node::SharedPtr & node, std::uint64_t fingerprint)
{
  const std::uint32_t slot_size = 4096;
  const std::uint32_t slot_count = 8;
  // [doc:memory_policy]
  flux::MemoryPolicy mem;
  mem.precommit = true;  // fault every page in at attach
  mem.lock = true;       // and mlock; RLIMIT_MEMLOCK must cover the segment

  flux::ros::Publisher pub(
    *node, "img", fingerprint, slot_size, slot_count, flux::Device::Cpu, mem);
  flux::ros::Subscription sub(*node, "img", fingerprint, {}, flux::QoS{}, flux::Device::Cpu, mem);

  bool committed = pub.pages_committed();
  bool locked = pub.pages_locked();
  // [doc:/memory_policy]
  sink(committed, locked, sub.lost(), 0);
}

void doc_executor(
  const rclcpp::Node::SharedPtr & node, flux::ros::Subscription & flux_sub,
  flux::ros::Subscription & control_sub)
{
  // [doc:executor]
  flux::ros::Executor ex(32);
  ex.add(flux_sub);
  ex.add(control_sub, 10);  // priority: visited first within a pass. default 0
  ex.add_ros_node(node);
  ex.spin();
  ex.stop();
  // [doc:/executor]

  // [doc:executor_api]
  bool merged = ex.uses_io_uring();
  std::size_t channels = ex.size();
  ex.interrupt();
  int frames = ex.dispatch();
  ex.wait_for_work(1'000'000);
  int ros_ran = ex.pump_ros();
  ex.set_ros_budget(16);
  int budget = ex.ros_budget();
  ex.set_pass_budget(16);
  int flux_budget = ex.pass_budget();
  bool more = ex.has_more();
  bool ros_woke = ex.take_ros_ready();
  // [doc:/executor_api]
  sink(merged, channels, frames, ros_ran + budget + flux_budget + (ros_woke || more ? 1 : 0));
}

void doc_partitioned(
  const rclcpp::Node::SharedPtr & node, flux::ros::Subscription & flux_sub,
  const rclcpp::CallbackGroup::SharedPtr & group)
{
  // [doc:partitioned]
  flux::ros::PartitionedExecutor ex;
  ex.add(flux_sub, group);
  ex.add_ros_node(node);
  ex.schedule(group, {flux::rt::Policy::Fifo, 90, {3}});
  ex.spin();
  ex.stop();
  ex.interrupt();
  // [doc:/partitioned]
}

void doc_rt()
{
  // [doc:rt]
  flux::rt::Options o;
  o.policy = flux::rt::Policy::Fifo;
  o.priority = 80;
  o.cpus = {3};

  flux::rt::Report rep = flux::rt::preflight(o, /*control_priority=*/90);
  flux::rt::apply(o);
  flux::rt::ThreadState st = flux::rt::current();
  flux::rt::ThreadState other = flux::rt::observe(flux::rt::this_tid());

  rep = flux::rt::apply_checked(o, flux::rt::Strictness::Hard, /*control_priority=*/90);
  // [doc:/rt]
  sink(rep.ok(), st.policy, other.priority);
}

void doc_rt_spec(
  const rclcpp::Node::SharedPtr & node, flux::ros::PartitionedExecutor & ex,
  const rclcpp::CallbackGroup::SharedPtr & group, int foreign_tid)
{
  // [doc:rt_spec]
  flux::ros::RtSpec spec = flux::ros::RtSpec::load();  // FLUX_RT_SPEC; empty spec when unset
  if (!spec.empty()) {
    const flux::ros::RtStage & st = spec.stage(*node, "infer");
    ex.schedule(group, st);  // target and control_priority ride along with the chain
    log(st.chain, st.node);  // which stage of which chain

    std::thread worker([&spec, &node] {  // a thread flux does not own applies its own stage
      flux::ros::apply_checked(spec.stage(*node, "infer_worker"));
      flux::ros::verify(spec.stage(*node, "infer_worker"), flux::rt::this_tid());
    });
    worker.join();
  }
  for (const flux::ros::RtStage * e : spec.external()) {  // the stages flux does not set
    log(e->node, flux::ros::verify(*e, foreign_tid).to_string());
  }
  // [doc:/rt_spec]
  sink(
    spec.stages().size(), spec.find("/control_node", "loop"), spec.stages()[0].group,
    spec.stages()[0].external, spec.stages()[0].opts.priority, spec.stages()[0].strict,
    spec.stages()[0].control_priority);
}

void doc_rt_report(const flux::rt::Report & rep)
{
  // [doc:rt_report]
  for (const flux::rt::Finding & f : rep.findings) {
    if (f.verdict == flux::rt::Verdict::Fail) {
      log(f.id, f.detail);
    }
  }
  const flux::rt::Finding * one = rep.find("cpu-online");
  // [doc:/rt_report]
  sink(one, flux::rt::Verdict::Ok, flux::rt::Verdict::Warn, flux::rt::Verdict::Unknown);
}

}  // namespace flux::doc_examples
