#ifndef FLUX_DETAIL_GROUP_MEMBER_HPP
#define FLUX_DETAIL_GROUP_MEMBER_HPP

#include "flux/detail/executor.hpp"
#include "flux/node_ref.hpp"

#include <rclcpp/callback_group.hpp>
#include <rclcpp/waitable.hpp>

#include <rclcpp/version.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace flux::detail
{

// A flux source as a member of a callback group, so that the flux executor serving the group
// finds it (flux::Executor::bridge_group). To rclcpp it is a waitable that is never ready: an
// rclcpp executor serving the group logs once that nothing delivers the source.
class GroupMember final : public rclcpp::Waitable
{
public:
  GroupMember(Source & src, std::int32_t priority, std::string topic)
  : src_(src), priority_(priority), topic_(std::move(topic))
  {
  }

  Source & source() const noexcept { return src_; }
  std::int32_t priority() const noexcept { return priority_; }
  void served() noexcept { served_ = true; }

#if RCLCPP_VERSION_MAJOR >= 28
  void add_to_wait_set(rcl_wait_set_t &) override { refuse_outside_flux(); }
  bool is_ready(const rcl_wait_set_t &) override { return false; }
  void execute(const std::shared_ptr<void> &) override {}
#else
  void add_to_wait_set(rcl_wait_set_t *) override { refuse_outside_flux(); }
  bool is_ready(rcl_wait_set_t *) override { return false; }
  void execute(std::shared_ptr<void> &) override {}
#endif
  std::shared_ptr<void> take_data() override { return nullptr; }
  void set_on_ready_callback(std::function<void(std::size_t, int)>) override
  {
    refuse_outside_flux();
  }
  void clear_on_ready_callback() override {}

  // Set on a thread while a flux executor runs on it. Each copy of flux_cpp has its own, as a
  // component library links one, so served() is what settles it once the member is bridged.
  static bool & in_flux_executor() noexcept;

private:
  void refuse_outside_flux();

  Source & src_;
  std::int32_t priority_;
  std::string topic_;
  std::atomic<bool> served_{false};
  std::atomic<bool> warned_{false};
};

// Adds `src` to `group`, or to the node's default group when it is null.
std::shared_ptr<GroupMember> join_group(
  const NodeRef & node, Source & src, const std::string & topic,
  const rclcpp::CallbackGroup::SharedPtr & group, std::int32_t priority);

}  // namespace flux::detail

#endif  // FLUX_DETAIL_GROUP_MEMBER_HPP
