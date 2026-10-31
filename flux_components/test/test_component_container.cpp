#include "probe.hpp"

#include <composition_interfaces/srv/list_nodes.hpp>
#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/unload_node.hpp>
#include <flux/component_manager.hpp>
#include <flux/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <gtest/gtest.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace
{

using namespace std::chrono_literals;
using composition_interfaces::srv::ListNodes;
using composition_interfaces::srv::LoadNode;
using composition_interfaces::srv::UnloadNode;

template <class P>
auto eventually(P done, std::chrono::milliseconds limit = 5000ms) -> bool
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (!done()) {
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

auto alive(pid_t tid) -> bool
{
  return std::filesystem::exists("/proc/self/task/" + std::to_string(tid));
}

// Finds the test components in the build tree rather than through the ament index, and records
// both hooks.
class Manager : public flux::ComponentManager
{
public:
  explicit Manager(const std::string & name) : flux::ComponentManager(name) {}

  auto get_component_resources(const std::string & package, const std::string &) const
    -> std::vector<ComponentResource> override
  {
    std::vector<ComponentResource> found;
    if (package != "flux_components_test") {
      return found;
    }
    for (const char * name : {"Plain", "FluxSub", "Late", "Reentrant", "Clocked"}) {
      found.emplace_back(std::string("flux_components_test::") + name, FLUX_TEST_COMPONENTS);
    }
    return found;
  }

protected:
  void on_node_added(const NodeBase::SharedPtr & node) override
  {
    probe::record(std::string("added/") + node->get_name());
  }
  void on_thread_start(
    const NodeBase::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & group) override
  {
    probe::record(std::string("hook/") + node->get_name(), group.get());
  }
};

class Container : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    static int n = 0;
    tag_ = "_" + std::to_string(n++);
    const std::string name = "flux_container_" + std::to_string(getpid()) + tag_;
    manager_ = std::make_shared<Manager>(name);
    spinner_ = std::thread([this] {
      try {
        manager_->spin();
      } catch (const std::exception & e) {
        error_ = e.what();
      }
    });
    client_ = std::make_shared<rclcpp::Node>(name + "_client");
    load_ = client_->create_client<LoadNode>("/" + name + "/_container/load_node");
    unload_ = client_->create_client<UnloadNode>("/" + name + "/_container/unload_node");
    list_ = client_->create_client<ListNodes>("/" + name + "/_container/list_nodes");
    ASSERT_TRUE(load_->wait_for_service(10s));
  }

  void TearDown() override
  {
    manager_->cancel();
    spinner_.join();
    EXPECT_EQ(error_, "");
    manager_.reset();
  }

  template <class Srv>
  auto call(
    const typename rclcpp::Client<Srv>::SharedPtr & client,
    const typename Srv::Request::SharedPtr & request) -> typename Srv::Response::SharedPtr
  {
    auto future = client->async_send_request(request);
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(client_);
    EXPECT_EQ(ex.spin_until_future_complete(future, 10s), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  auto load(
    const std::string & plugin, const std::string & node,
    const std::vector<rclcpp::Parameter> & parameters = {}) -> LoadNode::Response::SharedPtr
  {
    auto request = std::make_shared<LoadNode::Request>();
    request->package_name = "flux_components_test";
    request->plugin_name = "flux_components_test::" + plugin;
    request->node_name = node + tag_;
    for (const auto & p : parameters) {
      request->parameters.push_back(p.to_parameter_msg());
    }
    return call<LoadNode>(load_, request);
  }

  // Records of the node loaded as `node` in this test.
  [[nodiscard]] auto events(const std::string & node, const std::string & label) const
    -> std::vector<probe::Event>
  {
    return probe::events(node + tag_ + "/" + label);
  }

  // The hook ran on the thread of the group that recorded `label`, before its first callback.
  [[nodiscard]] auto hooked_thread(const std::string & node, const std::string & label) const
    -> pid_t
  {
    const std::string what = node + "/" + label;
    const auto first = events(node, label).front();
    for (const auto & hook : probe::events("hook/" + node + tag_)) {
      if (hook.key == first.key) {
        EXPECT_EQ(hook.tid, first.tid) << what;
        EXPECT_LT(hook.seq, first.seq) << what;
        return hook.tid;
      }
    }
    ADD_FAILURE() << "no hook ran for the group of " << what;
    return 0;
  }

  std::string tag_;
  std::shared_ptr<Manager> manager_;
  std::thread spinner_;
  std::string error_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Client<LoadNode>::SharedPtr load_;
  rclcpp::Client<UnloadNode>::SharedPtr unload_;
  rclcpp::Client<ListNodes>::SharedPtr list_;
};

TEST_F(Container, EachGroupOfAPlainComponentRunsOnItsOwnThread)
{
  const auto loaded = load("Plain", "plain");
  ASSERT_TRUE(loaded->success) << loaded->error_message;
  for (const char * label : {"default", "a", "b"}) {
    ASSERT_TRUE(eventually([&] { return !events("plain", label).empty(); })) << label;
  }
  EXPECT_EQ(probe::events("added/plain" + tag_).size(), 1u);
  const std::set<pid_t> tids{
    hooked_thread("plain", "default"), hooked_thread("plain", "a"), hooked_thread("plain", "b")};
  EXPECT_EQ(tids.size(), 3u);
  EXPECT_EQ(tids.count(gettid()), 0u);
  for (const char * label : {"default", "a", "b"}) {
    for (const auto & e : events("plain", label)) {
      EXPECT_EQ(e.tid, events("plain", label)[0].tid);
    }
  }
}

TEST_F(Container, AFluxSubscriptionInAComponentReceivesFrames)
{
  const std::string topic = "/c" + std::to_string(getpid()) + "_frames";
  flux::Publisher pub(*client_, topic, flux::kNoSchema, 4096, 4);
  const auto loaded = load("FluxSub", "flux_sub", {rclcpp::Parameter("topic", topic)});
  ASSERT_TRUE(loaded->success) << loaded->error_message;
  ASSERT_TRUE(eventually([&] {
    const std::uint8_t byte = 1;
    (void)pub.publish(&byte, 1);
    std::this_thread::sleep_for(5ms);
    return events("flux_sub", "frame").size() >= 3;
  }));
  ASSERT_TRUE(eventually([&] { return !events("flux_sub", "default").empty(); }));
  EXPECT_NE(hooked_thread("flux_sub", "frame"), hooked_thread("flux_sub", "default"));
}

TEST_F(Container, UnloadEndsTheThreadsOfItsNodeBeforeItReturns)
{
  const auto loaded = load("Plain", "unloaded");
  ASSERT_TRUE(loaded->success) << loaded->error_message;
  const auto request = std::make_shared<UnloadNode::Request>();
  request->unique_id = loaded->unique_id;
  std::vector<pid_t> tids;
  ASSERT_TRUE(eventually([&] {
    tids.clear();
    for (const auto & hook : probe::events("hook/unloaded" + tag_)) {
      if (alive(hook.tid)) {
        tids.push_back(hook.tid);
      }
    }
    return tids.size() == 3;
  }));
  const std::size_t before = events("unloaded", "a").size();

  ASSERT_TRUE(call<UnloadNode>(unload_, request)->success);
  for (const pid_t tid : tids) {
    EXPECT_FALSE(alive(tid)) << tid;
  }
  const std::size_t after = events("unloaded", "a").size();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(events("unloaded", "a").size(), after);
  EXPECT_GE(after, before);
}

TEST_F(Container, AGroupCreatedAfterTheLoadGetsAThreadAndTheHook)
{
  const auto loaded = load("Late", "late");
  ASSERT_TRUE(loaded->success) << loaded->error_message;
  ASSERT_TRUE(eventually([&] { return !events("late", "late").empty(); }));
  EXPECT_NE(hooked_thread("late", "late"), hooked_thread("late", "default"));
}

TEST_F(Container, AReentrantGroupRefusesTheLoad)
{
  const auto loaded = load("Reentrant", "reentrant");
  EXPECT_FALSE(loaded->success);
  EXPECT_NE(loaded->error_message.find("Reentrant"), std::string::npos) << loaded->error_message;
  const auto listed = call<ListNodes>(list_, std::make_shared<ListNodes::Request>());
  EXPECT_TRUE(listed->full_node_names.empty());
  std::this_thread::sleep_for(50ms);
  EXPECT_TRUE(events("reentrant", "reentrant").empty());
  EXPECT_TRUE(probe::events("hook/reentrant" + tag_).empty());
}

// rclcpp's TimeSource serves /clock on a thread and group of its own, which the container must
// leave to it.
TEST_F(Container, SimTimeReachesAComponent)
{
  const auto loaded = load("Clocked", "clocked", {rclcpp::Parameter("use_sim_time", true)});
  ASSERT_TRUE(loaded->success) << loaded->error_message;
  auto clock = client_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  rosgraph_msgs::msg::Clock now;
  now.clock.sec = 1234;
  EXPECT_TRUE(eventually([&] {
    clock->publish(now);
    std::this_thread::sleep_for(10ms);
    const auto seen = events("clocked", "now");
    return !seen.empty() && seen.back().value == 1234.0;
  }));
}

// The process takes the manager node's name, so containers can be told apart.
TEST(ContainerProcess, IsNamedAfterItsNodeAndEndsOnSigint)
{
  const std::string name = "flux_named_container_" + std::to_string(getpid());
  std::vector<std::string> args{FLUX_CONTAINER,    "--ros-args",  "-r",
                                "__node:=" + name, "--log-level", "warn"};
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & a : args) {
    argv.push_back(a.data());
  }
  argv.push_back(nullptr);
  pid_t pid = 0;
  ASSERT_EQ(posix_spawn(&pid, FLUX_CONTAINER, nullptr, nullptr, argv.data(), environ), 0);
  std::string comm;
  EXPECT_TRUE(eventually([&] {
    std::ifstream in("/proc/" + std::to_string(pid) + "/comm");
    std::getline(in, comm);
    return comm == name.substr(0, 15);
  }))
    << comm;
  kill(pid, SIGINT);
  int status = 0;
  ASSERT_EQ(waitpid(pid, &status, 0), pid);
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}

}  // namespace
