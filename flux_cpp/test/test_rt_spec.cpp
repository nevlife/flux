#include "flux/ros/rt_spec.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

// The chain declaration file. Every rule here exists because a node declaring
// its own priority in isolation cannot check it: the ordering spans processes, and so does the
// conflict between two chains that name the same stage.

namespace
{

// A file per test, named by pid so concurrent runs do not share one.
class SpecFile
{
public:
  explicit SpecFile(const std::string & body)
  : path_(
      "/tmp/flux_rt_spec_" + std::to_string(::getpid()) + "_" + std::to_string(++counter_) +
      ".yaml")
  {
    std::ofstream out(path_);
    out << body;
  }
  ~SpecFile() { std::remove(path_.c_str()); }
  const std::string & path() const { return path_; }

private:
  static int counter_;
  std::string path_;
};
int SpecFile::counter_ = 0;

constexpr const char * kGood = R"(
chains:
  perception_to_control:
    target: hard
    stages:
      - node: /camera_node
        group: dds_listener
        external: true
        expect_priority: 60
      - node: /camera_node
        policy: fifo
        priority: 70
        cpus: [2]
      - node: /perception_node
        group: infer
        policy: fifo
        priority: 75
      - node: /control_node
        group: loop
        policy: fifo
        priority: 90
)";

}  // namespace

TEST(RtSpec, ParsesAChainAndDerivesTheControlPriority)
{
  SpecFile f(kGood);
  const auto spec = flux::ros::RtSpec::load(f.path());
  EXPECT_FALSE(spec.empty());
  EXPECT_EQ(spec.stages().size(), 4u);

  const auto * cam = spec.find("/camera_node");
  ASSERT_NE(cam, nullptr);
  EXPECT_EQ(cam->opts.priority, 70);
  EXPECT_EQ(cam->opts.policy, flux::rt::Policy::Fifo);
  EXPECT_EQ(cam->opts.cpus, std::vector<std::uint32_t>{2});
  EXPECT_EQ(cam->strict, flux::rt::Strictness::Hard) << "target: hard must reach every stage";

  // The whole reason the chain lives in one file: no node can derive this from its own line.
  EXPECT_EQ(cam->control_priority, 90) << "upstream stages are checked against the control loop";

  const auto * ctl = spec.find("/control_node", "loop");
  ASSERT_NE(ctl, nullptr);
  EXPECT_EQ(ctl->control_priority, 0) << "the control stage is not checked against itself";

  const auto ext = spec.external();
  ASSERT_EQ(ext.size(), 1u);
  EXPECT_EQ(ext[0]->expect_priority, 60);
  EXPECT_EQ(ext[0]->opts.policy, flux::rt::Policy::Inherit) << "flux never sets an external stage";
}

TEST(RtSpec, RejectsAChainThatDoesNotClimb)
{
  // Perception above control starves the loop it feeds. This is preflight's priority-order rule
  // past two stages, and only the file can see it.
  SpecFile f(R"(
chains:
  inverted:
    target: soft
    stages:
      - node: /camera_node
        policy: fifo
        priority: 70
      - node: /perception_node
        policy: fifo
        priority: 95
      - node: /control_node
        policy: fifo
        priority: 90
)");
  EXPECT_THROW(flux::ros::RtSpec::load(f.path()), std::runtime_error);
}

TEST(RtSpec, NonRtStagesSitOutsideTheOrdering)
{
  // A SCHED_OTHER consumer at the end of a chain is not a priority inversion: it never takes part
  // in RT priority at all. Requiring it to keep climbing would reject a normal logging path.
  SpecFile f(R"(
chains:
  logging:
    target: soft
    stages:
      - node: /camera_node
        policy: fifo
        priority: 70
      - node: /logger_node
        policy: other
)");
  EXPECT_NO_THROW(flux::ros::RtSpec::load(f.path()));
}

TEST(RtSpec, AStageMaySitOnTwoChainsButNotTwoWays)
{
  const char * body = R"(
chains:
  control:
    target: hard
    stages:
      - node: /camera_node
        policy: fifo
        priority: 70
      - node: /control_node
        policy: fifo
        priority: 90
  logging:
    target: soft
    stages:
      - node: /camera_node
        policy: fifo
        priority: %d
      - node: /logger_node
        policy: other
)";
  char same[1024];
  std::snprintf(same, sizeof same, body, 70);
  SpecFile agree(same);
  EXPECT_NO_THROW(flux::ros::RtSpec::load(agree.path()))
    << "one stage feeding two chains is normal";

  char differ[1024];
  std::snprintf(differ, sizeof differ, body, 40);
  SpecFile conflict(differ);
  EXPECT_THROW(flux::ros::RtSpec::load(conflict.path()), std::runtime_error)
    << "two chains asking one thread to run differently is the conflict only a shared file sees";
}

TEST(RtSpec, RejectsWhatItDidNotRead)
{
  // No version field: a version number announces a schema change and says nothing about a typo.
  // Refusing every key we did not read catches both.
  SpecFile typo(R"(
chains:
  c:
    target: soft
    stages:
      - node: /a
        policy: fifo
        priorty: 70
)");
  EXPECT_THROW(flux::ros::RtSpec::load(typo.path()), std::runtime_error);

  SpecFile stray_target(R"(
chains:
  c:
    target: medium
    stages:
      - node: /a
        policy: other
)");
  EXPECT_THROW(flux::ros::RtSpec::load(stray_target.path()), std::runtime_error);

  SpecFile relative(R"(
chains:
  c:
    target: soft
    stages:
      - node: a
        policy: other
)");
  EXPECT_THROW(flux::ros::RtSpec::load(relative.path()), std::runtime_error)
    << "a node name that is not fully qualified names a different node than the running one";
}

TEST(RtSpec, ExternalStagesCarryNoThreadOptions)
{
  // An external stage is a record of somebody else's thread. Options here would read as something
  // flux applies, and it never does.
  SpecFile f(R"(
chains:
  c:
    target: soft
    stages:
      - node: /a
        external: true
        policy: fifo
        priority: 60
)");
  EXPECT_THROW(flux::ros::RtSpec::load(f.path()), std::runtime_error);
}

TEST(RtSpec, LookupByNodeRejectsAnUnknownLabel)
{
  rclcpp::init(0, nullptr);
  SpecFile f(kGood);
  const auto spec = flux::ros::RtSpec::load(f.path());

  auto node = std::make_shared<rclcpp::Node>("perception_node");
  EXPECT_NO_THROW(spec.stage(*node, "infer"));
  EXPECT_EQ(spec.stage(*node, "infer").opts.priority, 75);

  // Silently applying nothing is how a typo survives to runtime.
  EXPECT_THROW(spec.stage(*node, "infr"), std::runtime_error);

  auto stranger = std::make_shared<rclcpp::Node>("not_in_the_spec");
  EXPECT_THROW(spec.stage(*stranger), std::runtime_error);
  rclcpp::shutdown();
}

TEST(RtSpec, NoEnvIsAnEmptySpec)
{
  ::unsetenv("FLUX_RT_SPEC");
  const auto spec = flux::ros::RtSpec::load();
  EXPECT_TRUE(spec.empty()) << "declaring nothing stays the no-op it is today";
}

TEST(RtSpec, HardChainsGiveEachRtThreadItsOwnCore)
{
  // Splitting callbacks into groups is how they stop blocking each other; pinning two of those
  // groups to one core undoes it. SCHED_FIFO has no timeslice, so the first one there holds the
  // core until it blocks. It also breaks nohz_full, which needs at most one runnable task.
  SpecFile hard(R"(
chains:
  c:
    target: hard
    stages:
      - node: /a
        group: x
        policy: fifo
        priority: 70
        cpus: [2]
      - node: /b
        group: y
        policy: fifo
        priority: 80
        cpus: [2]
)");
  EXPECT_THROW(flux::ros::RtSpec::load(hard.path()), std::runtime_error);

  // Different cores is the arrangement the split was for.
  SpecFile apart(R"(
chains:
  c:
    target: hard
    stages:
      - node: /a
        group: x
        policy: fifo
        priority: 70
        cpus: [2]
      - node: /b
        group: y
        policy: fifo
        priority: 80
        cpus: [3]
)");
  EXPECT_NO_THROW(flux::ros::RtSpec::load(apart.path()));
}

TEST(RtSpec, SoftChainsMayShareAnRtCore)
{
  // Two RT threads on one core at different priorities is ordinary Linux, and the file cannot
  // tell whether the core is isolated at all. Strictness draws the line, as it does over
  // preflight's Warn findings.
  SpecFile f(R"(
chains:
  c:
    target: soft
    stages:
      - node: /a
        group: x
        policy: fifo
        priority: 70
        cpus: [2]
      - node: /b
        group: y
        policy: fifo
        priority: 80
        cpus: [2]
)");
  EXPECT_NO_THROW(flux::ros::RtSpec::load(f.path()));
}

TEST(RtSpec, TheCollisionIsFoundAcrossChainsToo)
{
  // Two chains can put two stages on one core without either chain being wrong on its own. Only
  // a file holding both sees it -- the same reason the conflicting-declaration check exists.
  SpecFile f(R"(
chains:
  control:
    target: hard
    stages:
      - node: /a
        group: x
        policy: fifo
        priority: 70
        cpus: [2]
  vision:
    target: hard
    stages:
      - node: /b
        group: y
        policy: fifo
        priority: 80
        cpus: [2]
)");
  EXPECT_THROW(flux::ros::RtSpec::load(f.path()), std::runtime_error);
}

TEST(RtSpec, NonRtStagesDoNotCollide)
{
  // SCHED_OTHER threads sharing a core is what every Linux box does. The rule is about RT
  // threads blocking each other, not about co-tenancy.
  SpecFile f(R"(
chains:
  c:
    target: hard
    stages:
      - node: /a
        group: x
        policy: other
        cpus: [2]
      - node: /b
        group: y
        policy: other
        cpus: [2]
)");
  EXPECT_NO_THROW(flux::ros::RtSpec::load(f.path()));
}
