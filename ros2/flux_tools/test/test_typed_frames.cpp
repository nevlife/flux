#include "flux/channel.hpp"
#include "flux_tools/cloud_from_frame.hpp"
#include "flux_tools/typed_frames.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <string>

namespace
{

using Cloud = sensor_msgs::flux_msg::PointCloud2;

// A GPU memory channel of PointCloud2 (here with its payload in host memory, which the frame does
// not depend on): its aux is a PointCloud2 frame with data empty, its payload the points. The
// rviz2 displays find it by the type's fingerprint and read it as one PointCloud2.
TEST(TypedFrames, ReadAGpuMemoryChannelAsItsType)
{
  const std::string key = "/flux_tools_gpu" + std::to_string(getpid());
  const std::uint64_t gpu = flux::gpu_fingerprint(Cloud::kFingerprint);
  const std::string name = flux::detail::signpost_name(key, gpu);
  auto pub =
    flux::detail::Channel::create(name, 4096, 2, gpu, {}, FLUX_DEFAULT_MAX_SUBSCRIBERS, {}, 4096);
  auto sub = flux::detail::Channel::open(name, gpu);

  flux_tools::TypedFrames frames(Cloud::kFingerprint);
  EXPECT_TRUE(frames.ours(gpu));
  EXPECT_TRUE(frames.ours(Cloud::kFingerprint));
  EXPECT_FALSE(frames.ours(flux::gpu_fingerprint(gpu)));
  // A channel made without a node has its key read back from the signpost name.
  std::string listed;
  for (const flux::Topic & t : flux::enumerate_topics()) {
    if (t.signpost == name) {
      listed = t.key;
    }
  }
  ASSERT_FALSE(listed.empty());
  EXPECT_EQ(frames.fingerprint_for(listed), gpu);

  flux::WriteSlot slot = pub.loan();
  ASSERT_TRUE(slot);
  constexpr std::size_t kPoints = 10;
  constexpr std::size_t kStep = 12;
  for (std::size_t p = 0; p < kPoints; ++p) {
    const std::array<float, 3> xyz = {static_cast<float>(p), 1, 2};
    std::memcpy(static_cast<std::uint8_t *>(slot.data()) + p * kStep, xyz.data(), kStep);
  }
  const auto aux = slot.aux();
  Cloud::Builder b(aux.data(), aux.size());
  b.set__header__frame_id("map");
  b.set__height(1);
  b.set__width(static_cast<std::uint32_t>(kPoints));
  auto f = b.alloc__fields(3);
  for (std::uint32_t i = 0; i < 3; ++i) {
    f[i].set__name(std::string(1, static_cast<char>('x' + i)));
    f[i].set__offset(4 * i);
    f[i].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
    f[i].set__count(1);
  }
  b.set__point_step(kStep);
  ASSERT_TRUE(b.ok__());
  slot.set_aux_len(b.size__());
  ASSERT_EQ(slot.commit(kPoints * kStep), flux::Published::Ok);

  const flux::FrameView frame = sub.take_blocking(1'000'000'000);
  ASSERT_TRUE(frame);
  std::string error;
  const auto msg = flux_tools::cloud_from_frame(frames.view<Cloud::View>(frame), error);
  ASSERT_TRUE(msg) << error;
  EXPECT_EQ(msg->header.frame_id, "map");
  EXPECT_EQ(msg->width, kPoints);
  float x = 0;
  std::memcpy(&x, msg->data.data() + (kPoints - 1) * kStep, sizeof x);
  EXPECT_EQ(x, 9.0F);
}

}  // namespace
