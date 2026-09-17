#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/raw_meta.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <flux_msgs/msg/flux_frame.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{

flux_msgs::msg::FluxFrame round_trip(const flux_bag::FluxFrameFields & f)
{
  std::vector<std::uint8_t> bytes(flux_bag::FluxFrameCdr::size(f));
  EXPECT_EQ(flux_bag::FluxFrameCdr::write(f, bytes.data()), bytes.size());

  rclcpp::SerializedMessage serialized(bytes.size());
  std::memcpy(serialized.get_rcl_serialized_message().buffer, bytes.data(), bytes.size());
  serialized.get_rcl_serialized_message().buffer_length = bytes.size();

  flux_msgs::msg::FluxFrame out;
  rclcpp::Serialization<flux_msgs::msg::FluxFrame>().deserialize_message(&serialized, &out);
  return out;
}

std::vector<std::uint8_t> rmw_bytes(const flux_msgs::msg::FluxFrame & m)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<flux_msgs::msg::FluxFrame>().serialize_message(&m, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  return {raw.buffer, raw.buffer + raw.buffer_length};
}

}  // namespace

TEST(FluxFrameCdr, MatchesRmwByteForByte)
{
  const std::string payload = "0123456789abcdef";
  const std::string meta = "wh";
  flux_bag::FluxFrameFields f;
  f.sec = 1700000000;
  f.nanosec = 123456789;
  f.frame_id = "cam";
  f.flux_topic = "/cam/left";
  f.fingerprint = 0x0123456789abcdefULL;
  f.codec = "raw";
  f.meta = meta.data();
  f.meta_size = meta.size();
  f.data = payload.data();
  f.data_size = payload.size();

  flux_msgs::msg::FluxFrame m;
  m.header.stamp.sec = f.sec;
  m.header.stamp.nanosec = f.nanosec;
  m.header.frame_id = "cam";
  m.flux_topic = "/cam/left";
  m.fingerprint = f.fingerprint;
  m.codec = "raw";
  m.meta.assign(meta.begin(), meta.end());
  m.data.assign(payload.begin(), payload.end());

  std::vector<std::uint8_t> ours(flux_bag::FluxFrameCdr::size(f));
  flux_bag::FluxFrameCdr::write(f, ours.data());
  EXPECT_EQ(ours, rmw_bytes(m));
}

TEST(FluxFrameCdr, DeserializesWithEmptyStringsAndSequences)
{
  flux_bag::FluxFrameFields f;
  f.fingerprint = 7;
  const auto out = round_trip(f);
  EXPECT_EQ(out.header.frame_id, "");
  EXPECT_EQ(out.flux_topic, "");
  EXPECT_EQ(out.fingerprint, 7u);
  EXPECT_EQ(out.codec, "");
  EXPECT_TRUE(out.meta.empty());
  EXPECT_TRUE(out.data.empty());
}

TEST(FluxFrameCdr, OddLengthsKeepAlignment)
{
  for (std::size_t topic_len = 0; topic_len < 9; ++topic_len) {
    for (std::size_t data_len = 0; data_len < 9; ++data_len) {
      const std::string topic(topic_len, 't');
      const std::string data(data_len, 'd');
      flux_bag::FluxFrameFields f;
      f.frame_id = "f";
      f.flux_topic = topic;
      f.fingerprint = 0xffffffffffffffffULL;
      f.codec = "h265";
      f.data = data.data();
      f.data_size = data.size();
      const auto out = round_trip(f);
      EXPECT_EQ(out.flux_topic, topic);
      EXPECT_EQ(out.fingerprint, 0xffffffffffffffffULL);
      EXPECT_EQ(out.codec, "h265");
      EXPECT_EQ(std::string(out.data.begin(), out.data.end()), data);
    }
  }
}

TEST(FluxFrameCdr, ReadIsTheInverseOfWrite)
{
  const std::string payload = "payload bytes";
  flux_bag::FluxFrameFields f;
  f.sec = 5;
  f.nanosec = 6;
  f.frame_id = "id";
  f.flux_topic = "/t";
  f.fingerprint = 9;
  f.codec = "raw";
  f.meta = "m";
  f.meta_size = 1;
  f.data = payload.data();
  f.data_size = payload.size();
  std::vector<std::uint8_t> bytes(flux_bag::FluxFrameCdr::size(f));
  flux_bag::FluxFrameCdr::write(f, bytes.data());

  flux_bag::FluxFrameFields back;
  ASSERT_TRUE(flux_bag::FluxFrameCdr::read(bytes.data(), bytes.size(), back));
  EXPECT_EQ(back.sec, 5);
  EXPECT_EQ(back.nanosec, 6u);
  EXPECT_EQ(back.frame_id, "id");
  EXPECT_EQ(back.flux_topic, "/t");
  EXPECT_EQ(back.fingerprint, 9u);
  EXPECT_EQ(back.codec, "raw");
  EXPECT_EQ(std::string(static_cast<const char *>(back.meta), back.meta_size), "m");
  EXPECT_EQ(std::string(static_cast<const char *>(back.data), back.data_size), payload);

  // Truncated anywhere: refused, never read past the end.
  for (std::size_t n = 0; n < bytes.size(); ++n) {
    EXPECT_FALSE(flux_bag::FluxFrameCdr::read(bytes.data(), n, back)) << n;
  }
}

TEST(FluxFrameCdr, ReadRefusesRmwBytesOfTheWrongShape)
{
  flux_msgs::msg::FluxFrame m;
  m.data = {1, 2, 3};
  const auto bytes = rmw_bytes(m);
  flux_bag::FluxFrameFields back;
  ASSERT_TRUE(flux_bag::FluxFrameCdr::read(bytes.data(), bytes.size(), back));
  EXPECT_EQ(back.data_size, 3u);
  std::vector<std::uint8_t> big_endian = bytes;
  big_endian[1] = 0x00;
  EXPECT_FALSE(flux_bag::FluxFrameCdr::read(big_endian.data(), big_endian.size(), back));
}

TEST(RawMeta, RoundTrip)
{
  flux_bag::RawMeta in;
  in.slot_size = 1 << 20;
  in.slot_count = 4;
  in.dtype = flux::DType::F32;
  in.ndim = 3;
  in.shape[0] = 480;
  in.shape[1] = 640;
  in.shape[2] = 3;
  std::uint8_t buf[flux_bag::RawMeta::kMaxSize];
  const std::size_t n = in.write(buf);
  EXPECT_EQ(n, 10u + 24u);

  flux_bag::RawMeta out;
  ASSERT_TRUE(out.read(buf, n));
  EXPECT_EQ(out.slot_size, in.slot_size);
  EXPECT_EQ(out.slot_count, in.slot_count);
  EXPECT_EQ(out.dtype, flux::DType::F32);
  EXPECT_EQ(out.ndim, 3);
  EXPECT_EQ(out.shape[2], 3u);
  EXPECT_FALSE(out.read(buf, n - 1));
  buf[8] = 200;
  EXPECT_FALSE(out.read(buf, n));
}
