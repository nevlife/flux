#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/raw_meta.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <flux_msgs/msg/flux_frame.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{

auto round_trip(const flux_bag::FluxFrameFields & f) -> flux_msgs::msg::FluxFrame
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

auto rmw_bytes(const flux_msgs::msg::FluxFrame & m) -> std::vector<std::uint8_t>
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
  std::array<std::uint8_t, flux_bag::RawMeta::kMaxSize> buf{};
  const std::size_t n = in.write(buf.data());
  EXPECT_EQ(n, 10u + 24u);

  flux_bag::RawMeta out;
  ASSERT_EQ(out.read(buf.data(), n), n);
  EXPECT_EQ(out.slot_size, in.slot_size);
  EXPECT_EQ(out.slot_count, in.slot_count);
  EXPECT_EQ(out.dtype, flux::DType::F32);
  EXPECT_EQ(out.ndim, 3);
  EXPECT_EQ(out.shape[2], 3u);
  // Longer is a tail after it, for AuxTail to judge.
  EXPECT_EQ(out.read(buf.data(), n + 5), n);
  EXPECT_EQ(out.read(buf.data(), n - 1), 0u);
  buf[8] = 200;
  EXPECT_EQ(out.read(buf.data(), n), 0u);
}

namespace
{

// A RawMeta of one dimension, then a tail of `aux_len` bytes for a channel of `aux_size`.
auto meta_with_tail(std::uint32_t aux_size, std::uint32_t aux_len) -> std::vector<std::uint8_t>
{
  flux_bag::RawMeta raw;
  raw.slot_size = 4096;
  raw.slot_count = 2;
  raw.ndim = 1;
  raw.shape[0] = 8;
  std::vector<std::uint8_t> aux(aux_len);
  for (std::size_t i = 0; i < aux.size(); ++i) {
    aux[i] = static_cast<std::uint8_t>(i + 1);
  }
  flux_bag::AuxTail tail;
  tail.aux_size = aux_size;
  tail.aux = aux.data();
  tail.aux_len = aux_len;
  std::vector<std::uint8_t> out(raw.size() + tail.size());
  const std::size_t n = raw.write(out.data());
  EXPECT_EQ(tail.write(out.data() + n), tail.size());
  return out;
}

// The tail after the RawMeta of `meta`, or false.
auto read_tail(const std::vector<std::uint8_t> & meta, flux_bag::AuxTail & tail) -> bool
{
  flux_bag::RawMeta raw;
  const std::size_t used = raw.read(meta.data(), meta.size());
  return used != 0 && tail.read(meta.data() + used, meta.size() - used);
}

void put_u32(std::vector<std::uint8_t> & meta, std::size_t at, std::uint32_t v)
{
  std::memcpy(meta.data() + at, &v, 4);
}

}  // namespace

TEST(AuxTail, RoundTrip)
{
  const std::vector<std::uint8_t> meta = meta_with_tail(316, 300);
  ASSERT_EQ(meta.size(), 18u + 12u + 300u);
  flux_bag::AuxTail tail;
  ASSERT_TRUE(read_tail(meta, tail));
  EXPECT_EQ(tail.aux_size, 316u);
  ASSERT_EQ(tail.aux_len, 300u);
  EXPECT_EQ(tail.aux, meta.data() + 30);
  for (std::size_t i = 0; i < tail.aux_len; ++i) {
    EXPECT_EQ(tail.aux[i], static_cast<std::uint8_t>(i + 1)) << i;
  }

  // A frame that wrote no aux on a channel that has some still says the channel's aux_size.
  ASSERT_TRUE(read_tail(meta_with_tail(FLUX_MAX_AUX_SIZE, 0), tail));
  EXPECT_EQ(tail.aux_size, std::uint32_t{FLUX_MAX_AUX_SIZE});
  EXPECT_EQ(tail.aux_len, 0u);
}

// A meta of exactly the codec's size, as in every bag recorded before the tail, has no aux.
TEST(AuxTail, NoTailIsNoAux)
{
  const std::vector<std::uint8_t> meta = meta_with_tail(0, 0);
  ASSERT_EQ(meta.size(), 18u);
  flux_bag::AuxTail tail;
  tail.aux_size = 9;
  ASSERT_TRUE(read_tail(meta, tail));
  EXPECT_EQ(tail.aux_size, 0u);
  EXPECT_EQ(tail.aux_len, 0u);
  EXPECT_EQ(tail.size(), 0u);
}

TEST(AuxTail, RefusesATailOfTheWrongShape)
{
  const std::vector<std::uint8_t> good = meta_with_tail(32, 16);
  flux_bag::AuxTail tail;
  ASSERT_TRUE(read_tail(good, tail));

  // Cut anywhere inside the tail, or one byte more.
  for (std::size_t n = 19; n < good.size(); ++n) {
    EXPECT_FALSE(read_tail({good.begin(), good.begin() + static_cast<std::ptrdiff_t>(n)}, tail))
      << n;
  }
  std::vector<std::uint8_t> longer = good;
  longer.push_back(0);
  EXPECT_FALSE(read_tail(longer, tail));

  // Version, aux_size and aux_len at 18, 22 and 26.
  const auto with = [&](std::size_t at, std::uint32_t v) {
    std::vector<std::uint8_t> m = good;
    put_u32(m, at, v);
    return m;
  };
  EXPECT_FALSE(read_tail(with(18, 0), tail));
  EXPECT_FALSE(read_tail(with(18, 2), tail));
  EXPECT_FALSE(read_tail(with(22, 0), tail));
  EXPECT_FALSE(read_tail(with(22, 15), tail));  // aux_len 16 past aux_size
  EXPECT_FALSE(read_tail(with(26, 15), tail));  // one byte more than aux_len says
  EXPECT_FALSE(read_tail(with(26, 17), tail));
  std::vector<std::uint8_t> big = meta_with_tail(FLUX_MAX_AUX_SIZE, 0);
  put_u32(big, 22, FLUX_MAX_AUX_SIZE + 1);
  EXPECT_FALSE(read_tail(big, tail));
}
