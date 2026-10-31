// What play_flux and convert_flux do with a recorded FluxFrame, on bytes a bag file holds. Built as
// a libFuzzer target with -DFLUX_LIBFUZZER (clang, -fsanitize=fuzzer); otherwise a test replays a
// fixed corpus: frames record_flux would write, truncated and with bits flipped. With
// FLUX_FUZZ_SEEDS=<dir> the test also writes the sound frames there, as the fuzzer's seeds.

#include "flux_bag/jpeg_frame.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{

// play_flux decodes into a loaned slot, which holds no more than the channel's slot size.
constexpr std::uint64_t kMaxFrame = std::uint64_t{4} << 20;

// True if the frame reached the JPEG decoder and decoded.
auto replay(const std::uint8_t * data, std::size_t size) -> bool
{
  flux_bag::FluxFrameFields f;
  flux_bag::JpegMeta m;
  if (!flux_bag::read_frame(data, size, f, m) || f.codec != "jpeg") {
    return false;
  }
  const std::uint64_t n = m.raw.nbytes();
  if (n > kMaxFrame) {
    return false;
  }
  std::vector<std::uint8_t> out(n);
  return flux_bag::unpack_jpeg(f, m, out.data());
}

}  // namespace

#ifdef FLUX_LIBFUZZER
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  replay(data, size);
  return 0;
}
#else

#include "sensor_msgs/flux/image.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>

namespace
{

// The bytes record_flux writes for one Image frame of `encoding`.
auto recorded(const char * encoding, std::uint32_t channels) -> std::vector<std::uint8_t>
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 8;
  const std::uint32_t step = kW * channels + 4;
  std::vector<std::uint64_t> storage(512);
  sensor_msgs::flux_msg::Image::Builder b(storage.data(), storage.size() * 8);
  b.set__header__frame_id("cam");
  b.set__height(kH);
  b.set__width(kW);
  b.set__encoding(encoding);
  b.set__step(step);
  auto px = b.alloc__data(std::size_t{step} * kH);
  for (std::size_t i = 0; i < px.size(); ++i) {
    px.data()[i] = static_cast<std::uint8_t>(i * 7);
  }
  const auto * frame = reinterpret_cast<const std::uint8_t *>(storage.data());  // NOLINT
  const std::size_t size = b.size__();

  flux_bag::JpegEncoder enc(90);
  const std::size_t n = enc.encode(px.data(), kW, kH, step, flux_bag::jpeg_space(encoding));
  flux_bag::JpegMeta m;
  m.raw.slot_size = 4096;
  m.raw.slot_count = 4;
  m.raw.ndim = 1;
  m.raw.shape[0] = size;
  m.at = static_cast<std::uint64_t>(px.data() - frame);
  m.len = px.size();
  std::vector<std::uint8_t> data(frame, frame + m.at);
  data.insert(data.end(), frame + m.at + m.len, frame + size);
  data.insert(data.end(), enc.data(), enc.data() + n);
  std::array<std::uint8_t, flux_bag::JpegMeta::kMaxSize> meta{};
  flux_bag::FluxFrameFields f;
  f.frame_id = "cam";
  f.flux_topic = "/cam";
  f.fingerprint = sensor_msgs::flux_msg::Image::kFingerprint;
  f.codec = "jpeg";
  f.meta = meta.data();
  f.meta_size = m.write(meta.data());
  f.data = data.data();
  f.data_size = data.size();
  std::vector<std::uint8_t> out(flux_bag::FluxFrameCdr::size(f));
  flux_bag::FluxFrameCdr::write(f, out.data());
  return out;
}

auto next(std::uint64_t & s) -> std::uint64_t
{
  s ^= s << 13;
  s ^= s >> 7;
  s ^= s << 17;
  return s;
}

TEST(FuzzFrame, RecordedFramesDecodeAndDamagedOnesStayInBounds)
{
  std::uint64_t seed = 0x9E3779B97F4A7C15;
  for (const auto & [encoding, channels] :
       {std::pair{"rgb8", 3U}, std::pair{"bgr8", 3U}, std::pair{"mono8", 1U}}) {
    const std::vector<std::uint8_t> good = recorded(encoding, channels);
    ASSERT_TRUE(replay(good.data(), good.size())) << encoding;
    if (const char * dir = std::getenv("FLUX_FUZZ_SEEDS")) {
      std::ofstream(std::string(dir) + "/" + encoding, std::ios::binary)
        << std::string(good.begin(), good.end());
    }
    for (std::size_t n = 0; n < good.size(); ++n) {
      replay(good.data(), n);
    }
    for (int round = 0; round < 4000; ++round) {
      std::vector<std::uint8_t> bad = good;
      for (int flips = 1 + static_cast<int>(next(seed) % 4); flips > 0; --flips) {
        bad[next(seed) % bad.size()] ^= static_cast<std::uint8_t>(1U << (next(seed) % 8));
      }
      replay(bad.data(), bad.size());
    }
  }
}

}  // namespace

#endif
