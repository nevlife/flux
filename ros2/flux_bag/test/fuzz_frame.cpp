// What play_flux and convert_flux do with a recorded FluxFrame, on bytes a bag file holds. Built as
// a libFuzzer target with -DFLUX_LIBFUZZER (clang, -fsanitize=fuzzer); otherwise a test replays a
// fixed corpus: frames record_flux would write, with and without an aux tail, truncated and with
// bits flipped. With FLUX_FUZZ_SEEDS=<dir> the test also writes the sound frames there, as the
// fuzzer's seeds.

#include "flux_bag/jpeg_frame.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace
{

// play_flux decodes into a loaned slot, which holds no more than the channel's slot size.
constexpr std::uint64_t kMaxFrame = std::uint64_t{4} << 20;

// True if the frame parsed and, for codec "jpeg", decoded.
auto replay(const std::uint8_t * data, std::size_t size) -> bool
{
  flux_bag::FluxFrameFields f;
  flux_bag::JpegMeta m;
  flux_bag::AuxTail aux;
  if (!flux_bag::read_frame(data, size, f, m, aux)) {
    return false;
  }
  // As play_flux: the aux bytes go into a slot's aux region of aux_size bytes.
  std::vector<std::uint8_t> slot_aux(aux.aux_size);
  if (aux.aux_len != 0) {
    std::memcpy(slot_aux.data(), aux.aux, aux.aux_len);
  }
  if (f.codec != "jpeg") {
    return true;
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

#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>

namespace
{

// The FluxFrame `f` with `prefix_size` bytes of codec meta in `meta`, then an aux tail of
// `aux_len` bytes when `aux_size` is not 0.
auto serialize(
  flux_bag::FluxFrameFields f, std::uint8_t * meta, std::size_t prefix_size, std::uint32_t aux_size,
  std::uint32_t aux_len) -> std::vector<std::uint8_t>
{
  std::vector<std::uint8_t> aux(aux_len);
  for (std::size_t i = 0; i < aux.size(); ++i) {
    aux[i] = static_cast<std::uint8_t>(i * 13 + 1);
  }
  flux_bag::AuxTail tail;
  tail.aux_size = aux_size;
  tail.aux = aux.data();
  tail.aux_len = aux_len;
  f.meta = meta;
  f.meta_size = prefix_size + tail.write(meta + prefix_size);
  std::vector<std::uint8_t> out(flux_bag::FluxFrameCdr::size(f));
  flux_bag::FluxFrameCdr::write(f, out.data());
  return out;
}

// The bytes record_flux writes for one Image frame of `encoding`.
auto recorded(const char * encoding, std::uint32_t channels, std::uint32_t aux_size = 0)
  -> std::vector<std::uint8_t>
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
  std::vector<std::uint8_t> meta(flux_bag::JpegMeta::kMaxSize + flux_bag::AuxTail::kMaxSize);
  flux_bag::FluxFrameFields f;
  f.frame_id = "cam";
  f.flux_topic = "/cam";
  f.fingerprint = sensor_msgs::flux_msg::Image::kFingerprint;
  f.codec = "jpeg";
  f.data = data.data();
  f.data_size = data.size();
  return serialize(f, meta.data(), m.write(meta.data()), aux_size, aux_size / 2);
}

// A raw frame of one u8 run with an aux tail, as record_flux writes a GPU image channel's frame.
auto recorded_raw(std::uint32_t aux_size, std::uint32_t aux_len) -> std::vector<std::uint8_t>
{
  const std::vector<std::uint8_t> data(96, 7);
  flux_bag::RawMeta m;
  m.slot_size = 4096;
  m.slot_count = 4;
  m.ndim = 1;
  m.shape[0] = data.size();
  std::vector<std::uint8_t> meta(flux_bag::RawMeta::kMaxSize + flux_bag::AuxTail::kMaxSize);
  flux_bag::FluxFrameFields f;
  f.flux_topic = "/cam";
  f.fingerprint = 0x780CD7C35A0C29C8ULL;
  f.codec = "raw";
  f.data = data.data();
  f.data_size = data.size();
  return serialize(f, meta.data(), m.write(meta.data()), aux_size, aux_len);
}

auto next(std::uint64_t & s) -> std::uint64_t
{
  s ^= s << 13;
  s ^= s >> 7;
  s ^= s << 17;
  return s;
}

// The tail comes after at and len, and the jpeg meta reads the same with it as without it.
TEST(FuzzFrame, AJpegFrameCarriesItsAuxTailAfterTheJpegMeta)
{
  const std::vector<std::uint8_t> plain = recorded("rgb8", 3);
  const std::vector<std::uint8_t> tailed = recorded("rgb8", 3, 64);
  flux_bag::FluxFrameFields f;
  flux_bag::JpegMeta without;
  flux_bag::JpegMeta with;
  flux_bag::AuxTail aux;
  ASSERT_TRUE(flux_bag::read_frame(plain.data(), plain.size(), f, without, aux));
  EXPECT_EQ(aux.aux_size, 0u);
  EXPECT_EQ(aux.aux_len, 0u);
  ASSERT_TRUE(flux_bag::read_frame(tailed.data(), tailed.size(), f, with, aux));
  EXPECT_EQ(f.meta_size, with.raw.size() + 16 + flux_bag::AuxTail::kHeader + 32);
  EXPECT_EQ(with.at, without.at);
  EXPECT_EQ(with.len, without.len);
  ASSERT_EQ(aux.aux_size, 64u);
  ASSERT_EQ(aux.aux_len, 32u);
  for (std::size_t i = 0; i < aux.aux_len; ++i) {
    EXPECT_EQ(aux.aux[i], static_cast<std::uint8_t>(i * 13 + 1)) << i;
  }
  EXPECT_TRUE(replay(tailed.data(), tailed.size()));
}

TEST(FuzzFrame, RecordedFramesDecodeAndDamagedOnesStayInBounds)
{
  std::uint64_t seed = 0x9E3779B97F4A7C15;
  std::vector<std::pair<std::string, std::vector<std::uint8_t> > > corpus;
  for (const auto & [encoding, channels] :
       {std::pair{"rgb8", 3U}, std::pair{"bgr8", 3U}, std::pair{"mono8", 1U}}) {
    corpus.emplace_back(encoding, recorded(encoding, channels));
  }
  corpus.emplace_back("rgb8_aux", recorded("rgb8", 3, 64));
  corpus.emplace_back("raw_aux", recorded_raw(316, 316));
  corpus.emplace_back("raw_aux_empty", recorded_raw(FLUX_MAX_AUX_SIZE, 0));
  for (const auto & [name, good] : corpus) {
    ASSERT_TRUE(replay(good.data(), good.size())) << name;
    if (const char * dir = std::getenv("FLUX_FUZZ_SEEDS")) {
      std::ofstream(std::string(dir) + "/" + name, std::ios::binary)
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
