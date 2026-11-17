// What play_flux and convert_flux do with a recorded FluxFrame, on bytes a bag file holds. Built as
// a libFuzzer target with -DFLUX_LIBFUZZER (clang, -fsanitize=fuzzer); otherwise a test replays a
// fixed corpus: frames record_flux would write, with and without an aux tail, truncated and with
// bits flipped. With FLUX_FUZZ_SEEDS=<dir> the test also writes the sound frames there, as the
// fuzzer's seeds.

#include "flux_bag/raw_meta.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace
{

// True if the frame parsed.
auto replay(const std::uint8_t * data, std::size_t size) -> bool
{
  flux_bag::FluxFrameFields f;
  flux_bag::RawMeta m;
  flux_bag::AuxTail aux;
  if (!flux_bag::read_frame(data, size, f, m, aux)) {
    return false;
  }
  // As play_flux: the aux bytes go into a slot's aux region of aux_size bytes.
  std::vector<std::uint8_t> slot_aux(aux.aux_size);
  if (aux.aux_len != 0) {
    std::memcpy(slot_aux.data(), aux.aux, aux.aux_len);
  }
  return true;
}

}  // namespace

#ifdef FLUX_LIBFUZZER
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  replay(data, size);
  return 0;
}
#else

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>

namespace
{

// A raw frame of one u8 run, with an aux tail of `aux_len` bytes when `aux_size` is not 0, as
// record_flux writes a GPU image channel's frame. Of `codec` when that is not "raw".
auto recorded_raw(std::uint32_t aux_size, std::uint32_t aux_len, const char * codec = "raw")
  -> std::vector<std::uint8_t>
{
  const std::vector<std::uint8_t> data(96, 7);
  flux_bag::RawMeta m;
  m.slot_size = 4096;
  m.slot_count = 4;
  m.ndim = 1;
  m.shape[0] = data.size();
  std::vector<std::uint8_t> aux(aux_len);
  for (std::size_t i = 0; i < aux.size(); ++i) {
    aux[i] = static_cast<std::uint8_t>(i * 13 + 1);
  }
  flux_bag::AuxTail tail;
  tail.aux_size = aux_size;
  tail.aux = aux.data();
  tail.aux_len = aux_len;
  std::vector<std::uint8_t> meta(flux_bag::RawMeta::kMaxSize + flux_bag::AuxTail::kMaxSize);
  const std::size_t prefix = m.write(meta.data());
  flux_bag::FluxFrameFields f;
  f.flux_topic = "/cam";
  f.fingerprint = 0x780CD7C35A0C29C8ULL;
  f.codec = codec;
  f.meta = meta.data();
  f.meta_size = prefix + tail.write(meta.data() + prefix);
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

// record_flux writes raw frames only; a frame of any other codec is not read as one.
TEST(FuzzFrame, AFrameOfAnotherCodecIsRefused)
{
  const std::vector<std::uint8_t> other = recorded_raw(0, 0, "jpeg");
  EXPECT_FALSE(replay(other.data(), other.size()));
}

TEST(FuzzFrame, RecordedFramesParseAndDamagedOnesStayInBounds)
{
  std::uint64_t seed = 0x9E3779B97F4A7C15;
  std::vector<std::pair<std::string, std::vector<std::uint8_t> > > corpus;
  corpus.emplace_back("raw", recorded_raw(0, 0));
  corpus.emplace_back("raw_aux", recorded_raw(316, 316));
  corpus.emplace_back("raw_aux_half", recorded_raw(64, 32));
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
