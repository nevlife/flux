#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// Stand-in main() for builds without libFuzzer, so the harnesses stay compiled and runnable under
// the default (gcc) toolchain instead of rotting until someone remembers to build them with clang.
// With paths on the command line it replays those files; with none it runs a built-in corpus of
// the shapes that motivated the harness -- truncation, forged lengths, bit flips, random blobs.
// colcon runs the latter, which is a regression test, not a fuzzing session.

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size);

namespace
{

// Deterministic, so a failure names one seed rather than "sometimes".
std::uint64_t next(std::uint64_t & s)
{
  s ^= s << 13;
  s ^= s >> 7;
  s ^= s << 17;
  return s;
}

void run(const std::vector<std::uint8_t> & buf)
{
  LLVMFuzzerTestOneInput(buf.data(), buf.size());
}

void builtin_corpus()
{
  for (std::size_t n = 0; n <= 512; n += 7) {  // truncation, including the degenerate sizes
    run(std::vector<std::uint8_t>(n, 0));
    run(std::vector<std::uint8_t>(n, 0xFF));  // every descriptor a forged maximum length
  }
  std::uint64_t seed = 0x9E3779B97F4A7C15ull;
  for (int round = 0; round < 4096; ++round) {
    std::vector<std::uint8_t> buf(1 + (next(seed) % 600));
    for (auto & b : buf) b = static_cast<std::uint8_t>(next(seed));
    run(buf);
    for (int flip = 0; flip < 8; ++flip) {  // bit flips on the blob we just ran
      buf[next(seed) % buf.size()] ^= static_cast<std::uint8_t>(1u << (next(seed) % 8));
      run(buf);
    }
  }
}

bool replay(const char * path)
{
  std::FILE * f = std::fopen(path, "rb");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", path);
    return false;
  }
  std::vector<std::uint8_t> buf;
  std::uint8_t chunk[4096];
  std::size_t got;
  while ((got = std::fread(chunk, 1, sizeof chunk, f)) > 0)
    buf.insert(buf.end(), chunk, chunk + got);
  std::fclose(f);
  run(buf);
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc <= 1) {
    builtin_corpus();
    std::printf("corpus ok\n");
    return 0;
  }
  for (int i = 1; i < argc; ++i) {
    if (!replay(argv[i])) return 1;
  }
  std::printf("replayed %d input(s)\n", argc - 1);
  return 0;
}
