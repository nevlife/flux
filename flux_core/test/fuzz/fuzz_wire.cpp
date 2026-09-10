#include "flux/wire.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

// Every frame flux reads was written by another process, so wire::Reader is a trust boundary:
// truncated frames, bit-flipped descriptors, forged lengths and random blobs must all end in
// bad() or an empty result, never an out-of-bounds read. The hand-written cases in
// test/unit/test_wire.cpp cover the shapes we thought of; this covers the ones we did not.
//
// The contract under test is exactly "no OOB, no abort" -- the sanitizer is the oracle, so this
// asserts nothing itself. Build it with -DFLUX_FUZZER=ON under clang (libFuzzer + ASan/UBSan);
// without that it links a replay main() and colcon runs it over a fixed corpus.

namespace
{

// Consume the first byte as an opcode and the next four as an offset, so the fuzzer can steer
// which accessor runs where instead of only mutating payload bytes.
struct Cursor
{
  const std::uint8_t * p;
  std::size_t n;
  std::size_t i = 0;

  std::uint8_t u8() { return i < n ? p[i++] : 0; }
  std::uint32_t u32()
  {
    std::uint32_t v = 0;
    for (int k = 0; k < 4; ++k) v = (v << 8) | u8();
    return v;
  }
  bool done() const { return i >= n; }
};

template <class T>
void touch(const flux::wire::Span<const T> & s)
{
  T acc{};
  for (std::size_t k = 0; k < s.size(); ++k) acc = s[k];  // force the reads the span promises
  (void)acc;
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t * data, std::size_t size)
{
  if (size < 6) return 0;
  Cursor c{data, size};
  const std::uint8_t plan = c.u8();
  // The frame is the tail; the head steers. A frame of 0 bytes is a legal input too.
  // Copy it to an over-aligned buffer: Reader checks each offset against its type's alignment
  // relative to the frame start, which only means anything if the start itself is aligned. Every
  // real frame is (payload_offset and payload_stride are page multiples), so handing it a
  // misaligned base would report a violation flux cannot produce.
  alignas(64) static std::uint8_t frame[8192];
  const std::size_t frame_size = (size - 1 < sizeof frame) ? size - 1 : sizeof frame;
  std::memcpy(frame, data + 1, frame_size);

  flux::wire::Reader r(frame, frame_size);

  for (int step = 0; step < 16 && !c.done(); ++step) {
    const std::uint8_t op = c.u8();
    const std::size_t off = c.u32();
    switch ((op ^ plan) % 12) {
      case 0:
        (void)r.get<std::uint8_t>(off);
        break;
      case 1:
        (void)r.get<std::uint32_t>(off);
        break;
      case 2:
        (void)r.get<double>(off);
        break;
      case 3:
        touch(r.block<std::uint8_t>(off, c.u32()));
        break;
      case 4:
        touch(r.block<double>(off, c.u32()));
        break;
      case 5:
        (void)r.desc(off);
        break;
      case 6:
        (void)r.len(off);
        break;
      case 7:
        touch(r.span<std::uint8_t>(off));
        break;
      case 8:
        touch(r.span<float>(off));
        break;
      case 9: {
        const std::string_view s = r.str(off);
        std::size_t acc = 0;
        for (char ch : s) acc += static_cast<unsigned char>(ch);
        (void)acc;
        break;
      }
      case 10: {
        const std::string_view s = r.str_at(off, c.u32());
        std::size_t acc = 0;
        for (char ch : s) acc += static_cast<unsigned char>(ch);
        (void)acc;
        break;
      }
      default: {
        const std::size_t stride = 1 + (c.u8() % 64);
        const std::size_t align = std::size_t{1} << (c.u8() % 4);
        const std::size_t e = r.elem(off, c.u32(), stride, align);
        if (!r.bad() && e < frame_size) (void)r.get<std::uint8_t>(e);
        break;
      }
    }
  }
  return 0;
}
