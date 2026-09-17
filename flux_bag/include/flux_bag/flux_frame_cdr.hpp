#ifndef FLUX_BAG_FLUX_FRAME_CDR_HPP
#define FLUX_BAG_FLUX_FRAME_CDR_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace flux_bag
{

// The fields of one flux_msgs/msg/FluxFrame, referenced rather than owned, so the frame bytes
// go from the flux slot into the serialized buffer with one copy and no message object.
struct FluxFrameFields
{
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
  std::string_view frame_id;
  std::string_view flux_topic;
  std::uint64_t fingerprint = 0;
  std::string_view codec;
  const void * meta = nullptr;
  std::size_t meta_size = 0;
  const void * data = nullptr;
  std::size_t data_size = 0;
};

// Serializes as rmw does on this platform: DDS-CDR encapsulation followed by XCDR1
// little-endian, primitives aligned to their size relative to the start of the payload.
// A test in this package deserializes the result through rclcpp::Serialization to hold
// that equivalence.
class FluxFrameCdr
{
public:
  static std::size_t size(const FluxFrameFields & f) { return encode(f, nullptr); }

  // `out` must hold size(f) bytes. Returns the bytes written.
  static std::size_t write(const FluxFrameFields & f, std::uint8_t * out) { return encode(f, out); }

  // The inverse: the string and sequence fields of `f` point into `in`, which must outlive
  // them. False when `in` is not a well-formed little-endian FluxFrame.
  static bool read(const std::uint8_t * in, std::size_t size, FluxFrameFields & f)
  {
    if (size < 4 || in[0] != 0x00 || in[1] != 0x01) return false;
    Reader r{in + 4, size - 4};
    return r.scalar(f.sec) && r.scalar(f.nanosec) && r.string(f.frame_id) &&
           r.string(f.flux_topic) && r.scalar(f.fingerprint) && r.string(f.codec) &&
           r.sequence(f.meta, f.meta_size) && r.sequence(f.data, f.data_size);
  }

private:
  static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "CDR bytes are written native-endian");

  struct Cursor
  {
    std::uint8_t * out;
    std::size_t pos = 0;

    void align(std::size_t n)
    {
      const std::size_t padded = (pos + n - 1) / n * n;
      if (out) std::memset(out + pos, 0, padded - pos);
      pos = padded;
    }
    void bytes(const void * src, std::size_t n)
    {
      if (out && n) std::memcpy(out + pos, src, n);
      pos += n;
    }
    template <typename T>
    void scalar(T v)
    {
      align(sizeof(T));
      bytes(&v, sizeof(T));
    }
    void string(std::string_view s)
    {
      scalar(static_cast<std::uint32_t>(s.size() + 1));
      bytes(s.data(), s.size());
      const std::uint8_t nul = 0;
      bytes(&nul, 1);
    }
    void sequence(const void * src, std::size_t n)
    {
      scalar(static_cast<std::uint32_t>(n));
      bytes(src, n);
    }
  };

  struct Reader
  {
    const std::uint8_t * in;
    std::size_t size;
    std::size_t pos = 0;

    bool take(std::size_t n, const std::uint8_t *& at)
    {
      if (size - pos < n) return false;
      at = in + pos;
      pos += n;
      return true;
    }
    template <typename T>
    bool scalar(T & v)
    {
      pos = (pos + sizeof(T) - 1) / sizeof(T) * sizeof(T);
      const std::uint8_t * at;
      if (pos > size || !take(sizeof(T), at)) return false;
      std::memcpy(&v, at, sizeof(T));
      return true;
    }
    bool string(std::string_view & s)
    {
      std::uint32_t n;
      const std::uint8_t * at;
      if (!scalar(n) || n == 0 || !take(n, at) || at[n - 1] != 0) return false;
      s = std::string_view(reinterpret_cast<const char *>(at), n - 1);
      return true;
    }
    bool sequence(const void *& p, std::size_t & n)
    {
      std::uint32_t len;
      const std::uint8_t * at;
      if (!scalar(len) || !take(len, at)) return false;
      p = at;
      n = len;
      return true;
    }
  };

  static std::size_t encode(const FluxFrameFields & f, std::uint8_t * out)
  {
    static constexpr std::uint8_t kEncapsulation[4] = {0x00, 0x01, 0x00, 0x00};
    if (out) std::memcpy(out, kEncapsulation, sizeof(kEncapsulation));
    Cursor c{out ? out + sizeof(kEncapsulation) : nullptr};
    c.scalar(f.sec);
    c.scalar(f.nanosec);
    c.string(f.frame_id);
    c.string(f.flux_topic);
    c.scalar(f.fingerprint);
    c.string(f.codec);
    c.sequence(f.meta, f.meta_size);
    c.sequence(f.data, f.data_size);
    return sizeof(kEncapsulation) + c.pos;
  }
};

}  // namespace flux_bag

#endif  // FLUX_BAG_FLUX_FRAME_CDR_HPP
