#ifndef FLUX_BAG_JPEG_FRAME_HPP
#define FLUX_BAG_JPEG_FRAME_HPP

#include "flux_bag/flux_frame_cdr.hpp"
#include "flux_bag/raw_meta.hpp"

#include "sensor_msgs/flux/image.hpp"

#include <csetjmp>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string_view>
#include <vector>

// jpeglib.h needs FILE and size_t declared first.
#include <jpeglib.h>

namespace flux_bag
{

// What FluxFrame.meta carries for codec "jpeg": the RawMeta of the frame as published, then the
// offset and length of its pixel run (the sensor_msgs/Image `data` field), u64 each. FluxFrame.data
// is the frame without that run, then the JPEG bytes, so every other byte replays as it was.
struct JpegMeta
{
  RawMeta raw;
  std::uint64_t at = 0;
  std::uint64_t len = 0;

  static constexpr std::size_t kMaxSize = RawMeta::kMaxSize + 16;

  std::size_t write(std::uint8_t * out) const
  {
    const std::size_t n = raw.write(out);
    std::memcpy(out + n, &at, 8);
    std::memcpy(out + n + 8, &len, 8);
    return n + 16;
  }

  bool read(const void * in, std::size_t n)
  {
    if (n < 16 || !raw.read(in, n - 16)) return false;
    const auto * p = static_cast<const std::uint8_t *>(in) + n - 16;
    std::memcpy(&at, p, 8);
    std::memcpy(&len, p + 8, 8);
    return at <= raw.nbytes() && len <= raw.nbytes() - at;
  }
};

// A recorded FluxFrame of codec "raw", whose meta.at and meta.len are 0, or "jpeg". The fields of
// `f` point into `in`.
inline bool read_frame(
  const std::uint8_t * in, std::size_t size, FluxFrameFields & f, JpegMeta & meta)
{
  if (!FluxFrameCdr::read(in, size, f)) return false;
  if (f.codec == "raw") {
    meta.at = meta.len = 0;
    return meta.raw.read(f.meta, f.meta_size);
  }
  return f.codec == "jpeg" && meta.read(f.meta, f.meta_size);
}

// The libjpeg color space of an Image encoding JPEG holds without loss of channels, or
// JCS_UNKNOWN.
inline J_COLOR_SPACE jpeg_space(std::string_view encoding)
{
  if (encoding == "rgb8") return JCS_EXT_RGB;
  if (encoding == "bgr8") return JCS_EXT_BGR;
  if (encoding == "mono8") return JCS_GRAYSCALE;
  return JCS_UNKNOWN;
}

inline int jpeg_channels(J_COLOR_SPACE s)
{
  return s == JCS_GRAYSCALE ? 1 : 3;
}

namespace detail
{
// libjpeg's default error handler exits the process.
struct JpegError
{
  jpeg_error_mgr mgr;
  std::jmp_buf jump;

  static void raise(j_common_ptr c)
  {
    std::longjmp(reinterpret_cast<JpegError *>(c->err)->jump, 1);  // NOLINT(*-reinterpret-cast)
  }
};
}  // namespace detail

// One per channel thread: keeps its libjpeg state and output buffer across frames.
class JpegEncoder
{
public:
  explicit JpegEncoder(int quality) : quality_(quality)
  {
    c_.err = jpeg_std_error(&err_.mgr);
    err_.mgr.error_exit = detail::JpegError::raise;
    jpeg_create_compress(&c_);
    dest_.init_destination = [](j_compress_ptr) {};
    dest_.empty_output_buffer = grow;
    dest_.term_destination = [](j_compress_ptr) {};
    c_.dest = &dest_;
    c_.client_data = this;
  }
  JpegEncoder(const JpegEncoder &) = delete;
  JpegEncoder & operator=(const JpegEncoder &) = delete;
  JpegEncoder(JpegEncoder &&) = delete;
  JpegEncoder & operator=(JpegEncoder &&) = delete;
  ~JpegEncoder() { jpeg_destroy_compress(&c_); }

  // Bytes of the JPEG of `height` rows `step` bytes apart, which data() holds until the next call;
  // 0 when libjpeg refuses them.
  std::size_t encode(
    const std::uint8_t * pixels, std::uint32_t width, std::uint32_t height, std::uint32_t step,
    J_COLOR_SPACE space)
  {
    rows_.resize(height);
    for (std::uint32_t y = 0; y < height; ++y) {
      rows_[y] = const_cast<std::uint8_t *>(pixels) + std::size_t{y} * step;  // NOLINT
    }
    if (out_.size() < kFirstBuffer) out_.resize(kFirstBuffer);
    if (setjmp(err_.jump)) {
      jpeg_abort_compress(&c_);
      return 0;
    }
    dest_.next_output_byte = out_.data();
    dest_.free_in_buffer = out_.size();
    c_.image_width = width;
    c_.image_height = height;
    c_.input_components = jpeg_channels(space);
    c_.in_color_space = space;
    jpeg_set_defaults(&c_);
    jpeg_set_quality(&c_, quality_, TRUE);
    jpeg_start_compress(&c_, TRUE);
    while (c_.next_scanline < height) {
      jpeg_write_scanlines(&c_, rows_.data() + c_.next_scanline, height - c_.next_scanline);
    }
    jpeg_finish_compress(&c_);
    return out_.size() - dest_.free_in_buffer;
  }

  const std::uint8_t * data() const noexcept { return out_.data(); }

private:
  // Grown by doubling and kept: after the first frames a channel's JPEGs fit without a resize.
  static constexpr std::size_t kFirstBuffer = 1 << 20;

  static boolean grow(j_compress_ptr c)
  {
    auto & self = *static_cast<JpegEncoder *>(c->client_data);
    const std::size_t used = self.out_.size();
    self.out_.resize(used * 2);
    self.dest_.next_output_byte = self.out_.data() + used;
    self.dest_.free_in_buffer = self.out_.size() - used;
    return TRUE;
  }

  int quality_;
  jpeg_compress_struct c_{};
  detail::JpegError err_{};
  jpeg_destination_mgr dest_{};
  std::vector<std::uint8_t> out_;
  std::vector<JSAMPROW> rows_;
};

// Decodes `jpeg` into `height` rows `step` bytes apart. False when it is not a JPEG of that
// width, height and color space.
inline bool decode_jpeg(
  const void * jpeg, std::size_t size, std::uint8_t * pixels, std::uint32_t width,
  std::uint32_t height, std::uint32_t step, J_COLOR_SPACE space)
{
  jpeg_decompress_struct d{};
  detail::JpegError err{};
  d.err = jpeg_std_error(&err.mgr);
  err.mgr.error_exit = detail::JpegError::raise;
  jpeg_create_decompress(&d);
  if (setjmp(err.jump)) {
    jpeg_destroy_decompress(&d);
    return false;
  }
  jpeg_mem_src(&d, static_cast<const unsigned char *>(jpeg), static_cast<unsigned long>(size));
  jpeg_read_header(&d, TRUE);
  d.out_color_space = space;
  jpeg_start_decompress(&d);
  const bool fits = d.output_width == width && d.output_height == height &&
                    d.output_components == jpeg_channels(space) &&
                    std::size_t{width} * d.output_components <= step;
  const std::size_t used = std::size_t{width} * d.output_components;
  while (fits && d.output_scanline < height) {
    JSAMPROW row = pixels + std::size_t{d.output_scanline} * step;
    jpeg_read_scanlines(&d, &row, 1);
    std::memset(row + used, 0, step - used);
  }
  if (fits) {
    jpeg_finish_decompress(&d);
  } else {
    jpeg_abort_decompress(&d);
  }
  jpeg_destroy_decompress(&d);
  return fits;
}

// Puts every byte of a codec "jpeg" frame but its pixels back in place in `out`, which holds
// m.raw.nbytes(). The JPEG bytes, or an empty view when `f` is shorter than `m` says.
inline std::basic_string_view<std::uint8_t> restore_fields(
  const FluxFrameFields & f, const JpegMeta & m, std::uint8_t * out)
{
  const std::uint64_t rest = m.raw.nbytes() - m.at - m.len;
  if (f.data_size <= m.at + rest) return {};
  const auto * in = static_cast<const std::uint8_t *>(f.data);
  std::memcpy(out, in, m.at);
  std::memcpy(out + m.at + m.len, in + m.at, rest);
  return {in + m.at + rest, f.data_size - m.at - rest};
}

// The frame as it was published, but for the loss JPEG brings to its pixels. False when the
// fields do not describe the pixel run `m` names or the JPEG does not decode into it.
inline bool unpack_jpeg(const FluxFrameFields & f, const JpegMeta & m, std::uint8_t * out)
{
  const auto jpeg = restore_fields(f, m, out);
  if (jpeg.empty()) return false;
  const sensor_msgs::flux_msg::Image::View img(out, m.raw.nbytes());
  const J_COLOR_SPACE space = jpeg_space(img.encoding());
  const auto pixels = img.data();
  if (
    !img.ok__() || space == JCS_UNKNOWN || pixels.data() != out + m.at || pixels.size() != m.len ||
    m.len != std::uint64_t{img.step()} * img.height()) {
    return false;
  }
  return decode_jpeg(
    jpeg.data(), jpeg.size(), out + m.at, img.width(), img.height(), img.step(), space);
}

}  // namespace flux_bag

#endif  // FLUX_BAG_JPEG_FRAME_HPP
