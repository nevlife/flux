#ifndef FLUX2_TOOLS_WHOLE_PIXEL_ROWS_HPP
#define FLUX2_TOOLS_WHOLE_PIXEL_ROWS_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace flux2_tools
{

// Ogre counts a row pitch in pixels, so rows whose step is not a whole number of pixels are packed
// into `scratch` first. Returns the rows to upload and sets `pitch`, in pixels. The caller has
// checked that `height` rows of `step` bytes are there and that `row_bytes` <= `step`.
inline const std::uint8_t * whole_pixel_rows(
  const std::uint8_t * rows, std::size_t row_bytes, std::size_t height, std::size_t step,
  std::size_t pixel_bytes, std::vector<std::uint8_t> & scratch, std::size_t & pitch)
{
  if (step % pixel_bytes == 0) {
    pitch = step / pixel_bytes;
    return rows;
  }
  scratch.resize(row_bytes * height);
  for (std::size_t r = 0; r < height; ++r) {
    std::memcpy(scratch.data() + r * row_bytes, rows + r * step, row_bytes);
  }
  pitch = row_bytes / pixel_bytes;
  return scratch.data();
}

}  // namespace flux2_tools

#endif  // FLUX2_TOOLS_WHOLE_PIXEL_ROWS_HPP
