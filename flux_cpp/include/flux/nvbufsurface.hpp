#ifndef FLUX_NVBUFSURFACE_HPP
#define FLUX_NVBUFSURFACE_HPP

// Jetson only: needs nvbufsurface.h (Jetson Multimedia API) and libnvbufsurface. flux itself does
// not include or link either.
#include "flux/channel.hpp"

#include <nvbufsurface.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <tuple>
#include <utility>

namespace flux
{

// A frame outside an image slot has no layout: this says what it is, for the copy.
struct ImageFormatHint
{
  ImageEncoding encoding = ImageEncoding::None;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
};

// NvBufSurfaces of one channel's frames for an engine (VIC, NVENC, NVJPG). An image slot's surface
// is imported once and is the slot itself; any other frame is copied into a surface of the
// cache's own, counted in copies(). A slot is known by the file its fd names, not the number: a
// re-attached subscription's slots may take the old ones' numbers. The old surfaces stay until the
// cache goes or clear(). One thread at a time; it must not outlive the channel.
class NvBufSurfaces
{
public:
  NvBufSurfaces() = default;
  NvBufSurfaces(const NvBufSurfaces &) = delete;
  auto operator=(const NvBufSurfaces &) -> NvBufSurfaces & = delete;
  NvBufSurfaces(NvBufSurfaces &&) = delete;
  auto operator=(NvBufSurfaces &&) -> NvBufSurfaces & = delete;
  ~NvBufSurfaces() { clear(); }

  // Lets every surface go, for one that the channel's restart left behind: once
  // `attach_generation()` changed and no engine or caller still uses a surface from before.
  void clear()
  {
    for (auto & [file, s] : imported_) {
      NvBufSurfaceDestroy(s);  // closes its dup of the slot's fd
    }
    imported_.clear();
    for (auto & [key, s] : own_) {
      NvBufSurfaceUnMap(s, 0, -1);
      NvBufSurfaceDestroy(s);
    }
    own_.clear();
  }

  // The slot's surface, or for another frame a copy as `hint` describes it. Null when neither
  // applies or the library refused.
  auto surface(const Frame & v, const ImageFormatHint & hint = {}) -> NvBufSurface *
  {
    if (NvBufSurface * s = slot(v.dmabuf_fd(), v.image_layout())) {
      return s;
    }
    return copy(static_cast<const std::uint8_t *>(v.data()), v.size(), hint);
  }

  // A loan's image slot, for an engine that writes the frame. Null outside an image slot.
  auto surface(const Loan & w) -> NvBufSurface * { return slot(w.dmabuf_fd(), w.image_layout()); }

  [[nodiscard]] auto copies() const noexcept -> std::uint64_t { return copies_; }

  static auto color(ImageEncoding e) noexcept -> NvBufSurfaceColorFormat
  {
    switch (e) {
      case ImageEncoding::Bgra8:
        return NVBUF_COLOR_FORMAT_BGRA;
      case ImageEncoding::Rgba8:
        return NVBUF_COLOR_FORMAT_RGBA;
      case ImageEncoding::Mono8:
        return NVBUF_COLOR_FORMAT_GRAY8;
      case ImageEncoding::Mono16:
        return NVBUF_COLOR_FORMAT_GRAY16_LE;
      case ImageEncoding::Nv12:
        return NVBUF_COLOR_FORMAT_NV12;
      case ImageEncoding::Uyvy:
        return NVBUF_COLOR_FORMAT_UYVY;
      case ImageEncoding::Yuyv:
        return NVBUF_COLOR_FORMAT_YUYV;
      case ImageEncoding::None:
        break;
    }
    return NVBUF_COLOR_FORMAT_INVALID;
  }

private:
  static auto create(ImageEncoding e, std::uint32_t w, std::uint32_t h) -> NvBufSurface *
  {
    NvBufSurfaceCreateParams p{};
    p.width = w;
    p.height = h;
    p.colorFormat = color(e);
    p.layout = NVBUF_LAYOUT_PITCH;
    p.memType = NVBUF_MEM_SURFACE_ARRAY;
    NvBufSurface * s = nullptr;
    if (p.colorFormat == NVBUF_COLOR_FORMAT_INVALID || NvBufSurfaceCreate(&s, 1, &p) != 0) {
      return nullptr;
    }
    s->numFilled = 1;
    return s;
  }

  auto slot(int fd, const std::optional<ImageLayout> & l) -> NvBufSurface *
  {
    if (fd < 0 || !l) {
      return nullptr;
    }
    struct stat st
    {
    };
    if (::fstat(fd, &st) != 0) {
      return nullptr;
    }
    const std::pair<dev_t, ino_t> file{st.st_dev, st.st_ino};
    if (const auto it = imported_.find(file); it != imported_.end()) {
      return it->second;
    }
    // The library's own parameters for the format, then flux's planes over them.
    const auto e = static_cast<ImageEncoding>(l->encoding);
    NvBufSurface * t = create(e, l->width[0], l->height[0]);
    if (t == nullptr) {
      return nullptr;
    }
    NvBufSurfaceMapParams mp{};
    const bool got = NvBufSurfaceGetMapParams(t, 0, &mp) == 0;
    NvBufSurfaceDestroy(t);
    if (!got || mp.num_planes != l->planes) {
      return nullptr;
    }
    for (std::uint32_t i = 0; i < l->planes; ++i) {
      mp.planes[i].pitch = l->pitch[i];
      mp.planes[i].offset = l->offset[i];
      mp.planes[i].psize = l->pitch[i] * l->height[i];
    }
    mp.totalSize = l->bytes;
    mp.extendedMapParams = nullptr;
    const int own = ::dup(fd);
    if (own < 0) {
      return nullptr;
    }
    mp.fd = static_cast<std::uint64_t>(own);
    NvBufSurface * s = nullptr;
    if (NvBufSurfaceImport(&s, &mp) != 0) {
      ::close(own);
      return nullptr;
    }
    s->numFilled = 1;
    imported_.emplace(file, s);
    return s;
  }

  // Rows `hint.step` apart (planes of another row length tight), the planes one after another.
  auto copy(const std::uint8_t * src, std::size_t n, const ImageFormatHint & hint) -> NvBufSurface *
  {
    if (src == nullptr || hint.encoding == ImageEncoding::None || hint.step == 0) {
      return nullptr;
    }
    const auto key = std::make_tuple(hint.encoding, hint.width, hint.height);
    NvBufSurface *& s = own_[key];
    if (s == nullptr) {
      s = create(hint.encoding, hint.width, hint.height);
      if (s == nullptr || NvBufSurfaceMap(s, 0, -1, NVBUF_MAP_READ_WRITE) != 0) {
        own_.erase(key);
        return nullptr;
      }
    }
    const NvBufSurfacePlaneParams & pp = s->surfaceList[0].planeParams;
    std::size_t at = 0;
    const std::size_t row0 = std::size_t{pp.width[0]} * pp.bytesPerPix[0];
    for (std::uint32_t p = 0; p < pp.num_planes; ++p) {
      const std::size_t row = std::size_t{pp.width[p]} * pp.bytesPerPix[p];
      const std::size_t step = row == row0 ? hint.step : row;
      if (row > step || at + step * pp.height[p] > n) {
        return nullptr;
      }
      auto * dst = static_cast<std::uint8_t *>(s->surfaceList[0].mappedAddr.addr[p]);
      for (std::uint32_t y = 0; y < pp.height[p]; ++y) {
        std::memcpy(dst + std::size_t{y} * pp.pitch[p], src + at + std::size_t{y} * step, row);
      }
      at += step * pp.height[p];
    }
    ++copies_;
    return s;
  }

  std::map<std::pair<dev_t, ino_t>, NvBufSurface *> imported_;
  std::map<std::tuple<ImageEncoding, std::uint32_t, std::uint32_t>, NvBufSurface *> own_;
  std::uint64_t copies_ = 0;
};

}  // namespace flux

#endif  // FLUX_NVBUFSURFACE_HPP
