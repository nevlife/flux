#ifndef FLUX_WIRE_HPP
#define FLUX_WIRE_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>

// Runtime for flux_gen adapters: read and write the flat frame layout flux_gen computes
// (flux_gen/layout.py). Header-only and ROS-free, so a generated header pulls in nothing but
// this. Hand-written code can use it too; the offsets are the generated constants.
//
// A frame is [ scalar block | var region ]. Fixed values sit inline in the scalar block;
// everything variable is an 8-byte descriptor {offset, length} pointing into the var region,
// offsets measured from the frame start so the blob stays pointer-free.
//
// A frame is written by another process, so Reader trusts none of it: every descriptor is
// checked against the frame size and against the alignment its type needs. A violation latches
// bad() and turns further reads into empty results, rather than handing out a pointer that runs
// off the end.
//
// The one thing Reader does assume is its own base pointer: offsets are checked for alignment
// relative to the frame start, so the start must itself be aligned for any type in the schema --
// 8 bytes, the widest scalar a flux frame can hold. Every flux frame clears that: payload_offset
// and payload_stride are page multiples (segment_layout.hpp) over a base that is page-aligned on
// a shm segment (mmap) and kCacheLine-aligned on a heap one (Segment::create_heap). Note the
// frame START is only guaranteed to 64 bytes, not to a page -- the page multiples are what the
// subscriber's mprotect boundary needs, and are a property of the offsets, not of every base. A
// caller passing anything else is outside the contract, not a peer flux defends against.

namespace flux::wire
{

struct Desc
{
  std::uint32_t off;
  std::uint32_t len;
};

static_assert(sizeof(Desc) == 8);

constexpr std::size_t kMaxFrameBytes = 0xFFFFFFFFu;  // descriptors are uint32

template <class T>
class Span
{
public:
  Span() = default;
  Span(T * p, std::size_t n) noexcept : p_(p), n_(n) {}

  T * data() const noexcept { return p_; }
  std::size_t size() const noexcept { return n_; }
  bool empty() const noexcept { return n_ == 0; }
  T * begin() const noexcept { return p_; }
  T * end() const noexcept { return p_ + n_; }
  T & operator[](std::size_t i) const noexcept { return p_[i]; }

private:
  T * p_ = nullptr;
  std::size_t n_ = 0;
};

namespace detail
{

// off + bytes <= size, without overflowing.
inline bool fits(std::size_t off, std::size_t bytes, std::size_t size) noexcept
{
  return off <= size && bytes <= size - off;
}

inline std::size_t align_up(std::size_t n, std::size_t a) noexcept
{
  return (n + a - 1) / a * a;
}

}  // namespace detail

// Read side of a received frame. Construct over FrameView::data()/size().
class Reader
{
public:
  Reader() = default;
  Reader(const void * base, std::size_t size) noexcept
  : base_(static_cast<const std::byte *>(base)), size_(size), bad_(base == nullptr)
  {
  }

  bool bad() const noexcept { return bad_; }
  explicit operator bool() const noexcept { return !bad_; }
  std::size_t size() const noexcept { return size_; }
  const std::byte * base() const noexcept { return base_; }

  template <class T>
  T get(std::size_t off) const noexcept
  {
    static_assert(std::is_trivially_copyable_v<T>);
    T v{};
    if (bad_ || !detail::fits(off, sizeof(T), size_)) {
      bad_ = true;
      return v;
    }
    std::memcpy(&v, base_ + off, sizeof(T));  // offsets are aligned by construction; be exact
    return v;
  }

  // A fixed T[N] block, inline in the scalar block rather than behind a descriptor.
  template <class T>
  Span<const T> block(std::size_t off, std::size_t n) const noexcept
  {
    if (!region(off, n, sizeof(T), alignof(T))) {
      return {};
    }
    return Span<const T>(reinterpret_cast<const T *>(base_ + off), n);
  }

  Desc desc(std::size_t off) const noexcept { return get<Desc>(off); }
  std::size_t len(std::size_t off) const noexcept { return desc(off).len; }

  // Elements of a variable array, aliasing the frame. Empty (and bad()) if the descriptor does
  // not fit or points somewhere a T cannot be read from.
  template <class T>
  Span<const T> span(std::size_t off) const noexcept
  {
    const Desc d = desc(off);
    if (!region(d.off, d.len, sizeof(T), alignof(T))) {
      return {};
    }
    return Span<const T>(reinterpret_cast<const T *>(base_ + d.off), d.len);
  }

  std::string_view str(std::size_t off) const noexcept
  {
    const Desc d = desc(off);
    if (bad_ || !detail::fits(d.off, d.len, size_)) {
      bad_ = true;
      return {};
    }
    return std::string_view(reinterpret_cast<const char *>(base_ + d.off), d.len);
  }

  // Entry `i` of a string array whose descriptor is at `off`.
  std::string_view str_at(std::size_t off, std::size_t i) const noexcept
  {
    const Desc d = desc(off);
    if (!region(d.off, d.len, sizeof(Desc), alignof(Desc)) || i >= d.len) {
      bad_ = true;
      return {};
    }
    return str(static_cast<std::size_t>(d.off) + i * sizeof(Desc));
  }

  // Frame offset of element `i` of a record/jagged array whose descriptor is at `off`.
  std::size_t elem(
    std::size_t off, std::size_t i, std::size_t stride, std::size_t align) const noexcept
  {
    const Desc d = desc(off);
    if (!region(d.off, d.len, stride, align) || i >= d.len) {
      bad_ = true;
      return 0;
    }
    return static_cast<std::size_t>(d.off) + i * stride;
  }

private:
  bool region(std::size_t off, std::size_t n, std::size_t esize, std::size_t align) const noexcept
  {
    if (bad_ || off % align != 0 || off > size_ || n > (size_ - off) / esize) {
      bad_ = true;
      return false;
    }
    return true;
  }

  const std::byte * base_ = nullptr;
  std::size_t size_ = 0;
  mutable bool bad_ = false;
};

// Write side: a bump allocator over a loaned slot. Each alloc aligns the cursor, hands back
// slot-resident memory to fill, and stores the descriptor. Allocation order is free -- the
// descriptor records where each part landed.
class Writer
{
public:
  Writer() = default;
  Writer(void * base, std::size_t capacity, std::size_t scalar_bytes) noexcept
  : base_(static_cast<std::byte *>(base)),
    cap_(capacity),
    used_(scalar_bytes),
    bad_(base == nullptr || scalar_bytes > capacity)
  {
    if (!bad_) {
      // The scalar block is the one region that must start clean: a descriptor its writer never
      // sets would otherwise be last frame's, and point at bytes this frame does not own.
      std::memset(base_, 0, scalar_bytes);
    }
  }

  bool bad() const noexcept { return bad_; }
  explicit operator bool() const noexcept { return !bad_; }
  std::size_t size() const noexcept { return used_; }  // bytes to commit
  std::byte * base() const noexcept { return base_; }

  // Latch failure from a generated adapter's own validation (a fixed-N array given the wrong
  // length), so commit() refuses exactly as it does for an out-of-bounds write.
  void poison() noexcept { bad_ = true; }

  template <class T>
  void put(std::size_t off, const T & v) noexcept
  {
    static_assert(std::is_trivially_copyable_v<T>);
    if (bad_ || !detail::fits(off, sizeof(T), cap_)) {
      bad_ = true;
      return;
    }
    std::memcpy(base_ + off, &v, sizeof(T));
  }

  // A fixed T[N] block, writable in place.
  template <class T>
  Span<T> block(std::size_t off, std::size_t n) noexcept
  {
    // Divide rather than multiply, as region() and reserve() do: `n * sizeof(T)` wraps for a
    // large n and the wrapped product then passes the bound it was meant to fail.
    if (bad_ || off > cap_ || n > (cap_ - off) / sizeof(T)) {
      bad_ = true;
      return {};
    }
    return Span<T>(reinterpret_cast<T *>(base_ + off), n);
  }

  // Reserve n elements and record the descriptor at `off`. The returned span aliases the slot:
  // filling it is the 0-copy write path (docs/en/copy_model.en.md). Contents are left as they were
  // -- the caller is about to overwrite them, and zeroing first would double the write traffic.
  template <class T>
  Span<T> alloc(std::size_t off, std::size_t n) noexcept
  {
    void * p = reserve(off, n, sizeof(T), alignof(T), false);
    return p ? Span<T>(reinterpret_cast<T *>(p), n) : Span<T>();
  }

  bool put_str(std::size_t off, std::string_view s) noexcept
  {
    void * p = reserve(off, s.size(), 1, 1, false);
    if (p == nullptr) {
      return false;
    }
    std::memcpy(p, s.data(), s.size());
    return true;
  }

  // Reserve a string array of n entries; entry i is written with put_str_at.
  bool alloc_strs(std::size_t off, std::size_t n) noexcept
  {
    return reserve(off, n, sizeof(Desc), alignof(Desc), true) != nullptr;
  }

  bool put_str_at(std::size_t off, std::size_t i, std::string_view s) noexcept
  {
    if (bad_ || !detail::fits(off, sizeof(Desc), cap_)) {
      bad_ = true;
      return false;
    }
    Desc d{};
    std::memcpy(&d, base_ + off, sizeof(d));
    if (i >= d.len) {
      bad_ = true;
      return false;
    }
    return put_str(static_cast<std::size_t>(d.off) + i * sizeof(Desc), s);
  }

  // Reserve n element blocks of `stride` bytes and return the first one's frame offset;
  // element i starts at that + i * stride. Zeroed: an element block holds descriptors, and one
  // its writer skips must read as empty rather than as a stale offset into this frame.
  std::size_t alloc_elems(
    std::size_t off, std::size_t n, std::size_t stride, std::size_t align) noexcept
  {
    void * p = reserve(off, n, stride, align, true);
    return p ? static_cast<std::size_t>(static_cast<std::byte *>(p) - base_) : 0;
  }

private:
  void * reserve(
    std::size_t off, std::size_t n, std::size_t esize, std::size_t align, bool zero) noexcept
  {
    if (bad_ || !detail::fits(off, sizeof(Desc), cap_)) {
      bad_ = true;
      return nullptr;
    }
    const std::size_t start = detail::align_up(used_, align);
    if (start > cap_ || n > (cap_ - start) / esize || start + n * esize > kMaxFrameBytes) {
      bad_ = true;
      return nullptr;
    }
    const Desc d{static_cast<std::uint32_t>(start), static_cast<std::uint32_t>(n)};
    std::memcpy(base_ + off, &d, sizeof(d));
    used_ = start + n * esize;
    if (zero) {
      std::memset(base_ + start, 0, n * esize);
    }
    return base_ + start;
  }

  std::byte * base_ = nullptr;
  std::size_t cap_ = 0;
  std::size_t used_ = 0;
  bool bad_ = false;
};

}  // namespace flux::wire

#endif  // FLUX_WIRE_HPP
