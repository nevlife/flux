#ifndef FLUX_WIRE_HPP
#define FLUX_WIRE_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>
#include <utility>

// Runtime for flux_gen adapters: read and write the flat frame layout flux_gen computes
// (flux_gen/src/layout.rs). Header-only and ROS-free, so a generated header pulls in nothing
// but this. Python adapters use it too, through flux.wire. Hand-written code can use it; the
// offsets are the generated constants.
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
// Reader assumes its base is aligned for the widest scalar, 8 bytes. A flux frame starts on a
// page: payload offset and stride are page multiples over a page-aligned base, shm or heap
// (flux_core layout.rs, segment.rs).

namespace flux::wire
{

// Typed views over frame bytes, each made only after region() checked its bounds and alignment.
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)

struct Desc
{
  std::uint32_t off;
  std::uint32_t len;
};

static_assert(sizeof(Desc) == 8);

constexpr std::size_t kMaxFrameBytes = 0xFFFFFFFFu;  // descriptors are uint32

// Experimental: a descriptor offset at or past this is kHeapBase + an offset into the publisher's
// arena, a shm file the frame's trailer names (FLUX_HEAP_BASE). Frames stay pointer-free.
constexpr std::size_t kHeapBase = std::size_t{1} << 31;
// Room the commit needs at the frame's end to name the arena: 64 bytes on an 8-byte boundary.
constexpr std::size_t kHeapTrailer = 64 + 7;

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

// Experimental: where a Writer puts what does not fit its slot. A loan's handle on the arena
// (flux::WriteSlot::heap()), valid while the loan lives; empty for none.
struct Heap
{
  std::byte * base = nullptr;
  std::size_t size = 0;
  const void * channel = nullptr;
  std::uint32_t slot = 0;
  // Arena memory the frame owns until its slot is claimed again; *off gets its arena offset.
  void * (*alloc)(const Heap &, std::size_t bytes, std::size_t align, std::uint64_t * off) =
    nullptr;
  // Hands the frame arena memory the malloc hook placed; release(owner) frees it later. False,
  // and release not kept, when it is not all in the arena.
  bool (*adopt)(
    const Heap &, const void * p, std::size_t bytes, void (*release)(void *), void * owner,
    std::uint64_t * off) = nullptr;
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

// A bool stored in a frame. Its byte may hold any value, and reading one other than 0 or 1 as a
// bool is undefined: any nonzero byte is true, as the Python adapter and the CDR bridge read it.
inline bool as_bool(const bool & b) noexcept
{
  unsigned char c = 0;
  std::memcpy(&c, &b, 1);
  return c != 0;
}

// A frame string up to its first NUL, the string a ROS message holds: rclpy cuts there, and
// rclcpp refuses to serialize one with a NUL inside.
inline std::string_view until_nul(std::string_view s) noexcept
{
  return s.substr(0, s.find('\0'));
}

// DLPack's DLDataType, as a tensor message carries it in dtype_code, dtype_bits, dtype_lanes.
struct DlType
{
  std::uint8_t code;
  std::uint8_t bits;
  std::uint16_t lanes;
};

constexpr std::uint8_t kDlInt = 0;
constexpr std::uint8_t kDlUInt = 1;
constexpr std::uint8_t kDlFloat = 2;
constexpr std::uint8_t kDlBfloat = 4;
constexpr std::uint8_t kDlComplex = 5;
constexpr std::uint8_t kDlBool = 6;
constexpr std::uint8_t kDlFloat8First = 7;
constexpr std::uint8_t kDlFloat8Last = 14;

// A tensor message's data starts on this boundary when a Writer lays it out.
constexpr std::size_t kTensorAlign = 64;

// 0 for a dtype flux does not hand out: lanes other than 1, opaque handles, sub-byte floats,
// codes DLPack does not define.
constexpr std::size_t dl_itemsize(DlType t) noexcept
{
  const std::size_t b = t.bits;
  const bool whole = b == 8 || b == 16 || b == 32 || b == 64;
  if (t.lanes != 1) {
    return 0;
  }
  switch (t.code) {
    case kDlInt:
    case kDlUInt:
      return whole ? b / 8 : 0;
    case kDlFloat:
      return whole && b != 8 ? b / 8 : 0;
    case kDlBfloat:
      return b == 16 ? 2 : 0;
    case kDlComplex:
      return b == 64 || b == 128 ? b / 8 : 0;
    default:
      return b == 8 && (t.code == kDlBool || (t.code >= kDlFloat8First && t.code <= kDlFloat8Last))
               ? 1
               : 0;
  }
}

template <class T>
constexpr DlType dl_type() noexcept
{
  static_assert(std::is_arithmetic_v<T>);
  constexpr auto bits = static_cast<std::uint8_t>(8 * sizeof(T));
  if constexpr (std::is_same_v<T, bool>) {
    return {kDlBool, bits, 1};
  } else if constexpr (std::is_floating_point_v<T>) {
    return {kDlFloat, bits, 1};
  } else {
    return {std::is_signed_v<T> ? kDlInt : kDlUInt, bits, 1};
  }
}

// Where a tensor message's fields sit in its scalar block: the generated kTensorAt.
struct TensorAt
{
  std::size_t code, bits, lanes, shape, strides, byte_offset, data;
};

// A tensor message's data as its other fields describe it. `data` is the first element; empty
// strides (in elements) are C-contiguous.
struct Tensor
{
  const void * data = nullptr;
  DlType dtype{};
  Span<const std::int64_t> shape;
  Span<const std::int64_t> strides;

  // Null unless the frame's dtype is T's.
  template <class T>
  const T * as() const noexcept
  {
    constexpr DlType t = dl_type<T>();
    return dtype.code == t.code && dtype.bits == t.bits && dtype.lanes == 1
             ? static_cast<const T *>(data)
             : nullptr;
  }
};

// Why these fields put an element of `t` outside the `len` bytes at `field`, or off a boundary
// of its size; nullptr when every element is inside and aligned.
inline const char * tensor_fault(
  const void * field, std::size_t len, DlType t, Span<const std::int64_t> shape,
  Span<const std::int64_t> strides, std::uint64_t byte_offset) noexcept
{
  const auto size = static_cast<std::int64_t>(dl_itemsize(t));
  if (size == 0) {
    return "a dtype flux does not read";
  }
  if (!strides.empty() && strides.size() != shape.size()) {
    return "strides that do not match the shape";
  }
  if (byte_offset > len) {
    return "a byte_offset past the data";
  }
  if ((reinterpret_cast<std::uintptr_t>(field) + byte_offset) % size != 0) {
    return "a first element off its alignment";
  }
  std::int64_t lo = 0;
  std::int64_t hi = 0;
  std::int64_t step = 1;
  bool empty = false;
  for (std::size_t i = shape.size(); i-- > 0;) {
    const std::int64_t n = shape[i];
    if (n < 0) {
      return "a negative extent";
    }
    const std::int64_t stride = strides.empty() ? step : strides[i];
    std::int64_t reach = 0;
    if (
      __builtin_mul_overflow(stride, n == 0 ? 0 : n - 1, &reach) ||
      __builtin_add_overflow(reach > 0 ? hi : lo, reach, reach > 0 ? &hi : &lo) ||
      (strides.empty() && __builtin_mul_overflow(step, n, &step))) {
      return "an extent that overflows";
    }
    empty = empty || n == 0;
  }
  if (empty) {
    return nullptr;
  }
  std::int64_t first = 0;
  std::int64_t end = 0;
  if (
    __builtin_mul_overflow(lo, size, &first) || __builtin_add_overflow(hi, 1, &end) ||
    __builtin_mul_overflow(end, size, &end)) {
    return "an extent that overflows";
  }
  const auto at = static_cast<std::int64_t>(byte_offset);
  if (first < -at || end > static_cast<std::int64_t>(len) - at) {
    return "elements past the data";
  }
  return nullptr;
}

// Read side of a received frame. Construct over FrameView::data()/size().
class Reader
{
public:
  Reader() = default;
  // `heap` is the arena the frame's trailer names, if any (flux::FrameView::heap()).
  Reader(const void * base, std::size_t size, Span<const std::byte> heap = {}) noexcept
  : base_(static_cast<const std::byte *>(base)),
    size_(size),
    heap_(heap.data()),
    heap_size_(heap.size()),
    bad_(base == nullptr)
  {
  }

  bool bad() const noexcept { return bad_; }
  explicit operator bool() const noexcept { return !bad_; }
  std::size_t size() const noexcept { return size_; }
  // The bytes descriptors can name: the frame's and its arena's.
  std::size_t budget() const noexcept { return size_ + heap_size_; }
  const std::byte * base() const noexcept { return base_; }

  template <class T>
  T get(std::size_t off) const noexcept
  {
    static_assert(std::is_trivially_copyable_v<T>);
    T v{};
    const std::byte * p = region(off, 1, sizeof(T), 1);
    if (p == nullptr) {
      return v;
    }
    if constexpr (std::is_same_v<T, bool>) {
      return std::to_integer<unsigned char>(*p) != 0;  // as as_bool()
    }
    std::memcpy(&v, p, sizeof(T));  // offsets are aligned by construction; be exact
    return v;
  }

  // A fixed T[N] block, inline in the scalar block rather than behind a descriptor.
  template <class T>
  Span<const T> block(std::size_t off, std::size_t n) const noexcept
  {
    const std::byte * p = region(off, n, sizeof(T), alignof(T));
    return p != nullptr ? Span<const T>(reinterpret_cast<const T *>(p), n) : Span<const T>();
  }

  Desc desc(std::size_t off) const noexcept { return get<Desc>(off); }
  // Element count of a variable array. 0 (and bad()) if its `esize`-byte elements do not fit, so
  // a loop bounded by it never indexes past the frame.
  std::size_t len(std::size_t off, std::size_t esize, std::size_t align) const noexcept
  {
    const Desc d = desc(off);
    return region(d.off, d.len, esize, align) != nullptr ? d.len : 0;
  }

  // Elements of a variable array, aliasing the frame. Empty (and bad()) if the descriptor does
  // not fit or points somewhere a T cannot be read from.
  template <class T>
  Span<const T> span(std::size_t off) const noexcept
  {
    const Desc d = desc(off);
    const std::byte * p = region(d.off, d.len, sizeof(T), alignof(T));
    return p != nullptr ? Span<const T>(reinterpret_cast<const T *>(p), d.len) : Span<const T>();
  }

  std::string_view str(std::size_t off) const noexcept
  {
    const Desc d = desc(off);
    const std::byte * p = region(d.off, d.len, 1, 1);
    return p != nullptr ? std::string_view(reinterpret_cast<const char *>(p), d.len)
                        : std::string_view();
  }

  // Entry `i` of a string array whose descriptor is at `off`.
  std::string_view str_at(std::size_t off, std::size_t i) const noexcept
  {
    const Desc d = desc(off);
    if (region(d.off, d.len, sizeof(Desc), alignof(Desc)) == nullptr || i >= d.len) {
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
    if (region(d.off, d.len, stride, align) == nullptr || i >= d.len) {
      bad_ = true;
      return 0;
    }
    return static_cast<std::size_t>(d.off) + i * stride;
  }

  // Aliases the data field. Empty (and bad()) when tensor_fault() finds one; `why` then names it,
  // unless a descriptor escaped the frame first.
  Tensor tensor(const TensorAt & at, const char ** why = nullptr) const noexcept
  {
    const DlType t{
      get<std::uint8_t>(at.code), get<std::uint8_t>(at.bits), get<std::uint16_t>(at.lanes)};
    const Span<const std::int64_t> shape = span<std::int64_t>(at.shape);
    const Span<const std::int64_t> strides = span<std::int64_t>(at.strides);
    const auto byte_offset = get<std::uint64_t>(at.byte_offset);
    const Span<const std::uint8_t> data = span<std::uint8_t>(at.data);
    if (bad_) {
      return {};
    }
    const char * fault = tensor_fault(data.data(), data.size(), t, shape, strides, byte_offset);
    if (fault != nullptr) {
      bad_ = true;
      if (why != nullptr) {
        *why = fault;
      }
      return {};
    }
    return {data.data() + byte_offset, t, shape, strides};
  }

private:
  // n elements of esize at `off`, in the frame or (past kHeapBase) the arena; null, and bad(),
  // unless all of them are inside and the first is on `align`.
  const std::byte * region(
    std::size_t off, std::size_t n, std::size_t esize, std::size_t align) const noexcept
  {
    const std::byte * base = base_;
    std::size_t size = size_;
    if (off >= kHeapBase) {
      base = heap_;
      size = heap_size_;
      off -= kHeapBase;
    }
    if (bad_ || base == nullptr || off % align != 0 || off > size || n > (size - off) / esize) {
      bad_ = true;
      return nullptr;
    }
    return base + off;
  }

  const std::byte * base_ = nullptr;
  std::size_t size_ = 0;
  const std::byte * heap_ = nullptr;
  std::size_t heap_size_ = 0;
  mutable bool bad_ = false;
};

// Write side: a bump allocator over a loaned slot. Each alloc aligns the cursor, hands back
// slot-resident memory to fill, and stores the descriptor. Allocation order is free -- the
// descriptor records where each part landed.
class Writer
{
public:
  Writer() = default;
  // With a `heap`, what does not fit the slot goes to the arena instead of failing.
  Writer(
    void * base, std::size_t capacity, std::size_t scalar_bytes, const Heap & heap = {}) noexcept
  : base_(static_cast<std::byte *>(base)),
    cap_(capacity),
    used_(scalar_bytes),
    heap_(heap),
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

  template <class T>
  void put(std::size_t off, const T & v) noexcept
  {
    static_assert(std::is_trivially_copyable_v<T>);
    std::byte * p = at(off, 1, sizeof(T));
    if (p != nullptr) {
      std::memcpy(p, &v, sizeof(T));
    }
  }

  // A fixed T[N] block, writable in place.
  template <class T>
  Span<T> block(std::size_t off, std::size_t n) noexcept
  {
    std::byte * p = at(off, n, sizeof(T));
    return p != nullptr ? Span<T>(reinterpret_cast<T *>(p), n) : Span<T>();
  }

  // Reserve n elements and record the descriptor at `off`. The returned span aliases the slot:
  // filling it is the 0-copy write path. Contents are left as they were -- the caller is about
  // to overwrite them, and zeroing first would double the write traffic.
  template <class T>
  Span<T> alloc(std::size_t off, std::size_t n) noexcept
  {
    void * p = reserve(off, n, sizeof(T), alignof(T), false);
    return p ? Span<T>(static_cast<T *>(p), n) : Span<T>();
  }

  // On an `align` boundary past T's own; the frame base has it, as a page does.
  template <class T>
  Span<T> alloc(std::size_t off, std::size_t n, std::size_t align) noexcept
  {
    void * p = reserve(off, n, sizeof(T), align > alignof(T) ? align : alignof(T), false);
    return p ? Span<T>(static_cast<T *>(p), n) : Span<T>();
  }

  // A C-contiguous tensor of `shape` in a tensor message: its dtype, shape, empty strides and
  // zero byte_offset, and data on a kTensorAlign boundary. Null (and bad()) if it does not fit
  // or `t` is not a dtype flux reads.
  void * tensor(const TensorAt & at, DlType t, Span<const std::int64_t> shape) noexcept
  {
    std::size_t bytes = dl_itemsize(t);
    bool ok = bytes != 0;
    for (const std::int64_t n : shape) {
      ok = ok && n >= 0 && !__builtin_mul_overflow(bytes, static_cast<std::size_t>(n), &bytes);
    }
    if (!ok) {
      bad_ = true;
      return nullptr;
    }
    put<std::uint8_t>(at.code, t.code);
    put<std::uint8_t>(at.bits, t.bits);
    put<std::uint16_t>(at.lanes, t.lanes);
    const Span<std::int64_t> s = alloc<std::int64_t>(at.shape, shape.size());
    if (s.size() == shape.size() && !shape.empty()) {
      std::memcpy(s.data(), shape.data(), shape.size() * sizeof(std::int64_t));
    }
    alloc<std::int64_t>(at.strides, 0);
    put<std::uint64_t>(at.byte_offset, 0);
    void * p = alloc<std::uint8_t>(at.data, bytes, kTensorAlign).data();
    return bad_ ? nullptr : p;
  }

  template <class T>
  Span<T> tensor(const TensorAt & at, Span<const std::int64_t> shape) noexcept
  {
    void * p = tensor(at, dl_type<T>(), shape);
    std::size_t n = 1;
    for (const std::int64_t s : shape) {
      n *= static_cast<std::size_t>(s);
    }
    return p ? Span<T>(static_cast<T *>(p), n) : Span<T>();
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
    const std::byte * p = at(off, 1, sizeof(Desc));
    if (p == nullptr) {
      return false;
    }
    Desc d{};
    std::memcpy(&d, p, sizeof(d));
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
    std::size_t at_off = 0;
    return reserve(off, n, stride, align, true, &at_off) != nullptr ? at_off : 0;
  }

  // Experimental: n elements at p, arena memory the malloc hook placed, become the array at `off`
  // without a copy; release(owner) runs when the slot is claimed again. False, with nothing
  // recorded, when they are not all in the arena.
  bool adopt(
    std::size_t off, const void * p, std::size_t n, std::size_t esize, void (*release)(void *),
    void * owner) noexcept
  {
    std::byte * dp = at(off, 1, sizeof(Desc));
    std::uint64_t aoff = 0;
    if (
      dp == nullptr || heap_.adopt == nullptr || p == nullptr || n > 0xFFFFFFFFu ||
      n > heap_.size / esize || !heap_.adopt(heap_, p, n * esize, release, owner, &aoff)) {
      return false;
    }
    const Desc d{static_cast<std::uint32_t>(kHeapBase + aoff), static_cast<std::uint32_t>(n)};
    std::memcpy(dp, &d, sizeof(d));
    return true;
  }

  // A container whose buffer was allocated under the malloc hook (flux::ArenaScope) becomes the
  // array at `off` without a copy; any other is copied. True when it was not copied.
  template <class C>
  bool adopt(std::size_t off, C && c)
  {
    using V = std::remove_cv_t<std::remove_reference_t<C>>;
    using T = typename V::value_type;
    static_assert(std::is_trivially_copyable_v<T>);
    auto * keep = new V(std::forward<C>(c));  // NOLINT(cppcoreguidelines-owning-memory)
    const auto release = [](void * k) {
      delete static_cast<V *>(k);  // NOLINT(cppcoreguidelines-owning-memory)
    };
    if (adopt(off, keep->data(), keep->size(), sizeof(T), release, keep)) {
      return true;
    }
    const Span<T> s = alloc<T>(off, keep->size());
    if (!keep->empty() && s.size() == keep->size()) {
      std::memcpy(s.data(), keep->data(), keep->size() * sizeof(T));
    }
    release(keep);
    return false;
  }

private:
  // n elements of esize at `off`, in the slot or (past kHeapBase) the arena; null, and bad(),
  // unless all of them are inside.
  std::byte * at(std::size_t off, std::size_t n, std::size_t esize) noexcept
  {
    std::byte * base = base_;
    std::size_t size = cap_;
    if (off >= kHeapBase) {
      base = heap_.base;
      size = heap_.size;
      off -= kHeapBase;
    }
    // Divide rather than multiply: `n * esize` wraps for a large n and the wrapped product then
    // passes the bound it was meant to fail.
    if (bad_ || base == nullptr || off > size || n > (size - off) / esize) {
      bad_ = true;
      return nullptr;
    }
    return base + off;
  }

  // In the slot while it fits, else in the arena.
  void * reserve(
    std::size_t off, std::size_t n, std::size_t esize, std::size_t align, bool zero,
    std::size_t * at_off = nullptr) noexcept
  {
    std::byte * dp = at(off, 1, sizeof(Desc));
    if (dp == nullptr) {
      return nullptr;
    }
    const bool heap = heap_.alloc != nullptr;
    const std::size_t cap = heap && cap_ >= kHeapTrailer ? cap_ - kHeapTrailer : cap_;
    const std::size_t limit = heap ? kHeapBase : kMaxFrameBytes;
    const std::size_t start = detail::align_up(used_, align);
    std::byte * p = nullptr;
    std::size_t where = 0;
    if (start <= cap && n <= (cap - start) / esize && start + n * esize <= limit) {
      p = base_ + start;
      where = start;
      used_ = start + n * esize;
    } else if (heap_.alloc != nullptr && n <= 0xFFFFFFFFu && n <= heap_.size / esize) {
      std::uint64_t aoff = 0;
      p = static_cast<std::byte *>(heap_.alloc(heap_, n * esize, align, &aoff));
      where = kHeapBase + static_cast<std::size_t>(aoff);
    }
    if (p == nullptr) {
      bad_ = true;
      return nullptr;
    }
    const Desc d{static_cast<std::uint32_t>(where), static_cast<std::uint32_t>(n)};
    std::memcpy(dp, &d, sizeof(d));
    if (zero) {
      std::memset(p, 0, n * esize);
    }
    if (at_off != nullptr) {
      *at_off = where;
    }
    return p;
  }

  std::byte * base_ = nullptr;
  std::size_t cap_ = 0;
  std::size_t used_ = 0;
  Heap heap_;
  bool bad_ = false;
};

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)

}  // namespace flux::wire

#endif  // FLUX_WIRE_HPP
