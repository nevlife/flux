#ifndef FLUX_WRAPPER_MESSAGES_HPP
#define FLUX_WRAPPER_MESSAGES_HPP

#include <builtin_interfaces/msg/time.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#ifdef USE_FLUX_ENABLED
#include <sensor_msgs/flux/image.hpp>
#include <sensor_msgs/flux/image_ros.hpp>
#include <sensor_msgs/flux/point_cloud2.hpp>
#include <sensor_msgs/flux/point_cloud2_ros.hpp>
#endif

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace flux_wrapper
{

template <class MsgT>
class Publisher;

template <class T>
class Span
{
public:
  Span() = default;
  Span(T * data, std::size_t size) noexcept : data_(data), size_(size) {}
  [[nodiscard]] auto data() const noexcept -> T * { return data_; }
  [[nodiscard]] auto size() const noexcept -> std::size_t { return size_; }
  [[nodiscard]] auto empty() const noexcept -> bool { return size_ == 0; }
  [[nodiscard]] auto begin() const noexcept -> T * { return data_; }
  [[nodiscard]] auto end() const noexcept -> T * { return data_ + size_; }
  auto operator[](std::size_t i) const noexcept -> T & { return data_[i]; }

private:
  T * data_ = nullptr;
  std::size_t size_ = 0;
};

// One PointField as read from a View; `name` aliases the message like the View does.
struct Field
{
  std::string_view name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
};

// Write side (Loan) and read side (View) of each carried type. Both have the fields of the ROS
// message, so a driver writes one piece of code for the ROS path and the flux path.
template <class MsgT>
class Loan;
template <class MsgT>
class View;

namespace detail
{

#ifdef USE_FLUX_ENABLED
template <class MsgT>
struct Adapter;
template <>
struct Adapter<sensor_msgs::msg::Image>
{
  using type = sensor_msgs::flux_msg::Image;
};
template <>
struct Adapter<sensor_msgs::msg::PointCloud2>
{
  using type = sensor_msgs::flux_msg::PointCloud2;
};
template <class MsgT>
using AdapterT = typename Adapter<MsgT>::type;
#endif

template <class MsgT>
class LoanBase
{
public:
  // False when the flux ring had no free slot or a field did not fit the slot.
  explicit operator bool() const noexcept
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      return builder_->ok__();
    }
#endif
    return msg_ != nullptr;
  }

  void set_header(const std_msgs::msg::Header & h)
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      builder_->set__header__stamp(h.stamp.sec, h.stamp.nanosec);
      builder_->set__header__frame_id(h.frame_id);
      return;
    }
#endif
    msg_->header = h;
  }

  // `n` bytes of the data field to write in place. Empty when they do not fit the flux slot.
  auto alloc_data(std::size_t n) -> Span<std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      auto s = builder_->alloc__data(n);
      return s.size() == n ? Span<std::uint8_t>(s.data(), n) : Span<std::uint8_t>();
    }
#endif
    msg_->data.resize(n);
    return {msg_->data.data(), n};
  }

protected:
  LoanBase() = default;
  explicit LoanBase(std::unique_ptr<MsgT> msg) : msg_(std::move(msg)) {}
#ifdef USE_FLUX_ENABLED
  using Builder = typename AdapterT<MsgT>::Builder;
  explicit LoanBase(Builder && b) : builder_(std::move(b)) {}
  std::optional<Builder> builder_;
#endif
  std::unique_ptr<MsgT> msg_;

  friend class flux_wrapper::Publisher<MsgT>;
};

// A flux View aliases the slot: valid only during the callback it was passed to.
template <class MsgT>
class ViewBase
{
public:
  explicit ViewBase(const MsgT & m) noexcept : msg_(&m) {}
#ifdef USE_FLUX_ENABLED
  explicit ViewBase(const typename AdapterT<MsgT>::View & v) noexcept : flux_(v) {}
#endif

  [[nodiscard]] auto stamp() const noexcept -> builtin_interfaces::msg::Time
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      builtin_interfaces::msg::Time t;
      t.sec = flux_->header__sec();
      t.nanosec = flux_->header__nanosec();
      return t;
    }
#endif
    return msg_->header.stamp;
  }
  [[nodiscard]] auto frame_id() const noexcept -> std::string_view
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return flux::wire::until_nul(flux_->header__frame_id());
    }
#endif
    return msg_->header.frame_id;
  }
  [[nodiscard]] auto data() const noexcept -> Span<const std::uint8_t>
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      auto s = flux_->data();
      return {s.data(), s.size()};
    }
#endif
    return {msg_->data.data(), msg_->data.size()};
  }

  // The message itself on the ROS path, for code that keeps it; null on the flux path.
  [[nodiscard]] auto ros_message() const noexcept -> const MsgT * { return msg_; }

  // A copy that outlives the callback.
  [[nodiscard]] auto to_msg() const -> MsgT
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return sensor_msgs::flux_msg::frame_to_msg(*flux_);
    }
#endif
    return *msg_;
  }

protected:
  const MsgT * msg_ = nullptr;
#ifdef USE_FLUX_ENABLED
  std::optional<typename AdapterT<MsgT>::View> flux_;
#endif
};

}  // namespace detail

#ifdef USE_FLUX_ENABLED
#define FLUX_WRAPPER_SET(field, value) \
  if (builder_) {                      \
    builder_->set__##field(value);     \
  } else {                             \
    msg_->field = value;               \
  }
#define FLUX_WRAPPER_GET(field) return flux_ ? flux_->field() : msg_->field;
#else
#define FLUX_WRAPPER_SET(field, value) msg_->field = value;
#define FLUX_WRAPPER_GET(field) return msg_->field;
#endif

template <>
class Loan<sensor_msgs::msg::Image> : public detail::LoanBase<sensor_msgs::msg::Image>
{
public:
  Loan() = default;

  void set_height(std::uint32_t v) { FLUX_WRAPPER_SET(height, v) }
  void set_width(std::uint32_t v) { FLUX_WRAPPER_SET(width, v) }
  void set_encoding(const std::string & v) { FLUX_WRAPPER_SET(encoding, v) }
  void set_is_bigendian(std::uint8_t v) { FLUX_WRAPPER_SET(is_bigendian, v) }
  void set_step(std::uint32_t v) { FLUX_WRAPPER_SET(step, v) }

private:
  using LoanBase::LoanBase;
  friend class Publisher<sensor_msgs::msg::Image>;
};

template <>
class Loan<sensor_msgs::msg::PointCloud2> : public detail::LoanBase<sensor_msgs::msg::PointCloud2>
{
public:
  Loan() = default;

  void set_height(std::uint32_t v) { FLUX_WRAPPER_SET(height, v) }
  void set_width(std::uint32_t v) { FLUX_WRAPPER_SET(width, v) }
  void set_is_bigendian(bool v) { FLUX_WRAPPER_SET(is_bigendian, v) }
  void set_point_step(std::uint32_t v) { FLUX_WRAPPER_SET(point_step, v) }
  void set_row_step(std::uint32_t v) { FLUX_WRAPPER_SET(row_step, v) }
  void set_is_dense(bool v) { FLUX_WRAPPER_SET(is_dense, v) }

  void set_fields(const std::vector<sensor_msgs::msg::PointField> & fields)
  {
#ifdef USE_FLUX_ENABLED
    if (builder_) {
      auto out = builder_->alloc__fields(fields.size());
      for (std::size_t i = 0; i < out.size(); ++i) {
        out[i].set__name(fields[i].name);
        out[i].set__offset(fields[i].offset);
        out[i].set__datatype(fields[i].datatype);
        out[i].set__count(fields[i].count);
      }
      return;
    }
#endif
    msg_->fields = fields;
  }

private:
  using LoanBase::LoanBase;
  friend class Publisher<sensor_msgs::msg::PointCloud2>;
};

template <>
class View<sensor_msgs::msg::Image> : public detail::ViewBase<sensor_msgs::msg::Image>
{
public:
  using ViewBase::ViewBase;

  [[nodiscard]] auto height() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(height) }
  [[nodiscard]] auto width() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(width) }
  [[nodiscard]] auto encoding() const noexcept -> std::string_view
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return flux::wire::until_nul(flux_->encoding());
    }
#endif
    return msg_->encoding;
  }
  [[nodiscard]] auto is_bigendian() const noexcept -> std::uint8_t
  {
    FLUX_WRAPPER_GET(is_bigendian)
  }
  [[nodiscard]] auto step() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(step) }
};

template <>
class View<sensor_msgs::msg::PointCloud2> : public detail::ViewBase<sensor_msgs::msg::PointCloud2>
{
public:
  using ViewBase::ViewBase;

  [[nodiscard]] auto height() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(height) }
  [[nodiscard]] auto width() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(width) }
  [[nodiscard]] auto fields_size() const noexcept -> std::size_t
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      return flux_->fields__size();
    }
#endif
    return msg_->fields.size();
  }
  [[nodiscard]] auto field(std::size_t i) const noexcept -> Field
  {
#ifdef USE_FLUX_ENABLED
    if (flux_) {
      const auto f = flux_->fields(i);
      return {flux::wire::until_nul(f.name()), f.offset(), f.datatype(), f.count()};
    }
#endif
    const auto & f = msg_->fields[i];
    return {f.name, f.offset, f.datatype, f.count};
  }
  [[nodiscard]] auto is_bigendian() const noexcept -> bool { FLUX_WRAPPER_GET(is_bigendian) }
  [[nodiscard]] auto point_step() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(point_step) }
  [[nodiscard]] auto row_step() const noexcept -> std::uint32_t { FLUX_WRAPPER_GET(row_step) }
  [[nodiscard]] auto is_dense() const noexcept -> bool { FLUX_WRAPPER_GET(is_dense) }
};

#undef FLUX_WRAPPER_SET
#undef FLUX_WRAPPER_GET

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_MESSAGES_HPP
