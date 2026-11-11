#ifndef FLUX_WRAPPER_SUBSCRIPTION_HPP
#define FLUX_WRAPPER_SUBSCRIPTION_HPP

#include "flux_wrapper/messages.hpp"
#include "flux_wrapper/runtime.hpp"

#include <rclcpp/rclcpp.hpp>

#ifdef USE_FLUX_ENABLED
#include <flux/subscription.hpp>
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace flux_wrapper
{

struct SubscriptionOptions
{
  // Null: the node's default group. On the flux path the subscription is served by the flux
  // executor that runs this group (flux_wrapper::Executor, a flux component container).
  rclcpp::CallbackGroup::SharedPtr callback_group;
  // Cuda: every frame's data field is in device memory (View::on_device()), whatever the
  // publisher's device. Where the GPU cannot read a host publisher's frame in place, flux copies it
  // to the device at take; on the ROS path each message is copied to one of max_borrow device
  // buffers of the subscription's. With Host, frames of a Device::Cuda publisher on a discrete GPU
  // are copied to host memory at take.
  Device device = Device::Host;
  // Views alive at once: the one in a callback and those a HeldCallback keeps. Over flux a frame
  // past it waits in the publisher's ring, and a publisher of max_borrow slots or fewer is refused,
  // with flux::Error at construction or, under an executor, logged once and left out until it
  // restarts. Over ROS it bounds only the device buffers of Device::Cuda: a message that finds
  // none free is skipped.
  std::uint32_t max_borrow = 1;
};

namespace detail
{

// Calls a Callback with a View that lives for the call, or a HeldCallback with one that keeps
// alive the frame or message it reads.
template <class MsgT>
class Sink
{
public:
  using Callback = std::function<void(const View<MsgT> &)>;
  using HeldCallback = std::function<void(std::shared_ptr<const View<MsgT>>)>;

  explicit Sink(Callback cb) : cb_(std::move(cb)) {}
  explicit Sink(HeldCallback cb) : held_(std::move(cb)) {}

  // make(owner) is the View of what owner keeps alive.
  template <class Owner, class Make>
  void operator()(Owner && owner, const Make & make) const
  {
    if (!held_) {
      cb_(make(owner));
      return;
    }
    // One allocation a frame, not proportional to the message, as message_filters' StampedFrame.
    auto kept = std::make_shared<Kept<std::decay_t<Owner>>>(std::forward<Owner>(owner));
    kept->view.emplace(make(kept->owner));
    held_(std::shared_ptr<const View<MsgT>>(kept, &*kept->view));
  }

private:
  template <class Owner>
  struct Kept
  {
    template <class O>
    explicit Kept(O && o) : owner(std::forward<O>(o))
    {
    }

    Owner owner;
    std::optional<View<MsgT>> view;
  };

  Callback cb_;
  HeldCallback held_;
};

#ifdef FLUX_WRAPPER_CUDA
// The device buffers of a ROS-path subscription with Device::Cuda, one a View alive at once, each
// on its own stream. A buffer is written again only once the View that read it is gone, and then
// after the work queued on its stream.
class StagingPool : public std::enable_shared_from_this<StagingPool>
{
public:
  class Lease
  {
  public:
    Lease() = default;
    Lease(std::shared_ptr<StagingPool> pool, std::size_t i, const void * device) noexcept
    : pool_(std::move(pool)), i_(i), device_(device)
    {
    }
    ~Lease()
    {
      if (pool_) {
        pool_->busy_[i_].store(false, std::memory_order_release);
      }
    }
    Lease(const Lease &) = delete;
    auto operator=(const Lease &) -> Lease & = delete;
    Lease(Lease && o) noexcept : pool_(std::move(o.pool_)), i_(o.i_), device_(o.device_) {}
    auto operator=(Lease &&) -> Lease & = delete;

    explicit operator bool() const noexcept { return pool_ != nullptr; }
    [[nodiscard]] auto device() const noexcept -> const void * { return device_; }
    [[nodiscard]] auto stream() const noexcept -> cudaStream_t
    {
      return pool_->bufs_[i_]->stream();
    }

  private:
    std::shared_ptr<StagingPool> pool_;
    std::size_t i_ = 0;
    const void * device_ = nullptr;
  };

  // Null when a stream could not be created.
  static auto create(std::uint32_t n) -> std::shared_ptr<StagingPool>
  {
    std::shared_ptr<StagingPool> pool(new StagingPool(n));
    for (auto & b : pool->bufs_) {
      b = DeviceStaging::create();
      if (!b) {
        return nullptr;
      }
    }
    return pool;
  }

  // Copies n bytes to a free buffer. Empty when every buffer is held or the copy failed.
  auto upload(const std::uint8_t * src, std::size_t n) -> Lease
  {
    for (std::size_t i = 0; i < bufs_.size(); ++i) {
      if (busy_[i].exchange(true, std::memory_order_acquire)) {
        continue;
      }
      const void * d = bufs_[i]->upload(src, n);
      if (d == nullptr) {
        busy_[i].store(false, std::memory_order_release);
        return {};
      }
      return {shared_from_this(), i, d};
    }
    return {};
  }

private:
  explicit StagingPool(std::uint32_t n) : bufs_(n), busy_(new std::atomic<bool>[n] {}) {}

  std::vector<std::unique_ptr<DeviceStaging>> bufs_;
  std::unique_ptr<std::atomic<bool>[]> busy_;
};

// A ROS message and the device buffer it was copied to.
template <class MsgT>
struct StagedMessage
{
  typename MsgT::ConstSharedPtr msg;
  StagingPool::Lease lease;
};
#endif

}  // namespace detail

// One topic over flux or over ROS, chosen once per process by use_flux(). The callback gets a
// View with the message's fields on both paths. On the flux path a Callback's View aliases the
// slot and is valid only during the call; a HeldCallback's View keeps its frame or message, and on
// the flux path its slot, as long as a copy of the pointer lives (SubscriptionOptions::max_borrow).
// Keep the Subscription alive as long as the executor that serves it, as a member of the node. On
// the flux path it also takes the frames of a publisher with Device::Cuda.
template <class MsgT>
class Subscription
{
public:
  using Callback = typename detail::Sink<MsgT>::Callback;
  using HeldCallback = typename detail::Sink<MsgT>::HeldCallback;

  template <class NodeT>
  Subscription(
    NodeT & node, const std::string & topic, const rclcpp::QoS & qos, Callback callback,
    const SubscriptionOptions & options = {})
  {
    open(
      node, topic, qos, std::make_shared<const detail::Sink<MsgT>>(std::move(callback)), options);
  }
  template <class NodeT>
  Subscription(
    NodeT & node, const std::string & topic, const rclcpp::QoS & qos, HeldCallback callback,
    const SubscriptionOptions & options = {})
  {
    open(
      node, topic, qos, std::make_shared<const detail::Sink<MsgT>>(std::move(callback)), options);
  }

  [[nodiscard]] auto uses_flux() const noexcept -> bool
  {
#ifdef USE_FLUX_ENABLED
    return flux_ != nullptr;
#else
    return false;
#endif
  }

  // Frames of a Device::Cuda publisher skipped for a bad aux header; the first is logged.
  [[nodiscard]] auto rejected() const noexcept -> std::uint64_t
  {
#ifdef USE_FLUX_ENABLED
    if (rejects_) {
      return rejects_->count.load(std::memory_order_relaxed);
    }
#endif
    return 0;
  }

private:
  using SinkPtr = std::shared_ptr<const detail::Sink<MsgT>>;

  template <class NodeT>
  void open(
    NodeT & node, const std::string & topic, const rclcpp::QoS & qos, SinkPtr sink,
    const SubscriptionOptions & options)
  {
    if (options.max_borrow == 0) {
      throw std::invalid_argument("flux_wrapper: max_borrow must be >= 1");
    }
    [[maybe_unused]] const bool cuda =
      detail::cuda_usable(node.get_logger(), topic, options.device);
#ifdef USE_FLUX_ENABLED
    if (use_flux()) {
      flux::QoS q(static_cast<std::uint32_t>(qos.depth()));
      if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
        q.transient_local(static_cast<std::uint32_t>(qos.depth()));
      }
      q.max_borrow(options.max_borrow);
      using Adapter = detail::AdapterT<MsgT>;
      const flux::Device device = cuda ? flux::Device::Cuda : flux::Device::Cpu;
      if (cuda) {
        flux_ = std::make_unique<flux::Subscription>(
          node, topic, Adapter::kFingerprint, q,
          [sink](flux::FrameView && f) {
            (*sink)(std::move(f), [](const flux::FrameView & g) {
              const typename Adapter::View v(g);
              return View<MsgT>(v, v.data__device().data(), g.stream());
            });
          },
          device);
      } else {
        flux_ = std::make_unique<flux::Subscription>(
          node, topic, Adapter::kFingerprint, q, [sink](flux::FrameView && f) {
            (*sink)(std::move(f), [](const flux::FrameView & g) {
              return View<MsgT>(typename Adapter::View(g));
            });
          });
      }
      // Views kept past the callback hold slots, and the publisher needs one to write into.
      flux_->require_publisher_slots(options.max_borrow + 1);
      flux_->add_to_callback_group(node, options.callback_group);
      open_gpu(node, topic, q, std::move(sink), options, device);
      return;
    }
#else
    if (flux_requested()) {
      RCLCPP_WARN_ONCE(
        node.get_logger(), "ENABLE_FLUX=1, but flux_wrapper was built without flux: ROS path");
    }
#endif
    rclcpp::SubscriptionOptions ros_options;
    ros_options.callback_group = options.callback_group;
#ifdef FLUX_WRAPPER_CUDA
    if (cuda) {
      std::shared_ptr<detail::StagingPool> pool = detail::StagingPool::create(options.max_borrow);
      if (pool) {
        ros_ = rclcpp::create_subscription<MsgT>(
          node, topic, qos,
          [sink, pool, logger = node.get_logger(), topic,
           logged =
             std::make_shared<std::atomic<bool>>(false)](const typename MsgT::ConstSharedPtr & m) {
            detail::StagingPool::Lease lease = pool->upload(m->data.data(), m->data.size());
            if (!lease) {
              if (!logged->exchange(true, std::memory_order_relaxed)) {
                RCLCPP_ERROR(
                  logger,
                  "%s: Device::Cuda: a message could not be copied to the device, its max_borrow "
                  "buffers held or the copy failed, and was skipped; later ones are not logged",
                  topic.c_str());
              }
              return;
            }
            (*sink)(
              detail::StagedMessage<MsgT>{m, std::move(lease)},
              [](const detail::StagedMessage<MsgT> & s) {
                return View<MsgT>(*s.msg, s.lease.device(), s.lease.stream());
              });
          },
          ros_options);
        return;
      }
      RCLCPP_WARN(
        node.get_logger(), "%s: Device::Cuda: no CUDA stream could be created; using Host",
        topic.c_str());
    }
#endif
    ros_ = rclcpp::create_subscription<MsgT>(
      node, topic, qos,
      [sink](const typename MsgT::ConstSharedPtr & m) {
        (*sink)(m, [](const typename MsgT::ConstSharedPtr & p) { return View<MsgT>(*p); });
      },
      ros_options);
  }

#ifdef USE_FLUX_ENABLED
  struct Rejects
  {
    Rejects(rclcpp::Logger l, std::string t) : logger(std::move(l)), topic(std::move(t)) {}

    rclcpp::Logger logger;
    std::string topic;
    std::atomic<std::uint64_t> count{0};
  };

  template <class NodeT>
  void open_gpu(
    NodeT & node, const std::string & topic, const flux::QoS & q, SinkPtr sink,
    const SubscriptionOptions & options, flux::Device device)
  {
    rejects_ = std::make_shared<Rejects>(node.get_logger(), topic);
    auto on_frame = [sink = std::move(sink), r = rejects_](flux::FrameView && f) {
      typename detail::Gpu<MsgT>::Fields fields;
      if (!detail::decode_gpu(f.aux(), fields)) {
        if (r->count.fetch_add(1, std::memory_order_relaxed) == 0) {
          RCLCPP_ERROR(
            r->logger,
            "%s: skipped a GPU frame with a bad aux header (%zu bytes); later ones are counted "
            "in rejected(), not logged",
            r->topic.c_str(), f.aux().size());
        }
        return;
      }
      (*sink)(std::move(f), [&fields](const flux::FrameView & g) { return View<MsgT>(g, fields); });
    };
    gpu_ = std::make_unique<flux::Subscription>(
      node, topic, detail::Gpu<MsgT>::kFingerprint, q, std::move(on_frame), device);
    gpu_->require_publisher_slots(options.max_borrow + 1);
    gpu_->add_to_callback_group(node, options.callback_group);
  }

  std::unique_ptr<flux::Subscription> flux_;
  std::shared_ptr<Rejects> rejects_;
  std::unique_ptr<flux::Subscription> gpu_;
#endif
  typename rclcpp::Subscription<MsgT>::SharedPtr ros_;
};

}  // namespace flux_wrapper

#endif  // FLUX_WRAPPER_SUBSCRIPTION_HPP
