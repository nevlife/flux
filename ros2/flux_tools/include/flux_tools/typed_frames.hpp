#ifndef FLUX_TOOLS_TYPED_FRAMES_HPP
#define FLUX_TOOLS_TYPED_FRAMES_HPP

#include "flux/schemas.hpp"
#include "flux/subscription.hpp"
#include "flux/topics.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace flux_tools
{

// The frames of one type's channels, read as that type: its own fingerprint, or its GPU memory
// channel (flux::gpu_fingerprint), whose frame is the aux and the payload made one.
class TypedFrames
{
public:
  explicit TypedFrames(std::uint64_t fingerprint) : fingerprint_(fingerprint) {}

  // A channel of this type, either way.
  [[nodiscard]] auto ours(std::uint64_t fingerprint) const -> bool
  {
    return fingerprint == fingerprint_ || fingerprint == flux::gpu_fingerprint(fingerprint_);
  }

  // The fingerprint to subscribe `topic` with: the GPU memory channel's when only that one has
  // this name.
  auto fingerprint_for(const std::string & topic) -> std::uint64_t
  {
    gpu_ = false;
    for (const flux::Topic & t : flux::enumerate_topics()) {
      if (t.key == topic && t.fingerprint == fingerprint_) {
        gpu_ = false;
        break;
      }
      if (t.key == topic && t.fingerprint == flux::gpu_fingerprint(fingerprint_)) {
        gpu_ = true;
      }
    }
    return gpu_ ? flux::gpu_fingerprint(fingerprint_) : fingerprint_;
  }

  // `frame` of the channel fingerprint_for() named, as View. A GPU memory channel's View aliases
  // this object until the next call. An empty View (ok__() false at its first read) for a frame
  // that does not match.
  template <class View>
  auto view(const flux::FrameView & frame) -> View
  {
    if (!gpu_) {
      return View(frame);
    }
    try {
      if (!type_) {
        // The installed types, read on the first GPU memory channel's frame; type_ points into
        // them.
        schemas_.emplace();
        const std::uint64_t gpu = flux::gpu_fingerprint(fingerprint_);
        if (!schemas_->contains_gpu(gpu)) {
          return View();
        }
        type_.emplace(schemas_->at_gpu(gpu));
      }
      type_->with_data(frame, joined_);
    } catch (const std::exception &) {
      return View();
    }
    return View(joined_.data(), joined_.size());
  }

private:
  std::uint64_t fingerprint_;
  bool gpu_ = false;
  std::optional<flux::Schemas> schemas_;
  std::optional<flux::Schemas::Type> type_;
  std::vector<std::uint8_t> joined_;
};

}  // namespace flux_tools

#endif  // FLUX_TOOLS_TYPED_FRAMES_HPP
