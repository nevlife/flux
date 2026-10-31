#ifndef FLUX_DETAIL_MANIFEST_HPP
#define FLUX_DETAIL_MANIFEST_HPP

#include "flux.h"

#include <memory>
#include <string>

namespace flux::detail
{

// Lists this endpoint for `flux topic` until destroyed. Listing only: the data path does not
// read it, so a full roster leaves the endpoint unlisted rather than failing it.
class Announcement
{
public:
  Announcement() = default;
  Announcement(
    const std::string & signpost, const std::string & key, const std::string & label,
    bool publisher)
  {
    const flux_manifest_entry e{signpost.c_str(), key.c_str(), label.c_str(), publisher};
    flux_announcement * a = nullptr;
    if (flux_announce(&e, &a) == FLUX_OK) {
      a_.reset(a);
    }
  }

private:
  struct Free
  {
    void operator()(flux_announcement * a) const noexcept { flux_announcement_free(a); }
  };
  std::unique_ptr<flux_announcement, Free> a_;
};

}  // namespace flux::detail

#endif  // FLUX_DETAIL_MANIFEST_HPP
