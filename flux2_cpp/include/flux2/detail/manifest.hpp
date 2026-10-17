#ifndef FLUX2_DETAIL_MANIFEST_HPP
#define FLUX2_DETAIL_MANIFEST_HPP

#include "flux2.h"

#include <memory>
#include <string>

namespace flux2::detail
{

// Lists this endpoint for `flux2 topic` until destroyed. Listing only: the data path does not
// read it, so a full roster leaves the endpoint unlisted rather than failing it.
class Announcement
{
public:
  Announcement() = default;
  Announcement(
    const std::string & signpost, const std::string & key, const std::string & label,
    bool publisher)
  {
    const flux2_manifest_entry e{signpost.c_str(), key.c_str(), label.c_str(), publisher};
    flux2_announcement * a = nullptr;
    if (flux2_announce(&e, &a) == FLUX2_OK) a_.reset(a);
  }

private:
  struct Free
  {
    void operator()(flux2_announcement * a) const noexcept { flux2_announcement_free(a); }
  };
  std::unique_ptr<flux2_announcement, Free> a_;
};

}  // namespace flux2::detail

#endif  // FLUX2_DETAIL_MANIFEST_HPP
