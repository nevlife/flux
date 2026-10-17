#ifndef FLUX2_TOPICS_HPP
#define FLUX2_TOPICS_HPP

#include "flux2/channel.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace flux2
{

using ChannelStats = flux2_channel_stats;

struct Endpoint
{
  std::uint32_t pid = 0;
  bool publisher = false;
  std::string label;
};

struct Topic
{
  std::string signpost;
  std::string key;
  std::uint64_t fingerprint = kNoSchema;
  // False: `key` is the flattened spelling read back from the signpost name.
  bool key_exact = false;
  std::vector<Endpoint> endpoints;
};

// Every channel on this host with a signpost, sorted by signpost.
inline std::vector<Topic> enumerate_topics()
{
  struct Free
  {
    void operator()(flux2_topic_list * l) const noexcept { flux2_topic_list_free(l); }
  };
  detail::require_abi();
  const std::unique_ptr<flux2_topic_list, Free> list(flux2_enumerate_topics());
  std::vector<Topic> out;
  const std::size_t n = flux2_topic_list_size(list.get());
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const flux2_topic_info & t = *flux2_topic_list_get(list.get(), i);
    Topic topic{t.signpost, t.key, t.fingerprint, t.key_exact, {}};
    for (std::size_t j = 0; j < t.endpoint_count; ++j) {
      const flux2_endpoint_info & e = t.endpoints[j];  // NOLINT(*-pointer-arithmetic)
      topic.endpoints.push_back({e.pid, e.publisher, e.label});
    }
    out.push_back(std::move(topic));
  }
  return out;
}

// A missing or unreadable channel reads as not live.
inline ChannelStats read_channel_stats(const std::string & signpost)
{
  detail::require_abi();
  ChannelStats s{};
  detail::check(flux2_read_channel_stats(signpost.c_str(), &s));
  return s;
}

}  // namespace flux2

#endif  // FLUX2_TOPICS_HPP
