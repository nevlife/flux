#ifndef FLUX_TOPICS_HPP
#define FLUX_TOPICS_HPP

#include "flux/channel.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace flux
{

using ChannelStats = flux_channel_stats;

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
inline auto enumerate_topics() -> std::vector<Topic>
{
  struct Free
  {
    void operator()(flux_topic_list * l) const noexcept { flux_topic_list_free(l); }
  };
  detail::require_abi();
  const std::unique_ptr<flux_topic_list, Free> list(flux_enumerate_topics());
  std::vector<Topic> out;
  const std::size_t n = flux_topic_list_size(list.get());
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const flux_topic_info & t = *flux_topic_list_get(list.get(), i);
    Topic topic{t.signpost, t.key, t.fingerprint, t.key_exact, {}};
    for (std::size_t j = 0; j < t.endpoint_count; ++j) {
      const flux_endpoint_info & e = t.endpoints[j];  // NOLINT(*-pointer-arithmetic)
      topic.endpoints.push_back({e.pid, e.publisher, e.label});
    }
    out.push_back(std::move(topic));
  }
  return out;
}

// A missing or unreadable channel reads as not live.
inline auto read_channel_stats(const std::string & signpost) -> ChannelStats
{
  detail::require_abi();
  ChannelStats s{};
  detail::check(flux_read_channel_stats(signpost.c_str(), &s));
  return s;
}

}  // namespace flux

#endif  // FLUX_TOPICS_HPP
