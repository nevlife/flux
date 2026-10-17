// Preloaded by test_trace: logs the trace events the Python binding emits, and commit and take for
// their stamps, one line each to the file FLUX2_TRACE_LOG names.
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace
{
std::FILE * log_file()
{
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): open until the process exits.
  static std::FILE * f = std::fopen(std::getenv("FLUX2_TRACE_LOG"), "w");
  return f;
}

template <class... A>
void line(const char * fmt, A... a)
{
  static std::mutex m;
  const std::lock_guard<std::mutex> l(m);
  std::fprintf(log_file(), fmt, a...);  // NOLINT(cppcoreguidelines-pro-type-vararg)
  std::fflush(log_file());
}
}  // namespace

extern "C" {

void flux2_trace_endpoint_init(
  const void * endpoint, const char * node, const char * key, const char * signpost,
  std::uint64_t fingerprint, std::int32_t role, std::int32_t lang)
{
  line(
    "endpoint_init %p %s %s %s %" PRIu64 " %d %d\n", endpoint, node, key, signpost, fingerprint,
    role, lang);
}

void flux2_trace_py_gil(std::int32_t phase)
{
  line("py_gil %d\n", phase);
}

void flux2_trace_py_ready(const void * channel, std::uint64_t ticket)
{
  line("py_ready %p %" PRIu64 "\n", channel, ticket);
}

void flux2_trace_commit(
  const void * channel, std::uint64_t ticket, std::uint32_t, std::uint64_t, std::int32_t,
  std::uint64_t stamp)
{
  line("commit %p %" PRIu64 " %" PRIu64 "\n", channel, ticket, stamp);
}

void flux2_trace_take(
  const void * channel, std::uint64_t ticket, std::uint32_t, std::uint64_t, std::uint64_t stamp)
{
  line("take %p %" PRIu64 " %" PRIu64 "\n", channel, ticket, stamp);
}
}
