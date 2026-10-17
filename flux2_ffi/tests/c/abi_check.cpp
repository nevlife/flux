// The C++ wrapper's first channel asks the library for its ABI version. Built once with the
// committed header (argv[1] "same") and once with a copy that claims another version ("other").
#include "flux2/channel.hpp"

#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char ** argv)
{
  const bool same = argc > 1 && std::strcmp(argv[1], "same") == 0;
  try {
    const flux2::detail::Channel ch(64, 4);
    return same ? 0 : 1;
  } catch (const flux2::Error & e) {
    std::fprintf(stderr, "%s\n", e.what());
    const bool named = std::string(e.what()).find("ABI version") != std::string::npos;
    return !same && e.status() == FLUX2_MISMATCH && named ? 0 : 2;
  }
}
