#include "flux/flux.hpp"

#include <gtest/gtest.h>

// Smoke test: proves the build/link/test pipeline works independently of any engine logic.
TEST(FluxCoreSmoke, VersionIsLinked)
{
  EXPECT_STREQ(flux::version(), flux::kVersion);
  EXPECT_NE(flux::kVersion[0], '\0');
}
