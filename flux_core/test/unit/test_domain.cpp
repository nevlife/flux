#include "flux/discovery.hpp"

#include <gtest/gtest.h>
#include <stdlib.h>

#include <string>

// The domain axis. Two domains never share a name, and one input never
// canonicalizes two ways -- a name built from raw environment text and a key built from a parsed
// value are how an isolation axis silently splits.

namespace
{

constexpr std::uint64_t kFp = 0xfeedfacecafebeefULL;

class ScopedEnv
{
public:
  ScopedEnv(const char * name, const char * value) : name_(name)
  {
    const char * old = ::getenv(name);
    had_ = old != nullptr;
    if (had_) old_ = old;
    if (value == nullptr) {
      ::unsetenv(name);
    } else {
      ::setenv(name, value, 1);
    }
  }
  ~ScopedEnv()
  {
    if (had_) {
      ::setenv(name_.c_str(), old_.c_str(), 1);
    } else {
      ::unsetenv(name_.c_str());
    }
  }
  ScopedEnv(const ScopedEnv &) = delete;
  ScopedEnv & operator=(const ScopedEnv &) = delete;

private:
  std::string name_;
  std::string old_;
  bool had_ = false;
};

}  // namespace

TEST(DomainTest, CanonicalRendersTheParsedInteger)
{
  EXPECT_EQ(flux::canonical_domain("0"), "0");
  EXPECT_EQ(flux::canonical_domain("7"), "7");
  EXPECT_EQ(flux::canonical_domain("007"), "7");
  EXPECT_EQ(flux::canonical_domain("4294967295"), "4294967295");
}

TEST(DomainTest, CanonicalRefusesAnythingButAPlainInteger)
{
  for (const char * bad :
       {"", "lab", "prodA1", "7x", "a/b", "-1", "+1", " 7", "7 ", "4294967296"}) {
    EXPECT_THROW(flux::canonical_domain(bad), std::invalid_argument) << bad;
  }
}

// The bug this axis is built to avoid: unset and the explicit default must be one domain, not two.
// A name that elides the component when it is default gives that one state two spellings.
// Resolved rather than latched on purpose: the latch would make every line here the same value
// and prove nothing. What is under test is the rule, and process_domain() runs it once.
TEST(DomainTest, UnsetAndExplicitDefaultAreOneDomain)
{
  const char * const kEnv = flux::kDefaultDomainEnv;
  const ScopedEnv no_label("FLUX_DOMAIN", nullptr);
  const ScopedEnv no_domain("ROS_DOMAIN_ID", nullptr);
  const std::string implicit = flux::segment_name("/t", kFp, flux::resolve_domain(kEnv));

  const ScopedEnv explicit_zero("ROS_DOMAIN_ID", "0");
  EXPECT_EQ(flux::segment_name("/t", kFp, flux::resolve_domain(kEnv)), implicit);

  const ScopedEnv empty_domain("ROS_DOMAIN_ID", "");
  EXPECT_EQ(flux::segment_name("/t", kFp, flux::resolve_domain(kEnv)), implicit);

  const ScopedEnv label_zero("FLUX_DOMAIN", "0");
  EXPECT_EQ(flux::segment_name("/t", kFp, flux::resolve_domain(kEnv)), implicit);
}

// The other half of the same bug: a domain written two ways is one domain.
TEST(DomainTest, PartitionIsRenderedFromTheParsedNumber)
{
  const char * const kEnv = flux::kDefaultDomainEnv;
  const ScopedEnv no_label("FLUX_DOMAIN", nullptr);
  const ScopedEnv seven("ROS_DOMAIN_ID", "7");
  const std::string canonical = flux::segment_name("/t", kFp, flux::resolve_domain(kEnv));

  const ScopedEnv padded("ROS_DOMAIN_ID", "007");
  EXPECT_EQ(flux::segment_name("/t", kFp, flux::resolve_domain(kEnv)), canonical);
  EXPECT_EQ(flux::resolve_domain(kEnv), "7");
}

TEST(DomainTest, FluxDomainWinsOverPartitionAndIsRenderedTheSameWay)
{
  const ScopedEnv label("FLUX_DOMAIN", "012");
  const ScopedEnv domain("ROS_DOMAIN_ID", "7");
  EXPECT_EQ(flux::resolve_domain("ROS_DOMAIN_ID"), "12");
}

TEST(DomainTest, UnusablePartitionThrowsRatherThanJoiningTheDefault)
{
  const ScopedEnv no_label("FLUX_DOMAIN", nullptr);
  for (const char * bad : {"abc", "7x", "-1", "+1", "99999999999", "1 "}) {
    const ScopedEnv domain("ROS_DOMAIN_ID", bad);
    EXPECT_THROW(flux::resolve_domain("ROS_DOMAIN_ID"), std::invalid_argument) << bad;
  }
}

TEST(DomainTest, UnusableFluxDomainThrowsRatherThanJoiningTheDefault)
{
  for (const char * bad : {"a/b", "lab"}) {
    const ScopedEnv label("FLUX_DOMAIN", bad);
    EXPECT_THROW(flux::resolve_domain(nullptr), std::invalid_argument) << bad;
    EXPECT_THROW(flux::resolve_domain("ROS_DOMAIN_ID"), std::invalid_argument) << bad;
  }
}

// A default that read nothing put a core caller in domain "0" while a ROS
// node on the same host was in "7", and neither end reported anything: they simply never met.
TEST(DomainTest, CoreReadsThePartitionVariableItNames)
{
  const ScopedEnv no_label("FLUX_DOMAIN", nullptr);
  const ScopedEnv domain("ROS_DOMAIN_ID", "7");
  EXPECT_EQ(flux::resolve_domain(flux::kDefaultDomainEnv), "7");
  EXPECT_EQ(flux::resolve_domain(), "7") << "the default variable is ROS_DOMAIN_ID, as in Python";
}

// The opt-out survives: a caller with no host domain to inherit says so.
TEST(DomainTest, NullPartitionEnvTakesTheDefaultDomain)
{
  const ScopedEnv no_label("FLUX_DOMAIN", nullptr);
  const ScopedEnv domain("ROS_DOMAIN_ID", "7");
  EXPECT_EQ(flux::resolve_domain(nullptr), flux::kDefaultDomain);
}

// rcl latches ROS_DOMAIN_ID at context init, so a node created after a setenv keeps the
// domain it started with. process_domain() latches the same way: reading it per endpoint let one
// process build names in two domains while its ROS half stayed in one domain.
//
// Every name in this process already carries the latched value, so this asserts stability across
// an environment change rather than a particular string -- the value depends on the environment
// the test binary was launched with, and pinning it here would make the test depend on that.
TEST(DomainTest, ProcessDomainIsLatchedForTheLifeOfTheProcess)
{
  const std::string first = flux::process_domain();
  {
    const ScopedEnv label("FLUX_DOMAIN", "4242");
    EXPECT_EQ(flux::process_domain(), first) << "process_domain re-read the environment";
    // resolve_domain is the query and does follow the change: the two are different questions.
    EXPECT_EQ(flux::resolve_domain(flux::kDefaultDomainEnv), "4242");
  }
  EXPECT_EQ(flux::process_domain(), first);
  // The names a process builds are what the latch is for.
  EXPECT_EQ(flux::segment_name("/t", kFp), flux::segment_name("/t", kFp, first));
}

TEST(DomainTest, DistinctDomainsNeverShareAName)
{
  EXPECT_NE(flux::segment_name("/t", kFp, "0"), flux::segment_name("/t", kFp, "1"));
  EXPECT_NE(flux::signpost_name("/t", kFp, "0"), flux::signpost_name("/t", kFp, "1"));
}

TEST(DomainTest, NameCarriesTheDomainAndStaysAValidShmName)
{
  const std::string name = flux::segment_name("/camera/image_raw", kFp, "7");
  EXPECT_EQ(name.front(), '/');
  EXPECT_EQ(name.find('/', 1), std::string::npos);
  EXPECT_LE(name.size(), 255u);
  EXPECT_NE(name.find(".s7."), std::string::npos);
}

TEST(DomainTest, SegmentNameRefusesAnUnvalidatedDomain)
{
  EXPECT_THROW(flux::segment_name("/t", kFp, "a/b"), std::invalid_argument);
  EXPECT_THROW(flux::segment_name("/t", kFp, ""), std::invalid_argument);
}

// A name is never cut: cutting drops the part that tells two keys or two schemas apart. So a key
// is refused past the length whose longest name (widest domain, pid and starttime) still fits.
TEST(DomainTest, AKeyPastTheLimitIsRefused)
{
  EXPECT_NO_THROW(flux::segment_name(std::string(185, 'k'), kFp));
  EXPECT_THROW(flux::segment_name(std::string(186, 'k'), kFp), std::invalid_argument);
  EXPECT_THROW(flux::signpost_name(std::string(186, 'k'), kFp), std::invalid_argument);
}

TEST(DomainTest, TheLongestNamesStayWholeAndDistinct)
{
  const std::string key(185, 'k');
  flux::OwnerId widest;
  widest.pid = 0xFFFFFFFFu;
  widest.starttime = 0xFFFFFFFFFFFFFFFFull;
  const std::string sp = flux::signpost_name(key, kFp, "4294967295");
  const std::string seg = flux::unique_segment_name(sp, widest);
  EXPECT_LE(seg.size(), 255u);
  EXPECT_EQ(seg.rfind(sp, 0), 0u);
  EXPECT_NE(
    seg, flux::unique_segment_name(flux::signpost_name(key, kFp ^ 1, "4294967295"), widest));
}
