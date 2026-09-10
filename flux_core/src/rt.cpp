#include "flux/rt.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>

#if defined(__linux__)
#include <dirent.h>
#include <fcntl.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace flux::rt
{

namespace
{

// Matches glibc CPU_SETSIZE; validate() must not depend on <sched.h> being present.
constexpr std::uint32_t kMaxCpus = 1024;

[[noreturn]] void fail(const std::string & why)
{
  throw std::invalid_argument("flux: " + why);
}

bool is_rt_policy(Policy p)
{
  return p == Policy::Fifo || p == Policy::RoundRobin;
}

const char * policy_name(Policy p)
{
  switch (p) {
    case Policy::Inherit:
      return "inherit";
    case Policy::Other:
      return "SCHED_OTHER";
    case Policy::Fifo:
      return "SCHED_FIFO";
    case Policy::RoundRobin:
      return "SCHED_RR";
  }
  return "?";
}

#if defined(__linux__)

static_assert(kMaxCpus == CPU_SETSIZE, "cpu id validation must match the kernel set size");

std::optional<std::string> read_file(const char * path)
{
  int fd = ::open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return std::nullopt;
  std::string out;
  char buf[4096];
  ssize_t n;
  while ((n = ::read(fd, buf, sizeof buf)) > 0) out.append(buf, static_cast<std::size_t>(n));
  ::close(fd);
  if (n < 0) return std::nullopt;
  return out;
}

std::string trimmed(std::string s)
{
  while (!s.empty() && (s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
  std::size_t i = 0;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
  return s.substr(i);
}

// The /sys cpu list format: "0-3,5,8-9". A blank file is an empty list.
std::vector<std::uint32_t> parse_cpu_list(const std::string & text)
{
  std::vector<std::uint32_t> out;
  const std::string t = trimmed(text);
  std::size_t pos = 0;
  while (pos < t.size()) {
    char * end = nullptr;
    const unsigned long first = std::strtoul(t.c_str() + pos, &end, 10);
    if (end == t.c_str() + pos) break;
    unsigned long last = first;
    pos = static_cast<std::size_t>(end - t.c_str());
    if (pos < t.size() && t[pos] == '-') {
      last = std::strtoul(t.c_str() + pos + 1, &end, 10);
      pos = static_cast<std::size_t>(end - t.c_str());
    }
    for (unsigned long c = first; c <= last; ++c) out.push_back(static_cast<std::uint32_t>(c));
    if (pos < t.size() && t[pos] == ',') ++pos;
  }
  return out;
}

bool contains(const std::vector<std::uint32_t> & set, std::uint32_t cpu)
{
  for (auto c : set) {
    if (c == cpu) return true;
  }
  return false;
}

std::optional<std::string> proc_status_field(const char * key)
{
  const auto status = read_file("/proc/self/status");
  if (!status) return std::nullopt;
  const std::string needle = std::string(key) + ":";
  std::size_t pos = status->find(needle);
  while (pos != 0 && pos != std::string::npos && (*status)[pos - 1] != '\n') {
    pos = status->find(needle, pos + 1);
  }
  if (pos == std::string::npos) return std::nullopt;
  const std::size_t begin = pos + needle.size();
  const std::size_t end = status->find('\n', begin);
  return trimmed(status->substr(begin, end - begin));
}

// CAP_SYS_NICE is capability 23 (linux/capability.h); parsed from CapEff so flux_core
// stays free of libcap.
std::optional<bool> has_cap_sys_nice()
{
  const auto cap_eff = proc_status_field("CapEff");
  if (!cap_eff) return std::nullopt;
  const std::uint64_t bits = std::strtoull(cap_eff->c_str(), nullptr, 16);
  return (bits >> 23) & 1;
}

int native_policy(Policy p)
{
  switch (p) {
    case Policy::Other:
      return SCHED_OTHER;
    case Policy::Fifo:
      return SCHED_FIFO;
    case Policy::RoundRobin:
      return SCHED_RR;
    case Policy::Inherit:
      break;
  }
  return -1;
}

void check_policy_permission(Report & rep, const Options & opts)
{
  rlimit rl{};
  const bool have_rlimit = ::getrlimit(RLIMIT_RTPRIO, &rl) == 0;
  const bool rlimit_covers = have_rlimit && (rl.rlim_cur == RLIM_INFINITY ||
                                             rl.rlim_cur >= static_cast<rlim_t>(opts.priority));
  const auto cap = has_cap_sys_nice();
  if (cap.value_or(false)) {
    rep.findings.push_back({"policy-permission", Verdict::Ok, "CAP_SYS_NICE is in CapEff"});
  } else if (rlimit_covers) {
    rep.findings.push_back(
      {"policy-permission", Verdict::Ok,
       "RLIMIT_RTPRIO soft limit " + std::to_string(rl.rlim_cur) + " covers requested " +
         std::to_string(opts.priority)});
  } else if (!cap.has_value()) {
    rep.findings.push_back(
      {"policy-permission", Verdict::Unknown, "cannot read CapEff from /proc/self/status"});
  } else {
    rep.findings.push_back(
      {"policy-permission", Verdict::Fail,
       "no CAP_SYS_NICE and RLIMIT_RTPRIO soft limit " +
         (have_rlimit ? std::to_string(rl.rlim_cur) : std::string("unreadable")) + " < requested " +
         std::to_string(opts.priority) +
         ": raise the rtprio ulimit (limits.conf) or grant CAP_SYS_NICE"});
  }
}

void check_rt_throttle(Report & rep)
{
  const auto runtime = read_file("/proc/sys/kernel/sched_rt_runtime_us");
  if (!runtime) {
    rep.findings.push_back(
      {"rt-throttle", Verdict::Unknown, "cannot read /proc/sys/kernel/sched_rt_runtime_us"});
    return;
  }
  const long long rt_us = std::strtoll(runtime->c_str(), nullptr, 10);
  if (rt_us < 0) {
    rep.findings.push_back({"rt-throttle", Verdict::Ok, "sched_rt_runtime_us=-1: throttling off"});
    return;
  }
  const auto period = read_file("/proc/sys/kernel/sched_rt_period_us");
  const std::string period_s = period ? trimmed(*period) : std::string("1000000");
  rep.findings.push_back(
    {"rt-throttle", Verdict::Warn,
     "sched_rt_runtime_us=" + std::to_string(rt_us) + " of period " + period_s +
       ": the kernel takes the CPU from RT tasks for the rest of every period, so a busy RT "
       "thread is preempted regardless of priority; hard RT wants sched_rt_runtime_us=-1"});
}

void check_priority_order(Report & rep, const Options & opts, int control_priority)
{
  if (opts.priority < control_priority) {
    rep.findings.push_back(
      {"priority-order", Verdict::Ok,
       "transport " + std::to_string(opts.priority) + " stays below control loop " +
         std::to_string(control_priority)});
  } else {
    rep.findings.push_back(
      {"priority-order", Verdict::Fail,
       "transport priority " + std::to_string(opts.priority) + " >= control loop " +
         std::to_string(control_priority) +
         ": transport must never preempt or starve control; set it strictly lower"});
  }
}

// Value of one boot parameter, or nullopt when the line does not carry it.
std::optional<std::string> boot_param(const std::string & cmdline, const std::string & key)
{
  const std::string needle = key + "=";
  std::size_t pos = cmdline.find(needle);
  while (pos != std::string::npos) {
    if (pos == 0 || cmdline[pos - 1] == ' ') {
      const std::size_t start = pos + needle.size();
      const std::size_t end = cmdline.find_first_of(" \n", start);
      return cmdline.substr(start, end == std::string::npos ? std::string::npos : end - start);
    }
    pos = cmdline.find(needle, pos + 1);
  }
  return std::nullopt;
}

// RCU defers freeing memory until every reader is done, and those deferred callbacks run on the
// cpu that queued them unless the cpu is offloaded. On an RT core that is an interruption of
// unpredictable length and timing, and it is the one isolation axis with no sysfs to read: the
// boot line is the source. nohz_full implies it -- the kernel offloads every nohz_full cpu,
// because a cpu with no tick cannot be the one servicing its own callbacks.
void check_rcu_offload(Report & rep, const Options & opts)
{
  const auto cmdline = read_file("/proc/cmdline");
  if (!cmdline) {
    rep.findings.push_back({"rcu-offload", Verdict::Unknown, "cannot read /proc/cmdline"});
    return;
  }
  const auto nocbs = boot_param(*cmdline, "rcu_nocbs");
  const auto nohz_file = read_file("/sys/devices/system/cpu/nohz_full");
  const auto offloaded = nocbs ? parse_cpu_list(*nocbs) : std::vector<std::uint32_t>{};
  const auto nohz = nohz_file ? parse_cpu_list(*nohz_file) : std::vector<std::uint32_t>{};

  std::string onboard;
  for (auto c : opts.cpus) {
    if (!contains(offloaded, c) && !contains(nohz, c)) {
      onboard += (onboard.empty() ? "" : ",") + std::to_string(c);
    }
  }
  if (onboard.empty()) {
    rep.findings.push_back({"rcu-offload", Verdict::Ok, "requested cpus offload RCU callbacks"});
    return;
  }
  rep.findings.push_back(
    {"rcu-offload", Verdict::Warn,
     "cpu " + onboard +
       " runs its own RCU callbacks: deferred kernel frees fire on the RT core at times nothing "
       "bounds. Add rcu_nocbs (or nohz_full, which implies it) for these cpus (rcu_nocbs: " +
       (nocbs ? *nocbs : std::string("unset")) + ")"});
}

// The handler name lives as a subdirectory of /proc/irq/<n>, so a caller can say which device
// an interrupt belongs to rather than only its number.
std::string irq_handler_name(const std::string & dir)
{
  DIR * d = ::opendir(dir.c_str());
  if (d == nullptr) return {};
  std::string name;
  while (const dirent * e = ::readdir(d)) {
    const std::string entry = e->d_name;
    if (entry == "." || entry == "..") continue;
    if (e->d_type == DT_DIR) {
      name = entry;
      break;
    }
  }
  ::closedir(d);
  return name;
}

// An interrupt handler preempts every task on its cpu, SCHED_FIFO 99 included -- interrupts are
// outside the priority system, not at the top of it. So an isolated core that still receives
// interrupts is not isolated for latency purposes, and this is usually the largest remaining
// source once isolcpus and nohz_full are set: one USB controller can raise tens of thousands.
void check_irq_affinity(Report & rep, const Options & opts)
{
  DIR * d = ::opendir("/proc/irq");
  if (d == nullptr) {
    rep.findings.push_back({"irq-affinity", Verdict::Unknown, "cannot read /proc/irq"});
    return;
  }
  int hits = 0;
  // Named handlers first: "irq 131 xhci_hcd" tells the reader which device to steer, while a
  // legacy numbered one says nothing beyond its number.
  std::vector<std::string> named;
  std::vector<std::string> bare;
  while (const dirent * e = ::readdir(d)) {
    const std::string irq = e->d_name;
    if (irq.empty() || !std::isdigit(static_cast<unsigned char>(irq[0]))) continue;
    const std::string dir = "/proc/irq/" + irq;
    // effective_affinity_list is where the interrupt actually lands; smp_affinity_list is only
    // where it may. Prefer the first and fall back when the kernel does not expose it.
    auto list = read_file((dir + "/effective_affinity_list").c_str());
    if (!list || trimmed(*list).empty()) list = read_file((dir + "/smp_affinity_list").c_str());
    if (!list) continue;
    const auto cpus = parse_cpu_list(*list);
    bool on_rt = false;
    for (auto c : opts.cpus) {
      if (contains(cpus, c)) on_rt = true;
    }
    if (!on_rt) continue;
    ++hits;
    const std::string name = irq_handler_name(dir);
    (name.empty() ? bare : named).push_back("irq " + irq + (name.empty() ? "" : " " + name));
  }
  ::closedir(d);

  if (hits == 0) {
    rep.findings.push_back(
      {"irq-affinity", Verdict::Ok, "no interrupt is routed to the requested cpus"});
    return;
  }
  named.insert(named.end(), bare.begin(), bare.end());
  std::string examples;
  for (std::size_t i = 0; i < named.size() && i < 3; ++i) {
    examples += (examples.empty() ? "" : ", ") + named[i];
  }
  rep.findings.push_back(
    {"irq-affinity", Verdict::Warn,
     std::to_string(hits) + " interrupt(s) can land on the requested cpus (" + examples +
       (hits > 3 ? ", ..." : "") +
       "): a handler preempts every task on its cpu regardless of RT priority. Steer them away "
       "with /proc/irq/<n>/smp_affinity_list (and stop irqbalance from putting them back)"});
}

void check_cpus(Report & rep, const Options & opts)
{
  const auto online_file = read_file("/sys/devices/system/cpu/online");
  if (!online_file) {
    rep.findings.push_back(
      {"cpu-online", Verdict::Unknown, "cannot read /sys/devices/system/cpu/online"});
  } else {
    const auto online = parse_cpu_list(*online_file);
    std::string offline;
    for (auto c : opts.cpus) {
      if (!contains(online, c)) offline += (offline.empty() ? "" : ",") + std::to_string(c);
    }
    if (offline.empty()) {
      rep.findings.push_back({"cpu-online", Verdict::Ok, "requested cpus online"});
    } else {
      rep.findings.push_back(
        {"cpu-online", Verdict::Fail,
         "cpu " + offline + " not online (online: " + trimmed(*online_file) + ")"});
    }
  }

  const auto isolated_file = read_file("/sys/devices/system/cpu/isolated");
  const auto nohz_file = read_file("/sys/devices/system/cpu/nohz_full");
  if (!isolated_file) {
    rep.findings.push_back(
      {"cpu-isolation", Verdict::Unknown, "cannot read /sys/devices/system/cpu/isolated"});
  } else {
    const auto isolated = parse_cpu_list(*isolated_file);
    std::string shared;
    for (auto c : opts.cpus) {
      if (!contains(isolated, c)) shared += (shared.empty() ? "" : ",") + std::to_string(c);
    }
    if (shared.empty()) {
      rep.findings.push_back(
        {"cpu-isolation", Verdict::Ok, "requested cpus are in isolcpus/isolated"});
    } else {
      rep.findings.push_back(
        {"cpu-isolation", Verdict::Warn,
         "cpu " + shared +
           " not isolated: pinning removes migration but the core still runs other tasks; "
           "isolate with isolcpus/nohz_full for exclusive use (isolated: " +
           trimmed(*isolated_file) + ", nohz_full: " + (nohz_file ? trimmed(*nohz_file) : "?") +
           ")"});
    }
  }

  std::string slow;
  bool no_cpufreq = false;
  for (auto c : opts.cpus) {
    const std::string path =
      "/sys/devices/system/cpu/cpu" + std::to_string(c) + "/cpufreq/scaling_governor";
    const auto governor = read_file(path.c_str());
    if (!governor) {
      no_cpufreq = true;
    } else if (trimmed(*governor) != "performance") {
      slow += (slow.empty() ? "" : ", ") + ("cpu" + std::to_string(c) + "=" + trimmed(*governor));
    }
  }
  if (!slow.empty()) {
    rep.findings.push_back(
      {"cpu-governor", Verdict::Warn,
       slow + ": frequency ramp adds wakeup latency; set performance on RT cores"});
  } else if (no_cpufreq) {
    rep.findings.push_back(
      {"cpu-governor", Verdict::Unknown, "no cpufreq on a requested cpu (VM or fixed clock)"});
  } else {
    rep.findings.push_back({"cpu-governor", Verdict::Ok, "governor performance on requested cpus"});
  }
}

void check_kernel_preemption(Report & rep)
{
  const auto realtime = read_file("/sys/kernel/realtime");
  const auto version = read_file("/proc/version");
  if (
    (realtime && trimmed(*realtime) == "1") ||
    (version && version->find("PREEMPT_RT") != std::string::npos)) {
    rep.findings.push_back({"kernel-preemption", Verdict::Ok, "PREEMPT_RT kernel"});
  } else if (version) {
    rep.findings.push_back(
      {"kernel-preemption", Verdict::Warn,
       "not a PREEMPT_RT kernel: in-kernel sections are not preemptible, so worst-case "
       "latency is unbounded by userspace scheduling alone"});
  } else {
    rep.findings.push_back({"kernel-preemption", Verdict::Unknown, "cannot read /proc/version"});
  }
}

void check_memory_lock(Report & rep)
{
  const auto vmlck = proc_status_field("VmLck");
  if (!vmlck) {
    rep.findings.push_back(
      {"memory-lock", Verdict::Unknown, "cannot read VmLck from /proc/self/status"});
    return;
  }
  const long long kb = std::strtoll(vmlck->c_str(), nullptr, 10);
  if (kb > 0) {
    rep.findings.push_back({"memory-lock", Verdict::Ok, "VmLck=" + trimmed(*vmlck)});
  } else {
    rep.findings.push_back(
      {"memory-lock", Verdict::Warn,
       "no locked memory: any page fault is an unbounded stall; lock and pre-fault the "
       "working set (segment pre-commit is a planned flux opt-in)"});
  }
}

#endif  // defined(__linux__)

}  // namespace

void Options::validate() const
{
  const bool rt = is_rt_policy(policy);
  if (rt && (priority < 1 || priority > 99)) {
    fail(
      std::string(policy_name(policy)) + " needs priority 1..99, got " + std::to_string(priority));
  }
  if (!rt && priority != 0) {
    fail(
      "priority " + std::to_string(priority) + " with policy " + policy_name(policy) +
      " would be ignored; set policy=Fifo or RoundRobin, or drop the priority");
  }
  for (auto c : cpus) {
    if (c >= kMaxCpus) {
      fail(
        "cpu id " + std::to_string(c) + " exceeds the kernel set size " + std::to_string(kMaxCpus));
    }
  }
}

void apply(const Options & opts)
{
  opts.validate();
  const bool want_policy = opts.policy != Policy::Inherit;
  const bool want_cpus = !opts.cpus.empty();
  if (!want_policy && !want_cpus) return;
#if !defined(__linux__)
  throw std::system_error(
    std::make_error_code(std::errc::function_not_supported), "flux: rt::apply targets Linux");
#else
  cpu_set_t old_set;
  if (want_cpus) {
    if (::sched_getaffinity(0, sizeof old_set, &old_set) != 0) {
      throw std::system_error(errno, std::generic_category(), "flux: sched_getaffinity");
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    for (auto c : opts.cpus) CPU_SET(static_cast<int>(c), &set);
    if (::sched_setaffinity(0, sizeof set, &set) != 0) {
      throw std::system_error(
        errno, std::generic_category(),
        "flux: sched_setaffinity refused the requested cpus; rt::preflight() explains this host");
    }
  }
  if (want_policy) {
    sched_param param{};
    param.sched_priority = opts.priority;
    if (::sched_setscheduler(0, native_policy(opts.policy), &param) != 0) {
      const int e = errno;
      if (want_cpus) ::sched_setaffinity(0, sizeof old_set, &old_set);
      throw std::system_error(
        e, std::generic_category(),
        "flux: sched_setscheduler(" + std::string(policy_name(opts.policy)) + ", " +
          std::to_string(opts.priority) + ") refused; rt::preflight() explains this host");
    }
  }
#endif
}

ThreadState current()
{
  ThreadState st;
#if defined(__linux__)
  st.policy = ::sched_getscheduler(0);
  sched_param param{};
  if (::sched_getparam(0, &param) == 0) st.priority = param.sched_priority;
  cpu_set_t set;
  if (::sched_getaffinity(0, sizeof set, &set) == 0) {
    for (int c = 0; c < CPU_SETSIZE; ++c) {
      if (CPU_ISSET(c, &set)) st.cpus.push_back(static_cast<std::uint32_t>(c));
    }
  }
#endif
  return st;
}

int this_tid()
{
#if !defined(__linux__)
  throw std::system_error(
    std::make_error_code(std::errc::function_not_supported), "flux: rt::this_tid targets Linux");
#else
  return static_cast<int>(::syscall(SYS_gettid));
#endif
}

ThreadState observe(int tid)
{
  // Not a convenience for self: sched_getscheduler reads the caller when handed 0, so an
  // uninitialized tid would answer about the wrong thread and look like a successful check.
  if (tid <= 0) fail("rt::observe needs a thread id; 0 would read the calling thread instead");
#if !defined(__linux__)
  throw std::system_error(
    std::make_error_code(std::errc::function_not_supported), "flux: rt::observe targets Linux");
#else
  const std::string who = "flux: rt::observe(" + std::to_string(tid) + ")";
  ThreadState st;
  errno = 0;
  st.policy = ::sched_getscheduler(tid);
  if (st.policy < 0) throw std::system_error(errno, std::generic_category(), who);
  sched_param param{};
  if (::sched_getparam(tid, &param) != 0) {
    throw std::system_error(errno, std::generic_category(), who + " sched_getparam");
  }
  st.priority = param.sched_priority;
  cpu_set_t set;
  if (::sched_getaffinity(tid, sizeof set, &set) != 0) {
    throw std::system_error(errno, std::generic_category(), who + " sched_getaffinity");
  }
  for (int c = 0; c < CPU_SETSIZE; ++c) {
    if (CPU_ISSET(c, &set)) st.cpus.push_back(static_cast<std::uint32_t>(c));
  }
  return st;
#endif
}

bool Report::ok() const
{
  for (const auto & f : findings) {
    if (f.verdict == Verdict::Fail) return false;
  }
  return true;
}

const Finding * Report::find(const char * id) const
{
  for (const auto & f : findings) {
    if (f.id == id) return &f;
  }
  return nullptr;
}

std::string Report::to_string() const
{
  static const char * words[] = {"ok", "warn", "fail", "unknown"};
  std::string out;
  for (const auto & f : findings) {
    out += f.id;
    out += ": ";
    out += words[static_cast<std::uint8_t>(f.verdict)];
    out += " - ";
    out += f.detail;
    out += '\n';
  }
  return out;
}

Report preflight(const Options & opts, int control_priority)
{
  opts.validate();
  Report rep;
#if !defined(__linux__)
  (void)control_priority;
  rep.findings.push_back({"platform", Verdict::Unknown, "flux rt targets Linux; nothing checked"});
#else
  if (is_rt_policy(opts.policy)) {
    check_policy_permission(rep, opts);
    check_rt_throttle(rep);
    if (control_priority > 0) {
      check_priority_order(rep, opts, control_priority);
    } else {
      // Not omitted: an absent finding reads the same as a passing one, and this is the check a
      // caller most needs to have run.
      rep.findings.push_back(
        {"priority-order", Verdict::Unknown,
         "control loop priority not declared; pass it to preflight() so transport can be checked "
         "to stay strictly below it"});
    }
  }
  if (!opts.cpus.empty()) {
    check_cpus(rep, opts);
    check_rcu_offload(rep, opts);
    check_irq_affinity(rep, opts);
  }
  check_kernel_preemption(rep);
  check_memory_lock(rep);
#endif
  return rep;
}

namespace
{
bool has_warn(const Report & rep)
{
  for (const auto & f : rep.findings) {
    if (f.verdict == Verdict::Warn) return true;
  }
  return false;
}
}  // namespace

Report apply_checked(const Options & opts, Strictness strict, int control_priority)
{
  opts.validate();
  // Same no-op contract as apply(): options that ask for nothing get nothing, including no
  // judgement. Preflighting here would report a host that no caller asked anything of.
  if (opts.policy == Policy::Inherit && opts.cpus.empty()) return Report{};

  Report rep = preflight(opts, control_priority);
  const bool blocked = !rep.ok() || (strict == Strictness::Hard && has_warn(rep));
  if (blocked) {
    throw std::runtime_error(
      std::string("flux: rt preflight refused this ") +
      (strict == Strictness::Hard ? "hard" : "soft") + "-RT request; nothing was applied\n" +
      rep.to_string());
  }
  apply(opts);

  // Confirm by reading back rather than by trusting the syscall's return. A request the
  // kernel accepted can still land elsewhere -- a cpuset narrows an affinity mask, and nothing
  // stops another thread from re-scheduling this one between the two calls. Trusting the return
  // is the same mistake as trusting apply() without preflight, one layer down.
#if defined(__linux__)
  const ThreadState got = current();
  if (opts.policy != Policy::Inherit) {
    if (got.policy != native_policy(opts.policy) || got.priority != opts.priority) {
      throw std::runtime_error(
        std::string("flux: rt::apply_checked read back policy ") + std::to_string(got.policy) +
        "/priority " + std::to_string(got.priority) + " after asking for " +
        policy_name(opts.policy) + "/" + std::to_string(opts.priority) +
        "; the thread is not running as declared");
    }
  }
  if (!opts.cpus.empty()) {
    for (auto c : got.cpus) {
      if (std::find(opts.cpus.begin(), opts.cpus.end(), c) == opts.cpus.end()) {
        throw std::runtime_error(
          "flux: rt::apply_checked read back cpu " + std::to_string(c) +
          ", which was not among the requested cpus; the affinity did not take");
      }
    }
    if (got.cpus.empty()) {
      throw std::runtime_error("flux: rt::apply_checked read back an empty cpu affinity");
    }
  }
#endif
  return rep;
}

}  // namespace flux::rt
