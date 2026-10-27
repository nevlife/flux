#include "probe.hpp"

#include <unistd.h>

#include <mutex>

namespace probe
{

namespace
{

struct Record
{
  std::mutex m;
  std::vector<Event> all;
};

Record & record_of_process()
{
  static Record r;
  return r;
}

}  // namespace

void record(const std::string & what, const void * key, double value)
{
  Record & r = record_of_process();
  const std::lock_guard<std::mutex> lk(r.m);
  r.all.push_back({what, key, gettid(), r.all.size(), value});
}

std::vector<Event> events(const std::string & what)
{
  Record & r = record_of_process();
  const std::lock_guard<std::mutex> lk(r.m);
  std::vector<Event> out;
  for (const Event & e : r.all) {
    if (e.what == what) out.push_back(e);
  }
  return out;
}

}  // namespace probe
