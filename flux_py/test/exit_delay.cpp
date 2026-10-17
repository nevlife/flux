// Preloaded by test_executor. An exit handler registered before Python starts runs after Python
// has finalized; sleeping there lets a daemon thread meet the finalized interpreter.
#include <unistd.h>

#include <cstdlib>

namespace
{
void late()
{
  usleep(400000);
}
__attribute__((constructor)) void init()
{
  std::atexit(late);
}
}  // namespace
