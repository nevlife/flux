// Shared by the C ABI tests that tests/abi.rs builds against the committed header.
#ifndef FLUX_TEST_CHECK_H
#define FLUX_TEST_CHECK_H

#include "flux.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define CHECK(c)                                                                                  \
  do {                                                                                            \
    if (!(c)) {                                                                                   \
      fprintf(stderr, "%s:%d: %s (last error: %s)\n", __FILE__, __LINE__, #c, flux_last_error()); \
      exit(1);                                                                                    \
    }                                                                                             \
  } while (0)

// A signpost name unique to this process.
static inline void test_signpost(const char * key, char * out, size_t cap)
{
  char k[128];
  size_t len = 0;
  snprintf(k, sizeof k, "/c%d_%s", (int)getpid(), key);
  CHECK(flux_signpost_name(k, FLUX_NO_SCHEMA, out, cap, &len) == FLUX_OK);
}

// A publisher with the default config but these settings.
static inline flux_status test_create(
  const char * name, uint32_t slot_size, uint32_t slot_count, uint32_t max_subscribers,
  uint32_t aux_size, uint64_t fingerprint, flux_channel ** out)
{
  flux_publisher_config c = flux_publisher_config_default();
  c.slot_size = slot_size;
  c.slot_count = slot_count;
  c.max_subscribers = max_subscribers;
  c.aux_size = aux_size;
  return flux_channel_create(name, &c, fingerprint, FLUX_MEMORY_DEFAULT, NULL, out);
}

static inline flux_status test_add(flux_executor * ex, flux_source src, int32_t priority)
{
  return flux_executor_add(ex, &src, priority);
}

static inline flux_executor * test_executor(void)
{
  flux_executor * ex = NULL;
  CHECK(flux_executor_new(&ex) == FLUX_OK);
  return ex;
}

#endif
