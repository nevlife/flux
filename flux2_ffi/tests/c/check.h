// Shared by the C ABI tests that tests/abi.rs builds against the committed header.
#ifndef FLUX2_TEST_CHECK_H
#define FLUX2_TEST_CHECK_H

#include "flux2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define CHECK(c)                                                                                   \
  do {                                                                                             \
    if (!(c)) {                                                                                    \
      fprintf(stderr, "%s:%d: %s (last error: %s)\n", __FILE__, __LINE__, #c, flux2_last_error()); \
      exit(1);                                                                                     \
    }                                                                                              \
  } while (0)

// A signpost name unique to this process.
static inline void test_signpost(const char * key, char * out, size_t cap)
{
  char k[128];
  size_t len = 0;
  snprintf(k, sizeof k, "/c%d_%s", (int)getpid(), key);
  CHECK(flux2_signpost_name(k, FLUX2_NO_SCHEMA, out, cap, &len) == FLUX2_OK);
}

static inline flux2_executor * test_executor(void)
{
  flux2_executor * ex = NULL;
  CHECK(flux2_executor_new(&ex) == FLUX2_OK);
  return ex;
}

#endif
