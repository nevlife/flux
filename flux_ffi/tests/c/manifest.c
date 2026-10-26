#include "check.h"

int main(void)
{
  char name[256];
  test_signpost("announce", name, sizeof name);
  char key[128];
  snprintf(key, sizeof key, "/c%d_announce", (int)getpid());
  flux_manifest_entry entry = {name, key, "/node", true};
  flux_announcement * a = NULL;
  CHECK(flux_announce(&entry, &a) == FLUX_OK);

  flux_topic_list * list = flux_enumerate_topics();
  const flux_topic_info * found = NULL;
  for (size_t i = 0; i < flux_topic_list_size(list); i++) {
    const flux_topic_info * t = flux_topic_list_get(list, i);
    if (strcmp(t->signpost, name) == 0) found = t;
  }
  CHECK(found != NULL);
  CHECK(found->key_exact && strcmp(found->key, key) == 0);
  CHECK(found->endpoint_count == 1);
  CHECK(found->endpoints[0].publisher && strcmp(found->endpoints[0].label, "/node") == 0);
  CHECK(flux_topic_list_get(list, (size_t)-1) == NULL);
  flux_topic_list_free(list);
  flux_announcement_free(a);
  shm_unlink(name);
  printf("ok\n");
  return 0;
}
