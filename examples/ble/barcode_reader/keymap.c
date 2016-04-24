#include <string.h>
#include "keymap.h"

extern const uint16_t keymap_fr[128];
extern const uint16_t keymap_fr_mac[128];
extern const uint16_t keymap_us[128];

const uint16_t *keymap_get(const char *lang)
{
  if (!strcmp(lang, "us"))
    return keymap_us;

  if (!strcmp(lang, "fr_mac"))
    return keymap_fr_mac;

  if (!strcmp(lang, "fr"))
    return keymap_fr;

  return NULL;
}

