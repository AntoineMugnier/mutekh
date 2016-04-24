#include <stdio.h>
#include <string.h>
#include "keymap_iterator.h"
#include "keymap.h"

int main(int argc, char **argv)
{
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <keymap lang> <message>\n", argv[0]);
    return 1;
  }

  const char *lang = argv[1];
  const char *message = argv[2];

  const uint16_t *keymap = keymap_get(lang);

  struct keymap_iterator_s ki;
  uint8_t modifiers, key;

  keymap_iterator_init(&ki, keymap, message, strlen(message));

  while (keymap_iterator_next(&ki, &modifiers, &key)) {
    printf("Modifiers: %02x, key: %02x\n", modifiers, key);
  }

  return 0;
}
