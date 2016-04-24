#ifndef KEYMAP_ITERATOR_H_
#define KEYMAP_ITERATOR_H_

#include <stdint.h>
#include <stdlib.h>
#include <hexo/error.h>

#define MOD_RALT 0x8000
#define MOD_LALT 0x4000
#define MOD_SHIFT 0x2000
#define MOD_DEAD 0x1000

struct keymap_iterator_s
{
  const uint16_t *keymap;

  uint8_t scancode;
  uint8_t modifiers;
  uint8_t dead;
  uint8_t must_release;
  uint8_t prepared;
};

/**
   @this initializes a keymap iterator able to run over the key
   sequence to send in order to transmit the passed string.
 */
void keymap_iterator_init(struct keymap_iterator_s *ki,
                          const uint16_t *keymap);

error_t keymap_iterator_set(struct keymap_iterator_s *ki,
                            uint8_t chr);

/**
   @this retrieves the next action to perform on HID.

   @param modifiers targets a byte containing a bitmap of pressed
   modifiers (from lsb = LEFT_CTRL to msb = RIGHT_GUI)
   @param key targets a scan code to send

   @returns 0 if something useful was returned, something else if
   iteration is over.
 */
error_t keymap_iterator_next(struct keymap_iterator_s *ki,
                             uint8_t *modifiers,
                             uint8_t *key);
                          

#endif
