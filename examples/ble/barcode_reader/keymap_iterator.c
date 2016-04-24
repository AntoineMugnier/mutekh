#include "keymap_iterator.h"
#include "hid_kbd.h"
#include <assert.h>

#define SCANCODE_MASK 0x00ff

void keymap_iterator_init(struct keymap_iterator_s *ki,
                          const uint16_t *keymap)
{
  ki->keymap = keymap;
  ki->scancode = 0;
  ki->modifiers = 0;
  ki->dead = 0;
  ki->must_release = 0;
}

error_t keymap_iterator_set(struct keymap_iterator_s *ki,
                            uint8_t chr)
{
  uint16_t next;

  if (chr > 127)
    return -EINVAL;

  next = ki->keymap[chr];
  if (next == 0)
    return -EINVAL;

  ki->prepared = 0;
  
  if (next & MOD_SHIFT)
    ki->modifiers |= 1 << (HID_KBD_LEFT_SHIFT & 0x7);

  if (next & MOD_LALT)
    ki->modifiers |= 1 << (HID_KBD_LEFT_ALT & 0x7);

  if (next & MOD_RALT)
    ki->modifiers |= 1 << (HID_KBD_RIGHT_ALT & 0x7);

  ki->scancode = next & SCANCODE_MASK;
  ki->dead = !!(next & MOD_DEAD);

  return 0;
}

error_t keymap_iterator_next(struct keymap_iterator_s *ki,
                         uint8_t *modifiers,
                         uint8_t *key)
{
  if (ki->modifiers && !ki->prepared) {
    ki->prepared = 1;

    *modifiers = ki->modifiers;
    *key = 0;

    return 0;
  }

  if (ki->must_release) {
    assert(!ki->scancode);

    *modifiers = ki->modifiers;
    *key = 0;

    if (ki->modifiers) {
      ki->modifiers = 0;
      // and do another release cycle for modifiers afterwards.
    } else {
      ki->must_release = 0;
      // next cycle will be dead key or next char
    }
    return 0;
  }

  assert(!ki->must_release);

  if (ki->scancode) {
    *modifiers = ki->modifiers;
    *key = ki->scancode;

    ki->scancode = 0;
    ki->must_release = 1;
    return 0;
  }

  assert(!ki->modifiers);
  assert(!ki->scancode);

  if (ki->dead) {
    *modifiers = 0;
    *key = ki->keymap[' '] & SCANCODE_MASK;

    ki->dead = 0;
    ki->must_release = 1;
    return 0;
  }

  return -ENOENT;
}
