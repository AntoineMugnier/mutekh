#include "hid_kbd.h"
#include "keymap.h"
#include "keymap_iterator.h"

const uint16_t keymap_fr[128] = {
  ['\t'] = HID_KBD_TAB,
  ['\n'] = HID_KBD_ENTER,
  [' '] = HID_KBD_SPACEBAR,

  ['&'] = HID_KBD_1,
  //[''] = HID_KBD_2,
  ['"'] = HID_KBD_3,
  ['\''] = HID_KBD_4,
  ['('] = HID_KBD_5,
  ['-'] = HID_KBD_6,
  //[''] = HID_KBD_7,
  ['_'] = HID_KBD_8,
  //[''] = HID_KBD_9,
  //[''] = HID_KBD_0,
  [')'] = HID_KBD_MINUS_UNDERSCORE,
  ['='] = HID_KBD_EQUAL_PLUS,

  ['1'] = MOD_SHIFT | HID_KBD_1,
  ['2'] = MOD_SHIFT | HID_KBD_2,
  ['3'] = MOD_SHIFT | HID_KBD_3,
  ['4'] = MOD_SHIFT | HID_KBD_4,
  ['5'] = MOD_SHIFT | HID_KBD_5,
  ['6'] = MOD_SHIFT | HID_KBD_6,
  ['7'] = MOD_SHIFT | HID_KBD_7,
  ['8'] = MOD_SHIFT | HID_KBD_8,
  ['9'] = MOD_SHIFT | HID_KBD_9,
  ['0'] = MOD_SHIFT | HID_KBD_0,
  //[''] = MOD_SHIFT | HID_KBD_MINUS_UNDERSCORE,
  ['+'] = MOD_SHIFT | HID_KBD_EQUAL_PLUS,

  //[''] = MOD_RALT | HID_KBD_1,
  ['~'] = MOD_DEAD | MOD_RALT | HID_KBD_2,
  ['#'] = MOD_RALT | HID_KBD_3,
  ['{'] = MOD_RALT | HID_KBD_4,
  ['['] = MOD_RALT | HID_KBD_5,
  ['|'] = MOD_RALT | HID_KBD_6,
  ['`'] = MOD_DEAD | MOD_RALT | HID_KBD_7,
  ['\\'] = MOD_RALT | HID_KBD_8,
  ['^'] = MOD_DEAD | MOD_RALT | HID_KBD_9,
  ['@'] = MOD_RALT | HID_KBD_0,
  [']'] = MOD_RALT | HID_KBD_MINUS_UNDERSCORE,
  ['}'] = MOD_RALT | HID_KBD_EQUAL_PLUS,

  ['a'] = HID_KBD_Q,
  ['z'] = HID_KBD_W,
  ['e'] = HID_KBD_E,
  ['r'] = HID_KBD_R,
  ['t'] = HID_KBD_T,
  ['y'] = HID_KBD_Y,
  ['u'] = HID_KBD_U,
  ['i'] = HID_KBD_I,
  ['o'] = HID_KBD_O,
  ['p'] = HID_KBD_P,
  //[''] = HID_KBD_OPEN_BRACKET,
  ['$'] = HID_KBD_CLOSE_BRACKET,

  ['A'] = MOD_SHIFT | HID_KBD_Q,
  ['Z'] = MOD_SHIFT | HID_KBD_W,
  ['E'] = MOD_SHIFT | HID_KBD_E,
  ['R'] = MOD_SHIFT | HID_KBD_R,
  ['T'] = MOD_SHIFT | HID_KBD_T,
  ['Y'] = MOD_SHIFT | HID_KBD_Y,
  ['U'] = MOD_SHIFT | HID_KBD_U,
  ['I'] = MOD_SHIFT | HID_KBD_I,
  ['O'] = MOD_SHIFT | HID_KBD_O,
  ['P'] = MOD_SHIFT | HID_KBD_P,
  //[''] = MOD_SHIFT | HID_KBD_OPEN_BRACKET,
  //[''] = MOD_SHIFT | HID_KBD_CLOSE_BRACKET,

  ['q'] = HID_KBD_A,
  ['s'] = HID_KBD_S,
  ['d'] = HID_KBD_D,
  ['f'] = HID_KBD_F,
  ['g'] = HID_KBD_G,
  ['h'] = HID_KBD_H,
  ['j'] = HID_KBD_J,
  ['k'] = HID_KBD_K,
  ['l'] = HID_KBD_L,
  ['m'] = HID_KBD_SEMICOLON,
  //[''] = HID_KBD_QUOTE,
  ['*'] = HID_KBD_BACKSLASH,

  ['Q'] = MOD_SHIFT | HID_KBD_A,
  ['S'] = MOD_SHIFT | HID_KBD_S,
  ['D'] = MOD_SHIFT | HID_KBD_D,
  ['F'] = MOD_SHIFT | HID_KBD_F,
  ['G'] = MOD_SHIFT | HID_KBD_G,
  ['H'] = MOD_SHIFT | HID_KBD_H,
  ['J'] = MOD_SHIFT | HID_KBD_J,
  ['K'] = MOD_SHIFT | HID_KBD_K,
  ['L'] = MOD_SHIFT | HID_KBD_L,
  ['M'] = MOD_SHIFT | HID_KBD_SEMICOLON,
  ['%'] = MOD_SHIFT | HID_KBD_QUOTE,
  //[''] = MOD_SHIFT | HID_KBD_BACKSLASH,

  ['w'] = HID_KBD_Z,
  ['x'] = HID_KBD_X,
  ['c'] = HID_KBD_C,
  ['v'] = HID_KBD_V,
  ['b'] = HID_KBD_B,
  ['n'] = HID_KBD_N,
  [','] = HID_KBD_M,
  [';'] = HID_KBD_COMMA,
  [':'] = HID_KBD_PERIOD,
  ['!'] = HID_KBD_SLASH,

  ['W'] = MOD_SHIFT | HID_KBD_Z,
  ['X'] = MOD_SHIFT | HID_KBD_X,
  ['C'] = MOD_SHIFT | HID_KBD_C,
  ['V'] = MOD_SHIFT | HID_KBD_V,
  ['B'] = MOD_SHIFT | HID_KBD_B,
  ['N'] = MOD_SHIFT | HID_KBD_N,
  ['?'] = MOD_SHIFT | HID_KBD_M,
  ['.'] = MOD_SHIFT | HID_KBD_COMMA,
  ['/'] = MOD_SHIFT | HID_KBD_PERIOD,
  //[''] = MOD_SHIFT | HID_KBD_SLASH,
};
