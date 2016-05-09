/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for a keyboard
*/

#ifndef LIBDEVICE_VALIO_KEYBOARD_H_
#define LIBDEVICE_VALIO_KEYBOARD_H_

#include <device/class/valio.h>

enum valio_keyboard_att_e
{
  /**
     Value for this attribute is an array of bytes where bits are set
     to 1 for each pressed key.

     Bits are used from LSB to MSB, from lower address to upper.  Bits
     are used in groups of columns, column per column, all rows in
     order.

     For instance, for a 3 column per 3 lines keyboard, bits will be,
     with B(Col, Row):

              Bit0   Bit1   Bit2   Bit3   Bit4   Bit5   Bit6   Bit7
     Byte 0   B(0,0) B(1,0) B(2,0) B(0,1) B(1,1) B(2,1) B(0,2) B(1,2)
     Byte 1   B(2,2) Padding...

     Requester should allocate enough bytes to store the whole array.
   */
  VALIO_KEYBOARD_MAP = CONFIG_DEVICE_VALIO_KEYBOARD_ATTRIBUTE_FIRST,
};

/**
   @this is an helper function to retrieve a given key state from an
   array of bytes.
 */
ALWAYS_INLINE
bool_t dev_valio_keyboard_key_get(const uint8_t *bits,
                                  uint8_t column_count,
                                  uint8_t column, uint8_t row)
{
  uint16_t bit = (row * column_count) + column;
  return (bits[bit >> 3] >> (bit & 7)) & 1;
}

/**
   @this is an helper function to set a given key state in an array of
   bytes.
 */
ALWAYS_INLINE
void dev_valio_keyboard_key_set(uint8_t *bits,
                                uint8_t column_count,
                                uint8_t column, uint8_t row,
                                bool_t value)
{
  uint16_t bit = (row * column_count) + column;

  if (value)
    bits[bit >> 3] |= 1 << (bit & 7);
  else
    bits[bit >> 3] &= ~(1 << (bit & 7));
}

#endif
