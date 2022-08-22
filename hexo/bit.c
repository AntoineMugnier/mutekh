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

    Copyright (c) 2017 Alexandre Becoulet <alexandre.becoulet@free.fr>
*/

#include <hexo/bit.h>
#include <hexo/endian.h>

extern inline uint_fast8_t __bit_ctz8(uint8_t x);
extern inline uint_fast8_t __bit_ctz16(uint16_t x);
extern inline uint_fast8_t __bit_ctz32(uint32_t x);
extern inline uint_fast8_t __bit_ctz64(uint64_t x);

extern inline uint_fast8_t __bit_clz8(uint8_t x);
extern inline uint_fast8_t __bit_clz16(uint16_t x);
extern inline uint_fast8_t __bit_clz32(uint32_t x);
extern inline uint_fast8_t __bit_clz64(uint64_t x);

extern inline uint_fast8_t __bit_popc8(uint8_t x);
extern inline uint_fast8_t __bit_popc16(uint16_t x);
extern inline uint_fast8_t __bit_popc32(uint32_t x);
extern inline uint_fast8_t __bit_popc64(uint64_t x);

extern inline void bitstring_set(uint8_t *bitstring, reg_t index);
extern inline void bitstring_clear(uint8_t *bitstring, reg_t index);
extern inline void bitstring_set_value(uint8_t *bitstring, reg_t index, bool_t value);
extern inline bool_t bitstring_get(const uint8_t *bitstring, reg_t index);

void bitstring_set32(uint8_t *bitstring, reg_t index, reg_t width, uint32_t value)
{
  if ((index % 8) + width <= 32) {
    uint32_t old = endian_le32_na_load(bitstring + (index / 8));
    uint32_t next = BIT_INSERT(old, value, index % 8, width);
    endian_le32_na_store(bitstring + (index / 8), next);

    return;
  }
  
  uint_fast8_t lsb_width = 32 - (index % 8);

  uint32_t lsb_old = endian_le32_na_load(bitstring + (index / 8));
  uint32_t lsb_next = BIT_INSERT(lsb_old, value, index % 8, lsb_width);
  endian_le32_na_store(bitstring + (index / 8), lsb_next);

  uint32_t msb_old = endian_le32_na_load(bitstring + (index / 8) + 4);
  uint32_t msb_next = BIT_INSERT(msb_old, value >> lsb_width,
                                 0, width - lsb_width);
  endian_le32_na_store(bitstring + (index / 8) + 4, msb_next);

  return;
}

uint32_t bitstring_get32(const uint8_t *bitstring, reg_t index, reg_t width)
{
  if ((index % 8) + width <= 32)
    return bit_get_mask(endian_le32_na_load(bitstring + (index / 8)),
                        index % 8, width);

  uint_fast8_t lsb_width = 32 - (index % 8);
  uint32_t lsb = bit_get_mask(endian_le32_na_load(bitstring + (index / 8)),
                              index % 8, lsb_width);
  uint32_t msb = bit_get_mask(endian_le32_na_load(bitstring + (index / 8)) + 4,
                              0, width - lsb_width);

  return lsb | (msb << lsb_width);
}
