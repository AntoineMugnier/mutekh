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

