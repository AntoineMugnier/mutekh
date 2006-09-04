/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef __ENDIAN_H_
#define __ENDIAN_H_

#include "types.h"

/***********************************************************************
 *		Words swap functions
 */

/*
static inline uint16_t cpu_endian_swap16(uint16_t x);
static inline uint32_t cpu_endian_swap32(uint32_t x);
static inline uint64_t cpu_endian_swap64(uint64_t x);
*/

#include "cpu/hexo/endian.h"

static inline uint16_t __endian_swap16(uint16_t x)
{
  return (x >> 8) | (x << 8);
}

/** swap bytes in 16 bits value */
static inline uint16_t endian_swap16(uint16_t x)
{
# ifdef HAS_CPU_ENDIAN_SWAP16
  return __builtin_constant_p(x) ? __endian_swap16(x) : cpu_endian_swap16(x);
#else
  return __endian_swap16(x);
# endif
}

static inline uint32_t __endian_swap32(uint32_t x)
{
  return (((x >> 24) & 0x000000ff) |
	  ((x >> 8 ) & 0x0000ff00) |
	  ((x << 8 ) & 0x00ff0000) |
	  ((x << 24) & 0xff000000));
}

/** swap bytes in 32 bits value */
static inline uint32_t endian_swap32(uint32_t x)
{
# ifdef HAS_CPU_ENDIAN_SWAP32
  return __builtin_constant_p(x) ? __endian_swap32(x) : cpu_endian_swap32(x);
#else
  return __endian_swap32(x);
# endif
}

static inline uint64_t __endian_swap64(uint64_t x)
{
  return (((uint64_t)endian_swap32(x      ) << 32) |
	  ((uint64_t)endian_swap32(x >> 32)      ));
}

/** swap bytes in 64 bits value */
static inline uint64_t endian_swap64(uint64_t x)
{
# ifdef HAS_CPU_ENDIAN_SWAP64
  return __builtin_constant_p(x) ? __endian_swap64(x) : cpu_endian_swap64(x);
#else
  return __endian_swap64(x);
# endif
}

/***********************************************************************
 *		Endian dependent words access functions
 */

# if defined (CPU_ENDIAN_ISBIG)

#  define endian_be16(x)	(x)
#  define endian_be32(x)	(x)
#  define endian_be64(x)	(x)
#  define endian_le16(x)	endian_swap16(x)
#  define endian_le32(x)	endian_swap32(x)
#  define endian_le64(x)	endian_swap64(x)

# elif defined (CPU_ENDIAN_ISLITTLE)

#  define endian_le16(x)	(x)
#  define endian_le32(x)	(x)
#  define endian_le64(x)	(x)
#  define endian_be16(x)	endian_swap16(x)
#  define endian_be32(x)	endian_swap32(x)
#  define endian_be64(x)	endian_swap64(x)

# else

#  error No endian mode defined in cpu/hexo/endian.h

# endif

/***********************************************************************
 *		Endian dependent bitfield declaration macro

 Bitfield can be declared trough the ENDIAN_BITFIELD() macro in the
 direct (big endian) order. bit field declaration order will be swaped
 on little endian machines. See example below:

 struct my_data_s
 {
   ENDIAN_BITFIELD(uint32_t   a:8,
                   uint32_t   b:8,
                   uint32_t   c:16
                  );

   ENDIAN_BITFIELD(uint32_t   d:16,
                   uint32_t   e:16
                  );
 };

 */

#define __ENDIAN_REVERSE_ARGS (b01, b02, b03, b04, b05, b06, b07, b08, b09, b10, b11, b12, b13, b14, b15, b16, ...)    \
			       b16; b15; b14; b13; b12; b11; b10; b09; b08; b07; b06; b05; b04; b03; b02; b01;

#define __ENDIAN_ARGS	      (b01, b02, b03, b04, b05, b06, b07, b08, b09, b10, b11, b12, b13, b14, b15, b16, ...)    \
			       b01; b02; b03; b04; b05; b06; b07; b08; b09; b10; b11; b12; b13; b14; b15; b16;

# if defined (CPU_ENDIAN_ISBIG)
#  define ENDIAN_BITFIELD(...)	__ENDIAN_ARGS(__VA_ARGS__,,,,,,,,,,,,,,,)
# elif defined (CPU_ENDIAN_ISLITTLE)
#  define ENDIAN_BITFIELD(...)	__ENDIAN_REVERSE_ARGS(__VA_ARGS__,,,,,,,,,,,,,,,)
# else
#  error No bitfield endian mode defined in cpu/hexo/endian.h
# endif

#endif

