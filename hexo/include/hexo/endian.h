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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

/**
 * @file
 * @module{Hexo}
 * @short Integer values byte-swaping and endian stuff
 */

#ifndef __ENDIAN_H_
#define __ENDIAN_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include "types.h"

/***********************************************************************
 *		Words swap functions
 */

/*
ALWAYS_INLINE uint16_t cpu_endian_swap16(uint16_t x);
ALWAYS_INLINE uint32_t cpu_endian_swap32(uint32_t x);
ALWAYS_INLINE uint64_t cpu_endian_swap64(uint64_t x);
*/

#include "cpu/hexo/endian.h"

/** @internal */
ALWAYS_INLINE uint16_t __endian_swap16(uint16_t x)
{
  return (x >> 8) | (x << 8);
}

/** @this swaps bytes in 16 bits value */
ALWAYS_INLINE uint16_t endian_swap16(uint16_t x)
{
# ifdef HAS_CPU_ENDIAN_SWAP16
  return __builtin_constant_p(x) ? __endian_swap16(x) : cpu_endian_swap16(x);
#else
  return __endian_swap16(x);
# endif
}

/** @internal */
ALWAYS_INLINE uint32_t __endian_swap32(uint32_t x)
{
  return (((x >> 24) & 0x000000ff) |
	  ((x >> 8 ) & 0x0000ff00) |
	  ((x << 8 ) & 0x00ff0000) |
	  ((x << 24) & 0xff000000));
}

/** @this swaps bytes in 32 bits value */
ALWAYS_INLINE uint32_t endian_swap32(uint32_t x)
{
# ifdef HAS_CPU_ENDIAN_SWAP32
  return __builtin_constant_p(x) ? __endian_swap32(x) : cpu_endian_swap32(x);
#else
  return __endian_swap32(x);
# endif
}

/** @internal */
ALWAYS_INLINE uint64_t __endian_swap64(uint64_t x)
{
  return (((uint64_t)endian_swap32(x      ) << 32) |
	  ((uint64_t)endian_swap32(x >> 32)      ));
}

/** @this swaps bytes in 64 bits value */
ALWAYS_INLINE uint64_t endian_swap64(uint64_t x)
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

# if defined (CONFIG_CPU_ENDIAN_BIG)

/** @this reads a big endian 16 bits value */
#  define endian_be16(x)	(x)
/** @this reads a big endian 32 bits value */
#  define endian_be32(x)	(x)
/** @this reads a big endian 64 bits value */
#  define endian_be64(x)	(x)
/** @this reads a little endian 16 bits value */
#  define endian_le16(x)	endian_swap16(x)
/** @this reads a little endian 32 bits value */
#  define endian_le32(x)	endian_swap32(x)
/** @this reads a little endian 64 bits value */
#  define endian_le64(x)	endian_swap64(x)

# elif defined (CONFIG_CPU_ENDIAN_LITTLE)

/** @this reads a big endian 16 bits value */
#  define endian_le16(x)	(x)
/** @this reads a big endian 32 bits value */
#  define endian_le32(x)	(x)
/** @this reads a big endian 64 bits value */
#  define endian_le64(x)	(x)
/** @this reads a little endian 16 bits value */
#  define endian_be16(x)	endian_swap16(x)
/** @this reads a little endian 32 bits value */
#  define endian_be32(x)	endian_swap32(x)
/** @this reads a little endian 64 bits value */
#  define endian_be64(x)	endian_swap64(x)

# else

#  error No endian mode defined in cpu/hexo/endian.h

# endif

/** @internal */
#define __ENDIAN_REVERSE_ARGS(b01, b02, b03, b04, b05, b06, b07, b08, b09, b10, b11, b12, b13, b14, b15, b16, ...)    \
			       b16; b15; b14; b13; b12; b11; b10; b09; b08; b07; b06; b05; b04; b03; b02; b01;

/** @internal */
#define __ENDIAN_ARGS(b01, b02, b03, b04, b05, b06, b07, b08, b09, b10, b11, b12, b13, b14, b15, b16, ...)    \
			       b01; b02; b03; b04; b05; b06; b07; b08; b09; b10; b11; b12; b13; b14; b15; b16;
# if defined (CONFIG_CPU_ENDIAN_BIG)

/**
 @multiple
 Endian dependent bitfield declaration macro.

 Bitfield can be declared trough the @ref #ENDIAN_BITFIELD macro in the
 direct (big endian) order. Bitfield declaration order will be swaped
 on little endian machines. See example below:

 @code
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
 @end code
 */

#  define ENDIAN_BITFIELD(...)	__ENDIAN_ARGS(__VA_ARGS__,,,,,,,,,,,,,,,)
# elif defined (CONFIG_CPU_ENDIAN_LITTLE)
#  define ENDIAN_BITFIELD(...)	__ENDIAN_REVERSE_ARGS(__VA_ARGS__,,,,,,,,,,,,,,,)
# else
#  error No bitfield endian mode defined in cpu/hexo/endian.h
# endif

/***********************************************************************
 *		Non aligned access functions
 */

#if defined (CONFIG_CPU_NONALIGNED_ACCESS)
/** @multiple @this loads from non aligned memory location with native endianess */
# define endian_16_na_load(a)		({ void * __a = (void*)(a); *((uint16_t*)(__a)); })
# define endian_32_na_load(a)		({ void * __a = (void*)(a); *((uint32_t*)(__a)); })
# define endian_64_na_load(a)		({ void * __a = (void*)(a); *((uint64_t*)(__a)); })
/** @multiple @this stores to non aligned memory location with native endianess */
# define endian_16_na_store(a, x)	({ void * __a = (void*)(a); *((uint16_t*)(__a)) = (x); })
# define endian_32_na_store(a, x)	({ void * __a = (void*)(a); *((uint32_t*)(__a)) = (x); })
# define endian_64_na_store(a, x)	({ void * __a = (void*)(a); *((uint64_t*)(__a)) = (x); })

/** @multiple @this allows direct non aligned word width memory access with endian permutation */
# define endian_le16_na_load(a)		endian_le16(endian_16_na_load(a))
# define endian_le32_na_load(a)		endian_le32(endian_32_na_load(a))
# define endian_le64_na_load(a)		endian_le64(endian_64_na_load(a))
# define endian_le16_na_store(a, x)	endian_16_na_store(a, endian_le16(x))
# define endian_le32_na_store(a, x)	endian_32_na_store(a, endian_le32(x))
# define endian_le64_na_store(a, x)	endian_64_na_store(a, endian_le64(x))

# define endian_be16_na_load(a)		endian_be16(endian_16_na_load(a))
# define endian_be32_na_load(a)		endian_be32(endian_32_na_load(a))
# define endian_be64_na_load(a)		endian_be64(endian_64_na_load(a))
# define endian_be16_na_store(a, x)	endian_16_na_store(a, endian_be16(x))
# define endian_be32_na_store(a, x)	endian_32_na_store(a, endian_be32(x))
# define endian_be64_na_store(a, x)	endian_64_na_store(a, endian_be64(x))

#else

/** @internal @multiple */
# define __ENDIAN_NAL_L(addr, type, index, shift)	((type)((uint8_t*)addr)[index] << shift)
# define __ENDIAN_NAS_R(addr, index, value, shift)	(((uint8_t*)addr)[index] = (uint8_t)(value >> shift))

/** @multiple @this allows memory access with endian permutation using multiple bytes access */

# define endian_le16_na_load(a)		({ void * __a = (void*)(a);                     \
                                         __ENDIAN_NAL_L(__a, uint16_t, 0, 0) |          \
                                         __ENDIAN_NAL_L(__a, uint16_t, 1, 8); })

# define endian_le32_na_load(a)		({ void * __a = (void*)(a);                     \
                                         __ENDIAN_NAL_L(__a, uint32_t, 0, 0)  |         \
                                         __ENDIAN_NAL_L(__a, uint32_t, 1, 8)  |         \
				         __ENDIAN_NAL_L(__a, uint32_t, 2, 16) |         \
                                         __ENDIAN_NAL_L(__a, uint32_t, 3, 24); })

# define endian_le64_na_load(a)		({ void * __a = (void*)(a);                     \
                                         __ENDIAN_NAL_L(__a, uint64_t, 0, 0)  |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 1, 8)  |         \
					 __ENDIAN_NAL_L(__a, uint64_t, 2, 16) |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 3, 24) |         \
					 __ENDIAN_NAL_L(__a, uint64_t, 4, 32) |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 5, 40) |         \
					 __ENDIAN_NAL_L(__a, uint64_t, 6, 48) |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 7, 56); })

# define endian_le16_na_store(a, x)	({ void * __a = (void*)(a);                     \
                                          const uint16_t __val = (x);			\
					 __ENDIAN_NAS_R(__a, 0, __val, 0);		\
					 __ENDIAN_NAS_R(__a, 1, __val, 8); __val; })

# define endian_le32_na_store(a, x)	({ void * __a = (void*)(a);                     \
                                          const uint32_t __val = (x);			\
					 __ENDIAN_NAS_R(__a, 0, __val, 0);		\
					 __ENDIAN_NAS_R(__a, 1, __val, 8);		\
					 __ENDIAN_NAS_R(__a, 2, __val, 16);		\
					 __ENDIAN_NAS_R(__a, 3, __val, 24); __val; })

# define endian_le64_na_store(a, x)	({ void * __a = (void*)(a);                     \
                                          const uint64_t __val = (x);			\
					 __ENDIAN_NAS_R(__a, 0, __val, 0);		\
					 __ENDIAN_NAS_R(__a, 1, __val, 8);		\
					 __ENDIAN_NAS_R(__a, 2, __val, 16);		\
					 __ENDIAN_NAS_R(__a, 3, __val, 24);		\
					 __ENDIAN_NAS_R(__a, 4, __val, 32);		\
					 __ENDIAN_NAS_R(__a, 5, __val, 40);		\
					 __ENDIAN_NAS_R(__a, 6, __val, 48);		\
					 __ENDIAN_NAS_R(__a, 7, __val, 56); __val; })

# define endian_be16_na_load(a)		({ void * __a = (void*)(a);                     \
                                         __ENDIAN_NAL_L(__a, uint16_t,  0, 8) |         \
                                         __ENDIAN_NAL_L(__a, uint16_t, 1, 0); })

# define endian_be32_na_load(a)		({ void * __a = (void*)(a);                     \
                                         __ENDIAN_NAL_L(__a, uint32_t, 0, 24) |         \
                                         __ENDIAN_NAL_L(__a, uint32_t, 1, 16) |         \
				         __ENDIAN_NAL_L(__a, uint32_t, 2, 8)  |         \
                                         __ENDIAN_NAL_L(__a, uint32_t, 3, 0); })

# define endian_be64_na_load(a)		({ void * __a = (void*)(a);                     \
                                         __ENDIAN_NAL_L(__a, uint64_t, 0, 56) |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 1, 48) |         \
					 __ENDIAN_NAL_L(__a, uint64_t, 2, 40) |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 3, 32) |         \
					 __ENDIAN_NAL_L(__a, uint64_t, 4, 24) |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 5, 16) |         \
					 __ENDIAN_NAL_L(__a, uint64_t, 6, 8)  |         \
                                         __ENDIAN_NAL_L(__a, uint64_t, 7, 0); })

# define endian_be16_na_store(a, x)	({ void * __a = (void*)(a);                     \
                                          const uint16_t __val = (x);			\
					 __ENDIAN_NAS_R(__a, 0, __val, 8);		\
					 __ENDIAN_NAS_R(__a, 1, __val, 0); __val; })

# define endian_be32_na_store(a, x)	({ void * __a = (void*)(a);                     \
                                          const uint32_t __val = (x);			\
					 __ENDIAN_NAS_R(__a, 0, __val, 24);		\
					 __ENDIAN_NAS_R(__a, 1, __val, 16);		\
					 __ENDIAN_NAS_R(__a, 2, __val, 8);		\
					 __ENDIAN_NAS_R(__a, 3, __val, 0); __val; })

# define endian_be64_na_store(a, x)	({ void * __a = (void*)(a);                     \
                                          const uint64_t __val = (x);			\
					 __ENDIAN_NAS_R(__a, 0, __val, 56);		\
					 __ENDIAN_NAS_R(__a, 1, __val, 48);		\
					 __ENDIAN_NAS_R(__a, 2, __val, 40);		\
					 __ENDIAN_NAS_R(__a, 3, __val, 32);		\
					 __ENDIAN_NAS_R(__a, 4, __val, 24);		\
					 __ENDIAN_NAS_R(__a, 5, __val, 16);		\
					 __ENDIAN_NAS_R(__a, 6, __val, 8);		\
					 __ENDIAN_NAS_R(__a, 7, __val, 0); __val; })


# if defined (CONFIG_CPU_ENDIAN_BIG)
/** @multiple @this loads from non aligned memory location with native endianess */
#  define endian_16_na_load(a)		endian_be16_na_load(a)
#  define endian_32_na_load(a)		endian_be32_na_load(a)
#  define endian_64_na_load(a)		endian_be64_na_load(a)
/** @multiple @this stores to non aligned memory location with native endianess */
#  define endian_16_na_store(a, x)	endian_be16_na_store(a, x)
#  define endian_32_na_store(a, x)	endian_be32_na_store(a, x)
#  define endian_64_na_store(a, x)	endian_be64_na_store(a, x)
# else
/** @multiple @this loads from non aligned memory location with native endianess */
#  define endian_16_na_load(a)		endian_le16_na_load(a)
#  define endian_32_na_load(a)		endian_le32_na_load(a)
#  define endian_64_na_load(a)		endian_le64_na_load(a)
/** @multiple @this stores to non aligned memory location with native endianess */
#  define endian_16_na_store(a, x)	endian_le16_na_store(a, x)
#  define endian_32_na_store(a, x)	endian_le32_na_store(a, x)
#  define endian_64_na_store(a, x)	endian_le64_na_store(a, x)
# endif

#endif

/***********************************************************************
 *		Address and values alignment
 */

/** @this returns true if value is a power of 2 */
#define ALIGN_ISPOWTWO(x)	!((x) & ((x) - 1))

/** @this returns true if value is aligned */
#define IS_ALIGNED(x, b)	!(((uintptr_t)(x)) & ((b) - 1))

/** @internal @this does not use aligment code if b==1 at compilation time. */
#define __ALIGN_CONSTANT(x, b, A) (__builtin_constant_p(b) ? ((b) == 1 ? (x) : A(x, b)) : A(x, b))

/** @internal */
#define __ALIGN_VALUE_UP(x, b)	((((x) - 1) | ((b) - 1)) + 1)
/** @this aligns value on the next power of two boundary  */
#define ALIGN_VALUE_UP(x, b)	__ALIGN_CONSTANT(x, b, __ALIGN_VALUE_UP)

/** @internal */
#define __ALIGN_VALUE_LOW(x, b)	((x) & ~((b) - 1))
/** @this aligns value on the next power of two boundary */
#define ALIGN_VALUE_LOW(x, b)	__ALIGN_CONSTANT(x, b, __ALIGN_VALUE_LOW)

/** @this aligns address on the next power of two boundary  */
#define ALIGN_ADDRESS_UP(x, b)	((void*)ALIGN_VALUE_UP((uintptr_t)(x), (b)))

/** @this aligns address on the next power of two boundary  */
#define ALIGN_ADDRESS_LOW(x, b)	((void*)ALIGN_VALUE_LOW((uintptr_t)(x), (b)))

#define __POW2_M1_CONSTANT_UP1(x)  ((x) | ((x) >> 1))
#define __POW2_M1_CONSTANT_UP2(x)  (__POW2_M1_CONSTANT_UP1(x) | (__POW2_M1_CONSTANT_UP1(x) >> 2))
#define __POW2_M1_CONSTANT_UP4(x)  (__POW2_M1_CONSTANT_UP2(x) | (__POW2_M1_CONSTANT_UP2(x) >> 4))
#define __POW2_M1_CONSTANT_UP8(x)  (__POW2_M1_CONSTANT_UP4(x) | (__POW2_M1_CONSTANT_UP4(x) >> (8 % (sizeof(x)*8))))
#define __POW2_M1_CONSTANT_UP16(x) (__POW2_M1_CONSTANT_UP8(x) | (__POW2_M1_CONSTANT_UP8(x) >> (16 % (sizeof(x)*8))))
/** @this returns a power of 2 minus one where the pow2 is greater than the specified value */
#define POW2_M1_CONSTANT_UP(x)    (__POW2_M1_CONSTANT_UP16(x) | (__POW2_M1_CONSTANT_UP16(x) >> (32 % (sizeof(x)*8))))

/** @this returns a power of 2 greater or equal to the specified value */
#define POW2_CONSTANT_UP(x) (POW2_M1_CONSTANT_UP(x - 1) + 1)

/***********************************************************************
 *		Bits extraction macro
 */

/** @this extracts bit at specified index */
#define BIT_EXTRACT(v, index) ((v) & (1 << (index)))

/** @this extracts count bits from specified index */
#define BITS_EXTRACT_FC(v, first, count) (((v) >> (first)) & ((1 << (count)) - 1))

/** @this extracts bits between specified first and last index */
#define BITS_EXTRACT_FL(v, first, last) BITS_EXTRACT_FC(v, first, (last) - (first) + 1)

C_HEADER_END

#endif

