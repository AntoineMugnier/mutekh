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
   @file
   @module{Hardware abstraction layer}
   @short Integer values byte-swaping and endian stuff

   @section {Non-aligned memory access}

     Non-aligned memory access is provided through various functions.
     Three flavors of accesses are provided:
     @list
     @item native CPU endianness accesses,
     @item little-endian accesses,
     @item big-endian accesses.
     @end list

     Depending on CPU module configuration and headers, each operation
     is implemented through dedicated assembly routine, native access
     or byte-based access.

     @ref #CONFIG_CPU_NONALIGNED_ACCESS

   @end section
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

#define __ENDIAN_SWAP16(x) \
  ((x >> 8) | (x << 8))

# ifdef HAS_CPU_ENDIAN_SWAP16
/** @this swaps bytes in 16 bits value */
#  define endian_swap16(x) \
  ((uint16_t)__builtin_choose_expr(__builtin_constant_p((x)),                     \
                                   __ENDIAN_SWAP16((uint16_t)(x)), \
                                   cpu_endian_swap16(x)))
#else
#  define endian_swap16(x) \
  ((uint16_t)__builtin_choose_expr(__builtin_constant_p((x)), \
                                   __ENDIAN_SWAP16((uint16_t)(x)),      \
                                   __endian_swap16(x)))
# endif


#define __ENDIAN_SWAP32(x)                                   \
  (((x >> 24) & 0x000000ff) | ((x >> 8 ) & 0x0000ff00) | \
   ((x << 8 ) & 0x00ff0000) | ((x << 24) & 0xff000000))

/** @internal */
ALWAYS_INLINE uint32_t __endian_swap32(uint32_t x)
{
  return (((x >> 24) & 0x000000ff) |
	  ((x >> 8 ) & 0x0000ff00) |
	  ((x << 8 ) & 0x00ff0000) |
	  ((x << 24) & 0xff000000));
}

# ifdef HAS_CPU_ENDIAN_SWAP32
/** @this swaps bytes in 32 bits value */
#  define endian_swap32(x) \
  ((uint32_t)__builtin_choose_expr(__builtin_constant_p((x)),                     \
                                   __ENDIAN_SWAP32((uint32_t)(x)), \
                                   cpu_endian_swap32(x)))
#else
#  define endian_swap32(x) \
  ((uint32_t)__builtin_choose_expr(__builtin_constant_p((x)), \
                                   __ENDIAN_SWAP32((uint32_t)(x)),      \
                                   __endian_swap32(x)))
# endif


/** @internal */
ALWAYS_INLINE uint64_t __endian_swap64(uint64_t x)
{
  return (((uint64_t)endian_swap32(x      ) << 32) |
	  ((uint64_t)endian_swap32(x >> 32)      ));
}

#define __ENDIAN_SWAP64(x)                             \
  (((uint64_t)__ENDIAN_SWAP32((uint32_t)x) << 32) |    \
   ((uint64_t)__ENDIAN_SWAP32(x >> 32)))

# ifdef HAS_CPU_ENDIAN_SWAP64
/** @this swaps bytes in 64 bits value */
#  define endian_swap64(x) \
  ((uint64_t)__builtin_choose_expr(__builtin_constant_p((x)),                     \
                                   __ENDIAN_SWAP64((uint64_t)(x)), \
                                   cpu_endian_swap64(x)))
#else
#  define endian_swap64(x) \
  ((uint64_t)__builtin_choose_expr(__builtin_constant_p((x)), \
                                   __ENDIAN_SWAP64((uint64_t)(x)),      \
                                   __endian_swap64(x)))
# endif

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


/**
   Endian-dependent structure contents reversal
 */

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

/**
   @this loads a 16-bit value stored at arbitrary address from memory
   with native CPU endianness.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 16-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint16_t endian_16_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_16_NA_LOAD)
  return cpu_endian_16_na_load(addr);
#elif CONFIG_CPU_NONALIGNED_ACCESS & 16
  return *(uint16_t*)addr;
#else
  const uint8_t *d = addr;
# if defined(CONFIG_CPU_ENDIAN_LITTLE)
  return ((uint16_t)d[1] << 8) | d[0];
# else
  return ((uint16_t)d[0] << 8) | d[1];
# endif
#endif
}

/**
   @this loads a 32-bit value stored at arbitrary address from memory
   with native CPU endianness.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 32-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint32_t endian_32_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_32_NA_LOAD)
  return cpu_endian_32_na_load(addr);
#elif CONFIG_CPU_NONALIGNED_ACCESS & 32
  return *(uint32_t*)addr;
#else
  const uint8_t *d = addr;
# if defined(CONFIG_CPU_ENDIAN_LITTLE)
  return endian_16_na_load(d) | ((uint32_t)endian_16_na_load(d + 2) << 16);
# else
  return ((uint32_t)endian_16_na_load(d) << 16) | endian_16_na_load(d + 2);
# endif
#endif
}

/**
   @this loads a 64-bit value stored at arbitrary address from memory
   with native CPU endianness.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 64-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint64_t endian_64_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_64_NA_LOAD)
  return cpu_endian_64_na_load(addr);
#elif CONFIG_CPU_NONALIGNED_ACCESS & 64
  return *(uint64_t*)addr;
#else
  const uint8_t *d = addr;
# if defined(CONFIG_CPU_ENDIAN_LITTLE)
  return endian_32_na_load(d) | ((uint64_t)endian_32_na_load(d + 4) << 32);
# else
  return ((uint64_t)endian_32_na_load(d) << 32) | endian_32_na_load(d + 4);
# endif
#endif
}

/**
   @this stores a 16-bit value at arbitrary address to memory with
   native CPU endianness.

   @xsee {Non-aligned memory access}

   @param addr Address to store 16-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_16_na_store(void *addr, uint16_t val)
{
#if defined(HAS_CPU_ENDIAN_16_NA_STORE)
  cpu_endian_16_na_store(addr, val);
#elif CONFIG_CPU_NONALIGNED_ACCESS & 16
  *(uint16_t*)addr = val;
#else
  uint8_t *d = addr;
# if defined(CONFIG_CPU_ENDIAN_LITTLE)
  d[0] = val;
  d[1] = val >> 8;
# else
  d[0] = val >> 8;
  d[1] = val;
# endif
#endif
}

/**
   @this stores a 32-bit value at arbitrary address to memory with
   native CPU endianness.

   @xsee {Non-aligned memory access}

   @param addr Address to store 32-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_32_na_store(void *addr, uint32_t val)
{
#if defined(HAS_CPU_ENDIAN_32_NA_STORE)
  cpu_endian_32_na_store(addr, val);
#elif CONFIG_CPU_NONALIGNED_ACCESS & 32
  *(uint32_t*)addr = val;
#else
  uint8_t *d = addr;
# if defined(CONFIG_CPU_ENDIAN_LITTLE)
  endian_16_na_store(d, val);
  endian_16_na_store(d + 2, val >> 16);
# else
  endian_16_na_store(d, val >> 16);
  endian_16_na_store(d + 2, val);
# endif
#endif
}

/**
   @this stores a 64-bit value at arbitrary address to memory with
   native CPU endianness.

   @xsee {Non-aligned memory access}

   @param addr Address to store 64-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_64_na_store(void *addr, uint64_t val)
{
#if defined(HAS_CPU_ENDIAN_64_NA_STORE)
  cpu_endian_64_na_store(addr, val);
#elif CONFIG_CPU_NONALIGNED_ACCESS & 64
  *(uint64_t*)addr = val;
#else
  uint8_t *d = addr;
# if defined(CONFIG_CPU_ENDIAN_LITTLE)
  endian_32_na_store(d, val);
  endian_32_na_store(d + 4, val >> 32);
# else
  endian_32_na_store(d, val >> 32);
  endian_32_na_store(d + 4, val);
# endif
#endif
}

/**
   @this loads a 16-bit little-endian value stored at arbitrary
   address from memory.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 16-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint16_t endian_le16_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_LE16_NA_LOAD)
  return cpu_endian_le16_na_load(addr);
#else
  return endian_le16(endian_16_na_load(addr));
#endif
}

/**
   @this loads a 32-bit little-endian value stored at arbitrary
   address from memory.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 32-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint32_t endian_le32_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_LE32_NA_LOAD)
  return cpu_endian_le32_na_load(addr);
#else
  return endian_le32(endian_32_na_load(addr));
#endif
}

/**
   @this loads a 64-bit little-endian value stored at arbitrary
   address from memory.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 64-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint64_t endian_le64_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_LE64_NA_LOAD)
  return cpu_endian_le64_na_load(addr);
#else
  return endian_le64(endian_64_na_load(addr));
#endif
}

/**
   @this stores a 16-bit value to memory using little-endian
   representation.

   @xsee {Non-aligned memory access}

   @param addr Address to store 16-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_le16_na_store(void *addr, uint16_t val)
{
#if defined(HAS_CPU_ENDIAN_LE16_NA_STORE)
  cpu_endian_le16_na_store(addr, val);
#else
  endian_16_na_store(addr, endian_le16(val));
#endif
}

/**
   @this stores a 32-bit value to memory using little-endian
   representation.

   @xsee {Non-aligned memory access}

   @param addr Address to store 32-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_le32_na_store(void *addr, uint32_t val)
{
#if defined(HAS_CPU_ENDIAN_LE32_NA_STORE)
  cpu_endian_le32_na_store(addr, val);
#else
  endian_32_na_store(addr, endian_le32(val));
#endif
}

/**
   @this stores a 64-bit value to memory using little-endian
   representation.

   @xsee {Non-aligned memory access}

   @param addr Address to store 64-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_le64_na_store(void *addr, uint64_t val)
{
#if defined(HAS_CPU_ENDIAN_LE64_NA_STORE)
  cpu_endian_le64_na_store(addr, val);
#else
  endian_64_na_store(addr, endian_le64(val));
#endif
}

/**
   @this loads a 16-bit big-endian value stored at arbitrary
   address from memory.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 16-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint16_t endian_be16_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_BE16_NA_LOAD)
  return cpu_endian_be16_na_load(addr);
#else
  return endian_be16(endian_16_na_load(addr));
#endif
}

/**
   @this loads a 32-bit big-endian value stored at arbitrary
   address from memory.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 32-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint32_t endian_be32_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_BE32_NA_LOAD)
  return cpu_endian_be32_na_load(addr);
#else
  return endian_be32(endian_32_na_load(addr));
#endif
}

/**
   @this loads a 64-bit big-endian value stored at arbitrary
   address from memory.

   @xsee {Non-aligned memory access}

   @param addr Address pointing a 64-bit value
   @returns Value at address
 */
ALWAYS_INLINE uint64_t endian_be64_na_load(const void *addr)
{
#if defined(HAS_CPU_ENDIAN_BE64_NA_LOAD)
  return cpu_endian_be64_na_load(addr);
#else
  return endian_be64(endian_64_na_load(addr));
#endif
}

/**
   @this stores a 16-bit value to memory using big-endian
   representation.

   @xsee {Non-aligned memory access}

   @param addr Address to store 16-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_be16_na_store(void *addr, uint16_t val)
{
#if defined(HAS_CPU_ENDIAN_BE16_NA_STORE)
  cpu_endian_be16_na_store(addr, val);
#else
  endian_16_na_store(addr, endian_be16(val));
#endif
}

/**
   @this stores a 32-bit value to memory using big-endian
   representation.

   @xsee {Non-aligned memory access}

   @param addr Address to store 32-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_be32_na_store(void *addr, uint32_t val)
{
#if defined(HAS_CPU_ENDIAN_BE32_NA_STORE)
  cpu_endian_be32_na_store(addr, val);
#else
  endian_32_na_store(addr, endian_be32(val));
#endif
}

/**
   @this stores a 64-bit value to memory using big-endian
   representation.

   @xsee {Non-aligned memory access}

   @param addr Address to store 64-bit value at
   @param val Value to store
 */
ALWAYS_INLINE void endian_be64_na_store(void *addr, uint64_t val)
{
#if defined(HAS_CPU_ENDIAN_BE64_NA_STORE)
  cpu_endian_be64_na_store(addr, val);
#else
  endian_64_na_store(addr, endian_be64(val));
#endif
}

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
#define BIT_EXTRACT(v, index) ((v) & (1ULL << (index)))

#define BIT_SET(v, index) ((v) |= (1ULL << (index)))

#define BIT_CLEAR(v, index) ((v) &= ~(1ULL << (index)))

/** @this extracts count bits from specified index */
#define BITS_EXTRACT_FC(v, first, count) (((v) >> (first)) & ((1ULL << (count)) - 1))

/** @this extracts bits between specified first and last index */
#define BITS_EXTRACT_FL(v, first, last) BITS_EXTRACT_FC(v, first, (last) - (first) + 1)

C_HEADER_END

#endif

