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

#ifndef INTTYPES_H_
#define INTTYPES_H_

#include <stdint.h>
#include <stddef.h>

#define __PRI8_PREFIX   "hh"

# if CPU_SIZEOF_SHORT == 16
#  define __PRI16_PREFIX "h"
# elif CPU_SIZEOF_INT == 16
#  define __PRI16_PREFIX
# else
#  error
# endif

# if CPU_SIZEOF_INT == 32
#  define __PRI32_PREFIX
# elif CPU_SIZEOF_LONG == 32
#  define __PRI32_PREFIX "l"
# else
#  error
# endif

# if CPU_SIZEOF_LONG == 64
#  define __PRI64_PREFIX "l"
# elif CPU_SIZEOF_LONGLONG == 64
#  define __PRI64_PREFIX "ll"
# else
#  error
# endif

# if INT_FAST8_SIZE == 8
#  define __PRIFAST8_PREFIX __PRI8_PREFIX
# elif INT_FAST8_SIZE == 16
#  define __PRIFAST8_PREFIX __PRI16_PREFIX
# elif INT_FAST8_SIZE == 32
#  define __PRIFAST8_PREFIX __PRI32_PREFIX
# elif INT_FAST8_SIZE == 64
#  define __PRIFAST8_PREFIX __PRI64_PREFIX
# else
#  error
# endif

# if INT_FAST16_SIZE == 8
#  define __PRIFAST16_PREFIX __PRI8_PREFIX
# elif INT_FAST16_SIZE == 16
#  define __PRIFAST16_PREFIX __PRI16_PREFIX
# elif INT_FAST16_SIZE == 32
#  define __PRIFAST16_PREFIX __PRI32_PREFIX
# elif INT_FAST16_SIZE == 64
#  define __PRIFAST16_PREFIX __PRI64_PREFIX
# else
#  error
# endif

# if INT_FAST32_SIZE == 8
#  define __PRIFAST32_PREFIX __PRI8_PREFIX
# elif INT_FAST32_SIZE == 16
#  define __PRIFAST32_PREFIX __PRI16_PREFIX
# elif INT_FAST32_SIZE == 32
#  define __PRIFAST32_PREFIX __PRI32_PREFIX
# elif INT_FAST32_SIZE == 64
#  define __PRIFAST32_PREFIX __PRI64_PREFIX
# else
#  error
# endif

# if INT_FAST64_SIZE == 8
#  define __PRIFAST64_PREFIX __PRI8_PREFIX
# elif INT_FAST64_SIZE == 16
#  define __PRIFAST64_PREFIX __PRI16_PREFIX
# elif INT_FAST64_SIZE == 32
#  define __PRIFAST64_PREFIX __PRI32_PREFIX
# elif INT_FAST64_SIZE == 64
#  define __PRIFAST64_PREFIX __PRI64_PREFIX
# else
#  error
# endif

# if INT_PTR_SIZE == 16
#  define __PRIPTR_PREFIX __PRI16_PREFIX
# elif INT_PTR_SIZE == 32
#  define __PRIPTR_PREFIX __PRI32_PREFIX
# elif INT_PTR_SIZE == 64
#  define __PRIPTR_PREFIX __PRI64_PREFIX
# else
#  error
# endif

/** @multiple Standard macro for use in format string of @ref printf
    and similar functions. */
# define PRId8		__PRI8_PREFIX  "d"
# define PRId16		__PRI16_PREFIX "d"
# define PRId32		__PRI32_PREFIX "d"
# define PRId64		__PRI64_PREFIX "d"
# define PRIdLEAST8	__PRIFAST8_PREFIX  "d"
# define PRIdLEAST16	__PRIFAST16_PREFIX "d"
# define PRIdLEAST32	__PRIFAST32_PREFIX "d"
# define PRIdLEAST64	__PRIFAST64_PREFIX "d"
# define PRIdFAST8	__PRIFAST8_PREFIX  "d"
# define PRIdFAST16	__PRIFAST16_PREFIX "d"
# define PRIdFAST32	__PRIFAST32_PREFIX "d"
# define PRIdFAST64	__PRIFAST64_PREFIX "d"

# define PRIi8		__PRI8_PREFIX  "i"
# define PRIi16		__PRI16_PREFIX "i"
# define PRIi32		__PRI32_PREFIX "i"
# define PRIi64		__PRI64_PREFIX "i"
# define PRIiLEAST8	__PRIFAST8_PREFIX  "i"
# define PRIiLEAST16	__PRIFAST16_PREFIX "i"
# define PRIiLEAST32	__PRIFAST32_PREFIX "i"
# define PRIiLEAST64	__PRIFAST64_PREFIX "i"
# define PRIiFAST8	__PRIFAST8_PREFIX  "i"
# define PRIiFAST16	__PRIFAST16_PREFIX "i"
# define PRIiFAST32	__PRIFAST32_PREFIX "i"
# define PRIiFAST64	__PRIFAST64_PREFIX "i"

# define PRIo8		__PRI8_PREFIX  "o"
# define PRIo16		__PRI16_PREFIX "o"
# define PRIo32		__PRI32_PREFIX "o"
# define PRIo64		__PRI64_PREFIX "o"
# define PRIoLEAST8	__PRIFAST8_PREFIX  "o"
# define PRIoLEAST16	__PRIFAST16_PREFIX "o"
# define PRIoLEAST32	__PRIFAST32_PREFIX "o"
# define PRIoLEAST64	__PRIFAST64_PREFIX "o"
# define PRIoFAST8	__PRIFAST8_PREFIX "o"
# define PRIoFAST16	__PRIFAST16_PREFIX "o"
# define PRIoFAST32	__PRIFAST32_PREFIX "o"
# define PRIoFAST64	__PRIFAST64_PREFIX "o"

# define PRIu8		__PRI8_PREFIX  "u"
# define PRIu16		__PRI16_PREFIX "u"
# define PRIu32		__PRI32_PREFIX "u"
# define PRIu64		__PRI64_PREFIX "u"
# define PRIuLEAST8	__PRIFAST8_PREFIX  "u"
# define PRIuLEAST16	__PRIFAST16_PREFIX "u"
# define PRIuLEAST32	__PRIFAST32_PREFIX "u"
# define PRIuLEAST64	__PRIFAST64_PREFIX "u"
# define PRIuFAST8	__PRIFAST8_PREFIX  "u"
# define PRIuFAST16	__PRIFAST16_PREFIX "u"
# define PRIuFAST32	__PRIFAST32_PREFIX "u"
# define PRIuFAST64	__PRIFAST64_PREFIX "u"

# define PRIx8		__PRI8_PREFIX  "x"
# define PRIx16		__PRI16_PREFIX "x"
# define PRIx32		__PRI32_PREFIX "x"
# define PRIx64		__PRI64_PREFIX "x"
# define PRIxLEAST8	__PRIFAST8_PREFIX  "x"
# define PRIxLEAST16	__PRIFAST16_PREFIX "x"
# define PRIxLEAST32	__PRIFAST32_PREFIX "x"
# define PRIxLEAST64	__PRIFAST64_PREFIX "x"
# define PRIxFAST8	__PRIFAST8_PREFIX  "x"
# define PRIxFAST16	__PRIFAST16_PREFIX "x"
# define PRIxFAST32	__PRIFAST32_PREFIX "x"
# define PRIxFAST64	__PRIFAST64_PREFIX "x"

# define PRIX8		__PRI8_PREFIX  "X"
# define PRIX16		__PRI16_PREFIX "X"
# define PRIX32		__PRI32_PREFIX "X"
# define PRIX64		__PRI64_PREFIX "X"
# define PRIXLEAST8	__PRIFAST8_PREFIX  "X"
# define PRIXLEAST16	__PRIFAST16_PREFIX "X"
# define PRIXLEAST32	__PRIFAST32_PREFIX "X"
# define PRIXLEAST64	__PRIFAST64_PREFIX "X"
# define PRIXFAST8	__PRIFAST8_PREFIX  "X"
# define PRIXFAST16	__PRIFAST16_PREFIX "X"
# define PRIXFAST32	__PRIFAST32_PREFIX "X"
# define PRIXFAST64	__PRIFAST64_PREFIX "X"

# define PRIdMAX	__PRI64_PREFIX "d"
# define PRIiMAX	__PRI64_PREFIX "i"
# define PRIoMAX	__PRI64_PREFIX "o"
# define PRIuMAX	__PRI64_PREFIX "u"
# define PRIxMAX	__PRI64_PREFIX "x"
# define PRIXMAX	__PRI64_PREFIX "X"

# define PRIdPTR	__PRIPTR_PREFIX "d"
# define PRIiPTR	__PRIPTR_PREFIX "i"
# define PRIoPTR	__PRIPTR_PREFIX "o"
# define PRIuPTR	__PRIPTR_PREFIX "u"
# define PRIxPTR	__PRIPTR_PREFIX "x"
# define PRIXPTR	__PRIPTR_PREFIX "X"

/** @multiple Standard macro for use in format string of @ref scanf
    and similar functions. */

# define SCNd8		__PRI8_PREFIX  "d"
# define SCNd16		__PRI16_PREFIX "d"
# define SCNd32		__PRI32_PREFIX "d"
# define SCNd64		__PRI64_PREFIX "d"
# define SCNdLEAST8	__PRIFAST8_PREFIX  "d"
# define SCNdLEAST16	__PRIFAST16_PREFIX "d"
# define SCNdLEAST32	__PRIFAST32_PREFIX "d"
# define SCNdLEAST64	__PRIFAST64_PREFIX "d"
# define SCNdFAST8	__PRIFAST8_PREFIX  "d"
# define SCNdFAST16	__PRIFAST16_PREFIX "d"
# define SCNdFAST32	__PRIFAST32_PREFIX "d"
# define SCNdFAST64	__PRIFAST64_PREFIX "d"

# define SCNi8		__PRI8_PREFIX  "i"
# define SCNi16		__PRI16_PREFIX "i"
# define SCNi32		__PRI32_PREFIX "i"
# define SCNi64		__PRI64_PREFIX "i"
# define SCNiLEAST8	__PRIFAST8_PREFIX  "i"
# define SCNiLEAST16	__PRIFAST16_PREFIX "i"
# define SCNiLEAST32	__PRIFAST32_PREFIX "i"
# define SCNiLEAST64	__PRIFAST64_PREFIX "i"
# define SCNiFAST8	__PRIFAST8_PREFIX  "i"
# define SCNiFAST16	__PRIFAST16_PREFIX "i"
# define SCNiFAST32	__PRIFAST32_PREFIX "i"
# define SCNiFAST64	__PRIFAST64_PREFIX "i"

# define SCNo8		__PRI8_PREFIX  "o"
# define SCNo16		__PRI16_PREFIX "o"
# define SCNo32		__PRI32_PREFIX "o"
# define SCNo64		__PRI64_PREFIX "o"
# define SCNoLEAST8	__PRIFAST8_PREFIX  "o"
# define SCNoLEAST16	__PRIFAST16_PREFIX "o"
# define SCNoLEAST32	__PRIFAST32_PREFIX "o"
# define SCNoLEAST64	__PRIFAST64_PREFIX "o"
# define SCNoFAST8	__PRIFAST8_PREFIX "o"
# define SCNoFAST16	__PRIFAST16_PREFIX "o"
# define SCNoFAST32	__PRIFAST32_PREFIX "o"
# define SCNoFAST64	__PRIFAST64_PREFIX "o"

# define SCNu8		__PRI8_PREFIX  "u"
# define SCNu16		__PRI16_PREFIX "u"
# define SCNu32		__PRI32_PREFIX "u"
# define SCNu64		__PRI64_PREFIX "u"
# define SCNuLEAST8	__PRIFAST8_PREFIX  "u"
# define SCNuLEAST16	__PRIFAST16_PREFIX "u"
# define SCNuLEAST32	__PRIFAST32_PREFIX "u"
# define SCNuLEAST64	__PRIFAST64_PREFIX "u"
# define SCNuFAST8	__PRIFAST8_PREFIX  "u"
# define SCNuFAST16	__PRIFAST16_PREFIX "u"
# define SCNuFAST32	__PRIFAST32_PREFIX "u"
# define SCNuFAST64	__PRIFAST64_PREFIX "u"

# define SCNx8		__PRI8_PREFIX  "x"
# define SCNx16		__PRI16_PREFIX "x"
# define SCNx32		__PRI32_PREFIX "x"
# define SCNx64		__PRI64_PREFIX "x"
# define SCNxLEAST8	__PRIFAST8_PREFIX  "x"
# define SCNxLEAST16	__PRIFAST16_PREFIX "x"
# define SCNxLEAST32	__PRIFAST32_PREFIX "x"
# define SCNxLEAST64	__PRIFAST64_PREFIX "x"
# define SCNxFAST8	__PRIFAST8_PREFIX  "x"
# define SCNxFAST16	__PRIFAST16_PREFIX "x"
# define SCNxFAST32	__PRIFAST32_PREFIX "x"
# define SCNxFAST64	__PRIFAST64_PREFIX "x"

# define SCNdMAX	__PRI64_PREFIX "d"
# define SCNiMAX	__PRI64_PREFIX "i"
# define SCNoMAX	__PRI64_PREFIX "o"
# define SCNuMAX	__PRI64_PREFIX "u"
# define SCNxMAX	__PRI64_PREFIX "x"

# define SCNdPTR	__PRIPTR_PREFIX "d"
# define SCNiPTR	__PRIPTR_PREFIX "i"
# define SCNoPTR	__PRIPTR_PREFIX "o"
# define SCNuPTR	__PRIPTR_PREFIX "u"
# define SCNxPTR	__PRIPTR_PREFIX "x"

#endif

