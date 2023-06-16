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

#if !defined(__ENDIAN_H_) || defined(__CPU_ENDIAN_H_)
#error This file can not be included directly
#else

#define __CPU_ENDIAN_H_

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 6

# define HAS_CPU_ENDIAN_SWAP16

ALWAYS_INLINE uint16_t cpu_endian_swap16(uint16_t x)
{
  uint16_t r;

  asm ("rev16	%0, %1"
       : "=l" (r)
       : "l" (x)
       );

  return r;
}

# define HAS_CPU_ENDIAN_SWAP32

ALWAYS_INLINE uint32_t cpu_endian_swap32(uint32_t x)
{
  uint32_t r;

  asm ("rev	%0, %1"
       : "=l" (r)
       : "l" (x)
       );

  return r;
}

#endif

//#define HAS_CPU_ENDIAN_SWAP64

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7

# define HAS_CPU_ENDIAN_16_NA_STORE

ALWAYS_INLINE void cpu_endian_16_na_store(void *addr, uint16_t val)
{
  asm ("strh    %[data], %[address]"
       : [address] "=Q" (*(uint16_t*)addr)
       : [data] "l" (val)
       );
}

# define HAS_CPU_ENDIAN_16_NA_LOAD

ALWAYS_INLINE uint16_t cpu_endian_16_na_load(const void *addr)
{
  uint16_t val;

  asm ("ldrh    %0, %1"
       : "=l" (val)
       : "Q" (*(uint16_t*)addr)
       );

  return val;
}

# define HAS_CPU_ENDIAN_32_NA_STORE

ALWAYS_INLINE void cpu_endian_32_na_store(void *addr, uint32_t val)
{
  asm ("str    %[data], %[address]"
       : [address] "=Q" (*(uint32_t*)addr)
       : [data] "l" (val)
       );
}

# define HAS_CPU_ENDIAN_32_NA_LOAD

ALWAYS_INLINE uint32_t cpu_endian_32_na_load(const void *addr)
{
  uint32_t val;

  asm ("ldr    %0, %1"
       : "=l" (val)
       : "Q" (*(uint32_t*)addr)
       );

  return val;
}

#endif

#endif

