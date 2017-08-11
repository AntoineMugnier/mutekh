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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2017

*/

#if !defined(__BIT_H_) || defined(__CPU_BIT_H_)
#error This file can not be included directly
#else

#define __CPU_BIT_H_

#define HAS_CPU_BIT_CLZ32
ALWAYS_INLINE uint32_t cpu_bit_clz32(uint32_t x)
{
  uint32_t r;

  asm ("bsrl	%1, %0"
       : "=r" (r)
       : "r" (x)
       );

  return 31 - r;
}

#define HAS_CPU_BIT_CTZ32
ALWAYS_INLINE uint32_t cpu_bit_ctz32(uint32_t x)
{
  uint32_t r;

  asm ("bsfl	%1, %0"
       : "=r" (r)
       : "r" (x)
       );

  return r;
}

#define HAS_CPU_BIT_CLZ64
ALWAYS_INLINE uint64_t cpu_bit_clz64(uint64_t x)
{
  uint64_t r;

  asm ("bsrq	%1, %0"
       : "=r" (r)
       : "r" (x)
       );

  return 63 - r;
}

#define HAS_CPU_BIT_CTZ64
ALWAYS_INLINE uint64_t cpu_bit_ctz64(uint64_t x)
{
  uint64_t r;

  asm ("bsfq	%1, %0"
       : "=r" (r)
       : "r" (x)
       );

  return r;
}

#endif
