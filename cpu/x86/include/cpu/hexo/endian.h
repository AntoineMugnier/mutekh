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

#if !defined(__ENDIAN_H_) || defined(CPU_ENDIAN_H_)
#error This file can not be included directly
#else

#define CPU_ENDIAN_H_

#define HAS_CPU_ENDIAN_SWAP16

ALWAYS_INLINE uint16_t cpu_endian_swap16(uint16_t x)
{
  asm ("rolw	$8, %0"
       : "=r" (x)
       : "0" (x)
       );

  return x;
}

#define HAS_CPU_ENDIAN_SWAP32

ALWAYS_INLINE uint32_t cpu_endian_swap32(uint32_t x)
{
  asm ("bswap	%0"
       : "=r" (x)
       : "0" (x)
       );

  return x;
}

#endif

