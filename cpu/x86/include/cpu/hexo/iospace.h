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

   CPU dependant IO spaces access functions.
 */

#if !defined(IOSPACE_H_) || defined(CPU_IOSPACE_H_)
#error This file can not be included directly
#else

#define CPU_IOSPACE_H_

ALWAYS_INLINE void
cpu_io_write_8(uintptr_t addr, uint8_t data)
{
  __asm__ volatile (
		    "outb	%0,	%1	\n"
		    :
		    : "a" (data)
		    , "d" ((uint16_t)addr)
		    );
}

ALWAYS_INLINE uint8_t
cpu_io_read_8(uintptr_t addr)
{
  uint8_t	data;

  __asm__ volatile (
		    "inb	%1,	%0	\n"
		    : "=a" (data)
		    : "d" ((uint16_t)addr)
		    );

  return data;
}

ALWAYS_INLINE void
cpu_io_write_16(uintptr_t addr, uint16_t data)
{
  __asm__ volatile (
		    "outw	%0,	%1	\n"
		    :
		    : "a" (data)
		    , "d" ((uint16_t)addr)
		    );
}

ALWAYS_INLINE uint16_t
cpu_io_read_16(uintptr_t addr)
{
  uint16_t	data;

  __asm__ volatile (
		    "inw	%1,	%0	\n"
		    : "=a" (data)
		    : "d" ((uint16_t)addr)
		    );

  return data;
}

ALWAYS_INLINE void
cpu_io_write_32(uintptr_t addr, uint32_t data)
{
  __asm__ volatile (
		    "outl	%0,	%1	\n"
		    :
		    : "a" (data)
		    , "d" ((uint16_t)addr)
		    );
}

ALWAYS_INLINE uint32_t
cpu_io_read_32(uintptr_t addr)
{
  uint32_t	data;

  __asm__ volatile (
		    "inl	%1,	%0	\n"
		    : "=a" (data)
		    : "d" ((uint16_t)addr)
		    );

  return data;
}

/****************************************************/

ALWAYS_INLINE void
cpu_mem_write_8(uintptr_t addr, uint8_t data)
{
  __asm__ volatile (
		    "movb	%1,	%0	\n"
		    : "=m" (*(uint8_t*)addr)
		    : "q" (data)
		    );
}

ALWAYS_INLINE uint8_t
cpu_mem_read_8(uintptr_t addr)
{
  uint8_t	data;

  __asm__ volatile (
		    "movb	%1,	%0	\n"
		    : "=q" (data)
		    : "m" (*(uint8_t*)addr)
		    );

  return data;
}

ALWAYS_INLINE void
cpu_mem_write_16(uintptr_t addr, uint16_t data)
{
  __asm__ volatile (
		    "movw	%1,	%0	\n"
		    : "=m" (*(uint16_t*)addr)
		    : "r" (data)
		    );
}

ALWAYS_INLINE uint16_t
cpu_mem_read_16(uintptr_t addr)
{
  uint16_t	data;

  __asm__ volatile (
		    "movw	%1,	%0	\n"
		    : "=r" (data)
		    : "m" (*(uint16_t*)addr)
		    );

  return data;
}

ALWAYS_INLINE uint32_t
cpu_mem_read_32(uintptr_t addr)
{
  uint32_t	data;

  __asm__ volatile (
		    "movl	%1,	%0	\n"
		    : "=r" (data)
		    : "m" (*(uint32_t*)addr)
		    );

  return data;
}


ALWAYS_INLINE void
cpu_mem_write_32(uintptr_t addr, uint32_t data)
{
  __asm__ volatile (
		    "movl	%1,	%0	\n"
		    : "=m" (*(uint32_t*)addr)
		    : "r" (data)
		    );
}

ALWAYS_INLINE uint64_t
cpu_mem_read_64(uintptr_t addr)
{
  return cpu_mem_read_32(addr) | ((uint64_t)cpu_mem_read_32(addr + 4) << 32);
}

ALWAYS_INLINE void
cpu_mem_write_64(uintptr_t addr, uint64_t data)
{
  cpu_mem_write_32(addr, data);
  cpu_mem_write_32(addr + 4, data >> 32);
}

#endif

