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

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define CPU_CPU_H_

static inline bool_t
cpu_isbootstrap(void)
{
  reg_t		reg;

  asm volatile (
		"mfc0	%0,	$15		\n"
		: "=r" (reg)
		);

  return (reg & 0x000003ff) == 0;
}

/**
   cpu cycle touner type
*/

typedef uint32_t cpu_cycle_t;

/**
   cpu cycle counter read function
*/

static inline cpu_cycle_t
cpu_cycle_count(void)
{
  uint32_t      result;

  asm volatile(
	       "mfc0	%0, $9"
	       : "=r" (result)
	       );

  return result;
}

#endif

