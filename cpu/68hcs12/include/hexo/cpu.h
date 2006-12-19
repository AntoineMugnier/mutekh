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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#define CPU_CPU_H_

/** general purpose regsiters count */
#define CPU_GPREG_COUNT	5

#define CPU_GPREG_NAMES { "A", "B", "X", "Y", "SP" }

static inline bool_t
cpu_isbootstrap(void)
{
  return 1;
}

typedef uint32_t cpu_cycle_t;

static inline cpu_cycle_t
__attribute__ ((deprecated))
cpu_cycle_count(void)
{
  return 0;
}

#endif
