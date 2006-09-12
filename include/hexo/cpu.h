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

#ifndef __CPU_H_
#define __CPU_H_

#include "types.h"

/** Cpu Local Descriptor pointer type */
struct cpu_cld_s;

/** init system wide cpu data */
error_t cpu_global_init(void);

/** send hardware reset/init signal to non first CPUs */
void cpu_start_other_cpu(void);

/** Setup CPU specific data */
struct cpu_cld_s *cpu_init(uint_fast8_t cpu_id);

/** return CPU id number, only available after arch_init() */
uint_fast8_t cpu_id(void);

/** return true if bootstap processor */
static inline bool_t cpu_isbootstrap(void);

/** return total cpus count */
uint_fast8_t arch_get_cpu_count(void);

/** unlock non first CPUs so that they can enter main_smp() */
void arch_start_other_cpu(void);

#include "cpu/hexo/cpu.h"

#endif

