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
#include "error.h"

/** init system wide cpu data */
error_t cpu_global_init(void);

/** send hardware reset/init signal to non first CPUs */
void cpu_start_other_cpu(void);

/** Setup CPU specific data */
void cpu_init(void);

/** get cpu local storage */
static void *cpu_get_cls(cpu_id_t cpu_id);

/** return CPU id number */
static cpu_id_t cpu_id(void);

/** return true if bootstap processor */
static bool_t cpu_isbootstrap(void);

/** return total cpus count */
cpu_id_t arch_get_cpu_count(void);

/** unlock non first CPUs so that they can enter main_smp() */
void arch_start_other_cpu(void);

#include "cpu/hexo/cpu.h"

cpu_cycle_t cpu_cycle_count(void);

/** cpu trap instruction */
void cpu_trap();

/** get cpu cache line size, return 0 if no dcache */
static size_t cpu_dcache_line_size();

/** invalidate the cpu data cache line containing this address */
static void cpu_dcache_invld(void *ptr);

/** invalidate all the cpu data cache lines within given range.
    size is in bytes. */
void cpu_dcache_invld_buf(void *ptr, size_t size);

#endif

