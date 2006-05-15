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


#ifndef INIT_H_
#define INIT_H_

#include "types.h"
#include "error.h"

/** boot address, (located in cpu/current/boot.S) */
void cpu_boot();

/** plateform dependant entry point. (located in arch/current/arch_init.c) */
void arch_init();

/** MutekH main function (located in main/main.c) */
int_fast8_t mutek_main(int_fast8_t argc, char **argv);

/** MutekH main function for non first CPU (located in main/main.c) */
void mutek_main_smp(void);

/** Cpu Local Descriptor pointer type */
struct cpu_cld_s;

/** init system wide cpu data */
error_t cpu_global_init(void);

/** send hardware reset/init signal to non first CPUs */
void cpu_start_other_cpu(void);

/** Setup CPU specific data */
struct cpu_cld_s *cpu_init(uint_fast8_t cpu_id);

/** return CPU id number */
uint_fast8_t cpu_id(void);

/** return total cpus count */
uint_fast8_t arch_get_cpu_count(void);

/** unlock non first CPUs so that they can enter main_smp() */
void arch_start_other_cpu(void);

#endif

