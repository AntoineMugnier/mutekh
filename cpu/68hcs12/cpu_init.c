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

#include <hexo/interrupt.h>
#include <hexo/types.h>
#include <hexo/error.h>
#include <string.h>

CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_hw_handler;
CPU_LOCAL cpu_exception_handler_t  *cpu_interrupt_ex_handler;
CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_sys_handler;

/** pointer to context local storage in cpu local storage */
CPU_LOCAL void *__cpu_context_data_base;

extern __ldscript_symbol_t __data_start;
extern __ldscript_symbol_t __data_end;
extern __ldscript_symbol_t __data_load_start;
extern __ldscript_symbol_t __bss_start;
extern __ldscript_symbol_t __bss_end;

error_t
cpu_global_init(void)
{
  return 0;
}

struct cpu_cld_s *cpu_init(uint_fast8_t cpu_id)
{
  return NULL;
}

void cpu_start_other_cpu(void)
{
}

uint_fast8_t cpu_id(void)
{
  return 0;
}

void init_data_and_bss(void)
{
  memcpy((char *)&__data_start, (char *)&__data_load_start, (char *)&__data_end - (char *)&__data_start);
  memset((char *)&__bss_start, 0, (char *)&__bss_end - (char *)&__bss_start);
}
