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


#if !defined(SEGMENT_H_) || defined(ARCH_SEGMENT_H_)
#error This file can not be included directly
#else

#define ARCH_SEGMENT_H_

#include "mutek/alloc.h"
#include "string.h"

/* cpu template segment load address defined in ld script*/
extern char __cpu_data_start, __cpu_data_end;

static inline void *
arch_cpudata_alloc(void)
{
  void			*cls;

  /* allocate memory and copy from template */
  if ((cls = mem_alloc(&__cpu_data_end - &__cpu_data_start, MEM_SCOPE_SYS)))
    {
      memcpy(cls, &__cpu_data_start, &__cpu_data_end - &__cpu_data_start);
    }

  return cls;
}

/* task template segment load address defined in ld script*/
extern char __task_data_start, __task_data_end;

static inline void *
arch_taskdata_alloc(void)
{
  void			*tls;

  /* allocate memory and copy from template */
  if ((tls = mem_alloc(&__cpu_data_end - &__cpu_data_start, MEM_SCOPE_SYS)))
    {
      memcpy(tls, &__task_data_start, &__task_data_end - &__task_data_start);
    }

  return tls;
}

static inline void
arch_taskdata_free(void *ptr)
{
  mem_free(ptr);
}




static inline void *
arch_taskstack_alloc(size_t size)
{
  return mem_alloc(size, MEM_SCOPE_SYS);
}

static inline void
arch_taskstack_free(void *ptr)
{
  mem_free(ptr);
}



#endif

