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


#ifndef ARCH_ALLOC_H_
#define ARCH_ALLOC_H_

#include <hexo/local.h>
#include <hexo/types.h>

/***************** Memory allocation interface ******************/

/** allocated memory scope is system local */
#define MEM_SCOPE_SYS		1
/** allocated memory scope is cpu local */
#define MEM_SCOPE_CPU		2
/** allocated memory scope is cpu cluster local */
#define MEM_SCOPE_CLUSTER	4
/** allocated memory scope is context local */
#define MEM_SCOPE_CONTEXT	8
/** use current default allocation policy */
#define MEM_SCOPE_DEFAULT	16

extern CONTEXT_LOCAL uint_fast8_t	mem_alloc_policy;

/** set default allocation policy */
void mem_alloc_set_policy(uint_fast8_t policy);

/** allocate memory */
void * mem_alloc(size_t size, uint_fast8_t scope);

/** free memory pointer */
void mem_free(void *ptr);

/***************** Memory allocatable region management ******************/


struct mem_alloc_header_s
{
  uintptr_t				size;
  union {
    uint8_t				data[0];
  };
};

struct mem_alloc_region_s
{
  
};

void mem_alloc_region_init();

#endif

