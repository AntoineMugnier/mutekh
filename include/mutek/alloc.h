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

#include "types.h"

/** allocated memory scope is system local */
#define MEM_SCOPE_SYS		1
/** allocated memory scope is cpu local */
#define MEM_SCOPE_CPU		2
/** allocated memory scope is thread local */
#define MEM_SCOPE_THREAD	4

/** allocate memory */
void * mem_alloc(size_t size, uint_fast8_t scope);
/** free memory pointer */
void mem_free(void *ptr);

#endif

