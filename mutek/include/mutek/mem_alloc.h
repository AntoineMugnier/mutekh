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
    Copyright Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

/**
 * @file
 * @module{Mutek}
 * @short Memory allocation stuff
 */


#ifndef MEM_ALLOC_H_ 
#define MEM_ALLOC_H_

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>

struct mem_alloc_region_s;

enum mem_scope_e
  {
    mem_scope_sys,
    mem_scope_cluster,
    mem_scope_context,
    mem_scope_cpu,
    mem_scope_default,
  };

void mem_alloc_region_init(struct mem_alloc_region_s *region, void *address, void *end);

void *mem_alloc(size_t size, enum mem_scope_e scope);

void mem_free(void *ptr);

void *mem_reserve(struct mem_alloc_region_s *region, void *start, size_t size);

size_t mem_alloc_getsize(void *ptr);

/*********************************/

/** @this returns the farless memory allocatable region, depending to the scope */
struct mem_alloc_region_s *mem_region_get_scope(enum mem_scope_e scope);

/** @this sets the region corresponding to a scope */
void mem_region_set_scope(enum mem_scope_e scope, struct mem_alloc_region_s *region);

#if ( defined(CONFIG_HEXO_DEVICE_TREE) && defined(CONFIG_FDT) )
/** @this initializes memory allocatable regions, exclude the sys region region. */
void mem_region_init(struct device_s *root, void *blob);
#else
void mem_region_init();
#endif

/** @this creates and initializes memory allocatable region. used by mem_regions_init. */
struct mem_alloc_region_s *mem_region_create(uintptr_t start, uintptr_t end, bool_t cached); 

#endif
