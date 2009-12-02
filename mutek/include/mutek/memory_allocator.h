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


#ifndef MEMORY_ALLOCATOR_H_ 
#define MEMORY_ALLOCATOR_H_

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>

struct memory_allocator_region_s;

/** @this initialize a memory region*/
struct memory_allocator_region_s *
memory_allocator_init(struct memory_allocator_region_s *container_region, void *start, void *end);

/** @this extend an existing memory region with a new memory space */
struct memory_allocator_header_s *
memory_allocator_extend(struct memory_allocator_region_s *region, void *start, size_t size);

/** @this allocate a new memory block in given region */
void *memory_allocator_pop(struct memory_allocator_region_s *region, size_t size);

/** @this free allocated memory block */
void memory_allocator_push(void *address);

/** @this reserve a memory space in given region */
void *memory_allocator_reserve(struct memory_allocator_region_s *region, void *start, size_t size);

/** @this return the size of given memory block */
size_t memory_allocator_getsize(void *ptr);

/** @this */
error_t memory_allocator_stats(struct memory_allocator_region_s *region,
			size_t *alloc_blocks,
			size_t *free_size,
			size_t *free_blocks);

/** @this */
bool_t memory_allocator_guard_check(struct memory_allocator_region_s *region);

#endif
