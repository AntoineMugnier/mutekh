/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2014
*/

/**
 @file
 @module{Mutek}
 @short Slab allocator

 Slab allocator is a specialized memory allocator able to efficiently
 allocate buffers of fixed size.
*/

#ifndef SLAB_H_ 
#define SLAB_H_

#include <hexo/types.h>
#include <hexo/error.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/container_slist.h>

/** @internal */
#define GCT_CONTAINER_ALGO_slab_unit_list SLIST
/** @internal */
#define GCT_CONTAINER_ALGO_slab_group_list SLIST
/** @internal */
#define GCT_CONTAINER_LOCK_slab_group_list HEXO_LOCK_IRQ

/** @internal */
struct slab_unit_s {
    GCT_CONTAINER_ENTRY(slab_unit_list, entry);
};

/** @internal */
GCT_CONTAINER_TYPES(slab_unit_list, struct slab_unit_s *, entry);

/** @internal */
struct slab_group_s {
    GCT_CONTAINER_ENTRY(slab_group_list, entry);
};

/** @internal */
GCT_CONTAINER_TYPES(slab_group_list, struct slab_group_s *, entry);

/**
   @this defines the grow policy function prototype for a slab.

   This function is called every time the slab needs to grow.

   @param slab Slab to grow for
   @param current Current allocator item count
   @returns count of items to allocate next
 */
#define SLAB_GROW(f) size_t (f)(struct slab_s *slab, size_t current)

typedef SLAB_GROW(slab_grow_func_t);

struct slab_s {
    slab_group_list_root_t group_list;
    slab_unit_list_root_t unit_list;
    size_t unit_size;
    size_t current_count;
    slab_grow_func_t grow;
    enum mem_scope_e scope;
};

GCT_CONTAINER_FCNS(slab_unit_list, static inline, slab_unit_list,
                   init, destroy, push, pop);

GCT_CONTAINER_FCNS(slab_group_list, static inline, slab_group_list,
                   init, destroy, push, pop, wrlock, unlock);

/**
   @this initializes a new slab.

   @param slab Slab structure to initialize
   @param unit_size Size for each allocated item
   @param grow Grow policy function
   @param scope Memory allocator scope
 */
void slab_init(
  struct slab_s *slab,
  size_t unit_size,
  slab_allocator_grow_func_t grow,
  enum mem_scope_e scope);

/**
   @this clears the slab.

   After calling this function, you may not use any allocated items
   any more.

   @param slab Slab to destroy
 */
void slab_cleanup(struct slab_s *slab);

/** @internal
    Grow slab and return one allocated item
*/
void *slab_nolock_grow(struct slab_s *slab);

/**
   @this allocates one element in the slab.

   @param slab The slab allocator
   @returns a newly allocated item
 */
static inline
void *slab_alloc(struct slab_s *slab)
{
    struct slab_unit_s *unit;

    slab_group_list_wrlock(&slab->group_list);

    unit = slab_unit_list_pop(&slab->unit_list);

    if (!unit)
        unit = slab_nolock_grow(slab);

    slab_group_list_unlock(&slab->group_list);

    return unit;
}

/**
   @this deallocs one element in the slab.

   @param slab Slab the pointer belongs to
   @param ptr Pointer to deallocate
 */
static inline
void slab_free(struct slab_s *slab, void *ptr)
{
    struct slab_unit_s *unit = ptr;

    slab_unit_list_push(&slab->unit_list, unit);
}

#endif
