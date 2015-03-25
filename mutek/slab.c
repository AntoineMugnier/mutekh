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

#include "include/mutek/mem_alloc.h"
#include "include/mutek/slab.h"

void slab_init(
    struct slab_s *slab,
    size_t unit_size,
    slab_grow_func_t grow,
    enum mem_scope_e scope)
{
    lock_init(&slab->lock);

    slab_group_list_init(&slab->group_list);
    slab_unit_list_init(&slab->unit_list);

    slab->unit_size = ALIGN_VALUE_UP(unit_size, 8);
    slab->grow = grow;
    slab->scope = scope;
}

void slab_cleanup(struct slab_s *slab)
{
    GCT_FOREACH(slab_group_list, &slab->group_list, group, {
            //slab_group_list_remove(&slab->group_list, &group->entry);
            mem_free(group);
        });

    slab_group_list_destroy(&slab->group_list);
    slab_unit_list_destroy(&slab->unit_list);

    lock_destroy(&slab->lock);
}

void *slab_nolock_grow(struct slab_s *slab)
{
    struct slab_group_s *group;
    uintptr_t base, unit;
    size_t next_count;
    size_t unit_bytes;
    size_t group_size;

    next_count = slab->grow(slab, slab->current_count);

    if (next_count == 0)
        return NULL;

    group_size = ALIGN_VALUE_UP(sizeof(*group), 8);
    unit_bytes = slab->unit_size * next_count;
    group = mem_alloc(group_size + unit_bytes, slab->scope);

    if (!group)
        return NULL;

    slab->current_count += next_count;

    slab_group_list_push(&slab->group_list, group);

    base = (uintptr_t)group + group_size;

    for (unit = slab->unit_size; unit < unit_bytes; unit += slab->unit_size)
      slab_unit_list_push(&slab->unit_list, (struct slab_unit_s *)(base + unit));

    return (void *)base;
}

extern inline
void slab_free(struct slab_s *slab, void *ptr);
extern inline
void *slab_alloc(struct slab_s *slab);
