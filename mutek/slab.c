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
    slab_group_list_init(&slab->group_list);
    slab_unit_list_init(&slab->unit_list);

    slab->unit_size = ALIGN_VALUE_UP(unit_size, sizeof(slab_unit_list_entry_t));
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
}

void *slab_nolock_grow(struct slab_s *slab)
{
    struct slab_group_s *group;
    struct slab_unit_s *unit;
    size_t i;
    size_t next_count;

    next_count = slab->grow(slab, slab->current_count);

    if (next_count == 0)
        return NULL;

    group = mem_alloc(sizeof(*group) + slab->item_size * next_count,
                      slab->scope);

    if (!group)
        return NULL;

    slab_group_list_wrlock(&slab->group_list);

    slab_group_list_nolock_push(&slab->group_list, group);

    unit = (void *)(group + 1);

    for (i = 1; i < next_count; ++i)
        slab_unit_list_push(&slab->unit_list, &unit[i]);

    slab_group_list_unlock(&slab->group_list);

    return unit;
}

