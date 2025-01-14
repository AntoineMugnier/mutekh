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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

#include <mutek/mem_alloc.h>
#include <mutek/mem_alloc.h>
#include <mutek/mem_region.h>

#include <mutek/printk.h>
#include <string.h>
#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/endian.h>
#include <hexo/local.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct/container_slist.h>

GCT_CONTAINER_TYPES(region_list, struct mem_region_s *, list_entry);
GCT_CONTAINER_KEY_TYPES(region_list, PTR, SCALAR, priority);

GCT_CONTAINER_FCNS(region_list, static inline, region_list,
                   init, destroy, wrlock, unlock);
GCT_CONTAINER_NOLOCK_FCNS(region_list, static inline, region_list_nolock,
                   push, head, next, remove);
GCT_CONTAINER_KEY_NOLOCK_FCNS(region_list, ASC, static inline, region_priority, priority,
                   sort);

CPU_LOCAL region_list_root_t region_root[mem_scope_e_count];

void mem_region_id_init(cpu_id_t cpu_id)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ID_ADDR (cpu_id, region_root);
  uint_fast16_t i;
  
  printk("MemRegion: root lists init for cpu %d\n",cpu_id);
  for (i=0; i<mem_scope_e_count; i++)
    {
      region_list_init(&root[i]);
    }
}

void mem_region_add(enum mem_scope_e scope, 
		    struct memory_allocator_region_s *region, 
		    uint_fast16_t priority)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ADDR (region_root);
  struct mem_region_s *region_item = memory_allocator_pop (region,
    sizeof(struct mem_region_s), CONFIG_MUTEK_MEMALLOC_ALIGN);

  printk("MemRegion: Add the region %x to the scope %d\n",region,scope);
  region_item->priority = priority;
  region_item->region = region;

  region_list_wrlock (&root[scope]);

  region_list_nolock_push (&root[scope], region_item);	
  region_priority_sort (&root[scope]);

  region_list_unlock (&root[scope]);

}

void mem_region_id_add(cpu_id_t cpu_id,
		    enum mem_scope_e scope,
		    struct memory_allocator_region_s *region,
		    uint_fast16_t priority)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ID_ADDR (cpu_id, region_root);
  struct mem_region_s *region_item = memory_allocator_pop (region ,
    sizeof(struct mem_region_s), CONFIG_MUTEK_MEMALLOC_ALIGN);

  printk("MemRegion: Add the region %x to the scope %d for cpu %d\n",region, scope, cpu_id);

  region_item->priority = priority;
  region_item->region = region;

  region_list_wrlock (&root[scope]);

  region_list_nolock_push (&root[scope], region_item);	
  region_priority_sort (&root[scope]);

  region_list_unlock (&root[scope]);

}

struct mem_region_s *mem_region_get_first(enum mem_scope_e scope)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ADDR (region_root);
  //  printk("MemRegion: Get the first region in the scope %d\n",scope);
  return region_list_nolock_head (&root[scope]);
}

struct mem_region_s *mem_region_id_get_first(cpu_id_t cpu_id, enum mem_scope_e scope)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ID_ADDR (cpu_id, region_root);
  //  printk("MemRegion: Get the first region in the scope %d\n",scope);
  return region_list_nolock_head (&root[scope]);
}

struct mem_region_s *mem_region_get_next(enum mem_scope_e scope, struct mem_region_s *region_item)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ADDR (region_root);
  //  printk("MemRegion: Get the next region of %x in the scope %d\n",region_item, scope);
  return region_list_nolock_next (&root[scope], region_item);
}

struct mem_region_s *mem_region_id_get_next(cpu_id_t cpu_id, enum mem_scope_e scope, struct mem_region_s *region_item)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ID_ADDR (cpu_id, region_root);
  //  printk("MemRegion: Get the next region of %x in the scope %d\n",region_item, scope);
  return region_list_nolock_next (&root[scope], region_item);
}

void mem_region_remove(enum mem_scope_e scope, struct memory_allocator_region_s *region)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ADDR (region_root);

  GCT_FOREACH_WRLOCK(region_list, &root[scope], item, {
      if (item->region == region)
	{
	  region_list_nolock_remove (&root[scope], item);
	  memory_allocator_push(item);
	  GCT_FOREACH_BREAK;
	}
    });
}

void mem_region_id_remove(cpu_id_t cpu_id,
			  enum mem_scope_e scope,
			  struct memory_allocator_region_s *region)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ID_ADDR (cpu_id, region_root);

  GCT_FOREACH_WRLOCK(region_list, &root[scope], item, {
      if (item->region == region)
	{
	  region_list_nolock_remove (&root[scope], item);
	  memory_allocator_push(item);
	  GCT_FOREACH_BREAK;
	}
    });
}

void mem_region_lock(enum mem_scope_e scope)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ADDR (region_root);
  //  printk("MemRegion: lock of the scope %d\n",scope);
  region_list_wrlock(&root[scope]);
}

void mem_region_id_lock(cpu_id_t cpu_id, enum mem_scope_e scope)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ID_ADDR (cpu_id, region_root);
  //  printk("MemRegion: lock of the scope %d\n",scope);
  region_list_wrlock(&root[scope]);
}

void mem_region_unlock(enum mem_scope_e scope)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ADDR (region_root);
  //  printk("MemRegion: unlock of the scope %d\n",scope);
  region_list_unlock(&root[scope]);
}

void mem_region_id_unlock(cpu_id_t cpu_id, enum mem_scope_e scope)
{
  region_list_root_t *root = (region_list_root_t *)CPU_LOCAL_ID_ADDR (cpu_id, region_root);
  //  printk("MemRegion: unlock of the scope %d\n",scope);
  region_list_unlock(&root[scope]);
}
