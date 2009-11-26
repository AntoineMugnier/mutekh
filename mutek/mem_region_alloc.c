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

    Copyright Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <string.h>
#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/endian.h>
#include "memalloc.h"

# if defined(CONFIG_MUTEK_MEMALLOC_SMART)

#include <gpct/cont_dlist.h>
#include <device/device.h>
#include <device/mem.h>
#include <device/driver.h>



struct mem_region_s
{
  CONTAINER_ENTRY_TYPE(DLIST) list_entry;
  uint_fast16_t cluster_id;
  struct mem_alloc_region_s *region;
};

CPU_LOCAL struct mem_alloc_region_s *local_cached_region;
CPU_LOCAL struct mem_alloc_region_s *local_uncached_region;

CPU_LOCAL struct mem_region_s *cached_region_queue;
CPU_LOCAL struct mem_region_s *uncached_region_queue;

CONTAINER_TYPE(region_queue, DLIST, struct mem_region_s, list_entry);
CONTAINER_KEY_TYPE(region_queue, SCALAR, cluster_id);
CONTAINER_FUNC(region_queue, DLIST, static inline, region_queue);
CONTAINER_KEY_FUNC(region_queue, DLIST, static inline, region_queue_id, cluster_id);

region_queue_root_t region_cached_list, region_uncached_list ;


#if ( defined(CONFIG_ARCH_DEVICE_TREE) && defined(CONFIG_FDT) )

#include <fdt/reader.h>

static inline void setif(enum mem_scope_e scope, struct mem_region_s *region)
{
	if ( region )
		mem_region_set_scope(scope, region->region);
	else
		mem_region_set_scope(scope, &mem_region_system);
}

void mem_region_init(struct device_s *root, void *blob)
{
  struct dev_mem_info_s mem_info;
  uint64_t addr=0;
  uint64_t size=0;
  uint_fast16_t i=0;

  region_queue_init(&region_cached_list);
  region_queue_init(&region_uncached_list);

  CONTAINER_FOREACH(device_list, CLIST, &root->children, {

      if (item->drv == NULL)
	CONTAINER_FOREACH_CONTINUE;

      if (item->drv->class == device_class_mem ){
	dev_mem_get_info(item, &mem_info);

	struct mem_region_s *region_item;

	/* looking for reserve memory space in the fdt: */
	/* First pass: the border memory space*/
	i=0;
	fdt_get_rsvmap(blob, i, &addr, &size);	
	
	while( ( addr != 0 || size != 0 ) )
	  {
	    addr -= mem_hdr_size;
	    size += mem_hdr_size;
	    if( addr <= mem_info.base + mem_hdr_size + sizeof(struct mem_alloc_region_s) &&
		addr+size > mem_info.base + mem_hdr_size + sizeof(struct mem_alloc_region_s) )
	      {
		if( addr + size >= mem_info.base + mem_info.size - ( mem_hdr_size + sizeof(struct mem_alloc_region_s) ) )
		  {/*all the region is reserve*/
		    printk("Memory region reserved at %x with a size of %x\n", mem_info.base, mem_info.size);
		    CONTAINER_FOREACH_CONTINUE;
		  }
		mem_info.size -= (addr + size) - mem_info.base;
		mem_info.base = (addr + size);
	      }
	    else if( addr < (mem_info.base + mem_info.size)  &&
		     addr+size >= (mem_info.base + mem_info.size - ( mem_hdr_size + sizeof(struct mem_alloc_region_s) ) ) )
	      {
		mem_info.size -= (mem_info.base + mem_info.size) - addr;
	      }
	    i++;
	    fdt_get_rsvmap(blob, i, &addr, &size);
	  }

	/* Check if data section, systeme scope and stack are in this region*/
	if ( mem_info.base <= CONFIG_RAM_ADDR &&
	     mem_info.base + mem_info.size > CONFIG_RAM_ADDR)
	  {
	    if ( CONFIG_RAM_ADDR > mem_info.base + mem_hdr_size )
	      {
		printk("Extend system scope: add %d bytes which start at %x\n",
		       CONFIG_RAM_ADDR - mem_info.base,
		       mem_info.base);

		mem_alloc_region_extend( mem_region_get_scope( mem_scope_sys ) ,
					 (void *)mem_info.base,
					 CONFIG_RAM_ADDR - mem_info.base );
	      }
	    if ( CONFIG_RAM_ADDR + CONFIG_RAM_SIZE < mem_info.base + mem_info.size - mem_hdr_size )
	      {
		printk("Extend system scope: add %d bytes which start at %x\n",
		       ( mem_info.base + mem_info.size ) - ( CONFIG_RAM_ADDR + CONFIG_RAM_SIZE ),
		       ( CONFIG_RAM_ADDR + CONFIG_RAM_SIZE ) );

		mem_alloc_region_extend( mem_region_get_scope( mem_scope_sys ) , 
					 (void *)( CONFIG_RAM_ADDR + CONFIG_RAM_SIZE ), 
					 ( mem_info.base + mem_info.size ) - ( CONFIG_RAM_ADDR + CONFIG_RAM_SIZE ) );
	      }
	    CONTAINER_FOREACH_CONTINUE;
	  }

	/* looking for the clusterid in the topology*/
	//TODO
	/* when the start and the end is well know, create the memory region */
	region_item = mem_alloc( sizeof(struct mem_region_s), mem_scope_sys );
	region_item->region = mem_region_create(mem_info.base, 
						mem_info.base + mem_info.size, 
						mem_info.flags & DEV_MEM_CACHED);

	printk("Memory region created at %x with a size of %x. ",mem_info.base, mem_info.size);
	if (mem_info.flags & DEV_MEM_CACHED)
	  printk("This region is cached\n");
	else
	  printk("This region is uncached\n");

	/* Second pass: the reserve memory space into the region*/
	i = 0;
	fdt_get_rsvmap(blob, i, &addr, &size);
	while( !( addr == 0 && size == 0 ) )
	  {
	    addr -= mem_hdr_size;
	    size += mem_hdr_size;
	    if( addr > mem_info.base && 
		addr+size < mem_info.base+mem_info.size )
	      {
		mem_reserve( region_item->region , (void*)(uint32_t)addr, size);
		printk("Memory reservation at %x with a size of %x. ",
		       (uint32_t)addr,
		       (uint32_t)size);
		printk("This reservation is done in the region %x\n",
		       mem_info.base);
	      }
	    i++;
	    fdt_get_rsvmap(blob, i, &addr, &size);
	  }
	
	/* Add the region to the corresponding list*/
	if(mem_info.flags & DEV_MEM_CACHED)
	  region_queue_push(&region_cached_list, region_item);
	else
	  region_queue_push(&region_uncached_list, region_item);
      }
    });
  
  region_queue_id_sort_ascend(&region_cached_list);
  region_queue_id_sort_ascend(&region_uncached_list);
  
  /**/
  /*FIXME: add CLUSTER suport, test if a cached and a uncached regions exist*/  
  if ( region_queue_count(&region_uncached_list) >= 1 )
    mem_region_set_scope(mem_scope_cluster,
			 region_queue_head(&region_uncached_list)->region );
  else
    mem_region_set_scope( mem_scope_cluster, &mem_region_system );

  if ( region_queue_count( &region_uncached_list ) >= 1 ) 
    {
      mem_region_set_scope(mem_scope_context,
			   region_queue_head(&region_cached_list)->region );
      mem_region_set_scope(mem_scope_cpu,
			   region_queue_head(&region_cached_list)->region );
    }
  else
    {
      mem_region_set_scope( mem_scope_context, &mem_region_system );
      mem_region_set_scope( mem_scope_cpu, &mem_region_system );
    }
  
#if defined(CONFIG_SMP)
  //FIXME: Set the scope for other cpus  
#endif

}

#endif /*CONFIG_DEVICE_TREE*/

struct mem_alloc_region_s *mem_region_create(uintptr_t start, uintptr_t end, bool_t cached)
{
  struct mem_alloc_region_s *region;

  if( cached )
    region = mem_alloc( sizeof( struct mem_alloc_region_s ), mem_scope_sys );
  else
    {
      region = (struct mem_alloc_region_s *)start;
      start += sizeof( struct mem_alloc_region_s );
    }
  mem_alloc_region_init( region, (void *)start, (void *)end );

  return region;
}

void mem_region_set_scope(enum mem_scope_e scope, struct mem_alloc_region_s *region)
{
  switch( scope )
    {
    case mem_scope_cluster:
      CPU_LOCAL_SET(local_uncached_region, region);
      break;

    case mem_scope_context:
    case mem_scope_cpu: 
      CPU_LOCAL_SET(local_cached_region, region);
    case mem_scope_sys:
    case mem_scope_default:
    default:
      break;
    }
}

struct mem_alloc_region_s *mem_region_get_scope(enum mem_scope_e scope)
{
  switch( scope )
    {
    case mem_scope_cluster:
      return CPU_LOCAL_GET(local_uncached_region);

    case mem_scope_context:
    case mem_scope_cpu:
      return CPU_LOCAL_GET(local_cached_region);

    case mem_scope_sys:
    case mem_scope_default:
    default:
      break;
    }
  return &mem_region_system;
}

static inline
struct mem_region_s *get_local_item(enum mem_scope_e scope, region_queue_root_t **region_list)
{
  switch( scope )
    {
    case mem_scope_cluster:
      *region_list = &region_uncached_list;
      return CPU_LOCAL_GET(uncached_region_queue);

    case mem_scope_context:
    case mem_scope_cpu:
      *region_list = &region_cached_list;
      return CPU_LOCAL_GET(cached_region_queue);

    case mem_scope_sys:
    case mem_scope_default:
    default: 
      *region_list = NULL;
      return NULL;

    
    }
  
}

#else /*CONFIG_MUTEK_MEMALLOC_SMART*/

void mem_region_init(struct device_s *root, void *blob)
{
}

void mem_region_set_scope(enum mem_scope_e scope, struct mem_alloc_region_s *region)
{
}

struct mem_alloc_region_s *mem_region_get_scope(enum mem_scope_e scope)
{
  return &mem_region_system;
}

struct mem_alloc_region_s *mem_region_create(uintptr_t start, uintptr_t end, bool_t cached)
{
  return NULL;
}

#endif /*CONFIG_MUTEK_MEMALLOC_SMART*/
