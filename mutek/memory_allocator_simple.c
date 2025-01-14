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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006
              Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

#include <mutek/mem_alloc.h>
#include <string.h>
#include <hexo/bit.h>

struct memory_allocator_region_s
{
  lock_t		lock;
  void			*next;
  void			*last;
};

size_t memory_allocator_region_size(struct memory_allocator_region_s *region)
{
  return 0;
}

void* memory_allocator_region_address(struct memory_allocator_region_s *region)
{
  return NULL;
}


struct memory_allocator_region_s *default_region;
/***************** Memory allocation interface ******************/

void *memory_allocator_pop(struct memory_allocator_region_s *region, size_t size, size_t align)
{
  void *next, *res;

  if (align < CONFIG_MUTEK_MEMALLOC_ALIGN)
    align = CONFIG_MUTEK_MEMALLOC_ALIGN;

  size = align_pow2_up(size, align);

  lock_spin(&region->lock);

  res = address_align_up((size_t*)region->next + 1, align);
  next = (uint8_t*)res + size;

  if (next <= region->last)
    {
      /* set a single word header to keep track of allocated size */
      ((size_t*)res)[-1] = size;
      region->next = next;
    }
  else
    {
      res = NULL;
    }

  lock_release(&region->lock);

#ifdef CONFIG_MUTEK_MEMALLOC_SCRAMBLE
  if (res)
    memset(res, 0x5a, size);
#endif

  return res;
}

struct memory_allocator_region_s *
memory_allocator_init(struct memory_allocator_region_s *container_region,
			   void *start, void *end)
{
  struct memory_allocator_region_s *region;

  start = address_align_up(start, CONFIG_MUTEK_MEMALLOC_ALIGN);
  end = address_align_down(end, CONFIG_MUTEK_MEMALLOC_ALIGN);

  if (container_region == NULL)
    {
      region = start;
      start = start + sizeof( struct memory_allocator_region_s );
    }
  else
    {
      region = memory_allocator_pop (container_region, sizeof (struct memory_allocator_region_s),
                                     CONFIG_MUTEK_MEMALLOC_ALIGN);
    }

  lock_init(&region->lock);

  region->next = start;
  region->last = end;
  
  return region;
}

size_t memory_allocator_getsize(void *ptr)
{
  return ((size_t*)ptr)[-1];
}

void *memory_allocator_reserve(struct memory_allocator_region_s *region, void *start, size_t size)
{
  return NULL;
}

struct memory_allocator_header_s *
memory_allocator_extend(struct memory_allocator_region_s *region, void *start, size_t size)
{
  return NULL;
}

void *memory_allocator_resize(void *address, size_t size)
{
  return NULL;
}


void memory_allocator_push(void *ptr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_SCRAMBLE
  memset(ptr, 0xa5, memory_allocator_getsize(ptr));
#endif
}

error_t memory_allocator_stats(struct memory_allocator_region_s *region,
			size_t *alloc_blocks,
			size_t *free_size,
			size_t *free_blocks)
{
  return 0;
}

bool_t memory_allocator_guard_check(struct memory_allocator_region_s *region)
{
  return 0;
}

void memory_allocator_region_check(struct memory_allocator_region_s *region)
{
}
