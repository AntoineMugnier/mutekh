
#include <hexo/vmem.h>
#include <hexo/endian.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>

struct vmem_page_region_s
{
  /* physical address of memory region */
  uintptr_t			paddr;
  /* memory region size */
  size_t			size;
  /* memory region page count */
  size_t			count;
  /* memory region free page count */
  size_t			free_count;
  /* first free page */
  uint_fast32_t			free_head;
  /* page allocation table */
  struct vmem_ppage_desc_s	*table;

  lock_t			lock;
};

struct vmem_ppage_desc_s
{  
  uint_fast32_t	is_free:1;
  /* reference count of allocated page,
     or pointer to next free page */
  uint_fast32_t	value:31;
};

static struct vmem_page_region_s vmem_region;

error_t vmem_ppage_region_init(uintptr_t paddr, uintptr_t paddr_end)
{
  struct vmem_page_region_s *r = &vmem_region;
  uint_fast32_t i;

  paddr = ALIGN_VALUE_UP(paddr, CONFIG_HEXO_VMEM_PAGESIZE);
  paddr_end = ALIGN_VALUE_LOW(paddr_end, CONFIG_HEXO_VMEM_PAGESIZE);

  assert(paddr_end > paddr);

  if (lock_init(&r->lock))
    goto err;

  r->paddr = paddr;
  r->size = paddr_end - paddr;
  r->free_count = r->count = r->size / CONFIG_HEXO_VMEM_PAGESIZE;
  r->free_head = 0;
  r->table = mem_alloc(r->count * sizeof (struct vmem_ppage_desc_s), MEM_SCOPE_SYS);

  if (!r->table)
    goto err_lock;

  for (i = 0; i < r->count; i++)
    {
      r->table[i].is_free = 1;
      r->table[i].value = i + 1;
    }

  r->table[r->count - 1].value = 0;

  return 0;

 err_lock:
  lock_destroy(&r->lock);
 err:
  return -ENOMEM;}


void vmem_ppage_region_destroy()
{
  struct vmem_page_region_s *r = &vmem_region;

  mem_free(r->table);
  lock_destroy(&r->lock);  
}

error_t vmem_ppage_alloc(uintptr_t *paddr)
{
  struct vmem_page_region_s *r = &vmem_region;
  struct vmem_ppage_desc_s *t;
  error_t res = -ENOMEM;

  LOCK_SPIN_IRQ(&r->lock);  

  if (r->free_count > 0)
    {
      t = r->table + r->free_head;
      *paddr = r->paddr + r->free_head * CONFIG_HEXO_VMEM_PAGESIZE;
      r->free_head = t->value;
      t->is_free = 0;
      t->value = 1;		/* intial refcount is 1 */
      r->free_count--;
      res = 0;
    }

  LOCK_RELEASE_IRQ(&r->lock);  

  printf("ppage alloc %i %p\n", res, *paddr);

  return res;
}

bool_t vmem_ppage_inrange(uintptr_t paddr)
{
  struct vmem_page_region_s *r = &vmem_region;

  assert(paddr % CONFIG_HEXO_VMEM_PAGESIZE == 0);

  return ((paddr >= r->paddr) &&
	  (paddr < r->paddr + r->count * CONFIG_HEXO_VMEM_PAGESIZE));
}

error_t vmem_ppage_reserve(uintptr_t paddr, uintptr_t paddr_end)
{
  struct vmem_page_region_s *r = &vmem_region;
  uint_fast32_t i, p, next;
  size_t size;
  error_t res = 0;

  paddr = ALIGN_VALUE_UP(paddr, CONFIG_HEXO_VMEM_PAGESIZE);
  paddr_end = ALIGN_VALUE_LOW(paddr_end, CONFIG_HEXO_VMEM_PAGESIZE);

  assert(paddr_end > paddr);

  size = (paddr_end - paddr) / CONFIG_HEXO_VMEM_PAGESIZE;

  LOCK_SPIN_IRQ(&r->lock);

  p = (paddr - r->paddr) / CONFIG_HEXO_VMEM_PAGESIZE;

  /* check if all region is free */
  for (i = p; i < p + size; i++)
    if (!r->table[i].is_free)
      {
	res = -ENOMEM;
	break;
      }

  if (!res)
    {
      /* mark pages as allocated */
      for (i = p; i < p + size; i++)
	{
	  r->table[i].is_free = 0;
	  r->table[i].value = 1;
	}

      r->free_count -= size;

      /* refresh all free pages links */
      for (i = 0; i < r->free_count; i++)
	{
	  if (r->table[i].is_free)
	    {
	      r->table[i].value = next;
	      next = i;
	    }
	}

      r->free_head = next;
    }

  LOCK_RELEASE_IRQ(&r->lock);  

  return res;
}

uintptr_t vmem_ppage_refnew(uintptr_t paddr)
{
  struct vmem_page_region_s *r = &vmem_region;
  struct vmem_ppage_desc_s *t;
  uint_fast32_t p;

  LOCK_SPIN_IRQ(&r->lock);

  assert(vmem_ppage_inrange(paddr));

  p = (paddr - r->paddr) / CONFIG_HEXO_VMEM_PAGESIZE;
  assert(p < r->count);
  t = r->table + p;
  assert(!t->is_free);

  t->value++;

  LOCK_RELEASE_IRQ(&r->lock);  

  return paddr;
}

void vmem_ppage_refdrop(uintptr_t paddr)
{
  struct vmem_page_region_s *r = &vmem_region;
  struct vmem_ppage_desc_s *t;
  uint_fast32_t p;

  LOCK_SPIN_IRQ(&r->lock);

  assert(vmem_ppage_inrange(paddr));

  p = (paddr - r->paddr) / CONFIG_HEXO_VMEM_PAGESIZE;
  assert(p < r->count);
  t = r->table + p;
  assert(!t->is_free);
  assert(t->value > 0);

  if (--t->value == 0)
    {
      t->value = r->free_head;
      r->free_head = p;
      t->is_free = 1;
      r->free_count++;
    }

  LOCK_RELEASE_IRQ(&r->lock);  
}

