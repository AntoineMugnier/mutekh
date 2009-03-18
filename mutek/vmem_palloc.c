
#include <mutek/vmem_palloc.h>

#include <hexo/endian.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>

//FIXME: add support for non contiguous memory segments

#define VMEM_PPAGE_ISFREE(x) ((x) & 0x80000000)
#define VMEM_PPAGE_VALUE(x) ((x) & 0x7fffffff)
#define VMEM_PPAGE_SET(isfree, value) ((isfree << 31) | (value))

error_t vmem_ppage_region_init(struct vmem_page_region_s *r,
			       uintptr_t paddr, uintptr_t paddr_end)
{
  uint_fast32_t i;

  paddr = ALIGN_VALUE_UP(paddr, CONFIG_HEXO_MMU_PAGESIZE);
  paddr_end = ALIGN_VALUE_LOW(paddr_end, CONFIG_HEXO_MMU_PAGESIZE);

  assert(paddr_end > paddr);

  if (lock_init(&r->lock))
    goto err;

  r->paddr = paddr;
  r->size = paddr_end - paddr;
  r->free_count = r->count = r->size / CONFIG_HEXO_MMU_PAGESIZE;
  r->free_head = 0;
  r->table = mem_alloc(r->count * sizeof (uint_fast32_t), MEM_SCOPE_SYS);

  if (!r->table)
    goto err_lock;

  for (i = 0; i < r->count; i++)
    r->table[i] = VMEM_PPAGE_SET(1, i + 1);

  r->table[r->count - 1] = VMEM_PPAGE_SET(1, 0);

  return 0;

 err_lock:
  lock_destroy(&r->lock);
 err:
  return -ENOMEM;
}


void vmem_ppage_region_destroy(struct vmem_page_region_s *r)
{
  mem_free(r->table);
  lock_destroy(&r->lock);  
}

error_t vmem_ppage_alloc(struct vmem_page_region_s *r, uintptr_t *paddr)
{
  uint_fast32_t *t;
  error_t res = -ENOMEM;

  LOCK_SPIN_IRQ(&r->lock);  

  if (r->free_count > 0)
    {
      t = r->table + r->free_head;
      *paddr = r->paddr + r->free_head * CONFIG_HEXO_MMU_PAGESIZE;
      r->free_head = VMEM_PPAGE_VALUE(*t);
      *t = VMEM_PPAGE_SET(0, 1);		/* intial refcount is 1 */
      r->free_count--;
      res = 0;
    }

  LOCK_RELEASE_IRQ(&r->lock);  

//  printf("ppage alloc %i %p\n", res, *paddr);

  return res;
}

bool_t vmem_ppage_inrange(struct vmem_page_region_s *r, uintptr_t paddr)
{
  assert(paddr % CONFIG_HEXO_MMU_PAGESIZE == 0);

  return ((paddr >= r->paddr) &&
	  (paddr < r->paddr + r->count * CONFIG_HEXO_MMU_PAGESIZE));
}

error_t vmem_ppage_reserve(struct vmem_page_region_s *r,
			   uintptr_t paddr, uintptr_t paddr_end)
{
  uint_fast32_t i, p;
  size_t size;
  error_t res = 0;

  paddr = ALIGN_VALUE_UP(paddr, CONFIG_HEXO_MMU_PAGESIZE);
  paddr_end = ALIGN_VALUE_LOW(paddr_end, CONFIG_HEXO_MMU_PAGESIZE);

  assert(paddr_end > paddr);

  size = (paddr_end - paddr) / CONFIG_HEXO_MMU_PAGESIZE;

  LOCK_SPIN_IRQ(&r->lock);

  p = (paddr - r->paddr) / CONFIG_HEXO_MMU_PAGESIZE;

  /* check if all region is free */
  for (i = p; i < p + size; i++)
    if (!VMEM_PPAGE_ISFREE(r->table[i]))
      {
	res = -ENOMEM;
	break;
      }

  if (!res)
    {
      uint_fast32_t *n, c = 0;

      for (n = &r->free_head; c < size; )
	{
	  uint_fast32_t i = VMEM_PPAGE_VALUE(*n);

	  if (i >= p && i < p + size)
	    {
	      *n = VMEM_PPAGE_VALUE(r->table[i]);
	      r->table[i] = VMEM_PPAGE_SET(0, 1);
	      c++;
	    }
	  else
	    {
	      n = &r->table[i];
	    }
	}


      r->free_count -= size;

      }

  LOCK_RELEASE_IRQ(&r->lock);  

  return res;
}

uintptr_t vmem_ppage_refnew(struct vmem_page_region_s *r, uintptr_t paddr)
{
  uint_fast32_t *t;
  uint_fast32_t p;

  LOCK_SPIN_IRQ(&r->lock);

  assert(vmem_ppage_inrange(paddr));

  p = (paddr - r->paddr) / CONFIG_HEXO_MMU_PAGESIZE;
  assert(p < r->count);
  t = r->table + p;
  assert(!t->is_free);

  (*t)++;

  LOCK_RELEASE_IRQ(&r->lock);  
  return paddr;
}

void vmem_ppage_refdrop(struct vmem_page_region_s *r, uintptr_t paddr)
{
  uint_fast32_t *t;
  uint_fast32_t p;

  LOCK_SPIN_IRQ(&r->lock);

  assert(vmem_ppage_inrange(paddr));

  p = (paddr - r->paddr) / CONFIG_HEXO_MMU_PAGESIZE;
  assert(p < r->count);
  t = r->table + p;
  assert(!VMEM_PPAGE_ISFREE(*t));
  assert(*t > 0);

  if (--(*t) == 0)
    {
      *t = VMEM_PPAGE_SET(1, r->free_head);
      r->free_head = p;
      r->free_count++;
    }

  LOCK_RELEASE_IRQ(&r->lock);  
}

