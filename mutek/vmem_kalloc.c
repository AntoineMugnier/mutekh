
#include <assert.h>
#include <hexo/mmu.h>
#include <hexo/endian.h>
#include <mutek/vmem_kalloc.h>
#include <mutek/page_alloc.h>

static uintptr_t next_v_page = CONFIG_HEXO_MMU_INITIAL_END;

void * vmem_vpage_kalloc(size_t count)
{
  uintptr_t paddr;
  uintptr_t vaddr;

  assert(count == 1); 		/* FIXME !!! */

  vaddr = next_v_page;
  next_v_page += CONFIG_HEXO_MMU_PAGESIZE;

  /* allocate a new physical page for page table */
  if (ppage_alloc(&paddr))
    return NULL;

  if (mmu_vpage_set(vaddr, paddr, MMU_PAGE_ATTR_RWX | MMU_PAGE_ATTR_PRESENT))
    {
      ppage_refdrop(paddr);
      return NULL;
    }

  return (void*)vaddr;
}

void vmem_vpage_kfree(void *vaddr, size_t count)
{
  uintptr_t paddr = mmu_vpage_get_paddr((uintptr_t)vaddr);

  assert(count == 1); 		/* FIXME !!! */

  ppage_refdrop(paddr);
}

uintptr_t vmem_vpage_io_map(uintptr_t paddr, size_t size)
{
  uintptr_t vaddr, res;

  if (paddr >= CONFIG_HEXO_MMU_INITIAL_START && paddr + size <= CONFIG_HEXO_MMU_INITIAL_END)
    return paddr;

  size = ALIGN_VALUE_UP(size, CONFIG_HEXO_MMU_PAGESIZE);

  res = vaddr = next_v_page;
  next_v_page += size;

  while (size)
    {
      mmu_vpage_set(vaddr, paddr, MMU_PAGE_ATTR_RW | MMU_PAGE_ATTR_PRESENT);

      paddr += CONFIG_HEXO_MMU_PAGESIZE;
      vaddr += CONFIG_HEXO_MMU_PAGESIZE;
      size -= CONFIG_HEXO_MMU_PAGESIZE;
    }

  return res;
}


