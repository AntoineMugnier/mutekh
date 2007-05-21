
#include <hexo/vmem.h>
#include <hexo/local.h>
#include <string.h>
#include <assert.h>

VMEM_X86_ALIGN cpu_x86_page_entry_t vmem_k_pagedir[1024];
VMEM_X86_ALIGN cpu_x86_page_entry_t vmem_k_mirror[1024];

struct vmem_context_s
{
  cpu_x86_page_entry_t	*pagedir; /* page directory */
  cpu_x86_page_entry_t	*mirror; /* page table mirroring page directory */
  uintptr_t		pagedir_paddr; /* page directory physical address */
  uint_fast16_t		k_count; /* kernel entries count */
  uintptr_t		v_next;	/* next virtual page to allocate */
};

VMEM_X86_ALIGN struct vmem_context_s vmem_k_context;

/* next virtual page to allocate on kernel page space */
uintptr_t vmem_k_next = CONFIG_HEXO_VMEM_INITIAL;

CPU_LOCAL struct vmem_context_s *vmem_context_cur = &vmem_k_context;

void vmem_global_init()
{
  uint_fast16_t	i;

  vmem_k_context.k_count = VMEM_INITIAL_PDE;
  vmem_k_context.v_next = CONFIG_HEXO_VMEM_INITIAL;
  vmem_k_context.pagedir = vmem_k_pagedir;
  vmem_k_context.mirror = vmem_k_mirror;
  vmem_k_context.pagedir_paddr = (uintptr_t)vmem_k_pagedir;

  memset(vmem_k_pagedir, 0, sizeof (vmem_k_pagedir));
  memset(vmem_k_mirror, 0, sizeof (vmem_k_mirror));

  /* identity map memory space with 4Mb pages */
  for (i = 0; i < VMEM_INITIAL_PDE; i++)
    {
      vmem_k_pagedir[i].p4m.present = 1;
      vmem_k_pagedir[i].p4m.writable = 1;
      vmem_k_pagedir[i].p4m.pagsize4m = 1;
      vmem_k_pagedir[i].p4m.global = 1;
      vmem_k_pagedir[i].p4m.address = i;
    }

  vmem_k_pagedir[VMEM_MIRROR_PDE].pte.present = 1;
  vmem_k_pagedir[VMEM_MIRROR_PDE].pte.writable = 1;
  vmem_k_pagedir[VMEM_MIRROR_PDE].pte.address = ((uintptr_t)vmem_k_mirror) >> 12;

  vmem_k_mirror[VMEM_MIRROR_PDE].p4k.present = 1;
  vmem_k_mirror[VMEM_MIRROR_PDE].p4k.writable = 1;
  vmem_k_mirror[VMEM_MIRROR_PDE].p4k.address = ((uintptr_t)vmem_k_mirror) >> 12;
}

void vmem_cpu_init()
{
  uint32_t tmp;

  vmem_x86_set_pagedir(vmem_k_context.pagedir_paddr);

  asm volatile (
		"mov %%cr4, %0		\n"
		"orl  $0x00000090, %0	\n" /* set PSE and PGE */
		"andl $0xffffffdf, %0	\n" /* clear PAE */
		"mov %0, %%cr4		\n"

		"mov %%cr0, %0		\n"
		"orl  $0x80000000, %0	\n" /* enable paging */
		"mov %0, %%cr0		\n"
		: "=r" (tmp)
		);
}

error_t vmem_context_init(struct vmem_context_s *ctx)
{
  cpu_x86_page_entry_t	*pagedir, *mirror;

  if (!(pagedir = vmem_vpage_kalloc()))
    goto err;

  if (!(mirror = vmem_vpage_kalloc()))
    goto err2;

  memset(ctx->pagedir, 0, VMEM_X86_PAGESIZE);
  memset(ctx->mirror, 0, VMEM_X86_PAGESIZE);

  uint32_t mirror_paddr = vmem_vpage_get_paddr((uintptr_t)mirror);
  uint32_t pagedir_paddr = vmem_vpage_get_paddr((uintptr_t)pagedir);

  ctx->pagedir = pagedir;
  ctx->mirror = mirror;
  ctx->pagedir_paddr = pagedir_paddr;
  ctx->k_count = 0;
  ctx->v_next = VMEM_USER_START_ADDR;

  /* setup page directory mirror */
  pagedir[VMEM_MIRROR_PDE].pte.present = 1;
  pagedir[VMEM_MIRROR_PDE].pte.writable = 1;
  pagedir[VMEM_MIRROR_PDE].pte.address = mirror_paddr >> 12;

  mirror[VMEM_MIRROR_PDE].p4k.present = 1;
  mirror[VMEM_MIRROR_PDE].p4k.writable = 1;
  mirror[VMEM_MIRROR_PDE].p4k.address = mirror_paddr >> 12;

  return 0;

 err2:
  vmem_vpage_kfree(pagedir);
 err:
  return -ENOMEM;
}

struct cpu_x86_page4k_entry_s *vmem_x86_get_vpage_entry(uint_fast16_t pde, uint_fast16_t pte)
{
  return (void*)(uintptr_t)(VMEM_MIRROR_ADDR | (pde << 12) | (pte << 2));
}

void vmem_context_destroy(void)
{
  struct vmem_context_s *ctx = vmem_context_get();
  uint_fast8_t i, j;

  /* refrdop all physical pages mapped in context */
  for (i = VMEM_USER_START_PDE; i <= VMEM_USER_END_PDE; i++)
    if (ctx->pagedir[i].pte.present)
      for (j = 0; j < 1024; j++)
	{
	  struct cpu_x86_page4k_entry_s *e = vmem_x86_get_vpage_entry(i, j);

	  if (e->present)
	    vmem_ppage_refdrop(e->address << 12);
	}

  /* switch to kernel context */
  vmem_x86_set_pagedir(vmem_k_context.pagedir_paddr);

  vmem_vpage_kfree(ctx->pagedir);
  vmem_vpage_kfree(ctx->mirror);
}

void vmem_context_switch_to(struct vmem_context_s *ctx)
{
  uint_fast16_t diff;

  if ((diff = vmem_k_context.k_count - ctx->k_count))
    {
      /* copy kernel part pagedir to context pagedir */
      memcpy(ctx->pagedir + ctx->k_count,
	     vmem_k_context.pagedir + ctx->k_count,
	     diff * sizeof(struct cpu_x86_pagetable_entry_s));

      memcpy(ctx->mirror + ctx->k_count,
	     vmem_k_context.mirror + ctx->k_count,
	     diff * sizeof(struct cpu_x86_page4k_entry_s));

      ctx->k_count += diff;
    }

  vmem_x86_set_pagedir(ctx->pagedir_paddr);

  CPU_LOCAL_SET(vmem_context_cur, ctx);
}

static error_t
vmem_x86_alloc_pagetable(uintptr_t vaddr)
{
  struct vmem_context_s *ctx;
  struct cpu_x86_pagetable_entry_s *pte;
  struct cpu_x86_page4k_entry_s *p4k;
  uintptr_t paddr;
  uint_fast16_t i = vaddr >> 22;

  printf("enter %s(%p)\n", __func__, vaddr);

  assert(i <= VMEM_USER_END_PDE);
  assert(i >= VMEM_INITIAL_PDE);

  /* allocate a new physical page for page table */
  if (vmem_ppage_alloc(&paddr))
    return -ENOMEM;

  ctx = vmem_context_get();
  pte = (void*)(ctx->pagedir + i);
  p4k = (void*)(ctx->mirror + i);

  assert(!pte->present);

  pte->present = 1;
  pte->writable = 1;
  pte->address = paddr >> 12;

  p4k->present = 1;
  p4k->writable = 1;
  p4k->address = paddr >> 12;

  /* clear page table */
  memset(vmem_x86_get_vpage_entry(i, 0), 0, VMEM_X86_PAGESIZE);

  if (vaddr < CONFIG_HEXO_VMEM_USERLIMIT)
    {
      if (ctx->k_count < i)
	ctx->k_count = i;

      if (ctx != &vmem_k_context)
	{
	  /* copy to real kernel context */
	  vmem_k_pagedir[i].pte = *pte;
	  vmem_k_mirror[i].p4k = *p4k;
	  vmem_k_context.k_count = ctx->k_count;
	}
    }

  return 0;
}

static struct cpu_x86_page4k_entry_s *
vmem_x86_get_vpage(uintptr_t vaddr)
{
  union cpu_x86_page_entry_s *pd;

  /* get pointer to appropiate pagedir. We must point to the real up
     to date kernel page directory here as we want to test the present
     bit ourself, we can't rely on update on exception mechanism. */
  if (vaddr < CONFIG_HEXO_VMEM_USERLIMIT)
    pd = vmem_k_pagedir;
  else
    pd = vmem_context_get()->pagedir;

  pd += (vaddr >> 22);

  if (!pd->pte.present)
    return NULL;

  /* return pointer to page entry mapped through mirror page table */
  return (void*)(uintptr_t)(VMEM_MIRROR_ADDR | ((vaddr & 0xfffffc00) >> 10));
}

static struct cpu_x86_page4k_entry_s *
vmem_x86_alloc_vpage(uintptr_t vaddr)
{
  union cpu_x86_page_entry_s *pd;

  printf("enter %s(%p)\n", __func__, vaddr);

  if (vaddr < CONFIG_HEXO_VMEM_USERLIMIT)
    pd = vmem_k_pagedir;
  else
    pd = vmem_context_get()->pagedir;

  pd += (vaddr >> 22);

  if (!pd->pte.present && vmem_x86_alloc_pagetable(vaddr))
    return NULL;

  return (void*)(uintptr_t)(VMEM_MIRROR_ADDR | ((vaddr & 0xfffffc00) >> 10));
}

vmem_pageattr_t vmem_vpage_get_attr(uintptr_t vaddr)
{
  vmem_pageattr_t attr = 0;
  struct cpu_x86_page4k_entry_s *e = vmem_x86_get_vpage(vaddr);

  if (e != NULL)
    {
      if (e->present)
	attr |= VMEM_PAGE_ATTR_PRESENT | VMEM_PAGE_ATTR_R | VMEM_PAGE_ATTR_X;

      if (e->writable)
	attr |= VMEM_PAGE_ATTR_W;

      if (e->userlevel)
	attr |= VMEM_PAGE_ATTR_USERLEVEL;

      if (e->dirty)
	attr |= VMEM_PAGE_ATTR_DIRTY;

      if (e->accessed)
	attr |= VMEM_PAGE_ATTR_ACCESSED;

      if (!e->nocache)
	attr |= VMEM_PAGE_ATTR_CACHED;
    }

  return attr;
}

uintptr_t vmem_vpage_get_paddr(uintptr_t vaddr)
{
  struct cpu_x86_page4k_entry_s *e = vmem_x86_get_vpage(vaddr);

  assert(e != NULL);

  return e->address << 12;
}

void * vmem_vpage_kalloc()
{
  uintptr_t paddr;
  uintptr_t vaddr;

  vaddr = vmem_k_context.v_next += VMEM_X86_PAGESIZE;

  struct cpu_x86_page4k_entry_s *e = vmem_x86_alloc_vpage(vaddr);

  printf("e %p\n", e);

  if (e == NULL)
    return NULL;

  /* allocate a new physical page for page table */
  if (vmem_ppage_alloc(&paddr))
    return NULL;

  e->present = 1;
  e->writable = 1;
  e->address = paddr >> 12;

  return (void*)vaddr;
}

void vmem_vpage_kfree(void *vaddr)
{
  uintptr_t paddr = vmem_vpage_get_paddr((uintptr_t)vaddr);

  vmem_ppage_refdrop(paddr);
}

uintptr_t vmem_vpage_io_map(uintptr_t paddr, size_t size)
{
  union cpu_x86_page_entry_s *pd;

  if (paddr + size <= CONFIG_HEXO_VMEM_INITIAL)
    return paddr;
}

