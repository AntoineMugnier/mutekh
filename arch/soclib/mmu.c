
#include <hexo/mmu.h>
#include <hexo/local.h>
#include <hexo/endian.h>

#include <string.h>
#include <assert.h>

#ifdef CONFIG_ARCH_SOCLIB
#include <soclib_addresses.h>
#endif

extern __ldscript_symbol_t  	__segment_text_start , __segment_text_end;
extern __ldscript_symbol_t  	__segment_data_uncached_start,__system_uncached_heap_start;

MMU_SOCLIB_ALIGN cpu_vcache_page_entry_t mmu_k_pagedir[1024];
MMU_SOCLIB_ALIGN cpu_vcache_page_entry_t mmu_k_mirror[1024];
MMU_SOCLIB_ALIGN struct mmu_context_s mmu_k_context;

static struct cpu_vcache_page4k_entry_s * mmu_vcache_alloc_vpage(uintptr_t vaddr);
static mmu_vpage_allocator_t *valloc;
static mmu_ppage_allocator_t *palloc;

CPU_LOCAL struct mmu_context_s *mmu_context_cur = &mmu_k_context;

inline bool_t mmu_is_user_vaddr(uintptr_t vaddr)
{
  return (vaddr >= CONFIG_HEXO_MMU_USER_START && vaddr < CONFIG_HEXO_MMU_USER_END);
}

void mmu_global_init(mmu_vpage_allocator_t *va, mmu_ppage_allocator_t *pa)
{
  uint_fast16_t	excep_pde,text_pde_s,uncached_pde_s,tty_pde,icu_pde,timer_pde;
  uint32_t t;

  valloc = va;
  palloc = pa;

  mmu_k_context.pagedir = mmu_k_pagedir;
  mmu_k_context.mirror = mmu_k_mirror;
  mmu_k_context.pagedir_paddr = (uintptr_t)mmu_k_pagedir;
  
  memset(mmu_k_pagedir, 0, sizeof (mmu_k_pagedir));
  memset(mmu_k_mirror, 0, sizeof (mmu_k_mirror));

  /* identity map memory space with 4Mb pages */
  
  t = (uint32_t)(&__segment_text_start);
  text_pde_s = t >> 22;
  t = (uint32_t)(&__segment_data_uncached_start);
  uncached_pde_s = t >> 22;
  
  mmu_k_pagedir[text_pde_s].p4m.entry_type = 2;
  mmu_k_pagedir[text_pde_s].p4m.non_cacheable = 0;
  mmu_k_pagedir[text_pde_s].p4m.writable = 1;
  mmu_k_pagedir[text_pde_s].p4m.executable = 1;
  mmu_k_pagedir[text_pde_s].p4m.global = 1;
  mmu_k_pagedir[text_pde_s].p4m.address = text_pde_s;

  if (text_pde_s != uncached_pde_s)
    {
      mmu_k_pagedir[uncached_pde_s].p4m.entry_type = 2;
#ifdef CONFIG_SMP
      mmu_k_pagedir[uncached_pde_s].p4m.non_cacheable = 1;
#else
      mmu_k_pagedir[uncached_pde_s].p4m.non_cacheable = 0;
#endif
      mmu_k_pagedir[uncached_pde_s].p4m.writable = 1;
      mmu_k_pagedir[uncached_pde_s].p4m.executable =0;
      mmu_k_pagedir[uncached_pde_s].p4m.global = 1;
      mmu_k_pagedir[uncached_pde_s].p4m.address = uncached_pde_s;  
    }

  mmu_k_context.k_count = uncached_pde_s + 1;

  mmu_k_pagedir[MMU_MIRROR_PDE].pte.entry_type = 1;
  mmu_k_pagedir[MMU_MIRROR_PDE].pte.address = ((uintptr_t)mmu_k_mirror) >> 12;

  mmu_k_mirror[MMU_MIRROR_PDE].p4k.entry_type = 2;
  mmu_k_mirror[MMU_MIRROR_PDE].p4k.writable = 1;
  mmu_k_mirror[MMU_MIRROR_PDE].p4k.global = 1;
  mmu_k_mirror[MMU_MIRROR_PDE].p4k.address = ((uintptr_t)mmu_k_mirror) >> 12 ;
  
}

void mmu_cpu_init()
{
#ifdef CONFIG_CPU_MIPS
  mmu_vcache_set_pagedir((uint32_t)mmu_k_context.pagedir_paddr);
  cpu_mips_mtc2(3, 1, 0);
#else
# error Add cpu mmu support here
#endif
}

struct mmu_context_s * mmu_get_kernel_context()
{
  return &mmu_k_context;
}

static void mmu_vcache_update_k_context(struct mmu_context_s *ctx)
{
  uint_fast16_t diff;

  if ((diff = mmu_k_context.k_count - ctx->k_count))
    {
      /* copy kernel part pagedir to context pagedir */
      memcpy(ctx->pagedir + ctx->k_count + MMU_KERNEL_START_PDE,
	     mmu_k_context.pagedir + ctx->k_count + MMU_KERNEL_START_PDE,
	     diff * sizeof(struct cpu_vcache_pagetable_entry_s));

      memcpy(ctx->mirror + ctx->k_count + MMU_KERNEL_START_PDE,
	     mmu_k_context.mirror + ctx->k_count + MMU_KERNEL_START_PDE,
	     diff * sizeof(struct cpu_vcache_page4k_entry_s));

//      printf("updating context %i %i %P\n", diff, ctx->k_count, ctx->pagedir, diff * 4);

      ctx->k_count += diff;
    }
}

error_t mmu_context_init(struct mmu_context_s *ctx)
{
  cpu_vcache_page_entry_t	*pagedir, *mirror;

  if ((pagedir = valloc(1)) == NULL)
    goto err;

  if ((mirror = valloc(1)) == NULL)
    goto err2;

  memset(pagedir, 0, CONFIG_HEXO_MMU_PAGESIZE);
  memset(mirror, 0, CONFIG_HEXO_MMU_PAGESIZE);

  uint32_t mirror_paddr = mmu_vpage_get_paddr((uintptr_t)mirror);
  uint32_t pagedir_paddr = mmu_vpage_get_paddr((uintptr_t)pagedir);

  ctx->pagedir = pagedir;
  ctx->mirror = mirror;
  ctx->pagedir_paddr = pagedir_paddr;
  ctx->k_count = 0;

  /* setup page directory mirror */
  pagedir[MMU_MIRROR_PDE].pte.entry_type = 1;
  pagedir[MMU_MIRROR_PDE].pte.address = mirror_paddr >> 12;

  mirror[MMU_MIRROR_PDE].p4k.entry_type = 2;
  mirror[MMU_MIRROR_PDE].p4k.writable = 1;
  mirror[MMU_MIRROR_PDE].p4k.address = mirror_paddr >> 12;

  mmu_vcache_update_k_context(ctx);

  return 0;

 err2:
  mmu_vpage_kfree(pagedir,1);
 err:
  return -ENOMEM;
}

static inline struct cpu_vcache_page4k_entry_s *
mmu_vcache_get_vpage_entry(uint_fast16_t pde, uint_fast16_t pte)//TODO:OK
{
  return (void*)(uintptr_t)(MMU_MIRROR_ADDR | (pde << 12) | (pte << 2));
}

void mmu_context_destroy(void)//TODO:ok
{
  struct mmu_context_s *ctx = mmu_context_get();
  uint_fast8_t i, j;

  /* refrdop all physical pages mapped in context */
  for (i = MMU_USER_START_PDE; i <= MMU_USER_END_PDE; i++)
    if (ctx->pagedir[i].pte.entry_type)
    {
     for (j = 0; j < 1024; j++)
	{
	  struct cpu_vcache_page4k_entry_s *e = mmu_vcache_get_vpage_entry(i, j);

	  if (e->entry_type)
	    mmu_ppage_refdrop(e->address << 12);
	}
	 mmu_ppage_refdrop(ctx->pagedir[i].pte.address<<12);
	 }

  /* switch to kernel context */
  mmu_vcache_set_pagedir(mmu_k_context.pagedir_paddr);

  mmu_vpage_kfree(ctx->pagedir,1);
  mmu_vpage_kfree(ctx->mirror,1);
}

void mmu_context_switch_to(struct mmu_context_s *ctx)//TODO:OK
{
  if(ctx!=mmu_context_get()){	
	  if(ctx!=&mmu_k_context)
	  mmu_vcache_update_k_context(ctx);

//	  printf("switch to ctx %p pd %p pdphys %p\n", ctx, ctx->pagedir, ctx->pagedir_paddr);

	  mmu_vcache_set_pagedir(ctx->pagedir_paddr);

	  CPU_LOCAL_SET(mmu_context_cur, ctx);
  }
}

static error_t
mmu_vcache_alloc_pagetable(uintptr_t vaddr)//FIXME: 
{
  struct mmu_context_s *ctx;
  struct cpu_vcache_pagetable_entry_s *pte;
  struct cpu_vcache_page4k_entry_s *p4k;
  uintptr_t paddr;
  uint_fast16_t i = vaddr >> 22;

//  printf("enter %s(%p)\n", __func__, vaddr);

//  assert(i <= MMU_USER_END_PDE);//FIXME: Peut être à revoir dans le cas de soclib
//  assert(i >= MMU_INITIAL_PDE);

  /* allocate a new physical page for page table */
  if (palloc(&paddr))
    return -ENOMEM;

//	printf("%s step 1 pass\n",__func__);

  ctx = mmu_context_get();
  pte = (void*)(ctx->pagedir + i);
  p4k = (void*)(ctx->mirror + i);

//	printf("%s step 2 pass\n",__func__);
  assert(!pte->entry_type);

  pte->entry_type =1;
  pte->address = paddr >> 12;
  
  p4k->entry_type=2;
  p4k->executable=0;
  p4k->user=((uint32_t)vaddr < (uint32_t)MMU_KERNEL_START_ADDR)?1:0;
  p4k->writable = 1;
  p4k->address = paddr >> 12;

//	printf("%s step 3 pass\n",__func__);
  /* clear page table */
//   printf("mmu_vcache_get_vpage_entry(i,0) = %p\n",mmu_vcache_get_vpage_entry(i, 0));
  memset(mmu_vcache_get_vpage_entry(i, 0), 0, CONFIG_HEXO_MMU_PAGESIZE);

//	printf("%s step 4 pass\n",__func__);
  if (vaddr >= MMU_KERNEL_START_ADDR)
    {
      if (ctx->k_count <= (i - MMU_KERNEL_START_PDE) )
	ctx->k_count = i - MMU_KERNEL_START_PDE + 1;

      if (ctx != &mmu_k_context)
	{
	  /* copy to real kernel context */
	  mmu_k_pagedir[i].pte = *pte;
	  mmu_k_mirror[i].p4k = *p4k;
	  mmu_k_context.k_count = ctx->k_count;
	}
    }
//	printf("%s step 5 pass\n",__func__);

  return 0;
}

static struct cpu_vcache_page4k_entry_s *
mmu_vcache_get_vpage(uintptr_t vaddr)//TODO:OK
{
  union cpu_vcache_page_entry_s *pd;

  /* get pointer to appropiate pagedir. We must point to the real up
     to date kernel page directory here as we want to test the present
     bit ourself, we can't rely on update on exception mechanism. */
  if (vaddr >= MMU_KERNEL_START_ADDR)
    pd = mmu_k_pagedir;
  else
    pd = mmu_context_get()->pagedir;

  pd += (vaddr >> 22);

  if (!pd->pte.entry_type)
    return NULL;

  /* return pointer to page entry mapped through mirror page table */
  return (void*)(uintptr_t)(MMU_MIRROR_ADDR | ((vaddr & 0xfffffc00) >> 10));
}

static struct cpu_vcache_page4k_entry_s *
mmu_vcache_alloc_vpage(uintptr_t vaddr)//TODO: ok
{
  union cpu_vcache_page_entry_s *pd;

//  printf("enter %s(%p)\n", __func__, vaddr);
		
  if (vaddr >= (uintptr_t)MMU_KERNEL_START_ADDR)
	{//printf("%s step 0 pass, if\n",__func__);			
    pd = mmu_k_pagedir;
 }
  else{
	//printf("%s step 0 pass, else\n",__func__);
    pd = mmu_context_get()->pagedir;
}
	//printf("%s step 1 pass\n",__func__);
  pd += (vaddr >> 22);
	//printf("%s step 2 pass\n",__func__);

  if (!pd->pte.entry_type && mmu_vcache_alloc_pagetable(vaddr))
    return NULL;
	//printf("%s step 3 pass\n",__func__);

  return (void*)(uintptr_t)(MMU_MIRROR_ADDR | ((vaddr & 0xfffffc00) >> 10));
}

mmu_pageattr_t mmu_vpage_get_attr(uintptr_t vaddr)//TODO: Global -> modifier la macro correspondante
{
  mmu_pageattr_t attr = 0;
  struct cpu_vcache_page4k_entry_s *e = mmu_vcache_get_vpage(vaddr);

  if (e != NULL)
    {
      if (e->entry_type)
	attr |= MMU_PAGE_ATTR_PRESENT | MMU_PAGE_ATTR_R | MMU_PAGE_ATTR_X;
		
		if (e->executable)
	attr |= MMU_PAGE_ATTR_X;
      
      if (e->writable)
	attr |= MMU_PAGE_ATTR_W;

      if (e->user)
	attr |= MMU_PAGE_ATTR_USERLEVEL;

      if (e->dirty)
	attr |= MMU_PAGE_ATTR_DIRTY;

      if (e->global)
	attr |= MMU_PAGE_ATTR_ACCESSED;

      if (e->non_cacheable)
	attr |= MMU_PAGE_ATTR_NOCACHE;
    }

  return attr;
}

error_t mmu_vpage_set(uintptr_t vaddr, uintptr_t paddr, mmu_pageattr_t attr)//TODO:OK
{
  struct cpu_vcache_page4k_entry_s *e = mmu_vcache_alloc_vpage(vaddr);//TODO: pourquoi juste les pages de 4k

  if (e == NULL)
    return -ENOMEM;

  assert((paddr & 0x3ff) == 0);
  e->address = paddr >> 12;

  e->entry_type = (attr & MMU_PAGE_ATTR_PRESENT) ? 2 : 0;
  e->executable = (attr & MMU_PAGE_ATTR_X) ? 1 : 0;
  e->writable = (attr & MMU_PAGE_ATTR_W) ? 1 : 0;
  e->user = (attr & MMU_PAGE_ATTR_USERLEVEL) ? 1 : 0;
  e->dirty = (attr & MMU_PAGE_ATTR_DIRTY) ? 1 : 0;
  e->global = (attr & MMU_PAGE_ATTR_ACCESSED) ? 1 : 0;
  e->non_cacheable = (attr & MMU_PAGE_ATTR_NOCACHE) ? 1 : 0;

  return 0;
}

/* set (logical or) and clear (logical nand) page attributes, may flush tlb */
void mmu_vpage_mask_attr(uintptr_t vaddr, mmu_pageattr_t setmask, mmu_pageattr_t clrmask)//TODO:OK
{
  struct cpu_vcache_page4k_entry_s *e = mmu_vcache_get_vpage(vaddr);

  assert(e != NULL);
  assert((setmask & clrmask) == 0);

  if (setmask & MMU_PAGE_ATTR_PRESENT)
    e->entry_type = 2;
  if (clrmask & MMU_PAGE_ATTR_PRESENT)
    e->entry_type = 0;
	
  if (setmask & MMU_PAGE_ATTR_X)
    e->executable = 1;
  if (clrmask & MMU_PAGE_ATTR_X)
    e->executable = 0;
  
  if (setmask & MMU_PAGE_ATTR_W)
    e->writable = 1;
  if (clrmask & MMU_PAGE_ATTR_W)
    e->writable = 0;

  if (setmask & MMU_PAGE_ATTR_USERLEVEL)
    e->user = 1;
  if (clrmask & MMU_PAGE_ATTR_USERLEVEL)
    e->user = 0;

  if (setmask & MMU_PAGE_ATTR_DIRTY)
    e->dirty = 1;
  if (clrmask & MMU_PAGE_ATTR_DIRTY)
    e->dirty = 0;

  if (setmask & MMU_PAGE_ATTR_ACCESSED)
    e->global = 1;
  if (clrmask & MMU_PAGE_ATTR_ACCESSED)
    e->global = 0;

  if (setmask & MMU_PAGE_ATTR_NOCACHE)
    e->non_cacheable = 1;
  if (clrmask & MMU_PAGE_ATTR_NOCACHE)
    e->non_cacheable = 0;
}

uintptr_t mmu_vpage_get_paddr(uintptr_t vaddr)//TODO:OK
{
  struct cpu_vcache_page4k_entry_s *e = mmu_vcache_get_vpage(vaddr);

  assert(e != NULL);

  return e->address << 12;
}

