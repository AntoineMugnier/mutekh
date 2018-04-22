/*
   This file is part of MutekH.
   
   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; version 2.1 of the License.
   
   MutekH is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.
   
   You should have received a copy of the GNU Lesser General Public
   License along with MutekH; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.

   Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <mutek/startup.h>
#include <arch/soclib/mem_checker.h>
#include <hexo/cpu.h>

#include <string.h>
#include <assert.h>

#ifdef CONFIG_SOCLIB_MEMCHECK

static void soclib_memcheck_cpu_init(uintptr_t id, uintptr_t stack, size_t size)
{
  /* create a new memchecker context */
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_R1, stack);
  cpu_mem_write_32(SOCLIB_MC_R2, size);
  cpu_mem_write_32(SOCLIB_MC_CTX_CREATE, id);

  /* switch to new memchecker context */
  cpu_mem_write_32(SOCLIB_MC_CTX_SET, id);

  /* enable all memchecker checks */
  cpu_mem_write_32(SOCLIB_MC_ENABLE, SOCLIB_MC_CHECK_ALL);

  /* leave memchecker command mode */
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
}

void soclib_memcheck_init()
{
  extern __ldscript_symbol_t __data_start;
  extern __ldscript_symbol_t __data_load_start;
  extern __ldscript_symbol_t __data_load_end;

  soclib_mem_check_region_status(
      (uint8_t*)&__data_start,
      (uint8_t*)&__data_load_end-(uint8_t*)&__data_load_start,
      SOCLIB_MC_REGION_GLOBAL);

  extern __ldscript_symbol_t __bss_start;
  extern __ldscript_symbol_t __bss_end;

  soclib_mem_check_region_status(
      (uint8_t*)&__bss_start,
      (uint8_t*)&__bss_end-(uint8_t*)&__bss_start,
      SOCLIB_MC_REGION_GLOBAL);

  soclib_memcheck_cpu_init(0, CONFIG_STARTUP_STACK_ADDR,
                           CONFIG_STARTUP_STACK_SIZE);
}

# ifdef CONFIG_ARCH_SMP
void soclib_memcheck_initsmp()
{
  if (!cpu_isbootstrap())
    {
      const struct cpu_tree_s *cpu = cpu_tree_lookup(cpu_id());
      assert(cpu != NULL && "processor id not found in the cpu tree.");
      soclib_memcheck_cpu_init(cpu->stack, cpu->stack,
                               CONFIG_HEXO_CPU_STACK_SIZE);
    }
}
# endif

#endif

/////////////////////////////////////////////////////////////////////

#ifdef CONFIG_MUTEK_MEMALLOC

#include <mutek/mem_alloc.h>
#include <mutek/mem_region.h>
#include <mutek/mem_alloc.h>

void soclib_mem_init()
{
  default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                         (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                 CONFIG_STARTUP_HEAP_SIZE));
}

#endif

/////////////////////////////////////////////////////////////////////


#ifdef CONFIG_SOCLIB_FDT

# include <device/driver.h>
# include <device/resources.h>
# include <device/device.h>

DEV_DECLARE_STATIC(fdt_dev, "fdt", 0, enum_fdt_drv,
                   DEV_STATIC_RES_MEM( CONFIG_SOCLIB_FDT_ROM_ADDRESS,
                                       CONFIG_SOCLIB_FDT_ROM_ADDRESS + 4096 )
                   );

#endif


/////////////////////////////////////////////////////////////////////


#ifdef CONFIG_SOCLIB_MEM_REGION
void soclib_mem_region_init(void)
{
  cpu_id_t cpu;
  uint_fast16_t i;
  
  for (cpu=0; cpu<device_get_cpu_count(); cpu++)
    {
      mem_region_id_init(cpu);
      
      for (i=0; i<mem_scope_e_count; i++)
	{
	  if (i == mem_scope_sys)
	    mem_region_id_add(cpu, i, default_region, 0);
	  else
	    mem_region_id_add(cpu, i, default_region, 200);
	}
    }

  default_region = NULL;
}
#endif

/////////////////////////////////////////////////////////////////////

#ifdef CONFIG_VMEM
void soclib_vmem_init()
{
#ifdef CONFIG_HEXO_MMU
    uint32_t t0=(uint32_t)(&__system_uncached_heap_start);
    uint32_t t1=(uint32_t)(&__system_uncached_heap_end);
    t0+=CONFIG_SOCLIB_VMEM_MALLOC_REGION_SIZE;

# ifdef CONFIG_VMEM_PHYS_ALLOC
    vmem_ppage_ops_init(&vmem_ops);
    initial_ppage_region = vmem_ops.ppage_initial_region_get();
    vmem_ops.ppage_region_init(initial_ppage_region, t0, t1);
# endif

# ifdef CONFIG_VMEM_KERNEL_ALLOC
    vmem_vpage_ops_init(&vmem_ops);
    vmem_ops.vpage_init();
# endif

#endif /*CONFIG_HEXO_MMU*/

#ifdef CONFIG_HEXO_MMU
    mmu_cpu_init();
#endif
}
#endif


/////////////////////////////////////////////////////////////////////

#ifdef CONFIG_ARCH_SMP

#include <hexo/iospace.h>

void soclib_start_cpus()
{
  // trigger WTI 0
  // cpu_mem_write_32(0xd2200000, 42);
}

#endif

