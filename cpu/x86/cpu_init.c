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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#include <hexo/alloc.h>
#include <string.h>
#include <hexo/interrupt.h>
#include <hexo/init.h>
#include <hexo/iospace.h>
#include <hexo/lock.h>
#include <hexo/segment.h>

#include <cpu/hexo/pmode.h>
#include <cpu/hexo/apic.h>
#include <arch/hexo/specific.h>

CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_hw_handler;
CPU_LOCAL cpu_exception_handler_t  *cpu_interrupt_ex_handler;
CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_sys_handler;

/** pointer to cpu local storage itself */
CPU_LOCAL void *__cpu_data_base;
/** pointer to context local storage itself */
CONTEXT_LOCAL void *__context_data_base;

/* CPU Local Descriptor structure */

/** gdt table lock */
static lock_t				gdt_lock;

/** CPU Global descriptor table */
static union cpu_x86_desc_s		*gdt;

#ifdef CONFIG_ARCH_IBMPC_MEMORY_SEGLIMIT
#define ARCH_IBMPC_SEGLIMIT	CONFIG_ARCH_IBMPC_MEMORY
#else
#define ARCH_IBMPC_SEGLIMIT	0xffffffff
#endif

error_t
cpu_global_init(void)
{
  uint_fast16_t	i;

  if (!(gdt = mem_alloc(ARCH_GDT_SIZE * sizeof (union cpu_x86_desc_s), MEM_SCOPE_SYS)))
    return -1;

  lock_init(&gdt_lock);

  cpu_x86_seg_setup(&gdt[ARCH_GDT_CODE_INDEX].seg, 0,
		    ARCH_IBMPC_SEGLIMIT, CPU_X86_SEG_EXEC_NC_R, 0, 1);

  cpu_x86_seg_setup(&gdt[ARCH_GDT_DATA_INDEX].seg, 0,
		    ARCH_IBMPC_SEGLIMIT, CPU_X86_SEG_DATA_UP_RW, 0, 1);

  /* mark all other GDT entries available */
  for (i = ARCH_GDT_FIRST_ALLOC; i < ARCH_GDT_SIZE; i++)
    gdt[i].seg.available = 1;

#ifdef CONFIG_SMP
  /* copy boot section below 1Mb for slave CPUs bootup */
  memcpy((void*)ARCH_SMP_BOOT_ADDR, (char*)&__boot_start, (char*)&__boot_end - (char*)&__boot_start);
#endif

  return 0;
}

static cpu_x86_segsel_t
cpu_x86_gdt_desc_alloc(void)
{
  cpu_x86_segsel_t	sel = 0;
  uint_fast16_t		i;

  for (i = ARCH_GDT_FIRST_ALLOC; i < ARCH_GDT_SIZE; i++)
    {
      if (gdt[i].seg.available)
	{
	  sel = i;
	  break;
	}
    }

  return sel;
}

cpu_x86_segsel_t
cpu_x86_segment_alloc(uintptr_t addr, uint32_t size, uint_fast8_t type)
{
  cpu_x86_segsel_t	sel;

  lock_spin(&gdt_lock);

  if ((sel = cpu_x86_gdt_desc_alloc()))
    {
      union cpu_x86_desc_s	*desc = gdt + sel;

      cpu_x86_seg_setup(&desc->seg, addr, size, type, 0, 1);
    }

  lock_release(&gdt_lock);

  return sel;
}

void
cpu_x86_segdesc_free(cpu_x86_segsel_t sel)
{
  lock_spin(&gdt_lock);

  gdt[sel].seg.available = 1;

  lock_release(&gdt_lock);  
}

struct cpu_cld_s
{
#ifdef CONFIG_SMP
  /* pointer to CPU local storage */
  void				*cpu_local_storage;
#endif
  /* CPU id */
  uint32_t			id;
  /* CPU Interrupt descriptor table */
  struct cpu_x86_gatedesc_s	idt[CPU_MAX_INTERRUPTS];
};

static CPU_LOCAL struct cpu_cld_s	*cpu_cld;

static void cpu_x86_init_apic(uint32_t cpu_id)
{
  struct cpu_x86_apic_s	*apic = cpu_apic_get_regaddr();
  uint32_t			i;

  /* update local APIC id */
  cpu_mem_write_32((uintptr_t)&apic->lapic_id, cpu_id << 24);

  /* enable CPU local APIC */
  i = cpu_mem_read_32((uintptr_t)&apic->spurious_int);
  i |= 0x100;
  cpu_mem_write_32((uintptr_t)&apic->spurious_int, i);
}

struct cpu_cld_s *cpu_init(uint_fast8_t cpu_id)
{
#ifdef CONFIG_SMP
  void			*cls;
  uint16_t		cls_sel;
#endif
  struct cpu_cld_s	*cld;
  uint_fast16_t		i;

  /* set GDT pointer */
  cpu_x86_set_gdt(gdt, ARCH_GDT_SIZE);

  /* enable and initialize x86 APIC */
  cpu_x86_init_apic(cpu_id);

  if (!(cld = mem_alloc(sizeof (struct cpu_cld_s), MEM_SCOPE_SYS)))
    goto err_cld;

  cld->id = cpu_id;

  /* setup cpu local storage */

#ifdef CONFIG_SMP
  if (!(cls = arch_cpudata_alloc()))
    goto err_cls;

  cld->cpu_local_storage = cls;

  if (!(cls_sel = cpu_x86_segment_alloc((uintptr_t)cls,
					arch_cpudata_size(),
					CPU_X86_SEG_DATA_UP_RW)))
    goto err_cls_seg;

  cpu_x86_datasegfs_use(cls_sel, 0);

  CPU_LOCAL_SET(__cpu_data_base, cls);
#endif

  /* activate defined segments */

  cpu_x86_dataseg_use(ARCH_GDT_DATA_INDEX, 0);
  cpu_x86_stackseg_use(ARCH_GDT_DATA_INDEX, 0);
  cpu_x86_codeseg_use(ARCH_GDT_CODE_INDEX, 0);

  CPU_LOCAL_SET(cpu_cld, cld);

  /* fill IDT with exceptions entry points */

  for (i = 0; i < CPU_EXCEPT_VECTOR_COUNT; i++)
    {
      uintptr_t	entry = ((uintptr_t)&x86_interrupt_ex_entry) + i * CPU_INTERRUPT_ENTRY_ALIGN;

      cpu_x86_gate_setup(cld->idt + i + CPU_EXCEPT_VECTOR,
			 ARCH_GDT_CODE_INDEX, entry,
			 CPU_X86_GATE_INT32, 0, 0);
    }

  /* fill IDT with hardware interrupts entry points */

  for (i = 0; i < CPU_HWINT_VECTOR_COUNT; i++)
    {
      uintptr_t	entry = ((uintptr_t)&x86_interrupt_hw_entry) + i * CPU_INTERRUPT_ENTRY_ALIGN;

      cpu_x86_gate_setup(cld->idt + i + CPU_HWINT_VECTOR,
			 ARCH_GDT_CODE_INDEX, entry,
			 CPU_X86_GATE_INT32, 0, 0);
    }

  /* fill IDT with syscall entry points */

  for (i = 0; i < CPU_SYSCALL_VECTOR_COUNT; i++)
    {
      uintptr_t	entry = ((uintptr_t)&x86_interrupt_sys_entry) + i * CPU_INTERRUPT_ENTRY_ALIGN;

      cpu_x86_gate_setup(cld->idt + i + CPU_SYSCALL_VECTOR,
			 ARCH_GDT_CODE_INDEX, entry,
			 CPU_X86_GATE_INT32, 0, 0);
    }

  cpu_x86_set_idt(cld->idt, CPU_MAX_INTERRUPTS);

  return cld;

 err_cls_seg:
#ifdef CONFIG_SMP
  mem_free(cls);
#endif
 err_cls:
  mem_free(cld);
 err_cld:
  return NULL;
}

void cpu_start_other_cpu(void)
{
#ifdef CONFIG_SMP
  struct cpu_x86_apic_s	*apic = cpu_apic_get_regaddr();
  uint32_t		i;

  /* broadcast an INIT IPI to other CPU */
  cpu_mem_write_32((uintptr_t)&apic->icr_0_31, 0x000c4500);

  /* 10 ms delay */
  for (i = 0; i < 4000000; i++)
    asm volatile ("nop\n");

  /* broadcast an SIPI IPI to other CPU */
  cpu_mem_write_32((uintptr_t)&apic->icr_0_31, 0x000c4611);

  /* 200 us delay */
  for (i = 0; i < 80000; i++)
    asm volatile ("nop\n");
 
  /* broadcast an SIPI IPI to other CPU */
  cpu_mem_write_32((uintptr_t)&apic->icr_0_31, 0x000c4611);
#endif
}

uint_fast8_t cpu_id(void)
{
#ifdef CONFIG_SMP
  struct cpu_cld_s	*cld = CPU_LOCAL_GET(cpu_cld);

  return cld->id;
#else
  return 0;
#endif
}

