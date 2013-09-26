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

*/


#include <mutek/mem_alloc.h>
#include <string.h>
#include <hexo/interrupt.h>
#include <mutek/startup.h>
#include <hexo/iospace.h>
#include <hexo/lock.h>
#include <hexo/cpu.h>

#include "cpu_private.h"

#include <cpu/hexo/pmode.h>
#include <cpu/hexo/msr.h>

/* CPU interrupt descriptor table */
struct cpu_x86_gatedesc_s cpu_idt[CPU_MAX_INTERRUPTS];

/** CPU Global descriptor table */
union cpu_x86_desc_s gdt[ARCH_GDT_SIZE];
static lock_t        gdt_lock;

void x86_pmode_tables_init(void)
{
  uint_fast16_t	i;

  lock_init(&gdt_lock);

  cpu_x86_seg_setup(&gdt[ARCH_GDT_CODE_INDEX].seg, 0,
		    0xffffffff, CPU_X86_SEG_EXEC_NC_R, 0, 1);

#ifdef CONFIG_HEXO_USERMODE
  cpu_x86_seg_setup(&gdt[ARCH_GDT_USER_CODE_INDEX].seg, 0,
		    0xffffffff, CPU_X86_SEG_EXEC_NC_R, 3, 1);
#endif

  cpu_x86_seg_setup(&gdt[ARCH_GDT_DATA_INDEX].seg, 0,
		    0xffffffff, CPU_X86_SEG_DATA_UP_RW, 0, 1);

#ifdef CONFIG_HEXO_USERMODE
  cpu_x86_seg_setup(&gdt[ARCH_GDT_USER_DATA_INDEX].seg, 0,
		    0xffffffff, CPU_X86_SEG_DATA_UP_RW, 3, 1);
#endif

  /* mark all other GDT entries available */
  for (i = ARCH_GDT_FIRST_ALLOC; i < ARCH_GDT_SIZE; i++)
    gdt[i].seg.available = 1;

  void CPU_NAME_DECL(exception_vector)();

  /* fill IDT with exceptions entry points */
  for (i = 0; i < CPU_EXCEPT_VECTOR_COUNT
#ifdef CONFIG_HEXO_IRQ
         + CPU_HWINT_VECTOR_COUNT
#endif
#ifdef CONFIG_HEXO_USERMODE
         + CPU_SYSCALL_VECTOR_COUNT
#endif
         ; i++)
    {
      uintptr_t	entry = ((uintptr_t)&CPU_NAME_DECL(exception_vector)) + i * CPU_INTERRUPT_ENTRY_ALIGN;

      cpu_x86_gate_setup(cpu_idt + i + CPU_EXCEPT_VECTOR,
			 ARCH_GDT_CODE_INDEX, entry,
			 CPU_X86_GATE_INT32, 0, 0);
    }
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

  assert(!gdt[sel].seg.available);
  gdt[sel].seg.available = 1;

  lock_release(&gdt_lock);  
}

