/*
 *   This file is part of MutekH.
 *   
 *   MutekH is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation; version 2.1 of the
 *   License.
 *   
 *   MutekH is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *   
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with MutekH; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 *   02110-1301 USA.
 *
 *   Copyright Francois Charot <charot@irisa.fr>  (c) 2008
 *   INRIA Rennes Bretagne Atlantique
 *
 */

#include <mutek/mem_alloc.h>
#include <hexo/asm.h>
#include <hexo/init.h>
#include <hexo/segment.h>
#include <hexo/cpu.h>
#include <hexo/local.h>
#include <hexo/interrupt.h>

/** pointer to context local storage in cpu local storage */
CPU_LOCAL void *__context_data_base;

#ifdef CONFIG_ARCH_SMP
void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

/* CPU Local Descriptor structure */

error_t
cpu_global_init(void)
{
  return 0;
}

extern __ldscript_symbol_t   __segment_excep_start;
extern __ldscript_symbol_t   __exception_base_ptr;

void cpu_init(void)
{
  /* Set exception vector */
  cpu_nios2_write_ctrl_reg(17, (reg_t)&__exception_base_ptr);

#ifdef CONFIG_ARCH_SMP
  void			*cls;

  /* setup cpu local storage */
  cls = cpu_local_storage[cpu_id()];

  /* set cpu local storage register base pointer */
  //  __asm__ volatile("wrctl ctl16, %0" : : "r" (cls));
  __asm__ volatile("mov " ASM_STR(CPU_NIOS2_CLS_REG) ", %0" : : "r" (cls));
#endif

}


