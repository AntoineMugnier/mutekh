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


#include <mem_alloc.h>
#include <hexo/init.h>
#include <hexo/segment.h>
#include <hexo/cpu.h>
#include <hexo/local.h>
#include <hexo/interrupt.h>

#include <drivers/device/icu/arm/icu-arm.h>
#include <device/device.h>
#include <device/driver.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

#ifdef CONFIG_DRIVER_ICU_ARM
CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_handler;
CPU_LOCAL struct device_s cpu_icu_dev;
#else
static uint8_t irq_stack[128];
#endif

CPU_LOCAL cpu_exception_handler_t  *cpu_exception_handler;

struct arm_exception_context_s {
	uint32_t user_pc;
	uint32_t user_cpsr;
	uint32_t user_r0;
};

#ifdef CONFIG_SMP
void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

struct arm_exception_context_s arm_exception_context[CONFIG_CPU_MAXCOUNT][3];

#define arm_setup_exception_stack(context, psr_mode)				   \
	asm volatile(													   \
		"mrs  r2, cpsr            \n\t"								   \
		"bic  r3, r2, #0x1f       \n\t"								   \
		"orr  r3, r3, %0          \n\t"								   \
		"msr  cpsr, r3            \n\t"								   \
		"mov  sp, %1              \n\t"								   \
		"msr  cpsr, r2            \n\t"								   \
		:															   \
		: "i"(psr_mode), "r"(context)								   \
		: "r2", "r3" );

/* CPU Local Descriptor structure */

error_t
cpu_global_init(void)
{

  return 0;
}

static void __arm_exception_setup()
{
#ifdef CONFIG_SOCLIB_MEMCHECK
	soclib_mem_check_disable(SOCLIB_MC_CHECK_SPFP);
#endif

	struct arm_exception_context_s *cpu_context = arm_exception_context[cpu_id()];
#ifndef CONFIG_DRIVER_ICU_ARM
	arm_setup_exception_stack(irq_stack+sizeof(irq_stack)-4, 0x12);
#else
	arm_setup_exception_stack(&cpu_context[0], 0x12); // IRQ
#endif
	arm_setup_exception_stack(&cpu_context[1], 0x17); // Abort
	arm_setup_exception_stack(&cpu_context[2], 0x1b); // Undef

#ifdef CONFIG_SOCLIB_MEMCHECK
	soclib_mem_check_enable(SOCLIB_MC_CHECK_SPFP);
#endif
}

void cpu_init(void)
{
#ifdef CONFIG_SMP
# if !defined(CONFIG_CPU_ARM_TLS_IN_C15)
#  error SMP and TLS unsupported
# endif

	void			*cls;

	/* setup cpu local storage */
	cls = arch_cpudata_alloc();
		
	cpu_local_storage[cpu_id()] = cls;
		
	asm volatile ("mcr p15,0,%0,c13,c0,3":: "r" (cls));
#endif

	__arm_exception_setup();

#ifdef CONFIG_DRIVER_ICU_ARM
  device_init(CPU_LOCAL_ADDR(cpu_icu_dev));
  icu_arm_init(CPU_LOCAL_ADDR(cpu_icu_dev), NULL);
#endif
}

void cpu_start_other_cpu(void)
{
#ifdef CONFIG_SMP

#endif
}

