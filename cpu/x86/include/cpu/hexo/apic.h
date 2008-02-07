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

#ifndef CPU_APIC_H_
#define CPU_APIC_H_

#include "msr.h"

/** x86 apic register structure */

typedef union
{
  volatile uint32_t	_32[4];
  volatile uint64_t	_64[2];
} cpu_x86_apic_reg_t;

/** x86 apic mapped memory registers */

typedef volatile struct
{
  /* 0000 */
  cpu_x86_apic_reg_t	res0000;
  cpu_x86_apic_reg_t	res0010;
  cpu_x86_apic_reg_t	lapic_id;
  cpu_x86_apic_reg_t	lapic_version;
  cpu_x86_apic_reg_t	res0040;
  cpu_x86_apic_reg_t	res0050;
  cpu_x86_apic_reg_t	res0060;
  cpu_x86_apic_reg_t	res0070;
  /* 0080 */
  cpu_x86_apic_reg_t	context_prio;
  cpu_x86_apic_reg_t	arbitr_prio;
  cpu_x86_apic_reg_t	processor_prio;
  cpu_x86_apic_reg_t	eoi;
  cpu_x86_apic_reg_t	res00c0;
  cpu_x86_apic_reg_t	logical_dest;
  cpu_x86_apic_reg_t	dest_format;
  cpu_x86_apic_reg_t	spurious_int;
  /* 0100 */
  cpu_x86_apic_reg_t	isr[8];		/* in service register */
  cpu_x86_apic_reg_t	tmr[8];		/* trigger mode register */
  /* 0200 */
  cpu_x86_apic_reg_t	irr[8];		/* interrupt request register */
  /* 0280 */
  cpu_x86_apic_reg_t	error;		/* error status register */
  cpu_x86_apic_reg_t	res0290;
  cpu_x86_apic_reg_t	res02a0;
  cpu_x86_apic_reg_t	res02b0;
  cpu_x86_apic_reg_t	res02c0;
  cpu_x86_apic_reg_t	res02d0;
  cpu_x86_apic_reg_t	res02e0;
  cpu_x86_apic_reg_t	res02f0;
  /* 0300 */
  cpu_x86_apic_reg_t	icr_0_31;	/* interrupt command register */
  cpu_x86_apic_reg_t	icr_32_63;	/* interrupt command register */
  cpu_x86_apic_reg_t	lvt_timer;	/* LVT Timer Register */
  cpu_x86_apic_reg_t	lvt_thermal;	/* LVT Thermal Sensor Register */
  cpu_x86_apic_reg_t	lvt_perf;	/* LVT Performance Monitoring Counters */
  cpu_x86_apic_reg_t	lvt_lint0;	/* LVT LINT0 Register */
  cpu_x86_apic_reg_t	lvt_lint1;	/* LVT LINT1 Register */
  cpu_x86_apic_reg_t	lvt_error;	/* LVT Error Register */
  /* 0380 */
  cpu_x86_apic_reg_t	timer_init;	/* Initial Count Register */
  cpu_x86_apic_reg_t	timer_curr;	/* Current Count Register */
  cpu_x86_apic_reg_t	res03a0;
  cpu_x86_apic_reg_t	res03b0;
  cpu_x86_apic_reg_t	res03c0;
  cpu_x86_apic_reg_t	res03d0;
  cpu_x86_apic_reg_t	timer_divide;	/* Divide Configuration Register */
  cpu_x86_apic_reg_t	res03f0;
} __attribute__ ((packed)) cpu_x86_apic_t;

/**
   x86 apic mapped memory registers address
   @return current apic register address
*/

static inline cpu_x86_apic_t *
cpu_apic_get_regaddr(void)
{
  return (void*)(uint32_t)(cpu_x86_read_msr(IA32_APIC_BASE_MSR) & 0xfffff000);
}

static inline void
cpu_apic_set_regaddr(cpu_x86_apic_t *addr)
{
  uint64_t x = cpu_x86_read_msr(IA32_APIC_BASE_MSR);

  cpu_x86_write_msr(IA32_APIC_BASE_MSR, (x & 0xfff) | (uint32_t)addr);
}

/**
   get x86 apic enabled state
   @return apic enabled state
*/

static inline bool_t
cpu_apic_isenabled(void)
{
  return cpu_x86_read_msr(IA32_APIC_BASE_MSR) & 0x800 ? 1 : 0;
}

/** local apic address */
#define ARCH_SMP_LOCAL_APICADDR	0xfee00000

/** CPU specific boot address for x86 SMP bootup sequence */
#define ARCH_SMP_BOOT_ADDR	0x00002000


#endif

