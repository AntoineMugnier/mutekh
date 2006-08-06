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

/** x86 apic register structure */

typedef union
{
  volatile uint32_t	_32[4];
  volatile uint64_t	_64[2];
} cpu_x86_apic___reg_t;

/** x86 apic mapped memory registers */

struct cpu_x86_apic_s
{
  /* 0000 */
  cpu_x86_apic___reg_t	res0000;
  cpu_x86_apic___reg_t	res0010;
  cpu_x86_apic___reg_t	lapic_id;
  cpu_x86_apic___reg_t	lapic_version;
  cpu_x86_apic___reg_t	res0040;
  cpu_x86_apic___reg_t	res0050;
  cpu_x86_apic___reg_t	res0060;
  cpu_x86_apic___reg_t	res0070;
  /* 0080 */
  cpu_x86_apic___reg_t	task_prio;
  cpu_x86_apic___reg_t	arbitr_prio;
  cpu_x86_apic___reg_t	processor_prio;
  cpu_x86_apic___reg_t	eoi;
  cpu_x86_apic___reg_t	res00c0;
  cpu_x86_apic___reg_t	logical_dest;
  cpu_x86_apic___reg_t	dest_format;
  cpu_x86_apic___reg_t	spurious_int;
  /* 0100 */
  cpu_x86_apic___reg_t	isr[8];		/* in service register */
  cpu_x86_apic___reg_t	tmr[8];		/* trigger mode register */
  /* 0200 */
  cpu_x86_apic___reg_t	irr[8];		/* interrupt request register */
  /* 0280 */
  cpu_x86_apic___reg_t	error;		/* error status register */
  cpu_x86_apic___reg_t	res0290;
  cpu_x86_apic___reg_t	res02a0;
  cpu_x86_apic___reg_t	res02b0;
  cpu_x86_apic___reg_t	res02c0;
  cpu_x86_apic___reg_t	res02d0;
  cpu_x86_apic___reg_t	res02e0;
  cpu_x86_apic___reg_t	res02f0;
  /* 0300 */
  cpu_x86_apic___reg_t	icr_0_31;	/* interrupt command register */
  cpu_x86_apic___reg_t	icr_32_63;	/* interrupt command register */
  cpu_x86_apic___reg_t	lvt_timer;	/* LVT Timer Register */
  cpu_x86_apic___reg_t	lvt_thermal;	/* LVT Thermal Sensor Register */
  cpu_x86_apic___reg_t	lvt_perf;	/* LVT Performance Monitoring Counters */
  cpu_x86_apic___reg_t	lvt_lint0;	/* LVT LINT0 Register */
  cpu_x86_apic___reg_t	lvt_lint1;	/* LVT LINT1 Register */
  cpu_x86_apic___reg_t	lvt_error;	/* LVT Error Register */
  /* 0380 */
  cpu_x86_apic___reg_t	timer_init;	/* Initial Count Register */
  cpu_x86_apic___reg_t	timer_curr;	/* Current Count Register */
  cpu_x86_apic___reg_t	res03a0;
  cpu_x86_apic___reg_t	res03b0;
  cpu_x86_apic___reg_t	res03c0;
  cpu_x86_apic___reg_t	res03d0;
  cpu_x86_apic___reg_t	timer_divide;	/* Divide Configuration Register */
  cpu_x86_apic___reg_t	res03f0;
} __attribute__ ((packed));

/**
   x86 apic mapped memory registers address
   @return current apic register address
*/

static inline struct cpu_x86_apic_s *
cpu_apic_get_regaddr(void)
{
  uint64_t	msr;

  asm ("rdmsr\n"
       : "=A" (msr)
       : "c" (0x1b)
       );

  return (struct cpu_x86_apic_s*)(uintptr_t)(msr & 0xfffff000);
}

/**
   get x86 apic enabled state
   @return apic enabled state
*/

static inline bool_t
cpu_apic_isenabled(void)
{
  uint64_t	msr;

  asm ("rdmsr\n"
       : "=A" (msr)
       : "c" (0x1b)
       );

  return msr & 0x800 ? 1 : 0;
}

/**
   x86 apic boot strap processor (BSP)
   @return true if processor is the bootstrap processor
*/

static inline bool_t
cpu_apic_isbootstrap(void)
{
  uint64_t	msr;

  asm ("rdmsr\n"
       : "=A" (msr)
       : "c" (0x1b)
       );

  return msr & 0x100 ? 1 : 0;
}

#endif

