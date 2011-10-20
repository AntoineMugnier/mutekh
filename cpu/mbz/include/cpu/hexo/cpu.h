/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Luc Delecroix <luc D delecroix A thalesgroup D com> (c) 2011
    Copyright Laurent Gantel <laurent D gantel A ensea D fr> (c) 2011
*/

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else

#ifndef __MUTEK_ASM__

#include <hexo/endian.h>

#define CPU_CPU_H_

#ifdef CONFIG_ARCH_SMP
extern void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];
#endif

/** general purpose regsiters count */
# define CPU_GPREG_COUNT	32

# define CPU_GPREG_NAMES_MB {                                            \
		  "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",	 \
		  "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",	 \
		  "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",\
		  "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",\
}

# define CPU_SPREG_COUNT	8192+32 /* a voir */

# define CPU_SPREG_NAMES_MB {	\
		  "pc", 	\
		  "msr",	\
		  "ear",	\
		  "esr",	\
		  "btr", 	\
		  "fsr", 	\
		  "edr",	\
		  "pid", 	\
		  "zpr", 	\
		  "tlblo",	\
		  "tlbhi", 	\
		  "tlbx", 	\
		  "tlbsx",	\
		  "pvr", 	\
}

#define CPU_TYPE_NAME mbz

static inline cpu_id_t
cpu_id(void)
{

# ifdef CONFIG_ARCH_SMP
#  error Cant compile SMP code without cpuid support
# endif
	return 0; /* A REVOIR : */

}

static inline bool_t
cpu_isbootstrap(void)
{
  return cpu_id() == 0;
}

static inline
reg_t cpu_get_stackptr()
{
    reg_t ret;
    asm("mov %0, r1": "=r"(ret));
    return ret;
}

static inline cpu_cycle_t
cpu_cycle_count(void)
{
/*
uint32_t      result;
 comptage nombre de cycle a voir pour MicroBlaze
	asm volatile (
	  "mftbl %0"
	  : "=r" (result)
	  ); */

# warning No CPU cycle counter
	return 0;

}

static inline void
cpu_trap()
{
	/* branchement à @0x18 */
	asm volatile("bri 0x18");

}

static inline void *cpu_get_cls(cpu_id_t cpu_id)
{
#ifdef CONFIG_ARCH_SMP
  return cpu_local_storage[cpu_id];
#endif
  return NULL;
}

static inline void cpu_dcache_invld(void *ptr)
{

#ifndef XPAR_MICROBLAZE_DCACHE_LINE_LEN
#define XPAR_MICROBLAZE_DCACHE_LINE_LEN   1
#endif

	asm (
			"andi    r5, r5, -(4 * XPAR_MICROBLAZE_DCACHE_LINE_LEN)"  /* Align to cache line */

			"addik	r6, r5, XPAR_MICROBLAZE_DCACHE_BYTE_SIZE"        /* Compute end */
			"andi    r6, r6, -(4 * XPAR_MICROBLAZE_DCACHE_LINE_LEN)"  /* Align to cache line */

			"L_start:"
			"wdc	r5, r0"                                          /* Invalidate the Cache */

			"cmpu	r18, r5, r6"                                     /* Are we at the end? */
			"blei	r18, L_done"

			"brid	L_start"                                         /* Branch to the beginning of the loop */
			"addik	r5, r5, (XPAR_MICROBLAZE_DCACHE_LINE_LEN * 4)"   /* Increment the addrees by 4 (delay slot) */

			"L_done:"
			"rtsd	r15, 8"                                          /* Return */
			"nop"
	);


}

static inline size_t cpu_dcache_line_size()
{

	  return XPAR_MICROBLAZE_DCACHE_LINE_LEN ;
}

#endif



#endif

