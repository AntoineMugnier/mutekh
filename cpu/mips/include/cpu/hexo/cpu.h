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

#if !defined(__CPU_H_) || defined(CPU_CPU_H_)
#error This file can not be included directly
#else
#define CPU_CPU_H_

#if defined(CONFIG_CPU_MIPS_VERSION) && CONFIG_CPU_MIPS_VERSION != CONFIG_CPU_MIPS_VERSION
# warning compiler mips version doesnt match configuration
#endif

/* Cacheable, non-coherent, write-through, no write allocate */
#define CPU_MIPS_CACHE_WTNA     0
/* Cacheable, non-coherent, write-through, write allocate */
#define CPU_MIPS_CACHE_WTA      1
/* Uncached */
#define CPU_MIPS_NO_CACHE       2
/* Cacheable, non-coherent, write-back, write allocate */
#define CPU_MIPS_CACHE_WB       3


#define CPU_MIPS_GP             28
#define CPU_MIPS_SP             29
#define CPU_MIPS_FP             30
#define CPU_MIPS_RA             31

#define CPU_MIPS_STATUS         12
#define CPU_MIPS_CAUSE          13
#define CPU_MIPS_EPC            14
#define CPU_MIPS_BADADDR        8
#define CPU_MIPS_EEPC           30

# define CPU_MIPS_STATUS_FPU    0x20000000
/** interrupts enabled */
# define CPU_MIPS_STATUS_EI     0x00000001

# define CPU_MIPS_STATUS_IM     0x0000fc00
# define CPU_MIPS_STATUS_IM_SHIFT 10

#ifdef CONFIG_CPU_MIPS_USE_ERET
/** exception mode */
# define CPU_MIPS_STATUS_EXL    0x00000002
/** user mode */
# define CPU_MIPS_STATUS_UM     0x00000010

#else
/** interruptes enabled */
# define CPU_MIPS_STATUS_EIc    CPU_MIPS_STATUS_EI
/** kernel mode when set */
# define CPU_MIPS_STATUS_KUc    0x00000002
/** previous interruptes enabled */
# define CPU_MIPS_STATUS_EIp    0x00000004
/** previous kernel mode when set */
# define CPU_MIPS_STATUS_KUp    0x00000008

#endif

# define CPU_MIPS_CAUSE_BD      0x80000000

#include <hexo/bit.h>

/** general purpose regsiters count */
# define CPU_GPREG_COUNT	32

# define CPU_GPREG_NAMES 								   \
"zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",					   \
"t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",						   \
"s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",						   \
"t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"						   \


# if CONFIG_CPU_MIPS_VERSION >= 32 

# define cpu_mips_mfc0(id, sel)			\
({						\
  reg_t _reg;					\
						\
  asm volatile ("mfc0	%0,	$%1, %2	\n"	\
		: "=r" (_reg)			\
		: "i" (id)			\
		, "i" (sel)			\
		);				\
						\
  _reg;						\
})

# define cpu_mips_mtc0(id, sel, val)		\
({						\
  reg_t _reg = val;				\
						\
  asm volatile ("mtc0	%0,	$%1, %2	\n"	\
				:: "r" (_reg)	\
				, "i" (id)	\
		, "i" (sel)			\
		);				\
})

# define cpu_mips_mfc2(id, sel)			\
({						\
  reg_t _reg;					\
						\
  asm volatile ("mfc2	%0,	$%1, %2	\n"	\
		: "=r" (_reg)			\
		: "i" (id)			\
		, "i" (sel)			\
		);				\
						\
  _reg;						\
})

# define cpu_mips_mtc2(id, sel, val)		\
({						\
  reg_t _reg = val;				\
						\
  asm volatile ("mtc2	%0,	$%1, %2	\n"	\
				:: "r" (_reg)	\
				, "i" (id)	\
		, "i" (sel)			\
		);				\
})


# else

# define cpu_mips_mfc0(id, sel)			\
({						\
  reg_t _reg;					\
						\
  asm volatile ("mfc0	%0,	$%1 \n"		\
		: "=r" (_reg)			\
		: "i" (id)			\
		);				\
						\
  _reg;						\
})

# define cpu_mips_mtc0(id, sel, val)										   \
({						\
  reg_t _reg = val;				\
						\
  asm volatile ("mtc0	%0,	$%1 \n"		\
				:: "r" (_reg)	\
				, "i" (id)	\
		);				\
})

# define cpu_mips_mfc2(id, sel)			\
({						\
  reg_t _reg;					\
						\
  asm volatile ("mfc2	%0,	$%1 \n"		\
		: "=r" (_reg)			\
		: "i" (id)			\
		);				\
						\
  _reg;						\
})

# define cpu_mips_mtc2(id, sel, val)		\
({						\
  reg_t _reg = val;				\
						\
  asm volatile ("mtc2	%0,	$%1 \n"		\
				:: "r" (_reg)	\
				, "i" (id)	\
		);				\
})


# endif

ALWAYS_INLINE
reg_t cpu_get_stackptr(void)
{
    reg_t ret;
    asm("move %0, $sp": "=r"(ret));
    return ret;
}

ALWAYS_INLINE cpu_id_t
cpu_id(void)
{
	return (reg_t)cpu_mips_mfc0(15, 1) & (reg_t)0x000003ff;
}

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
  return cpu_id() == CONFIG_ARCH_BOOTSTRAP_CPU_ID;
}

ALWAYS_INLINE void
cpu_trap(void)
{
  asm volatile ("break 0");
}

#define  MIPS32_CACHE_OP_INDEX_INVALIDATE  (0 << 2)
#define  MIPS32_CACHE_OP_INDEX_STORE_TAG   (2 << 2)
#define  MIPS32_CACHE_OP_HIT_INVALIDATE    (4 << 2)
#define  MIPS32_CACHE_OP_HIT_WRITEBACK     (5 << 2)

#define  MIPS32_CACHE_INSTRUCTION  0
#define  MIPS32_CACHE_DATA         1
#define  MIPS32_CACHE_TERTIARY     2
#define  MIPS32_CACHE_SECONDARY    3

ALWAYS_INLINE void cpu_dcache_invld(void *ptr)
{
  asm volatile (
# if CONFIG_CPU_MIPS_VERSION >= 32
		" cache %0, %1"
		: : "i" (0x11) , "R" (*(uint8_t*)(ptr))
# endif
		: "memory"
		);
}

ALWAYS_INLINE void cpu_dcache_flush(void *ptr)
{
  asm volatile (
# if CONFIG_CPU_MIPS_VERSION >= 32
		" cache %0, %1"
		: : "i" (0x19) , "R" (*(uint8_t*)(ptr))
# endif
		: "memory"
		);
}

ALWAYS_INLINE size_t cpu_dcache_line_size(void)
{
  reg_t r0 = cpu_mips_mfc0(16, 0);
  reg_t r1 = cpu_mips_mfc0(16, 1);

  if (bit_get(r0, 31))
    {
      r1 = bit_get_range(r1, 10, 12);

      if (r1)
	return 2 << r1;
    }

  return 8;
}

# endif

