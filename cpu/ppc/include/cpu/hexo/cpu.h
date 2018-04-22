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

#define PPC_MSR_IRQ_ENABLED     0x8000
#define PPC_MSR_USERMODE        0x4000
#define PPC_MSR_FPU_ENABLED     0x2000

/** general purpose regsiters count */
# define CPU_GPREG_COUNT	32

# define CPU_GPREG_NAMES                                                \
  "vol", " sp", "sd1", "ar0", "ar1", " a2", " a3", " a4",               \
    " a5", " a6", " a7", " v0", " v1", "sd0", " l0", " l1",             \
    " l2", " l3", " l4", " l5", " l6", " l7", " l8", " l9",             \
    "l10", "l11", "l12", "l13", "l14", "l15", "l16", "l17"


ALWAYS_INLINE cpu_id_t
cpu_id(void)
{
  reg_t         reg;

  asm volatile (
                "mfdcr %0, 0"
                : "=r" (reg)
                );

  return reg;
}

ALWAYS_INLINE
reg_t cpu_get_stackptr(void)
{
    reg_t ret;
    asm("mr %0, 1": "=r"(ret));
    return ret;
}

ALWAYS_INLINE bool_t
cpu_isbootstrap(void)
{
  return cpu_id() == CONFIG_ARCH_BOOTSTRAP_CPU_ID;
}

ALWAYS_INLINE void
cpu_trap(void)
{
  asm volatile ("trap");
}

ALWAYS_INLINE void cpu_dcache_invld(void *ptr)
{
  asm volatile (
                "dcbi 0, %0"
                :
                : "r" (ptr)
                : "memory"
                );
}

ALWAYS_INLINE void cpu_dcache_flush(void *ptr)
{
  asm volatile (
                "dcbf 0, %0"
                :
                : "r" (ptr)
                : "memory"
                );
}

ALWAYS_INLINE size_t cpu_dcache_line_size(void)
{
  return CONFIG_CPU_CACHE_LINE;
}

#endif

