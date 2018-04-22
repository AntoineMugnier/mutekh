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

/**
 * @file
 * @module {Core::Hardware abstraction layer}
 * @short Startup and misc cpu related functions
 */

#ifndef __CPU_H_
#define __CPU_H_

#include <hexo/types.h>
#include <hexo/error.h>

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <cpu/hexo/cpu.h>

#define __CPU_NAME_DECL(t, x) t##_##x
#define _CPU_NAME_DECL(t, x) __CPU_NAME_DECL(t, x)
/** @this can be used to declare and refer to a variable
    or function prefixed by cpu type name. */
#define CPU_NAME_DECL(x) _CPU_NAME_DECL(CONFIG_CPU_NAME, x)

/** return CPU architecture type name */
ALWAYS_INLINE const char *cpu_type_name(void);

/** return true if bootstap processor */
ALWAYS_INLINE bool_t cpu_isbootstrap(void);

/** cpu trap instruction */
ALWAYS_INLINE void cpu_trap(void);

/** get cpu cache line size, return 0 if no dcache */
ALWAYS_INLINE size_t cpu_dcache_line_size(void);

/** invalidate the cpu data cache line containing this address */
ALWAYS_INLINE void cpu_dcache_invld(void *ptr);

# if defined(CONFIG_CPU_CACHE)

/** invalidate all the cpu data cache lines within given range.
    size is in bytes. */
void cpu_dcache_invld_buf(void *ptr, size_t size);

/** invalidate all the cpu instruction cache lines within given range.
    size is in bytes. */
void cpu_icache_invld_buf(void *ptr, size_t size);

/** invalidate all the cpu data cache lines within given range.
    size is in bytes. */
void cpu_dcache_flush_buf(void *ptr, size_t size);

# else

ALWAYS_INLINE void
cpu_dcache_invld_buf(void *ptr, size_t size)
{
}

ALWAYS_INLINE void
cpu_icache_invld_buf(void *ptr, size_t size)
{
}

ALWAYS_INLINE void
cpu_dcache_flush_buf(void *ptr, size_t size)
{
}

# endif

# define _TO_STR(x) #x
# define TO_STR(x) _TO_STR(x)

/** @this returns the cpu type name */
ALWAYS_INLINE const char *
cpu_type_name(void)
{
  return TO_STR(CONFIG_CPU_NAME);
}

# undef _TO_STR
# undef TO_STR

C_HEADER_END

#endif
