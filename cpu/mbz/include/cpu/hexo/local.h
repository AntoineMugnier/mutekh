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

/**
   @file Task local and CPU local variables access
  */

#if !defined(LOCAL_H_) || defined(CPU_LOCAL_H_)
#error This file can not be included directly
#else

#define CPU_LOCAL_H_

# ifdef CONFIG_ARCH_SMP
/*
 * We could support not having dedicated registers with an indirection
 * through a global table and cpuid(), but for now, we'll be lazy
 * without proper need for it.
 */
#  error No SMP supported without TLS registers in c0

#else /* CONFIG_ARCH_SMP */

# define CPU_LOCAL

#endif /* CONFIG_ARCH_SMP */

/** pointer to context local storage in cpu local storage */
extern CPU_LOCAL void *__context_data_base;

/** context local storage type attribute */
# define CONTEXT_LOCAL	__attribute__((section (".contextdata")))

extern __ldscript_symbol_t __context_data_start;

#define CONTEXT_GET_TLS() ((uintptr_t)CPU_LOCAL_GET(__context_data_base))


#endif

