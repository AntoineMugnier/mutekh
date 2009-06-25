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

/**
 * @file
 * @module{Hexo}
 * @short Processor local and context local, language global variables features
 */

#ifndef LOCAL_H_
#define LOCAL_H_

# define CPU_LOCAL

#include "cpu/hexo/local.h"

/************************************************************************/

#ifdef CONFIG_SMP

extern void * cpu_local_storage[CONFIG_CPU_MAXCOUNT];

# ifndef CPU_GET_CLS_ID
#  define CPU_GET_CLS_ID(cpuid) cpu_local_storage[(cpuid)]
# endif

# ifndef CPU_LOCAL
#  define CPU_LOCAL
# endif

# ifndef CPU_LOCAL_SET
#  define CPU_LOCAL_SET(n, v) { *CPU_LOCAL_ADDR(n) = (v); }
# endif

# ifndef CPU_LOCAL_CLS_SET
#  define CPU_LOCAL_CLS_SET(cls, n, v) { *(typeof(n)*)((uintptr_t)(cls) + (uintptr_t)&(n)) = (v); }
# endif

# ifndef CPU_LOCAL_GET
#  define CPU_LOCAL_GET(n) ({ *CPU_LOCAL_ADDR(n); })
# endif

# ifndef CPU_LOCAL_CLS_GET
#  define CPU_LOCAL_CLS_GET(cls, n) ({ *(typeof(n)*)((uintptr_t)(cls) + (uintptr_t)&(n)); })
# endif

/** get address of cpu local object */
#ifndef CPU_LOCAL_ADDR
# define CPU_LOCAL_ADDR(n) ( (typeof(n)*)((uintptr_t)CPU_GET_CLS() + (uintptr_t)&(n)) )
#endif

/** get address of cpu local object for given cpu local storage */
# ifndef CPU_LOCAL_CLS_ADDR
#  define CPU_LOCAL_CLS_ADDR(cls, n)	( (typeof(n)*)((uintptr_t)(cls) + (uintptr_t)&(n)) )
# endif

/** get address of cpu local object for given cpuid */
# ifndef CPU_LOCAL_ID_ADDR
#  define CPU_LOCAL_ID_ADDR(cpuid, n) CPU_LOCAL_CLS_ADDR(CPU_GET_CLS_ID(cpuid), n)
# endif

#else

# define CPU_LOCAL_ADDR(n)   (&(n))
# define CPU_LOCAL_CLS_ADDR(cls, n) (&(n))
# define CPU_LOCAL_ID_ADDR(cpuid, n) (&(n))
# define CPU_LOCAL_GET(n)    (n)
# define CPU_LOCAL_SET(n, v)  { (n) = (v); }

#endif

/************************************************************************/

# ifndef CONTEXT_LOCAL
#  define CONTEXT_LOCAL
# endif

/** context local storage variable assignement */
#ifndef CONTEXT_LOCAL_SET
# define CONTEXT_LOCAL_SET(n, v)	{ *(typeof(n)*)((uintptr_t)CONTEXT_GET_TLS() + (uintptr_t)&(n)) = (v); }
#endif

/** context local storage variable assignement for different context */
#ifndef CONTEXT_LOCAL_TLS_SET
# define CONTEXT_LOCAL_TLS_SET(tls, n, v) { *(typeof(n)*)((uintptr_t)(tls) + (uintptr_t)&(n)) = (v); }
#endif

/** context local storage variable read access */
#ifndef CONTEXT_LOCAL_GET
# define CONTEXT_LOCAL_GET(n) 	({ *(typeof(n)*)((uintptr_t)CONTEXT_GET_TLS() + (uintptr_t)&(n)); })
#endif

/** context local storage variable read access for different context */
#ifndef CONTEXT_LOCAL_TLS_GET
# define CONTEXT_LOCAL_TLS_GET(tls, n) 	({ *(typeof(n)*)((uintptr_t)(tls) + (uintptr_t)&(n)); })
#endif

/** get address of context local object */
#ifndef CONTEXT_LOCAL_ADDR
# define CONTEXT_LOCAL_ADDR(n)	({ (void*)((uintptr_t)CONTEXT_GET_TLS() + (uintptr_t)&(n)); })
#endif

/** get address of context local object for different context */
#ifndef CONTEXT_LOCAL_TLS_ADDR
# define CONTEXT_LOCAL_TLS_ADDR(tls, n)	({ (void*)((uintptr_t)(tls) + (uintptr_t)&(n)); })
#endif

/************************************************************************/

/** cpu architecture local storage type attribute */
#ifdef CONFIG_SMP
#define CPUARCH_LOCAL	__attribute__((section (".cpuarchdata")))
#else
#define CPUARCH_LOCAL
#endif

/** cpu architecture local storage variable assignement */
#define CPUARCH_LOCAL_SET(n, v)  (n) = (v)

/** cpu architecture local storage variable read access */
#define CPUARCH_LOCAL_GET(n)    (n)

/** get address of cpu architecture local object */
#define CPUARCH_LOCAL_ADDR(n)   (&(n))

#endif

