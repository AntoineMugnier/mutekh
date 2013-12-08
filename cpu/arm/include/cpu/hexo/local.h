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

    Copyright (c) Nicolas Poullon <nipo@ssji.net>, 2009
	              Alexandre Becoulet <alexandre.becoulet@free.fr>, 2009

*/


/**
   @file Task local and CPU local variables access
  */

#if !defined(LOCAL_H_) || defined(CPU_LOCAL_H_)
#error This file can not be included directly
#else

#define CPU_LOCAL_H_

#include "cpu/hexo/specific.h"

#ifdef CONFIG_CPU_ARM_ARCH_PROFILE_A

# if CONFIG_CPU_ARM_ARCH_VERSION >= 6

/**************************************/

#  ifdef CONFIG_ARCH_SMP

#   undef CPU_LOCAL
#   define CPU_LOCAL	__attribute__((section (".cpudata")))

#   define CPU_GET_CLS()                                                  \
        ({                                                              \
                uintptr_t _ptr_;                                        \
                THUMB_TMP_VAR;                                          \
                                                                        \
                asm (                                                   \
                     THUMB_TO_ARM                                       \
                     "mrc p15,0,%[ptr],c13,c0,3\n\t"                    \
                     ARM_TO_THUMB                                       \
                     : [ptr] "=r" (_ptr_) /*,*/ THUMB_OUT(,)            \
  /* prevent optimize if memory has been reloaded (possible context switch) */ \
                     : "m" (*(reg_t*)4)                                 \
                     );                                                 \
                                                                        \
                _ptr_;                                                  \
        })

#  else /* if !CONFIG_ARCH_SMP */

#   define CPU_LOCAL

#  endif /* CONFIG_ARCH_SMP */

/**************************************/

/** context local storage type attribute */
#  define CONTEXT_LOCAL	__attribute__((section (".contextdata")))

/** get address of cpu local object */
#  define CONTEXT_GET_TLS()                                             \
        ({                                                              \
                uintptr_t _ptr_;                                        \
                THUMB_TMP_VAR;                                          \
                                                                        \
                asm (                                                   \
                     THUMB_TO_ARM                                       \
                     "mrc p15,0,%[ptr],c13,c0,4\n\t"                    \
                     ARM_TO_THUMB                                       \
                     : [ptr] "=r" (_ptr_) /*,*/ THUMB_OUT(,)            \
                     );                                                 \
                                                                        \
                _ptr_;                                                  \
        })


# else /* CONFIG_CPU_ARM_ARCH_VERSION < 6 */

#  ifdef CONFIG_ARCH_SMP
#   error No SMP supported without TLS register (arm arch < v6)
#  endif

#  define CPU_LOCAL

#  ifndef __MUTEK_ASM__
extern CPU_LOCAL void *__context_data_base;
#  endif

#  define CONTEXT_LOCAL	__attribute__((section (".contextdata")))
#  define CONTEXT_GET_TLS() ((uintptr_t)__context_data_base)

# endif /* CONFIG_CPU_ARM_ARCH_VERSION */

#endif /* defined(CONFIG_CPU_ARM_ARCH_PROFILE_A) */



/************************************************************************/

#ifdef CONFIG_CPU_ARM_ARCH_PROFILE_M

# define CPU_LOCAL

/** context local storage type attribute */
#  define CONTEXT_LOCAL	__attribute__((section (".contextdata")))

#  ifndef __MUTEK_ASM__
extern CONTEXT_LOCAL struct cpu_context_s arm_context_regs;
#  endif

/** get address of cpu local object */
#  define CONTEXT_GET_TLS()                                             \
        ({                                                              \
                uintptr_t _ptr_;                                        \
                THUMB_TMP_VAR;                                          \
                                                                        \
                asm (                                                   \
                     "mrs %0, psp\n\t"                                  \
                     : [ptr] "=r" (_ptr_) /*,*/ THUMB_OUT(,)            \
                     );                                                 \
                                                                        \
                _ptr_;                                                 \
        })

#endif /* defined(CONFIG_CPU_ARM_ARCH_PROFILE_A) */


#endif

