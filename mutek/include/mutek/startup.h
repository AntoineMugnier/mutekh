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
 * @module{Mutek}
 * @short System initialization and statup functions
 */

#ifndef INIT_H_
#define INIT_H_

#include <hexo/decls.h>
#include <hexo/types.h>

C_HEADER_BEGIN

INIT_BOOTSTRAP_PROTOTYPES;
INIT_SMP_PROTOTYPES;

/** boot address, (located in cpu/current/boot.S) */
void cpu_boot();

/** Called from boot assembly code (@ref cpu_boot). @This is the C
    code entrypoint for the bootstrap processor. If this function is
    entered by other processors, the control jump to the @ref
    mutekh_startup_smp function. @This performs all initializations in
    the @ref #INIT_MUTEKH_STARTUP group; this includes @ref #INIT_SMP. */
void mutekh_startup(void *arg);

/** Starting point of non-bootstrap processors. @This performs all
    initializations of the @ref #INIT_SMP group. */
void mutekh_startup_smp();

#ifdef CONFIG_ARCH_SMP
/** Spin barrier which can be used to synchronize processors when
    performing intializations of the @ref #INIT_SMP group. */
void mutekh_startup_smp_barrier();
#else
static inline void mutekh_startup_smp_barrier()
{
}
#endif

/** User application entry point. This function must be defined in the
    user application code. It is executed when @ref #INIT_APPLICATION
    initialization stage is reached.

    @This is executed by the bootstrap processor only, unless @ref
    #CONFIG_MUTEK_SMP_APP_START is defined. */
void app_start();

C_HEADER_END

#endif

