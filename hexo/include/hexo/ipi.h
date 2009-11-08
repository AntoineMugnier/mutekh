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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2009

*/

/**
 * @file
 * @module{Hexo}
 * @short Inter-processor interrupts
 */

#ifndef IPI_H_
#define IPI_H_

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_dlist.h>
#include <hexo/gpct_lock_hexo.h>

#include "error.h"
#include "local.h"
#include <device/device.h>

#define IPI_MSG_FUNC(n) void (n)(void *private)

typedef IPI_MSG_FUNC(ipi_msg_func_t);

extern CPU_LOCAL struct device_s *ipi_icu_dev;
extern CPU_LOCAL void *ipi_cpu_id;

#define CONTAINER_LOCK_ipi_queue HEXO_SPIN

CONTAINER_TYPE(ipi_queue, DLIST,
struct ipi_request_s
{
  ipi_msg_func_t *func;
  void *private;
  ipi_queue_entry_t queue_entry;
}, queue_entry);

CONTAINER_FUNC(ipi_queue, DLIST, static inline, ipi_queue);

/**
   Send an ipi to given processor. Processor is identified using its
   cpu local storage pointer.
   @return zero if ipi was sent
   @see #CPU_LOCAL_ID_ADDR
 */
error_t ipi_post(void *cpu_cls);

/**
   Attach the given callback for execution on target processor and
   send an ipi to given processor on success  Processor is identified using its
   cpu local storage pointer.

   @return zero if message was attached and ipi sent
   @see #CPU_LOCAL_ID_ADDR
 */
error_t ipi_post_rq(void *cpu_cls, struct ipi_request_s *rq);

/**
   Request processing of pending messages on current processor. Called from icu driver
 */
void ipi_process_rq();

/**
   Setup a IPI device for a given CPU.

   @param cpu_cls CPU's cls to hook up in
   @param ipi_icudev Icudev handling the IPIs
   @param privdata Icudev private data returned by @ref dev_icu_setupipi
 */
void ipi_hook_cpu(void *cpu_cls,
				  struct device_s *ipi_icudev,
				  void *privdata);

#endif

