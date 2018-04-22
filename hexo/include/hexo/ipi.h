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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2009

*/

/**
 * @file
 * @module {Core::Hardware abstraction layer}
 * @short Inter-processor interrupts
 */

#ifndef IPI_H_
#define IPI_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>
#include <gct/container_dlist.h>
#include <gct/container_slist.h>

#include "error.h"
#include "local.h"
#ifdef CONFIG_DEVICE
# include <device/device.h>
#endif

#define IPI_MSG_FUNC(n) void (n)(void *priv)

typedef IPI_MSG_FUNC(ipi_msg_func_t);

#define GCT_CONTAINER_ALGO_idle_cpu_queue SLIST
#define GCT_CONTAINER_ORPHAN_idle_cpu_queue

#define GCT_CONTAINER_LOCK_ipi_queue HEXO_LOCK
#define GCT_CONTAINER_ALGO_ipi_queue DLIST

struct ipi_request_s
{
  ipi_msg_func_t *func;
  void *priv;

  GCT_CONTAINER_ENTRY(ipi_queue, queue_entry);
};

GCT_CONTAINER_TYPES(ipi_queue, struct ipi_request_s *, queue_entry);

struct ipi_endpoint_s
{
    struct device_s *icu_dev;
    void *priv;
    ipi_queue_root_t ipi_fifo;
    GCT_CONTAINER_ENTRY(idle_cpu_queue, list_entry);
};

GCT_CONTAINER_FCNS(ipi_queue, inline, ipi_queue,
                   init, destroy, pushback, pop, wrlock, unlock);

GCT_CONTAINER_NOLOCK_FCNS(ipi_queue, inline, ipi_queue_nolock,
                          isempty);

extern CPU_LOCAL struct ipi_endpoint_s ipi_endpoint;


/**
   Send an ipi to given endpoint.

   @param endpoint Pointer to ipi endpoint
   @return zero if ipi was sent
 */
error_t ipi_post(struct ipi_endpoint_s *endpoint);

/**
   Attach the given callback for execution on target processor and
   send an ipi to given endpoint.

   @param endpoint Pointer to ipi endpoint
   @param rq Request buffer
   @return zero if message was attached and ipi sent
   @see #CPU_LOCAL_ID_ADDR
 */
error_t ipi_post_rq(struct ipi_endpoint_s *endpoint, struct ipi_request_s *rq);

/**
   Request processing of pending messages on current processor. Must
   be called from icu driver
 */
void ipi_process_rq(void);

/**
   Checks whether a given endpoint may receive IPIs.

   @param endpoint IPI endpoint to check
   @return whether endpoint may receive IPIs
*/
ALWAYS_INLINE
bool_t ipi_endpoint_isvalid(struct ipi_endpoint_s *endpoint)
{
    return endpoint != NULL && endpoint->icu_dev != NULL;
}

C_HEADER_END

#endif
