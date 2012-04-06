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
 * @module{Device drivers}
 * @short Devices definitions
 */

#ifndef __DEVICE_IRQ_H__
#define __DEVICE_IRQ_H__

struct device_s;
struct driver_s;

#include <hexo/types.h>
#include <hexo/error.h>

struct dev_irq_ep_s;
struct device_s;

#define DEV_IRQ_EP_PROCESS(n) bool_t (n) (struct dev_irq_ep_s *ep, int_fast16_t id)
typedef DEV_IRQ_EP_PROCESS(dev_irq_ep_process_t);

#define DEV_IRQ(n) struct dev_irq_ep_s * (n) (struct dev_irq_ep_s *src, int_fast16_t *id)

/** Common device class irq() function type. Must be called on
    interrupt request.

    * @param dev pointer to device descriptor
    * @return 1 if interrupt have been handled by the device
    */

/**
   @This is irq handling function of device node.

   @param src end point which relayed the irq.
   @param id local identifier of irq line for relaying device.

   Icu devices return pointer to next irq sink end-point or
   NULL. Non-icu devices always return NULL.

   The id must be changed to -1 when no irq were pending.
   Non-icu devices only set to -1 or 0.

   Icu devices have to determine the next sink endpoint from its
   internal registers or passed id value. On some systems the icu
   passes the decoded vector id to the processor in hardware and we
   need a way to pass this value back from one icu handler to the next
   one. Icu devices may change the id value so that it is relevant for
   the next handler.
*/
typedef DEV_IRQ(dev_irq_t);



#define DEV_IRQ_ACK(n) void (n) (struct dev_irq_ep_s *sink, int_fast16_t id, int_fast16_t next_id)
typedef DEV_IRQ_ACK(dev_irq_ack_t);

#ifdef CONFIG_DEBUG
enum dev_irq_ep_type_e
{
  DEV_IRQ_EP_SOURCE,
  DEV_IRQ_EP_SINK,
  DEV_IRQ_EP_MIXED,
};
#endif

/** Device irq end-point object. Irq source and sink endpoints are
    linked together to make irqs topology graph */
struct dev_irq_ep_s
{
  struct device_s *dev;

  /** Irq event handling function for endpoint */
  dev_irq_ep_process_t *process;

  union {
    /** For source ep: Device irq handler as provided by the device driver. */
    dev_irq_t *f_irq;
    /** For sink ep: IRQ ack function provided by the icu driver */
    dev_irq_ack_t *f_irq_ack;
  };

#ifdef CONFIG_DEBUG
  enum dev_irq_ep_type_e type;
#endif

  /** Number of links */
  uint_fast8_t links_count;

  union {
    /** Single link case */
    struct dev_irq_ep_s *single;

    /** Multiple links case */
    struct dev_irq_ep_s **array;
  }  /** For source ep: list of sink ep which can recieve the irq signal,
         for sink ep: list of source ep which can relay this irq */
    links;
};

/** @internal @This initializes a source end-point. The @ref
    device_irq_ep_unlink function can be used to perform end-point
    cleanup. */
void device_irq_ep_source_init(struct dev_irq_ep_s *source, struct device_s *dev, dev_irq_t *handler);

/** @internal @This initializes a source end-point. The @tt
    ack_handler parameter may be NULL if no acknowledgment handler is
    needed. The @ref device_irq_ep_unlink function can be used to
    perform end-point cleanup. */
void device_irq_ep_sink_init(struct dev_irq_ep_s *sink, struct device_s *dev, dev_irq_ack_t *ack_handler);

/** @internal @This creates a link between a source end-point and a sink-endpoint. */
error_t device_irq_ep_link(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink);

/** @internal @This removes a link between a source end-point and a
    sink end-point. An error is returned if no such link exists.

    If the @tt NULL pointer is passed as one of the argument, all
    links of the valid end-point pointer are removed. No error are
    returned if no link exist.  */
error_t device_irq_ep_unlink(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink);

/** @This initializes and links device interrupt source end-points to
    appropriate sink end-points of interrupt controllers as described
    in device resources. @This is usually called from device driver
    initialization function passing an array of source end-points
    allocated in the private driver data for the device. */
error_t device_irq_link(struct device_s *dev, dev_irq_t *handler,
                        struct dev_irq_ep_s *source, uint_fast8_t src_count);

/** @This unlink device interrupt source end-points. @This is usually
    called from device driver cleanup function passing an array
    of source end-points allocated in the private driver data. */
void device_irq_unlink(struct device_s *dev, struct dev_irq_ep_s *source, uint_fast8_t src_count);

#endif

