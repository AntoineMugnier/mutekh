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

#define DEV_IRQ_EP_PROCESS(n) void (n) (struct dev_irq_ep_s *ep, int_fast16_t *id)

/**
   @This is interrupt end-point processing function. It must propagate
   interrupt to other connected end-points. The implementation is
   different for source and sink end-points.

   The @tt id parameter can be used when multiple logical interrupts
   are multiplexed on the same end-point.

   Source and sink end-points reference implementations are provided
   below.

   Shortcuts and simplified versions of these functions are generally
   used instead of these default implementations for performance
   reasons. Actual function used for sink end-point processing depends
   on end-point links count and is updated when links are
   changed. Function used for source end-points processing depends on
   method used to initializes the end-point.

   @example libdevice/irq.c:source_process
   @example libdevice/irq.c:sink_process
 */
typedef DEV_IRQ_EP_PROCESS(dev_irq_ep_process_t);

#define DEV_IRQ(n) struct dev_irq_ep_s * (n) (struct dev_irq_ep_s *src, int_fast16_t *id)

/** Common device class irq() function type. Must be called on
    interrupt request.

    * @param dev pointer to device descriptor
    * @return 1 if interrupt have been handled by the device
    */

/**
   @This is irq handling function of provided by device driver. I must
   handle interrupt for the assocatied device. In case of interrupt
   controller device, it may return a pointer to the next sink
   end-point in the chain.

   @param src end point which relayed the irq.
   @param id identifier of logical irq for relaying device.

   Interrupt controller device drivers return pointer to next sink
   end-point or NULL. Other kind of device drivers always return NULL.

   Interrupt controller device drivers have to find the next sink
   end-point from their internal registers or passed id value. On some
   systems the Icu passes the decoded vector id to the processor in
   hardware and we need a way to pass this value back from one Icu
   handler to the next. Icu devices may change the id value so that it
   is relevant for the next handler.  */
typedef DEV_IRQ(dev_irq_t);



#define DEV_IRQ_ACK(n) void (n) (struct dev_irq_ep_s *sink, int_fast16_t id)
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

/** @internal @This creates a link between a source end-point and a sink-endpoint. */
config_depend(CONFIG_DEVICE_IRQ)
error_t device_irq_ep_link(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink);

/** @internal @This removes a link between a source end-point and a
    sink end-point. An error is returned if no such link exists.

    If the @tt NULL pointer is passed as one of the argument, all
    links of the valid end-point pointer are removed. No error are
    returned if no link exist.  */
config_depend(CONFIG_DEVICE_IRQ)
error_t device_irq_ep_unlink(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink);

/** @This initializes an array of device interrupt source
    end-points. The @ref device_irq_tail_source_init and @ref
    device_irq_icu_source_init function can be used to setup faster
    interrupt call path. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_source_init(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count, dev_irq_t *handler);

/** @This initializes an array of device interrupt source
    end-points. @This setups a shortcut for faster interrupts
    handling which prevent the handler from returning a sink
    end-point. This variant of @ref device_irq_source_init can not be
    used to setup source end-points for interrupt controllers devices. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_tail_source_init(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count, dev_irq_t *handler);

/** @This initializes an array of device interrupt source
    end-points. @This setups an end-point processing function directly
    instead of an interrupt handler. This shortcut improves interrupt
    processing speed. The interrupt controller drivers must provide a
    single function which re-implement the default end-point
    processing code along with the handler stuff. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_icu_source_init(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count, dev_irq_ep_process_t *process);

/** @This initializes an array of device interrupt sink end-points */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_sink_init(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t src_count, dev_irq_ack_t *ack_handler);

/** @This links device interrupt source end-points to appropriate sink
    end-points of interrupt controllers as described in device
    resources. @This is usually called from device driver
    initialization function passing an array of source end-points
    allocated in the private driver data for the device. */
config_depend(CONFIG_DEVICE_IRQ)
error_t device_irq_source_link(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count);

/** @This unlink device interrupt end-points. @This is usually
    called from device driver cleanup function passing an array
    of source/sink end-points allocated in the private driver data. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_source_unlink(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count);

/** @This unlink device interrupt end-points. @This is usually
    called from device driver cleanup function passing an array
    of source/sink end-points allocated in the private driver data. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_sink_unlink(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t sink_count);

/** @This is used when an IRQ line resource entry on a device has a
    @tt NULL pointer as reference to an interrupt controller. @This
    queries the enumerator device associated with the device. */
config_depend(CONFIG_DEVICE_IRQ)
struct device_s * device_get_default_icu(struct device_s *dev);

#endif

