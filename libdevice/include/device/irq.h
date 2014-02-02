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
 * @module{Devices support library}
 * @short Device interrupts graph
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

   The @tt id parameter is used when multiple logical interrupts
   are multiplexed on the same end-point.

   Sink end-point implementation depends on number of connected source
   end-points. Implementations for possible cases are provided by the
   device library. Sink end-points with no connection are bound to a
   lazy irq disabling function.

   Source end-points use an implementation provided by the device
   driver. Interrupt controller source end-point implementations are
   responsible for calling the sink end-point process function to
   relay interrupt event.

   @example libdevice/irq.c:source_process
   @example libdevice/irq.c:sink_process
 */
typedef DEV_IRQ_EP_PROCESS(dev_irq_ep_process_t);

//#define DEV_IRQ(n) void (n) (struct dev_irq_ep_s *src, int_fast16_t *id)

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
   end-point from their internal registers or passed logical interrupe
   id value. On some systems the interrupt controller passes the
   decoded logical interrupt id to the processor in hardware and we
   need a way to pass this value back from one software handler to the
   next one. Interrupt controller devices may update the id value so
   that it is relevant for the next handler. */
//typedef DEV_IRQ(dev_irq_t);

enum dev_irq_ep_type_e
{
  DEV_IRQ_EP_SOURCE,
  DEV_IRQ_EP_SINK,
  DEV_IRQ_EP_BYPASS,
};

struct dev_irq_bypass_s;

enum dev_irq_sense_modes_e
{
  DEV_IRQ_SENSE_HIGH_LEVEL = 1,
  DEV_IRQ_SENSE_LOW_LEVEL = 2,
  DEV_IRQ_SENSE_RISING_EDGE = 4,
  DEV_IRQ_SENSE_FALLING_EDGE = 8,
  DEV_IRQ_SENSE_UNKNOWN_HARDWIRED = 16,
};

/** Device irq end-point object. Irq source and sink endpoints are
    linked together to make irqs topology graph */
struct dev_irq_ep_s
{
  struct device_s *dev;

  /** Irq event handling function for endpoint */
  dev_irq_ep_process_t *process;

#ifdef CONFIG_DEBUG
  enum dev_irq_ep_type_e type;
#endif

  union {
    /** Single link case */
    struct dev_irq_ep_s *single;

    /** Multiple links case */
    struct dev_irq_ep_s **array;
  }  /** For source ep: list of sink ep which can recieve the irq signal,
         for sink ep: list of source ep which can relay this irq */
    links;

#ifdef CONFIG_DEVICE_IRQ_BYPASS
  /** pointer to first bypass end-point for this source end-point*/
  struct dev_irq_bypass_s *bypass_list;
#endif

  /** Number of links */
  uint16_t links_count;

  /** Interrupt sense mode capabilities
      @see devicu_enable_irq_t
      @see devicu_get_endpoint_t */
  uint8_t sense;
};

#ifdef CONFIG_DEVICE_IRQ_BYPASS
/** Device irq bypass end-point. This end-point type can be used to
    bypass parts of the irq topology graph when an interrupt is
    raised. It is setup when interrupts are enabled from the @ref
    devicu_enable_irq_t function if one or more interrupt controllers
    agree on being bypassed for interrupts handling.

    A bypass end-point can not be used to point to multiple source
    end-points. When trying to link more than one source because
    interrupt sharing is used, the bypass end-point is marked as
    unusable and all links are droped. This way shared interrupts
    require full irq graph traversal. */
struct dev_irq_bypass_s
{
  /** Pointer to source end-point. */
  struct dev_irq_ep_s *src;
  /** Pointer to next bypass end-point for the same source end-point.
      Points to itself when end-point is marked unusable. */
  struct dev_irq_bypass_s *next;
};

/** @This initializes bypass end-points. */
void device_irq_bypass_init(struct dev_irq_bypass_s *bypass, uint_fast8_t count);

/** @This calls @ref device_irq_bypass_unlink for bypass end-points in array. */
void device_irq_bypass_cleanup(struct dev_irq_bypass_s *bypass, uint_fast8_t count);

/** @This links a bypass end-point to given source end-point and
    return @tt 0 on success.

    If the bypass end-point is already linked to a different source
    end-point or if the @tt src pointer is NULL, the operation fails,
    the existing link is removed, the bypass end-point is marked a not
    usable any more and @tt -EBUSY is returned.

    If the bypass end-point is already linked to the passed source
    end-point, nothing is changed and @tt -EEXISTS is returned.
*/
error_t device_irq_bypass_link(struct dev_irq_ep_s *src, struct dev_irq_bypass_s *bypass);

/** @This drop the link to a source end-point if any. If the bypass
    end-point is marked as not usable, this mark is not removed. */
void device_irq_bypass_unlink(struct dev_irq_bypass_s *bypass);

/** @This removes all existing links between specified source
    end-point and bypass end-points. @This function is called from
    @ref device_irq_source_unlink. */
void device_irq_bypass_src_cleanup(struct dev_irq_ep_s *src);

#endif

/** @internal @This creates a link between a source end-point and a sink-endpoint. */
config_depend(CONFIG_DEVICE_IRQ)
error_t device_irq_ep_link(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink);

/** @internal @This removes a link between a source end-point and a
    sink end-point. An error is returned if no such link exists. */
config_depend(CONFIG_DEVICE_IRQ)
error_t device_irq_ep_unlink(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink);

/** @This initializes an array of interrupt source end-points. The
    device drivers must provide a function which implement the source
    end-point irq handler. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_source_init(struct device_s *dev, struct dev_irq_ep_s *sources,
                            uint_fast8_t count, dev_irq_ep_process_t *process,
                            enum dev_irq_sense_modes_e sense_capabilities);

/** @This initializes an array of device interrupt sink end-points */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_sink_init(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t count,
                          enum dev_irq_sense_modes_e sense_capabilities);

/** @This links device interrupt source end-points to appropriate sink
    end-points of interrupt controllers as described in device
    resources. @This is usually called from device driver
    initialization function passing an array of source end-points
    allocated in the private driver data for the device.

    When called from interrupt controller driver code, the @tt enable
    parameter must be 0. When called from other device drivers the
    bits in the @tt enable parameter must be 1 in order to enable
    device interrupts in all interrupt controllers along the path to
    the processor(s). The lsb is used for the first entry of the
    source end-points table. Once this function has been called,
    device drivers must be ready to receive interrupts.

    The source end-point sense mode capabilities flags must be set
    properly before calling this function. If the owner of the source
    end-point advertises multiple capabilities before enabling
    interrupts on its source end-points, it must check the selected
    configuration on return and configure the hardware
    accordingly. */
config_depend(CONFIG_DEVICE_IRQ)
error_t device_irq_source_link(struct device_s *dev, struct dev_irq_ep_s *sources,
                               uint_fast8_t count, uint32_t enable_mask);

/** @This unlink device interrupt end-points. @This is usually
    called from device driver cleanup function passing an array
    of source/sink end-points allocated in the private driver data. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_source_unlink(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t count);

/** @This unlink device interrupt end-points. @This is usually
    called from device driver cleanup function passing an array
    of source/sink end-points allocated in the private driver data. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_sink_unlink(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t count);

#endif

