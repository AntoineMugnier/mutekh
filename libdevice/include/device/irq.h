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
 * @module {Core::Devices support library}
 * @short Device interrupts graph
 */

#ifndef __DEVICE_IRQ_H__
#define __DEVICE_IRQ_H__

struct device_s;
struct driver_s;

#include <hexo/types.h>
#include <hexo/error.h>
#include <device/resources.h>

struct dev_irq_ep_s;
struct dev_irq_src_s;
struct dev_irq_sink_s;

/** Irq affinity mask @see dev_icu_link_t */
typedef uint32_t dev_irq_route_t;
typedef uint8_t dev_irq_id_t;

enum dev_irq_sense_modes_e
{
  /** IRQ is disabled */
  DEV_IRQ_SENSE_NONE                  = 0x0000,
  /** IRQ number transmitted over a dedicated bus. When multiple
      logical interrupts are multiplexed on the same endpoint, the @tt
      id parameter of the @ref dev_irq_src_process_t function indicates
      the logocal id of the triggered interrupt. */
  DEV_IRQ_SENSE_ID_BUS                = 0x0001,
  /** Level triggered irq line, active on logic high. */
  DEV_IRQ_SENSE_HIGH_LEVEL            = 0x0002,
  /** Level triggered irq line, active on logic low. */
  DEV_IRQ_SENSE_LOW_LEVEL             = 0x0004,
  /** Rising edge triggered irq line, sampled on clock edge. */
  DEV_IRQ_SENSE_RISING_EDGE           = 0x0008,
  /** Falling edge triggered irq line, sampled on clock edge. */
  DEV_IRQ_SENSE_FALLING_EDGE          = 0x0010,
  /** Rising or falling edge triggered irq line, sampled on clock
      edge. The @tt id parameter of the @ref dev_irq_src_process_t
      function indicates the edge polarity. */
  DEV_IRQ_SENSE_ANY_EDGE              = 0x0020,
};

/** Device irq endpoint object. Irq endpoints are linked together to
    make irqs topology graph */
struct dev_irq_ep_s
{
  struct device_s *dev;

  union {
    /** Single link case */
    struct dev_irq_ep_s *single;
#if defined(CONFIG_DEVICE_IRQ_SHARING) || defined(CONFIG_DEVICE_IRQ_MULTI_SINK)
    /** Multiple links case */
    struct dev_irq_ep_s **array;
#endif
  } links;

  /** Number of links */
  uint8_t link_count;
} __attribute__ ((packed,aligned(4)));


/** @csee dev_irq_src_process_t */
#define DEV_IRQ_SRC_PROCESS(n) void (n) (struct dev_irq_src_s *ep, dev_irq_id_t id)

/**
   @This is the interrupt endpoint processing function. It is
   implemented by device drivers to handle interrupts. It is called
   from the @ref device_irq_sink_process function in interrupt
   controller code.

   The meaning of the @tt id parameter depends on the sense mode
   specified by @ref dev_irq_sense_modes_e.
 */
typedef DEV_IRQ_SRC_PROCESS(dev_irq_src_process_t);

/** Device irq source endpoint of device generating interrupts. */
struct dev_irq_src_s
{
  struct dev_irq_ep_s base;

  /** Current irq trigger mode configured on linked sink endpoints. */
  uint8_t trig_mode;

  /** Logical id of interrupt, valid when @ref trig_mode is @ref DEV_IRQ_SENSE_ID_BUS */
  dev_irq_id_t irq_id;

  /** Irq event handling function for source endpoint */
  dev_irq_src_process_t *process;
};

STRUCT_INHERIT(dev_irq_src_s, dev_irq_ep_s, base);


/** @csee dev_irq_sink_update_t */
#define DEV_IRQ_SINK_UPDATE(n) void (n) (struct dev_irq_sink_s *sink, \
                                         enum dev_irq_sense_modes_e sense,  \
                                         dev_irq_id_t irq_id)

/** @This updates the interrupt sense mode of a sink endpoint. It is
    implemented by the driver of the interrupt controller. This function
    should not be called directly. @see device_irq_src_update
    @see device_irq_src_enable @see device_irq_src_disable. */
typedef DEV_IRQ_SINK_UPDATE(dev_irq_sink_update_t);

/** Device irq sink endpoint. Sink endpoints are stored in interrupt
    controllers device private data and can be accessed by calling the
    @ref dev_icu_get_sink_t function. */
struct dev_irq_sink_s
{
  struct dev_irq_ep_s base;

  /** Mask of irq sense modes supported by the interrrupt controller
      hardware. */
  uint8_t sense_all;

  /** Mask of irq sense modes supported by both: the interrrupt
      controller and the linked interrupt source devices. This field
      is updated by the @ref device_irq_source_link function. */
  uint8_t sense_link;

  /** Interrupt controller private flags. */
  uint8_t icu_pv;

  dev_irq_sink_update_t *update;
};

STRUCT_INHERIT(dev_irq_sink_s, dev_irq_ep_s, base);

/** @This updates the sense mode of all sink endpoints linked to the
    given source endpoint. This can be used when the trigger mode of
    the source endpoint in the device resources has been declared as
    @ref DEV_IRQ_SENSE_NONE. In this case the irq can not be shared. */
error_t device_irq_src_update(struct dev_irq_src_s *src, enum dev_irq_sense_modes_e trig_mode);

/** @This enables the IRQ sensing of all linked sink endpoints. This
    can be used when at least one valid trigger mode has been
    specified for the source endpoint in the device resources. The
    IRQ can not be shared. */
error_t device_irq_src_enable(struct dev_irq_src_s *src);

/** @This disables the IRQ sensing of all linked sink endpoints. The
    IRQ can not be shared. */
ALWAYS_INLINE error_t device_irq_src_disable(struct dev_irq_src_s *src)
{
  return device_irq_src_update(src, DEV_IRQ_SENSE_NONE);
}

/** @This initializes an array of interrupt source endpoints. This is
    called from the device driver initialization function. The device
    drivers must provide a function which implement the source
    endpoint irq handler. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_source_init(struct device_s *dev, struct dev_irq_src_s *sources,
                            uint_fast8_t count, dev_irq_src_process_t *process);

/** @This initializes an array of device interrupt sink
    endpoints. This is called from the interrupt controller
    initialization function. The drivers must provide a function which
    is able to update the interrupt sense mode according to the value
    of @tt sense_mask. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_sink_init(struct device_s *dev, struct dev_irq_sink_s *sinks,
                          uint_fast8_t count, dev_irq_sink_update_t *update,
                          enum dev_irq_sense_modes_e sense_mask);

/** @This links device interrupt source endpoints to appropriate sink
    endpoints of interrupt controllers as described in device
    resources. @This is usually called from the device driver
    initialization function passing an array of source endpoints
    allocated in the private driver data for the device.

    The enable mask specifies which source endpoint must have its
    sense mode updated. When an interrupt is shared, the interrupt may
    have been enabled previously. When calling this function, the
    device driver must be ready to handle interrupts.

    If multiple trigger modes are specified in the device resource
    entry, a single mode is retained. The @ref device_irq_src_enable
    and @ref device_irq_src_disable function can be used provided that
    the interrupt is not shared.

    If the @ref DEV_IRQ_SENSE_NONE trigger mode is specified in the
    device resource entry, the sense mode can be updated later by
    calling the @ref device_irq_src_update function.

    In any case, the device driver can test the current mode
    of endpoint by calling the @ref device_irq_modes function.
*/
config_depend(CONFIG_DEVICE_IRQ)
error_t device_irq_source_link(struct device_s *dev, struct dev_irq_src_s *sources,
                               uint_fast8_t count, uint32_t enable_mask);

/** @This function returns the current trigger mode of the source
    endpoint and the sense modes supported by the linked sink
    endpoint. If the source-end point is not linked, 0 is returned in
    @tt *modes. Either pointers may be @tt NULL. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_modes(struct dev_irq_src_s *src,
                      enum dev_irq_sense_modes_e *cur,
                      enum dev_irq_sense_modes_e *modes);

/** @This unlink device interrupt endpoints. @This is usually
    called from the device driver cleanup function, passing an array
    of source endpoints allocated in the private driver data. */
config_depend(CONFIG_DEVICE_IRQ)
void device_irq_source_unlink(struct device_s *dev, struct dev_irq_src_s *sources, uint_fast8_t count);

/** @This is called from the interrupt controller code to forward an
    irq to all source endpoints connected to one of the controller
    sink endpoint. */
inline void device_irq_sink_process(struct dev_irq_sink_s *sink, dev_irq_id_t id)
{
#ifdef CONFIG_DEVICE_IRQ_SHARING
  if (sink->base.link_count > 1)
    {
      uint_fast8_t i;
      for (i = 0; i < sink->base.link_count; i++)
        {
          struct dev_irq_src_s *src = dev_irq_src_s_cast(sink->base.links.array[i]);
          //  printk("irqN %u %p\n", id, src->dev);
          src->process(src, id);
        }
      return;
    }
#endif

  struct dev_irq_src_s *src = dev_irq_src_s_cast(sink->base.links.single);
  src->process(src, id);
}

/** @This adds an IRQ binding to the device resources list.

    An interrupt controller device path must be specifed in a previous
    @ref DEV_RES_DEV_PARAM resource named @tt icu. The device path is
    relative to the device owning the resource entry.

    This entry specifies how an output irq wire (source) of the
    device is connected to an input (sink) of the interrupt
    controller. This is used to connect software irq endpoints
    of the two device drivers.

    A mask of irq trigger modes supported by the source is specified.
    When the linking occurs, a single mode will be selected based on
    what is supported by both the sink and the source endpoints. If
    zero is used here, no particular mode will be configured and the
    driver will be responsible for updating the sense mode of the sink.

    When the trigger mode is @ref DEV_IRQ_SENSE_ID_BUS, the logical
    irq number associated to the device on the irq bus must be given
    in @tt irq_id_ . Zero is used if the hardware irq link is a single
    wire.

    For interrupt controllers with multiple source endpoints, irq
    routing must be configured. A single output is used when the irq
    is triggered. Which source endpoint is actually selected may be
    choosen statically or dynamically by the implementation among
    allowed routes defined by the route mask parameter.

    In order to have the same irq forwarded to multiple controllers
    simultaneously, the @ref #CONFIG_DEVICE_IRQ_MULTI_SINK token must
    be defined and multiple irq resource entries must be provided.

    In order to connect multiple interrupts wires to the same
    controller input, the @ref #CONFIG_DEVICE_IRQ_SHARING token must
    be defined.

    @csee DEV_RES_IRQ
*/
config_depend_and2_inline(CONFIG_DEVICE_IRQ, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_irq(struct device_s *dev, uint_fast8_t src_id,
                           uint_fast8_t sink_id, enum dev_irq_sense_modes_e trig_mode,
                           dev_irq_id_t irq_id, dev_irq_route_t route_mask),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, DEV_RES_IRQ);
  if (err)
    return err;

  r->u.irq.src_id = src_id;
  r->u.irq.sink_id = sink_id;
  r->u.irq.trig_mode = trig_mode;
  r->u.irq.irq_id = irq_id;
  r->u.irq.route_mask = route_mask;

  return 0;
})

#ifdef CONFIG_DEVICE_IRQ
/** @This specifies an irq resource entry in a static device resources
    table declaration.  @see device_res_add_irq @csee DEV_RES_IRQ
    @see #DEV_DECLARE_STATIC. */
# define DEV_STATIC_RES_IRQ(src_id_, sink_id_, trig_mode_, irq_id_, route_mask_) \
  {                                                                     \
    .type = DEV_RES_IRQ,                                                \
    .u = { .irq = {                                                     \
        .src_id = (src_id_),                                            \
        .sink_id = (sink_id_),                                          \
        .trig_mode = (trig_mode_),                                      \
        .irq_id = (irq_id_),                                            \
        .route_mask = (route_mask_),                                    \
      } }                                                               \
  }

/** @This provides a @ref DEV_RES_DEV_PARAM resource entry which
    specifies the interrupts controller device relevant for some @cref
    DEV_RES_IRQ entries. */
#define DEV_STATIC_RES_DEV_ICU(path_) DEV_STATIC_RES_DEVCLASS_PARAM("icu", path_, DRIVER_CLASS_ICU)

#else
/** @hidden */
# define DEV_STATIC_RES_IRQ(src_id_, sink_id_, trig_mode_, irq_id_, route_mask_) \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }

/** @hidden */
#define DEV_STATIC_RES_DEV_ICU(path_)                                   \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif


#endif

