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
   @file
   @module {Core::Devices support library}
   @short Interrupt controller driver API
   @index {Interrupt controller} {Device classes}
   @csee DRIVER_CLASS_ICU
*/

#ifndef __DEVICE_ICU_H__
#define __DEVICE_ICU_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/irq.h>

struct device_s;
struct driver_s;
struct device_icu_s;
struct ipi_endpoint_s;
struct dev_irq_ep_s;

/** @see dev_icu_get_sink_t */
#define DEV_ICU_GET_SINK(n)	struct dev_irq_sink_s * (n) (struct device_icu_s *accessor, uint_fast8_t id)

/** @This returns the interrupt sink endpoint with given id. @This
    returns @tt NULL if no endpoint is available with the passed id. */
typedef DEV_ICU_GET_SINK(dev_icu_get_sink_t);


/** @see dev_icu_link_t */
#define DEV_ICU_LINK(n)	error_t (n) (struct device_icu_s *accessor, struct dev_irq_sink_s *sink, \
                                     struct dev_irq_src_s *src, dev_irq_route_t *route_mask, \
                                     struct dev_irq_src_s **bypass)
/** @This configure the hardware after the link between a sink and a
    source endpoints have changed. This is called from the @ref
    device_irq_source_link and @ref device_irq_source_unlink functions.


    @This is called with a @tt NULL pointer for the @tt route_mask
    parameter when a link between two endpoint is to be broken. When
    two endpoints have been linked, this parameter indicates which
    source endpoints of this interrupt controller can be used to
    route the interrupt.

    When @tt *bypass is @tt NULL, a link has changed between the @tt
    sink endpoint of this interrupt controller and the @tt src
    endpoint of a device.

    When @ref #CONFIG_DEVICE_IRQ_BYPASS is defined and @tt *bypass is
    not @tt NULL, a link has changed between the sink endpoint of an
    other interrupt controller in the chain to the device and the @tt
    src endpoint of a device. This happens when the next interrupt
    controller allows to be bypassed. In this case, it is possible to
    either ignore this request or keep a link to the @tt src endpoint
    so that the processing function of the @tt src endpoint can be
    called directly when an interrupt occurs.

    The return value of this function indicates if the interrupt link
    has been successfully configured; this is the case if the function
    return @tt -EAGAIN or 0. If the interrupt controller accept to be
    bypassed, the function have to update the content of the @tt
    {bypass}, @tt irq_id and @tt route_mask parameters, then return
    @tt {-EAGAIN}. In this case the @tt bypass parameter must be
    updated to point to the source endpoint which may be passed. In
    any case the interrupt controller must be able to handle
    interrupts by providing a suitable @ref dev_irq_src_process_t
    function for its source endpoints.
*/
typedef DEV_ICU_LINK(dev_icu_link_t);

/** @This does nothing and is successful. */
DEV_ICU_LINK(device_icu_dummy_link);

DRIVER_CLASS_TYPES(DRIVER_CLASS_ICU, icu,
                   dev_icu_get_sink_t *f_get_sink;
                   dev_icu_link_t *f_link;
                   );

/** @see driver_icu_s ICU device class methods */
#define DRIVER_ICU_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_icu_s){   \
    .class_ = DRIVER_CLASS_ICU,                                  \
    .f_get_sink = prefix ## _get_sink,                           \
    .f_link = prefix ## _link,                                   \
  })

/** @This is used to bind an initialized source endpoint to the sink
    endpoint of in a icu device. This can be used to setup an irq
    handler without a device driver.

    @param source Endpoint to bind, may be defined outside a device
    @param icu_name Name of ICU device to lookup in device tree
    @param sink_id ICU sink endpoint index
    @param irq_id the logical id of the irq to enable.
    @returns 0 on success or a negative error
*/
config_depend(CONFIG_DEVICE_IRQ)
error_t device_icu_irq_bind(struct dev_irq_src_s *source, const char *icu_name,
                            uint_fast16_t sink_id, uint8_t irq_id,
                            enum dev_irq_sense_modes_e trig_mode, dev_irq_route_t route_mask);

#endif

