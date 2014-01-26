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
 * @short Interrupt controller driver API
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

/** Interrupt controller device class @ref devicu_get_endpoint_t function template. */
#define DEVICU_GET_ENDPOINT(n)	void * (n) (struct device_icu_s *idev, enum dev_irq_ep_type_e type, uint_fast8_t id)

/** @This gets interrupt end-point of given type with given id in
    range [0-N]. @This returns @tt NULL if out of range.

    If the hardware can be configured to handle various @ref
    dev_irq_sense_modes_e mode but the end-point is already linked and
    enabled, only the flags for the mode currently in use are set in
    the end-point. This function must take care of resetting the
    capabilities to all supported sense modes if the link count of the
    end-point is 0.

    @This is mandatory.
 */
typedef DEVICU_GET_ENDPOINT(devicu_get_endpoint_t);


/** Interrupt controller device class @ref devicu_enable_irq_t function template. */
#define DEVICU_ENABLE_IRQ(n)	bool_t (n) (struct device_icu_s *idev, struct dev_irq_ep_s *sink, \
                                            uint_fast16_t irq_id, struct dev_irq_ep_s *src, struct dev_irq_ep_s *dev_ep)

/** @This enables interrupt associated with given sink end-point and
    logical irq id. @This function must first call @ref
    device_icu_irq_enable in order to recursively enable interrupt
    along the path to processor(s).

    The return value of this function indicates if the interrupt has
    been successfully enabled and can be relayed through the sink
    end-point. Information attached to the device may be used to
    decide if a given interrupt path is suitable; this includes
    information about processor affinity for device interrupt
    handling.

    @param idev a pointer to the interrupt controller device.
    @param sink a pointer to the local sink end-point associated with irq to enable.
    @param irq_id the logical id of the irq to enable.
    @param src a pointer to a remote source end-point which must be processed on interrupt, may be NULL.
    @param dev_ep a pointer to the source end-point of regular device associated with interrupt.

    The pointer to the remote source end-point may be provided so that
    the interrupt controller may choose to store a shortcut pointer to
    this end-point in order to bypass the regular interrupt
    propagation call graph. When recursively calling @ref
    device_icu_irq_enable, the provided source end-point may be the
    local source end-point used to relay the interrupt or the @tt src
    pointer passed by caller. If the interrupt propagation doesn't
    need processing other that @tt irq_id based de-multiplexing (no
    acknowledgment or hardware register access) and de-multiplexing is
    not performed in this controller, forwarding the @tt src argument
    allows bypassing one or more end-point connections traversal when
    the interrupt is raised.

    This function must check capabilities of @tt src and @tt sink
    end-points and return an error if end-points sense modes are not
    compatibles (bitwise and == 0). If multiples modes are possibles,
    the interrupt controller must chose one and update both end-points
    so that only the flags of the selected mode remain set. If the
    owner of the source end-point advertises multiple capabilities
    before enabling interrupts on its source end-points, it must check
    the selected configuration on return of the @ref device_irq_source_link
    function and configure the hardware accordingly.

    @This is mandatory.
*/
typedef DEVICU_ENABLE_IRQ(devicu_enable_irq_t);


/** Interrupt controller device class @ref devicu_disable_irq_t function template. */
#define DEVICU_DISABLE_IRQ(n)	void (n) (struct device_icu_s *idev, struct dev_irq_ep_s *sink)

/** @This disables interrupt associated with given sink
    end-point. This function is called when a sink end-point links
    count become 0. This function may be called even if interrupts are
    already disabled. 

    @This is optional.
*/
typedef DEVICU_DISABLE_IRQ(devicu_disable_irq_t);

/** ICU device class methodes */

DRIVER_CLASS_TYPES(icu, 
                   devicu_get_endpoint_t *f_get_endpoint;
                   devicu_enable_irq_t *f_enable_irq;
                   devicu_disable_irq_t *f_disable_irq;
                   );

/** @This is used to propagate interrupt enabling along the irq
    routing path between device and processor(s). @see
    devicu_enable_irq_t @see device_irq_source_link. */
config_depend(CONFIG_DEVICE_IRQ)
bool_t device_icu_irq_enable(struct dev_irq_ep_s *local_src, uint_fast16_t target_irq_id,
                             struct dev_irq_ep_s *target_src, struct dev_irq_ep_s *dev_src);

#endif

