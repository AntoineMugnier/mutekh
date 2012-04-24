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

struct device_s;
struct driver_s;
struct device_icu_s;
struct ipi_endpoint_s;
struct dev_irq_ep_s;

#define DEVICU_GET_SINK(n)	struct dev_irq_ep_s * (n) (struct device_icu_s *idev, uint_fast8_t icu_in_id)

/** @This gets interrupt sink end-point for given incoming interrupt
    line of the controller and enables associated interrupt line. */
typedef DEVICU_GET_SINK(devicu_get_sink_t);


#define DEVICU_DISABLE_SINK(n)	void (n) (struct device_icu_s *idev, struct dev_irq_ep_s *sink, uint_fast8_t id)

/** @This disables interrupt line associated with given sink end-point
    and logical irq id. */
typedef DEVICU_DISABLE_SINK(devicu_disable_sink_t);


// FIXME

#define DEVICU_SETUP_IPI_EP(n)	error_t (n) (struct device_icu_s *idev, \
					     struct ipi_endpoint_s *endpoint, \
					     struct device_s *cpu)
/** @This setups an ipi end-point for sending intrruptions to the
    specified processor.  @This may delegate setup of the ipi
    end-point to a linked interrupt controller, the initial call to
    this function is performed directly on the processor device which
    will receive the ipi. */
typedef DEVICU_SETUP_IPI_EP(devicu_setup_ipi_ep_t);


/** ICU device class methodes */

DRIVER_CLASS_TYPES(icu, 
                   devicu_get_sink_t	*f_get_sink;
                   devicu_disable_sink_t *f_disable_sink;
#ifdef CONFIG_HEXO_IPI
                   devicu_setup_ipi_ep_t	*f_setup_ipi_ep;
#endif
                   );

#endif

