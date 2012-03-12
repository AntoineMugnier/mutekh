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

/** ICU device class enable() function template */
#define DEVICU_ENABLE(n)	error_t (n) (struct device_s *dev, uint_fast8_t irq, \
                                             bool_t enable, reg_t flags)

/** ICU device class enable() function type. Enable or Disable
    interrupt line.

    * @param dev pointer to device descriptor
    * @param irq icu interrupt line number
    * @param enable 0 disable interrupt
    * @param flags icu specific interrupt handling flags, default to 0
    */
typedef DEVICU_ENABLE(devicu_enable_t);



/** ICU device class sethndl() function template */
#define DEVICU_SETHNDL(n)	error_t (n) (struct device_s *dev, uint_fast8_t irq, dev_irq_t *hndl, void *data)
/** ICU device class sethndl() function type. Setup a new interrupt
    handler and associated private data.

    * @param dev pointer to device descriptor
    * @param irq icu interrupt line number
    * @param hndl pointer to handler function
    * @param data pointer to associated private data if any
    * @return negative error code
    */
typedef DEVICU_SETHNDL(devicu_sethndl_t);



/** ICU device class delhndl() function template */
#define DEVICU_DELHNDL(n)	error_t (n) (struct device_s *dev, uint_fast8_t irq, dev_irq_t *hndl)
/** ICU device class delhndl() function type. Remove interrupt
    handler. Several handlers may be registered when interrupt sharing
    is used.

    * @param dev pointer to device descriptor
    * @param irq icu interrupt line number
    * @param hndl pointer to handler function
    * @return negative error code
    */
typedef DEVICU_DELHNDL(devicu_delhndl_t);


struct ipi_endpoint_s;

/** ICU device class sendipi() function template */
#define DEVICU_SENDIPI(n)	error_t (n) (struct ipi_endpoint_s *endpoint)
/** ICU device class sendipi() function type. send an ipi to specified processor. */
typedef DEVICU_SENDIPI(devicu_sendipi_t);



/** ICU device class setupipi() function template */
#define DEVICU_SETUP_IPI_EP(n)	error_t (n) (struct device_s *dev, \
					     struct ipi_endpoint_s *endpoint, \
					     uint_fast8_t ipi_no)
/** ICU device class setupipi() function type. setup an ipi endpoint. */
typedef DEVICU_SETUP_IPI_EP(devicu_setup_ipi_ep_t);



/** ICU device class methodes */

struct driver_icu_s
{
  enum device_class_e cl;
  devicu_enable_t	*f_enable;
  devicu_sethndl_t	*f_sethndl;
  devicu_delhndl_t	*f_delhndl;
  devicu_sendipi_t	*f_sendipi;
  devicu_setup_ipi_ep_t	*f_setup_ipi_ep;
};

#endif

