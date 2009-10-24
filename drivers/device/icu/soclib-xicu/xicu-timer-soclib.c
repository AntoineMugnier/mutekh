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

    Copyright (c) UPMC, Lip6
         Alexandre Becoulet <alexandre.becoulet@lip6.fr>, 2006-2009
         Nicolas Pouillon <nipo@ssji.net>, 2009

*/


#include "xicu-soclib.h"
#include "xicu-soclib-private.h"

#include <hexo/alloc.h>
#include <device/driver.h>
#include <hexo/iospace.h>

#include <mutek/printk.h>

/*
 * timer device callback setup
 */

DEVTIMER_SETCALLBACK(xicu_timer_soclib_setcallback)
{
	struct xicu_soclib_timer_private_s *pv = dev->drv_pv;
	struct xicu_soclib_timer_handler_s *handler = &pv->pti_handlers[id];

	if (callback) {
		handler->hndl = callback;
		handler->data = private;

		cpu_mem_write_32(
			XICU_REG_ADDR(dev->addr[0],
						  XICU_MSK_PTI_ENABLE,
						  pv->output_line_no),
			endian_le32(1<<id));
	} else {
		cpu_mem_write_32(
			XICU_REG_ADDR(dev->addr[0],
						  XICU_MSK_PTI_DISABLE,
						  pv->output_line_no),
			endian_le32(1<<id));
	}
	return 0;
}

/*
 * timer device period setup
 */

DEVTIMER_SETPERIOD(xicu_timer_soclib_setperiod)
{
	cpu_mem_write_32(
		XICU_REG_ADDR(dev->addr[0],
					  XICU_PTI_PER,
					  id),
		endian_le32(period));
	return 0;
}

/*
 * timer device value change
 */

DEVTIMER_SETVALUE(xicu_timer_soclib_setvalue)
{
	cpu_mem_write_32(
		XICU_REG_ADDR(dev->addr[0],
					  XICU_PTI_VAL,
					  id),
		endian_le32(value));

	return 0;
}

/*
 * timer device value getter
 */

DEVTIMER_GETVALUE(xicu_timer_soclib_getvalue)
{
	return endian_le32(
		cpu_mem_read_32(
			XICU_REG_ADDR(dev->addr[0],
						  XICU_PTI_VAL,
						  id)));
}

/*
 * device irq, internally chained from ICU code
 */

bool_t xicu_timer_soclib_irq(
	struct device_s *timer_dev, int_fast8_t id)
{
	struct xicu_soclib_timer_private_s *pv = timer_dev->drv_pv;
	struct xicu_soclib_timer_handler_s *handler = &pv->pti_handlers[id];

	cpu_mem_read_32(XICU_REG_ADDR(timer_dev->addr[0],
								  XICU_PTI_ACK,
								  id));

	if (handler->hndl)
	    handler->hndl(handler->data);
	else
		printk("Xicu lost interrupt");
	return 0;
}

/* 
 * device close operation
 */

DEV_CLEANUP(xicu_timer_soclib_cleanup)
{
	struct xicu_soclib_timer_private_s *pv = dev->drv_pv;

	mem_free(pv);
}

const struct driver_s	xicu_timer_soclib_drv =
{
  .class		= device_class_timer,
  .f_init		= xicu_timer_soclib_init,
  .f_cleanup		= xicu_timer_soclib_cleanup,
//  .f_irq		= xicu_timer_soclib_irq,
  .f.timer = {
    .f_setcallback	= xicu_timer_soclib_setcallback,
    .f_setperiod	= xicu_timer_soclib_setperiod,
    .f_setvalue		= xicu_timer_soclib_setvalue,
    .f_getvalue		= xicu_timer_soclib_getvalue,
  }
};

/* 
 * device open operation
 */

DEV_INIT(xicu_timer_soclib_init)
{
	struct xicu_soclib_timer_private_s *pv;

	dev->drv = &xicu_timer_soclib_drv;

	/* allocate private driver data */
	pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

	pv->output_line_no = ((struct soclib_xicu_param_s*)params)->output_line_no;

	if (!pv)
		return -1;

	memset(pv, 0, sizeof(*pv));

	dev->drv_pv = pv;

	return 0;
}
