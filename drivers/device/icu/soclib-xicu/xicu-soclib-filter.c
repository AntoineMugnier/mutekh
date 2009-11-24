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

#include <mutek/mem_alloc.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <hexo/ipi.h>
#include <device/driver.h>
#include <device/device.h>

#define PARENT(dev) (((struct xicu_filter_private_s *)(dev->drv_pv))->parent)

DEVICU_ENABLE(xicu_filter_enable)
{
    struct xicu_filter_private_s *pv = dev->drv_pv;

	if ( irq & XICU_IRQ_IPI )
		xicu_root_enable_ipi(PARENT(dev), irq&0x1f, pv->output, enable);
	else
		xicu_root_enable_hwi(PARENT(dev), irq&0x1f, pv->output, enable);
}

DEVICU_SETHNDL(xicu_filter_sethndl)
{
	if ( irq & XICU_IRQ_IPI )
		return xicu_root_set_ipi_handler(PARENT(dev), irq&0x1f, hndl, data);
	else
		return xicu_root_set_hwi_handler(PARENT(dev), irq&0x1f, hndl, data);
}

DEVICU_DELHNDL(xicu_filter_delhndl)
{
	if ( irq & XICU_IRQ_IPI )
		return xicu_root_set_ipi_handler(PARENT(dev), irq&0x1f, NULL, NULL);
	else
		return xicu_root_set_hwi_handler(PARENT(dev), irq&0x1f, NULL, NULL);
}

DEV_IRQ(xicu_filter_handler)
{
//    struct xicu_soclib_handler_s *h;
    struct xicu_filter_private_s *pv = dev->drv_pv;

    uint32_t prio = endian_le32(cpu_mem_read_32(XICU_REG_ADDR(
			dev->addr[0], XICU_PRIO, pv->output)));

	if ( XICU_PRIO_HAS_WTI(prio) )
		return xicu_root_handle_ipi(PARENT(dev), XICU_PRIO_WTI(prio));

	if ( XICU_PRIO_HAS_HWI(prio) )
		return xicu_root_handle_hwi(PARENT(dev), XICU_PRIO_HWI(prio));

	if ( XICU_PRIO_HAS_PTI(prio) )
		return xicu_root_handle_timer(PARENT(dev), XICU_PRIO_PTI(prio));

	return 0;
}

DEVICU_SETUPIPI(xicu_filter_setupipi)
{
	struct xicu_filter_private_s *pv = dev->drv_pv;

	xicu_root_set_ipi_handler(PARENT(dev), ipi_no&0x1f, (dev_irq_t*)ipi_process_rq, NULL);
	xicu_root_enable_ipi(PARENT(dev), ipi_no&0x1f, pv->output, 1);

	return (void*)(uintptr_t)ipi_no;
}

DEVICU_SENDIPI(xicu_filter_sendipi)
{
/* 	printk("xicu send ipi to %x\n", cpu_icu_identifier); */

	cpu_mem_write_32(XICU_REG_ADDR(dev->addr[0],
					  XICU_WTI_REG,
					  ((uint32_t)cpu_icu_identifier) & 0x1f),
		endian_le32((uint32_t)cpu_icu_identifier));
	return 0;
}

static const struct driver_param_binder_s xicufilter_param_binder[] =
{
	PARAM_BIND(struct xicu_filter_param_s, parent, PARAM_DATATYPE_DEVICE_PTR),
	PARAM_BIND(struct xicu_filter_param_s, output_line, PARAM_DATATYPE_INT),
	{ 0 }
};

static const struct devenum_ident_s	xicu_filter_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("soclib:xicu:filter", sizeof(struct xicu_filter_param_s), xicufilter_param_binder),
	{ 0 }
};

const struct driver_s	xicu_filter_drv =
{
	.class           = device_class_icu,
	.id_table        = xicu_filter_ids,
	.f_irq           = xicu_filter_handler,
	.f_init          = xicu_filter_init,
	.f_cleanup       = xicu_filter_cleanup,
	.f.icu = {
		.f_enable    = xicu_filter_enable,
		.f_sethndl   = xicu_filter_sethndl,
		.f_delhndl   = xicu_filter_delhndl,
		.f_sendipi   = xicu_filter_sendipi,
		.f_setupipi  = xicu_filter_setupipi,
	}
};

REGISTER_DRIVER(xicu_filter_drv);

DEV_CLEANUP(xicu_filter_cleanup)
{
	struct xicu_filter_private_s	*pv = dev->drv_pv;

	mem_free(pv);
}

DEV_INIT(xicu_filter_init)
{
	struct xicu_filter_param_s *param = params;

	device_mem_map( dev , 1 );
	dev->drv = &xicu_filter_drv;

/* 	printk("Creating an XICU filter device. Parent %p [%p] output %d @%p; icu %p/%d\n", */
/* 		   param->parent, param->parent->drv, */
/* 		   param->output_line, param->parent->addr[0], */
/* 		   dev->icudev, dev->irq); */

	struct xicu_filter_private_s *pv = mem_alloc(sizeof(*pv), mem_scope_sys);

	if ( !pv )
		return -ENOMEM;

	dev->drv_pv = pv;

	pv->parent = param->parent;

	dev->addr[0] = PARENT(dev)->addr[0];

	pv->output = param->output_line;

	DEV_ICU_BIND(dev->icudev, dev, dev->irq, xicu_filter_handler);

	return 0;
}
