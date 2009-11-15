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

DEVICU_ENABLE(xicu_soclib_enable)
{
    struct xicu_soclib_private_s *pv = dev->drv_pv;

	if ( irq & XICU_IRQ_IPI ) {
		cpu_mem_write_32(
			XICU_REG_ADDR(dev->addr[0],
						  enable ? XICU_MSK_HWI_ENABLE : XICU_MSK_HWI_DISABLE,
						  pv->output_line_no),
			endian_le32(1 << (irq & 0x1f)));
	} else {
		cpu_mem_write_32(
			XICU_REG_ADDR(dev->addr[0],
						  enable ? XICU_MSK_HWI_ENABLE : XICU_MSK_HWI_DISABLE,
						  pv->output_line_no),
			endian_le32(1 << (irq & 0x1f)));
	}
}

DEVICU_SETHNDL(xicu_soclib_sethndl)
{
    struct xicu_soclib_private_s *pv = dev->drv_pv;
    struct xicu_soclib_handler_s *h;

	if ( irq & XICU_IRQ_IPI )
		h = pv->hwi_handlers + (irq & 0x1f);
	else
		h = pv->ipi_handlers + (irq & 0x1f);

    h->hndl = hndl;
    h->data = data;

    return 0;
}

DEVICU_DELHNDL(xicu_soclib_delhndl)
{
    struct xicu_soclib_private_s *pv = dev->drv_pv;
    struct xicu_soclib_handler_s *h;

	if ( irq & XICU_IRQ_IPI )
		h = pv->hwi_handlers + (irq & 0x1f);
	else
		h = pv->ipi_handlers + (irq & 0x1f);

    h->hndl = NULL;
    h->data = NULL;

    return 0;
}

DEV_IRQ(xicu_soclib_handler)
{
    struct xicu_soclib_handler_s *h;
    struct xicu_soclib_private_s *pv = dev->drv_pv;

    uint32_t prio = endian_le32(
		cpu_mem_read_32(XICU_REG_ADDR(dev->addr[0],
									  XICU_PRIO,
									  pv->output_line_no)));

	if ( XICU_PRIO_HAS_WTI(prio) ) {
//		printk("xicu wti: %x\n", XICU_PRIO_WTI(prio));
	
		h = pv->ipi_handlers + XICU_PRIO_HWI(prio);

		cpu_mem_read_32(XICU_REG_ADDR(dev->addr[0],
									  XICU_WTI_REG,
									  XICU_PRIO_HWI(prio)));
    
		/* call ipi handler */
		return h->hndl(h->data);
	}

	if ( XICU_PRIO_HAS_HWI(prio) ) {
//		printk("xicu hwi: %x\n", XICU_PRIO_HWI(prio));

		h = pv->hwi_handlers + XICU_PRIO_HWI(prio);
    
		/* call interrupt handler */
		return h->hndl(h->data);
	}

	if ( XICU_PRIO_HAS_PTI(prio) ) {
//		printk("xicu pti: %x\n", XICU_PRIO_PTI(prio));

		return xicu_timer_soclib_irq(&pv->timer, XICU_PRIO_PTI(prio));
	}

//	printk("xicu nothing\n");

	return 0;
}

DEVICU_SENDIPI(xicu_soclib_sendipi)
{
//	printk("xicu send ipi to %x\n", cpu_icu_identifier);

	cpu_mem_write_32(
		XICU_REG_ADDR(dev->addr[0],
					  XICU_WTI_REG,
					  ((uint32_t)cpu_icu_identifier) & 0x1f),
		endian_le32((uint32_t)cpu_icu_identifier));
	return 0;
}

DEVICU_SETUPIPI(xicu_soclib_setupipi)
{
//	struct xicu_soclib_private_s	*pv = dev->drv_pv;

	xicu_soclib_sethndl(dev, XICU_IRQ_IPI | ipi_no,
						(dev_irq_t*)ipi_process_rq, NULL);

	return (void*)(uintptr_t)ipi_no;
}

DEV_CLEANUP(xicu_soclib_cleanup)
{
  struct xicu_soclib_private_s	*pv = dev->drv_pv;

  device_unregister(&pv->timer);
  mem_free(pv);
}

static const struct driver_param_binder_s xicu_param_binder[] =
{
	PARAM_BIND(struct soclib_xicu_param_s, output_line_no, PARAM_DATATYPE_INT),
	{ 0 }
};

static const struct devenum_ident_s	xicu_soclib_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("soclib:xicu", sizeof(struct soclib_xicu_param_s), xicu_param_binder),
	{ 0 }
};

const struct driver_s	xicu_soclib_drv =
{
  .class		= device_class_icu,
  .id_table		= xicu_soclib_ids,
  .f_init		= xicu_soclib_init,
  .f_cleanup		= xicu_soclib_cleanup,
  .f.icu = {
    .f_enable		= xicu_soclib_enable,
    .f_sethndl		= xicu_soclib_sethndl,
    .f_delhndl		= xicu_soclib_delhndl,
    .f_sendipi		= xicu_soclib_sendipi,
    .f_setupipi		= xicu_soclib_setupipi,
  }
};

REGISTER_DRIVER(xicu_soclib_drv);

DEV_INIT(xicu_soclib_init)
{
  struct xicu_soclib_private_s	*pv;
  device_mem_map( dev , 1 );
  dev->drv = &xicu_soclib_drv;

  if (!(pv = mem_alloc(sizeof (*pv), (mem_scope_sys))))
    return -ENOMEM;

  dev->drv_pv = pv;

  device_init(&pv->timer);
  pv->output_line_no = ((struct soclib_xicu_param_s*)params)->output_line_no;
  pv->timer.addr[0] = dev->addr[0];
  pv->timer.irq = 0;
  pv->timer.icudev = dev;
  xicu_timer_soclib_init(&pv->timer, params);

  /* register as a child device */
  device_register(&pv->timer, dev, NULL);

  DEV_ICU_BIND(dev->icudev, dev, dev->irq, xicu_soclib_handler);

  return 0;
}

