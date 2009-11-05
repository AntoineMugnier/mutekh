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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include "icu-mips.h"

#include "icu-mips-private.h"

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <device/device.h>
#include <device/driver.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>

DEVICU_ENABLE(icu_mips_enable)
{
	reg_t status = cpu_mips_mfc0(12, 0);
	reg_t mask = 1 << (irq + 10);
	if (enable)
		status |= mask;
	else
		status &= ~mask;
	cpu_mips_mtc0(12, 0, status);
}

DEVICU_SETHNDL(icu_mips_sethndl)
{
	struct icu_mips_private_s	*pv = dev->drv_pv;
	struct icu_mips_handler_s	*h = pv->table + irq;

	h->hndl = hndl;
	h->data = data;

	return 0;
}

DEVICU_DELHNDL(icu_mips_delhndl)
{
	struct icu_mips_private_s	*pv = dev->drv_pv;
	struct icu_mips_handler_s	*h = pv->table + irq;

	reg_t status = cpu_mips_mfc0(12, 0);
	reg_t mask = 1 << (irq + 10);
	assert( (mask & status) == 0 && "You should have disabled this interrupt already" );
	(void)(mask&status);

	h->hndl = NULL;
	h->data = NULL;

	return 0;
}

static CPU_INTERRUPT_HANDLER(icu_mips_cpu_handler)
{
	struct device_s *dev = CPU_LOCAL_ADDR(cpu_icu_dev);
	struct icu_mips_private_s	*pv = dev->drv_pv;
	struct icu_mips_handler_s	*h;
	uint32_t irq_no = __builtin_ctz(irq);

	if ( irq_no >= ICU_MIPS_MAX_VECTOR ) {
		printk("Mips %d got spurious interrupt %i\n", cpu_id(), irq_no);
		return;
	}

	h = pv->table + irq_no;
	
	if (h->hndl)
		h->hndl(h->data);
	else
		printk("Mips %d lost interrupt %i\n", cpu_id(), irq_no);
}

DEV_CLEANUP(icu_mips_cleanup)
{
	reg_t status = cpu_mips_mfc0(12, 0);
	reg_t mask = 0x3f << 10;
	status &= ~mask;
	cpu_mips_mtc0(12, 0, status);
}

const struct driver_s	icu_mips_drv =
{
	.class		= device_class_icu,
	.f_init		= icu_mips_init,
	.f_cleanup		= icu_mips_cleanup,
	.f.icu = {
		.f_enable		= icu_mips_enable,
		.f_sethndl		= icu_mips_sethndl,
		.f_delhndl		= icu_mips_delhndl,
	}
};

DEV_INIT(icu_mips_init)
{
	struct icu_mips_private_s	*pv;

	dev->drv = &icu_mips_drv;

	/* FIXME allocation scope ? */
	pv = mem_alloc(sizeof (*pv), MEM_SCOPE_SYS);

	if ( pv == NULL )
		goto memerr;

	dev->drv_pv = pv;

	cpu_interrupt_sethandler(icu_mips_cpu_handler);

	return 0;

  memerr:
	return -ENOMEM;
}
