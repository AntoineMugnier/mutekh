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

    Copyright (c) Eric Guthmuller, 2010

*/

#include "icu-cm3.h"

#include "icu-cm3-private.h"

#include <string.h>

#include <hexo/types.h>
#include <device/device.h>
#include <device/driver.h>
#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>

#include "cpu/cm3/cm3.h"


DEVICU_SETHNDL(icu_cm3_sethndl)
{
	struct icu_cm3_private_s	*pv = dev->drv_pv;
	struct icu_cm3_handler_s	*h = pv->table;

	assert(irq < pv->intlinesnum && "Only intlinesnum irq line are available on CM3");
	h[irq].hndl = hndl;
	h[irq].data = data;

	return 0;
}

DEVICU_DELHNDL(icu_cm3_delhndl)
{
	struct icu_cm3_private_s	*pv = dev->drv_pv;
	struct icu_cm3_handler_s	*h = pv->table;

	assert(irq < pv->intlinesnum && "Only intlinesnum irq line are available on CM3");

	h[irq].hndl = NULL;
	h[irq].data = NULL;

	return 0;
}

DEV_IRQ(icu_cm3_handler)
{
	uint16_t irq = CM3_BASE_NVIC->NVIC_ITCTLR & 0x1ff;
        irq -= 16;
	struct icu_cm3_private_s	*pv = dev->drv_pv;
	struct icu_cm3_handler_s	*h = pv->table;

	if (h[irq].hndl)
		h[irq].hndl(h[irq].data);
	else
		printk("CM3 %d lost interrupt %i\n", cpu_id(), irq);
	return 0;
}

DEV_CLEANUP(icu_cm3_cleanup)
{
	struct icu_cm3_private_s	*pv = dev->drv_pv;
	CM3PS_NVIC registers = CM3_BASE_NVIC;
        size_t i;
	for ( i=0; i < (pv->intlinesnum / 32); i++) 
		CM3_BASE_NVIC->NVIC_ITENR[i] = 0; 

        mem_free(pv->table);

	mem_free(pv);
}

#ifdef CONFIG_DRIVER_ENUM_FDT
static const struct devenum_ident_s	icu_cm3_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("cm3:icu", 0, 0),
	{ 0 }
};
#endif

const struct driver_s	icu_cm3_drv =
{
    .class      = device_class_icu,
#ifdef CONFIG_DRIVER_ENUM_FDT
    .id_table   = icu_cm3_ids,
#endif
    .f_init     = icu_cm3_init,
    .f_cleanup  = icu_cm3_cleanup,
    .f_irq      = icu_cm3_handler,
    .f.icu = {
        .f_enable       = icu_cm3_enable,
        .f_sethndl      = icu_cm3_sethndl,
        .f_delhndl      = icu_cm3_delhndl,
    }
};

#ifdef CONFIG_DRIVER_ENUM_FDT
REGISTER_DRIVER(icu_cm3_drv);
#endif


DEV_INIT(icu_cm3_init)
{
	struct icu_cm3_private_s	*pv;
	uint_fast8_t i;

	dev->drv = &icu_cm3_drv;

	pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
        //	cm3_c_irq_dev = dev;
	if ( pv == NULL )
	  goto memerr;

        /* Set the irq line number */
        pv->intlinesnum = CM3_BASE_NVIC->NVIC_ITCTL_TYPER+1;
        pv->intlinesnum *=32;
        if(pv->intlinesnum > ICU_CM3_MAX_VECTOR)
          pv->intlinesnum = ICU_CM3_MAX_VECTOR;

        /* Allocate the vector interrupt table */
        pv->table = mem_alloc(sizeof(*pv->table)*(pv->intlinesnum), (mem_scope_sys));
        if ( pv->table == NULL )
          goto memerr;

        /* Desactivate all interrupt sources */
	for ( i=0; i < (pv->intlinesnum / 32); i++) 
		CM3_BASE_NVIC->NVIC_ITENR[i] = 0; 

        /* Set all priorities to 0 */
	for ( i=0; i < (pv->intlinesnum / 4); i++ ) {
		CM3_BASE_NVIC->NVIC_ITPRIR[i] = (uint32_t)(pv->table+32);
	}

        /* Set all handlers and their datas pointers to NULL */
        for ( i=0; i< pv->intlinesnum ; i++ ) {
		pv->table[i].hndl = NULL;
		pv->table[i].data = NULL;
        }
	
        /* Register private datas */
	dev->drv_pv = pv;

	assert(dev->icudev);
	DEV_ICU_BIND(dev->icudev, dev, dev->irq, icu_cm3_handler);

	return 0;

  memerr:
	return -ENOMEM;
}

DEVICU_ENABLE(icu_cm3_enable)
{
	struct icu_cm3_private_s	*pv = dev->drv_pv;

	assert(irq < pv->intlinesnum && "Only intlinesnum irq line are available on CM3");

        uint32_t i = irq / 32;

	if (enable) {
          CM3_BASE_NVIC->NVIC_ITENR[i] =CM3_BASE_NVIC->NVIC_ITENR[i] | (1 << (irq & 31)); 
	} else {
          CM3_BASE_NVIC->NVIC_ITENR[i] =CM3_BASE_NVIC->NVIC_ITENR[i] & ~(1 << (irq & 31)); 
	}

    return 0;
}
