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
	assert(irq < 32 && "Only 32 irq line are available on SAM7");

	struct icu_cm3_private_s	*pv = dev->drv_pv;
	struct icu_cm3_handler_s	*h = pv->table;

	h[irq].hndl = hndl;
	h[irq].data = data;

	return 0;
}

DEVICU_DELHNDL(icu_cm3_delhndl)
{
	assert(irq < 32 && "Only 32 irq line are available on SAM7");

	struct icu_cm3_private_s	*pv = dev->drv_pv;
	struct icu_cm3_handler_s	*h = pv->table;

	h[irq].hndl = NULL;
	h[irq].data = NULL;

	return 0;
}

DEV_IRQ(icu_cm3_handler)
{
#if !defined(CONFIG_CPU_ARM_CUSTOM_IRQ_HANDLER)
	CM3PS_NVIC registers = (void*)dev->addr[0];
	uint8_t irq = registers->NVIC_ITCTLR & 0x1f;
        irq -= 16;
	struct icu_cm3_private_s	*pv = dev->drv_pv;
	struct icu_cm3_handler_s	*h = pv->table[irq];


//	registers->AIC_ICCR = 1 << irq;

	if (h && h->hndl)
		h->hndl(h->data);
	else
		printk("SAM7 %d lost interrupt %i\n", cpu_id(), irq);

//	registers->AIC_EOICR = 0;

#endif
	return 0;
}

DEV_CLEANUP(icu_cm3_cleanup)
{
	struct tty_cm3_context_s	*pv = dev->drv_pv;
	AT91PS_AIC registers = (void*)dev->addr[0];

	registers->AIC_IDCR = (uint32_t)-1;

#if defined(CONFIG_DRIVER_ICU_ARM)
	DEV_ICU_UNBIND(dev->icudev, dev, dev->irq, icu_cm3_handler);
#endif

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
	AT91PS_AIC registers = (void*)dev->addr[0];

	dev->drv = &icu_cm3_drv;

	pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
	cm3_c_irq_dev = dev;

	if ( pv == NULL )
		goto memerr;
	
	pv->virq_refcount = 0;

	registers->AIC_IDCR = (uint32_t)-1;
	registers->AIC_ICCR = (uint32_t)-1;

	for (i=0; i < 8; i++) 
		AT91C_BASE_AIC->AIC_EOICR = 0; 

	for ( i=0; i<32; ++i ) {
		registers->AIC_SVR[i] = (uint32_t)(pv->table+32);
		pv->table[i].hndl = NULL;
		pv->table[i].data = NULL;
	}
	pv->table[32].hndl = NULL;
	pv->table[32].data = NULL;
	registers->AIC_SPU = (uint32_t)(pv->table+32);
//	registers->AIC_DCR = 1;

	pv->table[1].hndl = icu_cm3_handle_sysctrl;
	pv->table[1].data = dev;

	dev->drv_pv = pv;


	return 0;

  memerr:
	return -ENOMEM;
}

DEVICU_ENABLE(icu_cm3_enable)
{
	AT91PS_AIC registers = (void*)dev->addr[0];
	struct icu_cm3_private_s	*pv = cm3_c_irq_dev->drv_pv;

	registers->AIC_SMR[irq] = flags;

	if ( (1<<irq) & ICU_SAM7_SYSCTRL_VIRQS ) {
		if ( enable )
			pv->virq_refcount++;
		else
			pv->virq_refcount--;
		enable = !!(pv->virq_refcount);
		irq = 1;
	}

	if (enable) {
		registers->AIC_IECR = 1 << irq;
		registers->AIC_SVR[irq] = (uint32_t)(pv->table+irq);
	} else {
		registers->AIC_IDCR = 1 << irq;
		registers->AIC_SVR[irq] = (uint32_t)(pv->table+32);
	}

    return 0;
}
