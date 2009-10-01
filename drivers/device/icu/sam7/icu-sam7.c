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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2009

*/

#include "icu-sam7.h"

#include "icu-sam7-private.h"

#include <string.h>

#include <hexo/types.h>
#include <hexo/device.h>
#include <device/driver.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>

#include "arch/sam7/at91sam7x256.h"

DEVICU_SET_FLAGS(icu_sam7_set_flags)
{
	AT91PS_AIC registers = (void*)dev->addr[0];
//	struct icu_sam7_private_s	*pv = dev->drv_pv;

	assert(irq < 32 && "Only 32 irq line are available on SAM7");

	registers->AIC_SMR[irq] = flags;
}

DEVICU_SETHNDL(icu_sam7_sethndl)
{
	assert(irq < 32 && "Only 32 irq line are available on SAM7");

	struct icu_sam7_private_s	*pv = dev->drv_pv;
	struct icu_sam7_handler_s	*h = pv->table;

	h[irq].hndl = hndl;
	h[irq].data = data;

	return 0;
}

DEVICU_DELHNDL(icu_sam7_delhndl)
{
	assert(irq < 32 && "Only 32 irq line are available on SAM7");

	struct icu_sam7_private_s	*pv = dev->drv_pv;
	struct icu_sam7_handler_s	*h = pv->table;

	h[irq].hndl = NULL;
	h[irq].data = NULL;

	return 0;
}

DEV_IRQ(icu_sam7_handler)
{
	AT91PS_AIC registers = (void*)dev->addr[0];
//	struct icu_sam7_private_s	*pv = dev->drv_pv;
//	struct icu_sam7_handler_s	*h = pv->table[irq];
	struct icu_sam7_handler_s	*h = (void*)registers->AIC_IVR;

	assert(!"Not wanted");

	uint8_t irq = registers->AIC_ISR & 0x1f;

//	registers->AIC_ICCR = 1 << irq;

	if (h && h->hndl)
		h->hndl(h->data);
	else
		printk("SAM7 %d lost interrupt %i\n", cpu_id(), irq);

//	registers->AIC_EOICR = 0;

	return 0;
}

DEV_CLEANUP(icu_sam7_cleanup)
{
	struct tty_sam7_context_s	*pv = dev->drv_pv;
	AT91PS_AIC registers = (void*)dev->addr[0];

	registers->AIC_IDCR = (uint32_t)-1;

#if defined(CONFIG_DRIVER_ICU_ARM)
	DEV_ICU_UNBIND(dev->icudev, dev, dev->irq);
#endif

	mem_free(pv);
}

const struct driver_s	icu_sam7_drv =
{
    .class      = device_class_icu,
    .f_init     = icu_sam7_init,
    .f_cleanup  = icu_sam7_cleanup,
    .f_irq      = icu_sam7_handler,
    .f.icu = {
        .f_enable       = icu_sam7_enable,
        .f_set_flags    = icu_sam7_set_flags,
        .f_sethndl      = icu_sam7_sethndl,
        .f_delhndl      = icu_sam7_delhndl,
    }
};

#if 0
static
__attribute__ ((interrupt ("IRQ")))
void sam7_spurious_irq_handler()
{
	uint32_t irq;
	asm("ldr %0, [sp, #6*4]": "=r" (irq));
	kputs("SAM7 lost spurious interrupt\n");
	cpu_mem_write_32(0xfffff130, 1);
}
#endif

struct device_s	*sam7_c_irq_dev;

__attribute__ ((interrupt ("IRQ")))
void arm_c_irq_handler()
{
	AT91PS_AIC registers = (void*)sam7_c_irq_dev->addr[0];
	struct icu_sam7_handler_s	*h = 
		(struct icu_sam7_handler_s*) registers->AIC_IVR;

	if (h && h->hndl)
		h->hndl(h->data);
	registers->AIC_EOICR = 1;
}

__attribute__ ((interrupt ("FIQ")))
void arm_c_fiq_handler()
{
	AT91PS_AIC registers = (void*)sam7_c_irq_dev->addr[0];
	struct icu_sam7_handler_s	*h = 
		(struct icu_sam7_handler_s*) registers->AIC_IVR;

	if (h && h->hndl)
		h->hndl(h->data);
	registers->AIC_EOICR = 1;
}

DEV_INIT(icu_sam7_init)
{
	struct icu_sam7_private_s	*pv;
	uint_fast8_t i;
	AT91PS_AIC registers = (void*)dev->addr[0];

	dev->drv = &icu_sam7_drv;

	pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);
	sam7_c_irq_dev = dev;

	if ( pv == NULL )
		goto memerr;

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

	dev->drv_pv = pv;

#if defined(CONFIG_DRIVER_ICU_ARM)
	assert(dev->icudev);
	DEV_ICU_BIND(dev->icudev, dev, dev->irq, icu_sam7_handler);
#endif

	return 0;

  memerr:
	return -ENOMEM;
}

DEVICU_ENABLE(icu_sam7_enable)
{
	AT91PS_AIC registers = (void*)dev->addr[0];
	struct icu_sam7_private_s	*pv = sam7_c_irq_dev->drv_pv;

	if (enable) {
		registers->AIC_IECR = 1 << irq;
		registers->AIC_SVR[irq] = (uint32_t)(pv->table+irq);
	} else {
		registers->AIC_IDCR = 1 << irq;
		registers->AIC_SVR[irq] = (uint32_t)(pv->table+32);
	}
}

asm(
    ".globl arm_exc_irq              \n\t"
    ".func  arm_exc_irq              \n\t"
    ".type   arm_exc_irq, %function  \n\t"
    "arm_exc_irq:                    \n\t"
	"push {r0, r1, r2, r4, lr}       \n\t"
	"mov  r0, #0                     \n\t"
	"add  lr, pc, #4                 \n\t"
	//
	"pop  {r0, r1, r2, r4, lr}       \n\t"
	"subs pc, r14, #4                \n\t"
	".size   arm_exc_irq, .-arm_exc_irq     \n\t"
	".endfunc \n\t"

    ".globl sam7_irq_handler              \n\t"
    ".func  sam7_irq_handler              \n\t"
    ".type   sam7_irq_handler, %function  \n\t"
    "sam7_irq_handler:                    \n\t"
	"push {lr}                          \n\t"
	"mrs  lr, spsr                      \n\t"
	"push {r0, r1, r2, r3, lr}          \n\t"
    ".globl pwetpwet                    \n\t"
    "pwetpwet:                          \n\t"
	"ldr  r0, [sp, #4*6]                \n\t"
	"mov  lr, pc                        \n\t"
	"ldr  pc, =icu_sam7_handler         \n\t"
	"ldr  r0, =0xfffff130               \n\t"
	"str  lr, [r0]                      \n\t"
	"pop  {r0, r1, r2, r3, lr}          \n\t"
	"msr  spsr, lr                     \n\t"
	"pop  {lr}                         \n\t"
	"subs pc, r14, #4                \n\t"
	".size   sam7_irq_handler, .-sam7_irq_handler     \n\t"
	".endfunc \n\t"
	);
