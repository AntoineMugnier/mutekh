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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2018
*/

#include <hexo/asm.h>
#include <arch/efm32/devaddr.h>
#include <arch/efm32/rtc.h>
#include <arch/efm32/cmu.h>

#include "hwrand.h"

.syntax unified

/* This extracts entropy from the jitter of RC oscillators.
   It must be called with RTC running on LFRC and cpu running on HFRC. */

FUNC_START(.text, efm32_hw_rand32)
	push	{r0, r1, r2, r4, r5, r6, r7, lr}

        /* CRC */
	movs	r0, #HWRAND_CRC32_INIT

        /* LFSR init */
        movs    r1, #1

        /* RC tunning init */
	ldr	r6, = EFM32_CMU_ADDR + EFM32_CMU_HFRCOCTRL_ADDR
        ldr     r2, [r6]

.loop:
        /* read current RTC value */
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
	ldr	r5, = EFM32_RTC_ADDR + EFM32_RTC_CNT_ADDR
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
	ldr	r5, = EFM32_RTCC_ADDR + EFM32_RTCC_CNT_ADDR
#endif
	ldr	r7, [r5]

	movs	r3, #0
.rtc_poll_loop:
        /* poll on RTC, waiting for a value change */
	ldr	r4, [r5]
	adds	r3, #1
	cmp	r7, r4
	beq	.rtc_poll_loop

.rtc_changed:
	movs	r4, #1

        /* update CRC with jitter bit */
	lsrs	r5, r0, #1
	eors	r0, r3
	ands	r0, r4
	ldr	r3, = HWRAND_CRC32_POLY
	negs	r0, r0
	ands	r0, r3
	eors	r0, r5

        /* update LFSR */
	lsrs	r5, r1, #1
	ands	r1, r4
	movs	r3, #0xb8  /* polynomial */
	negs	r1, r1
	ands	r1, r3
	eors	r1, r5

        /* change HFRCO tunning */
        movs    r3, r2
	eors	r3, r1
        str     r3, [r6]

        /* while LFSR != 1 */
	cmp     r4, r1
	bne	.loop

        /* restore RC tunning */
        str     r2, [r6]

	pop	{r1, r2, r3, r4, r5, r6, r7, pc}

FUNC_END(efm32_hw_rand32)
