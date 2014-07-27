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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014
*/

#include <hexo/asm.h>
#include <arch/efm32_msc.h>

.syntax unified

#define EFM32_FLASH_ERASE_CODESIZE 0x14
FUNC_START(.text, efm32_flash_erase)

	push	{r4, r5, r6, r7, lr}

	/* copy code to stack */
	movs	r4,	#EFM32_FLASH_ERASE_CODESIZE
	ldr	r5,	= 3f
	sub	sp,	#EFM32_FLASH_ERASE_CODESIZE
	mov	r7,	sp
1:
	subs	r4,	#4
	ldr	r6,	[r5, r4]
	str	r6,	[r7, r4]
	bne	1b

	/* disable irqs */
	mrs	r6,	primask
	cpsid	i

	/* unlock and enable flash write feature */
	ldr	r5,	= EFM32_MSC_LOCK_LOCKKEY_UNLOCK
	str	r5,	[r0, #EFM32_MSC_LOCK_ADDR]

	movs	r5,	#EFM32_MSC_WRITECTRL_WREN
	str	r5,	[r0, #EFM32_MSC_WRITECTRL_ADDR]

	/* barrier and jump to code on stack */
	dsb
	isb

	adds	r7,	#1
	blx	r7

	/* lock access to flash write registers */
	movs	r5,	#0
	str	r5,	[r0, #EFM32_MSC_WRITECTRL_ADDR]
	str	r5,	[r0, #EFM32_MSC_LOCK_ADDR]

	/* return MSC status error bits */
	ldr	r0,	[r0, #EFM32_MSC_STATUS_ADDR]
	movs	r1,	#(EFM32_MSC_STATUS_LOCKED | EFM32_MSC_STATUS_INVADDR)
	ands	r0,	r1

	/* restore irqs and return */
	msr	primask,	r6

	add	sp, #EFM32_FLASH_ERASE_CODESIZE
	pop	{r4, r5, r6, r7, pc}

	.align	2
3:
	/* set address */
	str	r1,	[r0, #EFM32_MSC_ADDRB_ADDR]
	movs	r5,	#EFM32_MSC_WRITECMD_LADDRIM
	str	r5,	[r0, #EFM32_MSC_WRITECMD_ADDR]

	/* erase page */
	movs	r5,	#EFM32_MSC_WRITECMD_ERASEPAGE
	str	r5,	[r0, #EFM32_MSC_WRITECMD_ADDR]

1:
	/* wait for completion */
	ldr	r5,	[r0, #EFM32_MSC_STATUS_ADDR]
	lsrs	r5,	#(EFM32_MSC_STATUS_BUSY_SHIFT+1)
	bcs	1b

	bx	lr
	.align	2

FUNC_END(efm32_flash_erase)

#define EFM32_FLASH_WRITE_CODESIZE 0x28
FUNC_START(.text, efm32_flash_write)

	push	{r4, r5, r6, r7, lr}

	/* copy code to stack */
	movs	r4,	#EFM32_FLASH_WRITE_CODESIZE
	ldr	r5,	= 3f
	sub	sp,	#EFM32_FLASH_WRITE_CODESIZE
	mov	r7,	sp
1:
	subs	r4,	#4
	ldr	r6,	[r5, r4]
	str	r6,	[r7, r4]
	bne	1b

	/* disable irqs */
	mrs	r6,	primask
	cpsid	i

	/* unlock and enable flash write feature */
	ldr	r5,	= EFM32_MSC_LOCK_LOCKKEY_UNLOCK
	str	r5,	[r0, #EFM32_MSC_LOCK_ADDR]

	movs	r5,	#EFM32_MSC_WRITECTRL_WREN
	str	r5,	[r0, #EFM32_MSC_WRITECTRL_ADDR]

	/* barrier and jump to code on stack */
	dsb
	isb

	adds	r7,	#1
	blx	r7

	/* lock access to flash write registers */
	movs	r5,	#0
	str	r5,	[r0, #EFM32_MSC_WRITECTRL_ADDR]
	str	r5,	[r0, #EFM32_MSC_LOCK_ADDR]

	/* return value */
	movs	r0,	r7

	/* restore irqs and return */
	msr	primask,	r6

	add	sp, #EFM32_FLASH_WRITE_CODESIZE
	pop	{r4, r5, r6, r7, pc}

	.align	2
3:
	movs	r7,	#0
4:
	/* set address */
	str	r1,	[r0, #EFM32_MSC_ADDRB_ADDR]
	movs	r5,	#EFM32_MSC_WRITECMD_LADDRIM
	str	r5,	[r0, #EFM32_MSC_WRITECMD_ADDR]
2:
	/* set data */
	ldr	r4,	[r2]
	str	r4,	[r0, #EFM32_MSC_WDATA_ADDR]

	/* write once */
	movs	r5,	#EFM32_MSC_WRITECMD_WRITEONCE
	str	r5,	[r0, #EFM32_MSC_WRITECMD_ADDR]

1:
	/* wait for completion */
	ldr	r5,	[r0, #EFM32_MSC_STATUS_ADDR]
	lsrs	r5,	#(EFM32_MSC_STATUS_BUSY_SHIFT+1)
	bcs	1b

	/* readback and diff */
	ldr	r5,	[r1]
	eors	r5,	r4
	orrs	r7,	r5

	/* repeat */
	adds	r2,	#4
	adds	r1,	#4
	subs	r3,	#1
#ifdef CONFIG_EFM32_GECKO
	bne	4b
#else
	bne	2b
#endif

	bx	lr
	.align	2

FUNC_END(efm32_flash_write)


