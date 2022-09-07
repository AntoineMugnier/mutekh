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
#include <arch/efm32/msc.h>

.syntax unified

.macro LOAD_WORD dst addr tmp
#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
	ldr	\dst,     [\addr]
#else
        ldrb    \dst,     [\addr]
        ldrb    \tmp,     [\addr, #1]
        lsls    \tmp,     #8
        orrs    \dst,     \tmp
        ldrb    \tmp,     [\addr, #2]
        lsls    \tmp,     #16
        orrs    \dst,     \tmp
        ldrb    \tmp,     [\addr, #3]
        lsls    \tmp,     #24
        orrs    \dst,     \tmp
#endif
.endm

/*
  This function erases a page of flash. The msc_addr parameter
  must point to the Memory System Controller registers.

  The size of the page is device dependent. The return value
  contains the error bits of the MSC status register.
*/

FUNC_START(.text, flash_page_erase)

	push	{r4, r5, r6, r7, lr}

	ldr	r3,	= EFM32_MSC_ADDR

	/* copy code to stack */
	movs	r4,	4f - 3f
	ldr	r5,	= 3f
	sub	sp,	4f - 3f
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
	str	r5,	[r3, #EFM32_MSC_LOCK_ADDR]

	movs	r5,	#EFM32_MSC_WRITECTRL_WREN
	str	r5,	[r3, #EFM32_MSC_WRITECTRL_ADDR]

	/* barrier and jump to code on stack */
	dsb
	isb

	adds	r7,	#1
	blx	r7

	/* lock access to flash write registers */
	movs	r5,	#0
	str	r5,	[r3, #EFM32_MSC_WRITECTRL_ADDR]
	str	r5,	[r3, #EFM32_MSC_LOCK_ADDR]

	/* return MSC status error bits */
	ldr	r0,	[r3, #EFM32_MSC_STATUS_ADDR]
	movs	r1,	#(EFM32_MSC_STATUS_LOCKED | EFM32_MSC_STATUS_INVADDR)
	ands	r0,	r1

	/* restore irqs and return */
	msr	primask,	r6

	add	sp,     4f - 3f
	pop	{r4, r5, r6, r7, pc}

	.align	2
3:
	/* set address */
	str	r0,	[r3, #EFM32_MSC_ADDRB_ADDR]
	movs	r5,	#EFM32_MSC_WRITECMD_LADDRIM
	str	r5,	[r3, #EFM32_MSC_WRITECMD_ADDR]

	/* erase page */
	movs	r5,	#EFM32_MSC_WRITECMD_ERASEPAGE
	str	r5,	[r3, #EFM32_MSC_WRITECMD_ADDR]

1:
	/* wait for completion */
	ldr	r5,	[r3, #EFM32_MSC_STATUS_ADDR]
	lsrs	r5,	#(EFM32_MSC_STATUS_BUSY_SHIFT+1)
	bcs	1b

	bx	lr
	.align	2
4:
FUNC_END(flash_page_erase)

/*
  This function writes data to a page of flash. The msc_addr
  parameter must point to the Memory System Controller
  registers. The write operation can not span across multiple pages.

  The return value indicates if the new data in flash is different
  from the passed data buffer.
*/

FUNC_START(.text, flash_page_write)

	push	{r4, r5, r6, r7, lr}

	ldr	r3,	= EFM32_MSC_ADDR
        lsrs    r2,     #2

	/* copy code to stack */
	movs	r4,	5f - 3f
	ldr	r5,	= 3f
	sub	sp,	5f - 3f
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
	str	r5,	[r3, #EFM32_MSC_LOCK_ADDR]

	movs	r5,	#EFM32_MSC_WRITECTRL_WREN
	str	r5,	[r3, #EFM32_MSC_WRITECTRL_ADDR]

	/* barrier and jump to code on stack */
	dsb
	isb

	adds	r7,	#1
	blx	r7

	/* lock access to flash write registers */
	movs	r5,	#0
	str	r5,	[r3, #EFM32_MSC_WRITECTRL_ADDR]
	str	r5,	[r3, #EFM32_MSC_LOCK_ADDR]

	/* restore irqs and return */
	msr	primask,	r6

	add	sp,     5f - 3f
	pop	{r4, r5, r6, r7, pc}

	.align	2
3:
	/* set address */
	str	r0,	[r3, #EFM32_MSC_ADDRB_ADDR]
	movs	r5,	#EFM32_MSC_WRITECMD_LADDRIM
	str	r5,	[r3, #EFM32_MSC_WRITECMD_ADDR]
2:
	/* set data */
        LOAD_WORD r7,   r0,     r5 // current value
        LOAD_WORD r4,   r1,     r5 // new value
        mvns    r5,     r7
        orrs    r5,     r4
	str	r5,	[r3, #EFM32_MSC_WDATA_ADDR]

	/* write once */
	movs	r5,	#EFM32_MSC_WRITECMD_WRITEONCE
	str	r5,	[r3, #EFM32_MSC_WRITECMD_ADDR]

1:
	/* wait for completion */
	ldr	r5,	[r3, #EFM32_MSC_STATUS_ADDR]
	lsrs	r5,	#(EFM32_MSC_STATUS_BUSY_SHIFT+1)
	bcs	1b

	/* readback and test */
        LOAD_WORD r7,   r0,     r5
        mvns    r4,     r4
        ands    r7,     r4
        bne     4f

	/* repeat */
	adds	r1,	#4
	adds	r0,	#4
	subs	r2,	#1
#if EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32G
	bne	3b
#else
	bne	2b
#endif

        movs    r0, #0
	bx	lr
4:
        movs    r0, #1
	bx	lr

	.align	2
5:
FUNC_END(flash_page_write)
