/* This file is part of MutekH.
 *
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 *
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Alexandre Becoulet <alexandre.becoulet@free.fr>
 */

#include <hexo/asm.h>
#include <mutek/bytecode.h>

.if _offsetof(bc_context_s, mode, 1) != 18 * 4 + 1
  .err // need to update bc_backend_armv?m.pm
.endif

.if _offsetof(bc_context_s, pc, 4) != 16 * 4
  .err // need to update bc_backend_armv?m.pm
.endif

.syntax unified

#if CONFIG_CPU_ARM32M_ARCH_VERSION == 6
//   uint32_t arm32m_bc_div32(uint32_t a, uint32_t b, uint32_t *r)
//   {
//      uint32_t q = a / b;
//      *r = a - q * b;
//      return q;
//   }

FUNC_START(.text, arm32m_bc_div32)
	.globl	__aeabi_uidiv
	push	{r4, r5, r6, lr}
	mov	r6, r2
	mov	r5, r1
	mov	r4, r0
	bl	__aeabi_uidiv
	muls	r5, r0
	subs	r4, r4, r5
	str	r4, [r6]
	pop	{r4, r5, r6, pc}
FUNC_END(arm32m_bc_div32)
#endif

/* trampolines for native bytecode custom instructions */
FUNC_START(.text, arm32m_bc_trampoline)

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
    /* get opcode */
        sub lr, #1
        ldrh r0, [lr], #2
    /* save resume address */
        str lr, [r4, #_offsetof(bc_context_s, pc, 4)]
    /* return */
        pop    {r4, r5, r6, r7, r8, pc}
#endif
#if CONFIG_CPU_ARM32M_ARCH_VERSION == 6
    /* get opcode */
        mov  r3, lr
        subs r3, #1
        ldrh r0, [r3]
    /* save resume address */
        adds r3, #2
        str r3, [r4, #_offsetof(bc_context_s, pc, 4)]
    /* return */
        pop    {r4, r5, r6, r7, pc}
#endif
FUNC_END(arm32m_bc_trampoline)


/* trampolines for native bytecode custom conditional instructions */
FUNC_START(.text, arm32m_bc_trampoline_cond)

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7
    /* get opcode */
        sub lr, #1
        ldrh r0, [lr], #2
    /* save conditional skip amount */
        ldrh r2, [lr], #2
        strb r2, [r4, #_offsetof(bc_context_s, skip, 1)]
    /* save resume address */
        str lr, [r4, #_offsetof(bc_context_s, pc, 4)]
    /* return */
        pop    {r4, r5, r6, r7, r8, pc}
#endif
#if CONFIG_CPU_ARM32M_ARCH_VERSION == 6
    /* get opcode */
        mov  r3, lr
        subs r3, #1
        ldrh r0, [r3]
    /* save resume address */
        ldrh r2, [r3, #2]
        adds r3, #4
        str r3, [r4, #_offsetof(bc_context_s, pc, 4)]
    /* save conditional skip amount */
        adds r4, #_offsetof(bc_context_s, skip, 1)
        strb r2, [r4]
    /* return */
        pop    {r4, r5, r6, r7, pc}
#endif
FUNC_END(arm32m_bc_trampoline_cond)
