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

#include <mutek/bytecode.h>

#if CONFIG_CPU_ARM32M_ARCH_VERSION == 6
extern inline uint32_t arm32m_bc_div32(uint32_t a, uint32_t b, uint32_t *r)
{
  uint32_t q = a / b;
  *r = a - q * b;
  return q;
}
#endif

/* trampolines for native bytecode custom instructions */
asm(
    "    .section .text.arm32m_bc_trampoline\n"
    "    .code 16\n"
    "    .thumb_func\n"
    "    .globl arm32m_bc_trampoline\n"
    "    .func  arm32m_bc_trampoline\n"
    "    .type  arm32m_bc_trampoline, \%function\n"
    "arm32m_bc_trampoline:\n"
#if CONFIG_CPU_ARM32M_ARCH_VERSION == 7
    /* get opcode */
    "    sub lr, #1\n"
    "    ldrh r0, [lr], #2\n"
    /* save resume address */
    "    str lr, [r4, #64]\n"
    /* return */
    "    pop    {r4, r5, r6, r7, r8, pc}\n"
#endif
#if CONFIG_CPU_ARM32M_ARCH_VERSION == 6
    /* get opcode */
    "    mov  r3, lr\n"
    "    sub  r3, #1\n"
    "    ldrh r0, [r3]\n"
    /* save resume address */
    "    add r3, #2\n"
    "    str r3, [r4, #64]\n"
    /* return */
    "    pop    {r4, r5, r6, r7, pc}\n"
#endif
    "    .endfunc\n"
    "    .size arm32m_bc_trampoline, . - arm32m_bc_trampoline\n"

    "    .section .text.arm32m_bc_trampoline_cond\n"
    "    .code 16\n"
    "    .thumb_func\n"
    "    .globl arm32m_bc_trampoline_cond\n"
    "    .func  arm32m_bc_trampoline_cond\n"
    "    .type  arm32m_bc_trampoline_cond, \%function\n"
    "arm32m_bc_trampoline_cond:\n"
#if CONFIG_CPU_ARM32M_ARCH_VERSION == 7
    /* get opcode */
    "    sub lr, #1\n"
    "    ldrh r0, [lr], #2\n"
    /* save conditional skip amount */
    "    ldrh r2, [lr], #2\n"
    "    strb r2, [r4, #72]\n"
    /* save resume address */
    "    str lr, [r4, #64]\n"
    /* return */
    "    pop    {r4, r5, r6, r7, r8, pc}\n"
#endif
#if CONFIG_CPU_ARM32M_ARCH_VERSION == 6
    /* get opcode */
    "    mov  r3, lr\n"
    "    sub r3, #1\n"
    "    ldrh r0, [r3]\n"
    /* save resume address */
    "    ldrh r2, [r3, #2]\n"
    "    add r3, #4\n"
    "    str r3, [r4, #64]\n"
    /* save conditional skip amount */
    "    add r4, #72\n"
    "    strb r2, [r4]\n"
    /* return */
    "    pop    {r4, r5, r6, r7, pc}\n"
#endif
    "    .endfunc\n"
    "    .size arm32m_bc_trampoline_cond, . - arm32m_bc_trampoline_cond\n"
    );
