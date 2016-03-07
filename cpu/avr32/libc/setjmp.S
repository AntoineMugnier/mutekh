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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012

*/

#include <hexo/asm.h>
#include <hexo/cpu.h>

FUNC_START(.text, setjmp)

        stm     r12, r0, r1, r2, r3, r4, r5, r6, r7, sp, lr
        retal   0

FUNC_END(setjmp)

FUNC_START(.text, longjmp)

        ldm     r12, r0, r1, r2, r3, r4, r5, r6, r7, sp, lr
        cp      r11, 0
        reteq   1
        retal   r11

FUNC_END(longjmp)

