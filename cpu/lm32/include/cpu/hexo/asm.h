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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2011
*/

#ifndef CPU_ASM_H_
#define CPU_ASM_H_

asm(
".macro seta reg sym \n"
"       orhi \\reg, r0, hi(\\sym) \n"
"       ori  \\reg, \\reg, lo(\\sym) \n"
".endm \n"

".macro seti reg val \n"
"       orhi \\reg, r0, (\\val) >> 16 \n"
"       ori  \\reg, \\reg, (\\val) & 0xffff \n"
".endm \n"
    );

#endif

