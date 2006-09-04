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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


/** 
    @file

    IBMPC plateform specific declarations
*/

#ifndef ARCH_SPECIFIC_H_
#define ARCH_SPECIFIC_H_

/* boot section address in memory */
extern __ldscript_symbol_t __boot_start, __boot_end;

/** index of the mandatory GDT null descriptor */
#define ARCH_GDT_NULL		0

/** index of the code segment descriptor in GDT */
#define ARCH_GDT_CODE_INDEX	1

/** index of the data segment descriptor in GDT */
#define ARCH_GDT_DATA_INDEX	2

/** index of the cpu local storage segment */
#define ARCH_GDT_CPUDATA_INDEX	3

/** First GDT available descriptor */
#define ARCH_GDT_FIRST_ALLOC	4

/** size of the Globale Descriptor Table for x86 CPU */
#define ARCH_GDT_SIZE		256

/** CPU specific boot address for x86 SMP bootup sequence */
#define ARCH_SMP_BOOT_ADDR	0x11000

#endif

