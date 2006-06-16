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

.section        .boot,"ax",@progbits

.globl cpu_boot
cpu_boot:

	.set push
	.set noreorder

        li        $8,   0x0000FF00  # set up IT disable and kernel mode
        mtc0      $8,	$12         # once j fetched, stay in kernel mode with rfe

	/* get CPU id */

	/* change stack offset depending on CPU id FIXME */
        la         $sp,   0x002ffff0

	/* setup global data pointer */
	la	   $gp,   _gp

	/* jumpto arch_init function */

        la         $8,   arch_init
        j          $8
	nop

	.set pop
