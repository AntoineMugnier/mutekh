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
.global cpu_boot
cpu_boot:

	rjmp	1f

	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti

	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti

	reti
	reti
	reti
	reti
	reti
	reti
	reti
	reti

	reti
	reti
	reti
	reti

1:

	/* setup stack pointer */
	ldi	r16, hi8(__system_heap_end)
	out	0x3e, r16
	ldi	r16, lo8(__system_heap_end)
	out	0x3d, r16

	/* clear r1 register */
	clr	r1
	/* call arch_init function */
	rcall	arch_init

	/* stop processor execution */
	cli
	in	r16, 0x35
	ori	r16, 0x40
	out	0x35, r16
	sleep

