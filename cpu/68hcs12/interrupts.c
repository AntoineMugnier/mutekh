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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/local.h>

#define MK_VECTOR(id,handler)		\
  asm (".global vect" #id "\n"		\
       "vect" #id ":\n"			\
       "	ldab	#" #id "\n"	\
       "	bsr	" #handler "\n"	\
       "	rti")

/*
 * exceptions
 */

/* high level handlers */
void	exception_handler(uint8_t	id)
{
  CPU_LOCAL_GET(cpu_exception_handler)(id, 0, 0, NULL, NULL);
}

/* prehandlers */
MK_VECTOR(1, exception_handler);
MK_VECTOR(2, exception_handler);
MK_VECTOR(3, exception_handler);

/*
 * sw interrupts and IRQs
 */

/* high level handlers */
void	interrupt_handler(uint8_t	id)
{
  CPU_LOCAL_GET(cpu_interrupt_handler)(id);
}

void	syscall_handler(uint8_t	id)
{
  CPU_LOCAL_GET(cpu_syscall_handler)(id);
}

/* prehandlers */
MK_VECTOR(4, syscall_handler);
MK_VECTOR(5, interrupt_handler);
MK_VECTOR(6, interrupt_handler);
MK_VECTOR(7, interrupt_handler);
MK_VECTOR(8, interrupt_handler);
MK_VECTOR(9, interrupt_handler);
MK_VECTOR(10, interrupt_handler);
MK_VECTOR(11, interrupt_handler);
MK_VECTOR(12, interrupt_handler);
MK_VECTOR(13, interrupt_handler);
MK_VECTOR(14, interrupt_handler);
MK_VECTOR(15, interrupt_handler);
MK_VECTOR(16, interrupt_handler);
MK_VECTOR(17, interrupt_handler);
MK_VECTOR(18, interrupt_handler);
MK_VECTOR(19, interrupt_handler);
MK_VECTOR(20, interrupt_handler);
MK_VECTOR(21, interrupt_handler);
MK_VECTOR(22, interrupt_handler);
MK_VECTOR(23, interrupt_handler);
MK_VECTOR(24, interrupt_handler);
MK_VECTOR(25, interrupt_handler);
MK_VECTOR(26, interrupt_handler);
MK_VECTOR(27, interrupt_handler);
MK_VECTOR(28, interrupt_handler);
MK_VECTOR(29, interrupt_handler);
MK_VECTOR(30, interrupt_handler);
MK_VECTOR(31, interrupt_handler);
MK_VECTOR(32, interrupt_handler);
MK_VECTOR(33, interrupt_handler);
MK_VECTOR(34, interrupt_handler);
MK_VECTOR(35, interrupt_handler);
MK_VECTOR(36, interrupt_handler);
MK_VECTOR(37, interrupt_handler);
MK_VECTOR(38, interrupt_handler);
MK_VECTOR(39, interrupt_handler);
MK_VECTOR(40, interrupt_handler);
MK_VECTOR(41, interrupt_handler);
MK_VECTOR(42, interrupt_handler);
MK_VECTOR(43, interrupt_handler);
MK_VECTOR(44, interrupt_handler);
MK_VECTOR(45, interrupt_handler);
MK_VECTOR(46, interrupt_handler);
MK_VECTOR(47, interrupt_handler);
MK_VECTOR(48, interrupt_handler);
MK_VECTOR(49, interrupt_handler);
MK_VECTOR(50, interrupt_handler);
MK_VECTOR(51, interrupt_handler);
MK_VECTOR(52, interrupt_handler);
MK_VECTOR(53, interrupt_handler);
MK_VECTOR(54, interrupt_handler);
MK_VECTOR(55, interrupt_handler);
MK_VECTOR(56, interrupt_handler);
MK_VECTOR(57, interrupt_handler);
MK_VECTOR(58, interrupt_handler);
MK_VECTOR(59, interrupt_handler);
MK_VECTOR(60, interrupt_handler);
MK_VECTOR(61, interrupt_handler);
MK_VECTOR(62, interrupt_handler);
MK_VECTOR(63, interrupt_handler);
MK_VECTOR(64, interrupt_handler);
