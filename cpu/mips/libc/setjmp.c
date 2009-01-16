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

#include <setjmp.h>

reg_t setjmp(jmp_buf env)
{
  register reg_t env_ asm("t0") = (reg_t)env;
  register reg_t ret;

  asm volatile(
	       /* save gp registers */

	       "sw	$16,	16*4(%1)	\n"
	       "sw	$17,	17*4(%1)	\n"
	       "sw	$18,	18*4(%1)	\n"
	       "sw	$19,	19*4(%1)	\n"
	       "sw	$20,	20*4(%1)	\n"
	       "sw	$21,	21*4(%1)	\n"
	       "sw	$22,	22*4(%1)	\n"
	       "sw	$23,	23*4(%1)	\n"
	       "sw	$24,	24*4(%1)	\n"
	       "sw	$25,	25*4(%1)	\n"

	       "sw	$28,	28*4(%1)	\n"
	       "sw	$29,	29*4(%1)	\n"
	       "sw	$30,	30*4(%1)	\n"
	       "sw	$31,	31*4(%1)	\n"

	       "lui	%0,	0		\n"
	       "b	2f			\n"

	       /* restore point */
	       "__cpu_mips_setjmp_restore:	\n"

	       /* restore gp registers */

	       "lw	$16,	16*4(%1)	\n"
	       "lw	$17,	17*4(%1)	\n"
	       "lw	$18,	18*4(%1)	\n"
	       "lw	$19,	19*4(%1)	\n"
	       "lw	$20,	20*4(%1)	\n"
	       "lw	$21,	21*4(%1)	\n"
	       "lw	$22,	22*4(%1)	\n"
	       "lw	$23,	23*4(%1)	\n"
	       "lw	$24,	24*4(%1)	\n"
	       "lw	$25,	25*4(%1)	\n"

	       "lw	$28,	28*4(%1)	\n"
	       "lw	$29,	29*4(%1)	\n"
	       "lw	$30,	30*4(%1)	\n"
	       "lw	$31,	31*4(%1)	\n"

	       "lw	%0,	33*4(%1)	\n"
	       "2:				\n"

	       : "=&r" (ret)
	       : "r" (env_)
	       : "memory"
	       /* these registers are caller saved, just let gcc manage them. */
	       , "$1", "$2", "$3", "$4", "$5", "$6", "$7" /*, "$8" used for env_ */
	       , "$9", "$10", "$11", "$12", "$13", "$14", "$15"
	       );

  return ret;
}

void longjmp(jmp_buf env, reg_t val)
{
  register reg_t env_ asm("t0") = (reg_t)env;

  env[33] = val;
  asm volatile(
	       "j	__cpu_mips_setjmp_restore	\n"
	       :
	       : "r" (env_)
	       : "memory"
	       );
}

