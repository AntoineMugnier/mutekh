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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <netinet/if.h>

/*
 * test main.
 */

int_fast8_t		main()
{
#if 0
  /* #AC on intel */

  asm volatile("movl %%cr0, %%eax\n\t"
	       "orl $0x43000, %%eax\n\t"
	       "movl %%eax, %%cr0"
	       :
	       :
	       : "%eax");
  asm volatile("mov $0x23, %%ax\n\t"
	       "mov %%ax, %%ds\n\t"
	       "mov %%ax, %%es\n\t"
	       "mov %%ax, %%fs\n\t"
	       "mov %%ax, %%gs\n\t"
	       :
	       :
	       : "%eax");
  uint16_t tr = 0x28;
  asm volatile("ltr %0\n\t"
	       :
	       : "r" (tr));
  asm volatile("pushl $0x23\n\t"
	       "pushl %esp\n\t"
	       "pushl $0x43046\n\t"
	       "pushl $0x1b\n\t"
	       "pushl $1f\n\t"
	       "iret\n\t"
	       "1:");

  uint32_t *p = 3;

  printf("%p\n", p);

  *p = 42;
  printf("%d\n", *p);
#endif

  uint_fast32_t i;
  for (i = 0; i < 10000000; i++)
    ;

  if_up("eth0");
  if_up("eth1");

  return 0;
}

