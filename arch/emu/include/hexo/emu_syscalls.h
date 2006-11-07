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
    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#ifndef EMU_SYSCALLS_H_
#define EMU_SYSCALLS_H_

#include <hexo/types.h>
#include <stdarg.h>

/* FIXME linux x86 specific */

static inline reg_t
emu_do_syscall_va(uint_fast16_t id, size_t argc, va_list ap)
{
  reg_t		res;

  if (argc < 6)
    {
      asm volatile ("int $0x80\n"
		    : "=a" (res)
		    : "a" (id)
		    , "b" (va_arg(ap, reg_t))
		    , "c" (va_arg(ap, reg_t))
		    , "d" (va_arg(ap, reg_t))
		    , "S" (va_arg(ap, reg_t))
		    , "D" (va_arg(ap, reg_t))
		    );
    }
  else
    {
      uint_fast8_t	i;
      reg_t		params[argc];

      for (i = 0; i < argc; i++)
	params[i] = va_arg(ap, reg_t);

      asm volatile ("int $0x80\n"
		    : "=a" (res)
		    : "a" (id)
		    , "b" (params)
		    );
    }

  return res;
}

static inline reg_t
emu_do_syscall(uint_fast16_t id, size_t argc, ...)
{
  va_list	ap;
  reg_t		res;

  va_start(ap, argc);
  res = emu_do_syscall_va(id, argc, ap);
  va_end(ap);

  return res;
}

/******************************* mmap syscall */

#define EMU_SYSCALL_MMAP	90

#define PROT_READ		0x1
#define PROT_WRITE		0x2
#define PROT_EXEC		0x4

#define MAP_PRIVATE		0x02
#define MAP_ANONYMOUS		0x20

#define MAP_FAILED		((void *)-1)

/******************************* other syscalls */

#define EMU_SYSCALL_WRITE	4
#define EMU_SYSCALL_READ	3
#define EMU_SYSCALL_EXIT	1

#endif

