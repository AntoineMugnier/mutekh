/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/


#include <hexo/types.h>

#include <device/class/char.h>
#include <device/device.h>
#include <device/driver.h>

#include <arch/hexo/emu_syscalls.h>

/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) 2012 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <arch/hexo/emu_syscalls.h>

#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/char.h>

#define emu_tty_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

DRIVER_PV(struct {});

static DEV_CHAR_REQUEST(emu_tty_request)
{
  //  struct device_s *dev = accessor->dev;

  bool_t partial = 0;
  reg_t fd;
  reg_t id;

  assert(rq->size);

  switch (rq->type)
  {
  case DEV_CHAR_READ_PARTIAL:
    partial = 1;
  case DEV_CHAR_READ:
    fd = 0;
    id = EMU_SYSCALL_READ;
    break;

  case DEV_CHAR_WRITE_PARTIAL_FLUSH:
  case DEV_CHAR_WRITE_PARTIAL:
    partial = 1;
  case DEV_CHAR_WRITE_FLUSH:
  case DEV_CHAR_WRITE:
    fd = 1;
    id = EMU_SYSCALL_WRITE;
    break;

  default:
    rq->error = -ENOTSUP;
    goto end;
  }

  do {
    ssize_t size;
    do {
      size = emu_do_syscall(id, 3, fd, rq->data, rq->size);
    } while (size == -4 /* -EINTR */);

    if (size <= 0)
      rq->error = -EIO;
    else {
      rq->data += size;
      rq->size -= size;
      rq->error = 0;
    }

  } while (rq->size > 0 && !rq->error && !partial);

 end:
  kroutine_exec(&rq->base.kr);
}

const struct driver_s emu_tty_drv;

static DEV_INIT(emu_tty_init)
{
  return 0;
}

static DEV_CLEANUP(emu_tty_cleanup)
{
  return 0;
}

#define emu_tty_use dev_use_generic

DRIVER_DECLARE(emu_tty_drv, 0, "Emu TTY", emu_tty,
               DRIVER_CHAR_METHODS(emu_tty));

DRIVER_REGISTER(emu_tty_drv);
