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

static DEV_CHAR_REQUEST(emu_tty_request)
{
  //  struct device_s *dev = accessor->dev;

  bool_t nonblock = 0;
  reg_t fd;
  reg_t id;

  assert(rq->size);

  switch (rq->type)
  {
  case DEV_CHAR_READ_NONBLOCK:
    nonblock = 1;
  case DEV_CHAR_READ:
    fd = 0;
    id = EMU_SYSCALL_READ;
    break;

  case DEV_CHAR_WRITE_NONBLOCK:
    nonblock = 1;
  case DEV_CHAR_WRITE:
    fd = 1;
    id = EMU_SYSCALL_WRITE;
    break;

  default:
    rq->error = -EINVAL;
    kroutine_exec(&rq->kr, cpu_is_interruptible());
    return;
  }

  while (1) {
    ssize_t size = emu_do_syscall(id, 3, fd, rq->data, rq->size);

    if (size == 0)
      rq->error = EEOF;
    else if (size < 0)
      rq->error = EIO;
    else {
      rq->data += size;
      rq->size -= size;
      rq->error = 0;
    }

    if (rq->size == 0 || rq->error || nonblock) {
      kroutine_exec(&rq->kr, cpu_is_interruptible());
      return;
    }
  }
}

static const struct driver_char_s        emu_tty_char_drv =
{
  .class_               = DRIVER_CLASS_CHAR,
  .f_request            = emu_tty_request,
};

static DEV_INIT(emu_tty_init);
static DEV_CLEANUP(emu_tty_cleanup);

const struct driver_s   emu_tty_drv =
{
  .desc                 = "Unix TTY",
  .f_init               = emu_tty_init,
  .f_cleanup            = emu_tty_cleanup,
  .classes              = { &emu_tty_char_drv, 0 }
};

static DEV_INIT(emu_tty_init)
{
  dev->drv = &emu_tty_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;
}

static DEV_CLEANUP(emu_tty_cleanup)
{
}
