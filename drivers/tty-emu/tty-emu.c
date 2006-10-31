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

#include <hexo/device/char.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <arch/hexo/stdio.h>

#include "tty-emu.h"

/**************************************************************/

/*
 * device read operation
 */

DEVCHAR_READ(tty_emu_read)
{
  return fread(data, 1, size, stdin);
}

/*
 * device write operation
 */

DEVCHAR_WRITE(tty_emu_write)
{
  return fwrite(data, 1, size, stdout);
}

/*
 * device close operation
 */

DEV_CLEANUP(tty_emu_cleanup)
{
  fflush(stdout);
  fpurge(stdin);
}

/*
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	tty_emu_drv =
{
  .f_init		= tty_emu_init,
  .f_cleanup		= tty_emu_cleanup,
  .f_irq		= NULL,
  .f.chr = {
    .f_read		= tty_emu_read,
    .f_write		= tty_emu_write,
  }
};
#endif

DEV_INIT(tty_emu_init)
{
#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &tty_emu_drv;
#endif

  fflush(stdout);
  fpurge(stdin);

  return 0;
}

