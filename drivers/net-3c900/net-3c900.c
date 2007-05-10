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


#include <hexo/types.h>

#include <hexo/device/net.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>
#include <stdlib.h>

#include "net-3c900.h"

DEV_IRQ(net_3c900_irq)
{
  return 0;
}

DEV_INIT(net_3c900_init)
{
  printf("3com 3c900 driver init on device %p\n", dev);

  return 0;
}

DEV_CLEANUP(net_3c900_cleanup)
{
}

DEVNET_PREPAREPKT(net_3c900_preparepkt)
{
  return 0;
}

DEVNET_SENDPKT(net_3c900_sendpkt)
{
}

#ifndef CONFIG_STATIC_DRIVERS
static const struct devenum_ident_s	net_3c900_ids[] =
  {
    { .vendor = 0x10b7, .device = 0x9055, .class = -1 },
    { .vendor = 0x10b7, .device = 0x9200, .class = -1 },
    { 0 }
  };

const struct driver_s	net_3c900_drv =
{
  .id_table		= net_3c900_ids,

  .f_init		= net_3c900_init,
  .f_cleanup		= net_3c900_cleanup,
  .f_irq		= net_3c900_irq,
  .f.net = {
    .f_preparepkt	= net_3c900_preparepkt,
    .f_sendpkt		= net_3c900_sendpkt,
  }
};
#endif

