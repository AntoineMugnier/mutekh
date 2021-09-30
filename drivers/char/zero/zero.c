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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <hexo/types.h>
#include <device/class/char.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>

DRIVER_PV(struct {});

static DEV_CHAR_CANCEL(dev_zero_cancel)
{
  return -EBUSY;
}

static DEV_CHAR_REQUEST(dev_zero_request)
{
  switch (rq->type)
    {
      /* Get zeros */
    case DEV_CHAR_READ_NONBLOCK:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_FRAME:
      memset(rq->data, 0, rq->size);
    case DEV_CHAR_DISCARD:
      rq->data += rq->size;
      rq->size = 0;
    case DEV_CHAR_READ_POLL:
      rq->error = 0;
      break;

      /* Eat everything */
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_NONBLOCK_FLUSH:
    case DEV_CHAR_WRITE_NONBLOCK:
    case DEV_CHAR_WRITE_FRAME:
      rq->data += rq->size;
      rq->size = 0;
    case DEV_CHAR_WRITE_POLL:
      rq->error = 0;
      break;

    default:
      rq->error = -ENOTSUP;
    }

  dev_char_rq_done(rq);
}

static DEV_INIT(dev_zero_init)
{
  return 0;
}

static DEV_CLEANUP(dev_zero_cleanup)
{
  return 0;
}

#define dev_zero_use dev_use_generic

DRIVER_DECLARE(dev_zero_drv, 0, "dev-zero", dev_zero,
               DRIVER_CHAR_METHODS(dev_zero));

DRIVER_REGISTER(dev_zero_drv);

DEV_DECLARE_STATIC(char_zero_dev, "char-zero", 0, dev_zero_drv,
                   );
