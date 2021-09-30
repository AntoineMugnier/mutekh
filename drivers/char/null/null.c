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

static DEV_CHAR_CANCEL(dev_null_cancel)
{
  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
    case DEV_CHAR_DISCARD:
    case DEV_CHAR_READ_FRAME:
    case DEV_CHAR_READ_POLL:
      return 0;
    default:
      return -EBUSY;
    }
}

static DEV_CHAR_REQUEST(dev_null_request)
{
  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
    case DEV_CHAR_DISCARD:
    case DEV_CHAR_READ_FRAME:
    case DEV_CHAR_READ_POLL:
      rq->error = 0;
      break;                    /* wait for cancel */

    case DEV_CHAR_READ_NONBLOCK:
      rq->error = 0;
      dev_char_rq_done(rq);
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
      dev_char_rq_done(rq);
      break;

    default:
      rq->error = -ENOTSUP;
    }
}

static DEV_INIT(dev_null_init)
{
  return 0;
}

static DEV_CLEANUP(dev_null_cleanup)
{
  return 0;
}

#define dev_null_use dev_use_generic

DRIVER_DECLARE(dev_null_drv, 0, "dev-null", dev_null,
               DRIVER_CHAR_METHODS(dev_null));

DRIVER_REGISTER(dev_null_drv);

DEV_DECLARE_STATIC(char_null_dev, "char-null", 0, dev_null_drv,
                   );
