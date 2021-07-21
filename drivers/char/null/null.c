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

#include "null.h"

#include <hexo/types.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>

DRIVER_PV(struct {});

#define dev_null_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

DEV_CHAR_REQUEST(dev_null_request)
{
  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
      rq->error = -EPIPE;
      break;

      /* Eat everything */
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE:
      rq->data += rq->size;
      rq->size = 0;
      rq->error = 0;
      break;

    default:
      rq->error = -ENOTSUP;
    }

  dev_char_rq_done(rq);
}

DEV_INIT(dev_null_init)
{
  return 0;
}

DEV_CLEANUP(dev_null_cleanup)
{
  return 0;
}

#define dev_null_use dev_use_generic

DRIVER_DECLARE(dev_null_drv, 0, "dev-null", dev_null,
               DRIVER_CHAR_METHODS(dev_null));

DRIVER_REGISTER(dev_null_drv);

DEV_DECLARE_STATIC(char_null_dev, "char-null", 0, dev_null_drv,
                   );
