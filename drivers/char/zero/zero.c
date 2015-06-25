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

#include "zero.h"

#include <hexo/types.h>
#include <device/device.h>
#include <device/driver.h>

DEV_CHAR_REQUEST(dev_zero_request)
{
  switch (rq->type)
    {
      /* Get zeros */
    case DEV_CHAR_READ: {
      size_t size = rq->size;

      memset(rq->data, 0, rq->size);

      rq->size = 0;
      rq->error = 0;

      rq->callback(dev, rq, size);
      break;
    }

      /* Eat everything */
    case DEV_CHAR_WRITE: {
      size_t size = rq->size;

      rq->size = 0;
      rq->error = 0;

      rq->callback(dev, rq, size);
      break;
    }

    }
}

/* 
 * device close operation
 */

DEV_CLEANUP(dev_zero_cleanup)
{
}

/* 
 * device open operation
 */

#define dev_zero_use dev_use_generic

DRIVER_DECLARE(dev_zero_drv, "dev-zero", dev_zero,
               DRIVER_CHAR_METHODS(dev_zero));

DRIVER_REGISTER(dev_zero_drv);

DEV_INIT(dev_zero_init)
{
  dev->drv = &dev_zero_drv;

  return 0;
}

