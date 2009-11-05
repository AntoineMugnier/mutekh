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

#include <device/block.h>
#include <device/device.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <stdlib.h>
#include <string.h>

#include "block-ramdisk.h"
#include "block-ramdisk-private.h"

/**************************************************************/

/* 
 * device read/write request
 */

DEVBLOCK_REQUEST(block_ramdisk_request)
{
  struct block_ramdisk_context_s *pv = dev->drv_pv;
  struct dev_block_params_s *p = &pv->params;
  dev_block_lba_t lba = rq->lba;
  dev_block_lba_t count = rq->count;

  lock_spin(&dev->lock);

  if (lba + count <= p->blk_count)
    {
      size_t b;

      switch (rq->type)
	{
	case DEV_BLOCK_READ:
	  for (b = 0; b < count; b++)
	    memcpy(rq->data[b], pv->mem + ((lba + b) * p->blk_size), p->blk_size);
	  break;

	case DEV_BLOCK_WRITE:
	  for (b = 0; b < count; b++)
	    memcpy(pv->mem + ((lba + b) * p->blk_size), rq->data[b], p->blk_size);
	  break;
	}

      rq->error = 0;
      rq->count -= count;
      rq->lba += count;
      rq->callback(dev, rq, count);
    }
  else
    {
      rq->error = ERANGE;
      rq->callback(dev, rq, 0);
    }

  lock_release(&dev->lock);
}

/* 
 * device params
 */

DEVBLOCK_GETPARAMS(block_ramdisk_getparams)
{
  return &(((struct block_ramdisk_context_s *)(dev->drv_pv))->params);
}

/* 
 * device close operation
 */

DEV_CLEANUP(block_ramdisk_cleanup)
{
  struct block_ramdisk_context_s	*pv = dev->drv_pv;

  mem_free(pv->mem);
  mem_free(pv);
}

/* 
 * device open operation
 */

const struct driver_s	block_ramdisk_drv =
{
  .class		= device_class_block,
  .f_init		= block_ramdisk_init,
  .f_cleanup		= block_ramdisk_cleanup,
  .f_irq		= DEVICE_IRQ_INVALID,
  .f.blk = {
    .f_request		= block_ramdisk_request,
    .f_getparams	= block_ramdisk_getparams,
  }
};

DEV_INIT(block_ramdisk_init)
{
  struct block_ramdisk_context_s	*pv;

  dev->drv = &block_ramdisk_drv;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    goto err;

  pv->params.blk_size = CONFIG_DRIVER_BLOCK_RAMDISK_BLOCKSIZE;
  pv->params.blk_count = CONFIG_DRIVER_BLOCK_RAMDISK_SIZE;

  size_t sz = pv->params.blk_size * pv->params.blk_count;
  dev_block_lba_t c;

  /* if a ramdisk already exists, take its address */
  if (params)
  {
      pv->mem = params;
  } 
  else 
  {
      if ((pv->mem = mem_alloc(sz, MEM_SCOPE_SYS)) == NULL)
          goto err_pv;

      for (c = 0; c < pv->params.blk_count; c++)
          memset(pv->mem + (c * pv->params.blk_size), c & 0xFF, pv->params.blk_size);
  }

  dev->drv_pv = pv;
  return 0;

 err_pv:
  mem_free(pv);
 err:
  return -1;
}

