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

#include <hexo/device/block.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <stdlib.h>

#include "block-ramdisk.h"
#include "block-ramdisk-private.h"

/**************************************************************/

/* 
 * device read operation
 */

DEVBLOCK_READ(block_ramdisk_read)
{
  struct block_ramdisk_context_s *pv = dev->drv_pv;
  struct dev_block_params_s *p = &pv->params;
  dev_block_lba_t lba = rq->lba;

  if (lba < p->blk_count)
    {
      dev_block_lba_t count = rq->count;
      size_t c;
      uint8_t *data[count];
      dev_block_lba_t b;

      if (lba + count > p->blk_count)
	{
	  c = p->blk_count - lba;
	  rq->error = EEOF;
	}
      else
	c = count;

      for (b = 0; b < c; b++)
	data[b] = pv->mem + ((lba + b) << p->blk_sh_size);

      rq->count -= c;
      rq->lba += c;
      rq->data = data;
      rq->callback(dev, rq, c);
    }
  else
    {
      rq->error = ERANGE;
      rq->callback(dev, rq, 0);
    }
}

/* 
 * device write operation
 */

DEVBLOCK_WRITE(block_ramdisk_write)
{
  struct block_ramdisk_context_s *pv = dev->drv_pv;
  struct dev_block_params_s *p = &pv->params;
  dev_block_lba_t lba = rq->lba;

  if (lba < p->blk_count)
    {
      dev_block_lba_t count = rq->count;
      size_t c;
      dev_block_lba_t b;

      if (lba + count > p->blk_count)
	{
	  c = p->blk_count - lba;
	  rq->error = EEOF;
	}
      else
	c = count;


      for (b = 0; b < c; b++)
	memcpy(pv->mem + ((lba + b) << p->blk_sh_size), rq->data[b], p->blk_size);

      rq->count -= c;
      rq->lba += c;
      rq->callback(dev, rq, c);
    }
  else
    {
      rq->error = ERANGE;
      rq->callback(dev, rq, 0);
    }
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

  lock_destroy(&pv->lock);
  mem_free(pv->mem);
  mem_free(pv);
}

/*
 * IRQ handler
 */

DEV_IRQ(block_ramdisk_irq)
{
  return 1;
}

/* 
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	block_ramdisk_drv =
{
  .f_init		= block_ramdisk_init,
  .f_cleanup		= block_ramdisk_cleanup,
  .f_irq		= block_ramdisk_irq,
  .f.blk = {
    .f_read		= block_ramdisk_read,
    .f_write		= block_ramdisk_write,
    .f_getparams	= block_ramdisk_getparams,
  }
};
#endif

DEV_INIT(block_ramdisk_init)
{
  struct block_ramdisk_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &block_ramdisk_drv;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  pv->params.blk_size = 512;
  pv->params.blk_sh_size = 9;
  pv->params.blk_count = 2048 * 4; // 4 Mo. Ramdisk

  {
    size_t sz = pv->params.blk_size * pv->params.blk_count;
    dev_block_lba_t c;

    pv->mem = mem_alloc(sz, MEM_SCOPE_SYS);
    if (!pv->mem)
      {
	mem_free(pv);
	return -1;
      }
    for (c = 0; c < pv->params.blk_count; c++)
      memset(pv->mem + (c << pv->params.blk_sh_size), c & 0xFF, pv->params.blk_size);
  }
  lock_init(&pv->lock);

  dev->drv_pv = pv;
  return 0;
}
