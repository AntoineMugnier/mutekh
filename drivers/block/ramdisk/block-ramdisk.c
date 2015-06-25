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

#include <device/class/block.h>
#include <device/device.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <stdlib.h>
#include <string.h>

#include "block-ramdisk.h"
#include "block-ramdisk-private.h"

/**************************************************************/

DEV_BLOCK_REQUEST(block_ramdisk_request)
{
  struct block_ramdisk_context_s *pv = dev->drv_pv;
  struct dev_block_params_s *p = &pv->params;
  dev_block_lba_t lba = rq->lba + rq->progress;
  dev_block_lba_t count = rq->count - rq->progress;

  if (lba + count > p->blk_count)
    {
      rq->progress = -ERANGE;
      rq->callback(rq, 0, rq + 1);
      return;
    }

  lock_spin(&dev->lock);

  size_t b;

  switch (rq->type & DEV_BLOCK_OPMASK)
    {
    case DEV_BLOCK_READ:
      for (b = 0; b < count; b++)
	memcpy(rq->data[b], pv->mem + ((lba + b) * p->blk_size), p->blk_size);

      rq->progress += count;
      rq->callback(rq, count, rq + 1);
      break;

    case DEV_BLOCK_WRITE:
      for (b = 0; b < count; b++)
	memcpy(pv->mem + ((lba + b) * p->blk_size), rq->data[b], p->blk_size);

      rq->progress += count;
      rq->callback(rq, count, rq + 1);
      break;

    default:
      rq->progress = -ENOTSUP;
      rq->callback(rq, 0, rq + 1);
      break;
    }

  lock_release(&dev->lock);
}

DEV_BLOCK_GETPARAMS(block_ramdisk_getparams)
{
  return &(((struct block_ramdisk_context_s *)(dev->drv_pv))->params);
}

DEV_BLOCK_GETRQSIZE(block_ramdisk_getrqsize)
{
  return sizeof(struct dev_block_rq_s);
}

DEV_CLEANUP(block_ramdisk_cleanup)
{
  struct block_ramdisk_context_s	*pv = dev->drv_pv;

  if (! dev->addr[0])
	  mem_free(pv->mem);
  mem_free(pv);
}

#define block_ramdisk_use dev_use_generic

DRIVER_DECLARE(block_ramdisk_drv, "RamDisk", block_ramdisk,
               DRIVER_MEM_METHODS(block_ramdisk));

DRIVER_REGISTER(block_ramdisk_drv,
                DEV_ENUM_FDTNAME_ENTRY("ramdisk", 0, 0));

DEV_INIT(block_ramdisk_init)
{
  struct block_ramdisk_context_s	*pv;

  dev->drv = &block_ramdisk_drv;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));

  if (!pv)
    goto err;

  pv->params.blk_size = CONFIG_DRIVER_BLOCK_RAMDISK_BLOCKSIZE;
  pv->params.blk_count = CONFIG_DRIVER_BLOCK_RAMDISK_SIZE;

  size_t sz = pv->params.blk_size * pv->params.blk_count;
  dev_block_lba_t c;

  /* if a ramdisk already exists, take its address */
  if (dev->addr[0])
  {
      pv->mem = dev->addr[0];
  } 
  else 
  {
      if ((pv->mem = mem_alloc(sz, (mem_scope_sys))) == NULL)
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

