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
#include <hexo/device.h>
#include <device/driver.h>

#include <hexo/alloc.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <arch/hexo/emu_syscalls.h>

#include "block-file-emu.h"
#include "block-file-emu-private.h"

/**************************************************************/

DEVBLOCK_REQUEST(block_file_emu_request)
{
  struct block_file_emu_context_s *pv = dev->drv_pv;
  struct dev_block_params_s *p = &pv->params;
  dev_block_lba_t lba = rq->lba;
  dev_block_lba_t count = rq->count;

  if (lba + count <= p->blk_count)
    {
      reg_t id;
      size_t b;

      emu_do_syscall(EMU_SYSCALL_LSEEK, 3, pv->fd, lba * p->blk_size, EMU_SEEK_SET);

      switch (rq->type)
	{
	case DEV_BLOCK_READ:
	  id = EMU_SYSCALL_READ;
	  break;
	case DEV_BLOCK_WRITE:
	  id = EMU_SYSCALL_WRITE;
	  break;
	}

      for (b = 0; b < count; b++)
	emu_do_syscall(id, 3, pv->fd, rq->data[b], p->blk_size);

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
}

/* 
 * device params
 */

DEVBLOCK_GETPARAMS(block_file_emu_getparams)
{
  return &(((struct block_file_emu_context_s *)(dev->drv_pv))->params);
}

/* 
 * device close operation
 */

DEV_CLEANUP(block_file_emu_cleanup)
{
  struct block_file_emu_context_s	*pv = dev->drv_pv;

  emu_do_syscall(EMU_SYSCALL_CLOSE, 1, pv->fd);
  mem_free(pv);
}

/* 
 * device open operation
 */

const struct driver_s	block_file_emu_drv =
{
  .class		= device_class_block,
  .f_init		= block_file_emu_init,
  .f_cleanup		= block_file_emu_cleanup,
  .f.blk = {
    .f_request		= block_file_emu_request,
    .f_getparams	= block_file_emu_getparams,
  }
};

DEV_INIT(block_file_emu_init)
{
  struct block_file_emu_context_s	*pv;

  dev->drv = &block_file_emu_drv;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    goto err;

  assert(params);

  pv->fd = emu_do_syscall(EMU_SYSCALL_OPEN, 2, params, EMU_O_RDONLY);

  if (pv->fd < 0)
    {
      printk("Unable to open device file %s\n", params);
      goto err_pv;
    }

  size_t off = emu_do_syscall(EMU_SYSCALL_LSEEK, 3, pv->fd, 0, EMU_SEEK_END);

  pv->params.blk_size = CONFIG_DRIVER_BLOCK_EMU_BLOCKSIZE;
  pv->params.blk_count = off / CONFIG_DRIVER_BLOCK_EMU_BLOCKSIZE;

  printk("Emu block device : %u sectors\n", pv->params.blk_count);

  dev->drv_pv = pv;
  return 0;

 err_pv:
  mem_free(pv);
 err:
  return -1;
}

