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

#include <hexo/endian.h>

#include <hexo/types.h>

#include <device/class/block.h>
#include <device/device.h>
#include <device/class/icu.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <arch/mem_checker.h>

#include <stdlib.h>
#include <string.h>

struct soclib_block_context_s
{
  uintptr_t                 addr;
  struct dev_irq_ep_s       irq_ep;
  struct dev_block_params_s params;
  dev_blk_queue_root_t      queue;
};

#define SOCLIB_BLOCK_BUFFER 0
#define SOCLIB_BLOCK_LBA 4
#define SOCLIB_BLOCK_COUNT 8
#define SOCLIB_BLOCK_OP 12
#define SOCLIB_BLOCK_STATUS 16
#define SOCLIB_BLOCK_IRQ_ENABLE 20
#define SOCLIB_BLOCK_SIZE 24
#define SOCLIB_BLOCK_BLOCK_SIZE 28

#define SOCLIB_BLOCK_OP_NOOP 0
#define SOCLIB_BLOCK_OP_READ 1
#define SOCLIB_BLOCK_OP_WRITE 2

#define SOCLIB_BLOCK_STATUS_IDLE 0
#define SOCLIB_BLOCK_STATUS_BUSY 1
#define SOCLIB_BLOCK_STATUS_READ_SUCCESS 2
#define SOCLIB_BLOCK_STATUS_WRITE_SUCCESS 3
#define SOCLIB_BLOCK_STATUS_READ_ERROR 4
#define SOCLIB_BLOCK_STATUS_WRITE_ERROR 5
#define SOCLIB_BLOCK_STATUS_ERROR 6

/**************************************************************/

static void soclib_block_op_start(struct soclib_block_context_s *pv,
				  struct dev_block_rq_s *rq)
{
  //printk("New op: %d %d\n", rq->lba, rq->type);

  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_BUFFER,
                   endian_le32((uint32_t)rq->data[rq->progress]));
  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_LBA,
                   endian_le32(rq->lba + rq->progress));
  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_COUNT,
                   endian_le32(1));

  switch (rq->type & DEV_BLOCK_OPMASK)
    {
    case DEV_BLOCK_READ:
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP, endian_le32(SOCLIB_BLOCK_OP_READ));
      break;
    case DEV_BLOCK_WRITE:
      /* invalidate dcache, force dcache write before dma */
      cpu_dcache_invld_buf(rq->data[rq->progress], pv->params.blk_size);
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP, endian_le32(SOCLIB_BLOCK_OP_WRITE));
      break;
    }
}

/* add a new request in queue and start if idle */
static void soclib_block_rq_start(struct soclib_block_context_s *pv,
				  struct dev_block_rq_s *rq)
{
  bool_t idle = dev_blk_queue_isempty(&pv->queue);

  dev_blk_queue_pushback(&pv->queue, rq);

  if (idle)
    soclib_block_op_start(pv, rq);
}

/* drop current request and start next in queue if any */
static void soclib_block_rq_end(struct soclib_block_context_s *pv)
{
  struct dev_block_rq_s *rq;

  dev_blk_queue_pop(&pv->queue);

  if ((rq = dev_blk_queue_head(&pv->queue)) != NULL)
    soclib_block_op_start(pv, rq);
}

DEVBLOCK_REQUEST(soclib_block_request)
{
  struct device_s *dev = bdev->dev;
  struct soclib_block_context_s *pv = dev->drv_pv;
  struct dev_block_params_s *p = &pv->params;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->lba + rq->count <= p->blk_count)
    {
      switch (rq->type & DEV_BLOCK_OPMASK)
	{
	case DEV_BLOCK_READ:
	case DEV_BLOCK_WRITE:
	  soclib_block_rq_start(pv, rq);
	  break;

	default:
	  rq->progress = -ENOTSUP;
	  rq->callback(rq, 0);
	  break;
	}
    }
  else
    {
      rq->progress = -ERANGE;
      rq->callback(rq, 0);
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

DEVBLOCK_GETPARAMS(soclib_block_getparams)
{
  struct device_s *dev = bdev->dev;
  struct soclib_block_context_s *pv = dev->drv_pv;
  return &pv->params;
}

static DEV_IRQ_EP_PROCESS(soclib_block_irq)
{
  struct device_s *dev = ep->dev;
  struct soclib_block_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_block_rq_s *rq = dev_blk_queue_head(&pv->queue);

  uint32_t st = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_STATUS));

  /* printk("block dev irq %p, st: %x\n", dev, st); */

  switch (st)
    {
    case SOCLIB_BLOCK_STATUS_READ_SUCCESS:
      soclib_mem_mark_initialized(rq->data[rq->progress], pv->params.blk_size);
      /* invalidate dcache after dma */
      cpu_dcache_invld_buf(rq->data[rq->progress], pv->params.blk_size);

    case SOCLIB_BLOCK_STATUS_WRITE_SUCCESS:
      assert(rq != NULL);

      rq->progress++;
      rq->callback(rq, 1);

      if (rq->progress < rq->count)
        soclib_block_op_start(pv, rq);
      else
        soclib_block_rq_end(pv);
      break;

    case SOCLIB_BLOCK_STATUS_ERROR:
    case SOCLIB_BLOCK_STATUS_READ_ERROR:
    case SOCLIB_BLOCK_STATUS_WRITE_ERROR:
      assert(rq != NULL);

      rq->progress = -EIO;
      rq->callback(rq, 0);
      soclib_block_rq_end(pv);
      break;
    }

  lock_release(&dev->lock);
}

static const struct devenum_ident_s	soclib_block_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("soclib:vci_block_device"),
	{ 0 }
};

static const struct driver_block_s	soclib_block_block_drv =
{
  .class_		= DRIVER_CLASS_BLOCK,
  .f_request		= soclib_block_request,
  .f_getparams          = soclib_block_getparams,
};

static DEV_INIT(soclib_block_init);
static DEV_CLEANUP(soclib_block_cleanup);

const struct driver_s	soclib_block_drv =
{
  .desc                 = "SoCLib VciBlockDevice",
  .id_table		= soclib_block_ids,
  .f_init		= soclib_block_init,
  .f_cleanup		= soclib_block_cleanup,
  .classes              = { &soclib_block_block_drv, 0 }
};

REGISTER_DRIVER(soclib_block_drv);

static DEV_INIT(soclib_block_init)
{
  struct soclib_block_context_s	*pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  device_irq_source_init(dev, &pv->irq_ep, 1, &soclib_block_irq);

  dev_blk_queue_init(&pv->queue);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_q;

  pv->params.blk_size = endian_le32(
      cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_BLOCK_SIZE));
  pv->params.blk_count = endian_le32(
      cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_SIZE));

  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_IRQ_ENABLE,
                   endian_le32(1));

  printk("Soclib block device : %u sectors %u bytes per block\n",
	 pv->params.blk_count, pv->params.blk_size);

  dev->drv = &soclib_block_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_q:
  dev_blk_queue_destroy(&pv->queue);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_block_cleanup)
{
  struct soclib_block_context_s	*pv = dev->drv_pv;

  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_IRQ_ENABLE, 0);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  dev_blk_queue_destroy(&pv->queue);

  mem_free(pv);
}

