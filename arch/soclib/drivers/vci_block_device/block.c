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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <hexo/endian.h>

#include <hexo/types.h>

#include <device/class/mem.h>
#include <device/device.h>
#include <device/resources.h>
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

#define SOCLIB_BLOCK_BUFFER 0
#define SOCLIB_BLOCK_LBA 4
#define SOCLIB_BLOCK_COUNT 8
#define SOCLIB_BLOCK_OP 12
#define SOCLIB_BLOCK_STATUS 16
#define SOCLIB_BLOCK_IRQ_ENABLE 20
#define SOCLIB_BLOCK_SIZE 24
#define SOCLIB_BLOCK_BLOCK_SIZE 28
#define SOCLIB_BLOCK_TYPE 32
#define SOCLIB_BLOCK_OFFSET 36

enum soclib_block_ops_e
{
  SOCLIB_BLOCK_OP_NOOP,
  SOCLIB_BLOCK_OP_READ,
  SOCLIB_BLOCK_OP_WRITE,
  SOCLIB_BLOCK_OP_PARTIAL_READ,
  SOCLIB_BLOCK_OP_PARTIAL_WRITE,
  SOCLIB_BLOCK_OP_ERASE,
};

enum soclib_block_status_e
{
  SOCLIB_BLOCK_STATUS_IDLE,
  SOCLIB_BLOCK_STATUS_BUSY,
  SOCLIB_BLOCK_STATUS_READ_SUCCESS,
  SOCLIB_BLOCK_STATUS_WRITE_SUCCESS,
  SOCLIB_BLOCK_STATUS_READ_ERROR,
  SOCLIB_BLOCK_STATUS_WRITE_ERROR,
  SOCLIB_BLOCK_STATUS_ERROR,
  SOCLIB_BLOCK_STATUS_ERASE_SUCCESS,
  SOCLIB_BLOCK_STATUS_ERASE_ERROR,
};

enum soclib_block_type_e
{
  SOCLIB_BLOCK_TYPE_DISK,
  SOCLIB_BLOCK_TYPE_RODISK,
  SOCLIB_BLOCK_TYPE_NOR_FLASH,
};


struct soclib_block_context_s
{
  uintptr_t                 addr;
  struct dev_irq_ep_s       irq_ep;
  dev_request_queue_root_t  queue;

  uint_fast8_t              blk_log2;
  uint32_t                  blk_count;
  enum soclib_block_type_e  memtype;

  void                      *data;
  size_t                    size;

  size_t                    progress;
  bool_t                    running;
  enum dev_mem_rq_type_e    rqtype;
};

/**************************************************************/

static void soclib_block_op_start(struct soclib_block_context_s *pv,
				  struct dev_mem_rq_s *rq)
{
  if (pv->rqtype & DEV_MEM_OP_PAGE_ERASE)
    {
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_LBA,
                       endian_le32((rq->addr >> pv->blk_log2)));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_COUNT,
                       endian_le32(rq->size));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                       endian_le32(SOCLIB_BLOCK_OP_ERASE));
    }
  else if (pv->rqtype & (DEV_MEM_OP_PAGE_READ | DEV_MEM_OP_PAGE_WRITE))
    { 
      pv->size = 1 << (pv->blk_log2 + rq->sc_log2);
      pv->data = rq->sc_data[pv->progress];

      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_LBA,
                       endian_le32((rq->addr >> pv->blk_log2) +
                                   (pv->progress << rq->sc_log2)));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_BUFFER,
                       endian_le32((uint32_t)pv->data));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_COUNT,
                       endian_le32(1 << rq->sc_log2));

      if (pv->rqtype & DEV_MEM_OP_PAGE_WRITE)
        {
          cpu_dcache_invld_buf(pv->data, pv->size);
          cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                           endian_le32(SOCLIB_BLOCK_OP_WRITE));
        }
      else /* if (pv->rqtype & DEV_MEM_OP_PAGE_READ) */
        {
          cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                           endian_le32(SOCLIB_BLOCK_OP_READ));
        }
    }
  else if (pv->rqtype & (DEV_MEM_OP_PARTIAL_WRITE | DEV_MEM_OP_PARTIAL_READ))
    {
      pv->size = rq->size;
      pv->data = rq->data;

      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_LBA,
                       endian_le32((rq->addr >> pv->blk_log2)));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_COUNT,
                       endian_le32(rq->size));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_BUFFER,
                       endian_le32((uint32_t)pv->data));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OFFSET,
                       endian_le32((uint32_t)rq->addr & ((1 << pv->blk_log2) - 1)));
      cpu_dcache_invld_buf(rq->data, rq->size);
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                       endian_le32(pv->rqtype & DEV_MEM_OP_PARTIAL_WRITE
                                   ? SOCLIB_BLOCK_OP_PARTIAL_WRITE
                                   : SOCLIB_BLOCK_OP_PARTIAL_READ));
    }
}

static void soclib_block_rq_end(struct device_s *dev);

/* add a new request in queue and start if idle */
static void soclib_block_rq_start(struct device_s *dev)
{
  struct soclib_block_context_s *pv = dev->drv_pv;
  struct dev_mem_rq_s *rq = dev_mem_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (!pv->running)
    {
      pv->running = 1;
      pv->rqtype = rq->type & (DEV_MEM_OP_PAGE_READ | DEV_MEM_OP_PAGE_WRITE |
                               DEV_MEM_OP_PARTIAL_READ | DEV_MEM_OP_PARTIAL_WRITE |
                               DEV_MEM_OP_PAGE_ERASE);
      rq->err = 0;

      if (rq->band_mask & 0xfe)
        rq->err = -ENOTSUP;
      else if (rq->type & (DEV_MEM_OP_PAGE_ERASE | DEV_MEM_OP_PAGE_READ | DEV_MEM_OP_PAGE_WRITE))
        {
          if (rq->addr & ((1 << pv->blk_log2) - 1))
            rq->err = -EINVAL;  /* address not page aligned */
          else if (rq->size & ((1 << rq->sc_log2) - 1))
            rq->err = -EINVAL;  /* total count not aligned on scattered count */
          else if ((rq->addr >> pv->blk_log2) + rq->size > pv->blk_count)
            rq->err = -ERANGE;
        }
      else if (rq->type & (DEV_MEM_OP_PARTIAL_WRITE | DEV_MEM_OP_PARTIAL_READ |
                           DEV_MEM_OP_CACHE_INVALIDATE | DEV_MEM_OP_CACHE_FLUSH))
        {
          if (rq->type & (DEV_MEM_OP_PARTIAL_WRITE | DEV_MEM_OP_PARTIAL_READ))
            {
              if (rq->addr >> pv->blk_log2 != (rq->addr + rq->size - 1) >> pv->blk_log2)
                rq->err = -EINVAL;  /* partial access crosses page boundary */
            }
          else if (rq->addr + rq->size > (uint64_t)pv->blk_count << pv->blk_log2)
            rq->err = -ERANGE;
        }
      else
        {
          rq->err = -ENOTSUP;
        }
    }

  pv->progress = 0;

  if (!rq->err && (rq->band_mask & 1) && pv->rqtype && rq->size)
    soclib_block_op_start(pv, rq);
  else
    soclib_block_rq_end(dev);
}

/* drop current request and start next in queue if any */
static void soclib_block_rq_end(struct device_s *dev)
{
  struct soclib_block_context_s *pv = dev->drv_pv;
  struct dev_mem_rq_s *rq = dev_mem_rq_s_cast(dev_request_queue_head(&pv->queue));

  pv->rqtype = pv->rqtype & (pv->rqtype - 1);

  if (rq->err || !pv->rqtype)
    {
      dev_request_queue_pop(&pv->queue);
      lock_release(&dev->lock);
      kroutine_exec(&rq->base.kr, 0);
      lock_spin(&dev->lock);
      pv->running = 0;
    }

  if (!dev_request_queue_isempty(&pv->queue))
    soclib_block_rq_start(dev);
}

static DEV_MEM_REQUEST(soclib_block_request)
{
  struct device_s *dev = accessor->dev;
  struct soclib_block_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  dev_request_queue_pushback(&pv->queue, dev_mem_rq_s_base(rq));

  if (!pv->running)
    soclib_block_rq_start(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_MEM_INFO(soclib_block_info)
{
  struct device_s *dev = accessor->dev;
  struct soclib_block_context_s	*pv = dev->drv_pv;

  if (band_index > 0 || accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  info->size = pv->blk_count;
  info->page_log2 = pv->blk_log2;
  switch (pv->memtype)
    {
    case SOCLIB_BLOCK_TYPE_DISK:
      info->flags |= DEV_MEM_WRITABLE;
    case SOCLIB_BLOCK_TYPE_RODISK:
      info->type = DEV_MEM_DISK;
      break;
    case SOCLIB_BLOCK_TYPE_NOR_FLASH:
      info->type = DEV_MEM_FLASH;
      info->erase_log2 = info->page_log2;
      info->flags |= DEV_MEM_ERASE_ONE | DEV_MEM_WRITABLE |
        DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ;
      break;
    }

  return 0;
}

static DEV_IRQ_EP_PROCESS(soclib_block_irq)
{
  struct device_s *dev = ep->dev;
  struct soclib_block_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_mem_rq_s *rq = dev_mem_rq_s_cast(dev_request_queue_head(&pv->queue));

  enum soclib_block_status_e st = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_STATUS));

  switch (st)
    {
    case SOCLIB_BLOCK_STATUS_READ_SUCCESS:
      /* invalidate dcache after dma */
      cpu_dcache_invld_buf(pv->data, pv->size);
      soclib_mem_mark_initialized(pv->data, pv->size);

    case SOCLIB_BLOCK_STATUS_ERASE_SUCCESS:
    case SOCLIB_BLOCK_STATUS_WRITE_SUCCESS:
      assert(rq != NULL);

      if (rq->type & (DEV_MEM_OP_PAGE_READ | DEV_MEM_OP_PAGE_WRITE))
        {
          if ((++pv->progress << rq->sc_log2) < rq->size)
            {
              soclib_block_op_start(pv, rq);
              break;
            }
        }

      soclib_block_rq_end(dev);
      break;

    case SOCLIB_BLOCK_STATUS_ERROR:
    case SOCLIB_BLOCK_STATUS_READ_ERROR:
    case SOCLIB_BLOCK_STATUS_WRITE_ERROR:
    case SOCLIB_BLOCK_STATUS_ERASE_ERROR:
      assert(rq != NULL);

      rq->err = -EIO;
      soclib_block_rq_end(dev);
      break;

    case SOCLIB_BLOCK_STATUS_IDLE:
    case SOCLIB_BLOCK_STATUS_BUSY:
      break;
    }

  lock_release(&dev->lock);
}

static const struct dev_enum_ident_s	soclib_block_ids[] =
{
  DEV_ENUM_FDTNAME_ENTRY("soclib:vci_block_device"),
  { 0 }
};

static const struct driver_mem_s	soclib_block_mem_drv =
{
  .class_		= DRIVER_CLASS_MEM,
  .f_info               = soclib_block_info,
  .f_request		= soclib_block_request,
};

static DEV_INIT(soclib_block_init);
static DEV_CLEANUP(soclib_block_cleanup);

const struct driver_s	soclib_block_drv =
{
  .desc                 = "SoCLib VciBlockDevice",
  .id_table		= soclib_block_ids,
  .f_init		= soclib_block_init,
  .f_cleanup		= soclib_block_cleanup,
  .classes              = { &soclib_block_mem_drv, 0 }
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

  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &soclib_block_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  dev_request_queue_init(&pv->queue);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_q;

  pv->blk_count = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_SIZE));
  pv->blk_log2 = ffs(endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_BLOCK_SIZE))) - 1;
  pv->memtype = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_TYPE));

  pv->running = 0;

  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_IRQ_ENABLE,
                   endian_le32(1));

  dev->drv = &soclib_block_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_q:
  dev_request_queue_destroy(&pv->queue);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_block_cleanup)
{
  struct soclib_block_context_s	*pv = dev->drv_pv;

  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_IRQ_ENABLE, 0);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);
}

