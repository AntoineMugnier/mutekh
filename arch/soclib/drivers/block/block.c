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

#include <arch/soclib/mem_checker.h>

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


DRIVER_PV(struct soclib_block_context_s
{
  uintptr_t                 addr;
  struct dev_irq_src_s      irq_ep;
  dev_request_queue_root_t  queue;

  uint_fast8_t              blk_log2;
  uint32_t                  blk_count;
  enum soclib_block_type_e  memtype;

  void                      *data;
  size_t                    size;

  uint8_t                   progress;
  bool_t                    running;
  enum dev_mem_rq_type_e    rqtype;
});

/**************************************************************/

static error_t soclib_block_op_start(struct soclib_block_context_s *pv,
                                     struct dev_mem_rq_s *rq)
{
  if (pv->rqtype & _DEV_MEM_PAGE)
    {
      if (rq->page.page_log2 < pv->blk_log2)
        return -ERANGE;  /* unsupported page size */

      if (!rq->page.sc_count)
        return -EAGAIN;

      uint_fast8_t i = pv->progress;
      pv->data = rq->page.sc[i].data;
      pv->size = 1 << rq->page.page_log2;

      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_LBA,
                       endian_le32(rq->page.sc[i].addr >> pv->blk_log2));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_BUFFER,
                       endian_le32((uint32_t)pv->data));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_COUNT,
                       endian_le32(pv->size >> pv->blk_log2));

      switch (pv->rqtype & _DEV_MEM_OP_MASK)
        {
        case _DEV_MEM_READ:
          cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                           endian_le32(SOCLIB_BLOCK_OP_READ));
          return 0;
        case _DEV_MEM_WRITE:
          cpu_dcache_invld_buf(pv->data, pv->size);
          cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                           endian_le32(SOCLIB_BLOCK_OP_WRITE));
          return 0;
        case _DEV_MEM_ERASE:
          cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                           endian_le32(SOCLIB_BLOCK_OP_ERASE));
          return 0;
        }
    }
  else if (pv->rqtype & _DEV_MEM_PARTIAL)
    {
      if (rq->partial.addr + rq->partial.size
          > (uint64_t)pv->blk_count << pv->blk_log2)
        return -ERANGE;

      if (!rq->partial.size)
        return -EAGAIN;

      if (rq->partial.addr >> pv->blk_log2
          != (rq->partial.addr + rq->partial.size - 1) >> pv->blk_log2)
        return -ERANGE;  /* partial access crosses page boundary */

      pv->data = rq->partial.data;
      pv->size = rq->partial.size;

      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_LBA,
                       endian_le32((rq->partial.addr >> pv->blk_log2)));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_COUNT,
                       endian_le32(pv->size));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_BUFFER,
                       endian_le32((uint32_t)pv->data));
      cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OFFSET,
                       endian_le32((uint32_t)rq->partial.addr & bit_mask(0, pv->blk_log2)));

      switch (pv->rqtype & _DEV_MEM_OP_MASK)
        {
        case _DEV_MEM_READ:
          cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                           endian_le32(SOCLIB_BLOCK_OP_PARTIAL_READ));
          return 0;
        case _DEV_MEM_WRITE:
          cpu_dcache_invld_buf(pv->data, pv->size);
          cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_OP,
                           endian_le32(SOCLIB_BLOCK_OP_PARTIAL_WRITE));
          return 0;
        }
    }
  else if (!pv->rqtype)
    {
      return -EAGAIN;
    }

  return -ENOTSUP;
}

static void soclib_block_rq_end(struct device_s *dev);

/* add a new request in queue and start if idle */
static void soclib_block_rq_start(struct device_s *dev)
{
  struct soclib_block_context_s *pv = dev->drv_pv;
  struct dev_mem_rq_s *rq = dev_mem_rq_head(&pv->queue);

  assert(!pv->running);

  pv->running = 1;
  pv->rqtype = rq->type & ~(_DEV_MEM_FLUSH | _DEV_MEM_INVAL);
  pv->progress = 0;
  rq->err = 0;

  if (rq->band_mask != 1)
    rq->err = -ENOTSUP;
  else
    rq->err = soclib_block_op_start(pv, rq);

  if (rq->err || !pv->rqtype)
    soclib_block_rq_end(dev);
}

/* drop current request and start next in queue if any */
static void soclib_block_rq_end(struct device_s *dev)
{
  struct soclib_block_context_s *pv = dev->drv_pv;
  struct dev_mem_rq_s *rq = dev_mem_rq_head(&pv->queue);

  if (rq->err == -EAGAIN)
    rq->err = 0;

  dev_mem_rq_pop(&pv->queue);
  dev_mem_rq_done(rq);
  pv->running = 0;

  if (!dev_rq_queue_isempty(&pv->queue))
    soclib_block_rq_start(dev);
}

static DEV_MEM_REQUEST(soclib_block_request)
{
  struct device_s *dev = accessor->dev;
  struct soclib_block_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  dev_mem_rq_pushback(&pv->queue, rq);

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
      info->flags |= DEV_MEM_PAGE_WRITE | DEV_MEM_PAGE_READ;
    case SOCLIB_BLOCK_TYPE_RODISK:
      info->type = DEV_MEM_DISK;
      break;
    case SOCLIB_BLOCK_TYPE_NOR_FLASH:
      info->type = DEV_MEM_FLASH;
      info->erase_log2 = info->page_log2;
      info->flags |= DEV_MEM_PAGE_WRITE | DEV_MEM_PAGE_READ | DEV_MEM_ERASE_ONE |
        DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ;
      break;
    }

  return 0;
}

static DEV_IRQ_SRC_PROCESS(soclib_block_irq)
{
  struct device_s *dev = ep->base.dev;
  struct soclib_block_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_mem_rq_s *rq = dev_mem_rq_head(&pv->queue);

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

      if ((rq->type & _DEV_MEM_PAGE) &&
          ++pv->progress < rq->page.sc_count)
        {
          rq->err = soclib_block_op_start(pv, rq);
          if (!rq->err)
            break;
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


#define soclib_block_use dev_use_generic

static DEV_INIT(soclib_block_init)
{
  struct soclib_block_context_s	*pv;


  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &soclib_block_irq);

  dev_rq_queue_init(&pv->queue);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_q;

  pv->blk_count = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_SIZE));
  pv->blk_log2 = ffs(endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_BLOCK_SIZE))) - 1;
  pv->memtype = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_BLOCK_TYPE));

  pv->running = 0;

  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_IRQ_ENABLE,
                   endian_le32(1));


  return 0;

 err_q:
  dev_rq_queue_destroy(&pv->queue);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_block_cleanup)
{
  struct soclib_block_context_s	*pv = dev->drv_pv;

  if (pv->running)
    return -EBUSY;

  cpu_mem_write_32(pv->addr + SOCLIB_BLOCK_IRQ_ENABLE, 0);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  dev_rq_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(soclib_block_drv, 0, "Soclib Block Device", soclib_block,
               DRIVER_MEM_METHODS(soclib_block));

DRIVER_REGISTER(soclib_block_drv,
                DEV_ENUM_FDTNAME_ENTRY("soclib:block_device"));

