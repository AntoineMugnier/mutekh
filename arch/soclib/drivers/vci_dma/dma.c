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
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/class/dma.h>

#define TTY_SOCLIB_REG_SRC	0
#define TTY_SOCLIB_REG_DST	4
#define TTY_SOCLIB_REG_LEN	8
#define TTY_SOCLIB_REG_RESET	12
#define TTY_SOCLIB_REG_NOIRQ	16

struct dma_soclib_context_s
{
  /* dma input request queue and dma fifo */
  dev_dma_queue_root_t		queue;
  struct dev_irq_ep_s           irq_ep;
  uintptr_t                     addr;
};

static void dma_soclib_start(struct device_s *dev, const struct dev_dma_rq_s *rq)
{
  struct dma_soclib_context_s	*pv = dev->drv_pv;

  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_NOIRQ, 0);
  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_SRC, endian_le32((uintptr_t)rq->src));
  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_DST, endian_le32((uintptr_t)rq->dst));
  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_LEN, endian_le32(rq->size));
}

DEVDMA_REQUEST(dma_soclib_request)
{
  struct device_s               *dev = ddev->dev;
  struct dma_soclib_context_s	*pv = dev->drv_pv;

  if (rq->flags)
    return -ENOTSUP;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  rq->ddev = ddev;

  bool_t empty = dev_dma_queue_isempty(&pv->queue);

  dev_dma_queue_pushback(&pv->queue, rq);

  if (empty)
    dma_soclib_start(dev, rq);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_IRQ_EP_PROCESS(dma_soclib_irq)
{
  struct device_s *dev = ep->dev;
  struct dma_soclib_context_s *pv = dev->drv_pv;
  struct dev_dma_rq_s *rq;

  lock_spin(&dev->lock);

  rq = dev_dma_queue_head(&pv->queue);

  rq->callback(rq);
  dev_dma_queue_pop(&pv->queue);

  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_RESET, 0);

  if ((rq = dev_dma_queue_head(&pv->queue)))
    dma_soclib_start(dev, rq);

  lock_release(&dev->lock);
}

/* 
 * device open operation
 */

static const struct devenum_ident_s	dma_soclib_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("soclib:vci_dma"),
	{ 0 }
};

static const struct driver_dma_s	dma_soclib_dma_drv =
{
  .class_		= DRIVER_CLASS_DMA,
  .f_request		= dma_soclib_request,
};

static DEV_INIT(dma_soclib_init);
static DEV_CLEANUP(dma_soclib_cleanup);

const struct driver_s	dma_soclib_drv =
{
  .desc       = "SoCLib VciDma",
  .id_table		= dma_soclib_ids,
  .f_init		  = dma_soclib_init,
  .f_cleanup	= dma_soclib_cleanup,
  .classes    = { &dma_soclib_dma_drv, 0 }
};

REGISTER_DRIVER(dma_soclib_drv);

static DEV_INIT(dma_soclib_init)
{
  struct dma_soclib_context_s	*pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  dev_dma_queue_init(&pv->queue);

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &dma_soclib_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem;

  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_RESET, 0);

  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv = &dma_soclib_drv;

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(dma_soclib_cleanup)
{
  struct dma_soclib_context_s	*pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  dev_dma_queue_destroy(&pv->queue);

  mem_free(pv);
}
