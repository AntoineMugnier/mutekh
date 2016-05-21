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

DRIVER_PV(struct soclib_dma_context_s
{
  /* dma input request queue and dma fifo */
  dev_request_queue_root_t		queue;
  struct dev_irq_src_s        irq_ep;
  uintptr_t                   addr;
  bool_t                      busy;
});

static void soclib_dma_start(struct device_s *dev, const struct dev_dma_rq_s *rq)
{
  struct soclib_dma_context_s	*pv = dev->drv_pv;

  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_NOIRQ, 0);
  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_SRC, endian_le32((uintptr_t)rq->basic.src));
  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_DST, endian_le32((uintptr_t)rq->basic.dst));
  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_LEN, endian_le32(rq->basic.size));
}

static bool_t soclib_dma_validate_request(struct dev_dma_rq_s *rq)
{
  if (rq->type != DEV_DMA_BASIC)
    {
      rq->error = -ENOTSUP;
      return 1;
    }

  if ((rq->param[0].src_inc != DEV_DMA_INC_1_BYTE) ||
      (rq->param[0].dst_inc != DEV_DMA_INC_1_BYTE) ||
      (rq->param[0].channel) ||
       rq->param[0].const_data)
    {
      rq->error = -ENOTSUP;
      return 1;
    }

  assert(rq->basic.size);
  
  return 0;
}

static DEVDMA_REQUEST(soclib_dma_request)
{
  struct device_s             *dev = accessor->dev;
  struct soclib_dma_context_s	*pv = dev->drv_pv;


  LOCK_SPIN_IRQ(&dev->lock);

  req->error = 0;

  if (soclib_dma_validate_request(req))
    goto end;

  bool_t empty = dev_request_queue_isempty(&pv->queue);

  dev_request_queue_pushback(&pv->queue, &req->base);

  if (empty && !pv->busy)
    soclib_dma_start(dev, req);

end:

  LOCK_RELEASE_IRQ(&dev->lock);

  if (req->error)
    kroutine_exec(&req->base.kr);
}

static DEV_IRQ_SRC_PROCESS(soclib_dma_irq)
{
  struct device_s *dev = ep->base.dev;
  struct soclib_dma_context_s *pv = dev->drv_pv;
  struct dev_dma_rq_s *rq;

  lock_spin(&dev->lock);

  struct dev_request_s * base = dev_request_queue_head(&pv->queue);

  dev_request_queue_pop(&pv->queue);

  pv->busy = 1;

  lock_release(&dev->lock);

  kroutine_exec(&base->kr);

  lock_spin(&dev->lock);

  pv->busy = 0;

  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_RESET, 0);

  if ((rq = dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue))))
    soclib_dma_start(dev, rq);

  lock_release(&dev->lock);
}

/* 
 * device open operation
 */

#define soclib_dma_use dev_use_generic

static DEV_INIT(soclib_dma_init)
{
  struct soclib_dma_context_s	*pv;


  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  dev_request_queue_init(&pv->queue);

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  device_irq_source_init(dev, &pv->irq_ep, 1, &soclib_dma_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem;

  cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_RESET, 0);


  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_dma_cleanup)
{
  struct soclib_dma_context_s	*pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue) || pv->busy)
    return -EBUSY;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(soclib_dma_drv, 0, "Soclib Dma", soclib_dma,
               DRIVER_DMA_METHODS(soclib_dma));

DRIVER_REGISTER(soclib_dma_drv,
                DEV_ENUM_FDTNAME_ENTRY("soclib:dma"));

