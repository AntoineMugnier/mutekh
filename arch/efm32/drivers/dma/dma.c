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

    Copyright (c) 2013 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/dma.h>
#include <device/class/clock.h>

#include <cpu/pl230_dma.h>
#include <cpu/pl230_channel.h>
#include <arch/efm32_dma_request.h>
#include <arch/efm32_dma.h>

#define CHANNEL_SIZE 16

struct efm32_dma_context
{
  struct dev_irq_ep_s           irq_ep;
  /* base address of DMA */
  uintptr_t                     addr;
  /* lock for reentrant kroutine */
  uint32_t                      busy;
  dev_request_queue_root_t      queue[CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT];
  /* Primary channel memory region */
  uint32_t  *                   channel;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s    clk_ep;
#endif
};

static void efm32_dma_start(struct device_s *dev, struct dev_dma_rq_s *rq)
{
  uint32_t x, src, dst, len;
  struct efm32_dma_context *pv = dev->drv_pv;

  struct efm32_dev_dma_rq_s *exrq = (struct efm32_dev_dma_rq_s*)rq;

  uintptr_t addr = (uintptr_t)pv->channel + CHANNEL_SIZE * rq->channel;

  assert(rq->channel < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT);


  len = rq->size; 
  if (rq->size > 1024)
    len = 1024; 

  rq->size -= len;

  x = DMA_CHANNEL_CFG_N_MINUS_1(len - 1);

  if (rq->arch_rq)
    {
      DMA_CHANNEL_CFG_CYCLE_CTR_SETVAL(x, exrq->mode);
      DMA_CHANNEL_CFG_R_POWER_SETVAL(x, exrq->arbiter);
    }
  else
    {
      DMA_CHANNEL_CFG_CYCLE_CTR_SET(x, AUTOREQUEST);
      DMA_CHANNEL_CFG_R_POWER_SET(x, AFTER32);
    }

  DMA_CHANNEL_CFG_SRC_INC_SETVAL(x, rq->src_inc);
  DMA_CHANNEL_CFG_DST_INC_SETVAL(x, rq->dst_inc);

  uint8_t size = rq->src_inc; 
  
  if (rq->src_inc == DEV_DMA_INC_NONE)
   {
     if (rq->dst_inc == DEV_DMA_INC_NONE)
       size = DMA_CHANNEL_CFG_SRC_SIZE_BYTE;
     else 
       size = rq->dst_inc;
    }

  DMA_CHANNEL_CFG_SRC_SIZE_SETVAL(x, size);
  DMA_CHANNEL_CFG_DST_SIZE_SETVAL(x, size);

  src = rq->src;

  if (rq->src_inc != DEV_DMA_INC_NONE)
    {
      src += ((len - 1) << rq->src_inc);
      rq->src += len;
    }

  cpu_mem_write_32(addr + DMA_CHANNEL_SRC_DATA_END_ADDR, endian_le32(src));

  dst = rq->dst;

  if (rq->dst_inc != DEV_DMA_INC_NONE)
    {
      dst += ((len - 1) << rq->dst_inc);
      rq->dst += len;
    }
 
  cpu_mem_write_32(addr + DMA_CHANNEL_DST_DATA_END_ADDR, endian_le32(dst));

  cpu_mem_write_32(addr + DMA_CHANNEL_CFG_ADDR, endian_le32(x));
 
  x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IEN_ADDR));
  x |= EFM32_DMA_IEN_DONE(rq->channel);
  cpu_mem_write_32(pv->addr + EFM32_DMA_IEN_ADDR, endian_le32(x));
  
  /* Enable channel */
  x = PL230_DMA_CHENS_CH(rq->channel);
  cpu_mem_write_32(pv->addr + PL230_DMA_CHENS_ADDR, endian_le32(x));

  if (rq->arch_rq)
    /* Peripheral triggering source */
    {
      cpu_mem_write_32(pv->addr + EFM32_DMA_CH_CTRL_ADDR(rq->channel), endian_le32(exrq->trigsrc));
      x = PL230_DMA_CHREQMASKC_CH(rq->channel);
      cpu_mem_write_32(pv->addr + PL230_DMA_CHREQMASKC_ADDR, endian_le32(x));
    }
  else
    /* Software triggering source */
    {
      x = PL230_DMA_CHSWREQ_CH(rq->channel);
      cpu_mem_write_32(pv->addr + PL230_DMA_CHSWREQ_ADDR, endian_le32(x));
    }
}

static DEVDMA_REQUEST(efm32_dma_request)
{
  struct device_s         *dev = accessor->dev;
  struct efm32_dma_context	*pv = dev->drv_pv;

  struct efm32_dev_dma_rq_s *exrq = (struct efm32_dev_dma_rq_s *)req;
  struct dev_request_s *base = &req->base;

  LOCK_SPIN_IRQ(&dev->lock);

  req->err = 0;

  while(1)
    {
      if (exrq->rq.const_data || !exrq->rq.size)
        {
          req->err = -ENOTSUP;
          goto end;
        }

      if (!exrq->rq.arch_rq || exrq->lrq == NULL)
        {
          exrq = (struct efm32_dev_dma_rq_s *)req;
          break;
        }

      exrq = exrq->lrq;
    }

  while(1)
    {
      bool_t empty = dev_request_queue_isempty(&pv->queue[exrq->rq.channel]);

      dev_request_queue_pushback(&pv->queue[exrq->rq.channel], &exrq->rq.base);

      if (!empty || (pv->busy & (1 << exrq->rq.channel)))
        break;

      efm32_dma_start(dev, &exrq->rq);
  
      if (!exrq->rq.arch_rq || exrq->lrq == NULL)
        break;

      exrq = exrq->lrq;
    }

end:

  LOCK_RELEASE_IRQ(&dev->lock);

  if (req->err)
    kroutine_exec(&base->kr, cpu_is_interruptible());
}

static DEV_IRQ_EP_PROCESS(efm32_dma_irq)
{
  struct device_s *dev = ep->dev;
  struct efm32_dma_context *pv = dev->drv_pv;

  struct dev_dma_rq_s *rq; 
  struct efm32_dev_dma_rq_s *exrq; 

  lock_spin(&dev->lock);

  while(1)
  {
    uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IF_ADDR));

    if (!x)
      break;
 
    for (uint16_t i = 0; i < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT; i++) 

      {
        if (!(x & EFM32_DMA_IF_DONE(i)))
          continue;

        /* Clear Interrupt Flag */

        cpu_mem_write_32(pv->addr + EFM32_DMA_IFC_ADDR, endian_le32(EFM32_DMA_IF_DONE(i)));
       
        rq =  dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[i]));
        exrq = (struct efm32_dev_dma_rq_s *)rq;

        assert(rq != NULL && !rq->size);
        
        /* End of request */

        dev_request_queue_pop(&pv->queue[rq->channel]);
   
        pv->busy = 1 << rq->channel;
   
        lock_release(&dev->lock);
        kroutine_exec(&rq->base.kr, cpu_is_interruptible());
        lock_spin(&dev->lock);
   
        pv->busy = 0;
          
        /* Start Next request */

        if (dev_request_queue_isempty(&pv->queue[i]))
          continue;

        rq =  dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[i]));
        exrq = (struct efm32_dev_dma_rq_s *)rq;
     
        efm32_dma_start(dev, rq);

        /* Start linked request */

        while(1)
          {
            if (!exrq->rq.arch_rq || exrq->lrq == NULL)
              break;

            exrq = exrq->lrq;

            bool_t empty = dev_request_queue_isempty(&pv->queue[exrq->rq.channel]);

            dev_request_queue_pushback(&pv->queue[exrq->rq.channel], &exrq->rq.base);

            if (empty) 
              efm32_dma_start(dev, &exrq->rq);
          }
      }
  }
  
  lock_release(&dev->lock);
}

static const struct driver_dma_s efm32_dma_ctrl_drv =
{
  .class_ 	  = DRIVER_CLASS_DMA,
  .f_request	= efm32_dma_request,
};

static DEV_INIT(efm32_dma_init);
static DEV_CLEANUP(efm32_dma_cleanup);

const struct driver_s	efm32_dma_drv =
{
  .desc       = "EFM32 DMA driver",
  .f_init     = efm32_dma_init,
  .f_cleanup  = efm32_dma_cleanup,
  .classes    = { &efm32_dma_ctrl_drv, 0 }
};

REGISTER_DRIVER(efm32_dma_drv);

static DEV_INIT(efm32_dma_init)
{
  struct efm32_dma_context *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;
  
  /* allocate private driver data */
  pv = mem_alloc(sizeof(struct efm32_dma_context), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(struct efm32_dma_context));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, NULL);

  if (dev_clock_sink_link(dev, &pv->clk_ep, NULL, 0, 0))
    goto err_mem;

  if (dev_clock_sink_hold(&pv->clk_ep, NULL))
    goto err_clk;
#endif

  /* Check number of channel */

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PL230_DMA_STATUS_ADDR));

  x = PL230_DMA_STATUS_CHNUM_GET(x) + 1;

  assert(CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT <= x);

  size_t s = CHANNEL_SIZE * CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT;

  pv->channel = mem_alloc_align(s, 512, (mem_scope_sys));
  
  if (!pv->channel)
    goto err_mem;

  memset(pv->channel, 0, s);
  
  /* Enable DMA Controller */
  cpu_mem_write_32(pv->addr + PL230_DMA_CONFIG_ADDR, endian_le32(PL230_DMA_CONFIG_EN));

  /* Set Control Data Base Pointer */
  cpu_mem_write_32(pv->addr + PL230_DMA_CTRLBASE_ADDR, endian_le32((uint32_t)pv->channel));

  /* Select Primary Structure */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHALTC_ADDR, endian_le32(PL230_DMA_CHALTC_MASK));

  /* Disable Interrupts */
  cpu_mem_write_32(pv->addr + EFM32_DMA_IEN_ADDR, endian_le32(0));

  /*Disable Channels */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHENC_ADDR, endian_le32(PL230_DMA_CHENC_MASK));

  for(uint8_t i = 0; i < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT; i++) 
    dev_request_queue_init(pv->queue + i);

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_dma_irq, DEV_IRQ_SENSE_RISING_EDGE);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem;

  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv = &efm32_dma_drv;

  return 0;
 
 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
  err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_dma_cleanup)
{
  struct efm32_dma_context	*pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  for(uint8_t i = 0; i < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT; i++) 
    dev_request_queue_destroy(pv->queue + i);

  mem_free(pv);
}
