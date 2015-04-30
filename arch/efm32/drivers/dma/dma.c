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
  uint16_t                      busy;
  /* Waiting and running interleaved request */
  uint16_t                      intlwait;
  uint16_t                      intlrun;
  dev_request_queue_root_t      queue[CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT];
  /* Primary channel memory region */
  uintptr_t                     primary;
  /* Used for double buffering */
  bool_t                        alt;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s    clk_ep;
  /* count for release clock */
  uint16_t                      run;
#endif
};

static void efm32_dev_dma_set_ctrl(struct device_s *dev,
                                   struct dev_dma_rq_s * rq,
                                   struct dev_dma_entry_s *e, 
                                   uint8_t idx,
                                   bool_t alternate)
{
  struct efm32_dma_context *pv = dev->drv_pv;
  struct dev_dma_param_s *p = &rq->param[idx];
  struct efm32_dev_dma_rq_s *exrq = (struct efm32_dev_dma_rq_s *)rq;
  struct efm32_dev_dma_cfg_s * cfg = &exrq->cfg[idx];

  uintptr_t ctrladdr = alternate ? (uintptr_t)pv->primary + 0x100 : (uintptr_t)pv->primary; 
  ctrladdr += CHANNEL_SIZE * p->channel;

  size_t len = e->size; 
  if (len > 1024)
    len = 1024; 

  e->size -= len;

  uint32_t x = DMA_CHANNEL_CFG_N_MINUS_1(len - 1);

  uint8_t mode = DMA_CHANNEL_CFG_CYCLE_CTR_BASIC;

  if (rq->type == DEV_DMA_DBL_BUF_DST)
    mode = DMA_CHANNEL_CFG_CYCLE_CTR_PINGPONG;

  if (rq->arch_rq)
    {
      DMA_CHANNEL_CFG_CYCLE_CTR_SETVAL(x, mode);
      DMA_CHANNEL_CFG_R_POWER_SETVAL(x, cfg->arbiter);
    }
  else
    {
      DMA_CHANNEL_CFG_CYCLE_CTR_SET(x, AUTOREQUEST);
      DMA_CHANNEL_CFG_R_POWER_SET(x, AFTER32);
    }

  DMA_CHANNEL_CFG_SRC_INC_SETVAL(x, p->src_inc);
  DMA_CHANNEL_CFG_DST_INC_SETVAL(x, p->dst_inc);

  uint8_t size = p->src_inc; 
  
  if (p->src_inc == DEV_DMA_INC_NONE)
   {
     if (p->dst_inc == DEV_DMA_INC_NONE)
       size = DMA_CHANNEL_CFG_SRC_SIZE_BYTE;
     else 
       size = p->dst_inc;
   }

  DMA_CHANNEL_CFG_SRC_SIZE_SETVAL(x, size);
  DMA_CHANNEL_CFG_DST_SIZE_SETVAL(x, size);

  uintptr_t src = e->src;

  if (p->src_inc != DEV_DMA_INC_NONE)
    {
      src += ((len - 1) << p->src_inc);
      e->src += len;
    }


  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_SRC_DATA_END_ADDR, endian_le32(src));

  uintptr_t dst = e->dst;

  if (p->dst_inc != DEV_DMA_INC_NONE)
    {
      dst += ((len - 1) << p->dst_inc);
      e->dst += len;
    }

  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_DST_DATA_END_ADDR, endian_le32(dst));
  cpu_mem_write_32(ctrladdr + DMA_CHANNEL_CFG_ADDR, endian_le32(x));

  x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IEN_ADDR));
  x |= EFM32_DMA_IEN_DONE(p->channel);
  cpu_mem_write_32(pv->addr + EFM32_DMA_IEN_ADDR, endian_le32(x));
  
  /* Enable channel */
  x = PL230_DMA_CHENS_CH(p->channel);
  cpu_mem_write_32(pv->addr + PL230_DMA_CHENS_ADDR, endian_le32(x));

  if (rq->arch_rq)
    /* Peripheral triggering source */
    {
      cpu_mem_write_32(pv->addr + EFM32_DMA_CH_CTRL_ADDR(p->channel), endian_le32(cfg->trigsrc));
      x = PL230_DMA_CHREQMASKC_CH(p->channel);
      cpu_mem_write_32(pv->addr + PL230_DMA_CHREQMASKC_ADDR, endian_le32(x));
    }
  else
    /* Software triggering source */
    {
      x = PL230_DMA_CHSWREQ_CH(p->channel);
      cpu_mem_write_32(pv->addr + PL230_DMA_CHSWREQ_ADDR, endian_le32(x));
    }
}

static void efm32_dev_dma_config(struct device_s *dev, struct dev_dma_rq_s *rq)
{
#ifdef CONFIG_DRIVER_EFM32_DMA_DOUBLE_BUFFERING
  struct efm32_dma_context *pv = dev->drv_pv;
#endif

  switch (rq->type)
  {
    case DEV_DMA_BASIC:
      efm32_dev_dma_set_ctrl(dev, rq, &rq->basic, 0, 0);
      break;

#ifdef CONFIG_DRIVER_EFM32_DMA_DOUBLE_BUFFERING
    case DEV_DMA_DBL_BUF_DST:
      /* Primary and Alternate control configuration */
      efm32_dev_dma_set_ctrl(dev, rq, &rq->tr[0], 0, 0);
      efm32_dev_dma_set_ctrl(dev, rq, &rq->tr[1], 0, 1);
      pv->alt = 1;
      break;
#endif

    case DEV_DMA_INTERLEAVED:
      /* Read and write channels configuration. Read configuration must be done first */
      efm32_dev_dma_set_ctrl(dev, rq, &rq->tr[DEV_DMA_INTL_READ], DEV_DMA_INTL_READ, 0);
      efm32_dev_dma_set_ctrl(dev, rq, &rq->tr[DEV_DMA_INTL_WRITE], DEV_DMA_INTL_WRITE, 0);
      break;

    default:
      break;
  }
}

/* @This starts a interleaved request on DMA */
static bool_t efm32_dma_start_intl(struct device_s *dev, struct dev_dma_rq_s *rq)
{
  struct efm32_dma_context *pv = dev->drv_pv;
  uint8_t wchan = rq->param[DEV_DMA_INTL_WRITE].channel;

  /* Channels must be contigeous */
  assert(rq->param[DEV_DMA_INTL_READ].channel == (rq->param[DEV_DMA_INTL_WRITE].channel + 1));

  bool_t wempty = dev_request_queue_isempty(&pv->queue[wchan]);

  bool_t idle = !((pv->busy | pv->intlrun) & (1 << wchan));

  if (wempty && idle)
  {
    pv->intlrun |= (1 << wchan);
    return 1;
  }

  pv->intlwait |= (1 << wchan);
  return 0;

}

static DEVDMA_REQUEST(efm32_dma_request)
{
  struct device_s         *dev = accessor->dev;
  struct efm32_dma_context	*pv = dev->drv_pv;

  struct dev_request_s *base = &req->base;

  LOCK_SPIN_IRQ(&dev->lock);

  req->error = 0;

  bool_t notsup = (req->type == DEV_DMA_DBL_BUF_SRC) ||
#ifndef CONFIG_DRIVER_EFM32_DMA_DOUBLE_BUFFERING
                  (req->type == DEV_DMA_DBL_BUF_DST) ||
#endif
                  (req->type == DEV_DMA_SCATTER_GATHER);
  if (notsup)
    {
      req->error = -ENOTSUP;
      goto end;
    }

#ifdef CONFIG_DEVICE_CLOCK
  if (!pv->run++)
    dev_clock_sink_hold(&pv->clk_ep, 0);
#endif

  bool_t empty = dev_request_queue_isempty(&pv->queue[req->param[0].channel]);
  dev_request_queue_pushback(&pv->queue[req->param[0].channel], base);
    
  bool_t idle = !((pv->busy | pv->intlrun) & (1 << req->param[0].channel));

  if (empty & idle)
    {
      if (req->type == DEV_DMA_INTERLEAVED)
        {
          if (!efm32_dma_start_intl(dev, req))
            goto end;
        }
      efm32_dev_dma_config(dev, req);
    }
  
end:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (req->error)
    kroutine_exec(&base->kr, cpu_is_interruptible());
}

static inline void efm32_dev_dma_kroutine(struct device_s *dev, struct dev_request_s *base)
{
  struct efm32_dma_context *pv = dev->drv_pv;
       
  lock_release(&dev->lock);
  kroutine_exec(&base->kr, cpu_is_interruptible());
  lock_spin(&dev->lock);

#ifdef CONFIG_DEVICE_CLOCK
  pv->run--;
#endif
}

static DEV_IRQ_EP_PROCESS(efm32_dma_irq)
{
  struct device_s *dev = ep->dev;
  struct efm32_dma_context *pv = dev->drv_pv;
  struct dev_dma_rq_s *rq; 
  
  lock_spin(&dev->lock);

  while(1)
  {
    uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_DMA_IF_ADDR));

    if (!x)
      {
#ifdef CONFIG_DEVICE_CLOCK
        /* All queue are empty */
        if (!pv->run)
          dev_clock_sink_release(&pv->clk_ep);
#endif
        break;
      }
 
    for (uint16_t i = 0; i < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT; i++) 
      {
        if (!(x & EFM32_DMA_IF_DONE(i)))
          continue;

        /* Clear Interrupt Flag */
        cpu_mem_write_32(pv->addr + EFM32_DMA_IFC_ADDR, endian_le32(EFM32_DMA_IF_DONE(i)));
       
        rq =  dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[i]));
       
        bool_t end = 1;

        /* Not the end of a write interleaved transfer */
        if (!(pv->intlrun & (1 << i)))
        {
          assert(rq != NULL);
          pv->busy = 1 << i;
          switch (rq->type)
          {
            case DEV_DMA_INTERLEAVED:
            case DEV_DMA_BASIC:
              dev_request_queue_pop(&pv->queue[i]);
              efm32_dev_dma_kroutine(dev, &rq->base);
              break;
#ifdef CONFIG_DRIVER_EFM32_DMA_DOUBLE_BUFFERING
            case DEV_DMA_DBL_BUF_DST:
              rq->count -= 1;
              end = (rq->count == 0);
        
              if (end)
                dev_request_queue_pop(&pv->queue[i]);
        
              efm32_dev_dma_kroutine(dev, &rq->base);
        
              pv->alt ^= 1;
        
              if (!end)
                efm32_dev_dma_set_ctrl(dev, &rq->tr[pv->alt], rq, 0, pv->alt);
              break;
#endif
             default:
               assert(1);
               break;
          }
          pv->busy = 0;
        }

        pv->intlrun &= ~(1 << i);
        rq = dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[i]));

        /* Start Next request */

        /* A interleaved request is waiting on this channel */
        if (pv->intlwait & (1 << i))
          {
            pv->intlwait &= ~(1 << i);
            rq =  dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[i + 1]));
          }

        if (!end || rq == NULL)
          continue;
         
        if (rq->type == DEV_DMA_INTERLEAVED)
          {
            if (!efm32_dma_start_intl(dev, rq))
              continue;
          }

        efm32_dev_dma_config(dev, rq);
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

  if (dev_clock_sink_hold(&pv->clk_ep, 0))
    goto err_clk;
#endif

  /* Check number of channel */

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PL230_DMA_STATUS_ADDR));

  x = PL230_DMA_STATUS_CHNUM_GET(x) + 1;

  assert(CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT <= x);

#ifdef CONFIG_DRIVER_EFM32_DMA_DOUBLE_BUFFERING
  size_t s = CHANNEL_SIZE * x + 0x100;
#else
  size_t s = CHANNEL_SIZE * CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT;
#endif

  pv->primary = (uintptr_t)mem_alloc_align(s, 512, (mem_scope_sys));
  
  if (!pv->primary)
    goto err_mem;

  memset((uint8_t *)pv->primary, 0, s);
  
  /* Enable DMA Controller */
  cpu_mem_write_32(pv->addr + PL230_DMA_CONFIG_ADDR, endian_le32(PL230_DMA_CONFIG_EN));

  /* Set primary and alternate control address */
  cpu_mem_write_32(pv->addr + PL230_DMA_CTRLBASE_ADDR, endian_le32((uint32_t)pv->primary));

  /* Select Primary and Alternate structure */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHALTC_ADDR, endian_le32(PL230_DMA_CHALTC_MASK));

  /* Disable Interrupts */
  cpu_mem_write_32(pv->addr + EFM32_DMA_IEN_ADDR, endian_le32(0));

  /* Disable Channels */
  cpu_mem_write_32(pv->addr + PL230_DMA_CHENC_ADDR, endian_le32(PL230_DMA_CHENC_MASK));

  for(uint8_t i = 0; i < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT; i++) 
    dev_request_queue_init(pv->queue + i);

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_dma_irq, DEV_IRQ_SENSE_RISING_EDGE);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem;

  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv = &efm32_dma_drv;

#ifdef CONFIG_DEVICE_CLOCK
  /* Release clock */
  dev_clock_sink_release(&pv->clk_ep);
#endif

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

  dev_clock_sink_release(&pv->clk_ep);

  for(uint8_t i = 0; i < CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT; i++) 
    dev_request_queue_destroy(pv->queue + i);

  mem_free((uint8_t *)pv->primary);
  mem_free(pv);
}
