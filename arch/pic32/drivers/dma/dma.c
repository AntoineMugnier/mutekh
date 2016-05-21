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
#include <device/clock.h>

#include <arch/pic32/dma_request.h>
#include <arch/pic32/dma.h>

#define PIC32_DMA_MAX_SIZE 65536
#define PIC32_DMA_IRQ_MASK (PIC32_DMA_DCHINT_CHBCIE | PIC32_DMA_DCHINT_CHERIE)


static uintptr_t pic32_dma_vir_to_phys(uintptr_t addr)
{
  return (addr & 0x20000000) ? addr - 0xA0000000 : addr - 0x80000000;
}

DRIVER_PV(struct pic32_dma_context_s
{
  struct dev_irq_src_s          irq_ep[CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT];
  dev_request_queue_root_t      queue[CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT];
  /* base address of DMA */
  uintptr_t                     addr;
  uint8_t                       intlrun;
  uint8_t                       intlwait;
  /* lock for reentrant kroutine */
  uint8_t                       busy;
});


static void pic32_dev_dma_channel_cfg(struct device_s *dev,
                                      struct dev_dma_rq_s * rq,
                                      uint8_t idx)
{
  struct pic32_dma_context_s *pv = dev->drv_pv;
  struct pic32_dev_dma_rq_s *exrq = (struct pic32_dev_dma_rq_s *)rq;
  uint8_t chan = rq->param[idx].channel;

   ensure(!((rq->param[idx].src_inc == rq->param[idx].dst_inc) &&
      (rq->param[idx].src_inc == DEV_DMA_INC_NONE)));

#ifdef CONFIG_CPU_CACHE
   /* Flush buffer in cache */
   if (rq->param[idx].src_inc != DEV_DMA_INC_NONE)
     cpu_dcache_flush_buf((void *)rq->tr[idx].src, rq->tr[idx].size);
#endif

   /* Disable channel */
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHCTRL_ADDR(chan), 0);

   /* Channel cell size */
   uint32_t x = rq->arch_rq ? endian_le32(exrq->cfg[idx].cell_size) : 0;
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHCSIZ_ADDR(chan), x);

   /* Transfer source size */
   x = rq->param[idx].src_inc == DEV_DMA_INC_NONE ? 1 << rq->param[idx].dst_inc : rq->tr[idx].size;
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHSSIZ_ADDR(chan), endian_le32(x));

   /* Transfer destination size */
   x = rq->param[idx].dst_inc == DEV_DMA_INC_NONE ? 1 << rq->param[idx].src_inc : rq->tr[idx].size;
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHDSIZ_ADDR(chan), endian_le32(x));

   /* Transfer source address */
   x = pic32_dma_vir_to_phys(endian_le32(rq->tr[idx].src));
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHSSA_ADDR(chan), x);
   
   /* Transfer destination address */
   x = pic32_dma_vir_to_phys(endian_le32(rq->tr[idx].dst));
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHDSA_ADDR(chan), x);

   if (rq->arch_rq)
     {
       x = PIC32_DMA_DCHECTRL_SIRQEN | PIC32_DMA_DCHECTRL_SIRQ(exrq->cfg[idx].trigsrc);
       cpu_mem_write_32(pv->addr + PIC32_DMA_DCHECTRL_ADDR(chan), endian_le32(x));
     }

   /* Enable interrupt on channel */
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHINT_ADDR(chan), endian_le32(PIC32_DMA_IRQ_MASK));
 

   /* Enable channel */
   x = endian_le32(PIC32_DMA_DCHCTRL_EN);
   cpu_mem_write_32(pv->addr + PIC32_DMA_DCHCTRL_SET_ADDR(chan), x);
}
                                         
static void pic32_dma_start(struct device_s *dev, struct dev_dma_rq_s * rq, bool_t test)
{
  struct pic32_dma_context_s *pv = dev->drv_pv;
  uint32_t addr;

  switch (rq->type)
  {
    case DEV_DMA_BASIC:

      pic32_dev_dma_channel_cfg(dev, rq, 0);
  
      /* Start transfer */
      if (!rq->arch_rq)
        {
            addr = pv->addr + PIC32_DMA_DCHECTRL_ADDR(rq->param[0].channel);
            cpu_mem_write_32(addr, endian_le32(PIC32_DMA_DCHECTRL_CFORCE));
        }
      break;

    case DEV_DMA_INTERLEAVED:
        {
          /* Channel must be contiguous. Write channel must have lower priority */
          ensure(rq->tr[DEV_DMA_INTL_READ].size == rq->tr[DEV_DMA_INTL_WRITE].size);

          bool_t empty = 1;
          uint8_t wchan = rq->param[DEV_DMA_INTL_WRITE].channel;
           
          /* Test queue for write operation */
          if (test)
            empty = dev_request_queue_isempty(&pv->queue[wchan]);

          bool_t run = empty && !((pv->busy | pv->intlrun) & (1 << wchan));

          if (!run)
          {
            pv->intlwait |= (1 << wchan);
            return;
          }
    
          pv->intlrun |= (1 << wchan);

          /* Read channel configuration */
          pic32_dev_dma_channel_cfg(dev, rq, DEV_DMA_INTL_READ);
    
          /* Write channel configuration */
          pic32_dev_dma_channel_cfg(dev, rq, DEV_DMA_INTL_WRITE);
    
        }
      break;

    default:
      return;
  }
}

static DEVDMA_REQUEST(pic32_dma_request)
{
  struct device_s *dev = accessor->dev;
  struct pic32_dma_context_s	*pv = dev->drv_pv;

  struct dev_request_s *base = &req->base;

  LOCK_SPIN_IRQ(&dev->lock);

  uint8_t chan;
  bool_t empty = 0;
  bool_t run = 0;
  req->error = 0;

  if (req->tr[0].size > PIC32_DMA_MAX_SIZE)
  {
    req->error = -ENOTSUP;
    goto end;
  }

  switch (req->type)
  {
    case DEV_DMA_BASIC:

      chan = req->param[0].channel;
      empty = dev_request_queue_isempty(&pv->queue[chan]);
      run = empty && !((pv->busy | pv->intlrun) & (1 << chan));
      dev_request_queue_pushback(&pv->queue[chan], base);

      if (run)
        pic32_dma_start(dev, req, 0);

      break;

    case DEV_DMA_INTERLEAVED:

      chan = req->param[DEV_DMA_INTL_READ].channel;
      empty = dev_request_queue_isempty(&pv->queue[chan]);
      run = empty && !((pv->busy | pv->intlrun) & (1 << chan));
      dev_request_queue_pushback(&pv->queue[chan], base);

      if (run)
        pic32_dma_start(dev, req, 1);

      break;

    default:
      req->error = -ENOTSUP;

      break;
  }

end:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (req->error)
    kroutine_exec(&base->kr);
}

#ifdef CONFIG_CPU_CACHE
static void pic32_dma_invalidate_cache(struct dev_dma_rq_s * rq)
{
  switch (rq->type)
  {
    case DEV_DMA_BASIC:
      if (rq->param[0].dst_inc != DEV_DMA_INC_NONE)
        cpu_dcache_invld_buf((void *)rq->tr[0].dst, rq->tr[0].size);

    case DEV_DMA_INTERLEAVED:
      if (rq->param[DEV_DMA_INTL_READ].dst_inc != DEV_DMA_INC_NONE)
        cpu_dcache_invld_buf((void *)rq->tr[DEV_DMA_INTL_READ].dst, rq->tr[DEV_DMA_INTL_READ].size);
      if (rq->param[DEV_DMA_INTL_WRITE].dst_inc != DEV_DMA_INC_NONE)
        cpu_dcache_invld_buf((void *)rq->tr[DEV_DMA_INTL_WRITE].dst, rq->tr[DEV_DMA_INTL_WRITE].size);
      break;

    default:
      return;
  }
}
#endif

static DEV_IRQ_SRC_PROCESS(pic32_dma_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pic32_dma_context_s *pv = dev->drv_pv;
  struct dev_dma_rq_s *rq; 

  /* Channel number */
  uint_fast8_t chan = ep - pv->irq_ep;
  uint8_t msk = 1 << chan;
  
  lock_spin(&dev->lock);

  while(1)
  {
    uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_DMA_DCHINT_ADDR(chan)));

    /* Address error */
    if (x & PIC32_DMA_DCHINT_CHERIF)
      assert(0);

    if (!(x & PIC32_DMA_DCHINT_CHBCIF))
        break;
 
    /* Clear Interrupt flag and disable interrupt */
    cpu_mem_write_32(pv->addr + PIC32_DMA_DCHINT_ADDR(chan), PIC32_DMA_IRQ_MASK);

    if (pv->intlrun & msk)
    {
      pv->intlrun &= ~msk;
      goto next;
    }

    rq = dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[chan]));

    if (rq == NULL)
      break;

#ifdef CONFIG_CPU_CACHE 
    pic32_dma_invalidate_cache(rq);
#endif
    pv->busy = msk;
    dev_request_queue_pop(&pv->queue[chan]);
    lock_release(&dev->lock);
    kroutine_exec(&rq->base.kr);
    lock_spin(&dev->lock);
    pv->busy = 0;

next:
    /* Next request */
    rq = dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[chan]));

    bool_t test = 1;
    /* A interleaved request is waiting on this channel */
    if (pv->intlwait & msk)
      {
        pv->intlwait &= ~msk;
        rq = dev_dma_rq_s_cast(dev_request_queue_head(&pv->queue[chan - 1]));
        test = 0;
      }

    if (rq == NULL)
      continue;

    pic32_dma_start(dev, rq, test);
  }
  
  lock_release(&dev->lock);
}

#define pic32_dma_use dev_use_generic

static DEV_INIT(pic32_dma_init)
{
  struct pic32_dma_context_s *pv;

  
  /* allocate private driver data */
  pv = mem_alloc(sizeof(struct pic32_dma_context_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(struct pic32_dma_context_s));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* Enable Dma */
  cpu_mem_write_32(pv->addr + PIC32_DMA_CTRL_ADDR, PIC32_DMA_CTRL_ON);

  /* Disable DMA and channels */
  for(uint8_t i = 0; i < CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT; i++) 
    {
      cpu_mem_write_32(pv->addr + PIC32_DMA_DCHCTRL_ADDR(i), 0);
      dev_request_queue_init(&pv->queue[i]);
    }

  device_irq_source_init(dev, pv->irq_ep, CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT, &pic32_dma_irq);
  if (device_irq_source_link(dev, pv->irq_ep, CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT, -1))
    goto err_mem;


  return 0;
 
  err_mem:
    mem_free(pv);
  return -1;
}

static DEV_CLEANUP(pic32_dma_cleanup)
{
  struct pic32_dma_context_s	*pv = dev->drv_pv;

  device_irq_source_unlink(dev, pv->irq_ep, CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT);

  for(uint8_t i = 0; i < CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT; i++) 
    dev_request_queue_destroy(pv->queue + i);

  mem_free(pv);
}

DRIVER_DECLARE(pic32_dma_drv, 0, "PIC32 DMA", pic32_dma,
               DRIVER_DMA_METHODS(pic32_dma));

DRIVER_REGISTER(pic32_dma_drv);

