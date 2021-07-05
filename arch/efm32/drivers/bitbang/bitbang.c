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

    Copyright (c) 2018 Sebastien CERDAN <sebcerdan@gmail.com>

*/

#include <string.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/bitbang.h>
#include <device/class/timer.h>
#include <device/class/iomux.h>
#include <device/class/gpio.h>
#include <device/class/dma.h>
#include <device/class/bitbang.h>
#include <device/irq.h>
#include <device/clock.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

#include <arch/efm32/dma_source.h>
#include <arch/efm32/timer.h>

DRIVER_PV(struct efm32_bitbang_ctx_s
{
  struct device_s                  *dev;
  /* Timer used for bitbanging */
  uintptr_t                        addr;
  struct dev_irq_src_s             irq_ep;
  dev_request_queue_root_t         queue;
  /* DMA */                       
  uint16_t                         dma_tx_link;
  uint16_t                         dma_rx_link;
  struct dev_dma_rq_s              dma_rq;
  struct dev_dma_desc_s            dma_desc[2];
  struct device_dma_s              dma;
  struct device_iomux_s            iomux;

  struct kroutine_s                kr;
  uint32_t                         dvalue;
  uint8_t                          div;
  iomux_io_id_t                    id;

  struct dev_clock_sink_ep_s       clk_ep;
  struct dev_freq_s                freq;
  struct dev_freq_s                sfreq;
  bool_t                           pending;
  /* A transfer is on-going */
  bool_t                           running;
});

STRUCT_COMPOSE(efm32_bitbang_ctx_s, dma_rq);
STRUCT_COMPOSE(efm32_bitbang_ctx_s, kr);

static void efm32_bitbang_ctx_start_tx(struct efm32_bitbang_ctx_s *pv, struct dev_bitbang_rq_s * rq);
static void efm32_bitbang_ctx_start_rx(struct efm32_bitbang_ctx_s *pv, struct dev_bitbang_rq_s * rq);

static void efm32_bitbang_freq(struct efm32_bitbang_ctx_s *pv)
{
  struct dev_bitbang_rq_s *rq = dev_bitbang_rq_head(&pv->queue);

  if ((pv->sfreq.num == rq->unit.num) && (rq->unit.denom == pv->sfreq.denom))
    return;

  pv->sfreq.num = rq->unit.num;
  pv->sfreq.denom = rq->unit.denom;

  /* Compute scale factor to the requested frequency. */
  uint32_t scale = (pv->freq.num * rq->unit.denom) / (pv->freq.denom * rq->unit.num);

  pv->div = scale > 1024 ? 10 : bit_msb_index(scale);
}

static void bitbang_process_next(struct efm32_bitbang_ctx_s *pv)
{
  struct dev_bitbang_rq_s *rq = dev_bitbang_rq_head(&pv->queue);

  pv->pending = 0;

  if (rq == NULL || pv->running)
    return;
  
  pv->running = 1;
  efm32_bitbang_freq(pv);

  switch(rq->type)
    {
      case DEV_BITBANG_RD:
        efm32_bitbang_ctx_start_rx(pv, rq);
        break; 
      case DEV_BITBANG_WR: 
        efm32_bitbang_ctx_start_tx(pv, rq);
        break; 
      default:
        pv->running = 0;
        rq->error = -ENOTSUP;
        rq->base.drvdata = NULL;
        dev_bitbang_rq_pop(&pv->queue);
        dev_bitbang_rq_done(rq);
        break;
    }
}


static KROUTINE_EXEC(efm32_bitbang_process_next_kr)
{
  struct efm32_bitbang_ctx_s *pv = efm32_bitbang_ctx_s_from_kr(kr);

  LOCK_SPIN_IRQ(&pv->dev->lock);
  bitbang_process_next(pv);
  LOCK_RELEASE_IRQ(&pv->dev->lock);
}

static void efm32_bitbang_end_wr_rq(struct efm32_bitbang_ctx_s *pv)
{
  LOCK_SPIN_IRQ(&pv->dev->lock);

  struct dev_bitbang_rq_s *rq = dev_bitbang_rq_head(&pv->queue);

  assert(rq && rq->type == DEV_BITBANG_WR);

  rq->error = 0;
  rq->base.drvdata = NULL;

  pv->running = 0;
  /* End current request */
  dev_bitbang_rq_pop(&pv->queue);
  dev_bitbang_rq_done(rq);

  /* Process next request */
  kroutine_exec(&pv->kr);

  LOCK_RELEASE_IRQ(&pv->dev->lock);
}

static DEV_DMA_CALLBACK(bitbang_tx_dma_done)
{
  assert(err == 0);

  struct efm32_bitbang_ctx_s *pv = efm32_bitbang_ctx_s_from_dma_rq(rq);

  /* Set pin as input */
  DEVICE_OP(&pv->iomux, setup, pv->id, DEV_PIN_INPUT, IOMUX_INVALID_MUX, 0);

  /* Stop timer */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

  efm32_bitbang_end_wr_rq(pv);

  return 0;
}

static void efm32_bitbang_end_rd_rq(struct efm32_bitbang_ctx_s *pv, error_t err, size_t size)
{
  struct dev_bitbang_rq_s *rq = dev_bitbang_rq_head(&pv->queue);

  if (rq == NULL)
    return;

  assert(rq->type == DEV_BITBANG_RD);

  rq->count = size;
  rq->error = err;
  rq->base.drvdata = NULL;

  pv->running = 0;

  dev_bitbang_rq_pop(&pv->queue);
  dev_bitbang_rq_done(rq);
}

static DEV_DMA_CALLBACK(bitbang_rx_dma_done)
{
  assert(err == 0);

  struct efm32_bitbang_ctx_s *pv = efm32_bitbang_ctx_s_from_dma_rq(rq);

  LOCK_SPIN_IRQ(&pv->dev->lock);

  /* Turn off CC channel and stop timer */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(0), 0);

  uint32_t x = cpu_mem_read_32(pv->addr + EFM32_TIMER_IF_ADDR);
  x &= cpu_mem_read_32(pv->addr + EFM32_TIMER_IEN_ADDR);

  pv->pending = 1;

  if (x & EFM32_TIMER_IEN_OF)
    /* Timer irq is pending */
    goto end;

  /* Buffer overflow */
  efm32_bitbang_end_rd_rq(pv, -EIO, 0);

  /* Process next request */
  kroutine_exec(&pv->kr);

end:
  LOCK_RELEASE_IRQ(&pv->dev->lock);
  return 0;
}

static void efm32_bitbang_ctx_start_rx(struct efm32_bitbang_ctx_s *pv, struct dev_bitbang_rq_s * rq)
{
  /* Timer */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
  /* Timer top value is set to read timeout */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_TOP_ADDR, rq->read_timeout);

  uint32_t x = EFM32_TIMER_CTRL_MODE(UP) |
               pv->div << 24|
               EFM32_TIMER_CTRL_OSMEN |
               EFM32_TIMER_CTRL_FALLA(RELOADSTART) |
               EFM32_TIMER_CTRL_RISEA(RELOADSTART);

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, endian_le32(x));

  /* Set timer channel as input/capture mapped to external pin */
  x = EFM32_TIMER_CC_CTRL_MODE(INPUTCAPTURE) |
      EFM32_TIMER_CC_CTRL_FILT |
      EFM32_TIMER_CC_CTRL_ICEDGE(BOTH);

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(0), endian_le32(x));

  /* Enable Overflow irq */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IF_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, EFM32_TIMER_IEN_OF);

  /* Start DMA request */

  struct dev_dma_rq_s *drq = &pv->dma_rq;

  /* Desc 0 is used to discard first value */

  struct dev_dma_desc_s *desc = &pv->dma_desc[0];

  desc->src.reg.addr = pv->addr + EFM32_TIMER_CC_CCV_ADDR(0);
  desc->src.reg.width = rq->sym_width;
  desc->src.reg.size = 0;
  desc->src.reg.burst = 1;

  desc->dst.mem.addr = (uintptr_t)&pv->dvalue;
  desc->dst.mem.stride = 0;
  desc->dst.mem.inc = DEV_DMA_INC_1_UNITS;

  desc = &pv->dma_desc[1];

  desc->src.reg.addr = pv->addr + EFM32_TIMER_CC_CCV_ADDR(0);
  desc->src.reg.width = rq->sym_width;
  desc->src.reg.size = rq->count - 1;
  desc->src.reg.burst = 1;

  desc->dst.mem.addr = (uintptr_t)rq->symbols;
  desc->dst.mem.stride = 0;
  desc->dst.mem.inc = DEV_DMA_INC_1_UNITS;

  drq->dev_link.src = pv->dma_rx_link;
  drq->type = DEV_DMA_REG_MEM;
  drq->desc_count_m1 = 1;
  drq->f_done = bitbang_rx_dma_done;

  ensure(DEVICE_OP(&pv->dma, request, drq, NULL) == 0);
}

static void efm32_bitbang_ctx_start_tx(struct efm32_bitbang_ctx_s *pv, struct dev_bitbang_rq_s * rq)
{
  /* Set pin as output */
  DEVICE_OP(&pv->iomux, setup, pv->id, DEV_PIN_PUSHPULL, IOMUX_INVALID_MUX, 0);

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, pv->div << 24);
 
  uint32_t x = EFM32_TIMER_CC_CTRL_MODE(OUTPUTCOMPARE) |
               EFM32_TIMER_CC_CTRL_COFOA(TOGGLE);

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(0), endian_le32(x));

  pv->dvalue = 0xFFFF;

  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IF_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CNT_ADDR, 0);

  cpu_mem_write_32(pv->addr + EFM32_TIMER_TOP_ADDR, 1024);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_TOPB_ADDR, (uint32_t)rq->symbols);

  struct dev_dma_rq_s *drq = &pv->dma_rq;

  drq->dev_link.dst = pv->dma_tx_link;
  drq->type = DEV_DMA_MEM_REG;
  drq->desc_count_m1 = 1;
  drq->f_done = bitbang_tx_dma_done;

  struct dev_dma_desc_s *desc = &pv->dma_desc[0];

  desc->src.mem.addr = (uintptr_t)rq->symbols + (1 << rq->sym_width);
  desc->src.mem.size = rq->count - 2;
  desc->src.mem.width = rq->sym_width;
  desc->src.mem.inc = DEV_DMA_INC_1_UNITS;
  desc->src.mem.stride = 0;
  desc->dst.reg.addr = pv->addr + EFM32_TIMER_TOPB_ADDR;
  desc->dst.reg.burst = 1;

  /* Desc 1 is used to wait end of TX */

  desc = &pv->dma_desc[1];

  desc->src.mem.size = 1;
  desc->src.mem.width = rq->sym_width;
  desc->src.mem.inc = DEV_DMA_INC_0_UNIT;
  desc->src.mem.stride = 0;
  desc->src.mem.addr = (uintptr_t)&pv->dvalue;
  desc->dst.reg.addr = pv->addr + EFM32_TIMER_TOPB_ADDR;
  desc->dst.reg.burst = 1;

  /* Start DMA request */
  ensure(DEVICE_OP(&pv->dma, request, drq, NULL) == 0);

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_START));
}


static error_t efm32_bitbang_cancel_rx(struct efm32_bitbang_ctx_s *pv)
{
  struct dev_dma_rq_s *drq = &pv->dma_rq;

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

  uint32_t x = cpu_mem_read_32(pv->addr + EFM32_TIMER_IF_ADDR);
  x &= cpu_mem_read_32(pv->addr + EFM32_TIMER_IEN_ADDR);

  if (x & EFM32_TIMER_IEN_OF)
  /* Timer irq is pending */
    return -EBUSY;

  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IF_ADDR, 0);
  /* Turn off CC and stop timer */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(0), 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, 0);

  /* Cancel dma request */ 
  return DEVICE_OP(&pv->dma, cancel, drq);
}

static DEV_BITBANG_CANCEL(efm32_bitbang_cancel)
{
  struct device_s * dev = accessor->dev;
  struct efm32_bitbang_ctx_s * pv  = dev->drv_pv;

  error_t err = -EBUSY;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);

  struct dev_bitbang_rq_s *hrq = dev_bitbang_rq_head(&pv->queue);

  if (rq == hrq)
    {
      switch(rq->type)
        {
          case DEV_BITBANG_RD:
            if (pv->pending)
              break;
            err = efm32_bitbang_cancel_rx(pv);
            if (err)
              break;
            rq->base.drvdata = NULL;
            dev_bitbang_rq_pop(&pv->queue);
            bitbang_process_next(pv);
            break; 
          case DEV_BITBANG_WR:
            break; 
          default:
            break;
        }
    }
  else if (rq->base.drvdata == pv)
  /* Request is in queue and is not being processed */
    {
      err = 0;
      rq->base.drvdata = NULL;
      dev_bitbang_rq_remove(&pv->queue, rq);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}


static DEV_BITBANG_REQUEST(efm32_bitbang_request)
{
  struct device_s * dev = accessor->dev;
  struct efm32_bitbang_ctx_s *pv  = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  bool_t empty = dev_rq_queue_isempty(&pv->queue);
  dev_bitbang_rq_pushback(&pv->queue, rq);
  rq->base.drvdata = pv;

  if (empty && !pv->pending)
    bitbang_process_next(pv);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(efm32_bitbang_irq_process)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_bitbang_ctx_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_dma_rq_s *drq = &pv->dma_rq;

  /* Turn off CC and stop timer */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CC_CTRL_ADDR(0), 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CTRL_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

  /* Clean irq */
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IFC_ADDR, EFM32_TIMER_IFC_MASK);

  pv->pending = 1;

  /* Cancel dma request */ 
  error_t err = DEVICE_OP(&pv->dma, cancel, drq);

  if (err == -EBUSY)
  /* DMA irq handler will be called sooner */
    goto end;
  
  if (pv->dma_rq.cancel.size)
    efm32_bitbang_end_rd_rq(pv, 0, pv->dma_rq.cancel.size - 1);
  else
    efm32_bitbang_end_rd_rq(pv, 0, pv->dma_rq.cancel.size);

  /* Request is terminated here */
  bitbang_process_next(pv);

end:
  lock_release(&dev->lock);
}

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
static void efm32_bitbang_clk_changed(struct device_s *dev)
{
  struct efm32_bitbang_ctx_s *pv = dev->drv_pv;

  efm32_bitbang_freq(pv);

}
#endif

static DEV_INIT(efm32_bitbang_init)
{
  struct efm32_bitbang_ctx_s  *pv;

  pv = mem_alloc(sizeof(struct efm32_bitbang_ctx_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  pv->dev = dev;
  
  dev_rq_queue_init(&pv->queue);

  /* Init GPIO stuff */

  iomux_demux_t loc;

  if (device_iomux_setup(dev, "<io", &loc, &pv->id, NULL))
    goto err_mem;

  if (device_get_param_dev_accessor(dev, "iomux", &pv->iomux.base, DRIVER_CLASS_IOMUX))
    goto err_mem;

  enum dev_clock_ep_flags_e flags = DEV_CLOCK_EP_FREQ_NOTIFY |
                                    DEV_CLOCK_EP_POWER_CLOCK |
                                    DEV_CLOCK_EP_GATING_SYNC;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, flags, &pv->freq))
    goto err_mem;

  kroutine_init_deferred(&pv->kr, efm32_bitbang_process_next_kr);

  /* Timer initialisation */

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_clk;

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_IF_ADDR, 0);
 
  pv->div = EFM32_TIMER_CTRL_PRESC_DIV8;

  uint32_t x;
 
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) || \
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
 /* Enable Channel output on pin */
  x = EFM32_TIMER_ROUTEPEN_CCPEN(0);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_ROUTEPEN_ADDR, endian_le32(x));
 
  x = EFM32_TIMER_ROUTELOC0_CCLOC(0, loc);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_ROUTELOC0_ADDR, endian_le32(x));
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
 /* Enable Channel output on pin */
  x = EFM32_TIMER_ROUTE_CCPEN(0);
  EFM32_TIMER_ROUTE_LOCATION_SETVAL(x, loc);
  cpu_mem_write_32(pv->addr + EFM32_TIMER_ROUTE_ADDR, endian_le32(x));
#else
# error
#endif

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)

  pv->dvalue = 0xFFFF;

  uint32_t dma_tx_mask, dma_rx_mask;
  uint32_t dma_tx_link, dma_rx_link;

  if (device_res_get_dma(dev, 0, &dma_tx_mask, &dma_tx_link) ||
      device_res_get_dma(dev, 1, &dma_rx_mask, &dma_rx_link) ||
      device_get_param_dev_accessor(dev, "dma", &pv->dma.base, DRIVER_CLASS_DMA))
    goto err_clk;

  if (dma_tx_mask != dma_rx_mask)
    goto err_clk;

  pv->dma_rx_link = dma_rx_link | (EFM32_DMA_SIGNAL_TIMERCC0 << 8);
  pv->dma_tx_link = dma_tx_link | (EFM32_DMA_SIGNAL_TIMERUFOF << 8);

  struct dev_dma_rq_s *rq = &pv->dma_rq;
  
  rq->loop_count_m1 = 0;
  rq->chan_mask = dma_tx_mask;
  rq->cache_ptr = NULL;
#else
  goto err_clk;
#endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_bitbang_irq_process);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_clk;

  return 0;

 err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_bitbang_cleanup)
{
  struct efm32_bitbang_ctx_s *pv = dev->drv_pv;

  device_put_accessor(&pv->iomux.base);
  device_put_accessor(&pv->dma.base);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  cpu_mem_write_32(pv->addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  mem_free(pv);

  return 0;
}

static DEV_USE(efm32_bitbang_use)
{
    switch (op)
      {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
      case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
        struct dev_clock_notify_s *chg = param;
        struct dev_clock_sink_ep_s *sink = chg->sink;
        struct device_s *dev = sink->dev;
        struct efm32_bitbang_ctx_s *pv = dev->drv_pv;
        pv->freq = chg->freq;
        efm32_bitbang_clk_changed(dev);
        return 0;
    }
#endif
      default:
        return dev_use_generic(param, op);
      }
}

DRIVER_DECLARE(efm32_bitbang_drv, 0, "EFM32 bitbang", efm32_bitbang,
               DRIVER_BITBANG_METHODS(efm32_bitbang));

DRIVER_REGISTER(efm32_bitbang_drv);

