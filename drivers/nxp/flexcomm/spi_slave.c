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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2023
*/

#define LOGK_MODULE_ID "fsps"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/clock.h>
#include <device/irq.h>
#include <device/class/spi.h>
#include <device/class/iomux.h>
#include <device/class/dma.h>

#include "flexcomm.h"

struct flexcomm_spi_slave_pv_s
{
  uint32_t base;
  struct dev_irq_src_s irq_ep[1];

  struct dev_dma_rq_s dma_rx_rq;
  struct dev_dma_desc_s dma_rx_desc;
  struct dev_dma_rq_s dma_tx_rq;
  struct dev_dma_desc_s dma_tx_desc;
  struct device_dma_s dma;

  dev_request_queue_root_t queue;
  struct dev_spi_slave_rq_s *current;
};

STRUCT_COMPOSE(flexcomm_spi_slave_pv_s, dma_rx_rq);
STRUCT_COMPOSE(flexcomm_spi_slave_pv_s, dma_tx_rq);
DRIVER_PV(struct flexcomm_spi_slave_pv_s);

static DEV_DMA_CALLBACK(flexcomm_spi_slave_dma_rx_done)
{
  struct flexcomm_spi_slave_pv_s *pv = flexcomm_spi_slave_pv_s_from_dma_rx_rq(rq);
  struct device_s *dev = pv->irq_ep->base.dev;
  struct dev_spi_slave_rq_s *rq;

  {
    LOCK_SPIN_SCOPED(&dev->lock);
    rq = pv->current;
    pv->current = NULL;
  }

  logk_debug("DMA RX done, %d", err);

  if (rq) {
    if (tr->data.out)
      tr->data.out += tr->data.count;
    if (tr->data.in)
      tr->data.in += tr->data.count;
    tr->data.count = 0;
    kroutine_exec(&tr->kr);
  }
  return 0;
}

static DEV_DMA_CALLBACK(flexcomm_spi_slave_dma_tx_done)
{
  logk_debug("DMA TX done, %d", err);

  return 0;
}

static
void flexcomm_spi_slave_dma_enable(struct flexcomm_spi_slave_pv_s *pv, bool_t enable)
{
  uint32_t cfg = flexcomm_get32(pv->base, CFG);
  cfg = flexcomm_ins_val(cfg, CFG, ENABLE, 0);
  flexcomm_set32(pv->base, CFG, cfg);

  uint32_t fifocfg = flexcomm_get32(pv->base, CFG);
  fifocfg = flexcomm_ins_val(fifocfg, FIFOCFG, DMARX, enable);
  fifocfg = flexcomm_ins_val(fifocfg, FIFOCFG, DMATX, enable);
  flexcomm_set32(pv->base, FIFOCFG, fifocfg);

  cfg = flexcomm_ins_val(cfg, CFG, ENABLE, 1);
  flexcomm_set32(pv->base, CFG, cfg);
}

static void flexcomm_spi_slave_dma_transfer_start(struct flexcomm_spi_slave_pv_s *pv)
{
  struct dev_dma_desc_s *desc;
  struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;

  static const uint8_t dma_inc[8] = {
    DEV_DMA_INC_0_UNIT,
    DEV_DMA_INC_1_UNITS,
    DEV_DMA_INC_2_UNITS,
    DEV_DMA_INC_0_UNIT,
    DEV_DMA_INC_4_UNITS,
    DEV_DMA_INC_0_UNIT,
    DEV_DMA_INC_0_UNIT,
    DEV_DMA_INC_0_UNIT,
  };

  flexcomm_spi_slave_dma_enable(pv, 1);
  
  /* TX */
  desc = &pv->dma_tx_desc;
  desc->src.mem.addr = (uintptr_t)tr->data.out;
  desc->src.mem.inc = dma_inc[tr->data.out_width];
  desc->src.mem.size = tr->data.count - 1;

  /* RX */
  desc = &pv->dma_rx_desc;
  desc->src.mem.size = tr->data.count - 1;

  if (tr->data.in != NULL)
    {
      desc->dst.mem.addr = (uintptr_t)tr->data.in;
      desc->dst.mem.inc = dma_inc[tr->data.in_width];
    }
  else
    {
      static uint32_t dummy;
      desc->dst.mem.addr = (uintptr_t)&dummy;
      desc->dst.mem.inc = DEV_DMA_INC_0_UNIT;
    }
    
  if (tr->data.out)
    logk_trace("SPI DMA rq %d %d %P...", tr->data.count, tr->data.out_width, tr->data.out, tr->data.count);
  else
    logk_trace("SPI DMA rq %d %d [NULL]...", tr->data.count, tr->data.out_width);

  /* Start DMA request */ 
  error_t err = DEVICE_OP(&pv->dma, request, &pv->dma_rx_rq, &pv->dma_tx_rq, NULL);

  if (err)
    logk_error("DMA request failure: %d", err);
}
#endif

static
void flexcomm_spi_slave_transfer_start(struct flexcomm_spi_slave_pv_s *pv)
{
  struct dev_spi_ctrl_transfer_s *tr;
  tr = pv->current_transfer;

#if defined(CONFIG_DRIVER_NXP_FLEXCOMM_DMA)
  flexcomm_spi_slave_dma_enable(pv, 0);
#endif
    
  if (tr->data.out)
    logk_trace("SPI rq %d %d %P...", tr->data.count, tr->data.out_width, tr->data.out, tr->data.count);
  else
    logk_trace("SPI rq %d %d [NULL]...", tr->data.count, tr->data.out_width);

  pv->tx_left = tr->data.count;
  pv->rx_left = tr->data.count;

  flexcomm_spi_slave_io_do(pv);
}

static DEV_IRQ_SRC_PROCESS(flexcomm_spi_slave_irq)
{
  struct device_s *dev = ep->base.dev;
  struct flexcomm_spi_slave_pv_s *pv = dev->drv_pv;

  logk_trace("SPI IRQ");

  LOCK_SPIN_SCOPED(&dev->lock);

#if defined(CONFIG_DRIVER_NXP_FLEXCOMM_DMA)
  if (pv->use_dma)
    return;
#endif

  struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;

  if (!tr) {
    flexcomm_rx_irq_enable(pv, 0);
    flexcomm_tx_irq_enable(pv, 0);
    return;
  }

  logk_trace("CFG: %08x", flexcomm_get32(pv->base, CFG));
  logk_trace("STAT: %08x", flexcomm_get32(pv->base, STAT));
  logk_trace("TXDATCTL: %08x", flexcomm_get32(pv->base, FIFOCFG));
  logk_trace("FIFOCFG: %08x", flexcomm_get32(pv->base, FIFOCFG));
  logk_trace("FIFOSTAT: %08x", flexcomm_get32(pv->base, FIFOSTAT));
  logk_trace("INTEN: %08x", flexcomm_get32(pv->base, INTENSET));
  logk_trace("INT: %08x", flexcomm_get32(pv->base, INTSTAT));
  logk_trace("FIFOSIZE: %08x", flexcomm_get32(pv->base, FIFOSIZE));

  flexcomm_spi_slave_io_do(pv);
}

static DEV_SPI_CTRL_TRANSFER(flexcomm_spi_slave_transfer)
{
  struct device_s *dev = accessor->dev;
  struct flexcomm_spi_slave_pv_s *pv = dev->drv_pv;

  tr->err = 0;

  {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    if (pv->current_transfer != NULL) {
      tr->err = -EBUSY;
    } else if (tr->cs_op != DEV_SPI_CS_NOP_NOP) {
      tr->err = -ENOTSUP;
    } else if (tr->data.count > 0) {
      pv->current_transfer = tr;

#if defined(CONFIG_DRIVER_NXP_FLEXCOMM_DMA)
      pv->use_dma = tr->data.count > 8 && pv->dma_available;
            
      if (pv->use_dma)
        flexcomm_spi_slave_dma_transfer_start(pv);
      else
        flexcomm_spi_slave_transfer_start(pv);
#else
      flexcomm_spi_slave_transfer_start(pv);
#endif

      tr = NULL;
    }
  }

  if (tr)
    kroutine_exec(&tr->kr);
}

static DEV_USE(flexcomm_spi_slave_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct flexcomm_spi_slave_pv_s *pv = dev->drv_pv;

      pv->freq = chg->freq;
      flexcomm_spi_slave_rate_update(pv);

      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}

#define flexcomm_spi_slave_cscfg (dev_spi_ctrl_cscfg_t*)dev_driver_notsup_fcn

static DEV_INIT(flexcomm_spi_slave_init)
{
  struct flexcomm_spi_slave_pv_s *pv;
  uint32_t addr, tmp;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    goto free_pv;

  pv->base = addr;
  pv->current_transfer = NULL;
  pv->rate_1k = 1000;
    
#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_context_init(dev, &pv->spi_ctrl_ctx))
    goto free_pv;
#endif

  if (device_iomux_setup(dev, ">clk <miso? >mosi?", NULL, NULL, NULL))
    goto free_queue;

  device_irq_source_init(dev, pv->irq_ep, 1, &flexcomm_spi_slave_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_queue;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, 0
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
                         | DEV_CLOCK_EP_FREQ_NOTIFY
#endif
                         | DEV_CLOCK_EP_POWER_CLOCK
                         | DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto free_irq;

#if defined(CONFIG_DRIVER_NXP_FLEXCOMM_DMA)
  uint32_t rx_link, tx_link;
  uint32_t rx_mask, tx_mask;

  if (device_res_get_dma(dev, NXP_FLEXCOMM_DMA_RX, &rx_mask, &rx_link) ||
      device_res_get_dma(dev, NXP_FLEXCOMM_DMA_TX, &tx_mask, &tx_link) ||
      device_get_param_dev_accessor(dev, "dma", &pv->dma.base, DRIVER_CLASS_DMA)) {

    logk_warning("DMA unavailable");
    pv->dma_available = 0;
  } else {
    /* rx */
    struct dev_dma_rq_s *rq = &pv->dma_rx_rq;
    struct dev_dma_desc_s *desc = &pv->dma_rx_desc;

    desc->src.reg.addr = pv->base + FLEXCOMM_RXDAT_OFFSET;
    desc->src.reg.width = 0;
    desc->src.reg.burst = 1;

    rq->dev_link.src = rx_link;
    rq->type = DEV_DMA_REG_MEM;
    rq->desc_count_m1 = 0;
    rq->loop_count_m1 = 0;
    rq->chan_mask = rx_mask;
    rq->f_done = flexcomm_spi_slave_dma_rx_done;
    rq->cache_ptr = NULL;

    /* tx */
    rq = &pv->dma_tx_rq;
    desc = &pv->dma_tx_desc;

    desc->dst.reg.addr = pv->base + FLEXCOMM_TXDAT_OFFSET;
    desc->dst.reg.burst = 1;
    desc->src.mem.width = 0;

    rq->dev_link.dst = tx_link;
    rq->type = DEV_DMA_MEM_REG;
    rq->desc_count_m1 = 0;
    rq->loop_count_m1 = 0;
    rq->chan_mask = tx_mask;
    rq->f_done = flexcomm_spi_slave_dma_tx_done;
    rq->cache_ptr = NULL;
  }
#endif

  flexcomm_set32(pv->base, CFG, 0);
  flexcomm_set32(pv->base, PSELID, flexcomm_ins_enum(0, PSELID, PERSEL, SPI));

  tmp = flexcomm_get32(pv->base, ID);
  logk("Flexcomm ID 0x%04x v. %d.%d",
       flexcomm_get(tmp, ID, ID),
       flexcomm_get(tmp, ID, MAJOR_REV),
       flexcomm_get(tmp, ID, MINOR_REV)
       );
  tmp = flexcomm_get32(pv->base, PSELID);
  logk("Has %s%s%s%sID: %06x",
       flexcomm_get(tmp, PSELID, I2SPRESENT) ? "I2S, " : "",
       flexcomm_get(tmp, PSELID, I2CPRESENT) ? "I2C, " : "",
       flexcomm_get(tmp, PSELID, SPIPRESENT) ? "SPI, " : "",
       flexcomm_get(tmp, PSELID, USARTPRESENT) ? "USART, " : "",
       flexcomm_get(tmp, PSELID, ID)
       );
  tmp = flexcomm_get32(pv->base, FIFOSIZE);
  logk("Has %d FIFO entries",
       flexcomm_get(tmp, FIFOSIZE, SIZE)
       );
    
  flexcomm_set32(pv->base, CFG, flexcomm_ins_val(0, CFG, MASTER, 1));
  flexcomm_set32(pv->base, DIV, pv->freq.num / pv->freq.denom / 10000000);
  flexcomm_set32(pv->base, DLY, 0);
  flexcomm_set32(pv->base, FIFOCFG, 0
                 | flexcomm_bit(FIFOCFG, ENABLETX)
                 | flexcomm_bit(FIFOCFG, ENABLERX)
                 // This is one-shot to clear fifos
                 | flexcomm_bit(FIFOCFG, EMPTYRX)
                 | flexcomm_bit(FIFOCFG, EMPTYTX));
  flexcomm_set32(pv->base, FIFOTRIG, 0
                 // Txn start will set this up
                 );
  flexcomm_set32(pv->base, FIFOSTAT, -1);
  flexcomm_set32(pv->base, TXCTL, flexcomm_ins_val(0, TXCTL, LEN, 8));
  flexcomm_set32(pv->base, TXDATCTL, flexcomm_ins_val(0, TXDATCTL, FLEN, 8-1));
  flexcomm_set32(pv->base, CFG, 0
                 | flexcomm_ins_val(0, CFG, MASTER, 1)
                 | flexcomm_ins_val(0, CFG, ENABLE, 1));

  return 0;

 free_irq:
 free_queue:
#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif
 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(flexcomm_spi_slave_cleanup)
{
  struct flexcomm_spi_slave_pv_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (!dev_rq_queue_isempty(&pv->spi_ctrl_ctx.queue))
    return -EBUSY;

  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  flexcomm_set32(pv->base, FIFOTRIG, 0);
  flexcomm_set32(pv->base, FIFOSTAT, -1);
  flexcomm_set32(pv->base, CFG, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(flexcomm_spi_slave_drv, 0, "Flexcomm SPI", flexcomm_spi,
               DRIVER_SPI_CTRL_METHODS(flexcomm_spi));

DRIVER_REGISTER(flexcomm_spi_slave_drv);
