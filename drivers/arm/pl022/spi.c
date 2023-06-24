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

#define LOGK_MODULE_ID "fspi"

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

#include "pl022.h"

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
#include <device/class/dma.h>
#endif

struct pl022_spi_pv_s
{
  uint32_t base;

  struct dev_clock_sink_ep_s clk_ep;
  struct dev_freq_s freq;
  uint32_t rate_1k;

  struct dev_irq_src_s irq_ep[1];
  struct dev_spi_ctrl_transfer_s *current_transfer;
  struct dev_spi_ctrl_context_s spi_ctrl_ctx;

  size_t tx_left, rx_left;

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
  struct dev_dma_rq_s dma_rx_rq;
  struct dev_dma_desc_s dma_rx_desc;
  struct dev_dma_rq_s dma_tx_rq;
  struct dev_dma_desc_s dma_tx_desc;
  struct device_dma_s dma;

  bool_t dma_available;
  bool_t use_dma;
#endif
};

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
STRUCT_COMPOSE(pl022_spi_pv_s, dma_rx_rq);
STRUCT_COMPOSE(pl022_spi_pv_s, dma_tx_rq);
#endif

DRIVER_PV(struct pl022_spi_pv_s);

static
bool_t pl022_txfifo_is_full(struct pl022_spi_pv_s *pv)
{
  uint32_t sspsr = pl022_get32(pv->base, SSPSR);
  return !pl022_get(sspsr, SSPSR, TNF);
}

static
bool_t pl022_rxfifo_is_empty(struct pl022_spi_pv_s *pv)
{
  uint32_t sspsr = pl022_get32(pv->base, SSPSR);
  return !pl022_get(sspsr, SSPSR, RNE);
}

static
void pl022_rx_irq_enable(struct pl022_spi_pv_s *pv,
                            bool_t en)
{
  uint32_t sspimsc = pl022_get32(pv->base, SSPIMSC);
  sspimsc = pl022_ins_val(sspimsc, SSPIMSC, RXIM, en);
  pl022_set32(pv->base, SSPIMSC, sspimsc);
}

static
void pl022_tx_irq_enable(struct pl022_spi_pv_s *pv,
                            bool_t en)
{
  uint32_t sspimsc = pl022_get32(pv->base, SSPIMSC);
  sspimsc = pl022_ins_val(sspimsc, SSPIMSC, TXIM, en);
  pl022_set32(pv->base, SSPIMSC, sspimsc);
}

static
void pl022_spi_rate_update(struct pl022_spi_pv_s *pv)
{
  uint32_t div = (pv->freq.num) / (pv->freq.denom * 1024 * pv->bit_rate1k);
  pl022_set32(pv->base, CPSR, div);
}

static
DEV_SPI_CTRL_CONFIG(pl022_spi_config)
{
  struct device_s *dev = accessor->dev;
  struct pl022_spi_pv_s *pv = dev->drv_pv;

  if (cfg->word_width != 8)
    return -ENOTSUP;

  if (cfg->miso_pol != DEV_SPI_ACTIVE_LOW
      || cfg->mosi_pol != DEV_SPI_ACTIVE_LOW)
    return -ENOTSUP;

  if (cfg->bit_order != DEV_SPI_MSB_FIRST)
    return -ENOTSUP;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->tr != NULL)
    return -EBUSY;

  uint32_t sspcr1 = pl022_get32(pv->base, SSPCR1);
  pl022_set32(pv->base, SSPCR1, pl022_ins_val(sspcr1, SSPCR1, SSE, 0));

  uint32_t sspcr0 = 0;
  sspcr0 = pl022_ins_val(sspcr0, SSPCR0, SCR, cfg->word_width - 1);
  sspcr0 = pl022_ins_val(sspcr0, SSPCR0, SPO, !!(cfg->ck_mode & 2));
  sspcr0 = pl022_ins_val(sspcr0, SSPCR0, SPH, !!(cfg->ck_mode & 1));
  sspcr0 = pl022_ins_enum(sspcr0, SSPCR0, FRF, MOTOROLA);
  pl022_set32(pv->base, SSPCR0, sspcr0);

  if (pv->bit_rate1k != cfg->bit_rate1k)
    {
      pv->bit_rate1k = cfg->bit_rate1k;
      pl022_spi_rate_update(pv);
    }

  pl022_set32(pv->base, SSPCR1, sspcr1);

  return 0;
}

static
void pl022_spi_tr_put_one(struct pl022_spi_pv_s *pv,
                          struct dev_spi_ctrl_transfer_s *tr)
{
  uint8_t word = 0xff;

  if (tr->data.out) {
    switch (tr->data.out_width) {
    case 1:
      word = *(const uint8_t*)tr->data.out;
      break;
    case 2:
      word = *(const uint16_t*)tr->data.out;
      break;
    case 0:
    case 4:
      word = *(const uint32_t*)tr->data.out;
      break;
    }

    tr->data.out = (const void*)((uintptr_t)tr->data.out + tr->data.out_width);
  }
    
  logk_trace("SPI tx: %02x", word);
  pl022_set32(pv->base, SSPDR, word);
}

static
void pl022_spi_tr_get_one(struct pl022_spi_pv_s *pv,
                          struct dev_spi_ctrl_transfer_s *tr)
{
  uint8_t word = pl022_get32(pv->base, SSPDR) & 0xff;

  logk_trace("SPI rx: %02x", word);

  if (tr->data.in) {
    switch (tr->data.in_width) {
    case 1:
      *(uint8_t*)tr->data.in = word;
      break;
    case 2:
      *(uint16_t*)tr->data.in = word;
      break;
    case 4:
      *(uint32_t*)tr->data.in = word;
      break;
    }

    tr->data.in = (void*)((uint8_t*)tr->data.in + tr->data.in_width);
  }
}

static
void pl022_spi_io_do(struct pl022_spi_pv_s *pv)
{
  struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;
  assert(tr);
  bool_t again = 1;

  while (again) {
    again = 0;

    if (pv->rx_left && !pl022_rxfifo_is_empty(pv)) {
      pl022_spi_tr_get_one(pv, tr);
      pv->rx_left--;
      again = 1;
    }

    if (pv->tx_left && !pl022_txfifo_is_full(pv)) {
      pl022_spi_tr_put_one(pv, tr);
      pv->tx_left--;
      again = 1;
    }
  }

  pl022_rx_irq_enable(pv, pv->rx_left != 0);
  pl022_tx_irq_enable(pv, pv->tx_left != 0);

  if (!pv->rx_left && !pv->tx_left) {
    tr->data.count = 0;
    tr->err = 0;
    pv->current_transfer = NULL;
    kroutine_exec(&tr->kr);
  }

  logk_trace("Left rx: %d, tx: %d", pv->rx_left, pv->tx_left);
  logk_trace("SSPSR: %08x", pl022_get32(pv->base, SSPSR));
}

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
static DEV_DMA_CALLBACK(pl022_spi_dma_rx_done)
{
  struct pl022_spi_pv_s *pv = pl022_spi_pv_s_from_dma_rx_rq(rq);
  struct device_s *dev = pv->irq_ep->base.dev;
  struct dev_spi_ctrl_transfer_s *tr;

  {
    LOCK_SPIN_SCOPED(&dev->lock);
    tr = pv->current_transfer;
    pv->current_transfer = NULL;
  }
    
  logk_debug("DMA RX done, %d", err);

  if (tr) {
    if (tr->data.out)
      tr->data.out += tr->data.count;
    if (tr->data.in)
      tr->data.in += tr->data.count;
    tr->data.count = 0;
    kroutine_exec(&tr->kr);
  }
  return 0;
}

static DEV_DMA_CALLBACK(pl022_spi_dma_tx_done)
{
  logk_debug("DMA TX done, %d", err);

  return 0;
}

static
void pl022_spi_dma_enable(struct pl022_spi_pv_s *pv, bool_t enable)
{
  uint32_t sspcr1 = pl022_get32(pv->base, SSPCR1);
  pl022_set32(pv->base, SSPCR1, pl022_ins_val(sspcr1, SSPCR1, SSE, 0));

  uint32_t sspdmacr = pl022_get32(pv->base, CFG);
  sspdmacr = pl022_ins_val(sspdmacr, SSPDMACR, TXDMAE, enable);
  sspdmacr = pl022_ins_val(sspdmacr, SSPDMACR, RXDMAE, enable);
  pl022_set32(pv->base, SSPDMACR, sspdmacr);

  pl022_set32(pv->base, SSPCR1, sspcr1);
}

static void pl022_spi_dma_transfer_start(struct pl022_spi_pv_s *pv)
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

  pl022_spi_dma_enable(pv, 1);
  
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
void pl022_spi_transfer_start(struct pl022_spi_pv_s *pv)
{
  struct dev_spi_ctrl_transfer_s *tr;
  tr = pv->current_transfer;

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
  pl022_spi_dma_enable(pv, 0);
#endif
    
  if (tr->data.out)
    logk_trace("SPI rq %d %d %P...", tr->data.count, tr->data.out_width, tr->data.out, tr->data.count);
  else
    logk_trace("SPI rq %d %d [NULL]...", tr->data.count, tr->data.out_width);

  pv->tx_left = tr->data.count;
  pv->rx_left = tr->data.count;

  pl022_spi_io_do(pv);
}

static DEV_IRQ_SRC_PROCESS(pl022_spi_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pl022_spi_pv_s *pv = dev->drv_pv;

  logk_trace("SPI IRQ");

  LOCK_SPIN_SCOPED(&dev->lock);

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
  if (pv->use_dma)
    return;
#endif

  struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;

  if (!tr) {
    pl022_rx_irq_enable(pv, 0);
    pl022_tx_irq_enable(pv, 0);
    return;
  }

  pl022_spi_io_do(pv);
}

static DEV_SPI_CTRL_TRANSFER(pl022_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct pl022_spi_pv_s *pv = dev->drv_pv;

  tr->err = 0;

  {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    if (pv->current_transfer != NULL) {
      tr->err = -EBUSY;
    } else if (tr->cs_op != DEV_SPI_CS_NOP_NOP) {
      tr->err = -ENOTSUP;
    } else if (tr->data.count > 0) {
      pv->current_transfer = tr;

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
      pv->use_dma = tr->data.count > 8 && pv->dma_available;
            
      if (pv->use_dma)
        pl022_spi_dma_transfer_start(pv);
      else
        pl022_spi_transfer_start(pv);
#else
      pl022_spi_transfer_start(pv);
#endif

      tr = NULL;
    }
  }

  if (tr)
    kroutine_exec(&tr->kr);
}

static DEV_USE(pl022_spi_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct pl022_spi_pv_s *pv = dev->drv_pv;

      pv->freq = chg->freq;
      pl022_spi_rate_update(pv);

      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}

#define pl022_spi_cscfg (dev_spi_ctrl_cscfg_t*)dev_driver_notsup_fcn

static DEV_INIT(pl022_spi_init)
{
  struct pl022_spi_pv_s *pv;
  uint32_t addr, tmp;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    goto free_pv;

  struct arm_identification_s id;
  arm_identification_read(&id, base);
  arm_identification_dump(&id);

  assert((id.pid0 & 0xffffe) == 0x41022);
  uint8_t rev = bit_get_mask(id.pid0, 20, 4);
  uint16_t pl_code = bit_get_mask(id.pid0, 0, 12);
  logk("PL%03x rev %d", pl_code, rev);

  pv->base = addr;
  pv->current_transfer = NULL;
  pv->rate_1k = 1000;
    
#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_context_init(dev, &pv->spi_ctrl_ctx))
    goto free_pv;
#endif

  if (device_iomux_setup(dev, ">clk <miso? >mosi?", NULL, NULL, NULL))
    goto free_queue;

  device_irq_source_init(dev, pv->irq_ep, 1, &pl022_spi_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_queue;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, 0
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
                         | DEV_CLOCK_EP_FREQ_NOTIFY
#endif
                         | DEV_CLOCK_EP_POWER_CLOCK
                         | DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto free_irq;

#if defined(CONFIG_DRIVER_ARM_PL022_DMA)
  uint32_t rx_link, tx_link;
  uint32_t rx_mask, tx_mask;

  if (device_res_get_dma(dev, ARM_PL022_DMA_RX, &rx_mask, &rx_link) ||
      device_res_get_dma(dev, ARM_PL022_DMA_TX, &tx_mask, &tx_link) ||
      device_get_param_dev_accessor(dev, "dma", &pv->dma.base, DRIVER_CLASS_DMA)) {

    logk_warning("DMA unavailable");
    pv->dma_available = 0;
  } else {
    /* rx */
    struct dev_dma_rq_s *rq = &pv->dma_rx_rq;
    struct dev_dma_desc_s *desc = &pv->dma_rx_desc;

    desc->src.reg.addr = pv->base + PL022_RXDAT_OFFSET;
    desc->src.reg.width = 0;
    desc->src.reg.burst = 1;

    rq->dev_link.src = rx_link;
    rq->type = DEV_DMA_REG_MEM;
    rq->desc_count_m1 = 0;
    rq->loop_count_m1 = 0;
    rq->chan_mask = rx_mask;
    rq->f_done = pl022_spi_dma_rx_done;
    rq->cache_ptr = NULL;

    /* tx */
    rq = &pv->dma_tx_rq;
    desc = &pv->dma_tx_desc;

    desc->dst.reg.addr = pv->base + PL022_TXDAT_OFFSET;
    desc->dst.reg.burst = 1;
    desc->src.mem.width = 0;

    rq->dev_link.dst = tx_link;
    rq->type = DEV_DMA_MEM_REG;
    rq->desc_count_m1 = 0;
    rq->loop_count_m1 = 0;
    rq->chan_mask = tx_mask;
    rq->f_done = pl022_spi_dma_tx_done;
    rq->cache_ptr = NULL;
  }
#endif

  uint32_t sspcr1 = 0;
  pl022_set32(pv->base, SSPCR1, sspcr1);

  uint32_t sspcr0 = 0;
  sspcr0 = pl022_ins_val(sspcr0, SSPCR0, SCR, 7);
  sspcr0 = pl022_ins_val(sspcr0, SSPCR0, SPO, 0);
  sspcr0 = pl022_ins_val(sspcr0, SSPCR0, SPH, 0));
  sspcr0 = pl022_ins_enum(sspcr0, SSPCR0, FRF, MOTOROLA);
  pl022_set32(pv->base, SSPCR0, sspcr0);

  pl022_spi_rate_update(pv);

  pl022_set32(pv->base, SSPCR1, pl022_ins_val(sspcr1, SSPCR1, SSE, 1));
  pl022_set32(pv->base, SSPCR1, sspcr1);

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

static DEV_CLEANUP(pl022_spi_cleanup)
{
  struct pl022_spi_pv_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (!dev_rq_queue_isempty(&pv->spi_ctrl_ctx.queue))
    return -EBUSY;

  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  pl022_set32(pv->base, FIFOTRIG, 0);
  pl022_set32(pv->base, SSPSR, -1);
  pl022_set32(pv->base, CFG, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(pl022_spi_drv, 0, "ARM PL022 SPI", pl022_spi,
               DRIVER_SPI_CTRL_METHODS(pl022_spi));

DRIVER_REGISTER(pl022_spi_drv);
