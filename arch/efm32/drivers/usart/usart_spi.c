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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/spi.h>
#include <device/class/timer.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)
#include <device/class/dma.h>
#include <arch/efm32/dma_source.h>
#include <arch/efm32/dma_request.h>
#endif

#ifdef CONFIG_DRIVER_EFM32_DMA
#include <cpu/arm32m/pl230_channel.h>
#endif

#include <arch/efm32/irq.h>
#include <arch/efm32/usart.h>

#define EFM32_USART_FIFO_SIZE 2

DRIVER_PV(struct efm32_usart_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s           irq_ep;
#endif
  struct dev_spi_ctrl_transfer_s *tr;

  struct dev_spi_ctrl_context_s  spi_ctrl_ctx;

  struct dev_freq_s              freq;

  struct dev_clock_sink_ep_s     clk_ep;

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)
  struct dev_dma_rq_s            dma_rd_rq;
  struct dev_dma_desc_s          dma_rd_desc;
  DEV_DMA_RQ_TYPE(1)             dma_wr_rq; 
  struct device_dma_s            dma;
  struct device_s                *spi;
#endif

  uint32_t                       ctrl;
  uint32_t                       BITFIELD(clkdiv,24);
  uint32_t                       BITFIELD(frame,8);
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  uint32_t                       route;
  uint32_t                       enable;
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  uint16_t                       route;
#else
# error
#endif
  uint16_t                       bit_rate1k;
  uint8_t                        fifo_lvl;
  bool_t                         dma_use;
});

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)
STRUCT_COMPOSE(efm32_usart_spi_context_s, dma_rd_rq);
#endif

static void efm32_usart_spi_update_rate(struct device_s *dev, uint32_t bit_rate1k)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  pv->bit_rate1k = bit_rate1k;
  uint64_t d = (128 * pv->freq.num) / (bit_rate1k * 1024 * pv->freq.denom);
  pv->clkdiv = d < 256 ? 0 : (d >> 20 ? 0x1fffc0 : d - 256);
}

static DEV_SPI_CTRL_CONFIG(efm32_usart_spi_config)
{
  struct device_s *dev = accessor->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;
  else
    {
      if (cfg->word_width < 4 || cfg->word_width > 8)
        err = -ENOTSUP;
      else
        {
          pv->frame = cfg->word_width - 3;
          pv->ctrl = EFM32_USART_CTRL_SYNC;

          if (cfg->ck_mode == DEV_SPI_CK_MODE_2 || cfg->ck_mode == DEV_SPI_CK_MODE_3)
            pv->ctrl |= EFM32_USART_CTRL_CLKPOL;
          if (cfg->ck_mode == DEV_SPI_CK_MODE_1 || cfg->ck_mode == DEV_SPI_CK_MODE_3)
            pv->ctrl |= EFM32_USART_CTRL_CLKPHA;
          if (cfg->miso_pol == DEV_SPI_ACTIVE_LOW)
            pv->ctrl |= EFM32_USART_CTRL_RXINV;
          if (cfg->mosi_pol == DEV_SPI_ACTIVE_LOW)
            pv->ctrl |= EFM32_USART_CTRL_TXINV;
          if (cfg->bit_order == DEV_SPI_MSB_FIRST)
            pv->ctrl |= EFM32_USART_CTRL_MSBF;
          if (pv->bit_rate1k != cfg->bit_rate1k)
            efm32_usart_spi_update_rate(dev, cfg->bit_rate1k);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static bool_t efm32_usart_spi_transfer_tx(struct device_s *dev);

static bool_t efm32_usart_spi_transfer_rx(struct device_s *dev)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (pv->fifo_lvl > 0)
    {
      uint32_t st = cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
                      & endian_le32(EFM32_USART_STATUS_RXDATAV);

      if (!st)
#ifdef CONFIG_DEVICE_IRQ
        break;
#else
        continue;
#endif


      uint32_t word = (uint8_t)endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_RXDATA_ADDR));

      pv->fifo_lvl--;

      if (tr->data.in == NULL)
        continue;

      switch (tr->data.in_width)
        {
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



  if (tr->data.count > 0 || pv->fifo_lvl)
    return efm32_usart_spi_transfer_tx(dev);

  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

# ifdef CONFIG_DEVICE_CLOCK_GATING
  if (dev->start_count == 1)
    dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
# endif

  pv->tr = NULL;
  dev->start_count &= ~1;

  return 1;
}

static bool_t efm32_usart_spi_transfer_tx(struct device_s *dev)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  if (tr->data.count < EFM32_USART_FIFO_SIZE)
    cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, endian_le32(EFM32_USART_IEN_RXDATAV));

  while (tr->data.count > 0 && pv->fifo_lvl < EFM32_USART_FIFO_SIZE)
    {
      uint32_t word = 0;
      switch (tr->data.out_width)
        {
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

      cpu_mem_write_32(pv->addr + EFM32_USART_TXDATA_ADDR, endian_le32(word));

      tr->data.out = (const void*)((const uint8_t*)tr->data.out + tr->data.out_width);
      tr->data.count--;
      pv->fifo_lvl++;
    }

#ifdef CONFIG_DEVICE_IRQ
  return 0;
#else
  return efm32_usart_spi_transfer_rx(dev);
#endif
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(efm32_usart_spi_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
#ifdef CONFIG_DEVICE_CLOCK_GATING
      if (!dev->start_count)
        break;
#endif
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_IF_ADDR));

      if (!(x & endian_le32(EFM32_USART_IF_RXDATAV)))
        break;

      cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)
      if (pv->dma_use)
        break;
#endif

      struct dev_spi_ctrl_transfer_s *tr = pv->tr;

      assert(tr);
      assert(pv->fifo_lvl > 0);

      if (efm32_usart_spi_transfer_rx(dev))
        kroutine_exec(&tr->kr);
    }

 end:
  lock_release(&dev->lock);
}

#endif

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)

static DEV_DMA_CALLBACK(efm32_spi_dma_read_done)
{
  struct efm32_usart_spi_context_s *pv = 
    efm32_usart_spi_context_s_from_dma_rd_rq(rq);
  
  lock_spin(&pv->spi->lock);

  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  pv->tr = NULL;

  pv->spi->start_count &= ~1;
# ifdef CONFIG_DEVICE_CLOCK_GATING
  if (pv->spi->start_count == 0)
    dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
# endif

  lock_release(&pv->spi->lock);

  kroutine_exec(&tr->kr);
  return 0;
}

static DEV_DMA_CALLBACK(efm32_spi_dma_write_done)
{
  return 0;
}

static void efm32_usart_spi_start_dma(struct efm32_usart_spi_context_s *pv)
{
  pv->dma_use = 1;

  struct dev_dma_desc_s * desc = pv->dma_wr_rq.desc;
  struct dev_spi_ctrl_transfer_s * __restrict__ tr = pv->tr;

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

  /* TX */
  desc->src.mem.addr = (uintptr_t)tr->data.out;
  desc->src.mem.inc = dma_inc[tr->data.out_width];
  desc->src.mem.size = tr->data.count - 1;

  /* RX */
  desc = &pv->dma_rd_desc;
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

  /* Start DMA request */ 
  DEVICE_OP(&pv->dma, request, &pv->dma_rd_rq, &pv->dma_wr_rq, NULL);
}
#endif

static DEV_SPI_CTRL_TRANSFER(efm32_usart_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;

  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  tr->err = 0;

  if (pv->tr != NULL)
    tr->err = -EBUSY;
  else if (tr->cs_op != DEV_SPI_CS_NOP_NOP)
    tr->err = -ENOTSUP;
  else if (tr->data.count > 0)
    {
      pv->tr = tr;
      dev->start_count |= 1;

# ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
# endif

      cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, endian_le32(pv->clkdiv));
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
      cpu_mem_write_32(pv->addr + EFM32_USART_ROUTELOC0_ADDR, endian_le32(pv->route));
      cpu_mem_write_32(pv->addr + EFM32_USART_ROUTEPEN_ADDR, endian_le32(pv->enable));
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
      cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, endian_le32(pv->route));
#else
# error
#endif
      cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
      cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR, endian_le32(pv->frame));

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)
      pv->dma_use = 0;

      done = 0;

      if (tr->data.count > CONFIG_DRIVER_EFM32_USART_DMA_THRD)
        efm32_usart_spi_start_dma(pv);
      else
#endif
        {
          cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));
          cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, endian_le32(EFM32_USART_IEN_RXFULL));

          pv->fifo_lvl = 0;

          done = efm32_usart_spi_transfer_tx(dev);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr);
}

static DEV_USE(efm32_usart_spi_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_usart_spi_context_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      efm32_usart_spi_update_rate(dev, pv->bit_rate1k);
      return 0;
    }
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_usart_spi_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_usart_spi_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(efm32_usart_spi_init)
{
  struct efm32_usart_spi_context_s	*pv;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_FREQ_NOTIFY |
                     DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto err_mem;

  /* init state */
  pv->tr = NULL;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_context_init(dev, &pv->spi_ctrl_ctx))
    goto err_clk;
#endif

  /* disable and clear irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS |
                               EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

  /* synchronous mode, 8 bits */
  pv->ctrl = EFM32_USART_CTRL_SYNC;
  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
  pv->frame = 8 - 3;

  /* setup pinmux */
  iomux_demux_t loc[4];
  if (device_iomux_setup(dev, ">clk <miso? >mosi? >cs?", loc, NULL, NULL))
    goto err_clk;

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  if (loc[0] != IOMUX_INVALID_DEMUX)
    {
      pv->enable |= EFM32_USART_ROUTEPEN_CLKPEN;
      EFM32_USART_ROUTELOC0_CLKLOC_SETVAL(pv->route, loc[0]);
    }
  if (loc[1] != IOMUX_INVALID_DEMUX)
    {
      pv->enable |= EFM32_USART_ROUTEPEN_RXPEN;
      EFM32_USART_ROUTELOC0_RXLOC_SETVAL(pv->route, loc[1]);
    }
  if (loc[2] != IOMUX_INVALID_DEMUX)
    {
      pv->enable |= EFM32_USART_ROUTEPEN_TXPEN;
      EFM32_USART_ROUTELOC0_TXLOC_SETVAL(pv->route, loc[2]);
    }
  if (loc[3] != IOMUX_INVALID_DEMUX)
    {
      pv->enable |= EFM32_USART_ROUTEPEN_CSPEN;
      EFM32_USART_ROUTELOC0_CSLOC_SETVAL(pv->route, loc[3]);
    }

  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTELOC0_ADDR, endian_le32(pv->route));
  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTEPEN_ADDR, endian_le32(pv->enable));

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  pv->route =  EFM32_USART_ROUTE_CLKPEN;
  if (loc[1] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_RXPEN;
  if (loc[2] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_TXPEN;
  if (loc[3] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_CSPEN;

  EFM32_USART_ROUTE_LOCATION_SETVAL(pv->route, loc[0]);

  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, endian_le32(pv->route));
#else
# error
#endif

  /* setup bit rate */
  pv->bit_rate1k = 100;
  efm32_usart_spi_update_rate(dev, pv->bit_rate1k);
  cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, endian_le32(pv->clkdiv));

  /* enable the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXEN | EFM32_USART_CMD_TXEN |
                               EFM32_USART_CMD_MASTEREN));

  /* init irq endpoint */
#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_usart_spi_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_clk;
#endif

#if defined(CONFIG_DRIVER_EFM32_DMA) || defined(CONFIG_DRIVER_EFR32_DMA)

  uint32_t read_link, write_link;
  uint32_t read_mask, write_mask;

  pv->spi = dev;

  if (device_res_get_dma(dev, 0, &read_mask, &read_link) ||
      device_res_get_dma(dev, 1, &write_mask, &write_link) ||
      device_get_param_dev_accessor(dev, "dma", &pv->dma.base, DRIVER_CLASS_DMA))
    goto err_irq;

  /* READ */
  struct dev_dma_rq_s *rq = &pv->dma_rd_rq;
  struct dev_dma_desc_s *desc = rq->desc;

  desc->src.reg.addr = pv->addr + EFM32_USART_RXDATA_ADDR;
  desc->src.reg.width = 0;
  desc->src.reg.burst = 1;

  rq->dev_link.src = read_link | (EFM32_DMA_SIGNAL_USARTRXDATAV << 8);
  rq->type = DEV_DMA_REG_MEM;
  rq->desc_count_m1 = 0;
  rq->loop_count_m1 = 0;
  rq->chan_mask = read_mask;
  rq->f_done = efm32_spi_dma_read_done;
  rq->cache_ptr = NULL;

  /* WRITE */
  rq = &pv->dma_wr_rq.rq;
  desc = pv->dma_wr_rq.desc;

  desc->dst.reg.addr = pv->addr + EFM32_USART_TXDATA_ADDR;
  desc->dst.reg.burst = 1;
  desc->src.mem.width = 0;

  rq->dev_link.dst = write_link | (EFM32_DMA_SIGNAL_USARTTXBL << 8);
  rq->type = DEV_DMA_MEM_REG;
  rq->desc_count_m1 = 0;
  rq->loop_count_m1 = 0;
  rq->chan_mask = write_mask;
  rq->f_done = efm32_spi_dma_write_done;
  rq->cache_ptr = NULL;

#endif

# ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
# endif

  return 0;

 err_irq:
#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
#endif
 err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_usart_spi_cleanup)
{
  struct efm32_usart_spi_context_s	*pv = dev->drv_pv;

  if (pv->tr != NULL)
    return -EBUSY;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
#endif

  /* disable the usart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

#define efm32_usart_spi_cscfg (dev_spi_ctrl_cscfg_t*)dev_driver_notsup_fcn

DRIVER_DECLARE(efm32_usart_spi_drv, 0, "EFM32 USART (SPI)", efm32_usart_spi,
               DRIVER_SPI_CTRL_METHODS(efm32_usart_spi));

DRIVER_REGISTER(efm32_usart_spi_drv);

