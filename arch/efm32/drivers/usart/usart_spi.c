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
#include <device/class/clock.h>

#ifdef CONFIG_DRIVER_EFM32_DMA
#include <device/class/dma.h>

#include <cpu/pl230_channel.h>
#include <arch/efm32_dma_source.h>
#include <arch/efm32_dma_request.h>
#endif

#include <arch/efm32_irq.h>
#include <arch/efm32_usart.h>

#define EFM32_USART_FIFO_SIZE 2

struct efm32_usart_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s           irq_ep;
#endif
  struct dev_spi_ctrl_transfer_s *tr;
  uint32_t                       ctrl;
  uint32_t                       route;
  uint_fast8_t                   fifo_lvl;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_queue_s    queue;
#endif

  struct dev_freq_s              freq;
  uint32_t                       bit_rate;

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s     clk_ep;
#endif

#ifdef CONFIG_DRIVER_EFM32_DMA
  bool_t                         dma_use;
  struct device_dma_s            dma;
  struct efm32_dev_dma_rq_s      drq; 
#endif

};

static void efm32_usart_spi_update_rate(struct efm32_usart_spi_context_s *pv)
{
  uint32_t div = (128 * pv->freq.num) / (pv->bit_rate * pv->freq.denom) - 256;
  cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, endian_le32(div));
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
          cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR, endian_le32(cfg->word_width - 3));

          EFM32_USART_CTRL_CLKPOL_SETVAL(pv->ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_2 ||
                                                   cfg->ck_mode == DEV_SPI_CK_MODE_3);
          EFM32_USART_CTRL_CLKPHA_SETVAL(pv->ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_1 ||
                                                   cfg->ck_mode == DEV_SPI_CK_MODE_3);

          EFM32_USART_CTRL_RXINV_SETVAL(pv->ctrl, cfg->miso_pol == DEV_SPI_ACTIVE_LOW);
          EFM32_USART_CTRL_TXINV_SETVAL(pv->ctrl, cfg->mosi_pol == DEV_SPI_ACTIVE_LOW);
          EFM32_USART_CTRL_MSBF_SETVAL(pv->ctrl,  cfg->bit_order == DEV_SPI_MSB_FIRST);

          cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));

          if (pv->bit_rate != cfg->bit_rate)
            {
              pv->bit_rate = cfg->bit_rate;
              efm32_usart_spi_update_rate(pv);
            }
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
        return 0;           /* wait for more rx irq */
#else
        continue;
#endif

      uint32_t word = (uint8_t)endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_RXDATA_ADDR));

      pv->fifo_lvl--;

      if (tr->in == NULL)
        continue;

      switch (tr->in_width)
        {
        case 1:
          *(uint8_t*)tr->in = word;
          break;
        case 2:
          *(uint16_t*)tr->in = word;
          break;
        case 4:
          *(uint32_t*)tr->in = word;
          break;
        }

      tr->in = (void*)((uint8_t*)tr->in + tr->in_width);
    }

  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

  if (tr->count > 0)
    return efm32_usart_spi_transfer_tx(dev);

  pv->tr = NULL;

  return 1;
}

static bool_t efm32_usart_spi_transfer_tx(struct device_s *dev)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;


  /* enable expected irq */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR,
                   tr->count >= EFM32_USART_FIFO_SIZE
                   ? endian_le32(EFM32_USART_IEN_RXFULL)
                   : endian_le32(EFM32_USART_IEN_RXDATAV));

  while (tr->count > 0 && pv->fifo_lvl < EFM32_USART_FIFO_SIZE)
    {
      uint32_t word = 0;
      switch (tr->out_width)
        {
        case 1:
          word = *(const uint8_t*)tr->out;
          break;
        case 2:
          word = *(const uint16_t*)tr->out;
          break;
        case 0:
        case 4:
          word = *(const uint32_t*)tr->out;
          break;
        }

      cpu_mem_write_32(pv->addr + EFM32_USART_TXDATA_ADDR, endian_le32(word));

      tr->out = (const void*)((const uint8_t*)tr->out + tr->out_width);
      tr->count--;
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

  while (cpu_mem_read_32(pv->addr + EFM32_USART_IF_ADDR) &
         endian_le32(EFM32_USART_IF_RXDATAV | EFM32_USART_IF_RXFULL))
    {
      cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

#ifdef CONFIG_DRIVER_EFM32_DMA
      if (pv->dma_use)
        break;
#endif

      struct dev_spi_ctrl_transfer_s *tr = pv->tr;

      if (tr != NULL && efm32_usart_spi_transfer_rx(dev))
        {
          lock_release(&dev->lock);
          kroutine_exec(&tr->kr);
          lock_spin(&dev->lock);
        }


    }

  lock_release(&dev->lock);
}

#endif

static DEV_SPI_CTRL_SELECT(efm32_usart_spi_select)
{
  struct device_s *dev = accessor->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  if (cs_id > 0)
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;
  else
    {
      /* set polarity */
      EFM32_USART_CTRL_CSINV_SETVAL(pv->ctrl, !pt);

      switch (pc)
        {
        case DEV_SPI_CS_TRANSFER:
          pv->ctrl |= EFM32_USART_CTRL_AUTOCS;
          pv->route |= EFM32_USART_ROUTE_CSPEN;
          break;

        case DEV_SPI_CS_DEASSERT:
          pv->ctrl &= ~EFM32_USART_CTRL_AUTOCS;
          pv->route |= EFM32_USART_ROUTE_CSPEN;
          break;

        case DEV_SPI_CS_RELEASE:
          pv->ctrl &= ~EFM32_USART_CTRL_AUTOCS;
          pv->route &= ~EFM32_USART_ROUTE_CSPEN;
          break;

        case DEV_SPI_CS_ASSERT:
          err = -ENOTSUP;
        }

      cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, endian_le32(pv->route));
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#ifdef CONFIG_DRIVER_EFM32_DMA

static KROUTINE_EXEC(dma_callback)
{
  struct dev_dma_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, base.kr);
  struct device_s *dev = (struct device_s *)rq->base.pvdata;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  
  lock_spin(&dev->lock);

  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  pv->tr = NULL;

  lock_release(&dev->lock);

  kroutine_exec(&tr->kr);
}

static void efm32_usart_spi_start_dma(struct device_s *dev)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  
  pv->dma_use = 1;

  pv->drq.rq.error = 0;
  /* TX */
  pv->drq.rq.tr[DEV_DMA_INTL_WRITE].size = pv->tr->count;
  pv->drq.rq.tr[DEV_DMA_INTL_WRITE].src = (uintptr_t)pv->tr->out;
  pv->drq.rq.param[DEV_DMA_INTL_WRITE].src_inc = (0x20103 >> (pv->tr->out_width * 4)) & 0xf;
  /* RX */
  pv->drq.rq.tr[DEV_DMA_INTL_READ].size = pv->tr->count;
  pv->drq.rq.tr[DEV_DMA_INTL_READ].dst = (uintptr_t)pv->tr->in;
  pv->drq.rq.param[DEV_DMA_INTL_READ].dst_inc = (0x20103 >> (pv->tr->in_width * 4)) & 0xf;
  
  pv->drq.rq.base.pvdata = dev;

  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

  /* Start DMA request */ 
  DEVICE_OP(&pv->dma, request, &pv->drq.rq);

}
#endif

static DEV_SPI_CTRL_TRANSFER(efm32_usart_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;

  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    tr->err = -EBUSY;
  else
    {
      assert(tr->count > 0);
      tr->err = 0;
      pv->tr = tr;

#ifdef CONFIG_DRIVER_EFM32_DMA
      pv->dma_use = 0;

      done = 0;

      if (tr->count > CONFIG_DRIVER_EFM32_USART_DMA_THRD)
        {
          efm32_usart_spi_start_dma(dev);
          goto end;
        }
#endif
      cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
      cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

      pv->fifo_lvl = 0;

      done = efm32_usart_spi_transfer_tx(dev);
    }

 end:

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr);
}

#ifdef CONFIG_DEVICE_SPI_REQUEST

static DEV_SPI_CTRL_QUEUE(efm32_usart_spi_queue)
{
  struct device_s *dev = accessor->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  return &pv->queue;
}

#endif

static DEV_INIT(efm32_usart_spi_init);
static DEV_CLEANUP(efm32_usart_spi_cleanup);

#define efm32_usart_spi_use dev_use_generic

DRIVER_DECLARE(efm32_usart_spi_drv, 0, "EFM32 USART (SPI)", efm32_usart_spi,
               DRIVER_SPI_CTRL_METHODS(efm32_usart_spi));

DRIVER_REGISTER(efm32_usart_spi_drv);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(efm32_usart_spi_clk_changed)
{
  struct efm32_usart_spi_context_s *pv = ep->dev->drv_pv;
  pv->freq = *freq;
  efm32_usart_spi_update_rate(pv);
}
#endif

static DEV_INIT(efm32_usart_spi_init)
{
  struct efm32_usart_spi_context_s	*pv;
  device_mem_map( dev , 1 << 0 );

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, &efm32_usart_spi_clk_changed);

  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clk_ep, &ckinfo, 0, 0))
    goto err_mem;

  if (!DEV_FREQ_IS_VALID(ckinfo.freq))
    goto err_mem;
  pv->freq = ckinfo.freq;

  if (dev_clock_sink_hold(&pv->clk_ep, 0))
    goto err_clku;
#else
  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;
#endif

  /* init state */
  pv->tr = NULL;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_queue_init(dev, &pv->queue))
    goto err_clk;
#endif

  /* disable and clear irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS |
                               EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

  /* synchronous mode, 8 bits */
  pv->ctrl = EFM32_USART_CTRL_SYNC(SYNC);
  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
  cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR, endian_le32(8 - 3));

  /* setup pinmux */
  iomux_demux_t loc[4];
  if (device_iomux_setup(dev, ">clk <miso? >mosi? >cs?", loc, NULL, NULL))
    goto err_clk;

  pv->route =  EFM32_USART_ROUTE_CLKPEN;
  if (loc[1] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_RXPEN;
  if (loc[2] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_TXPEN;
  if (loc[3] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_CSPEN;

  EFM32_USART_ROUTE_LOCATION_SETVAL(pv->route, loc[0]);

  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, endian_le32(pv->route));

  /* setup bit rate */
  pv->bit_rate = 100000;
  efm32_usart_spi_update_rate(pv);

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

#ifdef CONFIG_DRIVER_EFM32_DMA

  /* RX */
  struct dev_resource_s * rx = device_res_get(dev, DEV_RES_DMA, 0); 

  if (rx->u.dma.channel > CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT)
    goto err_dma;

  /* TX */
  struct dev_resource_s * tx = device_res_get(dev, DEV_RES_DMA, 1); 

  if (tx->u.dma.channel > CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT)
    goto err_dma;

  if (strcmp(tx->u.dma.label, rx->u.dma.label))
    goto err_dma;

  if (device_get_accessor_by_path(&pv->dma, NULL, rx->u.dma.label, DRIVER_CLASS_DMA))
    goto err_dma;

  /* Set all fields that will not change */

  pv->drq.rq.arch_rq = 1;
  pv->drq.rq.type = DEV_DMA_INTERLEAVED;

  /* WRITE */

  pv->drq.rq.tr[DEV_DMA_INTL_WRITE].dst = pv->addr + EFM32_USART_TXDATA_ADDR;

  pv->drq.rq.param[DEV_DMA_INTL_WRITE].dst_inc = DEV_DMA_INC_NONE;
  pv->drq.rq.param[DEV_DMA_INTL_WRITE].const_data = 0;
  pv->drq.rq.param[DEV_DMA_INTL_WRITE].channel = tx->u.dma.channel;

  pv->drq.cfg[DEV_DMA_INTL_WRITE].trigsrc = tx->u.dma.config;
  pv->drq.cfg[DEV_DMA_INTL_WRITE].arbiter = DMA_CHANNEL_CFG_R_POWER_AFTER1;

  /* READ */

  pv->drq.rq.tr[DEV_DMA_INTL_READ].src = pv->addr + EFM32_USART_RXDATA_ADDR;

  pv->drq.rq.param[DEV_DMA_INTL_READ].src_inc = DEV_DMA_INC_NONE;
  pv->drq.rq.param[DEV_DMA_INTL_READ].const_data = 0;
  pv->drq.rq.param[DEV_DMA_INTL_READ].channel = rx->u.dma.channel;

  pv->drq.cfg[DEV_DMA_INTL_READ].trigsrc = rx->u.dma.config;
  pv->drq.cfg[DEV_DMA_INTL_READ].arbiter = DMA_CHANNEL_CFG_R_POWER_AFTER1;
  
  kroutine_init_immediate(&pv->drq.rq.base.kr, &dma_callback);

#endif

  return 0;

 err_dma:
   printk("EFM32 usart error on DMA ressource\n");
 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
#endif
 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(efm32_usart_spi_cleanup)
{
  struct efm32_usart_spi_context_s	*pv = dev->drv_pv;

  if (pv->tr != NULL)
    return -EBUSY;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  /* disable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
#endif

  /* disable the usart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_queue_cleanup(&pv->queue);
#endif

  mem_free(pv);

  return 0;
}
