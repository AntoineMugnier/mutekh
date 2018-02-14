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

#include <arch/pic32/irq.h>
#include <arch/pic32/spi.h>

#ifdef CONFIG_DRIVER_PIC32_DMA
#include <device/class/dma.h>
#include <arch/pic32/dma_request.h>
#endif

#define PIC32_SPI_FIFO_SIZE 16

DRIVER_PV(struct pic32_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s           irq_ep;
#endif
  struct dev_spi_ctrl_transfer_s *tr;
  uint32_t                       ctrl;
  uint32_t                       route;
  uint_fast8_t                   fifo_lvl;

  struct dev_spi_ctrl_context_s    spi_ctrl_ctx;

  struct dev_freq_s              freq;
  uint16_t                       bit_rate1k;
  enum dev_spi_polarity_e        pol;
  enum dev_spi_bit_order_e       bit_order;

#ifdef CONFIG_DRIVER_PIC32_DMA
  bool_t                         dma_use;
  struct device_dma_s            dma;
  struct pic32_dev_dma_rq_s      intlrq; 
  struct pic32_dev_dma_rq_s      brq; 
#endif
});

static void pic32_spi_update_rate(struct pic32_spi_context_s *pv)
{
  uint32_t d = pv->bit_rate1k * pv->freq.denom * 1024;
  uint32_t brg = pv->freq.num >> 1;

  assert(d);

  brg = PIC32_SPI_BRG_VAL(brg/d - 1);

  cpu_mem_write_32(pv->addr + PIC32_SPI_BRG_ADDR, endian_le32(PIC32_SPI_BRG_VAL(brg)));
}

static DEV_SPI_CTRL_CONFIG(pic32_spi_config)
{
  struct device_s *dev = accessor->dev;
  struct pic32_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;

  if (cfg->word_width != 8)
    err = -ENOTSUP;

  if (cfg->miso_pol != cfg->mosi_pol)
    err = -ENOTSUP;

  if (!err)
    {
      uint32_t x = PIC32_SPI_CON_MSTEN | PIC32_SPI_CON_ON | PIC32_SPI_CON_ENHBUF |
                   PIC32_SPI_CON_SRXISEL(NOT_EMPTY) | PIC32_SPI_CON_STXISEL(EMPTY);
  
      if (cfg->ck_mode == DEV_SPI_CK_MODE_2 || cfg->ck_mode == DEV_SPI_CK_MODE_3)
        x |= PIC32_SPI_CON_CKP;
  
      if (cfg->ck_mode == DEV_SPI_CK_MODE_0 || cfg->ck_mode == DEV_SPI_CK_MODE_2)
        x |= PIC32_SPI_CON_CKE | PIC32_SPI_CON_SMP;
  
      cpu_mem_write_32(pv->addr + PIC32_SPI_CON_ADDR, endian_le32(x));
  
      pv->bit_order = cfg->bit_order;
      pv->pol = cfg->mosi_pol;
  
      if (cfg->bit_rate1k && (pv->bit_rate1k != cfg->bit_rate1k))
        {
          pv->bit_rate1k = cfg->bit_rate1k;
          pic32_spi_update_rate(pv);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static uint32_t pic32_spi_format(struct device_s *dev, uint32_t word)
{
  struct pic32_spi_context_s *pv = dev->drv_pv;

  uint32_t w = word;

  if (pv->pol == DEV_SPI_ACTIVE_LOW)
    w = ~word;

  if (pv->bit_order == DEV_SPI_LSB_FIRST)
    {
      w = ((w & 0x0F) << 4) | ((0xF0 & w) >> 4);
      w = ((w & 0xCC) >> 2) | ((0x33 & w) << 2);
      w = ((w & 0xAA) >> 1) | ((0x55 & w) << 1);
    }

  return w;
}

static bool_t pic32_spi_transfer_tx(struct device_s *dev);

static bool_t pic32_spi_transfer_rx(struct device_s *dev)
{
  struct pic32_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (pv->fifo_lvl > 0)
    {
      uint32_t st = cpu_mem_read_32(pv->addr + PIC32_SPI_STATUS_ADDR)
                      & endian_le32(PIC32_SPI_STATUS_RBE);

      if (st)
#ifdef CONFIG_DEVICE_IRQ
        return 0;           /* wait for more rx irq */
#else
        continue;
#endif

      uint32_t word = (uint8_t)endian_le32(cpu_mem_read_32(pv->addr + PIC32_SPI_BUF_ADDR));

      word = pic32_spi_format(dev, word);

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

  if (tr->data.count > 0)
    return pic32_spi_transfer_tx(dev);

  pv->tr = NULL;
  device_irq_src_disable(&pv->irq_ep);

  return 1;
}

static bool_t pic32_spi_transfer_tx(struct device_s *dev)
{
  struct pic32_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (tr->data.count > 0 && pv->fifo_lvl < PIC32_SPI_FIFO_SIZE)
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

      word = pic32_spi_format(dev, word);

      cpu_mem_write_32(pv->addr + PIC32_SPI_BUF_ADDR, endian_le32(word));

      tr->data.out = (const void*)((const uint8_t*)tr->data.out + tr->data.out_width);
      tr->data.count--;
      pv->fifo_lvl++;
    }

#ifdef CONFIG_DEVICE_IRQ
  /* Enable TX interrupt */
  device_irq_src_enable(&pv->irq_ep);
  return 0;
#else
  return pic32_spi_transfer_rx(dev);
#endif
}


#ifdef CONFIG_DRIVER_PIC32_DMA

static void pic32_spi_restart(struct pic32_spi_context_s * pv)
{
  /* Reset controller to flush RX fifo */
  cpu_mem_write_32(pv->addr + PIC32_SPI_CON_CLR_ADDR, PIC32_SPI_CON_ON);
  cpu_mem_write_32(pv->addr + PIC32_SPI_CON_SET_ADDR, PIC32_SPI_CON_ON);
}


static KROUTINE_EXEC(dma_callback)
{
  struct dev_dma_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, base.kr);
  struct device_s *dev = (struct device_s *)rq->base.pvdata;
  struct pic32_spi_context_s *pv = dev->drv_pv;
  
  lock_spin(&dev->lock);

  if ((struct pic32_dev_dma_rq_s *)rq == &pv->brq)
  /* Basic request */
    {
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_SPI_STATUS_ADDR));
      if ((x & PIC32_SPI_STATUS_TBE) && (x & PIC32_SPI_STATUS_SRMT))
        pic32_spi_restart(pv);
      else
        {
          /* TX fifo is not empty */
          device_irq_src_enable(&pv->irq_ep);
          lock_release(&dev->lock);
          return;
        }
    }

  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  pv->tr = NULL;
  pv->dma_use = 0;

  lock_release(&dev->lock);

  kroutine_exec(&tr->kr);
}

static void pic32_spi_start_dma(struct device_s *dev)
{
  struct pic32_spi_context_s *pv = dev->drv_pv;
  uint8_t in_width = (0x20103 >> (pv->tr->data.in_width * 4)) & 0xf; 
  uint8_t out_width = (0x20103 >> (pv->tr->data.out_width * 4)) & 0xf; 

  pv->dma_use = 1;

  if (pv->tr->data.in)
  /* Interleaved request */
    {
      /* TX */
      if (pv->tr->data.out_width)
        {
          pv->intlrq.rq.tr[DEV_DMA_INTL_WRITE].size = pv->tr->data.count * pv->tr->data.out_width;
          pv->intlrq.rq.tr[DEV_DMA_INTL_WRITE].src = (uintptr_t)pv->tr->data.out;
          pv->intlrq.rq.param[DEV_DMA_INTL_WRITE].src_inc = out_width;
        }
      else
        {
          pv->intlrq.rq.tr[DEV_DMA_INTL_WRITE].size = pv->tr->data.count * pv->tr->data.in_width;
          pv->intlrq.rq.tr[DEV_DMA_INTL_WRITE].src = (uintptr_t)pv->tr->data.in;
          pv->intlrq.rq.param[DEV_DMA_INTL_WRITE].src_inc = in_width;
        }

      /* RX */
      pv->intlrq.rq.tr[DEV_DMA_INTL_READ].size = pv->tr->data.count * pv->tr->data.in_width;
      pv->intlrq.rq.tr[DEV_DMA_INTL_READ].dst = (uintptr_t)pv->tr->data.in;
      pv->intlrq.rq.param[DEV_DMA_INTL_READ].dst_inc = in_width;
      /* Start DMA request */ 
      DEVICE_OP(&pv->dma, request, &pv->intlrq.rq);
    }
  else
  /* Basic request */
    {
      pv->brq.rq.error = 0;
      pv->brq.rq.basic.size = pv->tr->data.count * pv->tr->data.out_width;
      pv->brq.rq.basic.src = (uintptr_t)pv->tr->data.out;
      pv->brq.rq.param[0].src_inc = out_width;
      /* Start DMA request */ 
      DEVICE_OP(&pv->dma, request, &pv->brq.rq);
    }
}
#endif

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(pic32_spi_irq)
{

  struct device_s *dev = ep->base.dev;
  struct pic32_spi_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

#ifdef CONFIG_DRIVER_PIC32_DMA
   if (pv->dma_use)
     {
        device_irq_src_disable(&pv->irq_ep);
        pic32_spi_restart(pv);
        /* Fifo is empty */
        pv->tr = NULL;
        pv->dma_use = 0;

        kroutine_exec(&tr->kr);
        goto end;
      }
#endif

  if (tr != NULL && pic32_spi_transfer_rx(dev))
    kroutine_exec(&tr->kr);

end:
  lock_release(&dev->lock);
}

#endif

static DEV_SPI_CTRL_TRANSFER(pic32_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct pic32_spi_context_s *pv = dev->drv_pv;

  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    tr->err = -EBUSY;
  else if (tr->cs_op != DEV_SPI_CS_NOP_NOP)
    tr->err = -ENOTSUP;
  else
    {
      assert(tr->data.count > 0);
      tr->err = 0;
      pv->tr = tr;

#ifdef CONFIG_DRIVER_PIC32_DMA
      pv->dma_use = 0;
      done = 0;

      if ((tr->data.count > 4) &&
          (pv->tr->data.out_width || pv->tr->data.in))
        {
          pic32_spi_start_dma(dev);
          goto end;
        }
#endif
      pv->fifo_lvl = 0;
      done = pic32_spi_transfer_tx(dev);
    }

end:

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr);
}

#define pic32_spi_cscfg (dev_spi_ctrl_cscfg_t*)dev_driver_notsup_fcn
#define pic32_spi_use dev_use_generic

static DEV_INIT(pic32_spi_init)
{
  struct pic32_spi_context_s	*pv;


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;

  /* init state */
  pv->tr = NULL;

  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_context_init(dev, &pv->spi_ctrl_ctx))
    goto err_mem;
#endif

  cpu_mem_write_32(pv->addr + PIC32_SPI_CON_ADDR, 0);
  cpu_mem_write_32(pv->addr + PIC32_SPI_CON2_ADDR, 0);

  /* setup pinmux */
  if (device_iomux_setup(dev, ">clk? <miso? >mosi? >cs?", NULL, NULL, NULL))
    goto err_mem;

  /* setup bit rate */
  pv->bit_rate1k = CONFIG_DRIVER_PIC32_SPI_BAUDRATE >> 10;
  pic32_spi_update_rate(pv);

  /* enable the spi */
  uint32_t x = PIC32_SPI_CON_MSTEN | PIC32_SPI_CON_ON | PIC32_SPI_CON_ENHBUF |
               PIC32_SPI_CON_SRXISEL(NOT_EMPTY) | PIC32_SPI_CON_STXISEL(EMPTY);

  cpu_mem_write_32(pv->addr + PIC32_SPI_CON_ADDR, endian_le32(x));

#ifdef CONFIG_DRIVER_PIC32_DMA

  /* RX */
  struct dev_resource_s * rx = device_res_get(dev, DEV_RES_DMA, 0); 

  if (rx->u.dma.channel > CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT)
    goto err_dma;

  /* TX */
  struct dev_resource_s * tx = device_res_get(dev, DEV_RES_DMA, 1); 

  if (tx->u.dma.channel > CONFIG_DRIVER_PIC32_DMA_CHANNEL_COUNT)
    goto err_dma;

  if (strcmp(tx->u.dma.label, rx->u.dma.label))
    goto err_dma;

  if (device_get_accessor_by_path(&pv->dma.base, NULL, rx->u.dma.label, DRIVER_CLASS_DMA))
    goto err_dma;

  /* Set interleaved request */

  pv->intlrq.rq.arch_rq = 1;
  pv->intlrq.rq.type = DEV_DMA_INTERLEAVED;
  pv->intlrq.rq.base.pvdata = dev;

  /* WRITE */

  pv->intlrq.rq.tr[DEV_DMA_INTL_WRITE].dst = pv->addr + PIC32_SPI_BUF_ADDR;
  pv->intlrq.rq.param[DEV_DMA_INTL_WRITE].dst_inc = DEV_DMA_INC_NONE;
  pv->intlrq.rq.param[DEV_DMA_INTL_WRITE].const_data = 0;
  pv->intlrq.rq.param[DEV_DMA_INTL_WRITE].channel = tx->u.dma.channel;
  pv->intlrq.cfg[DEV_DMA_INTL_WRITE].trigsrc = tx->u.dma.config;
  pv->intlrq.cfg[DEV_DMA_INTL_WRITE].cell_size = PIC32_SPI_FIFO_SIZE;

  /* READ */

  pv->intlrq.rq.tr[DEV_DMA_INTL_READ].src = pv->addr + PIC32_SPI_BUF_ADDR;
  pv->intlrq.rq.param[DEV_DMA_INTL_READ].src_inc = DEV_DMA_INC_NONE;
  pv->intlrq.rq.param[DEV_DMA_INTL_READ].const_data = 0;
  pv->intlrq.rq.param[DEV_DMA_INTL_READ].channel = rx->u.dma.channel;
  pv->intlrq.cfg[DEV_DMA_INTL_READ].trigsrc = rx->u.dma.config;
  pv->intlrq.cfg[DEV_DMA_INTL_READ].cell_size = 1;
  
  kroutine_init_immediate(&pv->intlrq.rq.base.kr, &dma_callback);

  /* Set basic request */

  pv->brq.rq.arch_rq = 1;
  pv->brq.rq.type = DEV_DMA_BASIC;
  pv->brq.rq.base.pvdata = dev;
  pv->brq.rq.basic.dst = pv->addr + PIC32_SPI_BUF_ADDR;
  pv->brq.rq.param[0].dst_inc = DEV_DMA_INC_NONE;
  pv->brq.rq.param[0].const_data = 0;
  pv->brq.rq.param[0].channel = tx->u.dma.channel;
  pv->brq.cfg[0].trigsrc = tx->u.dma.config;
  pv->brq.cfg[0].cell_size = PIC32_SPI_FIFO_SIZE;

  kroutine_init_immediate(&pv->brq.rq.base.kr, &dma_callback);

#endif

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_ep, 1, &pic32_spi_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, 0))
    goto err_mem;
#endif

  return 0;

 err_dma:
   printk("PIC32 SPI: Error on DMA ressource\n");
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(pic32_spi_cleanup)
{
  struct pic32_spi_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
#endif

  /* disable the spi */
  cpu_mem_write_32(pv->addr + PIC32_SPI_CON_ADDR, 0);
  cpu_mem_write_32(pv->addr + PIC32_SPI_CON2_ADDR, 0);

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  device_iomux_cleanup(dev);
  mem_free(pv);
}

DRIVER_DECLARE(pic32_spi_drv, 0, "PIC32 SPI", pic32_spi,
               DRIVER_SPI_CTRL_METHODS(pic32_spi));

DRIVER_REGISTER(pic32_spi_drv);


