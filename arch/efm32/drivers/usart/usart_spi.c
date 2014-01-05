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

#include <arch/efm32_usart.h>

#define EFM32_USART_FIFO_SIZE 2

struct efm32_usart_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s            irq_ep[2];
  bool_t                         use_irq;
#endif
  struct dev_spi_ctrl_transfer_s *tr;
  uint32_t                       ctrl;
  uint_fast8_t                   fifo_lvl;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_queue_s    queue;
#endif
};

static DEVSPI_CTRL_CONFIG(efm32_usart_spi_config)
{
  struct device_s *dev = scdev->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;

  if (pv->tr != NULL)
    return -EBUSY;

  if (cfg->word_width < 4 || cfg->word_width > 16)
    return -ENOTSUP;
  cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR, endian_le32(cfg->word_width - 3));

  EFM32_USART_CTRL_CLKPOL_SETVAL(pv->ctrl, (cfg->ck_mode >> 1) & 1);
  EFM32_USART_CTRL_CLKPHA_SETVAL(pv->ctrl, (cfg->ck_mode >> 0) & 1);
  EFM32_USART_CTRL_MSBF_SETVAL(pv->ctrl, cfg->bit_order);
  EFM32_USART_CTRL_CSINV_SETVAL(pv->ctrl, !cfg->cs_pol);
  EFM32_USART_CTRL_RXINV_SETVAL(pv->ctrl, cfg->miso_pol);
  EFM32_USART_CTRL_TXINV_SETVAL(pv->ctrl, cfg->mosi_pol);
  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));

#warning freq
  uint32_t div = 128 * (14000000 / cfg->bit_rate - 2);
  cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, endian_le32(div));

  return 0;
}

static void efm32_usart_spi_transfer_tx(struct device_s *dev, bool_t nested);

static void efm32_usart_spi_transfer_rx(struct device_s *dev, bool_t nested)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (pv->fifo_lvl > 0)
    {
      uint32_t st = cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
                      & endian_le32(EFM32_USART_STATUS_RXDATAV);

#ifdef CONFIG_DEVICE_IRQ
      if (pv->use_irq && !st)
        break;
#endif
      if (!st)
        continue;

      uint32_t word = endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_RXDATA_ADDR));
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

  if (tr->count > 0)
    {
#ifdef CONFIG_DEVICE_IRQ
      if (!pv->use_irq || pv->fifo_lvl == 0)
#endif
        return efm32_usart_spi_transfer_tx(dev, nested);
    }
  else
    {
#ifdef CONFIG_DEVICE_IRQ
      if (pv->use_irq)
        cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
#endif
      pv->tr = NULL;
      tr->err = 0;
      tr->callback(tr, nested);
    }
}

static void efm32_usart_spi_transfer_tx(struct device_s *dev, bool_t nested)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

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
  if (pv->use_irq)
    {
      uint32_t ien = (pv->fifo_lvl == EFM32_USART_FIFO_SIZE)
        ? EFM32_USART_IEN_RXFULL : EFM32_USART_IEN_RXDATAV;
      cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, endian_le32(ien));
      return;
    }
#endif

  return efm32_usart_spi_transfer_rx(dev, nested);
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_EP_PROCESS(efm32_usart_spi_irq)
{
  struct device_s *dev = ep->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));
  efm32_usart_spi_transfer_rx(dev, 0);
  lock_release(&dev->lock);
}

#endif

static DEVSPI_CTRL_SELECT(efm32_usart_spi_select)
{
  struct device_s *dev = scdev->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;

  switch (p)
    {
    case DEV_SPI_CS_SELECT_ON_NEXT_TRANSFER:
      pv->ctrl |= EFM32_USART_CTRL_AUTOCS;
      if (cs > 0)
        return -ENOTSUP;
      break;

    case DEV_SPI_CS_SELECT_NONE:
    case DEV_SPI_CS_DESELECT_NOW:
      pv->ctrl &= ~EFM32_USART_CTRL_AUTOCS;
      break;

    case DEV_SPI_CS_SELECT_NOW:
      return -ENOTSUP;
    }

  return 0;
}

static DEVSPI_CTRL_TRANSFER(efm32_usart_spi_transfer)
{
  struct device_s *dev = scdev->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    {
      err = -EBUSY;
      goto err;
    }

  assert(tr->count > 0);

  pv->tr = tr;
  tr->scdev = scdev;

  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR, endian_le32(EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));
  pv->fifo_lvl = 0;

#ifdef CONFIG_DEVICE_IRQ
  /* enable irqs */
#warning use_irq condition
  pv->use_irq = 1; /* bit_rate < CONFIG_DRIVER_EFM32_USART_SPI_IRQRATE; */

  if (pv->use_irq)
    {
      cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));
      /* initiate first transfer by filling TX fifo */
      efm32_usart_spi_transfer_tx(dev, 1);
    }
  else if (!polling)
    {
      pv->tr = NULL;
      err = -EAGAIN;
    }
  else
#endif
    {
      LOCK_RELEASE_IRQ_X(&dev->lock);

      /* perform the whole transfer without relying on irqs */
      efm32_usart_spi_transfer_tx(dev, 1);
      assert(pv->tr == NULL);
      return 1;
    }

 err:;
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

#ifdef CONFIG_DEVICE_SPI_REQUEST

static DEVSPI_CTRL_QUEUE(efm32_usart_spi_queue)
{
  struct device_s *dev = scdev->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  return &pv->queue;
}

#endif

static const struct driver_spi_ctrl_s	efm32_usart_spi_ctrl_drv =
{
  .class_		= DRIVER_CLASS_SPI_CTRL,
  .f_config		= efm32_usart_spi_config,
  .f_select		= efm32_usart_spi_select,
  .f_transfer		= efm32_usart_spi_transfer,
#ifdef CONFIG_DEVICE_SPI_REQUEST
  .f_queue		= efm32_usart_spi_queue,
#endif
};

static DEV_INIT(efm32_usart_spi_init);
static DEV_CLEANUP(efm32_usart_spi_cleanup);

const struct driver_s	efm32_usart_spi_drv =
{
  .desc                 = "EFM32 USART (SPI)",
  .f_init		= efm32_usart_spi_init,
  .f_cleanup		= efm32_usart_spi_cleanup,
  .classes              = { &efm32_usart_spi_ctrl_drv, 0 }
};

REGISTER_DRIVER(efm32_usart_spi_drv);

static DEV_INIT(efm32_usart_spi_init)
{
  struct efm32_usart_spi_context_s	*pv;
  device_mem_map( dev , 1 << 0 );

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  pv->tr = NULL;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_queue_init(&pv->queue);
#endif

  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS |
                               EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));

  /* disable and clear irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

  /* synchronous mode, 8 bits */
  pv->ctrl = EFM32_USART_CTRL_SYNC(SYNC);
  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
  cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR, endian_le32(8 - 3));

  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR,
                   endian_le32(EFM32_USART_ROUTE_RXPEN | EFM32_USART_ROUTE_TXPEN |
                               EFM32_USART_ROUTE_CLKPEN | EFM32_USART_ROUTE_CSPEN |
                               EFM32_USART_ROUTE_LOCATION(LOC1)));
#warning FIXME hardwired location

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, pv->irq_ep, 2, &efm32_usart_spi_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 2, 1))
    goto err_fifo;
#endif

  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXEN | EFM32_USART_CMD_TXEN |
                               EFM32_USART_CMD_MASTEREN));

  dev->drv = &efm32_usart_spi_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(efm32_usart_spi_cleanup)
{
  struct efm32_usart_spi_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, pv->irq_ep, 2);
  /* disable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
#endif

  /* disable the usart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_queue_cleanup(&pv->queue);
#endif

  mem_free(pv);
}
