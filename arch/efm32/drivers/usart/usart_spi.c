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

#include <arch/efm32_usart.h>

#define EFM32_USART_FIFO_SIZE 2

struct efm32_usart_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s            irq_ep[2];
#endif
  struct dev_spi_ctrl_transfer_s *tr;
  uint32_t                       ctrl;
  uint32_t                       route;
  uint_fast8_t                   fifo_lvl;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_queue_s    queue;
#endif
};

static DEVSPI_CTRL_CONFIG(efm32_usart_spi_config)
{
  struct device_s *dev = scdev->dev;
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

          EFM32_USART_CTRL_CLKPOL_SETVAL(pv->ctrl, (cfg->ck_mode >> 1) & 1);
          EFM32_USART_CTRL_CLKPHA_SETVAL(pv->ctrl, (cfg->ck_mode >> 0) & 1);
          EFM32_USART_CTRL_MSBF_SETVAL(pv->ctrl, cfg->bit_order);
          EFM32_USART_CTRL_RXINV_SETVAL(pv->ctrl, cfg->miso_pol);
          EFM32_USART_CTRL_TXINV_SETVAL(pv->ctrl, cfg->mosi_pol);
          cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));

#warning freq
          uint32_t div = 128 * (14000000 / cfg->bit_rate - 2);
          cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, endian_le32(div));
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

  if (tr->count > 0)
    return efm32_usart_spi_transfer_tx(dev);

  /* end of RX */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
  pv->tr = NULL;

  return 1;
}

static bool_t efm32_usart_spi_transfer_tx(struct device_s *dev)
{
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

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

      cpu_mem_write_32(pv->addr + EFM32_USART_TXDATA_ADDR, endian_le32((uint8_t)word));

      tr->out = (const void*)((const uint8_t*)tr->out + tr->out_width);
      tr->count--;
      pv->fifo_lvl++;
    }

#ifdef CONFIG_DEVICE_IRQ
  if (pv->fifo_lvl == EFM32_USART_FIFO_SIZE)
    cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, endian_le32(EFM32_USART_IEN_RXFULL));
  else
    cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, endian_le32(EFM32_USART_IEN_RXDATAV));

  return 0;
#else
  return efm32_usart_spi_transfer_rx(dev);
#endif
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_EP_PROCESS(efm32_usart_spi_irq)
{
  struct device_s *dev = ep->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (cpu_mem_read_32(pv->addr + EFM32_USART_IF_ADDR) & endian_le32(EFM32_USART_IF_RXDATAV))
    {
      cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));
      struct dev_spi_ctrl_transfer_s *tr = pv->tr;

      if (tr != NULL && efm32_usart_spi_transfer_rx(dev))
        {
          lock_release(&dev->lock);
          kroutine_exec(&tr->kr, 0);
          lock_spin(&dev->lock);
        }
    }

  lock_release(&dev->lock);
}

#endif

static DEVSPI_CTRL_SELECT(efm32_usart_spi_select)
{
  struct device_s *dev = scdev->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  if (cs_id > 0)
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    {
      err = -EBUSY;
    }
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

static DEVSPI_CTRL_TRANSFER(efm32_usart_spi_transfer)
{
  struct device_s *dev = scdev->dev;
  struct efm32_usart_spi_context_s *pv = dev->drv_pv;
  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    {
      tr->err = -EBUSY;
    }
  else
    {
      assert(tr->count > 0);

      pv->tr = tr;
      tr->scdev = scdev;

      cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
      cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));
      pv->fifo_lvl = 0;
      tr->err = 0;

      done = efm32_usart_spi_transfer_tx(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr, cpu_is_interruptible());     /* tail call */
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
  if (dev_spi_queue_init(dev, &pv->queue))
    goto err_mem;
#endif

  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS |
                               EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));

  /* disable and clear irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR, endian_le32(EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

  /* synchronous mode, 8 bits */
  pv->ctrl = EFM32_USART_CTRL_SYNC(SYNC);
  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(pv->ctrl));
  cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR, endian_le32(8 - 3));

  /* setup pinmux */
  iomux_demux_t loc[4];
  if (device_iomux_setup(dev, ">clk <miso? >mosi? >cs?", loc, NULL, NULL))
    goto err_mem;

  EFM32_USART_ROUTE_LOCATION_SETVAL(pv->route, loc[0]);

  pv->route =  EFM32_USART_ROUTE_CLKPEN;
  if (loc[1] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_RXPEN;
  if (loc[2] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_TXPEN;
  if (loc[3] != IOMUX_INVALID_DEMUX)
    pv->route |= EFM32_USART_ROUTE_CSPEN;

  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, endian_le32(pv->route));

  /* init irq endpoint */
#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, pv->irq_ep, 2,
                         &efm32_usart_spi_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_ep, 2, -1))
    goto err_mem;
#endif

  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXEN | EFM32_USART_CMD_TXEN |
                               EFM32_USART_CMD_MASTEREN));

  dev->drv = &efm32_usart_spi_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

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
