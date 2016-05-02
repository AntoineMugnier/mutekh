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
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2015
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

#include <arch/stm32/spi.h>


struct stm32_spi_private_s
{
  uintptr_t                      addr;

  bool_t                         use_cs:1;

  struct dev_irq_src_s           irq_ep;
  struct dev_spi_ctrl_transfer_s *tr;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_queue_s    queue;
#endif

  struct dev_freq_s              busfreq;
};

static
error_t stm32_spi_update_bitrate(struct stm32_spi_private_s *pv,
                                 uint32_t bit_rate)
{
  if (bit_rate == 0)
    return -EINVAL;

  uint32_t const div = pv->busfreq.num / (bit_rate * pv->busfreq.denom);
  if (div == 0)
    return -EINVAL;

  uint32_t logval = 31 - __builtin_clz(div);
  if (logval < 1)
    return -ERANGE;

  /* get the closest power of two bit rate and compute the log2. */
  logval += (div >> (logval - 1)) & 0x1;

  if (logval > 8)
    return -ERANGE;

  uintptr_t a = pv->addr + STM32_SPI_CR1_ADDR;
  uint32_t  x = endian_le32(cpu_mem_read_32(a));
  STM32_SPI_CR1_BR_SETVAL(x, logval - 1);
  cpu_mem_write_32(a, endian_le32(x));

  return 0;
}

static
DEV_SPI_CTRL_CONFIG(stm32_spi_config)
{
  struct device_s                  *dev = accessor->dev;
  struct stm32_spi_private_s *pv = dev->drv_pv;

  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    {
      err = -EBUSY;
      goto cfg_end;
    }

  /* check mosi/miso polarity */
  if (cfg->miso_pol != DEV_SPI_ACTIVE_HIGH ||
      cfg->mosi_pol != DEV_SPI_ACTIVE_HIGH)
    {
      err = -ENOTSUP;
      goto cfg_end;
    }

  /* bitrate */
  err = stm32_spi_update_bitrate(pv, cfg->bit_rate);
  if (err)
    goto cfg_end;

  uintptr_t a = pv->addr + STM32_SPI_CR1_ADDR;
  uint32_t  x = endian_le32(cpu_mem_read_32(a));

  /* data width. */
  switch (cfg->word_width)
    {
    case 8:
      STM32_SPI_CR1_DFF_SET(a, 8_BITS);
      break;
    case 16:
      STM32_SPI_CR1_DFF_SET(a, 16_BITS);
      break;
    default:
      err = -ENOTSUP;
      goto cfg_end;
    }

  /* clock */
  switch (cfg->ck_mode)
    {
    default:
      UNREACHABLE();

    case DEV_SPI_CK_MODE_0:
      STM32_SPI_CR1_CPHA_SET(x, FIRST_TRANS);
      STM32_SPI_CR1_CPOL_SET(x, 0);
      break;

    case DEV_SPI_CK_MODE_1:
      STM32_SPI_CR1_CPHA_SET(x, SECOND_TRANS);
      STM32_SPI_CR1_CPOL_SET(x, 0);
      break;

    case DEV_SPI_CK_MODE_2:
      STM32_SPI_CR1_CPHA_SET(x, FIRST_TRANS);
      STM32_SPI_CR1_CPOL_SET(x, 1);
      break;

    case DEV_SPI_CK_MODE_3:
      STM32_SPI_CR1_CPHA_SET(x, SECOND_TRANS);
      STM32_SPI_CR1_CPOL_SET(x, 1);
      break;
    }

  /* msb/lsb first */
  if (cfg->bit_order == DEV_SPI_LSB_FIRST)
    STM32_SPI_CR1_LSBFIRST_SET(x, 1);

  cpu_mem_write_32(a, endian_le32(x));

cfg_end:
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
void stm32_spi_transfer_tx(struct device_s *dev);

static
bool_t stm32_spi_transfer_rx(struct device_s *dev)
{
  struct stm32_spi_private_s     *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  uint32_t w = endian_le32(cpu_mem_read_32(pv->addr + STM32_SPI_DR_ADDR));

  if (tr->data.in != NULL)
    {
      switch (tr->data.in_width)
        {
        case 1:
          *(uint8_t *)tr->data.in = w;
          break;
        case 2:
          *(uint16_t *)tr->data.in = w;
          break;
        case 4:
          *(uint32_t *)tr->data.in = w;
          break;
        }

      tr->data.in = (void *)((uint8_t *)tr->data.in + tr->data.in_width);
    }

  bool_t end = !tr->data.count;

  if (!end)
    stm32_spi_transfer_tx(dev);

  return end;
}

static
void stm32_spi_transfer_tx(struct device_s *dev)
{
  struct stm32_spi_private_s     *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  uint32_t w = 0;

  switch (tr->data.out_width)
    {
    case 1:
      w = *(const uint8_t *)tr->data.out;
      break;
    case 2:
      w = *(const uint16_t *)tr->data.out;
      break;
    case 0:
    case 4:
      w = *(const uint32_t*)tr->data.out;
      break;
    }

  cpu_mem_write_32(pv->addr + STM32_SPI_DR_ADDR, endian_le32(w));

  tr->data.out = (const void *)((const uint8_t *)tr->data.out + tr->data.out_width);

  --tr->data.count;
}

static
DEV_IRQ_SRC_PROCESS(stm32_spi_irq)
{
  struct device_s            *dev = ep->base.dev;
  struct stm32_spi_private_s *pv  = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uintptr_t a = pv->addr + STM32_SPI_SR_ADDR;
      uint32_t  x = endian_le32(cpu_mem_read_32(a));

      // printk("spi irq%08x\n", x);
      if (!(x & STM32_SPI_SR_RXNE))
        break;

      struct dev_spi_ctrl_transfer_s *tr = pv->tr;
      if (tr == NULL)
        {
          cpu_mem_read_32(pv->addr + STM32_SPI_DR_ADDR);
        }
      else if (stm32_spi_transfer_rx(dev))
        {
          pv->tr = NULL;
          kroutine_exec(&tr->kr);
        }
    }

  lock_release(&dev->lock);
}

static
DEV_SPI_CTRL_SELECT(stm32_spi_select)
{
  struct device_s            *dev = accessor->dev;
  struct stm32_spi_private_s *pv  = dev->drv_pv;

  error_t err = 0;

  if (cs_id > 0 || !pv->use_cs)
    return -ENOTSUP;

  uintptr_t a = pv->addr + STM32_SPI_CR2_ADDR;
  uint32_t  x = endian_le32(cpu_mem_read_32(a));

  bool_t enable = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;
  else
    {
      switch (pc)
        {
        case DEV_SPI_CS_ASSERT:
          STM32_SPI_CR2_SSOE_SET(x, 1);
          enable = 1;
          break;

        case DEV_SPI_CS_RELEASE:
          STM32_SPI_CR2_SSOE_SET(x, 0);
          break;

        case DEV_SPI_CS_TRANSFER:
        case DEV_SPI_CS_DEASSERT:
          err = -ENOTSUP;
          break;
        }

      if (!err)
        cpu_mem_write_32(a, endian_le32(x));

      /* we need to re-enable the SPI device as SSOE = 0 disabled it. */
      if (enable)
        {
          a = pv->addr + STM32_SPI_CR1_ADDR;
          x = endian_le32(cpu_mem_read_32(a));
          STM32_SPI_CR1_SPE_SET(x, 1);
          STM32_SPI_CR1_MSTR_SET(x, MASTER);
          cpu_mem_write_32(a, endian_le32(x));
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_SPI_CTRL_TRANSFER(stm32_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct stm32_spi_private_s *pv = dev->drv_pv;

  assert(tr->data.count > 0);
  tr->err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr == NULL)
    {
      tr->err = 0;
      pv->tr = tr;
      stm32_spi_transfer_tx(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (tr->err)
    kroutine_exec(&tr->kr);
}

#ifdef CONFIG_DEVICE_SPI_REQUEST

static DEV_SPI_CTRL_QUEUE(stm32_spi_queue)
{
  struct device_s *dev = accessor->dev;
  struct stm32_spi_private_s *pv = dev->drv_pv;
  return &pv->queue;
}

#endif


#define stm32_spi_use dev_use_generic

static DEV_INIT(stm32_spi_init)
{
  struct stm32_spi_private_s *pv;


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  memset(pv, 0, sizeof(*pv));

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  if (device_get_res_freq(dev, &pv->busfreq, 0))
    goto err_mem;

  /* init state */
  pv->tr = NULL;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_queue_init(dev, &pv->queue))
    goto err_mem;
#endif

  /* disable device. */
  cpu_mem_write_32(pv->addr + STM32_SPI_CR1_ADDR, 0);

  /* setup pinmux */
  iomux_demux_t loc[4];
  if (device_iomux_setup(dev, ">clk <miso >mosi >cs?", loc, NULL, NULL))
    goto err_mem;

  /* TODO: add suppot for rxonly/txonly */
  uint32_t cr1 = 0;
  if (loc[3] != IOMUX_INVALID_MUX)
    pv->use_cs = 1;
  else
    {
      STM32_SPI_CR1_SSM_SET(cr1, 1);
      STM32_SPI_CR1_SSI_SET(cr1, 1);
    }

  /* init irq endpoint */
  device_irq_source_init(dev, &pv->irq_ep, 1, &stm32_spi_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_mem;

  /* enable the spi with rx and tx enabled in master mode. */
  STM32_SPI_CR1_SPE_SET(cr1, 1);
  STM32_SPI_CR1_MSTR_SET(cr1, MASTER);

  STM32_SPI_CR1_BR_SET(cr1, PCLK_DIV_256);

  cpu_mem_write_32(pv->addr + STM32_SPI_CR1_ADDR, endian_le32(cr1));

  /* flush rx */
  while (cpu_mem_read_32(pv->addr + STM32_SPI_SR_ADDR)
         & endian_le32(STM32_SPI_SR_RXNE | STM32_SPI_SR_OVR))
    cpu_mem_read_32(pv->addr + STM32_SPI_DR_ADDR);

  /* enable rx irqs */
  cpu_mem_write_32(pv->addr + STM32_SPI_CR2_ADDR, STM32_SPI_CR2_RXNEIE);

  dev->drv_pv = pv;

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32_spi_cleanup)
{
  struct stm32_spi_private_s	*pv = dev->drv_pv;

  if (pv->tr != NULL)
    return -EBUSY;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* disable irqs */
  cpu_mem_write_32(pv->addr + STM32_SPI_CR2_ADDR, 0);
  /* disable the usart */
  cpu_mem_write_32(pv->addr + STM32_SPI_CR1_ADDR, 0);

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_queue_cleanup(&pv->queue);
#endif

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(stm32_spi_drv, 0, "STM32 SPI", stm32_spi,
               DRIVER_SPI_CTRL_METHODS(stm32_spi));

DRIVER_REGISTER(stm32_spi_drv);

