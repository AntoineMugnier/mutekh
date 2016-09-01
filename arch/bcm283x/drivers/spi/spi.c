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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
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

#include <arch/bcm283x/spi.h>

#define BCM283X_SPI_FIFO_SIZE 16
#define BCM283X_CS_COUNT 3
#define BCM283X_SPI_CORE_CLK 250000000

DRIVER_PV(struct bcm283x_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s            irq_ep;
#endif
  struct dev_spi_ctrl_transfer_s *tr;
  uint32_t                       ctrl;
  uint_fast8_t                   fifo_lvl;
  enum dev_spi_bit_order_e       bit_order;

  struct dev_spi_ctrl_context_s    spi_ctrl_ctx;
});

static DEV_SPI_CTRL_CONFIG(bcm283x_spi_config)
{
//  printk("CONFIG\n");
  struct device_s *dev = accessor->dev;
  struct bcm283x_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;

  if (cfg->word_width != 8)
    err = -ENOTSUP;

  pv->bit_order = cfg->bit_order;

  BCM283X_SPI_CS_CPOL_SETVAL(pv->ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_2 ||
                                       cfg->ck_mode == DEV_SPI_CK_MODE_3);

  BCM283X_SPI_CS_CPHA_SETVAL(pv->ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_1 ||
                                       cfg->ck_mode == DEV_SPI_CK_MODE_3);

  cpu_mem_write_32(pv->addr + BCM283X_SPI_CS_ADDR, endian_le32(pv->ctrl));

  /* DIV must be rounded down to the nearest power of 2 */

  uint32_t div = (BCM283X_SPI_CORE_CLK/cfg->bit_rate);

  uint_fast8_t i = __builtin_clz(div);

  if (i < 16)
    i = 16;

  div = 1 << (sizeof(div) * 8 - 1 - i);

  cpu_mem_write_32(pv->addr + BCM283X_SPI_CLK_ADDR, endian_le32(div));

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static inline uint8_t bcm283x_spi_swap(uint8_t word)
{
  word = ((word & 0x0F) << 4) | ((0xF0 & word) >> 4);
  word = ((word & 0xCC) >> 2) | ((0x33 & word) << 2);
  word = ((word & 0xAA) >> 1) | ((0x55 & word) << 1);
  return word;
}

static bool_t bcm283x_spi_transfer_tx(struct device_s *dev);

static bool_t bcm283x_spi_transfer_rx(struct device_s *dev)
{
  struct bcm283x_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (pv->fifo_lvl > 0)
    {
      uint32_t st = cpu_mem_read_32(pv->addr + BCM283X_SPI_CS_ADDR);
      st = BCM283X_SPI_CS_RXD_GET(endian_le32(st));

      if (!st)
        continue;

      uint32_t word = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_SPI_FIFO_ADDR));
      pv->fifo_lvl--;

      if (tr->data.in == NULL)
        continue;

      if (pv->bit_order == DEV_SPI_LSB_FIRST)
        word = bcm283x_spi_swap(word);

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
     return bcm283x_spi_transfer_tx(dev);

#ifdef CONFIG_DEVICE_IRQ
  pv->tr = NULL;
#endif

  return 1;
}

static bool_t bcm283x_spi_transfer_tx(struct device_s *dev)
{
  struct bcm283x_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

#ifdef CONFIG_DEVICE_IRQ
  /* Enable TX DONE interrupt */
  BCM283X_SPI_CS_INTD_SET(pv->ctrl, ENABLED);
  cpu_mem_write_32(pv->addr + BCM283X_SPI_CS_ADDR, endian_le32(pv->ctrl));
#endif

  while (tr->data.count > 0 && pv->fifo_lvl < BCM283X_SPI_FIFO_SIZE)
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

      if (pv->bit_order == DEV_SPI_LSB_FIRST)
        word = bcm283x_spi_swap(word);

      cpu_mem_write_32(pv->addr + BCM283X_SPI_FIFO_ADDR, endian_le32(word));

      tr->data.out = (const void*)((const uint8_t*)tr->data.out + tr->data.out_width);
      tr->data.count--;
      pv->fifo_lvl++;
    }

#ifdef CONFIG_DEVICE_IRQ
  return 0;
#endif

  return bcm283x_spi_transfer_rx(dev);
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(bcm283x_spi_irq)
{
  struct device_s *dev = ep->base.dev;
  struct bcm283x_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  lock_spin(&dev->lock);

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_SPI_CS_ADDR));

  if (BCM283X_SPI_CS_RXD_GET(x))
    {
      /* Disable TX DONE interrupt */
      BCM283X_SPI_CS_INTD_SET(pv->ctrl, DISABLED);
      cpu_mem_write_32(pv->addr + BCM283X_SPI_CS_ADDR, endian_le32(pv->ctrl));

      if (tr != NULL && bcm283x_spi_transfer_rx(dev))
        {
          BCM283X_SPI_CS_TA_SET(pv->ctrl, IDLE);
          cpu_mem_write_32(pv->addr + BCM283X_SPI_CS_ADDR, endian_le32(pv->ctrl));

          kroutine_exec(&tr->kr);
        }
    }

  lock_release(&dev->lock);
}

#endif

static DEV_SPI_CTRL_TRANSFER(bcm283x_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct bcm283x_spi_context_s *pv = dev->drv_pv;
  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    tr->err = -EBUSY;
  else if (tr->cs_op != DEV_SPI_CS_NOP_NOP)
    tr->err = -ENOTSUP;
  else
    {
      assert(tr->data.count > 0);

      pv->tr = tr;
      pv->fifo_lvl = 0;
      tr->err = 0;

      BCM283X_SPI_CS_TA_SET(pv->ctrl, ACTIVE);
      BCM283X_SPI_CS_CLEAR_SET(pv->ctrl, NONE);
      cpu_mem_write_32(pv->addr + BCM283X_SPI_CS_ADDR, endian_le32(pv->ctrl));

      done = bcm283x_spi_transfer_tx(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr);
}

#define bcm283x_spi_cscfg (dev_spi_ctrl_cscfg_t*)dev_driver_notsup_fcn
#define bcm283x_spi_use dev_use_generic

static DEV_INIT(bcm283x_spi_init)
{
  struct bcm283x_spi_context_s	*pv;


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_IOMUX
  /* setup pinmux */
  if (device_iomux_setup(dev, ">clk <miso? >mosi? >cs0? >cs1?", NULL, NULL, NULL))
    goto err_mem;
#endif

  pv->tr = NULL;
  pv->bit_order = DEV_SPI_MSB_FIRST;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_context_init(dev, &pv->spi_ctrl_ctx))
    goto err_mem;
#endif
  pv->ctrl = (BCM283X_SPI_CS_CLEAR(RXTX) |
              BCM283X_SPI_CS_INTR(DISABLED) |
              BCM283X_SPI_CS_INTD(DISABLED) |
              BCM283X_SPI_CS_ADCS(MANUAL) |
              BCM283X_SPI_CS_LEN(SPI));

  cpu_mem_write_32(pv->addr + BCM283X_SPI_CS_ADDR, endian_le32(pv->ctrl));
#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &bcm283x_spi_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_fifo;
#endif


  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(bcm283x_spi_cleanup)
{
  struct bcm283x_spi_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  /* Set to reset state */
  cpu_mem_write_32(pv->addr + BCM283X_SPI_CS_ADDR, 0x1000);
#endif
#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(bcm283x_spi_drv, 0, "BCM283X SPI", bcm283x_spi,
               DRIVER_SPI_CTRL_METHODS(bcm283x_spi));

DRIVER_REGISTER(bcm283x_spi_drv);

