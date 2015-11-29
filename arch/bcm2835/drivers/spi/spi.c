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

#include <arch/bcm2835_spi.h>

#define BCM2835_SPI_FIFO_SIZE 16
#define BCM2835_CS_COUNT 3
#define BCM2835_SPI_CORE_CLK 250000000

struct bcm2835_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s            irq_ep;
#endif
  struct dev_spi_ctrl_transfer_s *tr;
  uint32_t                       ctrl;
  uint_fast8_t                   fifo_lvl;
  enum dev_spi_bit_order_e       bit_order;
  enum dev_spi_cs_policy_e       cs_policy;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_queue_s    queue;
#endif
};

static DEV_SPI_CTRL_CONFIG(bcm2835_spi_config)
{
//  printk("CONFIG\n");
  struct device_s *dev = accessor->dev;
  struct bcm2835_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;

  if (cfg->word_width != 8)
    err = -ENOTSUP;

  pv->bit_order = cfg->bit_order;

  BCM2835_SPI_CS_CPOL_SETVAL(pv->ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_2 ||
                                       cfg->ck_mode == DEV_SPI_CK_MODE_3);

  BCM2835_SPI_CS_CPHA_SETVAL(pv->ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_1 ||
                                       cfg->ck_mode == DEV_SPI_CK_MODE_3);

  cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, endian_le32(pv->ctrl));

  /* DIV must be rounded down to the nearest power of 2 */

  uint32_t div = (BCM2835_SPI_CORE_CLK/cfg->bit_rate);

  uint_fast8_t i = __builtin_clz(div);

  if (i < 16)
    i = 16;

  div = 1 << (sizeof(div) * 8 - 1 - i);
 
  cpu_mem_write_32(pv->addr + BCM2835_SPI_CLK_ADDR, endian_le32(div));

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static bool_t bcm2835_spi_transfer_tx(struct device_s *dev);

static bool_t bcm2835_spi_transfer_rx(struct device_s *dev)
{
  struct bcm2835_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (pv->fifo_lvl > 0)
    {
      uint32_t st = cpu_mem_read_32(pv->addr + BCM2835_SPI_CS_ADDR);
      st = BCM2835_SPI_CS_RXD_GET(endian_le32(st));

      if (!st)
        continue;

      uint32_t word = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_SPI_FIFO_ADDR));
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
     return bcm2835_spi_transfer_tx(dev);

#ifdef CONFIG_DEVICE_IRQ
  pv->tr = NULL;
#endif

  return 1;
}

static bool_t bcm2835_spi_transfer_tx(struct device_s *dev)
{
  struct bcm2835_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

#ifdef CONFIG_DEVICE_IRQ
  /* Enable TX DONE interrupt */
  BCM2835_SPI_CS_INTD_SET(pv->ctrl, ENABLED);
  cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, endian_le32(pv->ctrl));
#endif

  while (tr->count > 0 && pv->fifo_lvl < BCM2835_SPI_FIFO_SIZE)
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

      if (pv->bit_order == DEV_SPI_LSB_FIRST)
        {
          word = ((word & 0x0F) << 4) | ((0xF0 & word) >> 4);
          word = ((word & 0xCC) >> 2) | ((0x33 & word) << 2);
          word = ((word & 0xAA) >> 1) | ((0x55 & word) << 1);
        }

      cpu_mem_write_32(pv->addr + BCM2835_SPI_FIFO_ADDR, endian_le32(word));

      tr->out = (const void*)((const uint8_t*)tr->out + tr->out_width);
      tr->count--;
      pv->fifo_lvl++;
    }

#ifdef CONFIG_DEVICE_IRQ
  return 0;
#endif

  return bcm2835_spi_transfer_rx(dev);
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(bcm2835_spi_irq)
{
  struct device_s *dev = ep->base.dev;
  struct bcm2835_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  lock_spin(&dev->lock);

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_SPI_CS_ADDR));

  if (BCM2835_SPI_CS_RXD_GET(x))
    {
      /* Disable TX DONE interrupt */
      BCM2835_SPI_CS_INTD_SET(pv->ctrl, DISABLED);
      cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, endian_le32(pv->ctrl));

      if (tr != NULL && bcm2835_spi_transfer_rx(dev))
        {
          if (pv->cs_policy == DEV_SPI_CS_TRANSFER)
            {  
              BCM2835_SPI_CS_TA_SET(pv->ctrl, IDLE);
              cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, endian_le32(pv->ctrl));
            }

          lock_release(&dev->lock);
          kroutine_exec(&tr->kr);
          lock_spin(&dev->lock);
        }
    }

  lock_release(&dev->lock);
}

#endif

static DEV_SPI_CTRL_SELECT(bcm2835_spi_select)
{
  struct device_s *dev = accessor->dev;
  struct bcm2835_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  if (cs_id > BCM2835_CS_COUNT)
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;
  else
    {
      BCM2835_SPI_CS_CSPOL_SETVAL(cs_id, pv->ctrl, pt == DEV_SPI_ACTIVE_HIGH);
      BCM2835_SPI_CS_CS_SETVAL(pv->ctrl, cs_id);
  
      switch (pc)
        {
        case DEV_SPI_CS_ASSERT:
          BCM2835_SPI_CS_TA_SET(pv->ctrl, ACTIVE);
          break;
        case DEV_SPI_CS_RELEASE:
        case DEV_SPI_CS_TRANSFER:
          BCM2835_SPI_CS_TA_SET(pv->ctrl, IDLE);
          break;
        case DEV_SPI_CS_DEASSERT:
          err = -ENOTSUP;
          break;
        }

      pv->cs_policy = pc;
      cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, endian_le32(pv->ctrl));
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_SPI_CTRL_TRANSFER(bcm2835_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct bcm2835_spi_context_s *pv = dev->drv_pv;
  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    tr->err = -EBUSY;
  else
    {
      assert(tr->count > 0);

      pv->tr = tr;
      pv->fifo_lvl = 0;
      tr->err = 0;

      BCM2835_SPI_CS_TA_SET(pv->ctrl, ACTIVE); 
      BCM2835_SPI_CS_CLEAR_SET(pv->ctrl, NONE);
      cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, endian_le32(pv->ctrl));

      done = bcm2835_spi_transfer_tx(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr);
}

#ifdef CONFIG_DEVICE_SPI_REQUEST

static DEV_SPI_CTRL_QUEUE(bcm2835_spi_queue)
{
  struct device_s *dev = accessor->dev;
  struct bcm2835_spi_context_s *pv = dev->drv_pv;
  return &pv->queue;
}

#endif

static DEV_INIT(bcm2835_spi_init);
static DEV_CLEANUP(bcm2835_spi_cleanup);

#define bcm2835_spi_use dev_use_generic

DRIVER_DECLARE(bcm2835_spi_drv, 0, "BCM2835 SPI", bcm2835_spi,
               DRIVER_SPI_CTRL_METHODS(bcm2835_spi));

DRIVER_REGISTER(bcm2835_spi_drv);

static DEV_INIT(bcm2835_spi_init)
{
  struct bcm2835_spi_context_s	*pv;
  device_mem_map(dev , 1 << 0 );

  dev->status = DEVICE_DRIVER_INIT_FAILED;

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
  if (dev_spi_queue_init(dev, &pv->queue))
    goto err_mem;
#endif
  pv->ctrl = (BCM2835_SPI_CS_CLEAR(RXTX) |
              BCM2835_SPI_CS_INTR(DISABLED) |
              BCM2835_SPI_CS_INTD(DISABLED) |
              BCM2835_SPI_CS_ADCS(MANUAL) |
              BCM2835_SPI_CS_LEN(SPI));

  cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, endian_le32(pv->ctrl));
#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &bcm2835_spi_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_fifo;
#endif

  dev->drv = &bcm2835_spi_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(bcm2835_spi_cleanup)
{
  struct bcm2835_spi_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  /* Set to reset state */
  cpu_mem_write_32(pv->addr + BCM2835_SPI_CS_ADDR, 0x1000);
#endif
#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_queue_cleanup(&pv->queue);
#endif

  mem_free(pv);
}
