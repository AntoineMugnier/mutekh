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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014
*/

/*
  This driver provides 3 software interfaces: A SPI controller, a
  GPIO controller and an external interrupts controller.

  General purpose inputs and outputs are mapped to GPIO numbers 0-31
  and 32-63 respectively. General purpose inputs are mapped to irq
  sinks 0 to 31. General purpose outputs are mapped to SPI chip select
  lines 0 to 31.

  Actual number of available input and output lines is hardware
  implementation dependent.
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
#include <device/class/icu.h>
#include <device/class/gpio.h>
#include <device/class/spi.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include "soclib_spi.h"

struct soclib_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s            src_ep;
#endif

#ifdef CONFIG_DRIVER_SOCLIB_SPI_ICU
  struct dev_irq_sink_s          *sinks;
  uint32_t                       irq_mask;
#endif

  struct dev_spi_ctrl_transfer_s *tr;

  uint_fast8_t                   fifo_lvl;
  uint_fast8_t                   fifo_size;

#if defined(CONFIG_DRIVER_SOCLIB_SPI_ICU) || defined(CONFIG_DRIVER_SOCLIB_SPI_GPIO)
  uint_fast8_t                   gpin_cnt;
#endif
  uint_fast8_t                   gpout_cnt;

  struct dev_freq_s              freq;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_queue_s    queue;
#endif
};

/****************************************************** SPI controller */

static DEV_SPI_CTRL_CONFIG(soclib_spi_config)
{
  struct device_s *dev = accessor->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;
  else
    {
      uint32_t config = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_CONFIG_ADDR));

      if (cfg->word_width < 1 ||
          cfg->word_width > SOCLIB_SPI_CONFIG_WSIZE_GET(config) + 1 ||
          cfg->miso_pol != cfg->mosi_pol)
        err = -ENOTSUP;
      else
        {
          uint32_t ctrl = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_CTRL_ADDR));

          SOCLIB_SPI_CTRL_WSIZE_SET(ctrl, cfg->word_width);

          SOCLIB_SPI_CTRL_CKPOL_SETVAL(ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_2 ||
                                             cfg->ck_mode == DEV_SPI_CK_MODE_3);

          SOCLIB_SPI_CTRL_CKPHA_SETVAL(ctrl, cfg->ck_mode == DEV_SPI_CK_MODE_1 ||
                                             cfg->ck_mode == DEV_SPI_CK_MODE_3);

          SOCLIB_SPI_CTRL_LSBF_SETVAL(ctrl, cfg->bit_order == DEV_SPI_LSB_FIRST);
          SOCLIB_SPI_CTRL_DPOL_SETVAL(ctrl, cfg->miso_pol == DEV_SPI_CS_ACTIVE_HIGH);

          cpu_mem_write_32(pv->addr + SOCLIB_SPI_CTRL_ADDR, endian_le32(ctrl));

          uint32_t r = pv->freq.num / pv->freq.denom / 2 / cfg->bit_rate;
          if (r < 1)
            err = -ERANGE;

            cpu_mem_write_32(pv->addr + SOCLIB_SPI_CLKDIV_ADDR, endian_le32(r - 1));
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);


  return err;
}

static bool_t soclib_spi_transfer_tx(struct device_s *dev);

static bool_t soclib_spi_transfer_rx(struct device_s *dev)
{
  struct soclib_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (pv->fifo_lvl > 0)
    {
      uint32_t st = cpu_mem_read_32(pv->addr + SOCLIB_SPI_STATUS_ADDR)
                      & endian_le32(SOCLIB_SPI_STATUS_RXEMPTY);

      if (st)
#ifdef CONFIG_DEVICE_IRQ
        return 0;           /* wait for more rx irq */
#else
        continue;
#endif

      uint32_t word = (uint8_t)endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_FIFO_ADDR));
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
    return soclib_spi_transfer_tx(dev);

  /* end of RX */
  pv->tr = NULL;

  return 1;
}

static bool_t soclib_spi_transfer_tx(struct device_s *dev)
{
  struct soclib_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (tr->count > 0 && pv->fifo_lvl < pv->fifo_size)
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

      cpu_mem_write_32(pv->addr + SOCLIB_SPI_FIFO_ADDR, endian_le32((uint8_t)word));

      tr->out = (const void*)((const uint8_t*)tr->out + tr->out_width);
      tr->count--;
      pv->fifo_lvl++;
    }

#ifdef CONFIG_DEVICE_IRQ
  return 0;
#else
  return soclib_spi_transfer_rx(dev);
#endif
}

static DEV_SPI_CTRL_SELECT(soclib_spi_select)
{
  struct device_s *dev = accessor->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  if (cs_id >= pv->gpout_cnt)
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    {
      err = -EBUSY;
    }
  else
    {
      uint32_t mask = 1 << cs_id;
      uint32_t gpout = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPOUT_ADDR)) & ~mask;
      uint32_t csmask = (pt == DEV_SPI_CS_ACTIVE_LOW) << cs_id;

      switch (pc)
        {
        case DEV_SPI_CS_TRANSFER:
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPOUT_ADDR, endian_le32(gpout | csmask));
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_CSTGL_ADDR, endian_le32(mask));
          break;

        case DEV_SPI_CS_RELEASE:
        case DEV_SPI_CS_DEASSERT:
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPOUT_ADDR, endian_le32(gpout | csmask));
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_CSTGL_ADDR, 0);
          break;

        case DEV_SPI_CS_ASSERT:
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPOUT_ADDR, endian_le32(gpout | (csmask ^ mask)));
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_CSTGL_ADDR, 0);
          break;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_SPI_CTRL_TRANSFER(soclib_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;
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
      tr->accessor = accessor;

      pv->fifo_lvl = 0;
      tr->err = 0;

      cpu_mem_write_32(pv->addr + SOCLIB_SPI_TLEN_ADDR, endian_le32(tr->count));

      done = soclib_spi_transfer_tx(dev);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr, cpu_is_interruptible());     /* tail call */
}

#ifdef CONFIG_DEVICE_SPI_REQUEST

static DEV_SPI_CTRL_QUEUE(soclib_spi_queue)
{
  struct device_s *dev = accessor->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;
  return &pv->queue;
}

#endif

/****************************************************** GPIO */

#ifdef CONFIG_DRIVER_SOCLIB_SPI_GPIO

static DEV_GPIO_SET_MODE(soclib_spi_gpio_set_mode)
{
  return ((io_first < 32 && mode != DEV_PIN_INPUT) ||
          (io_first >= 32 && mode != DEV_PIN_PUSHPULL) ||
          (io_first >= 64))
    ? -ENOTSUP : 0;
}

static DEV_GPIO_SET_OUTPUT(soclib_spi_gpio_set_output)
{
  struct device_s *dev = gpio->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;

  if (io_first < 32 || (io_last - io_first) >= pv->gpout_cnt)
    return -ERANGE;

  io_first -= 32;
  io_last -= 32;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t mask = (1 << (io_last - io_first + 1)) - 1;
  uint32_t cm = (~endian_le32_na_load(clear_mask) & mask) << io_first;
  uint32_t sm = (endian_le32_na_load(set_mask) & mask) << io_first;

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPOUT_ADDR));
  uint32_t tg = cm & sm;
  x = ((x ^ tg) & (~cm | sm)) | sm;
  cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPOUT_ADDR, endian_le32(x));

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_GET_INPUT(soclib_spi_gpio_get_input)
{
  struct device_s *dev = gpio->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;

  if (io_first > 32 || (io_last - io_first) >= pv->gpin_cnt)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPIN_ADDR));
  endian_le32_na_store(data, x >> io_first);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

#endif

/****************************************************** ICU */

#ifdef CONFIG_DRIVER_SOCLIB_SPI_ICU

static DEV_ICU_GET_SINK(soclib_spi_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;

  if (id >= pv->gpin_cnt)
    return NULL;
  return pv->sinks + id;
}

static DEV_IRQ_SINK_UPDATE(soclib_spi_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      pv->irq_mask &= ~(1 << sink_id);
      goto end;

    case DEV_IRQ_SENSE_HIGH_LEVEL:
    case DEV_IRQ_SENSE_LOW_LEVEL:
    case DEV_IRQ_SENSE_RISING_EDGE:
    case DEV_IRQ_SENSE_FALLING_EDGE:
      break;

    default:
      return;
    }

  uint32_t mode = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPIRQ_MODE_ADDR));
  if (sense & (DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_LOW_LEVEL))
    SOCLIB_SPI_GPIRQ_MODE_PIN_SET(sink_id, mode, LEVEL);
  else
    SOCLIB_SPI_GPIRQ_MODE_PIN_SET(sink_id, mode, EDGE);
  cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPIRQ_MODE_ADDR, mode);

  uint32_t pol = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPIRQ_POL_ADDR));
  if (sense & (DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_RISING_EDGE))
    SOCLIB_SPI_GPIRQ_POL_PIN_SET(sink_id, pol, RISING);
  else
    SOCLIB_SPI_GPIRQ_POL_PIN_SET(sink_id, pol, FALLING);
  cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPIRQ_POL_ADDR, pol);

  pv->irq_mask |= 1 << sink_id;

 end:
  cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPIN_ADDR, pv->irq_mask);
}

#define soclib_spi_icu_link device_icu_dummy_link

#endif

/*******************************************************/

#define soclib_spi_gpio_request dev_gpio_request_async_to_sync
#define soclib_spi_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn

static DEV_INIT(soclib_spi_init);
static DEV_CLEANUP(soclib_spi_cleanup);

#define soclib_spi_use dev_use_generic

DRIVER_DECLARE(soclib_spi_drv, 0, "Soclib Spi", soclib_spi,
#ifdef CONFIG_DRIVER_SOCLIB_SPI_GPIO
               DRIVER_GPIO_METHODS(soclib_spi_gpio),
#endif
#ifdef CONFIG_DRIVER_SOCLIB_SPI_ICU
               DRIVER_ICU_METHODS(soclib_spi_icu),
#endif
               DRIVER_SPI_CTRL_METHODS(soclib_spi));

DRIVER_REGISTER(soclib_spi_drv,
                DEV_ENUM_FDTNAME_ENTRY("soclib:spi"));

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(soclib_spi_irq)
{
  struct device_s *dev = ep->base.dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t p = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_IRQPEND_ADDR));

      if (p == 0)
        break;

      if (p & (SOCLIB_SPI_IRQPEND_RXFULL | SOCLIB_SPI_IRQPEND_DONE))
        {
          struct dev_spi_ctrl_transfer_s *tr = pv->tr;

          if (tr != NULL && soclib_spi_transfer_rx(dev))
            {
              lock_release(&dev->lock);
              kroutine_exec(&tr->kr, 0);
              lock_spin(&dev->lock);
            }
        }

# ifdef CONFIG_DRIVER_SOCLIB_SPI_ICU
      if (p & SOCLIB_SPI_IRQPEND_GPIRQ)
        {
          uint32_t n = SOCLIB_SPI_IRQPEND_GPIRQN_GET(p);
          assert (n <= pv->gpin_cnt);

          struct dev_irq_sink_s *sink = pv->sinks + n;
          lock_release(&dev->lock);
          device_irq_sink_process(sink, 0);
          lock_spin(&dev->lock);
        }
# endif
    }

  lock_release(&dev->lock);
}

#endif

static DEV_INIT(soclib_spi_init)
{
  struct soclib_spi_context_s	*pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t addr;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    goto err_;

  /* reset controller */
  cpu_mem_write_32(addr + SOCLIB_SPI_CTRL_ADDR, SOCLIB_SPI_CTRL_RESET(RESET));

  uint32_t cfg = endian_le32(cpu_mem_read_32(addr + SOCLIB_SPI_CONFIG_ADDR));
  __unused__ uint_fast8_t gpin_cnt = SOCLIB_SPI_CONFIG_GPINCNT_GET(cfg) + 1;

  pv = mem_alloc(sizeof(*pv)
#ifdef CONFIG_DRIVER_SOCLIB_SPI_ICU
                 + sizeof(pv->sinks[0]) * gpin_cnt
#endif
                 , (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  pv->addr = addr;

  pv->freq.num   = 1000000;
  pv->freq.denom = 1;

  device_get_res_freq(dev, &pv->freq, 0);

  pv->tr = NULL;

  /* get fifo size */
  pv->fifo_size = 2 << SOCLIB_SPI_CONFIG_FSIZE_GET(cfg);

#if defined(CONFIG_DRIVER_SOCLIB_SPI_ICU) || defined(CONFIG_DRIVER_SOCLIB_SPI_GPIO)
  pv->gpin_cnt = gpin_cnt;
#endif
  pv->gpout_cnt = SOCLIB_SPI_CONFIG_GPOUTCNT_GET(cfg) + 1;

#ifdef CONFIG_DRIVER_SOCLIB_SPI_ICU
  pv->sinks = (void*)(pv + 1);
  pv->irq_mask = 0;
  if (gpin_cnt)
    device_irq_sink_init(dev, pv->sinks, gpin_cnt, &soclib_spi_icu_sink_update,
                         DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_LOW_LEVEL |
                         DEV_IRQ_SENSE_RISING_EDGE | DEV_IRQ_SENSE_FALLING_EDGE);
#endif

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_queue_init(dev, &pv->queue))
    goto err_mem;
#endif

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->src_ep, 1,
                         &soclib_spi_irq);

  if (device_irq_source_link(dev, &pv->src_ep, 1, -1))
    goto err_queue;

  cpu_mem_write_32(pv->addr + SOCLIB_SPI_IRQMASK_ADDR,
                   SOCLIB_SPI_IRQMASK_DONE | SOCLIB_SPI_IRQMASK_RXFULL
# ifdef CONFIG_DRIVER_SOCLIB_SPI_ICU
                   | SOCLIB_SPI_IRQMASK_GPIRQ
# endif
                   );
#endif

  dev->drv = &soclib_spi_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_queue:
#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_queue_cleanup(&pv->queue);
#endif
 err_mem:
  mem_free(pv);
 err_:
  return -1;
}

DEV_CLEANUP(soclib_spi_cleanup)
{
  struct soclib_spi_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
  cpu_mem_write_32(pv->addr + SOCLIB_SPI_IRQMASK_ADDR, 0);
  device_irq_source_unlink(dev, &pv->src_ep, 1);
#endif

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_queue_cleanup(&pv->queue);
#endif

  mem_free(pv);
}
