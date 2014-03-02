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
  struct dev_irq_ep_s            src_ep;
#endif

#ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU
  struct dev_irq_ep_s            *sinks;
  uint32_t                       irq_mask;
#endif

  struct dev_spi_ctrl_transfer_s *tr;

  uint_fast8_t                   fifo_lvl;
  uint_fast8_t                   fifo_size;

#if defined(CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU) || defined(CONFIG_DRIVER_SOCLIB_VCI_SPI_GPIO)
  uint_fast8_t                   gpin_cnt;
#endif
  uint_fast8_t                   gpout_cnt;

  uint32_t                       freq;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_queue_s    queue;
#endif
};

/****************************************************** SPI controller */

static DEVSPI_CTRL_CONFIG(soclib_spi_config)
{
  struct device_s *dev = scdev->dev;
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

          SOCLIB_SPI_CTRL_CKPOL_SETVAL(ctrl, (cfg->ck_mode >> 1) & 1);
          SOCLIB_SPI_CTRL_CKPHA_SETVAL(ctrl, cfg->ck_mode & 1);
          SOCLIB_SPI_CTRL_LSBF_SETVAL(ctrl, cfg->bit_order == DEV_SPI_LSB_FIRST);
          SOCLIB_SPI_CTRL_DPOL_SETVAL(ctrl, cfg->miso_pol == DEV_SPI_CS_ACTIVE_HIGH);

          cpu_mem_write_32(pv->addr + SOCLIB_SPI_CTRL_ADDR, endian_le32(ctrl));

          uint32_t r = pv->freq / 2 / cfg->bit_rate;
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

static DEVSPI_CTRL_SELECT(soclib_spi_select)
{
  struct device_s *dev = scdev->dev;
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

static DEVSPI_CTRL_TRANSFER(soclib_spi_transfer)
{
  struct device_s *dev = scdev->dev;
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
      tr->scdev = scdev;

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

static DEVSPI_CTRL_QUEUE(soclib_spi_queue)
{
  struct device_s *dev = scdev->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;
  return &pv->queue;
}

#endif

static const struct driver_spi_ctrl_s	soclib_spi_ctrl_drv =
{
  .class_		= DRIVER_CLASS_SPI_CTRL,
  .f_config		= soclib_spi_config,
  .f_select		= soclib_spi_select,
  .f_transfer		= soclib_spi_transfer,
#ifdef CONFIG_DEVICE_SPI_REQUEST
  .f_queue		= soclib_spi_queue,
#endif
};

/****************************************************** GPIO */

#ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_GPIO

static DEVGPIO_SET_MODE(soclib_spi_gpio_set_mode)
{
  return ((io_first < 32 && mode != DEV_GPIO_INPUT) ||
          (io_first >= 32 && mode != DEV_GPIO_OUTPUT) ||
          (io_first >= 64))
    ? -ENOTSUP : 0;
}

static DEVGPIO_SET_OUTPUT(soclib_spi_gpio_set_output)
{
  struct device_s *dev = gpio->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;

  if (io_first < 32 || io_last - 32 >= pv->gpin_cnt)
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

static DEVGPIO_GET_INPUT(soclib_spi_gpio_get_input)
{
  struct device_s *dev = gpio->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;

  if (io_last >= pv->gpout_cnt)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPIN_ADDR));
  endian_le32_na_store(data, x >> io_first);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

const struct driver_gpio_s  soclib_spi_gpio_drv =
{
  .class_         = DRIVER_CLASS_GPIO,
  .f_set_mode     = soclib_spi_gpio_set_mode,
  .f_set_output   = soclib_spi_gpio_set_output,
  .f_get_input    = soclib_spi_gpio_get_input,
  .f_watch        = (devgpio_watch_t*)&dev_driver_notsup_fcn,
  .f_cancel       = (devgpio_cancel_t*)&dev_driver_notsup_fcn,
};

#endif

/****************************************************** ICU */

#ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU

static DEVICU_GET_ENDPOINT(soclib_spi_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK: {

      if (id >= pv->gpin_cnt)
        return NULL;
      struct dev_irq_ep_s *ep = pv->sinks + id;
      if (!ep->links_count)
        ep->sense = DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_LOW_LEVEL |
          DEV_IRQ_SENSE_RISING_EDGE | DEV_IRQ_SENSE_FALLING_EDGE;
      return ep;
    }

    case DEV_IRQ_EP_SOURCE:
      if (id == 0)
        return &pv->src_ep;

    default:
      return NULL;
    }
}

static DEVICU_ENABLE_IRQ(soclib_spi_icu_enable_irq)
{
  struct device_s *dev = idev->dev;
  struct soclib_spi_context_s *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;
  bool_t done = 0;

  if (irq_id > 0)
    {
      printk("soclib spi %p: single wire IRQ must use 0 as logical IRQ id for %p device\n", dev, dev_ep->dev);
      return 0;
    }

  LOCK_SPIN_IRQ(&dev->lock);

  uint_fast8_t sense = src->sense & sink->sense;
  uint32_t mask = 1 << icu_in_id;

  if (sense)
    {
      if (!(mask & pv->irq_mask))
        {
          /* if more than one mode is left, keep only the lsb */
          sense = sense & ~(sense - 1);
          src->sense = sink->sense = sense;

          uint32_t mode = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPIRQ_MODE_ADDR));
          if (sense & (DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_LOW_LEVEL))
            SOCLIB_SPI_GPIRQ_MODE_PIN_SET(icu_in_id, mode, LEVEL);
          else
            SOCLIB_SPI_GPIRQ_MODE_PIN_SET(icu_in_id, mode, EDGE);
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPIRQ_MODE_ADDR, mode);

          uint32_t pol = endian_le32(cpu_mem_read_32(pv->addr + SOCLIB_SPI_GPIRQ_POL_ADDR));
          if (sense & (DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_RISING_EDGE))
            SOCLIB_SPI_GPIRQ_POL_PIN_SET(icu_in_id, pol, RISING);
          else
            SOCLIB_SPI_GPIRQ_POL_PIN_SET(icu_in_id, pol, FALLING);
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPIRQ_POL_ADDR, pol);

          /* enable */
          pv->irq_mask |= 1 << icu_in_id;
          cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPIN_ADDR, pv->irq_mask);
        }

      done = 1;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return done;
}

static DEVICU_DISABLE_IRQ(soclib_spi_icu_disable_irq)
{
  struct device_s *dev = idev->dev;
  struct soclib_spi_context_s *pv = idev->dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  LOCK_SPIN_IRQ(&dev->lock);

  pv->irq_mask &= ~(1 << icu_in_id);
  cpu_mem_write_32(pv->addr + SOCLIB_SPI_GPIN_ADDR, pv->irq_mask);

  LOCK_RELEASE_IRQ(&dev->lock);
}

const struct driver_icu_s  soclib_spi_icu_drv =
{
  .class_         = DRIVER_CLASS_ICU,
  .f_get_endpoint = soclib_spi_icu_get_endpoint,
  .f_enable_irq   = soclib_spi_icu_enable_irq,
  .f_disable_irq  = soclib_spi_icu_disable_irq,
};

#endif

/*******************************************************/

static const struct devenum_ident_s  soclib_spi_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("soclib:vci_spi"),
  { 0 }
};

static DEV_INIT(soclib_spi_init);
static DEV_CLEANUP(soclib_spi_cleanup);

const struct driver_s	soclib_spi_drv =
{
  .desc                 = "Soclib VciSpi",
  .id_table             = soclib_spi_ids,
  .f_init		= soclib_spi_init,
  .f_cleanup		= soclib_spi_cleanup,
  .classes              = { &soclib_spi_ctrl_drv,
#ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU
                            &soclib_spi_icu_drv,
#endif
#ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_GPIO
                            &soclib_spi_gpio_drv,
#endif
                            0 }
};

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_EP_PROCESS(soclib_spi_irq)
{
  struct device_s *dev = ep->dev;
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

# ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU
      if (p & SOCLIB_SPI_IRQPEND_GPIRQ)
        {
          uint32_t n = SOCLIB_SPI_IRQPEND_GPIRQN_GET(p);
          assert (n <= pv->gpin_cnt);

          struct dev_irq_ep_s *sink = pv->sinks + n;
          lock_release(&dev->lock);
          sink->process(sink, id);
          lock_spin(&dev->lock);
        }
# endif
    }

  lock_release(&dev->lock);
}

#endif

REGISTER_DRIVER(soclib_spi_drv);

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
#ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU
                 + sizeof(pv->sinks[0]) * gpin_cnt
#endif
                 , (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  pv->addr = addr;

  uint64_t f = 1000000ULL << 24; /* FIXME default freq */
  device_res_get_uint64(dev, DEV_RES_FREQ, 0, &f);
  pv->freq = f >> 24;

  pv->tr = NULL;

  /* get fifo size */
  pv->fifo_size = 2 << SOCLIB_SPI_CONFIG_FSIZE_GET(cfg);

#if defined(CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU) || defined(CONFIG_DRIVER_SOCLIB_VCI_SPI_GPIO)
  pv->gpin_cnt = gpin_cnt;
#endif
  pv->gpout_cnt = SOCLIB_SPI_CONFIG_GPOUTCNT_GET(cfg) + 1;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU
  pv->sinks = (void*)(pv + 1);
  pv->irq_mask = 0;
  if (gpin_cnt)
    device_irq_sink_init(dev, pv->sinks, gpin_cnt,
                         DEV_IRQ_SENSE_HIGH_LEVEL | DEV_IRQ_SENSE_LOW_LEVEL |
                         DEV_IRQ_SENSE_RISING_EDGE | DEV_IRQ_SENSE_FALLING_EDGE);
#endif

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_queue_init(dev, &pv->queue))
    goto err_mem;
#endif

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->src_ep, 1,
                         &soclib_spi_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, &pv->src_ep, 1, -1))
    goto err_queue;

  cpu_mem_write_32(pv->addr + SOCLIB_SPI_IRQMASK_ADDR,
                   SOCLIB_SPI_IRQMASK_DONE | SOCLIB_SPI_IRQMASK_RXFULL
# ifdef CONFIG_DRIVER_SOCLIB_VCI_SPI_ICU
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
