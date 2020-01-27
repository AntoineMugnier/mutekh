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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "1306"

/*

This driver only supports "4-bit" SPI mode of SSD1306 (MOSI, SCK, nCS and D/C).
D/C is used to switch between Data and commands.

Usage:

DEV_DECLARE_STATIC(lcd, "lcd", 0, sd1306_drv,
                   DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
                   DEV_STATIC_RES_DEV_TIMER("rtc* timer*"),
                   DEV_STATIC_RES_DEV_PARAM("gpio", "/gpio"),
                   DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", 24),
                   DEV_STATIC_RES_GPIO("reset", 23, 1),
                   DEV_STATIC_RES_GPIO("dc", 20, 1),
                   );

 */

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <device/class/mem.h>
#include <device/class/cmu.h>
#include <device/class/spi.h>

enum gpio_id_e
{
  GPIO_ID_RESET,
  GPIO_ID_DC,
  GPIO_COUNT,
};

struct sd1306_private_s
{
  struct device_spi_ctrl_s spi;
  struct device_gpio_s *gpio;
  struct device_timer_s *timer;
  struct dev_spi_ctrl_bytecode_rq_s spi_rq;
  dev_request_queue_root_t queue;
  gpio_id_t gpio_map[GPIO_COUNT];
  gpio_width_t gpio_wmap[GPIO_COUNT];
#if defined(CONFIG_DRIVER_MEM_SD1306_POWER_GATING)
  struct dev_clock_sink_ep_s power_source;
#endif
};

DRIVER_PV(struct sd1306_private_s);

#include "sd1306_spi.o.h"

static DEV_MEM_INFO(sd1306_info)
{
  if (band_index > 0)
    return -ENOENT;

  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));
  
  info->type = DEV_MEM_RAM;
  info->flags = 0
    | DEV_MEM_VOLATILE
    | DEV_MEM_PARTIAL_WRITE
    | DEV_MEM_CROSS_WRITE
    ;
  info->map_base = 0;
  info->size = 128 * 64 / 8;
  info->page_log2 = 7;

  return 0;
}

static
void sd1306_write(struct device_s *dev,
                  uint8_t page, uint8_t column,
                  const void *data, uint8_t count)
{
  struct sd1306_private_s *pv = dev->drv_pv;
  logk_trace("%s", __func__);

  pv->spi_rq.pvdata = dev;
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sd1306_bc_write,
                         SD1306_BC_WRITE_BCARGS(page, column, data, count));
}

static inline
void sd1306_next(struct device_s *dev)
{
  struct sd1306_private_s *pv = dev->drv_pv;

  logk_trace("%s", __func__);

  if (pv->spi_rq.pvdata)
    return;

  struct dev_mem_rq_s *mrq;

  while ((mrq = dev_mem_rq_head(&pv->queue))) {
    if (mrq->partial.size == 0) {
      dev_mem_rq_remove(&pv->queue, mrq);
      dev_mem_rq_done(mrq);
      continue;
    }

    uint8_t page = mrq->partial.addr >> 7;
    uint8_t col = mrq->partial.addr & 0x7f;
    const uint8_t *buffer = mrq->partial.data;
    uint8_t used = __MIN(mrq->partial.size, 128 - col);

    mrq->partial.addr += used;
    mrq->partial.data += used;
    mrq->partial.size -= used;

    sd1306_write(dev, page, col, buffer, used);

    return;
  }
}

static
KROUTINE_EXEC(sd1306_spi_done)
{
  struct sd1306_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s *dev = pv->spi_rq.pvdata;

  assert(dev);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->spi_rq.pvdata = NULL;

  sd1306_next(dev);
}

static
DEV_MEM_REQUEST(sd1306_request)
{
  struct device_s *dev = accessor->dev;
  struct sd1306_private_s *pv = dev->drv_pv;

  if (rq->type != DEV_MEM_OP_PARTIAL_WRITE) {
    rq->error = -ENOTSUP;
    dev_mem_rq_done(rq);
    return;
  }

  rq->error = 0;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  dev_mem_rq_pushback(&pv->queue, rq);
  sd1306_next(dev);
}

static inline void
sd1306_enable(struct device_s *dev)
{
  struct sd1306_private_s *pv = dev->drv_pv;

#if defined(CONFIG_DRIVER_MEM_SD1306_POWER_GATING)
  dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_POWER);
#endif
  dev_gpio_out(pv->gpio, pv->spi_rq.base.cs_cfg.id, 1);

  assert(pv->spi_rq.pvdata == NULL);
  pv->spi_rq.pvdata = dev;

  dev_timer_delay_t reset_latency;
  dev_timer_init_sec(pv->timer, &reset_latency, 0, 1, 1000);

  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &sd1306_bc_reset,
                         SD1306_BC_RESET_BCARGS(reset_latency));
}

static inline void
sd1306_disable(struct device_s *dev)
{
  struct sd1306_private_s *pv = dev->drv_pv;

#if defined(CONFIG_DRIVER_MEM_SD1306_POWER_GATING)
  dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_NONE);
#endif
  dev_gpio_out(pv->gpio, pv->gpio_map[GPIO_ID_RESET], 0);
  dev_gpio_out(pv->gpio, pv->spi_rq.base.cs_cfg.id, 0);
}

static DEV_USE(sd1306_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      if (!dev->start_count)
	sd1306_enable(dev);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      if (!dev->start_count)
	device_sleep_schedule(dev);
      return 0;
    }
#endif

    case DEV_USE_SLEEP: {
      struct device_s *dev = param;
      if (dev->start_count)
	return -EAGAIN;
      sd1306_disable(dev);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static
DEV_INIT(sd1306_init)
{
  struct sd1306_private_s *pv;
  error_t err;

  logk_trace("%s", __func__);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  dev_rq_queue_init(&pv->queue);

#if defined(CONFIG_DRIVER_MEM_SD1306_POWER_GATING)
  err = dev_drv_clock_init(dev, &pv->power_source, 0, 0, NULL);
  if (err)
    goto err_pv;
#endif
  
  static const struct dev_spi_ctrl_config_s spi_config = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .bit_rate1k = 7811,
    .word_width = 8,
  };

  err = dev_drv_spi_bytecode_init(dev, &pv->spi_rq, &sd1306_spi_bytecode,
                                  &spi_config, &pv->spi, &pv->gpio, &pv->timer);
  if (err)
    goto err_pv;

  err = device_gpio_setup(pv->gpio, dev,
                          ">reset:1 >dc:1", pv->gpio_map, pv->gpio_wmap);
  if (err)
    goto err_pv;

  pv->spi_rq.pvdata = NULL;
  pv->spi_rq.gpio_map = pv->gpio_map;
  pv->spi_rq.gpio_wmap = pv->gpio_wmap;

  dev_spi_ctrl_rq_init(&pv->spi_rq.base, &sd1306_spi_done);

  logk_trace("%s done", __func__);

  return 0;

 err_pv:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(sd1306_cleanup)
{
  logk_trace("%s", __func__);

  struct sd1306_private_s *pv = dev->drv_pv;

  if (pv->spi_rq.pvdata)
    return -EBUSY;

  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
  dev_rq_queue_destroy(&pv->queue);
#if defined(CONFIG_DRIVER_MEM_SD1306_POWER_GATING)
  dev_drv_clock_cleanup(dev, &pv->power_source);
#endif
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(sd1306_drv, 0, "SD1306", sd1306,
               DRIVER_MEM_METHODS(sd1306));

DRIVER_REGISTER(sd1306_drv);
