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

#define LOGK_MODULE_ID "1320"

/*

This driver only supports "4-bit" SPI mode of SSD1320 (MOSI, SCK, nCS and D/C).
D/C is used to switch between Data and commands.

Usage:

DEV_DECLARE_STATIC(lcd, "lcd", 0, ssd1320_drv,
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
#include <device/class/spi.h>

struct ssd1320_private_s
{
  struct device_spi_ctrl_s spi;
  struct device_gpio_s *gpio;
  struct device_timer_s *timer;
  struct dev_spi_ctrl_bytecode_rq_s spi_rq;
  dev_request_queue_root_t queue;
  gpio_id_t gpio_map[2];
  gpio_width_t gpio_wmap[2];
};

DRIVER_PV(struct ssd1320_private_s);

#include "ssd1320_spi.o.h"

static DEV_MEM_INFO(ssd1320_info)
{
  if (band_index > 0)
    return -ENOENT;

  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));
  
  info->type = DEV_MEM_RAM;
  info->flags = 0
    | DEV_MEM_WRITABLE
    | DEV_MEM_VOLATILE
    | DEV_MEM_PARTIAL_WRITE
    | DEV_MEM_CROSS_WRITE
    ;
  info->map_base = 0;
  info->size = 160 * 80 / 2;
  info->page_log2 = 7;

  return 0;
}

static
void ssd1320_write(struct device_s *dev,
                  uint8_t page, uint8_t column,
                  const void *data, uint8_t count)
{
  struct ssd1320_private_s *pv = dev->drv_pv;
  logk_trace("%s", __func__);

  pv->spi_rq.base.base.pvdata = dev;
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &ssd1320_bc_write,
                         SSD1320_BC_WRITE_BCARGS(page, column, data, count));
}

static inline
void ssd1320_next(struct device_s *dev)
{
  struct ssd1320_private_s *pv = dev->drv_pv;

  logk_trace("%s", __func__);

  if (pv->spi_rq.base.base.pvdata)
    return;

  struct dev_request_s *rq;

  while ((rq = dev_request_queue_head(&pv->queue))) {
    struct dev_mem_rq_s *mrq = dev_mem_rq_s_cast(rq);

    if (mrq->size == 0) {
      dev_request_queue_remove(&pv->queue, rq);
      kroutine_exec(&rq->kr);
      continue;
    }

    if (mrq->type & DEV_MEM_OP_PAGE_WRITE) {
      uint8_t page = mrq->addr >> 7;
      const uint8_t *buffer = mrq->sc_data[0];

      mrq->addr += 128;
      mrq->sc_data++;
      mrq->size--;

      ssd1320_write(dev, page, 2, buffer, 128);

      return;
    }

    if (mrq->type & DEV_MEM_OP_PARTIAL_WRITE) {
      uint8_t page = mrq->addr >> 7;
      uint8_t col = mrq->addr & 0x7f;
      const uint8_t *buffer = mrq->data;
      uint8_t used = __MIN(mrq->size, 128 - col);

      mrq->addr += used;
      mrq->data += used;
      mrq->size -= used;

      ssd1320_write(dev, page, col + 2, buffer, used);

      return;
    }

    assert(0);

    return;
  }
}

static
KROUTINE_EXEC(ssd1320_spi_done)
{
  struct ssd1320_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s *dev = pv->spi_rq.base.base.pvdata;

  assert(dev);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->spi_rq.base.base.pvdata = NULL;

  ssd1320_next(dev);
}

static
DEV_MEM_REQUEST(ssd1320_request)
{
  struct device_s *dev = accessor->dev;
  struct ssd1320_private_s *pv = dev->drv_pv;

  if (rq->band_mask != 1
      || rq->type & (DEV_MEM_OP_PARTIAL_READ | DEV_MEM_OP_PAGE_READ)
      || !(rq->type & (DEV_MEM_OP_PARTIAL_WRITE | DEV_MEM_OP_PARTIAL_WRITE))
      ) {
    rq->err = -ENOTSUP;
    kroutine_exec(&rq->base.kr);
    return;
  }

  rq->err = 0;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  dev_request_queue_pushback(&pv->queue, &rq->base);
  ssd1320_next(dev);
}

#define ssd1320_use dev_use_generic

static
DEV_INIT(ssd1320_init)
{
  struct ssd1320_private_s *pv;
  error_t err;

  logk_trace("%s", __func__);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  dev_request_queue_init(&pv->queue);

  static const struct dev_spi_ctrl_config_s spi_config = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .bit_rate = 1000000,
    .word_width = 8,
  };

  err = dev_drv_spi_bytecode_init(dev, &pv->spi_rq, &ssd1320_spi_bytecode,
                                  &spi_config, &pv->spi, &pv->gpio, &pv->timer);
  if (err)
    goto err_pv;

  err = device_res_gpio_map(dev, "reset:1 dc:1", pv->gpio_map, pv->gpio_wmap);
  if (err)
    goto err_pv;

  err = device_gpio_map_set_mode(pv->gpio, pv->gpio_map, pv->gpio_wmap, 2,
                                 DEV_PIN_PUSHPULL, DEV_PIN_PUSHPULL);
  if (err)
    goto err_pv;

  pv->spi_rq.base.base.pvdata = NULL;
  pv->spi_rq.gpio_map = pv->gpio_map;
  pv->spi_rq.gpio_wmap = pv->gpio_wmap;

  dev_timer_delay_t reset_latency;
  dev_timer_init_sec(pv->timer, &reset_latency, 0, 1, 1000);

  pv->spi_rq.base.base.pvdata = dev;
  kroutine_init_deferred(&pv->spi_rq.base.base.kr, &ssd1320_spi_done);
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &ssd1320_bc_reset,
                         SSD1320_BC_RESET_BCARGS(reset_latency));

  return 0;

 err_pv:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(ssd1320_cleanup)
{
  logk_trace("%s", __func__);

  struct ssd1320_private_s *pv = dev->drv_pv;

  if (pv->spi_rq.base.base.pvdata)
    return -EBUSY;

  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
  dev_request_queue_destroy(&pv->queue);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(ssd1320_drv, 0, "SSD1320", ssd1320,
               DRIVER_MEM_METHODS(ssd1320));

DRIVER_REGISTER(ssd1320_drv);

