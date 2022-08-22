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

    Copyright (c) 2019, Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "25hb"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <device/class/valio.h>
#include <device/class/spi.h>

#include <device/valio/pressure.h>

#include "lps25hb.h"

enum lps25hb_state_e
{
  LPS25HB_UNINITIALIZED,
  LPS25HB_RESETTING,
  LPS25HB_INITIALIZING,
  LPS25HB_IDLE,
  LPS25HB_READING,
  LPS25HB_WAITING,
};

struct lps25hb_private_s
{
  struct device_spi_ctrl_s spi;
  struct dev_spi_ctrl_transaction_rq_s spi_rq;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;

  dev_request_queue_root_t queue;

  uint8_t buffer[16];

  enum lps25hb_state_e state;
  
  int32_t mpa;
  int32_t mpa_delta;
};

DRIVER_PV(struct lps25hb_private_s);

static
void lps25hb_initialize(struct device_s *dev)
{
  struct lps25hb_private_s *pv = dev->drv_pv;

  logk_debug("%s initialize", __func__, pv->state);

  assert(pv->state == LPS25HB_UNINITIALIZED);
  pv->state = LPS25HB_RESETTING;

  pv->spi_rq.data.count = 2;
  pv->spi_rq.data.out = pv->buffer;
  pv->spi_rq.data.in = pv->buffer;
  pv->spi_rq.data.in_width = 1;
  pv->spi_rq.data.out_width = 1;
  memset(pv->buffer, 0, 6);
  pv->buffer[0] = 0x61;
  pv->buffer[1] = 0x80;
  
  dev_spi_transaction_start(&pv->spi, &pv->spi_rq);
}

static
void lps25hb_read(struct device_s *dev)
{
  struct lps25hb_private_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  if (pv->state == LPS25HB_UNINITIALIZED)
    return lps25hb_initialize(dev);

  if (pv->state != LPS25HB_IDLE)
    return;

  pv->state = LPS25HB_READING;
  pv->spi_rq.data.count = 6;
  pv->spi_rq.data.out = pv->buffer;
  pv->spi_rq.data.in = pv->buffer;
  pv->spi_rq.data.in_width = 1;
  pv->spi_rq.data.out_width = 1;
  memset(pv->buffer, 0, 6);
  pv->buffer[0] = 0xe8;

  dev_spi_transaction_start(&pv->spi, &pv->spi_rq);
}

static
void lps25hb_wait(struct device_s *dev)
{
  struct lps25hb_private_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  pv->state = LPS25HB_WAITING;

  DEVICE_OP(&pv->timer, request, &pv->timer_rq);
}

static
void lps25hb_pressure_update(struct device_s *dev, uint32_t mpa, uint16_t mk)
{
  struct lps25hb_private_s *pv = dev->drv_pv;

  bool_t changed = __ABS(pv->mpa - mpa) >= pv->mpa_delta;

  logk_debug("%s old %d new %d", __func__, pv->mpa, mpa);

  if (changed)
    pv->mpa = mpa;

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      struct valio_pressure_s *press = rq->data;

      if (rq->type == DEVICE_VALIO_WAIT_EVENT && !changed)
        GCT_FOREACH_CONTINUE;

      press->mpa = mpa;
      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });
}

static
DEV_VALIO_REQUEST(lps25hb_request)
{
  struct device_s *dev = accessor->dev;
  struct lps25hb_private_s *pv = dev->drv_pv;
  (void)pv;

  logk_debug("%s %p", __func__, rq);

  if (rq->type == DEVICE_VALIO_WRITE
      || rq->attribute != VALIO_PRESSURE_VALUE) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);
  rq->error = 0;
  dev_valio_rq_pushback(&pv->queue, rq);

  if (rq->type == DEVICE_VALIO_READ)
    lps25hb_read(dev);
  LOCK_RELEASE_IRQ(&dev->lock);
}

static
DEV_VALIO_CANCEL(lps25hb_cancel)
{
  struct device_s *dev = accessor->dev;
  struct lps25hb_private_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s %p", __func__, rq);

  GCT_FOREACH(dev_request_queue, &pv->queue, item_, {
      struct dev_valio_rq_s *item = dev_valio_rq_s_cast(item_);

      if (rq != item)
        GCT_FOREACH_CONTINUE;

      dev_valio_rq_remove(&pv->queue, item);
      err = 0;
      GCT_FOREACH_BREAK;
    });

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define lps25hb_use dev_use_generic

static
KROUTINE_EXEC(lps25hb_spi_done)
{
  struct lps25hb_private_s *pv = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s *dev = pv->spi_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
 
  logk_debug("state %d", pv->state);

  if (pv->spi_rq.error) {
    logk_debug("SPI rq error %d", pv->spi_rq.error);
  }

  if (pv->state == LPS25HB_RESETTING) {
    pv->state = LPS25HB_INITIALIZING;

    pv->spi_rq.data.count = 4;
    pv->spi_rq.data.out = pv->buffer;
    pv->spi_rq.data.in = pv->buffer;
    pv->spi_rq.data.in_width = 1;
    pv->spi_rq.data.out_width = 1;
    memset(pv->buffer, 0, 6);
    pv->buffer[0] = 0x60;
    pv->buffer[1] = 0xc0; // 0x20
    pv->buffer[2] = 0x08; // 0x21
    pv->buffer[3] = 0x00; // 0x22
  
    dev_spi_transaction_start(&pv->spi, &pv->spi_rq);
    return;

  } else if (pv->state == LPS25HB_INITIALIZING) {
    pv->state = LPS25HB_IDLE;
    logk_debug("init done");
    lps25hb_wait(dev);
  } else {
    pv->state = LPS25HB_IDLE;

    uint32_t hpa_x_4096 = pv->buffer[1] | (pv->buffer[2] << 8) | (pv->buffer[3] << 16);
    int16_t deg_c_x_480 = pv->buffer[4] | (pv->buffer[5] << 8);

    // 4096 lsb / hPa
    uint32_t mpa = hpa_x_4096 * 244 + (hpa_x_4096 >> 12) * 576;
    // 480 lsb / degree
    uint32_t mk = (((int32_t)deg_c_x_480 * 2133) >> 10) + 273150;
  
    lps25hb_pressure_update(dev, mpa, mk);
    lps25hb_wait(dev);
  }
}

static
KROUTINE_EXEC(lps25hb_timer_done)
{
  struct lps25hb_private_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->spi_rq.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s state %d", __func__, pv->state);

  if (pv->state == LPS25HB_WAITING) {
    pv->state = LPS25HB_IDLE;
    if (!dev_rq_queue_isempty(&pv->queue))
      lps25hb_read(dev);
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(lps25hb_init)
{
  struct lps25hb_private_s *pv;
  uintptr_t period, mpa_delta;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER);
  if (err)
    goto free_pv;
  
  dev_rq_queue_init(&pv->queue);

  err = device_get_param_uint(dev, "period", &period);
  if (err)
    period = 1000;

  err = device_get_param_uint(dev, "mpa_delta", &mpa_delta);
  if (err)
    mpa_delta = 1000;

  pv->mpa_delta = mpa_delta;

  static const struct dev_spi_ctrl_config_s spi_cfg = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .bit_rate1k = 900,
    .word_width = 8,
  };
  
  pv->state = LPS25HB_UNINITIALIZED;
  err = dev_drv_spi_transaction_init(dev, &pv->spi_rq, &spi_cfg, &pv->spi, NULL);
  if (err)
    goto put_timer;

  dev_timer_init_sec(&pv->timer, &pv->timer_rq.delay, 0, period, 1000);
  pv->spi_rq.pvdata = dev;
  pv->spi_rq.cs_op = DEV_SPI_CS_SET_CLR;
  pv->timer_rq.pvdata = dev;

  dev_spi_ctrl_rq_init(&pv->spi_rq.base, lps25hb_spi_done);
  dev_timer_rq_init(&pv->timer_rq, lps25hb_timer_done);

  return 0;

 put_timer:
  device_put_accessor(&pv->timer.base);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(lps25hb_cleanup)
{
  struct lps25hb_private_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue)
      || pv->state != LPS25HB_IDLE)
    return -EBUSY;

  dev_drv_spi_transaction_cleanup(&pv->spi, &pv->spi_rq);
  dev_rq_queue_destroy(&pv->queue);
  device_put_accessor(&pv->timer.base);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(lps25hb_drv, 0, "lps25hb", lps25hb,
               DRIVER_VALIO_METHODS(lps25hb));

DRIVER_REGISTER(lps25hb_drv);
