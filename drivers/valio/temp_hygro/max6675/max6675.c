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

#define LOGK_MODULE_ID "6675"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/valio.h>
#include <device/valio/temperature.h>
#include <device/class/spi.h>
#include <device/class/timer.h>

enum max6675_state_e
{
  MAX6675_IDLE,
  MAX6675_READING,
  MAX6675_WAITING,
};

struct max6675_context_s
{
  struct device_spi_ctrl_s spi;
  struct dev_spi_ctrl_transaction_rq_s spi_rq;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;

  dev_request_queue_root_t queue;

  uint16_t rdata;

  enum max6675_state_e state;
  
  int32_t temp;
  int32_t delta;
};

DRIVER_PV(struct max6675_context_s);

static
void max6675_read(struct device_s *dev)
{
  struct max6675_context_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  if (pv->state != MAX6675_IDLE)
    return;

  pv->state = MAX6675_READING;

  dev_spi_transaction_start(&pv->spi, &pv->spi_rq);
}

static
void max6675_wait(struct device_s *dev)
{
  struct max6675_context_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  pv->state = MAX6675_WAITING;

  DEVICE_OP(&pv->timer, request, &pv->timer_rq);
}

static
void max6675_temperature_update(struct device_s *dev, bool_t error, int32_t value)
{
  struct max6675_context_s *pv = dev->drv_pv;

  bool_t changed = __ABS(pv->temp - value) >= pv->delta;

  logk_debug("%s old %d new %d error %d", __func__, pv->temp, value, error);

  if (changed)
    pv->temp = value;

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      struct valio_temperature_s *temp = rq->data;

      if (rq->type == DEVICE_VALIO_WAIT_EVENT && !changed && !error)
        GCT_FOREACH_CONTINUE;

      if (error)
        rq->error = -EIO;

      temp->temperature = value;
      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });
}

static
DEV_VALIO_REQUEST(max6675_request)
{
  struct device_s *dev = accessor->dev;
  struct max6675_context_s *pv = dev->drv_pv;
  (void)pv;

  logk_debug("%s %p", __func__, req);

  if (rq->type == DEVICE_VALIO_WRITE
      || rq->attribute != VALIO_TEMPERATURE_VALUE) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(req);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);
  rq->error = 0;
  dev_valio_rq_pushback(&pv->queue, req);

  if (rq->type == DEVICE_VALIO_READ)
    max6675_read(dev);
  LOCK_RELEASE_IRQ(&dev->lock);
}

static
DEV_VALIO_CANCEL(max6675_cancel)
{
  struct device_s *dev = accessor->dev;
  struct max6675_context_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s %p", __func__, req);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);

      if (rq != req)
        GCT_FOREACH_CONTINUE;

      dev_valio_rq_remove(&pv->queue, rq);
      err = 0;
      GCT_FOREACH_BREAK;
    });

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define max6675_use dev_use_generic

static
KROUTINE_EXEC(max6675_spi_done)
{
  struct max6675_context_s *pv = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s *dev = pv->spi_rq.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s state %d", __func__, pv->state);
  
  pv->state = MAX6675_IDLE;

  uint16_t reg = endian_be16(pv->rdata);
  uint32_t qc = bit_get_range(reg, 3, 14);
  bool_t is_open = bit_get(reg, 2);
  
  max6675_temperature_update(dev, is_open, qc * 1000 / 4 + 273150);
  max6675_wait(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static
KROUTINE_EXEC(max6675_timer_done)
{
  struct max6675_context_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->spi_rq.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s state %d", __func__, pv->state);

  if (pv->state == MAX6675_WAITING) {
    pv->state = MAX6675_IDLE;
    if (!dev_rq_queue_isempty(&pv->queue))
      max6675_read(dev);
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(max6675_init)
{
  struct max6675_context_s *pv;
  uintptr_t period, delta;
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

  err = device_get_param_uint(dev, "delta", &delta);
  if (err)
    delta = 200;

  pv->delta = delta;

  static const struct dev_spi_ctrl_config_s spi_cfg = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .bit_rate1k = 1000,
    .word_width = 8,
  };
  
  //pv->state = MAX6675_UNINITIALIZED;
  err = dev_drv_spi_transaction_init(dev, &pv->spi_rq, &spi_cfg, &pv->spi, NULL);
  if (err)
    goto put_timer;

  dev_timer_init_sec(&pv->timer, &pv->timer_rq.delay, 0, period, 1000);
  pv->spi_rq.pvdata = dev;
  pv->spi_rq.data.count = 2;
  pv->spi_rq.data.out = &pv->rdata;
  pv->spi_rq.data.in = &pv->rdata;
  pv->spi_rq.data.in_width = 1;
  pv->spi_rq.data.out_width = 1;
  pv->timer_rq.pvdata = dev;

  dev_spi_ctrl_rq_init(&pv->spi_rq.base, max6675_spi_done);
  dev_timer_rq_init(&pv->timer_rq, max6675_timer_done);

  return 0;

 put_timer:
  device_put_accessor(&pv->timer.base);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(max6675_cleanup)
{
  struct max6675_context_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue)
      || pv->state != MAX6675_IDLE)
    return -EBUSY;

  dev_drv_spi_transaction_cleanup(&pv->spi, &pv->spi_rq);
  dev_rq_queue_destroy(&pv->queue);
  device_put_accessor(&pv->timer.base);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(max6675_drv, 0, "max6675", max6675,
               DRIVER_VALIO_METHODS(max6675));

DRIVER_REGISTER(max6675_drv);
