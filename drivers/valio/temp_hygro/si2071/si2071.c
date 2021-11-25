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

    Copyright (c) 2020, Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "2071"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <device/class/valio.h>
#include <device/class/i2c.h>
#include <device/class/timer.h>

#include <device/valio/temperature.h>
#include <device/valio/humidity.h>

#include "si2071.h"
#include "si2071_i2c.o.h"

enum si2071_state_e
{
  SI2071_INITING,
  SI2071_IDLE,
  SI2071_READING,
  SI2071_WAITING,
  SI2071_ERROR,
};

struct si2071_private_s
{
  struct device_i2c_ctrl_s i2c;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
  struct device_timer_s *timer;
  struct dev_timer_rq_s timer_rq;

  dev_request_queue_root_t queue;

  uint32_t last_temp;
  int32_t temp_delta;

  uint32_t last_humi;
  int32_t humi_delta;

  enum si2071_state_e state;
};

DRIVER_PV(struct si2071_private_s);

static void
si2071_error(struct device_s *dev, struct si2071_private_s *pv, error_t error)
{
  pv->state = SI2071_ERROR;

  struct dev_valio_rq_s *rq;
  while ((rq = dev_valio_rq_pop(&pv->queue))) {
    rq->error = error;
    dev_valio_rq_done(rq);
  }
}

static
void si2071_read(struct device_s *dev)
{
  struct si2071_private_s *pv = dev->drv_pv;

  logk_trace("%s %d", __func__, pv->state);

  if (pv->state == SI2071_WAITING) {
    error_t err = DEVICE_OP(pv->timer, cancel, &pv->timer_rq);

    if (err)
      return;

    pv->state = SI2071_IDLE;
  }

  if (pv->state != SI2071_IDLE)
    return;

  pv->state = SI2071_READING;
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &si2071_bc_read, 0);
}

static
void si2071_wait(struct device_s *dev)
{
  struct si2071_private_s *pv = dev->drv_pv;

  logk_trace("%s %d", __func__, pv->state);

  if (pv->state != SI2071_IDLE)
    return;

  pv->state = SI2071_WAITING;
  DEVICE_OP(pv->timer, request, &pv->timer_rq);
}

static
KROUTINE_EXEC(si2071_read_done)
{
  struct si2071_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;
  const uint32_t temp_code = bc_get_reg(&pv->i2c_rq.vm, SI2071_I2C_BCGLOBAL_TEMP);
  const uint32_t humi_code = bc_get_reg(&pv->i2c_rq.vm, SI2071_I2C_BCGLOBAL_HUM);

  logk_trace("%s %d", __func__, pv->i2c_rq.error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->i2c_rq.error) {
    si2071_error(dev, pv, pv->i2c_rq.error);
    return;
  }

  pv->state = SI2071_IDLE;

  // From datasheet: 175.72 * code / 65536 - 46.85
  // We are in millikelvin, and in order not to overflow uint32 with multiplication,
  // divide constants by 8.
  uint32_t temp = 21965 * temp_code / 8192 - 46850 + 273150;
  uint32_t humi = 1250 * humi_code / 65536 - 60;

  bool_t temp_changed = __ABS(pv->last_temp - temp) >= pv->temp_delta;
  bool_t humi_changed = __ABS(pv->last_humi - humi) >= pv->humi_delta;

  logk_trace("%s %dmK, %d/10%%RH", __func__, temp, humi);

  if (temp_changed)
    pv->last_temp = temp;

  if (humi_changed)
    pv->last_humi = humi;

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);

      switch (rq->attribute) {
      case VALIO_TEMPERATURE_VALUE: {
        struct valio_temperature_s *temp = rq->data;

        if (rq->type == DEVICE_VALIO_WAIT_EVENT && !temp_changed)
          GCT_FOREACH_CONTINUE;

        temp->temperature = pv->last_temp;
        break;
      }

      case VALIO_HUMIDITY: {
        struct valio_humidity_s *h = rq->data;

        if (rq->type == DEVICE_VALIO_WAIT_EVENT && !humi_changed)
          GCT_FOREACH_CONTINUE;

        h->mil = pv->last_humi;
        break;
      }
      }
      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });

  if (!dev_rq_queue_isempty(&pv->queue))
    si2071_wait(dev);
}

static
KROUTINE_EXEC(si2071_wait_done)
{
  struct si2071_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->timer_rq.pvdata;

  logk_trace("%s %d", __func__, pv->state);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->state == SI2071_WAITING)
    pv->state = SI2071_IDLE;

  if (!dev_rq_queue_isempty(&pv->queue))
    si2071_read(dev);
}

static
DEV_VALIO_REQUEST(si2071_request)
{
  struct device_s *dev = accessor->dev;
  struct si2071_private_s *pv = dev->drv_pv;

  logk_trace("%s %p type %d att %d", __func__, rq, rq->type, rq->attribute);

  if (rq->type == DEVICE_VALIO_WRITE) {
    rq->error = -EINVAL;
    dev_valio_rq_done(rq);
    return;
  }

  if (rq->attribute != VALIO_TEMPERATURE_VALUE &&
      rq->attribute != VALIO_HUMIDITY) {
    rq->error = -EINVAL;
    dev_valio_rq_done(rq);
    return;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->state == SI2071_ERROR) {
    rq->error = -EIO;
    dev_valio_rq_done(rq);
  } else {
    rq->error = 0;
    dev_valio_rq_pushback(&pv->queue, rq);

    if (rq->type == DEVICE_VALIO_READ)
      si2071_read(dev);
    else
      si2071_wait(dev);
  }
}

static
DEV_VALIO_CANCEL(si2071_cancel)
{
  struct device_s *dev = accessor->dev;
  struct si2071_private_s *pv = dev->drv_pv;

  logk_trace("%s %p", __func__, rq);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq_item = dev_valio_rq_s_cast(item);
      if (rq == rq_item) {
        dev_valio_rq_remove(&pv->queue, rq);
        return 0;
      }
    });

  return -ENOENT;
}

static void
si2071_clean(struct device_s *dev)
{
  struct si2071_private_s *pv = dev->drv_pv;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
  dev_rq_queue_destroy(&pv->queue);
  dev->drv_pv = NULL;
  mem_free(pv);
}

#define cal(x) (buf + (SI2071_CAL_ ## x - SI2071_REG_CALIB))

static
KROUTINE_EXEC(si2071_init_done)
{
  struct si2071_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  logk_trace("%s %d", __func__, pv->i2c_rq.error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->i2c_rq.error) {
    si2071_error(dev, pv, pv->i2c_rq.error);
    si2071_clean(dev);
    device_async_init_done(dev, pv->i2c_rq.error);
    return;
  }

  pv->state = SI2071_IDLE;
  device_async_init_done(dev, 0);

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, &si2071_read_done);
  si2071_read(dev);
}

#define si2071_use dev_use_generic

static
DEV_INIT(si2071_init)
{
  struct si2071_private_s *pv;
  uintptr_t period, delta;
  error_t err;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  dev_rq_queue_init(&pv->queue);

  err = device_get_param_uint(dev, "period", &period);
  if (err)
    period = 1000;

  err = device_get_param_uint(dev, "temperature_delta", &delta);
  if (err)
    delta = 200;

  pv->temp_delta = delta;

  err = device_get_param_uint(dev, "humidity_delta", &delta);
  if (err)
    delta = 10;

  pv->humi_delta = delta;

  pv->state = SI2071_INITING;
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &si2071_i2c_bytecode,
                                  &pv->i2c, NULL, &pv->timer);
  if (err)
    goto err_queue;

  dev_timer_rq_init(&pv->timer_rq, &si2071_wait_done);
  pv->timer_rq.pvdata = dev;
  dev_timer_init_sec(pv->timer, &pv->timer_rq.delay, 0, period, 1000);

  pv->i2c_rq.pvdata = dev;
  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, &si2071_init_done);
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &si2071_bc_initialize, 0);

  return -EAGAIN;

 err_queue:
  dev_rq_queue_destroy(&pv->queue);
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(si2071_cleanup)
{
  struct si2071_private_s *pv = dev->drv_pv;

  if (pv->state != SI2071_IDLE &&
      pv->state != SI2071_ERROR)
    return -EBUSY;

  si2071_clean(dev);

  return 0;
}

DRIVER_DECLARE(si2071_drv, 0, "SI2071", si2071,
               DRIVER_VALIO_METHODS(si2071));

DRIVER_REGISTER(si2071_drv);
