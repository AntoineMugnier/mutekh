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

    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "hts2"

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

#include "hts221.h"
#include "hts221_i2c.o.h"

enum hts221_state_e
{
  HTS221_INITING,
  HTS221_IDLE,
  HTS221_READING,
  HTS221_WAITING,
  HTS221_ERROR,
};

struct hts221_private_s
{
  struct device_i2c_ctrl_s i2c;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
  struct device_timer_s *timer;
  struct dev_timer_rq_s timer_rq;

  dev_request_queue_root_t queue;

  uint32_t t0_mk;
  uint32_t mk_per_lsb;
  uint32_t h0_t0_milx256;
  uint32_t milx256_per_lsb;

  uint32_t last_temp;
  int32_t temp_delta;

  uint16_t last_humi;
  int16_t humi_delta;

  enum hts221_state_e state;
};

DRIVER_PV(struct hts221_private_s);

static void
hts221_error(struct device_s *dev, struct hts221_private_s *pv, error_t error)
{
  pv->state = HTS221_ERROR;

  struct dev_valio_rq_s *rq;
  while ((rq = dev_valio_rq_pop(&pv->queue))) {
    rq->error = error;
    dev_valio_rq_done(rq);
  }
}

static
void hts221_read(struct device_s *dev)
{
  struct hts221_private_s *pv = dev->drv_pv;

  logk_trace("%s %d", __func__, pv->state);

  if (pv->state == HTS221_WAITING) {
    error_t err = DEVICE_OP(pv->timer, cancel, &pv->timer_rq);

    if (err)
      return;

    pv->state = HTS221_IDLE;
  }

  if (pv->state != HTS221_IDLE)
    return;

  pv->state = HTS221_READING;
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &hts221_bc_read, 0);
}

static
void hts221_wait(struct device_s *dev)
{
  struct hts221_private_s *pv = dev->drv_pv;

  logk_trace("%s %d", __func__, pv->state);

  if (pv->state != HTS221_IDLE)
    return;

  pv->state = HTS221_WAITING;
  DEVICE_OP(pv->timer, request, &pv->timer_rq);
}

static
KROUTINE_EXEC(hts221_read_done)
{
  struct hts221_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;
  const uint8_t *buf = bc_get_bytepack(&pv->i2c_rq.vm, HTS221_I2C_BCGLOBAL_BUFFER);

  logk_trace("%s %d", __func__, pv->i2c_rq.error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->i2c_rq.error) {
    hts221_error(dev, pv, pv->i2c_rq.error);
    return;
  }

  pv->state = HTS221_IDLE;

  uint32_t temp = pv->t0_mk
    + (int16_t)endian_le16_na_load(buf + 2) * pv->mk_per_lsb;

  uint16_t humi = ((int16_t)endian_le16_na_load(buf) * pv->milx256_per_lsb
                   + pv->h0_t0_milx256) / 256;

  bool_t temp_changed = __ABS(pv->last_temp - temp) >= pv->temp_delta;
  bool_t humi_changed = __ABS(pv->last_humi - humi) >= pv->humi_delta;

  logk_trace("%s %dmK, %d/10%%H", __func__, temp, humi);

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
    hts221_wait(dev);
}

static
KROUTINE_EXEC(hts221_wait_done)
{
  struct hts221_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->timer_rq.pvdata;

  logk_trace("%s %d", __func__, pv->state);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->state == HTS221_WAITING)
    pv->state = HTS221_IDLE;

  if (!dev_rq_queue_isempty(&pv->queue))
    hts221_read(dev);
}

static
DEV_VALIO_REQUEST(hts221_request)
{
  struct device_s *dev = accessor->dev;
  struct hts221_private_s *pv = dev->drv_pv;

  logk_trace("%s %p type %d att %d", __func__, req, rq->type, rq->attribute);

  if (rq->type == DEVICE_VALIO_WRITE) {
    rq->error = -EINVAL;
    dev_valio_rq_done(req);
    return;
  }

  if (rq->attribute != VALIO_TEMPERATURE_VALUE &&
      rq->attribute != VALIO_HUMIDITY) {
    rq->error = -EINVAL;
    dev_valio_rq_done(req);
    return;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->state == HTS221_ERROR) {
    rq->error = -EIO;
    dev_valio_rq_done(req);
  } else {
    rq->error = 0;
    dev_valio_rq_pushback(&pv->queue, req);

    if (rq->type == DEVICE_VALIO_READ)
      hts221_read(dev);
    else
      hts221_wait(dev);
  }
}

static
DEV_VALIO_CANCEL(hts221_cancel)
{
  struct device_s *dev = accessor->dev;
  struct hts221_private_s *pv = dev->drv_pv;

  logk_trace("%s %p", __func__, req);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      if (rq == req) {
        dev_valio_rq_remove(&pv->queue, rq);
        return 0;
      }
    });

  return -ENOENT;
}

static void
hts221_clean(struct device_s *dev)
{
  struct hts221_private_s *pv = dev->drv_pv;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
  dev_rq_queue_destroy(&pv->queue);
  dev->drv_pv = NULL;
  mem_free(pv);
}

#define cal(x) (buf + (HTS221_CAL_ ## x - HTS221_REG_CALIB))

static
KROUTINE_EXEC(hts221_init_done)
{
  struct hts221_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;
  const uint8_t *buf = bc_get_bytepack(&pv->i2c_rq.vm, HTS221_I2C_BCGLOBAL_BUFFER);

  logk_trace("%s %d", __func__, pv->i2c_rq.error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->i2c_rq.error) {
    hts221_error(dev, pv, pv->i2c_rq.error);
    hts221_clean(dev);
    device_async_init_done(dev, pv->i2c_rq.error);
    return;
  }

  pv->state = HTS221_IDLE;
  device_async_init_done(dev, 0);

  uint32_t t_msb = *cal(T1_T0_msb);
  uint32_t t0_degx8 = ((t_msb & 3) << 8) | *cal(T0_degC_x8);
  uint32_t t1_degx8 = (((t_msb >> 2) & 3) << 8) | *cal(T1_degC_x8);
  int16_t t0_value = endian_le16_na_load(cal(T0_OUT));
  int16_t t1_value = endian_le16_na_load(cal(T1_OUT));

  pv->mk_per_lsb = (int32_t)(t1_degx8 - t0_degx8) * 125
    / (int32_t)(t1_value - t0_value);
  pv->t0_mk = t0_degx8 * 125 + 273150 - t0_value * pv->mk_per_lsb;

  uint32_t h0_rhx2 = *cal(H0_rH_x2);
  uint32_t h1_rhx2 = *cal(H1_rH_x2);
  int16_t h0_t0_value = endian_le16_na_load(cal(H0_T0_OUT));
  int16_t h1_t0_value = endian_le16_na_load(cal(H1_T0_OUT));

  pv->milx256_per_lsb = (int32_t)(h1_rhx2 - h0_rhx2) * 5 * 256
    / (int32_t)(h1_t0_value - h0_t0_value);
  pv->h0_t0_milx256 = h0_rhx2 * 5 * 256 - h0_t0_value * pv->milx256_per_lsb;

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, &hts221_read_done);
  hts221_read(dev);
}

#define hts221_use dev_use_generic

static
DEV_INIT(hts221_init)
{
  struct hts221_private_s *pv;
  uintptr_t period, delta;
  dev_timer_delay_t delay;
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

  pv->state = HTS221_INITING;
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &hts221_i2c_bytecode,
                                  &pv->i2c, NULL, &pv->timer);
  if (err)
    goto err_queue;

  dev_timer_rq_init(&pv->timer_rq, &hts221_wait_done);
  pv->timer_rq.pvdata = dev;
  dev_timer_init_sec(pv->timer, &pv->timer_rq.delay, 0, period, 1000);

  dev_timer_init_sec(pv->timer, &delay, 0, 1, 1000);
  if (!delay)
    delay = 1;

  bc_set_reg(&pv->i2c_rq.vm, HTS221_I2C_BCGLOBAL_CONV_TIME, delay);

  pv->i2c_rq.pvdata = dev;
  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, &hts221_init_done);
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &hts221_bc_initialize, 0);

  return -EAGAIN;

 err_queue:
  dev_rq_queue_destroy(&pv->queue);
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(hts221_cleanup)
{
  struct hts221_private_s *pv = dev->drv_pv;

  if (pv->state != HTS221_IDLE &&
      pv->state != HTS221_ERROR)
    return -EBUSY;

  hts221_clean(dev);

  return 0;
}

DRIVER_DECLARE(hts221_drv, 0, "HTS221", hts221,
               DRIVER_VALIO_METHODS(hts221));

DRIVER_REGISTER(hts221_drv);
