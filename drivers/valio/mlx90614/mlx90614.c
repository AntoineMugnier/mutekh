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

#define LOGK_MODULE_ID "mlx9"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/valio.h>
#include <device/valio/temperature.h>
#include <device/class/i2c.h>

#include "mlx90614_regs.h"
#include "mlx90614_i2c.o.h"

enum mlx90614_state_e
{
  MLX90614_UNINITIALIZED,
  MLX90614_IDLE,
  MLX90614_READING,
  MLX90614_WAITING,
};

enum mlx90614_channel_e
{
  MLX90614_CHANNEL_AMBIANT,
  MLX90614_CHANNEL_IR1,
  MLX90614_CHANNEL_IR2,
  MLX90614_CHANNEL_COUNT,
};

struct mlx90614_context_s
{
  struct device_i2c_ctrl_s i2c;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
  struct device_timer_s *timer;
  struct dev_timer_rq_s timer_rq;
  
  enum mlx90614_state_e state;
  dev_request_queue_root_t queue;

  bool_t dual;

  int32_t delta;
  int32_t temp[MLX90614_CHANNEL_COUNT];
};

DRIVER_PV(struct mlx90614_context_s);

static
void mlx90614_read(struct device_s *dev)
{
  struct mlx90614_context_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  if (pv->state == MLX90614_UNINITIALIZED)
    return;

  if (pv->state == MLX90614_READING)
    return;

  if (pv->state == MLX90614_WAITING)
    DEVICE_OP(pv->timer, cancel, &pv->timer_rq);

  pv->state = MLX90614_READING;

  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &mlx90614_sensor_read,
                         MLX90614_SENSOR_READ_BCARGS());
}

static
void mlx90614_wait(struct device_s *dev)
{
  struct mlx90614_context_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  pv->state = MLX90614_WAITING;

  DEVICE_OP(pv->timer, request, &pv->timer_rq);
}

static
void mlx90614_temperature_update(struct device_s *dev, enum mlx90614_channel_e channel, int32_t value)
{
  struct mlx90614_context_s *pv = dev->drv_pv;

  bool_t changed = __ABS(pv->temp[channel] - value) >= pv->delta;

  logk_debug("%s channel %d old %d new %d", __func__,
             channel, pv->temp[channel], value);

  if (changed)
    pv->temp[channel] = value;

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      enum mlx90614_channel_e ch = (uintptr_t)rq->base.drvdata;
      struct valio_temperature_s *temp = rq->data;

      if (ch != channel)
        GCT_FOREACH_CONTINUE;

      if (rq->type == DEVICE_VALIO_WAIT_EVENT && !changed)
        GCT_FOREACH_CONTINUE;

      temp->temperature = value;
      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });
}

static
DEV_VALIO_REQUEST(mlx90614_request)
{
  struct device_s *dev = accessor->dev;
  struct mlx90614_context_s *pv = dev->drv_pv;
  (void)pv;

  logk_debug("%s %p", __func__, rq);

  if (pv->state == MLX90614_UNINITIALIZED) {
    rq->error = -EAGAIN;
    dev_valio_rq_done(rq);
    return;
  }

  if (rq->type == DEVICE_VALIO_WRITE
      || rq->attribute != VALIO_TEMPERATURE_VALUE) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);
  rq->error = 0;
  rq->base.drvdata = (void *)(uintptr_t)accessor->number;
  dev_valio_rq_pushback(&pv->queue, rq);

  if (rq->type == DEVICE_VALIO_READ)
    mlx90614_read(dev);
  LOCK_RELEASE_IRQ(&dev->lock);
}

static
DEV_VALIO_CANCEL(mlx90614_cancel)
{
  struct device_s *dev = accessor->dev;
  struct mlx90614_context_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s %p", __func__, rq);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *frq = dev_valio_rq_s_cast(item);

      if (frq != rq)
        GCT_FOREACH_CONTINUE;

      dev_valio_rq_remove(&pv->queue, frq);
      err = 0;
      GCT_FOREACH_BREAK;
    });

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static
DEV_USE(mlx90614_use)
{
  switch (op) {
  case DEV_USE_GET_ACCESSOR: {
    struct device_accessor_s *accessor = param;
    struct device_s *dev = accessor->dev;
    struct mlx90614_context_s *pv = dev->drv_pv;
    
    if (accessor->number >= MLX90614_CHANNEL_IR2 + pv->dual)
      return -ENOTSUP;
    return 0;
  }

  case DEV_USE_LAST_NUMBER: {
    struct device_accessor_s *accessor = param;
    struct device_s *dev = accessor->dev;
    struct mlx90614_context_s *pv = dev->drv_pv;

    accessor->number = MLX90614_CHANNEL_IR2 + pv->dual - 1;
    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static
KROUTINE_EXEC(mlx90614_i2c_done)
{
  struct mlx90614_context_s *pv = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s state %d", __func__, pv->state);

  if (pv->state == MLX90614_UNINITIALIZED) {
    if (!pv->i2c_rq.error) {
      uint32_t id0 = bc_get_reg(&pv->i2c_rq.vm, MLX90614_DISCOVER_BCOUT_ID0);
      uint32_t id1 = bc_get_reg(&pv->i2c_rq.vm, MLX90614_DISCOVER_BCOUT_ID1);
      uint32_t dual = bc_get_reg(&pv->i2c_rq.vm, MLX90614_DISCOVER_BCOUT_DUAL);
      logk("Found ambiant +%s IR sensor ID 0x%08x%08x",
           dual ? " dual" : "", id1, id0);

      pv->dual = dual;
    }

    device_async_init_done(dev, pv->i2c_rq.error);

    if (pv->i2c_rq.error)
      goto out;
  }
  
  pv->state = MLX90614_IDLE;

  for (size_t i = 0; i < MLX90614_CHANNEL_IR2 + pv->dual; ++i) {
    int16_t raw = bc_get_reg(&pv->i2c_rq.vm, MLX90614_SENSOR_READ_BCOUT_TA + i);
    int32_t mk = (int32_t)raw * 20;

    logk_debug("%s raw=%04x %dmK", __func__, (uint16_t)raw, mk);
  
    mlx90614_temperature_update(dev, i, mk);
  }

  mlx90614_wait(dev);

 out:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static
KROUTINE_EXEC(mlx90614_timer_done)
{
  struct mlx90614_context_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s state %d", __func__, pv->state);

  if (pv->state == MLX90614_WAITING) {
    pv->state = MLX90614_IDLE;
    if (!dev_rq_queue_isempty(&pv->queue))
      mlx90614_read(dev);
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(mlx90614_init)
{
  struct mlx90614_context_s *pv;
  uintptr_t period, delta;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_rq_queue_init(&pv->queue);

  err = device_get_param_uint(dev, "period", &period);
  if (err)
    period = 1000;

  err = device_get_param_uint(dev, "delta", &delta);
  if (err)
    delta = 200;

  pv->delta = delta;
  
  //pv->state = MLX90614_UNINITIALIZED;
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &mlx90614_i2c_bytecode,
                                  &pv->i2c, NULL, &pv->timer);
  if (err)
    goto free_pv;

  dev_timer_init_sec(pv->timer, &pv->timer_rq.delay, 0, period, 1000);

  pv->i2c_rq.pvdata = dev;
  pv->timer_rq.pvdata = dev;

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, mlx90614_i2c_done);
  dev_timer_rq_init(&pv->timer_rq, mlx90614_timer_done);

  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &mlx90614_discover,
                         MLX90614_DISCOVER_BCARGS());

  return -EAGAIN;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(mlx90614_cleanup)
{
  struct mlx90614_context_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue)
      || pv->state != MLX90614_IDLE)
    return -EBUSY;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
  dev_rq_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(mlx90614_drv, 0, "Mlx90614", mlx90614,
               DRIVER_VALIO_METHODS(mlx90614));

DRIVER_REGISTER(mlx90614_drv);
