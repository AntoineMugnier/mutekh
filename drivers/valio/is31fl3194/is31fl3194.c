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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2021
*/

#define LOGK_MODULE_ID "3194"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/i2c.h>
#include <device/class/valio.h>
#include <device/valio/led.h>

#include "is31fl3194.h"
#include "is31fl3194_i2c.o.h"

enum fl3194_state_e
{
  FL3194_INITING,
  FL3194_IDLE,
  FL3194_UPDATING,
  FL3194_ERROR,
};

struct fl3194_s
{
  struct device_i2c_ctrl_s i2c;
  struct device_timer_s *timer;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_bc_rq;

  dev_request_queue_root_t queue;

  enum fl3194_state_e state;
};

STRUCT_COMPOSE(fl3194_s, i2c_bc_rq);
DRIVER_PV(struct fl3194_s);

static void
fl3194_error(struct fl3194_s *pv, error_t error)
{
  pv->state = FL3194_ERROR;

  struct dev_valio_rq_s *rq;
  while ((rq = dev_valio_rq_pop(&pv->queue))) {
    rq->error = error;
    dev_valio_rq_done(rq);
  }
}

static void
fl3194_clean(struct device_s *dev)
{
  struct fl3194_s *pv = dev->drv_pv;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_bc_rq);
  dev_rq_queue_destroy(&pv->queue);
  dev->drv_pv = NULL;
  mem_free(pv);
}

static
void fl3194_run(struct device_s *dev)
{
  struct fl3194_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;

  logk_trace("%s %d", __func__, pv->state);

  if (pv->state != FL3194_IDLE)
    return;

  rq = dev_valio_rq_head(&pv->queue);
  if (!rq) {
    logk_trace("%s no rq", __func__);
    return;
  }

  const struct valio_led_luminosity_s *s = rq->data;
  
  pv->state = FL3194_UPDATING;

  logk_trace("%s -> %d %d %d", __func__, s->lum[0], s->lum[1], s->lum[2]);

  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_bc_rq, &fl3194_bc_update,
                         7, s->lum[0], s->lum[1], s->lum[2]);
}

static
KROUTINE_EXEC(fl3194_update_done)
{
  struct dev_i2c_ctrl_rq_s *i2c_rq = dev_i2c_ctrl_rq_from_kr(kr);
  struct dev_i2c_ctrl_bytecode_rq_s *i2c_bc_rq = dev_i2c_ctrl_bytecode_rq_s_cast(i2c_rq);
  struct fl3194_s *pv  = fl3194_s_from_i2c_bc_rq(i2c_bc_rq);
  struct device_s *dev = pv->i2c_bc_rq.pvdata;
  struct dev_valio_rq_s *rq;
  
  logk_trace("%s %d", __func__, pv->i2c_bc_rq.error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->i2c_bc_rq.error) {
    fl3194_error(pv, pv->i2c_bc_rq.error);
    return;
  }

  pv->state = FL3194_IDLE;

  rq = dev_valio_rq_pop(&pv->queue);
  rq->error = 0;
  dev_valio_rq_done(rq);

  fl3194_run(dev);
}

static
KROUTINE_EXEC(fl3194_init_done)
{
  struct dev_i2c_ctrl_rq_s *i2c_rq = dev_i2c_ctrl_rq_from_kr(kr);
  struct dev_i2c_ctrl_bytecode_rq_s *i2c_bc_rq = dev_i2c_ctrl_bytecode_rq_s_cast(i2c_rq);
  struct fl3194_s *pv  = fl3194_s_from_i2c_bc_rq(i2c_bc_rq);
  struct device_s *dev = pv->i2c_bc_rq.pvdata;

  logk_trace("%s %d", __func__, pv->i2c_bc_rq.error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->i2c_bc_rq.error) {
    fl3194_error(pv, pv->i2c_bc_rq.error);
    fl3194_clean(dev);
    device_async_init_done(dev, pv->i2c_bc_rq.error);
    return;
  }

  pv->state = FL3194_IDLE;
  device_async_init_done(dev, 0);

  dev_i2c_ctrl_rq_init(&pv->i2c_bc_rq.base, &fl3194_update_done);
  fl3194_run(dev);
}

static DEV_VALIO_REQUEST(fl3194_request)
{
  struct device_s *dev = accessor->dev;
  struct fl3194_s *pv = dev->drv_pv;
  bool_t was_empty = 0;

  logk_trace("%s", __func__);

  if (rq->attribute != VALIO_LED) {
    rq->error = -EINVAL;
    goto done;
  }

  switch (rq->type) {
  case DEVICE_VALIO_WRITE: {
    rq->error = 0;
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);
    was_empty = dev_rq_queue_isempty(&pv->queue);
    dev_valio_rq_pushback(&pv->queue, rq);
    if (was_empty)
      fl3194_run(dev);
    return;
  }

  default:
    rq->error = -ENOTSUP;
    goto done;
  }

 done:
  dev_valio_rq_done(rq);
}

static DEV_VALIO_CANCEL(fl3194_cancel)
{
  struct device_s *dev = accessor->dev;
  struct fl3194_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item,
              if (item == &rq->base) {
                err = 0;
                GCT_FOREACH_BREAK;
              });

  if (err == 0)
    dev_valio_rq_remove(&pv->queue, rq);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define fl3194_use dev_use_generic

static DEV_INIT(fl3194_init)
{
  struct fl3194_s *pv;
  dev_timer_delay_t delay;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev_rq_queue_init(&pv->queue);

  pv->state = FL3194_INITING;
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_bc_rq, &is31fl3194_i2c_bytecode,
                                  &pv->i2c, NULL, &pv->timer);
  if (err)
    goto free_pv;

  dev->drv_pv = pv;

  pv->i2c_bc_rq.pvdata = dev;
  dev_i2c_ctrl_rq_init(&pv->i2c_bc_rq.base, &fl3194_init_done);

  logk_debug("Starting init");

  dev_timer_init_sec(pv->timer, &delay, 0, 100, 1000);
  if (!delay)
    delay = 1;

  bc_set_reg(&pv->i2c_bc_rq.vm, IS31FL3194_I2C_BCGLOBAL_RESET_TIME, delay);

  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_bc_rq, &fl3194_bc_initialize, 0);

  return -EAGAIN;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(fl3194_cleanup)
{
  struct fl3194_s *pv = dev->drv_pv;

  if (pv->state != FL3194_IDLE &&
      pv->state != FL3194_ERROR)
    return -EBUSY;

  fl3194_clean(dev);

  return 0;
}

DRIVER_DECLARE(is31fl3194_drv, 0, "IS31FL3194 RGB LED", fl3194,
               DRIVER_VALIO_METHODS(fl3194));

DRIVER_REGISTER(is31fl3194_drv);

