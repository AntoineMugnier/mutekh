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

#define LOGK_MODULE_ID "mx44"

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
#include <device/valio/luminosity.h>
#include <device/class/i2c.h>

#include "max44009_regs.h"
#include "max44009_i2c.o.h"

enum max44009_state_e
{
  MAX44009_INITING,
  MAX44009_IDLE,
  MAX44009_READING,
  MAX44009_WAITING,
  MAX44009_SHUTTING_DOWN,
  MAX44009_WAIT_SETUP,
};

struct max44009_context_s
{
  struct device_i2c_ctrl_s i2c;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
  struct dev_irq_src_s irq_ep;
  
  enum max44009_state_e state;
  dev_request_queue_root_t queue;

  uint8_t irq_pending;
  uint8_t limits_dirty;
};

DRIVER_PV(struct max44009_context_s);

static KROUTINE_EXEC(max44009_read_done);
static KROUTINE_EXEC(max44009_wait_setup_done);
static KROUTINE_EXEC(max44009_shutdown_done);

static
void max44009_read(struct device_s *dev)
{
  struct max44009_context_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  switch (pv->state) {
  case MAX44009_IDLE:
  case MAX44009_WAITING:
    break;

  default:
    return;
  }

  pv->state = MAX44009_READING;
  pv->irq_pending = 0;

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, max44009_read_done);
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &max44009_bc_read,
                         MAX44009_BC_READ_BCARGS());
}

static
void max44009_wait_setup(struct device_s *dev)
{
  struct max44009_context_s *pv = dev->drv_pv;

  logk_debug("%s state %d %d < mlx < %d", __func__, pv->state,
             bc_get_reg(&pv->i2c_rq.vm, MAX44009_I2C_BCGLOBAL_IF_BELOW),
             bc_get_reg(&pv->i2c_rq.vm, MAX44009_I2C_BCGLOBAL_IF_ABOVE));

  switch (pv->state) {
  case MAX44009_IDLE:
  case MAX44009_WAITING:
    break;

  default:
    return;
  }

  pv->state = MAX44009_WAIT_SETUP;
  pv->limits_dirty = 0;

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, max44009_wait_setup_done);
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &max44009_bc_wait_setup,
                         MAX44009_BC_WAIT_SETUP_BCARGS());
}

static
void max44009_shutdown(struct device_s *dev)
{
  struct max44009_context_s *pv = dev->drv_pv;

  logk_debug("%s state %d", __func__, pv->state);

  switch (pv->state) {
  case MAX44009_IDLE:
  case MAX44009_WAITING:
    break;

  default:
    return;
  }

  pv->state = MAX44009_SHUTTING_DOWN;

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, max44009_shutdown_done);
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &max44009_bc_shutdown,
                         MAX44009_BC_SHUTDOWN_BCARGS());
}

static
DEV_VALIO_REQUEST(max44009_request)
{
  struct device_s *dev = accessor->dev;
  struct max44009_context_s *pv = dev->drv_pv;
  (void)pv;

  logk_debug("%s %p", __func__, req);

  if (pv->state == MAX44009_INITING) {
    rq->error = -EAGAIN;
    dev_valio_rq_done(req);
    return;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->attribute) {
  case VALIO_LUMINOSITY_VALUE:
    if (rq->type == DEVICE_VALIO_WRITE)
      goto notsup;

    rq->error = 0;
    dev_valio_rq_pushback(&pv->queue, req);

    if (rq->type == DEVICE_VALIO_READ)
      max44009_read(dev);
    return;

  case VALIO_LUMINOSITY_LIMITS:
    if (rq->type == DEVICE_VALIO_WRITE) {
      struct valio_luminosity_limits_s *l = rq->data;

      bc_set_reg(&pv->i2c_rq.vm, MAX44009_I2C_BCGLOBAL_IF_ABOVE, l->if_above);
      bc_set_reg(&pv->i2c_rq.vm, MAX44009_I2C_BCGLOBAL_IF_BELOW, l->if_below);
      rq->error = 0;
      dev_valio_rq_done(req);

      pv->limits_dirty = 1;
      if (!dev_rq_queue_isempty(&pv->queue))
        max44009_wait_setup(dev);

      return;
    }
    // Fallthrough

  default:
  notsup:
    rq->error = -ENOTSUP;
    dev_valio_rq_done(req);
    return;
  }
}

static
DEV_VALIO_CANCEL(max44009_cancel)
{
  struct device_s *dev = accessor->dev;
  struct max44009_context_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_debug("%s %p", __func__, req);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);

      if (rq != req)
        GCT_FOREACH_CONTINUE;

      dev_valio_rq_remove(&pv->queue, rq);
      err = 0;
      GCT_FOREACH_BREAK;
    });

  if (dev_rq_queue_isempty(&pv->queue))
    device_sleep_schedule(dev);

  return err;
}

static
KROUTINE_EXEC(max44009_read_done)
{
  struct max44009_context_s *pv = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->state = MAX44009_IDLE;

  uint32_t value = bc_get_reg(&pv->i2c_rq.vm, MAX44009_BC_READ_BCOUT_READING);
  uint32_t if_above = bc_get_reg(&pv->i2c_rq.vm, MAX44009_I2C_BCGLOBAL_IF_ABOVE);
  uint32_t if_below = bc_get_reg(&pv->i2c_rq.vm, MAX44009_I2C_BCGLOBAL_IF_BELOW);

  logk_debug("%s %dmlx", __func__, value);
  
  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      struct valio_luminosity_s *l = rq->data;

      if (rq->type == DEVICE_VALIO_WAIT_EVENT
          && value < if_above && value > if_below)
        GCT_FOREACH_CONTINUE;

      l->mlux = value;
      rq->error = 0;
      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });

  device_sleep_schedule(dev);
}

static
KROUTINE_EXEC(max44009_wait_setup_done)
{
  struct max44009_context_s *pv = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_debug("%s", __func__);

  pv->state = MAX44009_WAITING;

  if (dev_rq_queue_isempty(&pv->queue))
    max44009_shutdown(dev);
  else if (pv->limits_dirty)
    max44009_wait_setup(dev);
  else if (pv->irq_pending)
    max44009_read(dev);
}

static
KROUTINE_EXEC(max44009_shutdown_done)
{
  struct max44009_context_s *pv = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->state = MAX44009_IDLE;

  if (!dev_rq_queue_isempty(&pv->queue))
    max44009_read(dev);
}

static
DEV_IRQ_SRC_PROCESS(max44009_irq)
{
  struct device_s *dev = ep->base.dev;
  struct max44009_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  logk_debug("%s state %d", __func__, pv->state);

  pv->irq_pending = 1;

  max44009_read(dev);

  lock_release(&dev->lock);
}

static
KROUTINE_EXEC(max44009_init_done)
{
  struct max44009_context_s *pv = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_debug("%s state %d", __func__, pv->state);

  assert(pv->state == MAX44009_INITING);

  device_async_init_done(dev, pv->i2c_rq.error);

  if (pv->i2c_rq.error)
    return;

  pv->state = MAX44009_IDLE;

  max44009_read(dev);
}

static DEV_USE(max44009_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct max44009_context_s *pv = dev->drv_pv;

    if (dev_rq_queue_isempty(&pv->queue))
      max44009_shutdown(dev);
    else
      max44009_wait_setup(dev);

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(max44009_init)
{
  struct max44009_context_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_rq_queue_init(&pv->queue);

  device_irq_source_init(dev, &pv->irq_ep, 1, &max44009_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;
  
  //pv->state = MAX44009_INITING;
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &max44009_i2c_bytecode,
                                  &pv->i2c, NULL, NULL);
  if (err)
    goto free_irq;

  pv->i2c_rq.pvdata = dev;
  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, max44009_init_done);
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &max44009_bc_shutdown,
                         MAX44009_BC_SHUTDOWN_BCARGS());

  return -EAGAIN;

 free_irq:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(max44009_cleanup)
{
  struct max44009_context_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue)
      || pv->state != MAX44009_IDLE)
    return -EBUSY;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
  dev_rq_queue_destroy(&pv->queue);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(max44009_drv, 0, "Max44009", max44009,
               DRIVER_VALIO_METHODS(max44009));

DRIVER_REGISTER(max44009_drv);
