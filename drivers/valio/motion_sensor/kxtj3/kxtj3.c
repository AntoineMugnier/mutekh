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

#define LOGK_MODULE_ID "kxtj"

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
#include <device/valio/motion_sensor.h>

#include "kxtj3.h"
#include "kxtj3_i2c.o.h"

enum kxtj3_state_e
{
  KXTJ3_INITING,
  KXTJ3_ERROR,
  KXTJ3_DISABLING,
  KXTJ3_DISABLED,
  KXTJ3_ENABLING,
  KXTJ3_ENABLED,
  KXTJ3_READING,
};

struct kxtj3_s
{
  struct device_i2c_ctrl_s i2c;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_bc_rq;
  struct dev_irq_src_s irq_ep;
  struct device_timer_s *timer;
  struct valio_ms_config_s config;

  struct kroutine_s runner;
  dev_request_queue_root_t queue;
  /* Data range, log2(max_val)-1 */
  uint8_t range_l2m1;
  /* Data resolution 8, 12, 14 */
  uint8_t resolution;
  bool_t config_dirty;
  bool_t i2c_busy;
  bool_t data_available;
  bool_t read_pending;

  /* Activity mode (whether we receive all data or just above threshold) */
  bool_t active;
  /* Reference values */
  int16_t reference_data[3];
  /* Last activity date */
  dev_timer_value_t last_motion;
  /* Time (in ticks) before no motion is translated to inactivity */
  dev_timer_delay_t motion_timeout;
  
  enum kxtj3_state_e state;
};

STRUCT_COMPOSE(kxtj3_s, i2c_bc_rq);
DRIVER_PV(struct kxtj3_s);

static void
kxtj3_error(struct kxtj3_s *pv, error_t error)
{
  pv->state = KXTJ3_ERROR;

  struct dev_valio_rq_s *rq;
  while ((rq = dev_valio_rq_pop(&pv->queue))) {
    rq->error = error;
    dev_valio_rq_done(rq);
  }
}

static void
kxtj3_clean(struct device_s *dev)
{
  struct kxtj3_s *pv = dev->drv_pv;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_bc_rq);
  dev_rq_queue_destroy(&pv->queue);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev->drv_pv = NULL;
  mem_free(pv);
}

static
KROUTINE_EXEC(kxtj3_run)
{
  struct kxtj3_s *pv = KROUTINE_CONTAINER(kr, *pv, runner);
  //  struct device_s *dev = pv->irq_ep.base.dev;
  struct dev_valio_rq_s *rq;

  rq = dev_valio_rq_head(&pv->queue);

  logk_debug("%s rq %p state %d%s%s%s",
             __func__, rq, pv->state,
             pv->config_dirty ? " config" : "",
             pv->data_available ? " data" : "",
             pv->i2c_busy ? " busy" : "");

  if (pv->i2c_busy)
    return;
  
  if ((rq && pv->state == KXTJ3_DISABLED) || pv->config_dirty) {
    uint8_t ctrl1, ctrl2, data_ctrl, wakeup_count, na_counter;
    uint16_t thresh;
    uint8_t owu_interval; /* in log2(interval / 10ms) */
    uint8_t interval; /* in log2(interval / 625us) */
    uint32_t owu_ms;

    interval = bit_msb_index(((pv->config.period * (uint32_t)(1024 / 0.625)) / 1024) | 1);
    interval = __MIN(interval, 11);
    owu_interval = __MIN(interval, 7);

    owu_ms = __MAX(1, 1280 >> (7 - interval));

    ctrl1 = 0
      // PC1 is added by BC
      | KXTJ3_CTRL1_RES
      | KXTJ3_CTRL1_RES_LUT(pv->range_l2m1)
      | ((pv->active || pv->read_pending) ? KXTJ3_CTRL1_DRDYE : 0)
      | KXTJ3_CTRL1_WUFE;

    ctrl2 = 0
      | KXTJ3_CTRL2_OWU_INTERVAL_LUT(owu_interval);

    data_ctrl = 0
      | KXTJ3_DATA_CTRL_OSA_INTERVAL_LUT(interval);

    wakeup_count = __MAX(1, pv->config.wakeup_time / owu_ms);
    na_counter = pv->active ? -1 : 1;

    thresh = (pv->config.threshold * (uint32_t)((1 << 12) * 1024 / 1000)) / 1024;
    thresh &= 0xfff0;
    
    logk_debug("Enable, period %d ms, thresh %d mg, sleep_time %d ms,"
               " wakeup_time %d ms, odr %d, owu %d",
               pv->config.period, pv->config.threshold,
               pv->config.sleep_time, pv->config.wakeup_time,
               interval, owu_interval);
    
    logk_debug("ctrl %02x %02x, data %02x, wu %02x, na %02x, th %04x",
               ctrl1, ctrl2, data_ctrl, wakeup_count, na_counter, thresh);
      
    pv->state = KXTJ3_ENABLING;
    pv->i2c_busy = 1;
    pv->config_dirty = 0;
    pv->data_available = 0;
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_bc_rq, &kxtj3_bc_configure, 0x3f,
                           ctrl1, ctrl2, data_ctrl, wakeup_count, na_counter, thresh);
    return;
  }

  if (pv->state == KXTJ3_ENABLED && pv->data_available) {
    if (rq) {
      pv->state = KXTJ3_READING;
      pv->i2c_busy = 1;
      pv->data_available = 0;

      logk_debug("Reading data");

      dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_bc_rq, &kxtj3_bc_read, 0);
      return;
    } else {
      pv->state = KXTJ3_DISABLING;
      pv->i2c_busy = 1;

      logk_debug("Disabling");

      dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_bc_rq, &kxtj3_bc_shutdown, 0);
      return;
    }
  }
}

static
KROUTINE_EXEC(kxtj3_i2c_done)
{
  struct dev_i2c_ctrl_rq_s *i2c_rq = dev_i2c_ctrl_rq_from_kr(kr);
  struct dev_i2c_ctrl_bytecode_rq_s *i2c_bc_rq = dev_i2c_ctrl_bytecode_rq_s_cast(i2c_rq);
  struct kxtj3_s *pv  = kxtj3_s_from_i2c_bc_rq(i2c_bc_rq);
  struct device_s *dev = pv->i2c_bc_rq.pvdata;

  logk_trace("%s err %d", __func__, pv->i2c_bc_rq.error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->i2c_busy = 0;

  logk_debug("I2c done, state %d, err %d",
             pv->state, pv->i2c_bc_rq.error);
  
  if (pv->i2c_bc_rq.error)
    kxtj3_error(pv, pv->i2c_bc_rq.error);

  switch (pv->state) {
  case KXTJ3_INITING:
    if (pv->i2c_bc_rq.error) {
      kxtj3_clean(dev);
      device_async_init_done(dev, pv->i2c_bc_rq.error);
      return;
    }

    pv->state = KXTJ3_DISABLED;
    device_async_init_done(dev, 0);

    kroutine_exec(&pv->runner);
    return;

  case KXTJ3_ERROR:
    return;

  case KXTJ3_DISABLED:
  case KXTJ3_ENABLED:
    kroutine_exec(&pv->runner);
    return;

  case KXTJ3_ENABLING:
    if (pv->i2c_bc_rq.error) {
      pv->state = KXTJ3_DISABLING;
      dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_bc_rq, &kxtj3_bc_shutdown, 0);
    } else {
      pv->state = KXTJ3_ENABLED;
      kroutine_exec(&pv->runner);
    }
    return;

  case KXTJ3_DISABLING:
    pv->state = KXTJ3_DISABLED;
    kroutine_exec(&pv->runner);
    return;

  case KXTJ3_READING: {
    pv->state = KXTJ3_ENABLED;
    
    const uint8_t sh = 11;
    bool_t active = 0;
    int16_t data[3];

    for (size_t i = 0; i < 3; ++i) {
      int32_t data32 = (int16_t)bc_get_reg(&pv->i2c_bc_rq.vm, KXTJ3_I2C_BCGLOBAL_X + i);
      /* Normalize to s5.14 fixed point (sign extended to 32b) */
      data32 <<= pv->range_l2m1;

      /* Scale to mg, only pay for 1 mul and 1 shr */
      data[i] = -(data32 * (int32_t)((1000.0 * (1 << sh)) / (1 << 14))) >> sh;

      active |= abs(data[i] - pv->reference_data[i]) > pv->config.threshold;
    }

    if (active)
      for (size_t i = 0; i < 3; ++i)
        pv->reference_data[i] = data[i];

    logk_debug("Read done %d %d %d active %d",
               data[0], data[1], data[2], active);

    GCT_FOREACH(dev_request_queue, &pv->queue, item, {
        struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
        struct valio_ms_state_s *state = (struct valio_ms_state_s *)rq->data;

        if (rq->type == DEVICE_VALIO_WAIT_EVENT && !active && !pv->active)
          GCT_FOREACH_CONTINUE;

        dev_valio_rq_remove(&pv->queue, rq);
        state->active = active;
        state->data.axis[VALIO_MS_ACCEL_X] = data[0];
        state->data.axis[VALIO_MS_ACCEL_Y] = data[1];
        state->data.axis[VALIO_MS_ACCEL_Z] = data[2];
        dev_valio_rq_done(rq);
      });

    dev_timer_value_t now;
    DEVICE_OP(pv->timer, get_value, &now, 0);

    if (active) {
      pv->last_motion = now;
    } else if (pv->last_motion + pv->motion_timeout > now) {
      // Keep active high until timeout
      active = 1;
    }

    if ((pv->active || pv->read_pending) != active)
      pv->config_dirty = 1;

    pv->active = active;
    pv->read_pending = 0;
    kroutine_exec(&pv->runner);
    return;
  }
  }
}

static DEV_VALIO_REQUEST(kxtj3_request)
{
  struct device_s *dev = accessor->dev;
  struct kxtj3_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->attribute) {
  case VALIO_MS_STATE:
    switch (rq->type) {
    case DEVICE_VALIO_READ:
      if (pv->state >= KXTJ3_ENABLED) {
        struct valio_ms_state_s *state = rq->data;
        state->data.axis[VALIO_MS_ACCEL_X] = pv->reference_data[0];
        state->data.axis[VALIO_MS_ACCEL_Y] = pv->reference_data[1];
        state->data.axis[VALIO_MS_ACCEL_Z] = pv->reference_data[2];
        state->active = pv->active;
        goto done;
      }
      // fallthrough
      pv->read_pending = 1;
    case DEVICE_VALIO_WAIT_EVENT:
      goto accept;
    default:
      goto reject;
    }

  case VALIO_MS_CONFIG:
    switch (rq->type) {
    case DEVICE_VALIO_WRITE:
      pv->config = *(const struct valio_ms_config_s *)rq->data;
      pv->config_dirty = 1;

      dev_timer_init_sec(pv->timer, &pv->motion_timeout, 0,
                         pv->config.sleep_time, 1000);

      kroutine_exec(&pv->runner);
      goto done;

    case DEVICE_VALIO_READ:
      *(struct valio_ms_config_s *)rq->data = pv->config;
      goto done;

    default:
      goto reject;
    }

  default:
    goto reject;
  }

 reject:
  rq->error = -ENOTSUP;
  dev_valio_rq_done(rq);
  return;

 done:
  rq->error = 0;
  dev_valio_rq_done(rq);
  return;

 accept:
  dev_valio_rq_pushback(&pv->queue, rq);
  kroutine_exec(&pv->runner);
}

static DEV_VALIO_CANCEL(kxtj3_cancel)
{
  struct device_s *dev = accessor->dev;
  struct kxtj3_s *pv = dev->drv_pv;
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

static DEV_IRQ_SRC_PROCESS(kxtj3_irq)
{
  struct device_s *dev = ep->base.dev;
  struct kxtj3_s *pv = dev->drv_pv;

  logk_trace("%s", __func__);

  lock_spin(&dev->lock);

  pv->data_available = 1;
  kroutine_exec(&pv->runner);

  lock_release(&dev->lock);
}

#define kxtj3_use dev_use_generic

static DEV_INIT(kxtj3_init)
{
  struct kxtj3_s *pv;
  dev_timer_delay_t delay;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev_rq_queue_init(&pv->queue);

  pv->state = KXTJ3_INITING;
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_bc_rq, &kxtj3_i2c_bytecode,
                                  &pv->i2c, NULL, &pv->timer);
  if (err)
    goto free_pv;

  device_irq_source_init(dev, &pv->irq_ep, 1, &kxtj3_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_bc;

  dev->drv_pv = pv;

  kroutine_init_deferred(&pv->runner, kxtj3_run);

  pv->config.period = 50;
  pv->config.wakeup_time = 10;
  pv->config.sleep_time = 1000;
  pv->config.threshold = 25;
  
  pv->i2c_bc_rq.pvdata = dev;
  dev_i2c_ctrl_rq_init(&pv->i2c_bc_rq.base, &kxtj3_i2c_done);

  logk_debug("Starting init");

  dev_timer_init_sec(pv->timer, &delay, 0, 10, 1000);
  if (!delay)
    delay = 1;

  pv->i2c_busy = 1;
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_bc_rq, &kxtj3_bc_initialize,
                         1, delay);

  /* +/- 8g, 12 bits */
  pv->range_l2m1 = 2;
  pv->resolution = 12;
  
  return -EAGAIN;

 free_bc:
  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_bc_rq);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(kxtj3_cleanup)
{
  struct kxtj3_s *pv = dev->drv_pv;

  if ((pv->state != KXTJ3_DISABLED &&
       pv->state != KXTJ3_ERROR) || pv->i2c_busy)
    return -EBUSY;

  kxtj3_clean(dev);

  return 0;
}

DRIVER_DECLARE(kxtj3_drv, 0, "KXTJ3-1057 Accelerometer", kxtj3,
               DRIVER_VALIO_METHODS(kxtj3));

DRIVER_REGISTER(kxtj3_drv)
;
