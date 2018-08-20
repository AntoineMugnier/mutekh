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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#define LOGK_MODULE_ID "mpu6"

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

#include <device/class/icu.h>
#include <device/class/valio.h>
#include <device/valio/motion_sensor.h>

#include "mpu650x_regs.h"
#include "mpu650x.h"

//#define dprintk printk
#ifndef dprintk
# define dprintk(x...) do{}while(0)
#endif

#if defined(CONFIG_DRIVER_MPU650X_SPI)
# include "mpu650x_spi.o.h"
# define bus_bytecode_start dev_spi_bytecode_start
# define bus_bytecode_cleanup dev_drv_spi_bytecode_cleanup
#elif defined(CONFIG_DRIVER_MPU650X_I2C)
# include "mpu650x_i2c.o.h"
# define bus_bytecode_start dev_i2c_bytecode_start
# define bus_bytecode_cleanup dev_drv_i2c_bytecode_cleanup
#endif

DRIVER_PV(struct mpu650x_context_s);

static
bool_t bus_is_busy(struct device_s *dev)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

  return pv->bus_rq.pvdata != NULL;
}

static
void mpu650x_run(struct device_s *dev, bytecode_entry_t *entry)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

  assert(!bus_is_busy(dev));

  pv->bus_rq.pvdata = dev;
  bus_bytecode_start(&pv->bus, &pv->bus_rq, entry, 0);
}

static
void mpu650x_config_set(struct device_s *dev, const struct valio_ms_config_s *config)
{
  struct mpu650x_context_s *pv = dev->drv_pv;
  uintptr_t wom_th, smplrt_div, lp_rate;

  if (config->threshold < 4)
    wom_th = 1;
  else if (config->threshold > 1000)
    wom_th = 255;
  else
    wom_th = config->threshold / 4;

  if (config->period == 0)
    smplrt_div = 0;
  else if (config->period > 256)
    smplrt_div = 255;
  else
    smplrt_div = config->period - 1;

  uint32_t tmp = config->wakeup_time ? config->wakeup_time : 1;
  if (is_pow2(tmp))
    tmp >>= 1;
  if (!tmp)
    tmp = 1;

  uint8_t high_bit = bit_msb_index(tmp);
  lp_rate = (high_bit > 11) ? 11 : 12 - high_bit;

  pv->stable_count = config->sleep_time / (smplrt_div + 1) + 1;

  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_WOM_THRESH, wom_th);
  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_SMPLRT_DIV, smplrt_div);
  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_LP_RATE, lp_rate);
}

static
void mpu650x_config_get(struct device_s *dev, struct valio_ms_config_s *config)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

  config->period = bc_get_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_SMPLRT_DIV) + 1;
  config->threshold = bc_get_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_WOM_THRESH) * 4;
  config->sleep_time = config->period * pv->stable_count;
  config->wakeup_time = 1 << (12 - bc_get_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_LP_RATE));
}

static
void mpu650x_state_advance(struct device_s *dev)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

 again:
  if (pv->bus_rq.pvdata)
    return;

  if (pv->target_state == pv->state) {
    if (pv->state >= MPU650X_POWER_OFF && pv->irq_pending) {
      pv->irq_pending = 0;

      //      dprintk("mpu650x_irq_handle\n");
      mpu650x_run(dev, &mpu650x_irq_handle);
    }

    return;
  }

  dprintk("%s %d -> %d%s\n", __FUNCTION__, pv->state, pv->target_state, pv->irq_pending ? " irq" : "");

  if (pv->target_state < pv->state) {
    switch (pv->target_state) {
    case MPU650X_GATE_OFF:
      pv->state = MPU650X_GATE_OFF;
      if (pv->bus_started) {
        pv->bus_started = 0;
        device_stop(&pv->bus.base);
      }
#ifdef CONFIG_DRIVER_MPU650X_POWERGATE
      if (dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_NONE) == -EAGAIN)
        return;
      goto again;
#endif

    case MPU650X_GATE_ON:
    case MPU650X_POWER_OFF:
      if (pv->bus_started) {
        dprintk("mpu650x_poweroff\n");
        mpu650x_run(dev, &mpu650x_poweroff);
      }
      pv->state = pv->target_state;
      return;

    default:
      pv->state = pv->target_state - 1;
      goto again;
    }
  } else {
    switch (pv->state) {
    case MPU650X_GATE_OFF:
#ifdef CONFIG_DRIVER_MPU650X_POWERGATE
      if (dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_POWER) == -EAGAIN)
        return;
#endif
      pv->state = MPU650X_GATE_ON;
      goto again;

    case MPU650X_GATE_ON:
      if (!pv->bus_started) {
        pv->bus_started = 1;
        device_start(&pv->bus.base);
      }

      if (pv->target_state == MPU650X_POWER_OFF) {
        dprintk("mpu650x_poweroff\n");
        mpu650x_run(dev, &mpu650x_poweroff);
        pv->state = MPU650X_POWER_OFF;
        return;
      }
      // Fallthrough
    case MPU650X_POWER_OFF:
      dprintk("mpu650x_poweron\n");
      mpu650x_run(dev, &mpu650x_poweron);
      pv->state = MPU650X_POWER_ON;
      return;

    case MPU650X_POWER_ON:
      dprintk("mpu650x_streaming_enable\n");
      mpu650x_run(dev, &mpu650x_streaming_enable);
      pv->stable_left = pv->stable_count;
      pv->state = MPU650X_STREAMING;
      return;

    case MPU650X_STREAMING:
      dprintk("mpu650x_wom_enable\n");
      mpu650x_run(dev, &mpu650x_wom_enable);
      pv->state = MPU650X_WOM;
      pv->irq_pending = 0;
      return;

    case MPU650X_WOM:
      assert(0);
    }
  }
}

static
void mpu650x_state_switch(struct device_s *dev, enum mpu650x_state_e state)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

  dprintk("%s %d -> %d (%d)\n", __FUNCTION__, pv->state, state, pv->target_state);

  pv->target_state = state;

  mpu650x_state_advance(dev);
}

static
void mpu650x_state_reset(struct device_s *dev, enum mpu650x_state_e state)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

  dprintk("%s %d <- %d (%d)\n", __FUNCTION__, pv->state, state, pv->target_state);

  pv->state = __MIN(pv->state, state);

  mpu650x_state_advance(dev);
}

static
void mpu650x_request_serve_queue(struct device_s *dev)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

  assert(dev);

  dprintk("%s fresh %d moved %d stable %d/%d just %d\n", __FUNCTION__,
          pv->has_fresh_data, pv->has_moved,
          pv->stable_left, pv->stable_count,
          pv->wom_just_changed);

  if (!pv->has_fresh_data)
    return;

  if (dev_rq_queue_isempty(&pv->queue)) {
    //    dprintk("%s queue empty\n", __FUNCTION__);
    device_sleep_schedule(dev);
  }

  if (pv->wom_just_changed) {
    pv->wom_just_changed--;
    pv->has_moved = 0;
    return;
  }

  GCT_FOREACH(dev_request_queue, &pv->queue, brq,
              struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(brq);
              struct valio_ms_state_s *s = rq->data;

              switch (pv->state) {
              case MPU650X_WOM:
                if (!pv->has_moved
                    && rq->type == DEVICE_VALIO_WAIT_EVENT)
                  GCT_FOREACH_CONTINUE;

              default:
                for (size_t i = 0; i < 6; ++i)
                  s->data.axis[i] = pv->value_last[i];
                s->active = pv->stable_left || pv->has_moved;
                rq->error = 0;
                dev_valio_rq_remove(&pv->queue, rq);
                dev_valio_rq_done(rq);
                break;
              }
              );

  if (pv->has_moved) {
    if (pv->state == MPU650X_WOM) {
      pv->wom_just_changed = 1;
      mpu650x_state_switch(dev, MPU650X_STREAMING);
    }
    pv->stable_left = pv->stable_count;
  } else if (pv->stable_left) {
    pv->stable_left--;
  } else if (pv->state == MPU650X_STREAMING) {
    pv->wom_just_changed = 1;
    mpu650x_state_switch(dev, MPU650X_WOM);
  }

  pv->has_moved = 0;
  pv->has_fresh_data = 0;
}

static DEV_IRQ_SRC_PROCESS(mpu650x_irq)
{
  struct device_s *dev = ep->base.dev;
  struct mpu650x_context_s *pv = dev->drv_pv;

  //dprintk("%s\n", __FUNCTION__);

  lock_spin(&dev->lock);

  if (pv->state >= MPU650X_POWER_OFF) {
    if (bus_is_busy(dev)) {
      pv->irq_pending = 1;
    } else {
      //      dprintk("mpu650x_irq_handle (direct)\n");
      mpu650x_run(dev, &mpu650x_irq_handle);
    }
  }

  lock_release(&dev->lock);
}

static KROUTINE_EXEC(mpu650x_bus_done)
{
  struct mpu650x_context_s *pv = KROUTINE_CONTAINER(kr, *pv, bus_rq.base.base.kr);
  struct device_s *dev = pv->bus_rq.pvdata;

  assert(dev);

  dprintk("%s %d %P\n", __FUNCTION__, pv->bus_rq.error, pv->value_last, 12);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->bus_rq.pvdata = NULL;

  if (!pv->bus_rq.error) {
    pv->error_count = 0;
  } else {
    pv->error_count++;

    if (pv->error_count == 3) {
      logk_error("Too many i2c errors, device reset");
      mpu650x_state_reset(dev, __MAX(pv->state - 1, 0));
    } else if (pv->error_count > 6) {
      logk_error("Too many i2c errors, device powered off");
      mpu650x_state_switch(dev, MPU650X_GATE_OFF);
    }
  }

  mpu650x_request_serve_queue(dev); 
  mpu650x_state_advance(dev);
}

static DEV_VALIO_REQUEST(mpu650x_request)
{
  struct device_s *dev = accessor->dev;
  struct mpu650x_context_s *pv = dev->drv_pv;

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->attribute) {
  case VALIO_MS_STATE: {
    struct valio_ms_state_s *s = rq->data;

    switch (rq->type) {
    case DEVICE_VALIO_READ:
      if (pv->state >= MPU650X_STREAMING) {
        for (size_t i = 0; i < 6; ++i)
          s->data.axis[i] = pv->value_last[i];
        s->active = !!pv->stable_left;
        rq->error = 0;
        break;
      }
      // fallthrough
    case DEVICE_VALIO_WAIT_EVENT:
      dev_valio_rq_pushback(&pv->queue, req);

      if (pv->state < MPU650X_STREAMING)
        mpu650x_state_switch(dev, MPU650X_STREAMING);
      return;

    default:
      rq->error = -ENOTSUP;
      break;
    }

    dev_valio_rq_done(req);
    break;
  }

  case VALIO_MS_CONFIG: {
    struct valio_ms_config_s *c = rq->data;

    switch (rq->type) {
    case DEVICE_VALIO_WRITE:
      dprintk("%s config write\n", __FUNCTION__);
      mpu650x_config_set(dev, c);
      mpu650x_state_reset(dev, MPU650X_POWER_OFF);
      rq->error = 0;
      break;

    case DEVICE_VALIO_READ:
      dprintk("%s config read\n", __FUNCTION__);
      mpu650x_config_get(dev, c);
      rq->error = 0;
      break;

    default:
      rq->error = -ENOTSUP;
      break;
    }

    dev_valio_rq_done(req);
    break;
  }

  case VALIO_MS_CALIB: {
    struct valio_ms_data_s *d = rq->data;

    switch (rq->type) {
    case DEVICE_VALIO_WRITE:
      dprintk("%s calib write\n", __FUNCTION__);
      for (size_t i = 0; i < 6; ++i)
        pv->offset[i] = d->axis[i];
      rq->error = 0;
      break;

    case DEVICE_VALIO_READ:
      dprintk("%s calib read\n", __FUNCTION__);
      for (size_t i = 0; i < 6; ++i)
        d->axis[i] = pv->offset[i];
      rq->error = 0;
      break;

    default:
      rq->error = -ENOTSUP;
      break;
    }

    dev_valio_rq_done(req);
    break;
  }

  default:
    break;
  }
}

static DEV_VALIO_CANCEL(mpu650x_cancel)
{
  struct device_s *dev = accessor->dev;
  struct mpu650x_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item,
              if (item == &rq->base) {
                dev_valio_rq_remove(&pv->queue, req);
                return 0;
              });

  return -ENOENT;
}

static DEV_USE(mpu650x_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct mpu650x_context_s *pv = dev->drv_pv;

    if (dev_rq_queue_isempty(&pv->queue))
      mpu650x_state_switch(dev, MPU650X_GATE_OFF);

    return 0;
  }

#ifdef CONFIG_DRIVER_MPU650X_POWERGATE
  case DEV_USE_CLOCK_SINK_GATE_DONE: {
    struct dev_clock_sink_ep_s *sink = param;
    struct device_s *dev = sink->dev;

    mpu650x_state_advance(dev);

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(mpu650x_init)
{
  struct mpu650x_context_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_rq_queue_init(&pv->queue);

#ifdef CONFIG_DRIVER_MPU650X_POWERGATE
  err = dev_drv_clock_init(dev, &pv->power_source, 0, 0, NULL);
  if (err)
    goto free_pv;
#endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &mpu650x_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;

#if defined(CONFIG_DRIVER_MPU650X_SPI)
  static const struct dev_spi_ctrl_config_s mpu650x_spi_config = {
    .dirty = 0,
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .cs_pol = DEV_SPI_ACTIVE_LOW,
    .bit_rate1k = 1000,
    .word_width = 8,
  };

  err = dev_drv_spi_bytecode_init(dev, &pv->bus_rq, &mpu650x_bus_bytecode,
                                  &mpu650x_spi_config, &pv->bus, NULL, &pv->timer);
#elif defined(CONFIG_DRIVER_MPU650X_I2C)
  err = dev_drv_i2c_bytecode_init(dev, &pv->bus_rq, &mpu650x_bus_bytecode,
                                  &pv->bus, NULL, &pv->timer);
#endif
  if (err)
    goto free_pv;

  dev_timer_delay_t ten_ms;
  dev_timer_init_sec(pv->timer, &ten_ms, 0, 1, 100);

  dev_i2c_ctrl_rq_init(&pv->bus_rq.base, &mpu650x_bus_done);
  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_PV, (uintptr_t)pv);

  pv->stable_count = 20;
  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_WOM_THRESH, 4);
  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_SMPLRT_DIV, 49);
  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_LP_RATE, 6);
  bc_set_reg(&pv->bus_rq.vm, MPU650X_BUS_BCGLOBAL_TEN_MS, ten_ms);

#if 0
  pv->state = MPU650X_GATE_OFF;
  pv->target_state = MPU650X_GATE_OFF;
  pv->stable_left = 0;
  pv->stable_left = 0;
#endif

#ifndef CONFIG_DRIVER_MPU650X_POWERGATE
  mpu650x_run(dev, &mpu650x_poweron);
  pv->state = MPU650X_POWER_ON;
#endif

  return 0;

 free_bus:
  bus_bytecode_cleanup(&pv->bus, &pv->bus_rq);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(mpu650x_cleanup)
{
  struct mpu650x_context_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue)
      || pv->state != MPU650X_POWER_OFF)
    return -EBUSY;

#ifdef CONFIG_DRIVER_MPU650X_POWERGATE
  dev_drv_clock_cleanup(dev, &pv->power_source);
#endif

  bus_bytecode_cleanup(&pv->bus, &pv->bus_rq);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_rq_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(mpu650x_drv, 0, "MPU650x", mpu650x,
               DRIVER_VALIO_METHODS(mpu650x));

DRIVER_REGISTER(mpu650x_drv);
