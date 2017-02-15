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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2017
*/

#define LOGK_MODULE_ID "ic62"

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

#include "icm20602_regs.h"
#include "icm20602.h"

#if defined(CONFIG_DRIVER_ICM20602_SPI)
# include "icm20602_spi.o.h"
# define bus_bytecode_start dev_spi_bytecode_start
# define bus_bytecode_cleanup dev_drv_spi_bytecode_cleanup
#elif defined(CONFIG_DRIVER_ICM20602_I2C)
# include "icm20602_i2c.o.h"
# define bus_bytecode_start dev_i2c_bytecode_start
# define bus_bytecode_cleanup dev_drv_i2c_bytecode_cleanup
#endif

DRIVER_PV(struct icm20602_context_s);

static
bool_t bus_is_busy(struct device_s *dev)
{
  struct icm20602_context_s *pv = dev->drv_pv;

  return pv->bus_rq.base.base.pvdata != NULL;
}

static
void icm20602_run(struct device_s *dev, bytecode_entry_t *entry)
{
  struct icm20602_context_s *pv = dev->drv_pv;

  assert(!bus_is_busy(dev));

  pv->bus_rq.base.base.pvdata = dev;
  bus_bytecode_start(&pv->bus, &pv->bus_rq, entry, 0);
}

static
void icm20602_config_set(struct device_s *dev, const struct valio_ms_config_s *config)
{
  struct icm20602_context_s *pv = dev->drv_pv;
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

  uint8_t high_bit = 31 - __builtin_clzl(tmp);
  lp_rate = (high_bit > 11) ? 11 : 12 - high_bit;

  pv->stable_count = config->sleep_time / (smplrt_div + 1) + 1;

  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_WOM_THRESH, wom_th);
  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_SMPLRT_DIV, smplrt_div);
  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_LP_RATE, lp_rate);
}

static
void icm20602_config_get(struct device_s *dev, struct valio_ms_config_s *config)
{
  struct icm20602_context_s *pv = dev->drv_pv;

  config->period = bc_get_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_SMPLRT_DIV) + 1;
  config->threshold = bc_get_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_WOM_THRESH) * 4;
  config->sleep_time = config->period * pv->stable_count;
  config->wakeup_time = 1 << (12 - bc_get_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_LP_RATE));
}

static
void icm20602_state_advance(struct device_s *dev)
{
  struct icm20602_context_s *pv = dev->drv_pv;

 again:
  if (pv->bus_rq.base.base.pvdata)
    return;

  if (pv->target_state == pv->state) {
    if (pv->state >= ICM20602_POWER_OFF && pv->irq_pending) {
      pv->irq_pending = 0;

      logk_debug("icm20602_irq_handle");
      icm20602_run(dev, &icm20602_irq_handle);
    }

    return;
  }

  logk_trace("%s %d -> %d%s", __FUNCTION__,
             pv->state, pv->target_state, pv->irq_pending ? " irq" : "");

  if (pv->target_state < pv->state) {
    switch (pv->target_state) {
    case ICM20602_GATE_OFF:
      pv->state = ICM20602_GATE_OFF;
#ifdef CONFIG_DRIVER_ICM20602_POWERGATE
      if (dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_NONE) == -EAGAIN)
        return;
      goto again;
#endif

    case ICM20602_GATE_ON:
    case ICM20602_POWER_OFF:
      logk_debug("icm20602_poweroff");
      icm20602_run(dev, &icm20602_poweroff);
      pv->state = pv->target_state;
      return;

    default:
      pv->state = pv->target_state - 1;
      goto again;
    }
  } else {
    switch (pv->state) {
    case ICM20602_GATE_OFF:
#ifdef CONFIG_DRIVER_ICM20602_POWERGATE
      if (dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_POWER) == -EAGAIN)
        return;
#endif
      pv->state = ICM20602_GATE_ON;
      goto again;

    case ICM20602_GATE_ON:
      if (pv->target_state == ICM20602_POWER_OFF) {
        logk_debug("icm20602_poweroff");
        icm20602_run(dev, &icm20602_poweroff);
        pv->state = ICM20602_POWER_OFF;
        return;
      }
      // Fallthrough
    case ICM20602_POWER_OFF:
      logk_debug("icm20602_poweron");
      icm20602_run(dev, &icm20602_poweron);
      pv->state = ICM20602_POWER_ON;
      return;

    case ICM20602_POWER_ON:
      logk_debug("icm20602_streaming_enable");
      icm20602_run(dev, &icm20602_streaming_enable);
      pv->stable_left = pv->stable_count;
      pv->state = ICM20602_STREAMING;
      return;

    case ICM20602_STREAMING:
      logk_debug("icm20602_wom_enable");
      icm20602_run(dev, &icm20602_wom_enable);
      pv->state = ICM20602_WOM;
      pv->irq_pending = 0;
      return;

    case ICM20602_WOM:
      assert(0);
    }
  }
}

static
void icm20602_state_switch(struct device_s *dev, enum icm20602_state_e state)
{
  struct icm20602_context_s *pv = dev->drv_pv;

  logk_debug("%s %d -> %d (%d)", __FUNCTION__, pv->state, state, pv->target_state);

  pv->target_state = state;

  icm20602_state_advance(dev);
}

static
void icm20602_state_reset(struct device_s *dev, enum icm20602_state_e state)
{
  struct icm20602_context_s *pv = dev->drv_pv;

  logk_debug("%s %d <- %d (%d)", __FUNCTION__, pv->state, state, pv->target_state);

  pv->state = __MIN(pv->state, state);

  icm20602_state_advance(dev);
}

static
void icm20602_request_serve_queue(struct device_s *dev)
{
  struct icm20602_context_s *pv = dev->drv_pv;

  assert(dev);

  logk_debug("%s fresh %d moved %d stable %d/%d just %d", __FUNCTION__,
          pv->has_fresh_data, pv->has_moved,
          pv->stable_left, pv->stable_count,
          pv->wom_just_changed);

  if (dev_request_queue_isempty(&pv->queue)) {
    //    logk_debug("%s queue empty", __FUNCTION__);
    device_sleep_schedule(dev);
  }

  if (!pv->has_fresh_data)
    return;

  if (pv->wom_just_changed) {
    pv->wom_just_changed--;
    pv->has_moved = 0;
    return;
  }

  GCT_FOREACH(dev_request_queue, &pv->queue, brq,
              struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(brq);
              struct valio_ms_state_s *s = rq->data;

              switch (pv->state) {
              case ICM20602_WOM:
                if (!pv->has_moved
                    && rq->type == DEVICE_VALIO_WAIT_EVENT)
                  GCT_FOREACH_CONTINUE;

              default:
                for (size_t i = 0; i < 6; ++i)
                  s->data.axis[i] = pv->value_last[i];
                s->active = pv->stable_left || pv->has_moved;
                rq->error = 0;
                dev_request_queue_remove(&pv->queue, &rq->base);
                kroutine_exec(&rq->base.kr);
                break;
              }
              );

  if (pv->has_moved) {
    if (pv->state == ICM20602_WOM) {
      pv->wom_just_changed = 1;
      icm20602_state_switch(dev, ICM20602_STREAMING);
    }
    pv->stable_left = pv->stable_count;
  } else if (pv->stable_left) {
    pv->stable_left--;
  } else if (pv->state == ICM20602_STREAMING) {
    pv->wom_just_changed = 1;
    icm20602_state_switch(dev, ICM20602_WOM);
  }

  pv->has_moved = 0;
  pv->has_fresh_data = 0;
}

static DEV_IRQ_SRC_PROCESS(icm20602_irq)
{
  struct device_s *dev = ep->base.dev;
  struct icm20602_context_s *pv = dev->drv_pv;

  //logk_debug("%s", __FUNCTION__);

  lock_spin(&dev->lock);

  if (pv->state >= ICM20602_POWER_OFF) {
    if (bus_is_busy(dev)) {
      pv->irq_pending = 1;
    } else {
      //      logk_debug("icm20602_irq_handle (direct)");
      icm20602_run(dev, &icm20602_irq_handle);
    }
  }

  lock_release(&dev->lock);
}

static KROUTINE_EXEC(icm20602_bus_done)
{
  struct icm20602_context_s *pv = KROUTINE_CONTAINER(kr, *pv, bus_rq.base.base.kr);
  struct device_s *dev = pv->bus_rq.base.base.pvdata;

  assert(dev);

  logk_debug("%s %d %P", __FUNCTION__, pv->bus_rq.base.err, pv->value_last, 12);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->bus_rq.base.base.pvdata = NULL;

  if (!pv->bus_rq.base.err) {
    pv->error_count = 0;
  } else {
    pv->error_count++;

    if (pv->error_count == 3) {
      logk_error("Too many i2c errors, device reset");
      icm20602_state_reset(dev, __MAX(pv->state - 1, 0));
    } else if (pv->error_count > 6) {
      logk_error("Too many i2c errors, device powered off");
      icm20602_state_switch(dev, ICM20602_GATE_OFF);
    }
  }

  icm20602_request_serve_queue(dev);
  icm20602_state_advance(dev);
}

static DEV_VALIO_REQUEST(icm20602_request)
{
  struct device_s *dev = accessor->dev;
  struct icm20602_context_s *pv = dev->drv_pv;

  logk_debug("%s", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (req->attribute) {
  case VALIO_MS_STATE: {
    struct valio_ms_state_s *s = req->data;

    switch (req->type) {
    case DEVICE_VALIO_READ:
      if (pv->state >= ICM20602_STREAMING) {
        for (size_t i = 0; i < 6; ++i)
          s->data.axis[i] = pv->value_last[i];
        s->active = !!pv->stable_left;
        req->error = 0;
        break;
      }
      // fallthrough
    case DEVICE_VALIO_WAIT_EVENT:
      dev_request_queue_pushback(&pv->queue, &req->base);

      if (pv->state < ICM20602_STREAMING)
        icm20602_state_switch(dev, ICM20602_STREAMING);
      return;

    default:
      req->error = -ENOTSUP;
      break;
    }

    kroutine_exec(&req->base.kr);
    break;
  }

  case VALIO_MS_CONFIG: {
    struct valio_ms_config_s *c = req->data;

    switch (req->type) {
    case DEVICE_VALIO_WRITE:
      logk_debug("%s config write", __FUNCTION__);
      icm20602_config_set(dev, c);
      icm20602_state_reset(dev, ICM20602_POWER_OFF);
      req->error = 0;
      break;

    case DEVICE_VALIO_READ:
      logk_debug("%s config read", __FUNCTION__);
      icm20602_config_get(dev, c);
      req->error = 0;
      break;

    default:
      req->error = -ENOTSUP;
      break;
    }

    kroutine_exec(&req->base.kr);
    break;
  }

  case VALIO_MS_CALIB: {
    struct valio_ms_data_s *d = req->data;

    switch (req->type) {
    case DEVICE_VALIO_WRITE:
      logk_debug("%s calib write", __FUNCTION__);
      for (size_t i = 0; i < 6; ++i)
        pv->offset[i] = d->axis[i];
      req->error = 0;
      break;

    case DEVICE_VALIO_READ:
      logk_debug("%s calib read", __FUNCTION__);
      for (size_t i = 0; i < 6; ++i)
        d->axis[i] = pv->offset[i];
      req->error = 0;
      break;

    default:
      req->error = -ENOTSUP;
      break;
    }

    kroutine_exec(&req->base.kr);
    break;
  }

  default:
    break;
  }
}

static DEV_VALIO_CANCEL(icm20602_cancel)
{
  struct device_s *dev = accessor->dev;
  struct icm20602_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item,
              if (item == &req->base) {
                dev_request_queue_remove(&pv->queue, &req->base);
                return 0;
              });

  return -ENOENT;
}

static DEV_USE(icm20602_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct icm20602_context_s *pv = dev->drv_pv;

    if (dev_request_queue_isempty(&pv->queue))
      icm20602_state_switch(dev, ICM20602_GATE_OFF);

    return 0;
  }

#ifdef CONFIG_DRIVER_ICM20602_POWERGATE
  case DEV_USE_CLOCK_SINK_GATE_DONE: {
    struct dev_clock_sink_ep_s *sink = param;
    struct device_s *dev = sink->dev;

    icm20602_state_advance(dev);

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(icm20602_init)
{
  struct icm20602_context_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_queue_init(&pv->queue);

#ifdef CONFIG_DRIVER_ICM20602_POWERGATE
  err = dev_drv_clock_init(dev, &pv->power_source, 0, 0, NULL);
  if (err)
    goto free_pv;
#endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &icm20602_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;

#if defined(CONFIG_DRIVER_ICM20602_SPI)
  static const struct dev_spi_ctrl_config_s icm20602_spi_config = {
    .dirty = 0,
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .cs_pol = DEV_SPI_ACTIVE_LOW,
    .bit_rate = 1000000,
    .word_width = 8,
  };

  err = dev_drv_spi_bytecode_init(dev, &pv->bus_rq, &icm20602_bus_bytecode,
                                  &icm20602_spi_config, &pv->bus, NULL, &pv->timer);
#elif defined(CONFIG_DRIVER_ICM20602_I2C)
  err = dev_drv_i2c_bytecode_init(dev, &pv->bus_rq, &icm20602_bus_bytecode,
                                  &pv->bus, NULL, &pv->timer);
#endif
  if (err)
    goto free_pv;

  dev_timer_delay_t ten_ms;
  dev_timer_init_sec(pv->timer, &ten_ms, 0, 1, 100);

  kroutine_init_deferred(&pv->bus_rq.base.base.kr, &icm20602_bus_done);
  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_PV, (uintptr_t)pv);

  pv->stable_count = 20;
  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_WOM_THRESH, 4);
  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_SMPLRT_DIV, 49);
  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_LP_RATE, 6);
  bc_set_reg(&pv->bus_rq.vm, ICM20602_BUS_BCGLOBAL_TEN_MS, ten_ms);

#if 0
  pv->state = ICM20602_GATE_OFF;
  pv->target_state = ICM20602_GATE_OFF;
  pv->stable_left = 0;
  pv->stable_left = 0;
#endif

#ifndef CONFIG_DRIVER_ICM20602_POWERGATE
  icm20602_run(dev, &icm20602_poweron);
  pv->state = ICM20602_POWER_ON;
#endif

  return 0;

 free_bus:
  bus_bytecode_cleanup(&pv->bus, &pv->bus_rq);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(icm20602_cleanup)
{
  struct icm20602_context_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue)
      || pv->state != ICM20602_POWER_OFF)
    return -EBUSY;

#ifdef CONFIG_DRIVER_ICM20602_POWERGATE
  dev_drv_clock_cleanup(dev, &pv->power_source);
#endif

  bus_bytecode_cleanup(&pv->bus, &pv->bus_rq);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(icm20602_drv, 0, "ICM20602", icm20602,
               DRIVER_VALIO_METHODS(icm20602));

DRIVER_REGISTER(icm20602_drv);
