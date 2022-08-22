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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
*/

#define LOGK_MODULE_ID "adxl"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <device/class/valio.h>
#include <device/class/spi.h>

#include <device/valio/motion_sensor.h>

#include "adxl362.h"
#include "adxl362_regs.h"
#include "adxl362_app.o.h"

static void adxl362_write_config(struct adxl362_private_s *pv);
static void adxl362_read_conv(struct adxl362_private_s *pv);

static void
adxl362_error(struct device_s *dev, struct adxl362_private_s *pv, error_t error)
{
  logk_debug("%s %d", __func__, error);

  pv->state = ADXL362_STATE_ERROR;

  if (pv->irq_ep.base.dev)
    device_irq_src_disable(&pv->irq_ep);

  struct dev_valio_rq_s *rq;
  while ((rq = dev_valio_rq_pop(&pv->queue)))
    {
      rq->error = error;
      dev_valio_rq_done(rq);
    }
}

static void
adxl362_clean(struct device_s *dev, struct adxl362_private_s *pv)
{
  logk_trace("%s", __func__);

  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
  dev_rq_queue_destroy(&pv->queue);
  mem_free(pv);
}

/*----------------------------------------------------------------------------*/

static inline void
adxl362_next(struct adxl362_private_s *pv)
{
  logk_trace("%s state %d read pending %d conf pending %d",
             __func__, pv->state, pv->read_pending, pv->config_pending);

  if (pv->spi_rq.pvdata)
    return;

  if ((!dev_rq_queue_isempty(&pv->queue) && pv->state == ADXL362_STATE_IDLE)
      || pv->config_pending)
    adxl362_write_config(pv);
  else if (pv->read_pending)
    adxl362_read_conv(pv);
}

/*----------------------------------------------------------------------------*/

static
KROUTINE_EXEC(adxl362_read_conv_done)
{
  struct adxl362_private_s  *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s           *dev = pv->spi_rq.pvdata;
  int32_t data[3];

  LOCK_SPIN_IRQ(&dev->lock);

  logk_trace("%s", __func__);

  pv->spi_rq.pvdata = NULL;

  if (pv->state != ADXL362_STATE_RUNNING) {
    logk_trace("%s not running", __func__);
    pv->read_pending = 1;
    goto next;
  }

  if (pv->spi_rq.error)
    {
      logk_error("%s SPI error: %d", __func__, pv->spi_rq.error);
      adxl362_error(dev, pv, pv->spi_rq.error);
      goto end;
    }

  for (size_t i = 0; i < 3; ++i) {
    data[i] = bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_READ_CONV_BCOUT_XDATA + i);
    data[i] *= ADXL362_MG_PER_LSB;
    data[i] += pv->offset[i];
  }

  uint8_t status = bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_READ_CONV_BCOUT_STATUS);
  bool_t active = !!(status & ADXL362_REG_STATUS_AWAKE_MASK);

  if (dev_rq_queue_isempty(&pv->queue))
    device_sleep_schedule(dev);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
    struct valio_ms_state_s *state = (struct valio_ms_state_s *)rq->data;

    if (rq->type == DEVICE_VALIO_WAIT_EVENT && !active && !pv->was_active)
      GCT_FOREACH_CONTINUE;

    dev_valio_rq_remove(&pv->queue, rq);
    state->active = active;
    state->data.axis[VALIO_MS_ACCEL_X] = data[0];
    state->data.axis[VALIO_MS_ACCEL_Y] = data[1];
    state->data.axis[VALIO_MS_ACCEL_Z] = data[2];
    dev_valio_rq_done(rq);
  });

  pv->was_active = active;

next:
  adxl362_next(pv);

end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static
KROUTINE_EXEC(adxl362_idle_set_done)
{
  struct adxl362_private_s  *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s           *dev = pv->spi_rq.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  pv->spi_rq.pvdata = NULL;
  pv->state = ADXL362_STATE_IDLE;

  if (!dev_rq_queue_isempty(&pv->queue))
    adxl362_write_config(pv);
  
  LOCK_RELEASE_IRQ(&dev->lock);
}

static void
adxl362_read_conv(struct adxl362_private_s *pv)
{
  struct device_s *dev = pv->irq_ep.base.dev;

  logk_trace("%s", __func__);

  pv->spi_rq.pvdata = dev;
  pv->read_pending = 0;
  dev_spi_ctrl_rq_init(&pv->spi_rq.base, &adxl362_read_conv_done);
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &adxl362_bc_read_conv, 0);
}

static void
adxl362_shutdown(struct adxl362_private_s *pv)
{
  struct device_s *dev = pv->irq_ep.base.dev;

  logk_trace("%s", __func__);

  pv->spi_rq.pvdata = dev;
  dev_spi_ctrl_rq_init(&pv->spi_rq.base, &adxl362_idle_set_done);
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &adxl362_bc_set_idle, 0);
}

static
DEV_IRQ_SRC_PROCESS(adxl362_irq)
{
  struct device_s           *dev = ep->base.dev;
  struct adxl362_private_s  *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  logk_trace("%s", __func__);

  if (pv->state == ADXL362_STATE_ERROR)
    goto end;

  pv->read_pending = 1;
  adxl362_next(pv);

 end:
  lock_release(&dev->lock);
}

static
KROUTINE_EXEC(adxl362_config_done)
{
  struct adxl362_private_s  *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s           *dev = pv->spi_rq.pvdata;
  error_t                    err = pv->spi_rq.error;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s spi err %d", __func__, err);

  pv->spi_rq.pvdata = NULL;

  if (pv->state != ADXL362_STATE_ERROR)
    {
      pv->state = ADXL362_STATE_RUNNING;

      if (err)
        {
          adxl362_error(dev, pv, err);
          adxl362_clean(dev, pv);
        }
      else
        {
          adxl362_next(pv);
        }
    }

 end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static
KROUTINE_EXEC(adxl362_init_done)
{
  struct adxl362_private_s  *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s           *dev = pv->spi_rq.pvdata;
  error_t                    err = pv->spi_rq.error;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_debug("%s spi err %d", __func__, err);

  pv->spi_rq.pvdata = NULL;

  if (!err)
    {
      device_irq_source_init(dev, &pv->irq_ep, 1, &adxl362_irq);
      err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
    }
  
  device_async_init_done(dev, err);

  if (err)
    {
      adxl362_error(dev, pv, err);
      adxl362_clean(dev, pv);
    }
  else
    {
      pv->state = ADXL362_STATE_IDLE;
      adxl362_next(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static void
adxl362_write_config(struct adxl362_private_s *pv)
{
  struct device_s *dev = pv->irq_ep.base.dev;

  logk_trace("%s", __func__);

  // ODR calculation
  uint8_t odr = ADXL362_REG_FILTER_CTL_ODR_12_5_HZ;
  uint32_t period = 80;

  while (odr < ADXL362_REG_FILTER_CTL_ODR_400_HZ)
    {
      if (pv->config.period > period)
        break;
      odr++;
      period >>= 1;
    }

  // Registers
  uint8_t filter_ctl = 0
    | ADXL362_DEFAULT_RANGE
    | ADXL362_REG_FILTER_CTL_HALF_BW_MASK // Does this half the ODR ?
    | odr;

  uint8_t power_ctl = 0
    | ADXL362_REG_POWER_CTL_AUTOSLEEP_MASK
    | ADXL362_REG_POWER_CTL_MEASURE(MEASUREMENT);

  uint16_t threshold = __MIN(0x07ff, pv->config.threshold / ADXL362_MG_PER_LSB);
  uint8_t time_act = __MIN((pv->config.wakeup_time << odr) / 80, 0xff);
  uint16_t time_inact = __MIN((pv->config.sleep_time << odr) / 80, 0xffff);

  uint8_t act_inact_ctl = 0
    | ADXL362_REG_ACT_INACT_CTL_LINKLOOP(LINKED)
    | ADXL362_REG_ACT_INACT_CTL_ACT_REF_MASK
    | ADXL362_REG_ACT_INACT_CTL_INACT_REF_MASK
    | ADXL362_REG_ACT_INACT_CTL_ACT_EN_MASK
    | ADXL362_REG_ACT_INACT_CTL_INACT_EN_MASK;

  logk_debug("%s thresh: %d, act_inact: 0x%02x, power: 0x%02x, filter: 0x%02x, time_act: %d, time_inact: %d",
             __func__, threshold, act_inact_ctl, power_ctl,
             filter_ctl, time_act, time_inact);

  uint32_t config[4] = {
    byte_pack32(threshold & 0xff,
                time_act,
                threshold >> 8,
                threshold & 0xff
                ),
    byte_pack32(act_inact_ctl,
                time_inact >> 8,
                time_inact & 0xff,
                threshold >> 8
                ),
    byte_pack32(0,
                0
                | ADXL362_REG_INTMAP1_INACT_MASK
                | ADXL362_REG_INTMAP1_ACT_MASK
                | ADXL362_REG_INTMAP1_DATA_READY_MASK
                | ADXL362_REG_INTMAP1_INT_LOW_MASK,
                0x80,
                0x00
                ),
    byte_pack32(0,
                0,
                power_ctl,
                filter_ctl
                )
  };

  pv->spi_rq.pvdata = dev;
  pv->config_pending = 0;
  dev_spi_ctrl_rq_init(&pv->spi_rq.base, &adxl362_config_done);
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &adxl362_bc_config,
                         ADXL362_BC_CONFIG_BCARGS(config[0], config[1],
                                                  config[2], config[3]));
}

/*----------------------------------------------------------------------------*/

static
DEV_VALIO_REQUEST(adxl362_request)
{
  struct device_s           *dev = accessor->dev;
  struct adxl362_private_s  *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_trace("%s state %d", __func__, pv->state);

  rq->error = 0;

  if (pv->state == ADXL362_STATE_INIT)
    {
      rq->error = -EBUSY;
      goto end;
    }

  if (pv->state == ADXL362_STATE_ERROR)
    {
      rq->error = -EIO;
      goto end;
    }

  rq->error = -ENOTSUP;
  struct valio_ms_data_s    *data;
  struct valio_ms_config_s  *config;

  switch (rq->type)
    {
      case DEVICE_VALIO_READ:
        switch (rq->attribute) {
        case VALIO_MS_CALIB:
          rq->error = 0;
          data = (struct valio_ms_data_s *)rq->data;
          data->axis[VALIO_MS_ACCEL_X] = pv->offset[VALIO_MS_ACCEL_X];
          data->axis[VALIO_MS_ACCEL_Y] = pv->offset[VALIO_MS_ACCEL_Y];
          data->axis[VALIO_MS_ACCEL_Z] = pv->offset[VALIO_MS_ACCEL_Z];
          dev_valio_rq_done(req);
          break;

        case VALIO_MS_CONFIG:
          rq->error = 0;
          config = (struct valio_ms_config_s *)rq->data;
          *config = pv->config;
          dev_valio_rq_done(req);
          break;

        case VALIO_MS_STATE:
          rq->error = 0;
          dev_valio_rq_pushback(&pv->queue, req);
          pv->read_pending = 1;
          adxl362_next(pv);
          break;
        }
        break;

      case DEVICE_VALIO_WRITE:
        switch (rq->attribute) {
        case VALIO_MS_CALIB:
          rq->error = 0;
          data = (struct valio_ms_data_s *)rq->data;
          pv->offset[VALIO_MS_ACCEL_X] = data->axis[VALIO_MS_ACCEL_X];
          pv->offset[VALIO_MS_ACCEL_Y] = data->axis[VALIO_MS_ACCEL_Y];
          pv->offset[VALIO_MS_ACCEL_Z] = data->axis[VALIO_MS_ACCEL_Z];
          dev_valio_rq_done(req);
          break;

        case VALIO_MS_CONFIG:
          rq->error = 0;
          config = (struct valio_ms_config_s *)rq->data;
          pv->config = *config;
          pv->config_pending = 1;
          pv->read_pending = 1;
          adxl362_next(pv);
          dev_valio_rq_done(req);
          break;
        }
        break;

    case DEVICE_VALIO_WAIT_EVENT:
      switch (rq->attribute) {
      case VALIO_MS_STATE:
        rq->error = 0;
        dev_valio_rq_pushback(&pv->queue, req);
        adxl362_next(pv);
        break;
      }
      break;
    }

end:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (rq->error < 0)
    dev_valio_rq_done(req);
}

/*----------------------------------------------------------------------------*/

static
DEV_VALIO_CANCEL(adxl362_cancel)
{
  struct device_s           *dev = accessor->dev;
  struct adxl362_private_s  *pv = dev->drv_pv;

  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);

  logk_trace("%s", __func__);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
    if (rq == req)
      {
        dev_valio_rq_remove(&pv->queue, rq);
        err = 0;
        GCT_FOREACH_BREAK;
      }
  });

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/*----------------------------------------------------------------------------*/

static DEV_USE(adxl362_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct adxl362_private_s *pv = dev->drv_pv;

    if (dev_rq_queue_isempty(&pv->queue) && pv->state == ADXL362_STATE_RUNNING)
      adxl362_shutdown(pv);

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static
DEV_INIT(adxl362_init)
{
  struct adxl362_private_s *pv;
  error_t err;

  logk_trace("%s", __func__);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  dev_rq_queue_init(&pv->queue);

  pv->state = ADXL362_STATE_INIT;

  static const struct dev_spi_ctrl_config_s spi_config = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .bit_rate1k = 1000,
    .word_width = 8,
  };

  err = dev_drv_spi_bytecode_init(dev, &pv->spi_rq, &adxl362_app_bytecode,
                                  &spi_config, &pv->spi, &pv->gpio, &pv->timer);
  if (err)
    goto err_pv;

  static const gpio_width_t pin_wmap[] = {1};

  if (device_res_gpio_map(dev, "irq:1", pv->pin_map, NULL))
    goto err_pv;

  pv->spi_rq.gpio_map = pv->pin_map;
  pv->spi_rq.gpio_wmap = pin_wmap;

  pv->spi_rq.pvdata = NULL;

  pv->config.period = 20;
  pv->config.sleep_time = 500;
  pv->config.wakeup_time = 100;
  pv->config.threshold = 50;

  dev_timer_delay_t reset_latency;
  dev_timer_init_sec(pv->timer, &reset_latency, 0, 1, 2);

  pv->spi_rq.pvdata = dev;
  dev_spi_ctrl_rq_init(&pv->spi_rq.base, &adxl362_init_done);
  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &adxl362_bc_reset,
                         ADXL362_BC_RESET_BCARGS(reset_latency));

  return -EAGAIN;

 err_pv:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(adxl362_cleanup)
{
  logk_trace("%s", __func__);

  struct adxl362_private_s *pv = dev->drv_pv;

  if (pv->spi_rq.pvdata)
    return -EBUSY;

  switch (pv->state) {
  case ADXL362_STATE_IDLE:
  case ADXL362_STATE_ERROR:
    break;
  default:
    return -EBUSY;
  }

  adxl362_clean(dev, pv);

  return 0;
}

DRIVER_DECLARE(adxl362_drv, 0, "ADXL362", adxl362,
               DRIVER_VALIO_METHODS(adxl362));

DRIVER_REGISTER(adxl362_drv);

