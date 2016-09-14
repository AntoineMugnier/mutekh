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
#include "adxl362_bytecode.o.h"

#if 0
  #define dprintk printk
#else
  #define dprintk(x...) do {} while(0)
#endif

static void adxl362_write_config(struct adxl362_private_s *pv);
static void adxl362_read_conv(struct adxl362_private_s *pv);

static void
adxl362_error(struct device_s *dev, struct adxl362_private_s *pv, error_t error)
{
  dprintk("%s\n", __FUNCTION__);

  pv->state = ADXL362_STATE_ERROR;

  device_irq_src_disable(&pv->irq_ep);

  struct dev_valio_rq_s *rq;
  while ((rq = dev_valio_rq_s_cast(dev_request_queue_pop(&pv->wait_event_queue))))
    {
      rq->error = error;
      kroutine_exec(&rq->base.kr);
    }

  while ((rq = dev_valio_rq_s_cast(dev_request_queue_pop(&pv->queue))))
    {
      rq->error = error;
      kroutine_exec(&rq->base.kr);
    }
}

static void
adxl362_clean(struct device_s *dev, struct adxl362_private_s *pv)
{
  dprintk("%s\n", __FUNCTION__);

  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);

  dev_request_queue_destroy(&pv->queue);
  dev_request_queue_destroy(&pv->wait_event_queue);
  mem_free(pv);
}

/*----------------------------------------------------------------------------*/

static inline void
adxl362_next(struct adxl362_private_s *pv)
{
  dprintk("%s\n", __FUNCTION__);

  if (pv->config_pending)
    adxl362_write_config(pv);
  else if (pv->read_pending)
    adxl362_read_conv(pv);
  else
    pv->state = ADXL362_STATE_STANDBY;
}

/*----------------------------------------------------------------------------*/

static
KROUTINE_EXEC(adxl362_read_conv_done)
{
  struct adxl362_private_s  *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s           *dev = pv->spi_rq.base.base.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  dprintk("%s\n", __FUNCTION__);

  pv->read_pending = 0;

  if (pv->spi_rq.base.err || pv->state == ADXL362_STATE_ERROR)
    {
      adxl362_error(dev, pv, -EHOSTUNREACH);
      goto end;
    }

  int16_t x = (int8_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_READ_CONV_BCOUT_XDATA);
  int16_t y = (int8_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_READ_CONV_BCOUT_YDATA);
  int16_t z = (int8_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_READ_CONV_BCOUT_ZDATA);

  x *=  ADXL362_MG_PER_LSB;
  y *=  ADXL362_MG_PER_LSB;
  z *=  ADXL362_MG_PER_LSB;

  x -=  pv->calibration[VALIO_MS_ACCEL_X];
  y -=  pv->calibration[VALIO_MS_ACCEL_Y];
  z -=  pv->calibration[VALIO_MS_ACCEL_Z];

  uint8_t status = (uint8_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_READ_CONV_BCOUT_STATUS);

  bool_t ms_status = VALIO_MS_STATUS_INACTIVE;
  if (status & ADXL362_REG_STATUS_AWAKE_MASK)
    ms_status = VALIO_MS_STATUS_ACTIVE;

  GCT_FOREACH(dev_request_queue, &pv->wait_event_queue, item, {

    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
    struct valio_ms_state_s *ms_state = (struct valio_ms_state_s *)rq->data;

    if (ms_state->status == ms_status)
      {
        dev_request_queue_remove(&pv->wait_event_queue, dev_valio_rq_s_base(rq));
        ms_state->data.axis[VALIO_MS_ACCEL_X] = x;
        ms_state->data.axis[VALIO_MS_ACCEL_Y] = y;
        ms_state->data.axis[VALIO_MS_ACCEL_Z] = z;
        kroutine_exec(&rq->base.kr);
      }
  });

  struct dev_valio_rq_s *rq;
  while ((rq = dev_valio_rq_s_cast(dev_request_queue_pop(&pv->queue))))
    {
      struct valio_ms_state_s *ms_state = rq->data;
      ms_state->status = ms_status;
      ms_state->data.axis[VALIO_MS_ACCEL_X] = x;
      ms_state->data.axis[VALIO_MS_ACCEL_Y] = y;
      ms_state->data.axis[VALIO_MS_ACCEL_Z] = z;
      kroutine_exec(&rq->base.kr);
    }

  adxl362_next(pv);

end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static void
adxl362_read_conv(struct adxl362_private_s *pv)
{
  dprintk("%s\n", __FUNCTION__);

  pv->state = ADXL362_STATE_READ_CONV;

  kroutine_init_deferred(&pv->spi_rq.base.base.kr, &adxl362_read_conv_done);
  ensure(!dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &adxl362_bc_read_conv,
                                 0));
}

/*----------------------------------------------------------------------------*/

static
DEV_IRQ_SRC_PROCESS(adxl362_irq)
{
  struct device_s           *dev = ep->base.dev;
  struct adxl362_private_s  *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  dprintk("%s\n", __FUNCTION__);

  if (pv->state == ADXL362_STATE_ERROR)
    {
      adxl362_error(dev, pv, -EHOSTUNREACH);
      goto end;
    }

  pv->read_pending = 1;
  if (pv->state == ADXL362_STATE_STANDBY)
    adxl362_read_conv(pv);

end:
  lock_release(&dev->lock);
}

/*----------------------------------------------------------------------------*/

static inline void
adxl362_save_config(struct adxl362_private_s *pv,
                    uint8_t power_ctl, uint8_t filter_ctl,
                    uint16_t thres_act, uint16_t thres_inact,
                    uint16_t time_act, uint16_t time_inact)
{
  dprintk("%s\n", __FUNCTION__);

  if (power_ctl & ADXL362_REG_POWER_CTL_WAKEUP_MASK)
    {
      pv->config.active_data_rate_hz = 6;
      pv->config.inactive_data_rate_hz = 6;
    }
  else
    {
      switch (filter_ctl & ADXL362_REG_FILTER_CTL_ODR_MASK)
        {
          case ADXL362_REG_FILTER_CTL_ODR(25_HZ):
            pv->config.active_data_rate_hz = 25;
            break;
          case ADXL362_REG_FILTER_CTL_ODR(50_HZ):
            pv->config.active_data_rate_hz = 50;
            break;
          case ADXL362_REG_FILTER_CTL_ODR(100_HZ):
            pv->config.active_data_rate_hz = 100;
            break;
          case ADXL362_REG_FILTER_CTL_ODR(200_HZ):
            pv->config.active_data_rate_hz = 200;
            break;
          case ADXL362_REG_FILTER_CTL_ODR(400_HZ):
            pv->config.active_data_rate_hz = 400;
            break;
          default:
            break;
        }

      if (power_ctl & ADXL362_REG_POWER_CTL_AUTOSLEEP_MASK)
        pv->config.inactive_data_rate_hz = 6;
      else
        pv->config.inactive_data_rate_hz = pv->config.active_data_rate_hz;
    }

    pv->config.active_threshold_mg = (thres_act * CONFIG_DRIVER_ADXL362_RANGE_MG) >> 11;
    pv->config.inactive_threshold_mg = (thres_inact * CONFIG_DRIVER_ADXL362_RANGE_MG) >> 11;

    pv->config.active_time_ms = (time_act * 1000) / pv->config.inactive_data_rate_hz;
    pv->config.inactive_time_ms = (time_inact * 1000) / pv->config.active_data_rate_hz;
}

static
KROUTINE_EXEC(adxl362_config_done)
{
  struct adxl362_private_s  *pv  = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s           *dev = pv->spi_rq.base.base.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  dprintk("%s\n", __FUNCTION__);

  pv->config_pending = 0;

  if (pv->spi_rq.base.err || pv->state == ADXL362_STATE_ERROR)
    {
      adxl362_error(dev, pv, -EHOSTUNREACH);
      adxl362_clean(dev, pv);
      if (pv->state == ADXL362_STATE_INIT)
        device_async_init_done(dev, -EIO);
      goto end;
    }

  uint32_t config[4] = {
    (uint32_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_CONFIG_BCOUT_CONFIG1_R),
    (uint32_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_CONFIG_BCOUT_CONFIG2_R),
    (uint32_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_CONFIG_BCOUT_CONFIG3_R),
    (uint32_t)bc_get_reg(&pv->spi_rq.vm, ADXL362_BC_CONFIG_BCOUT_CONFIG4_R)
  };

  uint8_t power_ctl = (config[3] >> 24) & 0xff;
  uint8_t filter_ctl = (config[3] >> 16) & 0xff;
  uint16_t thres_act = (config[0] >> 16) & 0x7f;
  uint16_t thres_inact = (config[1] >> 8) & 0x7f;
  uint16_t time_act = config[1] & 0xff;
  uint16_t time_inact = ((config[2] & 0xff) << 8) | ((config[1] >> 24) & 0xff);

  adxl362_save_config(pv, power_ctl, filter_ctl, thres_act, thres_inact, time_act, time_inact);

  if (pv->state == ADXL362_STATE_INIT)
    {
      device_irq_source_init(dev, &pv->irq_ep, 1, &adxl362_irq);
      if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
        {
          adxl362_error(dev, pv, -EHOSTUNREACH);
          adxl362_clean(dev, pv);
          device_async_init_done(dev, -EIO);
          goto end;
        }

      device_async_init_done(dev, 0);
    }

  adxl362_next(pv);

end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static void
adxl362_write_config(struct adxl362_private_s *pv)
{
  dprintk("%s\n", __FUNCTION__);

  if (pv->state != ADXL362_STATE_INIT)
    pv->state = ADXL362_STATE_CONFIG;

  uint8_t odr;
  uint8_t wakeup;
  switch (pv->new_config.active_data_rate_hz)
    {
      default:
        pv->new_config.active_data_rate_hz = 6;
      case 6:
        odr = ADXL362_REG_FILTER_CTL_ODR(100_HZ); /* default value, unused */
        wakeup = ADXL362_REG_POWER_CTL_WAKEUP_MASK;
        break;
      case 25:
        odr = ADXL362_REG_FILTER_CTL_ODR(25_HZ);
        wakeup = 0;
        break;
      case 50:
        odr = ADXL362_REG_FILTER_CTL_ODR(50_HZ);
        wakeup = 0;
        break;
      case 100:
        odr = ADXL362_REG_FILTER_CTL_ODR(100_HZ);
        wakeup = 0;
        break;
      case 200:
        odr = ADXL362_REG_FILTER_CTL_ODR(200_HZ);
        wakeup = 0;
        break;
      case 400:
        odr = ADXL362_REG_FILTER_CTL_ODR(400_HZ);
        wakeup = 0;
        break;
    }

  uint8_t autosleep;
  if (pv->new_config.inactive_data_rate_hz == 6)
    autosleep = ADXL362_REG_POWER_CTL_AUTOSLEEP_MASK;
  else if (pv->new_config.inactive_data_rate_hz == pv->new_config.active_data_rate_hz)
    autosleep = 0;
  else
    {
      autosleep = 0;
      pv->new_config.inactive_data_rate_hz = pv->new_config.active_data_rate_hz;
    }

  uint8_t filter_ctl = ADXL362_DEFAULT_RANGE |
                       ADXL362_REG_FILTER_CTL_HALF_BW_MASK |
                       odr;

  uint8_t power_ctl = autosleep |
                      ADXL362_REG_POWER_CTL_MEASURE(MEASUREMENT) |
                      wakeup;

  uint16_t thres_act;
  if (pv->new_config.active_threshold_mg < CONFIG_DRIVER_ADXL362_RANGE_MG)
    thres_act = (pv->new_config.active_threshold_mg << 11) / CONFIG_DRIVER_ADXL362_RANGE_MG;
  else
    thres_act = 2048 - 1;

  uint8_t thres_act_l = thres_act & 0xff;
  uint8_t thres_act_h = (thres_act >> 8) & 0x07;

  uint16_t thres_inact;
  if (pv->new_config.inactive_threshold_mg < CONFIG_DRIVER_ADXL362_RANGE_MG)
    thres_inact = (pv->new_config.inactive_threshold_mg << 11) / CONFIG_DRIVER_ADXL362_RANGE_MG;
  else
    thres_inact = 2048 - 1;

  uint8_t thres_inact_l = thres_inact & 0xff;
  uint8_t thres_inact_h = (thres_inact >> 8) & 0x07;

  uint32_t time;
  if (pv->new_config.inactive_data_rate_hz == 6)
    time = 0;
  else
    time = pv->new_config.active_time_ms * pv->new_config.inactive_data_rate_hz / 1000;
  if (time > 0xff)
    time = 0xff;
  uint8_t time_act = time & 0xff;

  time = pv->new_config.inactive_time_ms * pv->new_config.active_data_rate_hz / 1000;
  if (time > 0xffff)
    time = 0xffff;
  uint8_t time_inact_l = time & 0xff;
  uint8_t time_inact_h = (time >> 8) & 0xff;

  /* Set delay after at 500ms */
  dev_timer_delay_t reset_latency;
  dev_timer_init_sec(pv->timer, &reset_latency, NULL, 500, 1000);

  uint32_t config[4] = {
    byte_pack32(
      thres_act_h,
      thres_act_l,
      ADXL362_REG_THRESH_ACT_L,
      ADXL362_CMD_WRITE_REG
    ),
    byte_pack32(
      time_inact_l,
      thres_inact_h,
      thres_inact_l,
      time_act
    ),
    byte_pack32(
      ADXL362_DEFAULT_FIFO_SAMPLES,
      ADXL362_DEFAULT_FIFO_CONTROL,
      ADXL362_DEFAULT_ACT_INACT_CTL,
      time_inact_h
    ),
    byte_pack32(
      power_ctl,
      filter_ctl,
      ADXL362_DEFAULT_INTMAP2,
      ADXL362_DEFAULT_INTMAP1
    )
  };

  kroutine_init_deferred(&pv->spi_rq.base.base.kr, &adxl362_config_done);
  ensure(!dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &adxl362_bc_config,
    ADXL362_BC_CONFIG_BCARGS(config[0], config[1], config[2], config[3], reset_latency)));
}

/*----------------------------------------------------------------------------*/

static
DEV_VALIO_REQUEST(adxl362_request)
{
  struct device_s           *dev = accessor->dev;
  struct adxl362_private_s  *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  dprintk("%s\n", __FUNCTION__);

  req->error = 0;

  if (pv->state == ADXL362_STATE_INIT)
    {
      req->error = -EBUSY;
      goto end;
    }

  if (pv->state == ADXL362_STATE_ERROR)
    {
      req->error = -EHOSTUNREACH;
      goto end;
    }

  req->error = -EINVAL;
  struct valio_ms_data_s          *ms_data;
  struct valio_ms_accel_config_s  *ms_config;

  switch (req->type)
    {
      case DEVICE_VALIO_READ:
        if (req->attribute == VALIO_MS_CALIBRATE)
          {
            req->error = 0;
            ms_data = (struct valio_ms_data_s *)req->data;
            ms_data->axis[VALIO_MS_ACCEL_X] = pv->calibration[VALIO_MS_ACCEL_X];
            ms_data->axis[VALIO_MS_ACCEL_Y] = pv->calibration[VALIO_MS_ACCEL_Y];
            ms_data->axis[VALIO_MS_ACCEL_Z] = pv->calibration[VALIO_MS_ACCEL_Z];
            kroutine_exec(&req->base.kr);
          }
        else if (req->attribute == VALIO_MS_ACCEL_CONFIG)
          {
            if (pv->config_pending)
              req->error = -EBUSY;
            else
              {
                req->error = 0;
                ms_config = (struct valio_ms_accel_config_s *)req->data;
                memcpy(ms_config, &pv->config, sizeof(struct valio_ms_accel_config_s));
              }
            kroutine_exec(&req->base.kr);
          }
        else if (req->attribute == VALIO_MS_STATE)
          {
            req->error = 0;
            dev_request_queue_pushback(&pv->queue, &req->base);
            pv->read_pending = 1;
            if (pv->state == ADXL362_STATE_STANDBY)
              adxl362_read_conv(pv);
          }
        break;

      case DEVICE_VALIO_WRITE:
        if (req->attribute == VALIO_MS_CALIBRATE)
          {
            req->error = 0;
            ms_data = (struct valio_ms_data_s *)req->data;
            pv->calibration[VALIO_MS_ACCEL_X] = ms_data->axis[VALIO_MS_ACCEL_X];
            pv->calibration[VALIO_MS_ACCEL_Y] = ms_data->axis[VALIO_MS_ACCEL_Y];
            pv->calibration[VALIO_MS_ACCEL_Z] = ms_data->axis[VALIO_MS_ACCEL_Z];
            kroutine_exec(&req->base.kr);
          }
        else if (req->attribute == VALIO_MS_ACCEL_CONFIG)
          {
            if (pv->config_pending)
              req->error = -EBUSY;
            else
              {
                req->error = 0;
                ms_config = (struct valio_ms_accel_config_s *)req->data;
                memcpy(&pv->new_config, ms_config, sizeof(struct valio_ms_accel_config_s));
                pv->config_pending = 1;
                if (pv->state == ADXL362_STATE_STANDBY)
                  adxl362_write_config(pv);
              }
            kroutine_exec(&req->base.kr);
          }
        break;

      case DEVICE_VALIO_WAIT_EVENT:
        if (req->attribute == VALIO_MS_STATE)
          {
            req->error = 0;
            dev_request_queue_pushback(&pv->wait_event_queue, &req->base);
            pv->read_pending = 1;
            if (pv->state == ADXL362_STATE_STANDBY)
              adxl362_read_conv(pv);
          }
        break;
    }

end:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (req->error < 0)
    kroutine_exec(&req->base.kr);
}

/*----------------------------------------------------------------------------*/

static
DEV_VALIO_CANCEL(adxl362_cancel)
{
  struct device_s           *dev = accessor->dev;
  struct adxl362_private_s  *pv = dev->drv_pv;

  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);

  dprintk("%s\n", __FUNCTION__);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
    if (rq == req)
      {
        dev_request_queue_remove(&pv->queue, dev_valio_rq_s_base(rq));
        err = 0;
        goto end;
      }
  });

  GCT_FOREACH(dev_request_queue, &pv->wait_event_queue, item, {
    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
    if (rq == req)
      {
        dev_request_queue_remove(&pv->wait_event_queue, dev_valio_rq_s_base(rq));
        err = 0;
        goto end;
      }
  });

end:
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

/*----------------------------------------------------------------------------*/

#define adxl362_use dev_use_generic

static
DEV_INIT(adxl362_init)
{
  dprintk("%s\n", __FUNCTION__);

  struct adxl362_private_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  dev_request_queue_init(&pv->queue);
  dev_request_queue_init(&pv->wait_event_queue);

  pv->state = ADXL362_STATE_INIT;

  if (dev_drv_spi_bytecode_init(
      dev, &pv->spi_rq, &adxl362_bytecode, &pv->spi, &pv->gpio, &pv->timer))
    goto err_pv;

  static const struct dev_spi_ctrl_config_s spi_config = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .bit_rate = 1000000,
    .word_width = 8,
  };

  pv->spi_rq.base.config = (struct dev_spi_ctrl_config_s *)&spi_config;
  pv->spi_rq.base.base.pvdata = dev;

  pv->new_config.active_data_rate_hz = CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ;
  pv->new_config.inactive_data_rate_hz = CONFIG_DRIVER_ADXL362_INACT_DATA_RATE_HZ;
  pv->new_config.active_threshold_mg = CONFIG_DRIVER_ADXL362_ACT_THRESHOLD_MG;
  pv->new_config.inactive_threshold_mg = CONFIG_DRIVER_ADXL362_INACT_THRESHOLD_MG;
  pv->new_config.active_time_ms = CONFIG_DRIVER_ADXL362_ACT_TIME_MS;
  pv->new_config.inactive_time_ms = CONFIG_DRIVER_ADXL362_INACT_TIME_MS;

  adxl362_write_config(pv);

  return -EAGAIN;

  err_pv:
    mem_free(pv);
    return -1;
}

static
DEV_CLEANUP(adxl362_cleanup)
{
  dprintk("%s\n", __FUNCTION__);

  struct adxl362_private_s *pv = dev->drv_pv;

  if (pv->state != ADXL362_STATE_INIT &&
      pv->state != ADXL362_STATE_STANDBY &&
      pv->state != ADXL362_STATE_ERROR)
    return -EBUSY;

  adxl362_clean(dev, pv);

  return 0;
}

DRIVER_DECLARE(adxl362_drv, 0, "ADXL362", adxl362,
               DRIVER_VALIO_METHODS(adxl362));

DRIVER_REGISTER(adxl362_drv);

