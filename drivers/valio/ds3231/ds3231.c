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

    Copyright (c) 2016, Nicolas Pouillon, <nipo@ssji.net>
*/

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <device/driver.h>
#include <device/request.h>
#include <device/class/valio.h>
#include <device/valio/hwclock.h>
#include <device/class/i2c.h>

//#define dprintk printk
#ifndef dprintk
#define dprintk(x...) do {} while (0)
#endif

#define R_TIME_SECONDS    0x0
#define  R_TIME_SECONDS_MASK     0x7f
#define R_TIME_MINUTES       0x1
#define  R_TIME_MINUTES_MASK     0x7f
#define R_TIME_HOURS         0x2
#define  R_TIME_HOURS_MASK       0x3f
#define R_TIME_WEEKDAYS      0x3
#define  R_TIME_WEEKDAYS_MASK    0x07
#define R_TIME_DAYS          0x4
#define  R_TIME_DAYS_MASK        0x3f
#define R_TIME_CENTURY_MONTH 0x5
#define  R_TIME_CENTURY_MASK     0x80
#define  R_TIME_MONTH_MASK       0x0f
#define R_TIME_YEARS         0x6
#define  R_TIME_YEARS_MASK       0xff

#define T(x) (R_TIME_##x - R_TIME_SECONDS)

#define R_CONTROL           0xe
#define  R_CONTROL_EOSC      0x80
#define  R_CONTROL_BBSWQ     0x40
#define  R_CONTROL_CONV      0x20
#define  R_CONTROL_RS2       0x10
#define  R_CONTROL_RS1       0x08
#define  R_CONTROL_INTCN     0x04
#define  R_CONTROL_A2IE      0x02
#define  R_CONTROL_A1IE      0x01
#define R_CONTROL_STATUS    0xf
#define  R_CONTROL_STATUS_OSF      0x80
#define  R_CONTROL_STATUS_EN32KHZ  0x08
#define  R_CONTROL_STATUS_BSY      0x04
#define  R_CONTROL_STATUS_A2F      0x02
#define  R_CONTROL_STATUS_A1F      0x01

struct ds3231_priv_s
{
  struct dev_i2c_ctrl_transaction_rq_s i2c_txn;
  struct dev_i2c_ctrl_transaction_data_s i2c_transfer[2];
  struct device_i2c_ctrl_s i2c;
  uint8_t reg;
  uint8_t buffer[7];
  
  dev_request_queue_root_t queue;
};

DRIVER_PV(struct ds3231_priv_s);

static inline uint8_t bcd2dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0xf);
}

static inline uint8_t dec2bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

static inline uint16_t bcd2year(uint8_t bcd)
{
    return 2000 + bcd2dec(bcd);
}

static inline uint8_t year2bcd(uint16_t dec)
{
    return dec2bcd(dec % 100);
}

static inline uint8_t dow2wd(enum valio_hwclock_dow_e dow)
{
  return (dow - DEV_HWCLOCK_DOW_SUNDAY) + 1;
}

static inline enum valio_hwclock_dow_e wd2dow(uint8_t wd)
{
  return wd - 1 + DEV_HWCLOCK_DOW_SUNDAY;
}

static void ds3231_request_run(struct device_s *dev,
                                struct ds3231_priv_s *pv)
{
  struct dev_valio_rq_s *rq
    = dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (!rq)
    return;

  pv->reg = R_TIME_SECONDS;
  pv->i2c_transfer[1].size = R_TIME_YEARS - R_TIME_SECONDS + 1;

  if (rq->type == DEVICE_VALIO_READ) {
    pv->i2c_transfer[1].type = DEV_I2C_CTRL_TRANSACTION_READ;
    memset(pv->buffer, 0, sizeof(pv->buffer));
  } else {
    struct valio_hwclock_s *clk = rq->data;

    pv->buffer[T(SECONDS)] = dec2bcd(clk->sec);
    pv->buffer[T(MINUTES)] = dec2bcd(clk->min);
    pv->buffer[T(HOURS)] = dec2bcd(clk->hour);
    pv->buffer[T(DAYS)] = dec2bcd(clk->day);
    pv->buffer[T(WEEKDAYS)] = dow2wd(clk->dow);
    pv->buffer[T(CENTURY_MONTH)] = dec2bcd(clk->month)
      | (clk->year >= 2100 ? R_TIME_CENTURY_MASK : 0);
    pv->buffer[T(YEARS)] = year2bcd(clk->year);

    pv->i2c_transfer[1].type = DEV_I2C_CTRL_TRANSACTION_WRITE;
  }

  dev_i2c_transaction_start(&pv->i2c, &pv->i2c_txn);
}

static KROUTINE_EXEC(ds3231_done)
{
  struct ds3231_priv_s *pv
    = KROUTINE_CONTAINER(kr, *pv, i2c_txn.base.base.kr);
  struct device_s *dev = pv->i2c_txn.base.base.pvdata;
  struct dev_valio_rq_s *rq;

  LOCK_SPIN_IRQ(&dev->lock);
  rq = dev_valio_rq_s_cast(dev_request_queue_pop(&pv->queue));
  ds3231_request_run(dev, pv);
  LOCK_RELEASE_IRQ(&dev->lock);

  assert(rq);

  if (pv->i2c_txn.base.err) {
    rq->error = pv->i2c_txn.base.err;
  } else if (rq->type == DEVICE_VALIO_READ) {
    struct valio_hwclock_s *clk = rq->data;
    
    rq->error = 0;

    clk->sec = bcd2dec(pv->buffer[T(SECONDS)] & 0x7f);
    clk->min = bcd2dec(pv->buffer[T(MINUTES)] & 0x7f);
    clk->hour = bcd2dec(pv->buffer[T(HOURS)] & 0x3f);
    clk->day = bcd2dec(pv->buffer[T(DAYS)] & 0x3f);
    clk->dow = wd2dow(pv->buffer[T(WEEKDAYS)] & 0x07);
    clk->month = bcd2dec(pv->buffer[T(CENTURY_MONTH)] & 0x1f);
    clk->year = bcd2year(pv->buffer[T(YEARS)]);
  }

  kroutine_exec(&rq->base.kr);
}

static DEV_VALIO_REQUEST(ds3231_request)
{
  struct device_s *dev = accessor->dev;
  struct ds3231_priv_s *pv = dev->drv_pv;

  dprintk("%s\n", __FUNCTION__);

  if (req->attribute != VALIO_HWCLOCK_DATE
      || (req->type != DEVICE_VALIO_READ
          && req->type != DEVICE_VALIO_WRITE)) {
    req->error = -ENOTSUP;
    kroutine_exec(&req->base.kr);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);
  bool_t was_empty = dev_request_queue_isempty(&pv->queue);

  dev_request_queue_push(&pv->queue, &req->base);

  if (was_empty)
    ds3231_request_run(dev, pv);
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_VALIO_CANCEL(ds3231_cancel)
{
  struct device_s *dev = accessor->dev;
  struct ds3231_priv_s *pv = dev->drv_pv;
  error_t err = 0;
  
  LOCK_SPIN_IRQ(&dev->lock);
  if (req == dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue)))
    err = -EBUSY;
  else
    dev_request_queue_remove(&pv->queue, &req->base);
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}


#define ds3231_use dev_use_generic

static DEV_INIT(ds3231_init)
{
  struct ds3231_priv_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = dev_drv_i2c_transaction_init(dev, &pv->i2c_txn, &pv->i2c);
  if (err)
    goto err_pv;

  pv->i2c_txn.base.base.pvdata = dev;
  pv->i2c_txn.transfer = pv->i2c_transfer;
  pv->i2c_txn.transfer_count = 2;
  pv->i2c_transfer[0].data = &pv->reg;
  pv->i2c_transfer[0].size = 1;
  pv->i2c_transfer[0].type = DEV_I2C_CTRL_TRANSACTION_WRITE;
  pv->i2c_transfer[1].data = pv->buffer;

  kroutine_init_deferred(&pv->i2c_txn.base.base.kr, ds3231_done);
  
  dev_request_queue_init(&pv->queue);

  dev->drv_pv = pv;

  return 0;

 err_pv:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(ds3231_cleanup)
{
  struct ds3231_priv_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_drv_i2c_transaction_cleanup(&pv->i2c, &pv->i2c_txn);
  dev_request_queue_destroy(&pv->queue);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(ds3231_drv, 0, "DS3231 HW Clock", ds3231,
               DRIVER_VALIO_METHODS(ds3231));

DRIVER_REGISTER(ds3231_drv);
