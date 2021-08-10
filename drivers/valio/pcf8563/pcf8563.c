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

#define R_CONTROL_STATUS1    0x0
#define  R_CONTROL_STATUS1_TEST1    0x80
#define  R_CONTROL_STATUS1_STOP     0x20
#define  R_CONTROL_STATUS1_TESTC    0x08
#define R_CONTROL_STATUS2    0x1
#define  R_CONTROL_STATUS2_TITP     0x10
#define  R_CONTROL_STATUS2_AF       0x08
#define  R_CONTROL_STATUS2_TF       0x04
#define  R_CONTROL_STATUS2_AIE      0x02
#define  R_CONTROL_STATUS2_TIE      0x01
#define R_CONTROL_CLKOUT     0xd
#define  R_CONTROL_CLKOUT_ENABLE    0x80
#define  R_CONTROL_CLKOUT_32768HZ   0x00
#define  R_CONTROL_CLKOUT_1024HZ    0x01
#define  R_CONTROL_CLKOUT_32HZ      0x02
#define  R_CONTROL_CLKOUT_1HZ       0x03

#define R_TIME_VL_SECONDS    0x2
#define  R_TIME_SECONDS_MASK     0x7f
#define  R_TIME_VL_MASK          0x80
#define R_TIME_MINUTES       0x3
#define  R_TIME_MINUTES_MASK     0x7f
#define R_TIME_HOURS         0x4
#define  R_TIME_HOURS_MASK       0x3f
#define R_TIME_DAYS          0x5
#define  R_TIME_DAYS_MASK        0x3f
#define R_TIME_WEEKDAYS      0x6
#define  R_TIME_WEEKDAYS_MASK    0x07
#define R_TIME_CENTURY_MONTH 0x7
#define  R_TIME_CENTURY_MASK     0x80
#define  R_TIME_MONTH_MASK       0x0f
#define R_TIME_YEARS         0x8
#define  R_TIME_YEADS_MASK       0xff

#define T(x) (R_TIME_##x - R_TIME_VL_SECONDS)

#define R_ALARM_MINUTE       0x9
#define  R_ALARM_MINUTE_MASK     0x7f
#define  R_ALARM_MINUTE_ENABLE   0x80
#define R_ALARM_HOUR         0xa
#define  R_ALARM_HOUR_MASK       0x3f
#define  R_ALARM_HOUR_ENABLE     0x80
#define R_ALARM_DAY          0xb
#define  R_ALARM_DAY_MASK        0x3f
#define  R_ALARM_DAY_ENABLE      0x80
#define R_ALARM_WEEKDAY      0xc
#define  R_ALARM_WEEKDAY_MASK    0x07
#define  R_ALARM_WEEKDAY_ENABLE  0x80

#define R_TIMER_CONTROL      0xe
#define  R_TIMER_CONTROL_ENABLE      0x80
#define  R_TIMER_CONTROL_4096HZ      0x00
#define  R_TIMER_CONTROL_64HZ        0x01
#define  R_TIMER_CONTROL_1HZ         0x02
#define  R_TIMER_CONTROL_1_60HZ      0x03
#define R_TIMER              0xf

struct pcf8563_priv_s
{
  struct dev_i2c_ctrl_transaction_rq_s i2c_txn;
  struct dev_i2c_ctrl_transaction_data_s i2c_transfer[2];
  struct device_i2c_ctrl_s i2c;
  uint8_t reg;
  uint8_t buffer[7];
  
  dev_request_queue_root_t queue;
};

DRIVER_PV(struct pcf8563_priv_s);

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
  return (dow - DEV_HWCLOCK_DOW_SUNDAY);
}

static inline enum valio_hwclock_dow_e wd2dow(uint8_t wd)
{
  return wd + DEV_HWCLOCK_DOW_SUNDAY;
}

static void pcf8563_request_run(struct device_s *dev,
                                struct pcf8563_priv_s *pv)
{
  struct dev_valio_rq_s *rq
    = dev_valio_rq_head(&pv->queue);

  if (!rq)
    return;

  pv->reg = R_TIME_VL_SECONDS;
  pv->i2c_transfer[1].size = R_TIME_YEARS - R_TIME_VL_SECONDS + 1;

  if (rq->type == DEVICE_VALIO_READ) {
    pv->i2c_transfer[1].type = DEV_I2C_CTRL_TRANSACTION_READ;
    memset(pv->buffer, 0, sizeof(pv->buffer));
  } else {
    struct valio_hwclock_s *clk = rq->data;

    pv->buffer[T(VL_SECONDS)] = dec2bcd(clk->sec);
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

static KROUTINE_EXEC(pcf8563_done)
{
  struct pcf8563_priv_s *pv
    = KROUTINE_CONTAINER(kr, *pv, i2c_txn.base.base.kr);
  struct device_s *dev = pv->i2c_txn.pvdata;
  struct dev_valio_rq_s *rq;

  LOCK_SPIN_IRQ(&dev->lock);
  rq = dev_valio_rq_pop(&pv->queue);
  pcf8563_request_run(dev, pv);
  LOCK_RELEASE_IRQ(&dev->lock);

  assert(rq);

  if (pv->i2c_txn.error) {
    rq->error = pv->i2c_txn.error;
  } else if (rq->type == DEVICE_VALIO_READ) {
    struct valio_hwclock_s *clk = rq->data;
    
    if (pv->buffer[T(VL_SECONDS)] & R_TIME_VL_MASK) {
      rq->error = -EBADDATA;
    } else {
      rq->error = 0;

      clk->sec = bcd2dec(pv->buffer[T(VL_SECONDS)] & 0x7f);
      clk->min = bcd2dec(pv->buffer[T(MINUTES)] & 0x7f);
      clk->hour = bcd2dec(pv->buffer[T(HOURS)] & 0x3f);
      clk->day = bcd2dec(pv->buffer[T(DAYS)] & 0x3f);
      clk->dow = wd2dow(pv->buffer[T(WEEKDAYS)] & 0x07);
      clk->month = bcd2dec(pv->buffer[T(CENTURY_MONTH)] & 0x1f);
      clk->year = bcd2year(pv->buffer[T(YEARS)]);
    }
  }

  dev_valio_rq_done(rq);
}

static DEV_VALIO_REQUEST(pcf8563_request)
{
  struct device_s *dev = accessor->dev;
  struct pcf8563_priv_s *pv = dev->drv_pv;

  dprintk("%s\n", __FUNCTION__);

  if (rq->attribute != VALIO_HWCLOCK_DATE
      || (rq->type != DEVICE_VALIO_READ
          && rq->type != DEVICE_VALIO_WRITE)) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
    return;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  bool_t was_empty = dev_rq_queue_isempty(&pv->queue);

  dev_valio_rq_pushback(&pv->queue, rq);

  if (was_empty)
    pcf8563_request_run(dev, pv);
}

static DEV_VALIO_CANCEL(pcf8563_cancel)
{
  struct device_s *dev = accessor->dev;
  struct pcf8563_priv_s *pv = dev->drv_pv;
  
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  if (rq == dev_valio_rq_head(&pv->queue))
    return -EBUSY;

  dev_valio_rq_remove(&pv->queue, rq);

  return 0;
}


#define pcf8563_use dev_use_generic

static DEV_INIT(pcf8563_init)
{
  struct pcf8563_priv_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = dev_drv_i2c_transaction_init(dev, &pv->i2c_txn, &pv->i2c);
  if (err)
    goto err_pv;

  pv->i2c_txn.pvdata = dev;
  pv->i2c_txn.transfer = pv->i2c_transfer;
  pv->i2c_txn.transfer_count = 2;
  pv->i2c_transfer[0].data = &pv->reg;
  pv->i2c_transfer[0].size = 1;
  pv->i2c_transfer[0].type = DEV_I2C_CTRL_TRANSACTION_WRITE;
  pv->i2c_transfer[1].data = pv->buffer;

  dev_i2c_ctrl_rq_init(&pv->i2c_txn.base, pcf8563_done);
  
  dev_rq_queue_init(&pv->queue);

  dev->drv_pv = pv;

  return 0;

 err_pv:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(pcf8563_cleanup)
{
  struct pcf8563_priv_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_drv_i2c_transaction_cleanup(&pv->i2c, &pv->i2c_txn);
  dev_rq_queue_destroy(&pv->queue);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(pcf8563_drv, 0, "PCF8563 HW Clock", pcf8563,
               DRIVER_VALIO_METHODS(pcf8563));

DRIVER_REGISTER(pcf8563_drv);
