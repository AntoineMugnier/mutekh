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

/*

  This implements a RGB LED driver using a software timer and GPIO
  control to do PWM.  It consumes CPU cycles to do PWM.  It is meant
  to provide the full 24-bit RGB control, but may actually not be able
  to provide this depending on timer resolution and CPU capacity.

  Sample instantiation:

  DEV_DECLARE_STATIC(led_dev, "led", 0, rgb24_gpio_drv,
                     DEV_STATIC_RES_DEV_TIMER("/rtc* /timer*"),
                     DEV_STATIC_RES_DEV_GPIO("/gpio"),
                     DEV_STATIC_RES_GPIO("_red", 21, 1),
                     DEV_STATIC_RES_GPIO("_green", 22, 1),
                     DEV_STATIC_RES_GPIO("_blue", 23, 1),
                     DEV_STATIC_RES_UINT_PARAM("active_mask", 0),
                     DEV_STATIC_RES_UINT_PARAM("hz", 32),
                     );

  active_mask may be ommitted and defaults to 0x7 (active high LED
  control).

  hz is pwm frequency.

 */

#define LOGK_MODULE_ID "xrgb"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/timer.h>
#include <device/class/gpio.h>
#include <device/class/valio.h>
#include <device/valio/led.h>

enum rgb24_gpio_id_e
{
  RGB24_GPIO_RED,
  RGB24_GPIO_GREEN,
  RGB24_GPIO_BLUE,
  RGB24_GPIO_COUNT,
};

struct led_data_s
{
  dev_timer_delay_t on_time;
  bool_t dirty;
  uint8_t lum;

  bool_t active_value;
  gpio_id_t gpio_id;
};

struct rgb24_gpio_s
{
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  struct device_gpio_s gpio;
  dev_timer_delay_t period;
  dev_timer_value_t last_on;
  uint32_t hz;

  bool_t scheduled;
  bool_t started;

  struct led_data_s led[RGB24_GPIO_COUNT];
};

STRUCT_COMPOSE(rgb24_gpio_s, timer_rq);
DRIVER_PV(struct rgb24_gpio_s);

static
void rgb24_gpio_gpio_run(struct rgb24_gpio_s *pv)
{
  dev_timer_value_t now, next_on, next_deadline;

 again:
  DEVICE_OP(&pv->timer, get_value, &now, 0);

  next_on = pv->last_on + pv->period;

  if (next_on + pv->period < now) {
    pv->last_on = now;
    next_on = now + pv->period;
  }

  logk_debug("run now %d last_on: %d, next_on: %d",
             (int32_t)now, (int32_t)pv->last_on, (int32_t)next_on);
  
  next_deadline = next_on;
  bool_t any_blink = 0, off_to_go = 0;

  for (size_t i = 0; i < 3; ++i) {
    struct led_data_s *led = &pv->led[i];
    if (led->dirty) {
      led->dirty = 0;
      dev_timer_init_sec(&pv->timer, &led->on_time, NULL, led->lum, 255 * pv->hz);
    }

    dev_timer_value_t off_date = pv->last_on + led->on_time;
    bool_t is_on = now < off_date;

    logk_debug("led %d, on_time: %d, is_on %d, off at %d", i, (int32_t)led->on_time, is_on, (int32_t)off_date);

    if (led->lum != 0 && led->lum != 255)
      any_blink = 1;

    if (led->lum != 255 && is_on)
      off_to_go = 1;
    
    dev_gpio_out(&pv->gpio, led->gpio_id,
                 is_on ? led->active_value : !led->active_value);
    if (is_on && next_deadline > off_date)
      next_deadline = off_date;
  }

  logk_debug("any %d next deadline %d", any_blink, (int32_t)next_deadline);

  if (!off_to_go)
    pv->last_on = next_on;
  
  if (any_blink) {
    pv->timer_rq.deadline = next_deadline;
    pv->timer_rq.delay = 0;

    if (!pv->started) {
      pv->started = 1;
      device_start(&pv->timer.base);
    }
    
    error_t err = DEVICE_OP(&pv->timer, request, &pv->timer_rq);

    logk_debug("err %d", err);

    if (err == -ETIMEDOUT)
      goto again;

    pv->scheduled = 1;
  } else {
    if (pv->started) {
      pv->started = 0;
      device_stop(&pv->timer.base);
    }
  }
}

static KROUTINE_EXEC(rgb24_gpio_timer_done)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct rgb24_gpio_s *pv = rgb24_gpio_s_from_timer_rq(rq);
  struct device_s *dev = pv->timer_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  pv->scheduled = 0;
  rgb24_gpio_gpio_run(pv);
}

static DEV_VALIO_REQUEST(rgb24_gpio_request)
{
  struct device_s *dev = accessor->dev;
  struct rgb24_gpio_s *pv = dev->drv_pv;

  logk_trace("%s", __FUNCTION__);

  if (rq->attribute != VALIO_LED || rq->type != DEVICE_VALIO_WRITE) {
    rq->error = -ENOTSUP;
  } else {
    const struct valio_led_luminosity_s *s = rq->data;

    rq->error = 0;

    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    for (size_t i = 0; i < 3; ++i) {
      if (pv->led[i].lum != s->lum[i]) {
        pv->led[i].dirty = 1;
        pv->led[i].lum = s->lum[i];
      }
    }

    if (!pv->started) {
      pv->started = 1;
      device_start(&pv->timer.base);
    }

    if (!pv->scheduled)
      rgb24_gpio_gpio_run(pv);
  }

  dev_valio_rq_done(rq);
}

static DEV_VALIO_CANCEL(rgb24_gpio_cancel)
{
  return -ENOENT;
}

#define rgb24_gpio_use dev_use_generic

static DEV_INIT(rgb24_gpio_init)
{
  static const char *led_name[RGB24_GPIO_COUNT] = {
    [RGB24_GPIO_RED] = "red",
    [RGB24_GPIO_GREEN] = "green",
    [RGB24_GPIO_BLUE] = "blue",
  };

  struct rgb24_gpio_s *pv;
  error_t err;
  uintptr_t tmp;
  gpio_width_t width;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER);
  if (err) {
    logk_fatal("Cannot get timer accessor");
    goto free_pv;
  }

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err) {
    logk_fatal("Cannot get GPIO accessor");
    goto put_timer;
  }

  err = device_get_param_uint(dev, "hz", &tmp);
  pv->hz = err ? 16 : tmp;

  err = device_get_param_uint(dev, "active_mask", &tmp);
  if (err)
    tmp = 7;

  dev_timer_init_sec(&pv->timer, &pv->period, NULL, 1, pv->hz);

  for (uint8_t i = 0; i < RGB24_GPIO_COUNT; ++i) {
    err = device_gpio_get_setup(&pv->gpio, dev, led_name[i], &pv->led[i].gpio_id, &width);
    if (err) {
      logk_fatal("No '%s' param", led_name[i]);
      goto put_gpio;
    }

    pv->led[i].active_value = bit_get(tmp, i);
    dev_gpio_mode(&pv->gpio, pv->led[i].gpio_id, DEV_PIN_PUSHPULL);
  }

  dev_timer_rq_init(&pv->timer_rq, rgb24_gpio_timer_done);
  pv->timer_rq.pvdata = dev;

  return 0;

 put_gpio:
  device_put_accessor(&pv->gpio.base);
 put_timer:
  device_put_accessor(&pv->timer.base);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(rgb24_gpio_cleanup)
{
  struct rgb24_gpio_s *pv = dev->drv_pv;

  device_put_accessor(&pv->timer.base);
  device_put_accessor(&pv->gpio.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(rgb24_gpio_drv, 0, "Software RGB24 LED", rgb24_gpio,
               DRIVER_VALIO_METHODS(rgb24_gpio));

DRIVER_REGISTER(rgb24_gpio_drv);
