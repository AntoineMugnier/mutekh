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

  This implements a RGB LED driver driving all three colors channels
  as on/off only. It gives 8-color RGB, hence RGB3.  It exposes LED
  control through relevant valio subclass.  Threshold for lightning a
  channel up is 0x80 (included).

  Sample instantiation:

  DEV_DECLARE_STATIC(led_dev, "led", 0, rgb3_gpio_drv,
                     DEV_STATIC_RES_DEV_GPIO("/gpio"),
                     DEV_STATIC_RES_GPIO("_red", 21, 1),
                     DEV_STATIC_RES_GPIO("_green", 22, 1),
                     DEV_STATIC_RES_GPIO("_blue", 23, 1),
                     DEV_STATIC_RES_UINT_PARAM("active_mask", 0),
                     );

  active_mask may be ommitted and defaults to 0x7 (active high LED
  control).

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

#include <device/class/gpio.h>
#include <device/class/valio.h>
#include <device/valio/led.h>

enum rgb3_gpio_id_e
{
  RGB3_GPIO_RED,
  RGB3_GPIO_GREEN,
  RGB3_GPIO_BLUE,
  RGB3_GPIO_COUNT,
};

struct led_data_s
{
  bool_t active_value;
  uint8_t lum;
  gpio_id_t gpio_id;
};

struct rgb3_gpio_s
{
  struct device_gpio_s gpio;
  struct led_data_s led[RGB3_GPIO_COUNT];
};

DRIVER_PV(struct rgb3_gpio_s);

static
void rgb3_gpio_gpio_run(struct rgb3_gpio_s *pv)
{
  for (size_t i = 0; i < 3; ++i) {
    struct led_data_s *led = &pv->led[i];
    bool_t is_on = led->lum >= 0x80;
    
    dev_gpio_out(&pv->gpio, led->gpio_id,
                 is_on ? led->active_value : !led->active_value);
  }
}

static DEV_VALIO_REQUEST(rgb3_gpio_request)
{
  struct device_s *dev = accessor->dev;
  struct rgb3_gpio_s *pv = dev->drv_pv;

  logk_trace("%s", __FUNCTION__);

  if (rq->attribute != VALIO_LED || rq->type != DEVICE_VALIO_WRITE) {
    rq->error = -ENOTSUP;
  } else {
    const struct valio_led_luminosity_s *s = rq->data;

    rq->error = 0;

    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    for (size_t i = 0; i < 3; ++i) {
      if (pv->led[i].lum != s->lum[i]) {
        pv->led[i].lum = s->lum[i];
      }
    }

    rgb3_gpio_gpio_run(pv);
  }

  dev_valio_rq_done(rq);
}

static DEV_VALIO_CANCEL(rgb3_gpio_cancel)
{
  return -ENOENT;
}

#define rgb3_gpio_use dev_use_generic

static DEV_INIT(rgb3_gpio_init)
{
  static const char *led_name[RGB3_GPIO_COUNT] = {
    [RGB3_GPIO_RED] = "red",
    [RGB3_GPIO_GREEN] = "green",
    [RGB3_GPIO_BLUE] = "blue",
  };

  struct rgb3_gpio_s *pv;
  error_t err;
  uintptr_t tmp;
  gpio_width_t width;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err) {
    logk_fatal("Cannot get GPIO accessor");
    goto free_pv;
  }

  err = device_get_param_uint(dev, "active_mask", &tmp);
  if (err)
    tmp = 7;

  for (uint8_t i = 0; i < RGB3_GPIO_COUNT; ++i) {
    err = device_gpio_get_setup(&pv->gpio, dev, led_name[i], &pv->led[i].gpio_id, &width);
    if (err) {
      logk_fatal("No '%s' param", led_name[i]);
      goto put_gpio;
    }

    pv->led[i].active_value = bit_get(tmp, i);
    dev_gpio_mode(&pv->gpio, pv->led[i].gpio_id, DEV_PIN_PUSHPULL);
  }

  rgb3_gpio_gpio_run(pv);

  return 0;

 put_gpio:
  device_put_accessor(&pv->gpio.base);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(rgb3_gpio_cleanup)
{
  struct rgb3_gpio_s *pv = dev->drv_pv;

  device_put_accessor(&pv->gpio.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(rgb3_gpio_drv, 0, "Software RGB3 LED", rgb3_gpio,
               DRIVER_VALIO_METHODS(rgb3_gpio));

DRIVER_REGISTER(rgb3_gpio_drv);
