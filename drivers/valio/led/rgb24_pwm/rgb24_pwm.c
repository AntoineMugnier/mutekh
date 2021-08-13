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

  This implements a RGB LED driver using a PWM device as backend.  It
  provides the full 24-bit RGB control.  Pin mapping is done as part
  of PWM device instantiation.  This driver uses the first three
  channels of the backend PWM device.

  Sample instantiation:

  DEV_DECLARE_STATIC(led_dev, "led", 0, rgb24_pwm_drv,
                     DEV_STATIC_RES_DEVCLASS_PARAM("pwm", "/pwm0", DRIVER_CLASS_PWM),
                     DEV_STATIC_RES_UINT_PARAM("hz", 32),
                     DEV_STATIC_RES_UINT_PARAM("active_mask", 0),
                     );

  active_mask may be ommitted and defaults to 0x7 (active high LED
  control).

  hz is target pwm frequency.

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

#include <device/class/pwm.h>
#include <device/class/valio.h>
#include <device/valio/led.h>

struct rgb24_pwm_s
{
  struct device_pwm_s pwm;
  struct dev_pwm_rq_s pwm_rq;
  struct dev_pwm_config_s pwm_config[3];
};

STRUCT_COMPOSE(rgb24_pwm_s, pwm_rq);
DRIVER_PV(struct rgb24_pwm_s);

static DEV_VALIO_REQUEST(rgb24_pwm_request)
{
  struct device_s *dev = accessor->dev;
  struct rgb24_pwm_s *pv = dev->drv_pv;

  logk_trace("%s", __FUNCTION__);

  if (rq->attribute != VALIO_LED || rq->type != DEVICE_VALIO_WRITE) {
    rq->error = -ENOTSUP;
  } else {
    const struct valio_led_luminosity_s *s = rq->data;
    bool_t one = 0;

    rq->error = 0;

    LOCK_SPIN_IRQ_SCOPED(&dev->lock);
    
    for (size_t i = 0; i < 3; ++i) {
      if (pv->pwm_config[i].duty.num != s->lum[i]) {
        pv->pwm_config[i].param_mask = DEV_PWM_MASK_DUTY;
        pv->pwm_config[i].duty.num = s->lum[i];
        one = 1;
      }
    }

    if (one)
      DEVICE_OP(&pv->pwm, config, &pv->pwm_rq);
  }

  dev_valio_rq_done(rq);
}

static DEV_VALIO_CANCEL(rgb24_pwm_cancel)
{
  return -ENOENT;
}

#define rgb24_pwm_use dev_use_generic

static DEV_INIT(rgb24_pwm_init)
{
  struct rgb24_pwm_s *pv;
  error_t err;
  uintptr_t tmp, hz;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "pwm", &pv->pwm.base, DRIVER_CLASS_PWM);
  if (err) {
    logk_fatal("Cannot get pwm accessor");
    goto free_pv;
  }

  err = device_get_param_uint(dev, "hz", &tmp);
  hz = err ? 128 : tmp;

  err = device_get_param_uint(dev, "active_mask", &tmp);
  if (err)
    tmp = 7;

  for (uint8_t i = 0; i < 3; ++i) {
    pv->pwm_config[i].pol = bit_get(tmp, i) ? DEV_PWM_POL_HIGH : DEV_PWM_POL_LOW;
    pv->pwm_config[i].freq.num = hz;
    pv->pwm_config[i].freq.denom = 1;
    pv->pwm_config[i].duty.num = 0;
    pv->pwm_config[i].duty.denom = 255;
    pv->pwm_config[i].chan_mask = bit(i);
    pv->pwm_config[i].param_mask = DEV_PWM_MASK_FREQ | DEV_PWM_MASK_DUTY | DEV_PWM_MASK_POL;
  }

  pv->pwm_rq.cfg_count = 3;
  pv->pwm_rq.cfg = pv->pwm_config;
  DEVICE_OP(&pv->pwm, config, &pv->pwm_rq);

  return 0;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(rgb24_pwm_cleanup)
{
  struct rgb24_pwm_s *pv = dev->drv_pv;

  device_put_accessor(&pv->pwm.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(rgb24_pwm_drv, 0, "PWM RGB24 LED", rgb24_pwm,
               DRIVER_VALIO_METHODS(rgb24_pwm));

DRIVER_REGISTER(rgb24_pwm_drv);
