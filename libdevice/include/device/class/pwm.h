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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Sébastien Cerdan <scerdan@gmail.com> (c) 2014

*/

/**
   @file
   @module {Core::Devices support library}
   @short Pulse Width Modulation controller driver API
   @index {Pulse Width Modulation controller} {Device classes}
   @csee DRIVER_CLASS_PWM

   This device class API allows configuration of multiple PWM channels
   provided by a single device.

   The channel mapping exposed by the driver can be specified in the
   device resources. When no mapping resources is specified, the API
   channel indices follows the hardware channel order.
@code
   DEV_STATIC_RES_UINT_PARAM("remap", LUT_8_4_DEF(1, 2, 0))
@end code
*/

#ifndef __DEVICE_PWM_H__
#define __DEVICE_PWM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>

#include <hexo/enum.h>
#include <hexo/lut.h>


/* Forward declarations. */
struct device_pwm_s;


/** PWM polarity. */
ENUM_DESCRIPTOR(dev_pwm_polarity_e, strip:DEV_PWM_POL_, upper);

enum dev_pwm_polarity_e
{
  /* The duty cycle represents the high part of the output signal. */
  DEV_PWM_POL_HIGH,

  /* The duty cycle represents the low part of the output signal. */
  DEV_PWM_POL_LOW,
};

/* Configuration mask. */
enum dev_pwm_mask_e
{
  /* Not used. */
  DEV_PWM_MASK_NONE = 0x0,

  /* PWM frequency. */
  DEV_PWM_MASK_FREQ = 0x1,

  /* PWM channel duty cycle. */
  DEV_PWM_MASK_DUTY = 0x2,

  /* PWM channel polarity. */
  DEV_PWM_MASK_POL  = 0x4,
};

struct dev_pwm_rq_s
{
  /* Mask of impacted channels. */
  uint32_t                chan_mask;

  /* Mask of updated parameters. */
  uint_fast8_t            param_mask;

  /* Frequency parameter */
  struct dev_freq_s       freq;

  /* Duty cycle parameter */
  struct dev_freq_ratio_s duty;

  /* Output polarity parameter */
  enum dev_pwm_polarity_e pol;
};

/** @see dev_pwm_config_t */
#define DEV_PWM_CONFIG(n) error_t (n)(const struct device_pwm_s *pdev, \
                                       struct dev_pwm_rq_s  *rq)

/** @This configures some PWM channels. Channels are selected by the
    @ref dev_pwm_rq_s::chan_mask field.

    The @ref dev_pwm_rq_s::param_mask field selects the parameters
    that are updated. A single call to @this, can set the
    configuration of multiple channels atomically if this is supported
    by the the hardware.

    Depending on the hardware design, some parameters may be shared
    between channels. This is usually the case for the frequency
    parameters. The @tt -ENOTSUP error is reported when such a
    parameter is not changed for all channels at the same time.

    A channel output has no transitions when the duty cycle
    is either 0 or 1. The duty cycle of channels is set to 0 when
    the driver is initialized.

    The @ref device_start function must be called in order to start
    operation. All channels are started simultaneously when the device
    is started.

    The PWM output signal will change as soon as possible. This may
    occur after the return of the function.
*/
typedef DEV_PWM_CONFIG(dev_pwm_config_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_PWM, pwm,
                   dev_pwm_config_t *f_config;
                  );

/** @see driver_pwm_s */
#define DRIVER_PWM_METHODS(prefix)                            \
  ((const struct driver_class_s*)&(const struct driver_pwm_s){  \
    .class_ = DRIVER_CLASS_PWM,                               \
    .f_config = prefix ## _config,                            \
  })

DEV_REQUEST_WAIT_FUNC(pwm);

#endif

