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
 * @file
 * @module{Devices support library}
 * @short PWM driver configuration API
 */

#ifndef __DEVICE_PWM_H__
#define __DEVICE_PWM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>


/* forward declarations. */
struct device_pwm_s;


/** PWM polarity. */
enum dev_pwm_polarity_e
{
  DEV_PWM_POL_NONE,
  DEV_PWM_POL_HIGH,
  DEV_PWM_POL_LOW,
};

/* PWM fraction. */
struct dev_pwm_fract_s
{
  uint_fast32_t num;
  uint_fast32_t denom;
};

#define DEVPWM_FREQ(n) error_t (n)(struct device_pwm_s    *pdev, \
                                   struct dev_pwm_fract_s *freq) \
/**/

/** @This configures the frequency of the PWM output. If the PWM device has
    multiple channels, all of them will have the same frequency.

    If the @tt freq value is zero, the function acts as a getter and retreive
    the current PWM frequency.

    If the required frequency is impossible to configure, the return value is
    @tt -ENOTSUP.
*/
typedef DEVPWM_FREQ(devpwm_freq_t);


#define DEVPWM_DUTY(n) error_t (n)(struct device_pwm_s    *pdev,   \
                                   uint_fast8_t           channel, \
                                   struct dev_pwm_fract_s *duty)   \
/**/

/** @This configures the duty cycle of a given channel of the PWM device.

    If the @tt duty value is zero, the function acts as a getter and
    retreive the current PWM channel duty cycle.

    If the @tt channel value is out of bounds of the PWM device, the return
    value is @tt -EIO.

    If the duty cycle cannot be configured the return value is -ENOTSUP.

    Note: The duty cycle must be give as a value in the range (0,1].
**/
typedef DEVPWM_DUTY(devpwm_duty_t);


#define DEVPWM_POLARITY(n) error_t (n)(struct device_pwm_s     *pdev,   \
                                       uint_fast8_t            channel, \
                                       enum dev_pwm_polarity_e *pol)    \
/**/

/** @This configures the duty cycle of a given channel of the PWM device.

    If the @tt pol value is @tt DEV_PWM_POL_NONE, the function acts as a
    getter and retreive the current PWM channel duty cycle.

    If the @tt channel value is out of bounds of the PWM device, the return
    value is @tt -EIO.

    If the duty cycle cannot be configured the return value is -ENOTSUP.
**/
typedef DEVPWM_POLARITY(devpwm_polarity_t);


#define DEVPWM_START_STOP(n) error_t (n) (struct device_pwm_s *pdev,   \
                                          uint_fast8_t        channel, \
                                          bool_t              start)   \
/**/

/** @This starts/stops the given PWM channel. */
typedef DEVPWM_START_STOP(devpwm_start_stop_t);

/** Driver types. */
DRIVER_CLASS_TYPES(pwm,
                   devpwm_freq_t       *f_freq;
                   devpwm_duty_t       *f_duty;
                   devpwm_polarity_t   *f_polarity;
                   devpwm_start_stop_t *f_start_stop;
                  );

#endif

