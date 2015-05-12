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
#include <device/request.h>
#include <device/resources.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <enums.h>


/* Forward declarations. */
struct device_pwm_s;


/** PWM polarity. */
ENUM_DESCRIPTOR(dev_pwm_polarity_e, strip:DEV_PWM_POL_, upper);

enum dev_pwm_polarity_e
{
  /* The duty cycle represents the high part of the output signal,
     which is at the beginning of each period. */
  DEV_PWM_POL_HIGH,

  /* The duty cycle represents the low part of the output signal,
     which is at the beginning of each period. */
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

struct dev_pwm_config_s
{
  /* Frequency */
  struct dev_freq_s       freq;

  /* Duty cycle. */
  struct dev_freq_ratio_s duty;

  /* Output polarity. */
  enum dev_pwm_polarity_e pol;

  /* Setter mask. */
  uint_fast8_t            mask;
};

struct dev_pwm_rq_s
{
  /* Generic request. */
  struct dev_request_s          base;

  /* Error. */
  error_t                       error;

  /* Channel configurations. */
  const struct dev_pwm_config_s **cfg;

  /* Channel mask. */
  uint32_t                      chan_mask;
};

STRUCT_INHERIT(dev_pwm_rq_s, dev_request_s, base);

#define DEV_PWM_CONFIG(n) void (n)(struct device_pwm_s *pdev, \
                                  struct dev_pwm_rq_s  *rq)   \
/**/

/** @This tries to configure some PWM channels. The first channel to configure
    is identified by the device accessor. Subsequent channels with a
    configuration specified in the @tt cfg array of the request are selected by
    the @tt chan_mask field. The least significant bit of the mask corresponds
    to the API instance number given by the accessor and must be set. The size
    of the array must match the number of bits set in the @tt chan_mask field.

    The @ref dev_pwm_config_s::mask field selects the parameters that are used
    in the configuration. All channels require being started by calling
    @ref device_start before doing any configuration.

    If a configuration parameter is shared between PWM channels, one
    configuration may conflict with other channels. It is not allowed to update
    a shared parameter on different channels using separate calls to this
    function if they were already started by a call to @ref device_start.  When
    the value of a shared configuration parameter is changed, all passed @ref
    dev_pwm_config_s must agree on the mask and value for this parameter.
    This way the caller does not need to know if the parameter is actually shared.

    With a single call to @this, one can set the configuration of multiple
    channels atomically if this is supported by the the hardware. The channel
    output signal is actually started on the first call to @this.

    The @tt kr is executed upon completion. In case of error, the @tt dev_request_s::kr
    kroutine is called with the @tt error field set appropriately. If at least one
    error condition is detected for a single channel no configuration is performed at
    all on other channels.

    @table 2 {Error values}
 
    @item Error @item description
    @c---------------------------------------------------
 
    @item 0 
    @item Success

    @item -ECANCELED
    @item New configuration requested while previous one is not completed

    @item -ENOTSUP
    @item Conflict on shared parameter

    @item -ERANGE
    @item Configuration can not be achieved

    @item -EINVAL
    @item Configuration of a channel not started
    
    @end table

*/
typedef DEV_PWM_CONFIG(dev_pwm_config_t);

/** Driver types. */
DRIVER_CLASS_TYPES(pwm,
                   dev_pwm_config_t *f_config;
                  );


/** @This configures the PWM devices and blocks until the configuration is
    effective or an error occured.

    If the configuration cannot be applied, the function returns an error code.

    This function takes a list of device accessors along with associated
    configurations. It may performs multiple calls to the @ref dev_pwm_config_t
    function if multiple devices are involved.
 */

config_depend(CONFIG_DEVICE_PWM)

#if defined(CONFIG_MUTEK_SCHEDULER)

inline error_t dev_pwm_wait_config(struct device_pwm_s *pdev, const struct dev_pwm_config_s *cfg)
{
     const struct dev_pwm_config_s * pcfg[1] = {cfg};
     struct dev_request_status_s status;

     struct dev_pwm_rq_s rq =
     {
       .cfg = pcfg,
       .chan_mask = 1,
       .error = 0,
     };

     dev_request_sched_init(&rq.base, &status);

     DEVICE_OP(pdev, config, &rq);

     dev_request_sched_wait(&status);

     return rq.error;
}
#endif

inline error_t dev_pwm_spin_config(struct device_pwm_s *pdev, const struct dev_pwm_config_s *cfg)
{
     const struct dev_pwm_config_s * pcfg[1] = {cfg};
     struct dev_request_status_s status;

     struct dev_pwm_rq_s rq =
     {
       .cfg = pcfg,
       .chan_mask = 1,
       .error = 0,
     };

     dev_request_spin_init(&rq.base, &status);

     DEVICE_OP(pdev, config, &rq);

     dev_request_spin_wait(&status);

     return rq.error;
}

#endif

