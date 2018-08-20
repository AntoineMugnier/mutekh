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

#include <hexo/enum.h>


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
};

struct dev_pwm_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
  };

  /* Channel configurations. */
  const struct dev_pwm_config_s *cfg;

  /* Channel mask. */
  uint32_t                chan_mask;

  /* Setter mask. */
  uint_fast8_t            mask;
};

DEV_REQUEST_INHERIT(pwm); DEV_REQUEST_QUEUE_OPS(pwm);

/** @see dev_pwm_config_t */
#define DEV_PWM_CONFIG(n) void (n)(struct device_pwm_s *pdev, \
                                  struct dev_pwm_rq_s  *rq)

/** @This configures some PWM channels. The first channel to configure
    is identified by the device accessor number. Subsequent channels
    with a configuration specified in the @tt cfg array of the request
    are selected by the @tt chan_mask field. The least significant bit
    of the mask corresponds to the API instance number given by the
    accessor and must be set. The size of the array must match the
    number of bits set in the @tt chan_mask field.

    The @ref dev_pwm_rq_s::mask field selects the parameters that are
    updated. A single call to @this, can set the configuration of
    multiple channels atomically if this is supported by the the
    hardware.

    Depending on the hardware design, some parameters may be shared
    between channels. This is usually the case for the frequency
    parameters. The configuration of all started channels with a
    shared parameter must be updated at the same time. A channel is
    considered started when it's duty cycle is neither 0 nor 1. In any
    case, if the value of a shared parameter is not updated simultaneously
    for all started channels, the @tt -ENOTSUP error is reported.

    The duty cycle of channels is set to 0 when the driver is
    initialized. The @ref device_start function can be called in order
    to keep the internal counter running when channels are not
    started.

    The @tt kr is executed upon completion. In case of error, the @tt
    dev_request_s::kr kroutine is called with the @tt error field set
    appropriately. If at least one error condition is detected for a
    single channel no configuration is performed at all on other
    channels.

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

    @end table

   The kroutine of the request may be executed from within this
   function. Please read @xref {Nested device request completion}. */
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


/** @This configures the PWM devices and blocks until the configuration is
    effective or an error occured.

    If the configuration cannot be applied, the function returns an error code.

    @param pdev a pwm device accessor pointing to a pwm channel.
    @param cfg  the configuration to apply to the pwm channel.
    @param mask a bitmask defining which configuration fields to consider.
 */
config_depend_and2_inline(CONFIG_DEVICE_PWM, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_pwm_wait_config(struct device_pwm_s *pdev, const struct dev_pwm_config_s *cfg, uint_fast8_t mask),
{
     struct dev_request_status_s status;

     struct dev_pwm_rq_s rq =
     {
       .cfg = cfg,
       .chan_mask = 1,
       .mask = mask,
     };

     dev_request_sched_init(&rq.base, &status);

     DEVICE_OP(pdev, config, &rq);

     dev_request_sched_wait(&status);

     return rq.error;
});

config_depend_and2_inline(CONFIG_DEVICE_PWM, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_pwm_wait_rq(struct device_pwm_s *pdev, struct dev_pwm_rq_s *rq),
{
     struct dev_request_status_s status;

     dev_request_sched_init(&rq->base, &status);

     DEVICE_OP(pdev, config, rq);

     dev_request_sched_wait(&status);

     return rq->error;
});

/** @see dev_pwm_wait_config */
BUSY_WAITING_FUNCTION
config_depend_inline(CONFIG_DEVICE_PWM,
error_t dev_pwm_spin_config(struct device_pwm_s *pdev, const struct dev_pwm_config_s *cfg, uint_fast8_t mask),
{
     struct dev_request_status_s status;

     struct dev_pwm_rq_s rq =
     {
       .cfg = cfg,
       .chan_mask = 1,
       .mask = mask,
     };

     dev_request_spin_init(&rq.base, &status);

     DEVICE_OP(pdev, config, &rq);

     dev_request_spin_wait(&status);

     return rq.error;
})

#endif

