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

#include <mutek/kroutine.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <enums.h>


/* forward declarations. */
struct device_pwm_s;

struct dev_pwm_rq_queue_s;


/** PWM polarity. */
ENUM_DESCRIPTOR(dev_pwm_polarity_e, strip:DEV_PWM_POL_, lower);

enum dev_pwm_polarity_e
{
  /* The duty cycle represents the high part of the output signal,
     which is at the beginning of each period. */
  DEV_PWM_POL_HIGH,

  /* The duty cycle represents the low part of the output signal,
     which is at the beginning of each period. */
  DEV_PWM_POL_LOW,
};

/* Configuration mode. */
ENUM_DESCRIPTOR(dev_pwm_mode_e, strip:DEV_PWM_MODE_, lower);

enum dev_pwm_mode_e
{
  /** Shared mode: the configuration of the PWM controller and/or channels
      may be altered, including if multiple accessors are registered.
   */
  DEV_PWM_MODE_SHARED,

  /** Exclusive mode: the configuration of the PWM controller and/or
      channels cannot be altered using another accessor than the one that
      were used for setting up the current configuration. 
   */
  DEV_PWM_MODE_EXCL,

  /** Fixed mode: the configuration of the PWM controller and/or channels
      cannot be altered until an accessor on the device is registered.
   */
  DEV_PWM_MODE_FIXED,
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

  /* Mode. */
  DEV_PWM_MASK_MODE = 0x8,
};

struct dev_pwm_config_s
{
  /* Kroutine. */
  struct kroutine_s       kr;

  /* Associated device. */
  struct device_pwm_s     *pdev;

  /* Kroutine private data. */
  void                    *pvdata;

  /* Frequency */
  struct dev_freq_s       freq;

  /* Duty cycle. */
  struct dev_freq_ratio_s duty;

  /* Output polarity. */
  enum dev_pwm_polarity_e pol;

  /* Configuration mode. */
  enum dev_pwm_mode_e     mode;

  /* Setter mask. */
  uint_fast8_t            mask;

  /* Error. */
  error_t                 error;
};


#define DEVPWM_CONFIG(n) error_t (n)(struct device_pwm_s     *pdev, \
                                     struct dev_pwm_config_s *cfg)  \
/**/

/** @This tries to apply the configuration. The @tt kr and the @tt mask
    must be initialized accordingly.

    If the function returns 0, then the configuration request is consistent.
    However, this does not guarantee that the configuration can be
    applied. The @tt error field must be used inside the kroutine to check
    effectiveness of the configuration.

    The value of the @tt mask defines the parameters that must be took
    into account for the configuration. Depending on the PWM channel share
    the controller with other channel, the change of the frequency may
    affect other channels. This is architecture dependant.

    The @tt mode field defines the way the PWM controller and channels may
    be updated. Depending on the chosen mode, a unique accessor (i.e.
    task) can alter the configuration or, at the opposite, everyone can
    alter the configuration. @see dev_pwm_mode_e. If the mode is
    @ref #DEV_PWM_MODE_EXCL, then the request fifo is filtered to consume
    request pushed by the owning accessor. The request order is kept stable.

    If the configuration alter the duty cycle or the polarity of the channel,
    the @tt chan field must be initialized accordingly, otherwise the
    configuration may have unexpected behavior.

    If the @tt error field value is -EINVAL, the configuration failed to
    be applied.
*/
typedef DEVPWM_CONFIG(devpwm_config_t);


#define GCT_CONTAINER_ALGO_dev_pwm_queue CLIST

struct dev_pwm_request_s
{
  /* Kroutine called on completion. */
  struct kroutine_s         kr;

  /* PWM device. */
  struct device_pwm_s       *pdev;

  /* Kroutine private data. */
  void                      *pvdata;

  /* Parent queue. */
  struct dev_pwm_rq_queue_s *queue;

  /* Associated configuration. */
  struct dev_pwm_config_s   *cfg;

  /* Priority. */
  bool_t                    priority:1;

  /* Error. */
  error_t                   error;

  /* GCT request queue hook. */
  GCT_CONTAINER_ENTRY(dev_pwm_queue, queue_entry);
};

/* GCT stuff. */
GCT_CONTAINER_TYPES(dev_pwm_queue, struct dev_pwm_request_s *, queue_entry);
GCT_CONTAINER_FCNS(dev_pwm_queue, inline, dev_pwm_queue,
                   init, destroy, isempty, head, remove, push, pushback);

/** Configuration queue. */
struct dev_pwm_rq_queue_s
{
    /** queue. */
    dev_pwm_queue_root_t     queue;

    /** lock (interruptible) */
    lock_irq_t               lock;

    /** request marker to prevent infinite loops. */
    struct dev_pwm_request_s *marker;
};


#define DEVPWM_QUEUE(n) struct dev_pwm_rq_queue_s * (n) ( \
    struct device_pwm_s *pdev)                            \
/**/

/** @This gives access to the request queue allocated in the PWM device driver
    private data.
 */
typedef DEVPWM_QUEUE(devpwm_queue_t);

/** Driver types. */
DRIVER_CLASS_TYPES(pwm,
                   devpwm_config_t *f_config;
                   devpwm_queue_t  *f_queue;
                  );


/** @This initializes a PWM request queue for use in a PWM controller devie
    driver. It is usually called from the @ref DEVICE_INIT function to
    initialize a queue stored in the driver private data.
*/
error_t dev_pwm_rq_queue_init(struct dev_pwm_rq_queue_s *q);


/** @this cleans up the queue and release lock resource. */
void dev_pwm_rq_queue_cleanup(struct dev_pwm_rq_queue_s *q);


/** @This initializes a configuration request. */
error_t dev_pwm_request_init(struct device_pwm_s      *pdev,
                             struct dev_pwm_request_s *rq);


/** @This starts a configuration request. When this function is called the
    request is pushed in the processing queue according to the request
    priority.
 */
error_t dev_pwm_request_start(struct dev_pwm_request_s *rq);


/** @This execute pending requests in the given queue. @This should be called
    from the driver @ref #DEV_USE function on event @tt DEV_USE_PUT_ACCESSOR.
    It is used to restart execution in the case of an exclusive mode.
 */
void dev_pwm_execute(struct dev_pwm_rq_queue_s *q);


#if defined(CONFIG_MUTEK_SCHEDULER)

/** @This configures the PWM device and blocks until the configuration is
    effective or an error occured.

    If the configuration cannot be applied, the function returns -EINVAL.
 */
error_t dev_pwm_config(struct device_pwm_s     *pdev,
                       struct dev_pwm_config_s *cfg);

#endif

#endif

