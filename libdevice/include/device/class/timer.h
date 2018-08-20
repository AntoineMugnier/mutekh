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

    Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2012

*/

/**
   @file
   @module {Core::Devices support library}
   @short Timer device driver API
   @index {Timer device} {Device classes}
   @csee DRIVER_CLASS_TIMER

   The timer device API allows, reading the current counter value and
   the current configuration of a timer device as well as queuing
   delay requests. Some timer implementations may no support requests
   and may only provide a counter value. The counter value exposed by
   the API is increasing.

   The timer need to be started by calling the @ref device_start
   function in order to have it running when no request are
   queued. Depending on hardware design, some timer can't be put in
   stopped state or lose the current timer value when stopped.

   A configuration function is provided to read the timer parameters
   useful to perform time unit conversion and to change the
   resolution. This also provides some hints about timer hardware
   capabilities.

   @section {Timer resolution}

   The resolution specifies the divider between the timer input clock
   and the timer counter value.

   @end section

   @section {Timer frequency}

   There are two common ways for the driver to find the timer input
   frequency. When the driver contains a clock sink endpoint, the
   associated frequency will be computed by the clock provider device
   and may be stored by the driver of the timer. In this case the
   configuration will be updated when the clock frequency changes.

   When @ref #CONFIG_DEVICE_CLOCK token is not defined, the @ref
   device_get_res_freq function can be used by the driver. This
   function relies on the @ref DEV_RES_FREQ resources attached to the
   device. Multiple such resources are needed when the device as
   multiple input clocks.

   @end section

   @section {Hardware implementation}

   There are basically two types of timer hardware design. Those with
   a counter register which generate a periodic interrupts when it
   wraps and those with an ever increasing counter along with an
   hardware deadline comparator. There are also simple cycle counters
   with no interrupt generation capabilities.

   Depending on timer hardware design, setting the timer resolution
   will either configure an hardware prescaler which divide the timer
   input clock or simply update the timer interrupt tick period. In
   the former case, the timer hardware generally support insertion and
   cancellation of requests without loss of accuracy or race condition
   and a tick-less approach is used by the driver. The timer
   resolution can then be set as low as 1 for best time granularity.

   In case of a periodic interrupt based timer which increases a
   software counter, setting the resolution to a low value will
   generate many interrupts. The driver should then bound the usable
   resolution range. This kind of timer can also be used as a high
   resolution counter which is not able the handle requests. This is
   done by exposing the hardware counter value directly. Drivers may
   choose to provide both implementations using two different device
   numbers.

   Some timers can be driven using a mixed approach when a hardware
   deadline comparator is available with a somewhat limited bits
   width. This often provides a good trade-off between resolution and
   periodic interrupt interval.

   @end section
*/

#ifndef __DEVICE_TIMER_H__
#define __DEVICE_TIMER_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/bit.h>

#include <mutek/kroutine.h>
#include <gct_platform.h>
#include <gct/container_clist.h>

#include <device/driver.h>
#include <device/request.h>
#include <device/resources.h>

struct device_s;
struct driver_s;
struct driver_timer_s;
struct device_timer_s;

/** Timer absolute value type */
typedef uint64_t dev_timer_value_t;
/** Timer relative value type */
typedef uint32_t dev_timer_delay_t;
/** Timer device resolution type */
typedef uint32_t dev_timer_res_t;
/** Timer configuration revision number */
typedef uint32_t dev_timer_cfgrev_t;

#define PRItimerDelay PRIu32
#define PRItimerRes   PRIu32
#define PRItimerValue PRIu64
#define PRItimerRev   PRIu32

struct dev_freq_s;

/** Timer request @csee dev_timer_request_t */
struct dev_timer_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, pvdata);
    FIELD_USING(struct dev_request_s, drvdata);
  };

  /** absolute timer deadline, used when @tt delay is 0 */
  dev_timer_value_t             deadline;
  /** timer delay */
  dev_timer_delay_t             delay;
  /** expected configuration revision, ignored if 0 */
  dev_timer_cfgrev_t            rev;
};

DEV_REQUEST_INHERIT(timer);

/* expand the dev_timer_pqueue_insert function usable with generic
   device request priority queue. */
GCT_CONTAINER_KEY_TYPES(dev_request_pqueue, CUSTOM, SCALAR,
                        dev_timer_rq_s_cast(dev_request_pqueue_item)->deadline, dev_timer_pqueue);

GCT_CONTAINER_KEY_FCNS(dev_request_pqueue, ASC, inline, __dev_timer_pqueue, dev_timer_pqueue,
                       remove, insert);

DEV_REQUEST_PQUEUE_OPS(timer);

ENUM_DESCRIPTOR(dev_timer_capabilities_e, strip:DEV_TIMER_CAP_, upper, or);

enum dev_timer_capabilities_e
{
  /** The timer device is able to handle requests. */
  DEV_TIMER_CAP_REQUEST = 1,
  /** The timer device can be stopped. */
  DEV_TIMER_CAP_STOPPABLE = 2,
  /** The timer counter value is not lost and can be read when the
      timer is stopped. This flag is set if @ref
      DEV_TIMER_CAP_STOPPABLE is cleared. */
  DEV_TIMER_CAP_KEEPVALUE = 4,
  /** The timer input frequency may be changed dynamically. */
  DEV_TIMER_CAP_VARFREQ = 8,
  /** Deadline of enqueued requests are not adjusted properly when the
      timer input frequency changes. This implies @ref
      DEV_TIMER_CAP_VARFREQ. */
  DEV_TIMER_CAP_CLKSKEW = 16,
  /** The timer support resolution as low as 1. If @ref
      DEV_TIMER_CAP_REQUEST is set, an hardware deadline comparator is
      used. */
  DEV_TIMER_CAP_HIGHRES = 32,
  /** The timer never generates an irq for the sole purpose of
      increasing a counter. */
  DEV_TIMER_CAP_TICKLESS = 64,
};

/** Timer configuration @csee dev_timer_config_t */
struct dev_timer_config_s
{
  /** timer input frequency */
  struct dev_freq_s             freq;
  /** timer maximum value */
  dev_timer_value_t             max;
  /** revision id associated with this configuration, increased on
      configuration update. Can not be even. */
  dev_timer_cfgrev_t            rev;
  /** timer resolution */
  dev_timer_res_t               res;
  /** timer device capabilities */
  enum dev_timer_capabilities_e BITFIELD(cap,8);
};

/** Timer skew structure used to measure skew between two timers */
struct dev_timer_skew_s
{
  /* Difference between two timer values */
  int64_t  d;
  /* Ratio between two timer clocks */
  uint32_t num;
  uint32_t denom;
};

/** @see dev_timer_request_t */
#define DEV_TIMER_REQUEST(n)	error_t  (n) (struct device_timer_s *accessor, \
                                              struct dev_timer_rq_s *rq)

/**
   @This enqueues a timeout event request. The @tt delay and @tt
   kr fields of the request must have been initialized. If @tt delay
   is zero, the @tt deadline field must specify an absolute deadline
   timer value.

   If the function returns 0, the @ref kroutine_exec function will be
   called when the deadline is reached. It's ok to call any of the
   timer functions from the kroutine; especially the request can be
   enqueued again.

   If the deadline has already been reached, the function does nothing
   and return @tt -ETIMEDOUT.

   If the @tt revision field of the request is not 0 and doesn't match
   the internal value of the configuration revision number, the
   request is rejected and @tt -EAGAIN is returned. This allows
   ensuring time converted delays of the request is consistent with
   the current timer configuration. Valid revision numbers are odd.

   If the timer has not been started explicitly by calling the @ref
   device_start function, it will run until the request queue becomes
   empty again.

   @This may return @tt -EBUSY if the timer hardware resource is not
   available. @This may return -ENOTSUP if either there is no timer
   matching the requested device number or the timer implementation
   doesn't support requests.

   A new request may be submitted from the kroutine handler function.
   Please read @xref {Nested device request submission}.
*/
typedef DEV_TIMER_REQUEST(dev_timer_request_t);

/** @see dev_timer_cancel_t */
#define DEV_TIMER_CANCEL(n)	error_t  (n) (struct device_timer_s *accessor, \
                                              struct dev_timer_rq_s *rq)

/**
   @This cancel a timeout request event.

   The request is removed from the timer event queue. @This function
   returns @tt -ETIMEDOUT if the request is not in the queue any more
   (deadline already reached). Requests with the @tt drvdata field set
   to @tt NULL are considered not queued.

   @This returns 0 on success.
*/
typedef DEV_TIMER_CANCEL(dev_timer_cancel_t);


/** @see dev_timer_get_value_t */
#define DEV_TIMER_GET_VALUE(n)	error_t (n) (struct device_timer_s *accessor, \
                                             dev_timer_value_t *value,  \
                                             dev_timer_cfgrev_t rev)

/**
   @This reads the current raw timer value. The timer value is
   increasing with time.

   @This function may returns @tt -EIO if the timer value can not be
   accessed. This can append when trying to access a processor local
   timer from the wrong processor.

   If the @tt rev argument is not 0 and doesn't match the internal
   value of the configuration revision number, @tt -EAGAIN is returned.

   @This may return @tt -EBUSY if the timer hardware resource is not
   available. @This returns @tt -ENOTSUP if there is no timer matching the
   requested device number.
*/
typedef DEV_TIMER_GET_VALUE(dev_timer_get_value_t);



/** @see dev_timer_config_t */
#define DEV_TIMER_CONFIG(n)	error_t (n) (struct device_timer_s *accessor, \
                                             struct dev_timer_config_s *cfg, \
                                             dev_timer_res_t res)

/**
   @This reads the current timer configuration and optionally sets the
   resolution. The @ref dev_timer_config_s object is updated with the
   current configuration if the @tt cfg parameter is not @tt NULL.

   The @ref dev_timer_config_s::freq field is set to 0 if the timer
   frequency is not known.

   The resolution is set when the @tt res parameter is not 0. The
   actual resolution can be different from the requested resolution
   depending on hardware design of the timer. The current and possibly
   new resolution is stored in the @tt res field.

   The maximum timer value reached when the timer overlaps is stored
   in the @tt max field. This value is a power of two minus one.

   The timer resolution can not be changed if the timer has been
   started by calling @ref device_start or by queuing a request. @This
   returns @tt -EBUSY in this case.

   If the new resolution value is different from the requested one,
   @tt -ERANGE is returned. This can only occur when trying to change
   the resolution.

   @This returns @tt -ENOTSUP if there is no timer matching the
   requested device number.
*/
typedef DEV_TIMER_CONFIG(dev_timer_config_t);


DRIVER_CLASS_TYPES(DRIVER_CLASS_TIMER, timer,
                   dev_timer_request_t *f_request;
                   dev_timer_request_t *f_cancel;
                   dev_timer_get_value_t *f_get_value;
                   dev_timer_config_t *f_config;
                   );

#define DRIVER_TIMER_METHODS(prefix)                            \
  ((const struct driver_class_s*)&(const struct driver_timer_s){        \
    .class_ = DRIVER_CLASS_TIMER,                               \
    .f_request = prefix ## _request,                            \
    .f_cancel = prefix ## _cancel,                              \
    .f_get_value = prefix ## _get_value,                        \
    .f_config = prefix ## _config,                              \
  })

/** @This initializes a timer delay from the given delay value in
    seconds unit. The delay is specified in seconds when r_unit is 1,
    in msec when r_unit is 1000 and so on. Actual delay in timer units
    is computed from timer frequency and current timer resolution.

    @This returns a negative error code if the timer value can not be
    read (-EIO) or if the timer overlap period is to short for the
    delay (-ERANGE). */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_init_sec(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                           dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit);

/** @see dev_timer_init_sec. */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_init_sec_round(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                                 dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit);

/** @see dev_timer_init_sec. */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_init_sec_ceil(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                                dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit);

/** @This multiplies the given fraction by the @em {timer frequency /
    timer resolution} fraction. The fraction will be simplifyed if the
    @tt reduce parameter is set.

    The resulting fraction can be used for conversion between timer
    units and second based delay.*/
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_frac(struct device_timer_s *accessor,
                       uint64_t *num, uint64_t *denom,
                       dev_timer_cfgrev_t *rev, bool_t reduce);

/** @This works like @ref dev_timer_init_sec but convert from timer
    unit to second based unit. */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_get_sec(struct device_timer_s *accessor, uint64_t *stime,
                          dev_timer_cfgrev_t *rev, dev_timer_value_t tvalue, uint32_t r_unit);

/** @This computes two shift amounts which can be used for fast
    conversion between a delay in second based unit and a delay in
    timer unit. Shifting will result in a delay rounded to the lower
    or higher power of 2.

    Left shifting by the @tt shift_a value when converting from a
    second based delay will yield a timer delay which can be up to 2
    times longer. Left shifting by the @tt shift_b value when
    converting from a second based delay will yield a timer delay
    which will be divided by 2 in the worst case. A negative value
    implies a right shift in this case. The @ref dev_timer_delay_shift_s2t
    function can be used to perform those conversions.

    Right shifting by the @tt shift_b value when converting from a
    timer delay will yield a second based delay which can be up to 2
    times longer. Right shifting by the @tt shift_a value when
    converting from a timer delay will yield a second based delay
    which will be divided by 2 in the worst case. A negative value
    implies a left shift in this case. The @ref dev_timer_delay_shift_t2s
    function can be used to perform those conversions.

    Either the @tt shift_a or the @tt shift_b pointer may be @tt NULL.
*/
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_shift_sec(struct device_timer_s *accessor,
                            int8_t *shift_a, int8_t *shift_b,
                            dev_timer_cfgrev_t *rev,
                            dev_timer_delay_t s_delay, uint32_t r_unit);

/** @This applies the shift amount computed by the @ref
    dev_timer_shift_sec function. The conversion range can be checked
    by calling @ref dev_timer_delay_check_s2t. */
config_depend(CONFIG_DEVICE_TIMER)
ALWAYS_INLINE dev_timer_delay_t
dev_timer_delay_shift_s2t(int_fast8_t shift, dev_timer_delay_t delay)
{
  return shift > 0 ? delay << shift : delay >> -shift;
}

/** @This checks the range of the specified delay value for conversion
    by the @ref dev_timer_delay_shift_s2t function. */
config_depend(CONFIG_DEVICE_TIMER)
ALWAYS_INLINE error_t
dev_timer_delay_check_s2t(int_fast8_t shift, dev_timer_delay_t delay)
{
  return shift > 0 && bit_clz_unsafe(delay) < shift ? -ERANGE : 0;
}

/** @This applies the shift amount computed by the @ref
    dev_timer_shift_sec function. The conversion range can be checked
    by calling @ref dev_timer_delay_check_s2t. */
config_depend(CONFIG_DEVICE_TIMER)
ALWAYS_INLINE dev_timer_delay_t
dev_timer_delay_shift_t2s(int_fast8_t shift, dev_timer_delay_t delay)
{
  return shift > 0 ? delay >> shift : delay << -shift;
}

/** @This checks the range of the specified delay value for conversion
    by the @ref dev_timer_delay_shift_t2s function. */
config_depend(CONFIG_DEVICE_TIMER)
ALWAYS_INLINE error_t
dev_timer_delay_check_t2s(int_fast8_t shift, dev_timer_delay_t delay)
{
  return shift < 0 && bit_clz_unsafe(delay) < -shift ? -ERANGE : 0;
}

ALWAYS_INLINE bool_t dev_timer_request_is_scheduled(const struct dev_timer_rq_s *rq)
{
  return rq->base.drvdata != NULL;
}

/** @This checks if the time specified by @tt delay has elapsed since
    the timer had the value specified in @tt start.

    @This may return a negative error code. If the timer value can not
    be read, @tt -EIO is returned. If the delay is too large to fit in
    the @ref dev_timer_delay_t type, @tt -ERANGE is returned. On
    success, 0 is returned if the time has not elapsed yet and 1 is
    returned on timeout. */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_check_timeout(struct device_timer_s *accessor,
                                dev_timer_delay_t delay,
                                const dev_timer_value_t *start);

/** Synchronous timer wait function. @This uses the scheduler API to
    put the current context in wait state waiting for the given
    request to terminate. */
config_depend_and2_inline(CONFIG_DEVICE_TIMER, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_timer_wait_rq(struct device_timer_s *accessor,
                               struct dev_timer_rq_s *rq),
{
  struct dev_request_status_s st;

  dev_request_sched_init(&rq->base, &st);
  error_t err = DEVICE_OP(accessor, request, rq);

  if (!err) {
    dev_request_sched_wait(&st);
    err = 0;
  }

  return err;
});

/** Synchronous timer wait function. @This uses the scheduler API to
    put the current context in wait state waiting for the specified
    deadline. */
config_depend_and2_inline(CONFIG_DEVICE_TIMER, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_timer_wait_deadline(struct device_timer_s *accessor,
                                dev_timer_value_t deadline,
                                dev_timer_cfgrev_t rev),
{
  struct dev_timer_rq_s rq;
  rq.delay = 0;
  rq.deadline = deadline;
  rq.rev = rev;

  return dev_timer_wait_rq(accessor, &rq);
});

/** Synchronous timer wait function. @This uses the scheduler API to
    put the current context in wait state waiting for the specified
    delay. The function never returns @tt -ETIMEDOUT. */
config_depend_and2_inline(CONFIG_DEVICE_TIMER, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_timer_wait_delay(struct device_timer_s *accessor,
                             dev_timer_delay_t delay,
                             dev_timer_cfgrev_t rev),
{
  struct dev_timer_rq_s rq;
  rq.deadline = 0;
  rq.delay = delay;
  rq.rev = rev;

  error_t err = dev_timer_wait_rq(accessor, &rq);

  if (err == -ETIMEDOUT)
    err = 0;

  return err;
});

#ifdef CONFIG_DEVICE_TIMER
/** @This provides a @ref DEV_RES_DEV_PARAM resource entry which
    specifies a dependency on a timer device. */
# define DEV_STATIC_RES_DEV_TIMER(path_)                                \
  DEV_STATIC_RES_DEVCLASS_PARAM("timer", path_, DRIVER_CLASS_TIMER)
#else
/** @hidden */
# define DEV_STATIC_RES_DEV_TIMER(path_)                                \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

#endif

