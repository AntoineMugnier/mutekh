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
   @module{Devices support library}
   @short Timer device driver API

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
   frequency. When the driver contains a clock sink end-point, the
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
  struct dev_request_s          rq;

  /** absolute timer deadline, used when @tt delay is 0 */
  dev_timer_value_t             deadline;
  /** timer delay */
  dev_timer_delay_t             delay;
  /** expected configuration revision, ignored if 0 */
  dev_timer_cfgrev_t            rev;
};

STRUCT_INHERIT(dev_timer_rq_s, dev_request_s, rq);

/* expand the dev_timer_pqueue_insert function usable with generic
   device request priority queue. */
GCT_CONTAINER_KEY_TYPES(dev_request_pqueue, CUSTOM, SCALAR,
                        dev_timer_rq_s_cast(dev_request_pqueue_item)->deadline, dev_timer_pqueue);

GCT_CONTAINER_KEY_FCNS(dev_request_pqueue, ASC, inline, dev_timer_pqueue, dev_timer_pqueue,
                       remove, insert);

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
  /** timer input clock accuracy */
  struct dev_freq_accuracy_s    acc;
  /** timer maximum value */
  dev_timer_value_t             max;
  /** revision id associated with this configuration, increased on
      configuration update. Can not be even. */
  dev_timer_cfgrev_t            rev;
  /** timer resolution */
  dev_timer_res_t               res;
  /** timer device capabilities */
  enum dev_timer_capabilities_e cap:8;
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


/** @see #dev_timer_get_value_t */
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



/** @see #dev_timer_get_freq_t */
#define DEV_TIMER_CONFIG(n)	error_t (n) (struct device_timer_s *accessor, \
                                             struct dev_timer_config_s *cfg, \
                                             dev_timer_res_t res)

/**
   @This reads the current timer configuration and optionally sets the
   resolution. The @ref dev_timer_config_s object is updated with the
   current configuration if the @tt cfg parameter is not @tt NULL.

   The @ref dev_timer_rq_s::freq field is set to 0 if the frequency is
   not known.

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


DRIVER_CLASS_TYPES(timer,
                   dev_timer_request_t *f_request;
                   dev_timer_request_t *f_cancel;
                   dev_timer_get_value_t *f_get_value;
                   dev_timer_config_t *f_config;
                   );

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

error_t dev_timer_get_sec(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                          dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit);

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
  return shift > 0 && __builtin_clz(delay) < shift ? -ERANGE : 0;
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
  return shift < 0 && __builtin_clz(delay) < -shift ? -ERANGE : 0;
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

/** Synchronous timer sleep function. @This uses the scheduler API to
    put the current context in wait state waiting for the specified
    delay.

    The request can be first initialized by calling @ref
    dev_timer_init_sec. It can then be reused multiple times to
    save timer units conversion computation.
*/
config_depend_and2(CONFIG_DEVICE_TIMER, CONFIG_MUTEK_SCHEDULER)
error_t dev_timer_sleep(struct device_timer_s *accessor, struct dev_timer_rq_s *rq);

/** Synchronous timer busy-wait function. @This spins in a loop
    waiting for the requested delay. @This returns @tt -ERANGE if the
    requested delay is greater than half the maximum timer value
    because counter overlap can not be handled properly in this
    case. @see dev_timer_sleep */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_busy_wait(struct device_timer_s *accessor, dev_timer_delay_t delay);


#endif

