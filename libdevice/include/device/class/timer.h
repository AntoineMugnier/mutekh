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
 * @file
 * @module{Devices support library}
 * @short Timer device driver API
 */

#ifndef __DEVICE_TIMER_H__
#define __DEVICE_TIMER_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/kroutine.h>
#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#include <device/driver.h>

struct device_s;
struct driver_s;
struct driver_timer_s;
struct device_timer_s;

/** Timer absolute value type */
typedef uint64_t dev_timer_value_t;
/** Timer relative value type */
typedef uint32_t dev_timer_delay_t;

struct dev_timer_rq_s;

/** Timer request @csee devtimer_request_t */
struct dev_timer_rq_s
{
  struct kroutine_s             kr;
  CONTAINER_ENTRY_TYPE(CLIST)   queue_entry; //< used by driver to enqueue requests

  dev_timer_value_t             deadline;    //< absolute timer deadline
  dev_timer_delay_t             delay;       //< timer delay
  void                          *pvdata;     //< pv data for callback
  void                          *drvdata;    //< driver private data
  struct device_timer_s         *tdev;       //< pointer to associated timer device
};

CONTAINER_TYPE(dev_timer_queue, CLIST, struct dev_timer_rq_s, queue_entry);
CONTAINER_FUNC(dev_timer_queue, CLIST, static inline, dev_timer_queue);

CONTAINER_KEY_TYPE(dev_timer_queue, PTR, SCALAR, deadline);
CONTAINER_KEY_FUNC(dev_timer_queue, CLIST, static inline, dev_timer_queue, deadline);

/** @see devtimer_request_t */
#define DEVTIMER_REQUEST(n)	error_t  (n) (struct device_timer_s *tdev, struct dev_timer_rq_s *rq)

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
   and return @tt ETIMEDOUT.

   If the timer has not been started explicitly by calling the @ref
   devtimer_start_stop_t function, it will run until the request queue
   becomes empty again.

   @This returns 0 on success. @This may return @tt -EBUSY if the
   timer hardware resource is not available. @This may return -ENOTSUP
   if there is no timer matching the requested device number.
*/
typedef DEVTIMER_REQUEST(devtimer_request_t);

/** @see devtimer_cancel_t */
#define DEVTIMER_CANCEL(n)	error_t  (n) (struct device_timer_s *tdev, struct dev_timer_rq_s *rq)

/**
   @This cancel a timeout request event.

   The request is removed from the timer event queue. @This function
   returns @tt -ETIMEDOUT if the request is not in the queue any more
   (deadline already reached).

   @This returns 0 on success.
*/
typedef DEVTIMER_CANCEL(devtimer_cancel_t);


/** @see devtimer_start_stop_t */
#define DEVTIMER_START_STOP(n)	error_t  (n) (struct device_timer_s *tdev, bool_t start)

/**
   Timer device class start/stop function.

   @This starts or stop the timer. The timer will be automatically
   started when the requests queue is not empty but will return to
   stopped state unless it has been explicitly started by calling this
   function.

   If the this function is called more than once with the @tt start
   parameter set, it must be called the same number of times with a
   value of 0 for the timer to actually stop. The timer value becomes
   undefined when the timer is stopped.

   @This returns 0 on success or if the timer doesn't support being stopped.

   @This returns @tt -EINVAL when trying to stop a timer that has not
   been started.

   @This returns @tt -EBUSY if the timer hardware resource is not
   available. @This returns @tt -ENOTSUP if there is no timer matching
   the requested device number.

   @This is mandatory.
*/
typedef DEVTIMER_START_STOP(devtimer_start_stop_t);



/** @see #devtimer_get_value_t */
#define DEVTIMER_GET_VALUE(n)	error_t (n) (struct device_timer_s *tdev, dev_timer_value_t *value)

/**
   @This reads the current raw timer value. The timer value is
   increasing with time.

   @This function may returns @tt -EIO if the timer value can not be
   accessed. This can append when trying to access a processor local
   timer from the wrong processor.

   The timer value may not be readable if the timer is currently
   stopped. In this case this function will return @tt -EBUSY.  @This
   returns @tt -ENOTSUP if there is no timer matching the requested
   device number.

   @This is mandatory.
*/
typedef DEVTIMER_GET_VALUE(devtimer_get_value_t);


/** Timer device resolution type */
typedef uint32_t dev_timer_res_t;

/** @see devtimer_resolution_t */
#define DEVTIMER_RESOLUTION(n)	error_t (n) (struct device_timer_s *tdev, dev_timer_res_t *res, dev_timer_value_t *max)

/**
   @This can be used to query and change how the timer input clock is
   divided. The timer resolution can not be changed if the timer is
   not in stopped state.

   Depending on timer hardware capabilities, setting the timer
   resolution will either configure an hardware prescaler to divide
   timer input clock or simply update the timer cyclic period. If the
   timer hardware has been designed to support request insertion and
   removal without loss of accuracy or race condition, a tick-less
   approach is used. In this case the timer resolution can generally
   be set as low as 1 for best time granularity. In case of a count to
   zero type of timer, setting the resolution to a low value will
   generate many interrupts.

   @This tries to set timer resolution to value pointed to by @tt res
   if the pointer is not @tt NULL and the value is not zero. The
   actual resolution can be different from the requested resolution
   depending on hardware capabilities. The current and possibly new
   resolution is stored in @tt *res if the pointer is not @tt NULL .

   @This also stores the maximum timer value reached when the timer
   overlaps in @tt *max if the pointer is not @tt NULL. This value
   must be a power of two minus one.

   The @tt -EBUSY value is returned when trying to change the
   resolution while the timer has been started. If setting the timer
   resolution is not supported, @tt -ENOTSUP is returned. If the new
   resolution value is different from the requested one, @tt -ERANGE
   is returned. These error conditions can only occur when trying to
   change the resolution.

   @This returns @tt -ENOTSUP if there is no timer matching the
   requested device number.

   @This is mandatory.
*/
typedef DEVTIMER_RESOLUTION(devtimer_resolution_t);


DRIVER_CLASS_TYPES(timer,
                   devtimer_request_t *f_request;
                   devtimer_request_t *f_cancel;
                   devtimer_start_stop_t *f_start_stop;
                   devtimer_get_value_t *f_get_value;
                   devtimer_resolution_t *f_resolution;
                   );

/** @This initializes a timer delay from the given delay value in
    seconds unit. The delay is specified in seconds when r_unit is 1,
    in msec when r_unit is 1000 and so on. Actual delay in timer units
    is computed from timer frequency and current timer resolution.

    @This function uses the frequency resource entry of the device or
    the value of the @ref #CONFIG_DEVICE_TIMER_DEFAULT_FREQ macro if
    no such entry is available. For devices with multiple timer
    instances, multiple frequency resource entries are expected.

    @This returns a negative error code if the timer value can not be
    read (-EIO) or if the timer overlap period is to short for the
    delay (-ERANGE). */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_init_sec(struct device_timer_s *tdev, dev_timer_delay_t *delay,
                           dev_timer_delay_t s_delay, uint32_t r_unit);

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
error_t dev_timer_shift_sec(struct device_timer_s *tdev,
                            int8_t *shift_a, int8_t *shift_b,
                            dev_timer_delay_t s_delay, uint32_t r_unit);

/** @This applies the shift amount computed by the @ref
    dev_timer_shift_sec function. The conversion range can be checked
    by calling @ref dev_timer_delay_check_s2t. */
config_depend(CONFIG_DEVICE_TIMER)
static inline dev_timer_delay_t
dev_timer_delay_shift_s2t(int_fast8_t shift, dev_timer_delay_t delay)
{
  return shift > 0 ? delay << shift : delay >> -shift;
}

/** @This checks the range of the specified delay value for conversion
    by the @ref dev_timer_delay_shift_s2t function. */
config_depend(CONFIG_DEVICE_TIMER)
static inline error_t
dev_timer_delay_check_s2t(int_fast8_t shift, dev_timer_delay_t delay)
{
  return shift > 0 && __builtin_clz(delay) < shift ? -ERANGE : 0;
}

/** @This applies the shift amount computed by the @ref
    dev_timer_shift_sec function. The conversion range can be checked
    by calling @ref dev_timer_delay_check_s2t. */
config_depend(CONFIG_DEVICE_TIMER)
static inline dev_timer_delay_t
dev_timer_delay_shift_t2s(int_fast8_t shift, dev_timer_delay_t delay)
{
  return shift > 0 ? delay >> shift : delay << -shift;
}

/** @This checks the range of the specified delay value for conversion
    by the @ref dev_timer_delay_shift_t2s function. */
config_depend(CONFIG_DEVICE_TIMER)
static inline error_t
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
error_t dev_timer_check_timeout(struct device_timer_s *tdev,
                                dev_timer_delay_t delay,
                                const dev_timer_value_t *start);

/** Synchronous timer sleep function. @This uses the scheduler API to
    put the current context in wait state waiting for the specified
    delay. This function acts as @ref dev_timer_busy_wait if either
    the scheduler is disabled in configuration, irqs are disabled in
    configuration, or the timer do not support enqueuing requests
    (@ref devtimer_request_t operation).

    The request can be first initialized by calling @ref
    dev_timer_init_sec. It can then be reused multiple times to
    save timer units conversion computation.
*/
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_sleep(struct device_timer_s *tdev, struct dev_timer_rq_s *rq);

/** Synchronous timer busy-wait function. @This spins in a loop
    waiting for the requested delay. @This returns @tt -ERANGE if the
    requested delay is greater than half the maximum timer value
    because counter overlap can not be handled properly in this
    case. @see dev_timer_sleep */
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_busy_wait(struct device_timer_s *tdev, struct dev_timer_rq_s *rq);


#endif

