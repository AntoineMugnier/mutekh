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
 * @module{Device drivers}
 * @short Timer device driver API
 */

#ifndef __DEVICE_TIMER_H__
#define __DEVICE_TIMER_H__

#include <hexo/types.h>
#include <hexo/error.h>

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

/** timer device class callback function template */
#define DEVTIMER_CALLBACK(n)    bool_t (n) (struct dev_timer_rq_s *rq)
/** Timer device request callback. This function is called when the
    timer deadline is reached.

    The request is rescheduled if the function returns true. If the
    request @tt delay field was not zero when the request was first
    scheduled, the delay must not be changed and the next callback
    will be called with the same interval. If the delay field was
    zero, the @tt deadline field value can be updated to a new value. */
typedef DEVTIMER_CALLBACK(devtimer_callback_t);

/** Timer request @csee devtimer_request_t */
struct dev_timer_rq_s
{
  dev_timer_value_t             deadline;    //< absolute timer deadline
  dev_timer_delay_t             delay;       //< timer delay
  devtimer_callback_t           *callback;   //< callback function
  void                          *pvdata;     //< pv data for callback
  void                          *drvdata;    //< driver private data
  struct device_timer_s         *tdev;       //< pointer to associated timer device
  CONTAINER_ENTRY_TYPE(CLIST)   queue_entry; //< used by driver to enqueue requests
};

CONTAINER_TYPE(dev_timer_queue, CLIST, struct dev_timer_rq_s, queue_entry);
CONTAINER_FUNC(dev_timer_queue, CLIST, static inline, dev_timer_queue);

CONTAINER_KEY_TYPE(dev_timer_queue, PTR, SCALAR, deadline);
CONTAINER_KEY_FUNC(dev_timer_queue, CLIST, static inline, dev_timer_queue, deadline);

/** Timer device class request() function template. */
#define DEVTIMER_REQUEST(n)	error_t  (n) (struct device_timer_s *tdev, struct dev_timer_rq_s *rq, bool_t cancel)

/**
   Timer device class request function. Enqueue or cancel a timer request.

   @param rq pointer to request.

   When enqueuing a new event, the @tt delay and @tt callback fields
   of the request must be initialized. If @tt delay is zero, the @tt
   deadline field must be initialized with an absolute deadline timer
   value.

   The callback function will be called immediately from within this
   function if the deadline has already been reached. In this case the
   @tt ETIMEDOUT positive value is returned unless the callback asks to
   reschedule the request by returning true.

   If the timer has not been started explicitly by calling the @ref
   devtimer_start_stop_t function, it will run until the request queue
   becomes empty again.

   When the cancel operation is selected, the request is removed from
   the timer event queue. @This function returns @tt -ETIMEDOUT if the
   request was not found (already reached).

   When a request is removed from the driver queue, its @tt drvdata
   field becomes @tt NULL.

   @This is mandatory.
*/
typedef DEVTIMER_REQUEST(devtimer_request_t);


/** Timer device class cancel() function template. */
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

   @This returns @tt -EINVAL if the timer has not be started or
   -ENOTSUP if the timer does not support being stopped.

   @This is mandatory.
*/
typedef DEVTIMER_START_STOP(devtimer_start_stop_t);



/** Timer device class getvalue() function template */
#define DEVTIMER_GET_VALUE(n)	error_t (n) (struct device_timer_s *tdev, dev_timer_value_t *value)

/**
   @This reads the current raw timer value. The timer value is
   increasing with time.

   @This function may returns @tt -EIO if the timer value can not be
   accessed. This can append when trying to access a processor local
   timer from the wrong processor.

   @csee #DEVTIMER_GETVALUE @csee #dev_timer_getvalue

   @This is mandatory.
*/
typedef DEVTIMER_GET_VALUE(devtimer_get_value_t);


/** Timer device resolution @see #DEVTIMER_RES_FIXED_POINT */
typedef uint32_t dev_timer_res_t;

/** Timer device resolution() function template */
#define DEVTIMER_RESOLUTION(n)	error_t (n) (struct device_timer_s *tdev, dev_timer_res_t *res, dev_timer_value_t *max)

/**
   @This can be used to query and change how the timer input clock is
   divided. The timer resolution can not be changed if the timer is
   not in stopped state.

   Depending on timer hardware capabilities, setting the timer
   resolution will either configure an hardware prescaler to divide
   timer input clock or simply update the timer cyclic period. If the
   timer hardware has been designed to handle deadlines addition and
   removal without loss of accuracy or race condition, a tick-less
   approach is used. In this case the timer resolution can generally
   be set as low as 1 for best time granularity.

   @This tries to set timer resolution to value pointed to by @tt res
   if the pointer is not @tt NULL and the value is not zero. The
   actual resolution can be different from the requested resolution
   depending on hardware capabilities. The current and possibly new
   resolution is stored in @tt *res if the pointer is not @tt NULL .

   @This also stores the maximum timer value reached when the timer
   overlaps in @tt *max if the pointer is not @tt NULL. This value
   must be a power of two minus one.

   The @tt -EBUSY value is returned when trying to change the
   resolution while the timer has been started. If setting timer
   resolution is not supported, @tt -ENOTSUP is returned. If the new
   resolution value is different from the requested one, @tt -ERANGE
   is returned. These error conditions can only occur when trying to
   change the resolution.

   @This is mandatory.

   @see #DEVTIMER_RES_FIXED_POINT
*/
typedef DEVTIMER_RESOLUTION(devtimer_resolution_t);


DRIVER_CLASS_TYPES(timer,
                   devtimer_request_t *f_request;
                   devtimer_start_stop_t *f_start_stop;
                   devtimer_get_value_t *f_get_value;
                   devtimer_resolution_t *f_resolution;
                   );

/** @This initializes a request with the given timer delay. The delay
    is specified in seconds when r_unit is 1, in msec when r_unit is
    1000 and so on. Actual delay in timer units is computed from timer
    frequency and current timer resolution.

    @This function uses the frequency resource entry of the device or
    the value of the @ref CONFIG_DEVICE_TIMER_DEFAULT_FREQ macro if
    such entry is available not available. For devices with multiple
    timer instance, multiple frequency resource entries must be available.
*/
config_depend(CONFIG_DEVICE_TIMER)
error_t dev_timer_init_rq_sec(struct device_timer_s *tdev, struct dev_timer_rq_s *rq,
                              dev_timer_delay_t delay, uint32_t r_unit);

/** Synchronous timer sleep function. @This uses the scheduler API to
    put the current context in wait state waiting for the specified
    delay. This function acts as @ref dev_timer_busy_wait if either
    the scheduler is disabled in configuration, irqs are disabled in
    configuration, or the timer do not support enqueuing requests
    (@ref devtimer_request_t operation).

    The request can be first initialized by calling @ref
    dev_timer_init_rq, @ref dev_timer_init_rq_sec or @ref
    dev_timer_init_rq_usec. It can then be reused multiple times to
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

