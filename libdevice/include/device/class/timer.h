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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

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

    The request is rescheduled if the function return true. If the
    request @tt delay field was not zero when the request was first
    scheduled, the delay must not be changed and the next callback
    will be called with the same interval. If the delay field was
    zero, the @tt field deadline value can be updated to a new value. */
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
#define DEVTIMER_REQUEST(n)	error_t  (n) (struct device_timer_s *tdev, struct dev_timer_rq_s *rq)

/**
   Timer device class request function. Enqueue a timer request.

   @param rq pointer to request.

   The resquest @tt delay and @tt callback fields must be initialized.  If
   @tt delay is zero, the @tt deadline field must be initialized with an
   absolute deadline timer value.

   Request callback will be called immediately from within this
   function if the deadline has already been reached.

   @This is optional, if @ref #DEVICE_HAS_OP returns false,
   the timer device can not be used to schedule events.
*/
typedef DEVTIMER_REQUEST(devtimer_request_t);


/** Timer device class cancel() function template. */
#define DEVTIMER_CANCEL(n)	error_t  (n) (struct device_timer_s *tdev, struct dev_timer_rq_s *rq)

/**
   Timer device class cancel function. Cancel a queued timer request.

   @param rq pointer to cancel.

   @return @tt -ENOENT if the request was not found (already reached).

   @This is optional but must be available along with @ref devtimer_request_t.
*/
typedef DEVTIMER_CANCEL(devtimer_cancel_t);



/** Timer device class getvalue() function template */
#define DEVTIMER_GET_VALUE(n)	dev_timer_value_t (n) (struct device_timer_s *tdev)

/**
   @This reads the current raw timer value. The timer value is increasing with time.

   @param value pointer to value storage.

   @csee #DEVTIMER_GETVALUE @csee #dev_timer_getvalue

   @This is mandatory.
*/
typedef DEVTIMER_GET_VALUE(devtimer_get_value_t);


/** Timer device fixed point resolution @see #DEVTIMER_RES_FIXED_POINT */
typedef uint64_t dev_timer_res_t;

/** Compute timer device fixed point resolution value from real value */
#define DEVTIMER_RES_FIXED_POINT(v) ((uint64_t)(v * (1ULL << 32)))

/** Timer device resolution() function template */
#define DEVTIMER_RESOLUTION(n)	void (n) (struct device_timer_s *tdev, dev_timer_res_t *res, dev_timer_value_t *max)

/**
   @This tries to set timer resolution to value pointed to by @tt
   new_res if the pointer is not @tt NULL and the current value is not
   zero.

   @This puts the new timer resolution, which can be different from
   the requested resolution depending on hardware capabilities, in @tt
   *res if the pointer is not NULL.

   @This puts the maximum timer value reached when the timer is to
   overlap in @tt *max if the pointer is not @tt NULL.

   If the new resolution value is different from the requested one,
   @tt -ERANGE is returned.  If setting timer resolution is not
   supported, @tt -ENOTSUP is returned.

   @This is mandatory.

   @see #DEVTIMER_RES_FIXED_POINT
*/
typedef DEVTIMER_RESOLUTION(devtimer_resolution_t);


DRIVER_CLASS_TYPES(timer,
                   devtimer_request_t *f_request;
                   devtimer_cancel_t *f_cancel;
                   devtimer_get_value_t *f_get_value;
                   devtimer_resolution_t *f_resolution;
                   );

#endif

