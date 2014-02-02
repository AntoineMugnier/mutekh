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

    Copyright (c) 2014 Alexandre Becoulet <alexandre.becoulet@free.fr>
    Copyright (c) 2014 Sebastien Cerdan <sebcerdan@gmail.com>

*/

/**
 * @file
 * @module{Devices support library}
 * @short General purpose IO driver API
 */

#ifndef __DEVICE_GPIO_H__
#define __DEVICE_GPIO_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/kroutine.h>
#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#include <device/driver.h>

struct dev_gpio_request_s;
struct device_gpio_s;
typedef uint_fast8_t gpio_id_t;

enum dev_gpio_mode_e
{
  DEV_GPIO_INPUT,
  DEV_GPIO_OUTPUT,
  DEV_GPIO_TRISTATE,
  DEV_GPIO_PULLUP,
  DEV_GPIO_PULLDOWN,
};

static const uint8_t dev_gpio_mask1[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const uint8_t dev_gpio_mask0[8] = { };

/** @see devgpio_set_mode_t */
#define DEVGPIO_SET_MODE(n) error_t (n)(const struct device_gpio_s *gpio, \
                                        gpio_id_t io_first, gpio_id_t io_last, \
                                        const uint8_t *mask, enum dev_gpio_mode_e mode)
/**
   This function changes the mode of one or several IOs.

   The @tt mask parameter is a bit vector which specifies IOs that
   will have their mode changed. The lsb of the first byte in the
   vector specifies the mode change for the IO at index @tt
   io_first. The size of the @tt mask array must be aligned on a 64
   bits boundary. Padding bits at the end of the mask vectors can have
   any value.

   When a single IO needs to be modified, the @tt io_first and @tt
   io_last parameters must be equal and the predefined vector @ref
   dev_gpio_mask1 can be used.
*/
typedef DEVGPIO_SET_MODE(devgpio_set_mode_t);



/** @see devgpio_set_output_t */
#define DEVGPIO_SET_OUTPUT(n) error_t (n)(const struct device_gpio_s *gpio, \
                                          gpio_id_t io_first, gpio_id_t io_last, \
                                          const uint8_t *set_mask, const uint8_t *clear_mask)

/**
   This function changes the output state of one or several IOs.

   Two bit vectors are used to specify how to change the output values
   of the IOs in the given range:

   @table 3
     @item @tt set_mask @item @tt clear_mask @item operation
     @item        0     @item         0      @item clear
     @item        0     @item         1      @item unchanged
     @item        1     @item         0      @item toggle
     @item        1     @item         1      @item set
   @end table

   The lsb of the first byte in the @tt set_mask and @tt clear_mask
   vectors specify the value change for the IO at index @tt
   io_first. The size of the bit vector arrays must be aligned on a 64
   bits boundary. Padding bits at the end of the mask vectors can have
   any value.

   When a single IO needs to be modified, the @tt io_first and @tt
   io_last parameters must be equal and the predefined vector @ref
   dev_gpio_mask1 can be used.
*/
typedef DEVGPIO_SET_OUTPUT(devgpio_set_output_t);



/** @see devgpio_get_input_t */
#define DEVGPIO_GET_INPUT(n) error_t (n)(const struct device_gpio_s *gpio, \
                                         gpio_id_t io_first, gpio_id_t io_last, \
                                         uint8_t *data)
/**
   This function reads the input value of one or several IOs.

   The lsb of the first byte in the @tt data vector contains the value
   of the IO at index @tt io_first.

   The size of the @tt data array must be aligned on a 64 bits
   boundary. The values of the unused bits at the end of the array are
   undefined.
*/
typedef DEVGPIO_GET_INPUT(devgpio_get_input_t);



/** @see devgpio_watch_t */
#define DEVGPIO_WATCH(n) error_t (n)(const struct device_gpio_s *gpio, \
                                       struct dev_gpio_request_s *rq)
/**
   This function registers an IO watch request.

   The request kroutine is invoked when a change matching the request
   is detected. The @tt io_first, @tt io_last, @tt mask, @tt watch,
   @tt one_shot and @tt kr fields of the request must be set before
   calling this function. If the @tt one_shot is set, a single event
   will be reported.

   It's implementation defined if the same IO pin can be watched by
   multiple requests at the same time. If the driver is not able to
   accept multiple such requests, it must return @tt -EBUSY.

   The @ref kroutine_exec function is called on @tt rq->kr when a
   watched event occurs. Other functions of the driver API can be
   called from the kroutine.

   Depending on the hardware and the watched event types, this operation
   may not be supported by all devices; in this case the function will
   return @tt -ENOTSUP.
*/
typedef DEVGPIO_WATCH(devgpio_watch_t);

/** @see devgpio_cancel_t */
#define DEVGPIO_CANCEL(n) error_t (n)(const struct device_gpio_s *gpio, \
                                      struct dev_gpio_request_s *rq)
/**
   This function cancels a previously registered IO watch request.
 */
typedef DEVGPIO_CANCEL(devgpio_cancel_t);

enum dev_gpio_event_type_e
{
  DEV_GPIO_EVENT_TOGGLE       = 1,
  DEV_GPIO_EVENT_RAISING      = 2,
  DEV_GPIO_EVENT_FALLING      = 4,
};

struct dev_gpio_request_s
{
  union {
    struct kroutine_s         kr;

    /** used by driver to enqueue requests */
    CONTAINER_ENTRY_TYPE(CLIST) queue_entry;
  };

  /** index of the first io to monitor */
  gpio_id_t                   io_first;
  /** index of the last io to monitor */
  gpio_id_t                   io_last;

  /** mask of ios to watch in the given range. May be @tt NULL. */
  const uint8_t               *mask;

  /** callback private data */
  void                        *pv;

  /** type of event to watch, multiple values from @ref
     dev_gpio_event_type_e can be order together. */
  enum dev_gpio_event_type_e  watch:8;

  /** type of event detected, updated by the driver before invocation
      of the callback. */
  enum dev_gpio_event_type_e  occur:8;

  /** specifies if the request must be canceled after the next event. */
  bool_t                      one_shot;

  const struct device_gpio_s  *gdev;
  void                        *drv_pv;
};


DRIVER_CLASS_TYPES(gpio,
                   devgpio_set_mode_t *f_set_mode;
                   devgpio_set_output_t *f_set_output;
                   devgpio_get_input_t *f_get_input;
                   devgpio_watch_t *f_watch;
                   devgpio_cancel_t *f_cancel;
		   );

#endif

