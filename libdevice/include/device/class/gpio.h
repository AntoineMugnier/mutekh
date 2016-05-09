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
   @file
   @module {Core::Devices support library}
   @short General purpose IO driver API
   @index {General purpose IO} {Device classes}
   @csee DRIVER_CLASS_GPIO

   @section {Purpose}

   This class provides functions to read and update pins of a GPIO
   controller device.

   This class does not provide any pin change notification
   feature. This feature is provided by the @xref {Interrupt
   controller driver API}{Interrupt controller driver API}. A GPIO
   device driver may implement both classes.

   This class enables performing the following operations on a GPIO device:
   @list
   @item setting the mode of a pin,
   @item reading the current value of a pin,
   @item setting or toggling the value of a pin.
   @end list

   An operation involving multiple pins must be performed atomically
   by the driver, provided that it is supported by the hardware.
   Groups of pins which reside in the same IO bank can usually be
   accessed atomically. These constraints are implementation
   dependent.

   @end section

   @section {Synchronous and asynchronous operation}

   Most GPIO controller devices are memory-mapped. Most client use
   cases are synchronous.  Nevertheless, some GPIO controller devices
   are behind I2C or SPI buses which can not be accessed
   instantaneously; that's why the GPIO driver API provide both, a
   synchronous and an asynchronous request based set of functions.

   As most driver do not care for asynchronous operations, @tt
   libdevice provides a generic asynchonous wrapper that relies on
   synchronous operations provided by the driver. This way, all
   drivers are able to provide asynchonous request handling without
   additional code.

   On the other hand, drivers for GPIO controllers behind buses cannot
   implement synchronous operations and can only implement the
   asynchronous request handling approach.

   Client code relying upon GPIO class should consider whether
   supporting only synchronous capable drivers is a strong limitation
   for code portability. The asynchronous request based API is more
   cumbersome to use directly and may be slower but it will work with
   any driver. Some convenient helpers are provided which rely on the
   scheduler to put the current context in wait state during an
   asynchronous request. This offers the most portable and easy to
   use API when the client code runs in a scheduler context (thread).

   @end section
 */

#ifndef __DEVICE_GPIO_H__
#define __DEVICE_GPIO_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/kroutine.h>
#include <gct_platform.h>
#include <gct/container_clist.h>

#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>
#include <device/irq.h>

struct dev_gpio_rq_s;
struct device_gpio_s;

#if CONFIG_DEVICE_GPIO_MAX_ID < 255
typedef uint8_t gpio_id_t;
#define GPIO_INVALID_ID 255
#else
typedef uint16_t gpio_id_t;
#define GPIO_INVALID_ID 65535
#endif

#if CONFIG_DEVICE_GPIO_MAX_WIDTH < 255
typedef uint8_t gpio_width_t;
#else
typedef uint16_t gpio_width_t;
#endif

extern const uint8_t dev_gpio_mask1[8];
extern const uint8_t dev_gpio_mask0[8];

/** @see dev_gpio_set_mode_t */
#define DEV_GPIO_SET_MODE(n) error_t (n)(const struct device_gpio_s *gpio, \
                                        gpio_id_t io_first, gpio_id_t io_last, \
                                        const uint8_t *mask, enum dev_pin_driving_e mode)
/**
   This function changes the mode of one or several IOs.

   The @tt mask parameter is a bit vector which specifies IOs that
   will have their mode changed. The lsb of the first byte in the
   vector specifies the mode change for the IO at index @tt
   io_first. The size of the @tt mask array must be a multiple of 8
   bytes.  Padding bits at the end of the mask vectors can have any
   value.

   When a single IO needs to be modified, the @tt io_first and @tt
   io_last parameters must be equal and the predefined vector @ref
   dev_gpio_mask1 can be used.
*/
typedef DEV_GPIO_SET_MODE(dev_gpio_set_mode_t);



/** @see dev_gpio_set_output_t */
#define DEV_GPIO_SET_OUTPUT(n) error_t (n)(const struct device_gpio_s *gpio, \
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

   The new value of an output can be computed using this formula:
   @em {new = set_mask ^ (old & ( set_mask ^ clear_mask ))}.

   The lsb of the first byte in the @tt set_mask and @tt clear_mask
   vectors specify the value change for the IO at index @tt
   io_first. The size of the bit vector arrays must be be a multiple
   of 8 bytes. Padding bits at the end of the mask vectors can have
   any value.

   When a single IO needs to be modified, the @tt io_first and @tt
   io_last parameters must be equal and the predefined vector @ref
   dev_gpio_mask1 can be used.
*/
typedef DEV_GPIO_SET_OUTPUT(dev_gpio_set_output_t);



/** @see dev_gpio_get_input_t */
#define DEV_GPIO_GET_INPUT(n) error_t (n)(const struct device_gpio_s *gpio, \
                                         gpio_id_t io_first, gpio_id_t io_last, \
                                         uint8_t *data)
/**
   This function reads the input value of one or several IOs.

   The lsb of the first byte in the @tt data vector contains the value
   of the IO at index @tt io_first.

   The size of the @tt data array must be a multiple of 8 bytes. The
   values of the unused bits at the end of the array are undefined.
*/
typedef DEV_GPIO_GET_INPUT(dev_gpio_get_input_t);



/** @see dev_gpio_input_irq_range_t */
#define DEV_GPIO_INPUT_IRQ_RANGE(n) error_t (n)(const struct device_gpio_s *gpio, \
                                                gpio_id_t io_first, gpio_id_t io_last, \
                                                const uint8_t *mask,    \
                                                enum dev_irq_sense_modes_e mode, \
                                                uint_fast8_t ep_id)
/**
   @this declares a range of pins as common IRQ source.  This must be
   used for a device that also implements the ICU device class.

   This configures endpoint @tt ep_id as an interrupt source for
   changes detected on pins in range @tt io_first to @tt io_last where
   bits in @tt mask are set.

   Relevant @tt ep_id to use for this call is device-specific.

   Pin direction and other characteristics (pullups, etc.) must be set
   through @tt set_mode call.

   Any selected change in @tt mode for any selected pin triggers an
   IRQ on the endpoint.

   When changes are then notified through the IRQ callback, there is
   no way to tell which pin triggered the IRQ.  It is up to client
   code to read the input pin status and take action.
*/
typedef DEV_GPIO_INPUT_IRQ_RANGE(dev_gpio_input_irq_range_t);



/** @see dev_gpio_request_t */
#define DEV_GPIO_REQUEST(n) void (n)(const struct device_gpio_s *gpio, \
                                     struct dev_gpio_rq_s *req)
/**
   This function enqueues a request to the GPIO driver.

   Kroutine is callen upon completion.
*/
typedef DEV_GPIO_REQUEST(dev_gpio_request_t);


enum dev_gpio_request_type
{
  DEV_GPIO_MODE,
  DEV_GPIO_SET_OUTPUT,
  DEV_GPIO_GET_INPUT,
  DEV_GPIO_INPUT_IRQ_RANGE,
};

#define GCT_CONTAINER_ALGO_dev_gpio_queue CLIST

struct dev_gpio_rq_s
{
  struct dev_request_s base;

  error_t error;

  /** index of the first io to act on */
  gpio_id_t                   io_first;
  /** index of the last io to act on */
  gpio_id_t                   io_last;

  enum dev_gpio_request_type type;

  union {
    struct {
      /** mask of ios to set mode for. */
      const uint8_t               *mask;
      enum dev_pin_driving_e      mode;
    } mode;

    struct {
      /** mask to set, @see dev_gpio_set_output_t */
      const uint8_t               *set_mask;
      /** mask to clear, @see dev_gpio_set_output_t */
      const uint8_t               *clear_mask;
    } output;

    struct {
      /** data buffer to read */
      uint8_t                     *data;
    } input;

    struct {
      const uint8_t               *mask;
      enum dev_irq_sense_modes_e  mode;
      uint_fast8_t                ep_id;
    } input_irq_range;
  };
};

STRUCT_INHERIT(dev_gpio_rq_s, dev_request_s, base);

/** Helper that implements asynchronous f_request from other
    synchronous primitives.
*/
extern DEV_GPIO_REQUEST(dev_gpio_request_async_to_sync);


DRIVER_CLASS_TYPES(DRIVER_CLASS_GPIO, gpio,
                   dev_gpio_set_mode_t *f_set_mode;
                   dev_gpio_set_output_t *f_set_output;
                   dev_gpio_get_input_t *f_get_input;
                   dev_gpio_input_irq_range_t *f_input_irq_range;
                   dev_gpio_request_t *f_request;
  );

/** @see driver_gpio_s */
#define DRIVER_GPIO_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_gpio_s){   \
    .class_ = DRIVER_CLASS_GPIO,                                  \
    .f_set_mode = prefix ## _set_mode,                            \
    .f_set_output = prefix ## _set_output,                        \
    .f_get_input = prefix ## _get_input,                          \
    .f_input_irq_range = prefix ## _input_irq_range,              \
    .f_request = prefix ## _request,                              \
  })

/** Blocking GPIO device request function. This function uses a
    busy wait loop during the request. @see dev_gpio_wait_rq */
BUSY_WAITING_FUNCTION
config_depend_inline(CONFIG_DEVICE_GPIO,
error_t dev_gpio_spin_rq(struct device_gpio_s *accessor,
                         struct dev_gpio_rq_s *rq),
{
  struct dev_request_status_s st;
  dev_request_spin_init(&rq->base, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_spin_wait(&st);
  return rq->error;
})

/** Blocking GPIO device request function. This function uses the
    scheduler api to put the current context in wait state during the
    request. @see dev_gpio_request_t
    @xsee {Synchronous and asynchronous operation} */
config_depend_and2_inline(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_gpio_wait_rq(struct device_gpio_s *accessor,
                         struct dev_gpio_rq_s *rq),
{
  struct dev_request_status_s st;
  dev_request_sched_init(&rq->base, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_sched_wait(&st);
  return rq->error;
})

/** @This sets the value of a single gpio pin relying on the
    synchronous driver API. This will not work with all GPIO controllers.
    @see dev_gpio_set_output_t @see dev_gpio_wait_out
    @xsee {Synchronous and asynchronous operation} */
config_depend_alwaysinline(CONFIG_DEVICE_GPIO,
error_t dev_gpio_out(struct device_gpio_s *accessor, gpio_id_t id, bool_t x),
{
  const uint8_t *p = x ? dev_gpio_mask1 : dev_gpio_mask0;
  return DEVICE_OP(accessor, set_output, id, id, p, p);
})

/** @This sets the value of a single gpio pin. This function uses the
    scheduler api to put the current context in wait state during the
    request. @see dev_gpio_set_output_t @see dev_gpio_out
    @xsee {Synchronous and asynchronous operation} */
config_depend_and2_inline(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_gpio_wait_out(struct device_gpio_s *accessor, gpio_id_t id, bool_t x),
{
  struct dev_gpio_rq_s rq;
  struct dev_request_status_s st;
  const uint8_t *p = x ? dev_gpio_mask1 : dev_gpio_mask0;
  rq.io_first = rq.io_last = id;
  rq.type = DEV_GPIO_SET_OUTPUT;
  rq.output.set_mask = rq.output.clear_mask = p;
  dev_request_sched_init(&rq.base, &st);
  DEVICE_OP(accessor, request, &rq);
  dev_request_sched_wait(&st);
  return rq.error;
})

/** @This changes the mode of a single gpio pin relying on the
    synchronous driver API. This will not work with all GPIO controllers.
    @see dev_gpio_set_mode_t @see dev_gpio_wait_mode
    @xsee {Synchronous and asynchronous operation} */
config_depend_alwaysinline(CONFIG_DEVICE_GPIO,
error_t dev_gpio_mode(struct device_gpio_s *accessor, gpio_id_t id,
                      enum dev_pin_driving_e mode),
{
  return DEVICE_OP(accessor, set_mode, id, id, dev_gpio_mask1, mode);
})

/** @This changes the mode of a single gpio pin. This function uses the
    scheduler api to put the current context in wait state during the
    request. @see dev_gpio_set_mode_t @see dev_gpio_mode
    @xsee {Synchronous and asynchronous operation} */
config_depend_and2_inline(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_gpio_wait_mode(struct device_gpio_s *accessor, gpio_id_t id,
                           enum dev_pin_driving_e mode),
{
  struct dev_gpio_rq_s rq;
  struct dev_request_status_s st;
  rq.io_first = rq.io_last = id;
  rq.type = DEV_GPIO_MODE;
  rq.mode.mask = dev_gpio_mask1;
  rq.mode.mode = mode;
  dev_request_sched_init(&rq.base, &st);
  DEVICE_OP(accessor, request, &rq);
  dev_request_sched_wait(&st);
  return rq.error;
})

/** @This reads the value of a single gpio pin relying on the
    synchronous driver API. This will not work with all GPIO controllers.
    @see dev_gpio_get_input_t @see dev_gpio_wait_input
    @xsee {Synchronous and asynchronous operation} */
config_depend_alwaysinline(CONFIG_DEVICE_GPIO,
bool_t dev_gpio_input(struct device_gpio_s *accessor, gpio_id_t id, error_t *err),
{
  uint8_t x[8];
  error_t e = DEVICE_OP(accessor, get_input, id, id, x);
  if (err != NULL)
    *err = e;
  return x[0] & 1;
})

/** @This reads the value of a single gpio pin. This function uses the
    scheduler api to put the current context in wait state during the
    request. @see dev_gpio_get_input_t @see dev_gpio_input
    @xsee {Synchronous and asynchronous operation} */
config_depend_and2_inline(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_CONTEXT_SCHED,
bool_t dev_gpio_wait_input(struct device_gpio_s *accessor, gpio_id_t id, error_t *err),
{
  struct dev_gpio_rq_s rq;
  struct dev_request_status_s st;
  uint8_t x[8];
  rq.io_first = rq.io_last = id;
  rq.type = DEV_GPIO_GET_INPUT;
  rq.input.data = x;
  dev_request_sched_init(&rq.base, &st);
  DEVICE_OP(accessor, request, &rq);
  dev_request_sched_wait(&st);
  if (err != NULL)
    *err = rq.error;
  return x[0] & 1;
})

/** @This changes the mode of multiple GPIO pins using the synchronous API. */
config_depend(CONFIG_DEVICE_GPIO)
error_t device_gpio_map_set_mode(struct device_gpio_s *accessor,
                                 const gpio_id_t *map, const gpio_width_t *wmap,
                                 uint_fast8_t count, /* enum dev_pin_driving_e */ ...);


/** @This adds a GPIO pins binding to the device resources list.
    @csee DEV_RES_GPIO @see #DEV_STATIC_RES_GPIO */
config_depend_and2_alwaysinline(CONFIG_DEVICE_GPIO, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_gpio(struct device_s *dev, const char *label,
                            gpio_id_t id, gpio_width_t width),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_GPIO, label, NULL, &r);
  if (err)
    return err;

  r->u.gpio.id = id;
  r->u.gpio.width = width;

  return 0;
})

#ifdef CONFIG_DEVICE_GPIO
/** @This specifies a GPIO resource entry in a static device resources
    table declaration. @csee DEV_RES_GPIO
    @see #DEV_DECLARE_STATIC @see #DEV_STATIC_RES_DEV_GPIO */
# define DEV_STATIC_RES_GPIO(label_, id_, width_)                       \
  {                                                                     \
      .type = DEV_RES_GPIO,                                             \
         .u = { .gpio = {                                               \
        .label = (label_),                                              \
        .id = (id_),                                                    \
        .width = (width_),                                              \
      } }                                                               \
  }

/** @This provides a @ref DEV_RES_DEV_PARAM resource entry which
    specifies the GPIO controller device relevant for the @cref
    DEV_RES_GPIO entries. */
# define DEV_STATIC_RES_DEV_GPIO(path_)                                 \
  DEV_STATIC_RES_DEVCLASS_PARAM("gpio", path_, DRIVER_CLASS_GPIO)
#else
/** @hidden */
# define DEV_STATIC_RES_GPIO(label_, id_, width_)                       \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }

/** @hidden */
# define DEV_STATIC_RES_DEV_GPIO(path_)                                \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

/**
   This initializes an array of GPIO ids from a list of pin labels. If
   the @tt wmap parameter is not @tt NULL, the associated size of pin
   range is also stored.

   The list is composed by space separated pin label names. All pin
   names present in the list must match an available device resource
   unless the name is suffixed by @tt{?}. A value of -1 is stored in
   the array if the pin is not available in the device tree.

   If a label in the list is suffixed by @em {:width}, the pin width
   declared in the resource must match the @em width number.
*/
config_depend(CONFIG_DEVICE_GPIO)
error_t device_res_gpio_map(struct device_s *dev, const char *pin_list,
                            gpio_id_t *map, gpio_width_t *wmap);


#endif


