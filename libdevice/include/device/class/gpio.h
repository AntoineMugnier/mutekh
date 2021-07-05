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

   This class enables performing the following operations on a GPIO device:
   @list
   @item setting the mode of a set of pins,
   @item reading the current value of a set of pins,
   @item setting or toggling the value of a set of pins.
   @item waiting until the value of a set of pins is not as expected.
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



/** @see dev_gpio_request_t */
#define DEV_GPIO_REQUEST(n) void (n)(const struct device_gpio_s *gpio, \
                                     struct dev_gpio_rq_s *rq)
/**
   This function enqueues a request to the GPIO driver.

   Kroutine is callen upon completion.
*/
typedef DEV_GPIO_REQUEST(dev_gpio_request_t);

/** @see dev_gpio_cancel_t */
#define DEV_GPIO_CANCEL(n) error_t (n)(const struct device_gpio_s *gpio, \
                                       struct dev_gpio_rq_s *rq)
/** This function cancels a GPIO request.

    The function returns 0 if the request has been cancelled or @tt
    -EBUSY if the request has already ended or will terminate very
    soon. It may also return @tt -ENOTSUP. The request kroutine is not
    executed when this function returns 0.
*/
typedef DEV_GPIO_CANCEL(dev_gpio_cancel_t);

enum dev_gpio_request_type
{
  /** @This performs the same action as the dev_gpio_set_mode_t
      function. The request is handled in order, on the same queue as
      the @ref DEV_GPIO_SET_OUTPUT and @ref DEV_GPIO_GET_INPUT
      requests. */
  DEV_GPIO_MODE,
  /** @This performs the same action as the dev_gpio_get_input_t
      function. The request is handled in order, on the same queue as
      the @ref DEV_GPIO_MODE and @ref DEV_GPIO_GET_INPUT requests. */
  DEV_GPIO_SET_OUTPUT,
  /** @This performs the same action as the dev_gpio_set_output_t
      function. The request is handled in order, on the same queue as
      the @ref DEV_GPIO_MODE and @ref DEV_GPIO_SET_OUTPUT requests. */
  DEV_GPIO_GET_INPUT,
  /** @This waits until a masked set of input pins have a different
      value from what is specified in the request. The request is not
      handled in order with other requests. Not all implementation are
      required to handle multiple such requests at the same time. The
      request will terminate immediately with the @tt -EBUSY error in
      this case. The request terminates immediately when the set of
      inputs is empty. */
  DEV_GPIO_UNTIL,
};

#define GCT_CONTAINER_ALGO_dev_gpio_queue CLIST

struct dev_gpio_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** index of the first io to act on */
  gpio_id_t                   io_first;
  /** index of the last io to act on */
  gpio_id_t                   io_last;

  enum dev_gpio_request_type type;

  union {
    /** @see DEV_GPIO_MODE @see dev_gpio_set_mode_t */
    struct {
      const uint8_t               *mask;
      enum dev_pin_driving_e      mode;
    } mode;

    /** @see DEV_GPIO_SET_OUTPUT @see dev_gpio_set_output_t */
    struct {
      const uint8_t               *set_mask;
      const uint8_t               *clear_mask;
    } output;

    /** @see DEV_GPIO_GET_INPUT @see dev_gpio_get_input_t */
    struct {
      uint8_t                     *data;
    } input;

    /** @see DEV_GPIO_UNTIL */
    struct {
      /** This specifies which input bits are tested against the
          current @tt data value. */
      const uint8_t               *mask;
      /** The request terminates when any masked input bit is not
          equal to @tt data. */
      const uint8_t               *data;
    } until;
  };
};

DEV_REQUEST_INHERIT(gpio); DEV_REQUEST_QUEUE_OPS(gpio);

/** Helper that implements asynchronous f_request from other
    synchronous primitives.
*/
extern DEV_GPIO_REQUEST(dev_gpio_request_async_to_sync);


DRIVER_CLASS_TYPES(DRIVER_CLASS_GPIO, gpio,
                   dev_gpio_set_mode_t *f_set_mode;
                   dev_gpio_set_output_t *f_set_output;
                   dev_gpio_get_input_t *f_get_input;
                   dev_gpio_request_t *f_request;
                   dev_gpio_cancel_t *f_cancel;
  );

/** @see driver_gpio_s */
#define DRIVER_GPIO_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_gpio_s){   \
    .class_ = DRIVER_CLASS_GPIO,                                  \
    .f_set_mode = prefix ## _set_mode,                            \
    .f_set_output = prefix ## _set_output,                        \
    .f_get_input = prefix ## _get_input,                          \
    .f_request = prefix ## _request,                              \
    .f_cancel = prefix ## _cancel,                                \
  })

DEV_REQUEST_WAIT_FUNC(gpio);

/** @This sets the value of a single gpio pin relying on the
    synchronous driver API. This will not work with all GPIO controllers.
    @see dev_gpio_set_output_t @see dev_gpio_wait_out
    @xsee {Synchronous and asynchronous operation} */
config_depend_alwaysinline(CONFIG_DEVICE_GPIO,
error_t dev_gpio_out(const struct device_gpio_s *accessor, gpio_id_t id, bool_t x),
{
  const uint8_t *p = x ? dev_gpio_mask1 : dev_gpio_mask0;
  return DEVICE_OP(accessor, set_output, id, id, p, p);
})

/** @This sets the value of a single gpio pin. This function uses the
    scheduler api to put the current context in wait state during the
    request. @see dev_gpio_set_output_t @see dev_gpio_out
    @xsee {Synchronous and asynchronous operation} */
config_depend_and2_inline(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_gpio_wait_out(const struct device_gpio_s *accessor, gpio_id_t id, bool_t x),
{
  struct dev_gpio_rq_s rq;
  const uint8_t *p = x ? dev_gpio_mask1 : dev_gpio_mask0;
  rq.io_first = rq.io_last = id;
  rq.type = DEV_GPIO_SET_OUTPUT;
  rq.output.set_mask = rq.output.clear_mask = p;
  return dev_gpio_wait_rq(accessor, &rq);
})

/** @This changes the mode of a single gpio pin relying on the
    synchronous driver API. This will not work with all GPIO controllers.
    @see dev_gpio_set_mode_t @see dev_gpio_wait_mode
    @xsee {Synchronous and asynchronous operation} */
config_depend_alwaysinline(CONFIG_DEVICE_GPIO,
error_t dev_gpio_mode(const struct device_gpio_s *accessor, gpio_id_t id,
                      enum dev_pin_driving_e mode),
{
  return DEVICE_OP(accessor, set_mode, id, id, dev_gpio_mask1, mode);
})

/** @This changes the mode of a single gpio pin. This function uses the
    scheduler api to put the current context in wait state during the
    request. @see dev_gpio_set_mode_t @see dev_gpio_mode
    @xsee {Synchronous and asynchronous operation} */
config_depend_and2_inline(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_gpio_wait_mode(const struct device_gpio_s *accessor, gpio_id_t id,
                           enum dev_pin_driving_e mode),
{
  struct dev_gpio_rq_s rq;
  rq.io_first = rq.io_last = id;
  rq.type = DEV_GPIO_MODE;
  rq.mode.mask = dev_gpio_mask1;
  rq.mode.mode = mode;
  return dev_gpio_wait_rq(accessor, &rq);
})

/** @This reads the value of a single gpio pin relying on the
    synchronous driver API. This will not work with all GPIO controllers.
    @see dev_gpio_get_input_t @see dev_gpio_wait_input
    @xsee {Synchronous and asynchronous operation} */
config_depend_alwaysinline(CONFIG_DEVICE_GPIO,
bool_t dev_gpio_input(const struct device_gpio_s *accessor, gpio_id_t id, error_t *err),
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
bool_t dev_gpio_wait_input(const struct device_gpio_s *accessor, gpio_id_t id, error_t *err),
{
  struct dev_gpio_rq_s rq;
  uint8_t x[8];
  rq.io_first = rq.io_last = id;
  rq.type = DEV_GPIO_GET_INPUT;
  rq.input.data = x;
  dev_gpio_wait_rq(accessor, &rq);
  if (err != NULL)
    *err = rq.error;
  return x[0] & 1;
})

/** @This reads the value of a single gpio pin. This function uses the
    scheduler api to put the current context in wait state during the
    request. @see dev_gpio_get_input_t @see dev_gpio_input
    @xsee {Synchronous and asynchronous operation} */
config_depend_and2_inline(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_gpio_wait_until(const struct device_gpio_s *accessor, gpio_id_t id,
                            bool_t x),
{
  struct dev_gpio_rq_s rq;
  const uint8_t *p = x ? dev_gpio_mask1 : dev_gpio_mask0;
  rq.io_first = rq.io_last = id;
  rq.type = DEV_GPIO_UNTIL;
  rq.until.mask = dev_gpio_mask1;
  rq.until.data = p;
  return dev_gpio_wait_rq(accessor, &rq);
})

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
   @This retrieves index of GPIO pins declared device resources from
   a list of pin labels. It can optionally set the mode of the pins.

   The index of the pins named in the @tt pin_list string are stored
   in the @tt map array. If the @tt wmap parameter is not @tt NULL,
   the associated size of pin range is also stored.

   The list is composed by space separated pin label names. All pin
   names present in the list must match an available device resource
   unless the name is suffixed by @tt{?}. The value @ref
   GPIO_INVALID_ID is stored in the @tt map array if an optional pin
   is not declared.

   If a label in the list is suffixed by @em {:width}, the pin width
   declared in the resource must match the specified width.

   If a label is prefixed by a pin direction symbol, the mode of the
   associated pins is changed accordingly. The @tt accessor parameter
   may be @tt NULL if this feature is not used.

   @section {Examples}
   @code
   gpio_id_t map[3];
   device_res_gpio_map(&gpio, dev, "<irq:1 >rst:1 <bus:8 >sleep?:1", map, NULL);
   @end code

   @code
   gpio_id_t map[2];
   gpio_width_t wmap[2];
   device_res_gpio_map(NULL, dev, "ioa iob?", map, wmap);
   @end code
   @end section

   @see dev_pin_driving_e @see device_gpio_get_setup
*/
config_depend(CONFIG_DEVICE_GPIO)
error_t device_gpio_setup(struct device_gpio_s *accessor,
                          struct device_s *dev, const char *pin_list,
                          gpio_id_t *map, gpio_width_t *wmap);

/** @This setups an accessor to the gpio device declared using @ref
    #DEV_STATIC_RES_DEV_GPIO in the resources then calls the @ref
    device_gpio_setup function.

    When the function succeed, the accessor is left initialized and
    must be released by the caller. */
config_depend(CONFIG_DEVICE_GPIO)
error_t device_gpio_get_setup(struct device_gpio_s *accessor,
                              struct device_s *dev, const char *pin_list,
                              gpio_id_t *map, gpio_width_t *wmap);

__attribute__((deprecated("use the device_gpio_setup* function instead")))
error_t device_res_gpio_map(struct device_s *dev, const char *pin_list,
                            gpio_id_t *map, gpio_width_t *wmap);

__attribute__((deprecated("pass direction symbols to the device_gpio_setup* function instead")))
error_t device_gpio_map_set_mode(const struct device_gpio_s *accessor,
                                 const gpio_id_t *map, const gpio_width_t *wmap,
                                 uint_fast8_t count, /* enum dev_pin_driving_e */ ...);

#endif
