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
   @module{Devices support library}
   @short General purpose IO driver API

   @section {Description}

   GPIO Controller class abstracts explicit access to input/output
   pins.

   This class does not provide any notification framework.  This
   feature is provided by ICU class.  A GPIO device may implement both
   classes.

   GPIOs may be accessed through this class for three basic
   operations:
   @list
   @item Setting mode,
   @item Reading current value,
   @item Setting a value.
   @end list

   All operations can be done atomically on an set of pins, the only
   constraint is that they must all reside in the same pin bank.  Pin
   banking constraints are implementation dependant.

   @end section

   @section {Asynchronous Operations}

   Most GPIO devices are memory-mapped. Most client use cases are
   synchronous.  Nevertheless, some GPIO devices are behind
   asynchronous busses (like I2C, SPI).

   As most driver do not care for asynchronous operations, @tt
   libdevice provides a generic asynchonous wrapper that relies on
   synchronous operations.  This way, all synchronous drivers are able
   to provide asynchonous request handling.

   On the other hand, devices behind asynchonous busses cannot
   implement synchronous operations and can only provide asynchonous
   request handling.

   Client code relying upon GPIO class should consider whether
   supporting only synchonous devices is a strong limitation for code
   portability.

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

   Kroutine is callen upon completion.  You may enqueue a request from
   kroutine code.
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

STRUCT_COMPOSE(dev_gpio_rq_s, base);

/** Helper that implements asynchronous f_request from other
    synchronous primitives.
*/
extern DEV_GPIO_REQUEST(dev_gpio_request_async_to_sync);


DRIVER_CLASS_TYPES(gpio,
                   dev_gpio_set_mode_t *f_set_mode;
                   dev_gpio_set_output_t *f_set_output;
                   dev_gpio_get_input_t *f_get_input;
                   dev_gpio_input_irq_range_t *f_input_irq_range;
                   dev_gpio_request_t *f_request;
  );

#define DRIVER_GPIO_METHODS(prefix)                               \
  &(const struct driver_gpio_s){                                  \
    .class_ = DRIVER_CLASS_GPIO,                                  \
    .f_set_mode = prefix ## _set_mode,                            \
    .f_set_output = prefix ## _set_output,                        \
    .f_get_input = prefix ## _get_input,                          \
    .f_input_irq_range = prefix ## _input_irq_range,              \
    .f_request = prefix ## _request,                              \
  }

/** Synchronous gpio device request function. This function use a
    busy wait loop during the request. @see dev_gpio_wait_rq */
config_depend(CONFIG_DEVICE_GPIO)
inline error_t dev_gpio_spin_rq(struct device_gpio_s *accessor,
                                struct dev_gpio_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_spin_init(&rq->base, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_spin_wait(&st);
  return rq->error;
}

/** Synchronous gpio device request function. This function use the
    scheduler api to put the current context in wait state during the
    request. */
config_depend_and2(CONFIG_DEVICE_GPIO, CONFIG_MUTEK_SCHEDULER)
inline error_t dev_gpio_wait_rq(struct device_gpio_s *accessor,
                                struct dev_gpio_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_sched_init(&rq->base, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_sched_wait(&st);
  return rq->error;
}


/** @This changes the mode of multiple GPIO pins. */
config_depend(CONFIG_DEVICE_GPIO)
error_t device_gpio_map_set_mode(struct device_gpio_s *accessor,
                                 const gpio_id_t *map, const gpio_width_t *wmap,
                                 uint_fast8_t count, /* enum dev_pin_driving_e */ ...);


/** @This adds a GPIO pins binding to the device resources list.

    This entry specifies a pin label name along with a range of
    contiguous pin ids associated to the label. It used to specify how
    a range of pins of the device with its device specific function
    identified by the label, are connected to a GPIO controller. A
    link to the GPIO controller for which the pin id range is relevant
    must be specified in a separate @ref DEV_RES_DEV_PARAM resource
    entry named @tt gpio.

    @see #DEV_STATIC_RES_GPIO
*/
ALWAYS_INLINE error_t device_res_add_gpio(struct device_s *dev, const char *label,
                                          gpio_id_t id, gpio_width_t width)
{
#ifdef CONFIG_DEVICE_GPIO
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_GPIO, NULL, label, &r);
  if (err)
    return err;

  r->u.gpio.id = id;
  r->u.gpio.width = width;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_GPIO
/** @This can be used to include a GPIO resource entry in a static
    device resources table declaration. The label name must be a static
    string. @see device_res_add_gpio @see #DEV_DECLARE_STATIC_RESOURCES */
# define DEV_STATIC_RES_GPIO(label_, id_, width_)                       \
  {                                                                     \
      .type = DEV_RES_GPIO,                                             \
         .u = { .gpio = {                                               \
        .label = (label_),                                              \
        .id = (id_),                                                    \
        .width = (width_),                                              \
      } }                                                               \
  }
#else
# define DEV_STATIC_RES_GPIO(label_, id_, width_)                       \
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


