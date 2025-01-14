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
   @short IO muxing driver API
   @index {IO muxing} {Device classes}
   @csee DRIVER_CLASS_IOMUX

   @section {Purpose}

   This class provides functions to configure the pin muxing used by a
   device @b internal to a chip.

   This is mainly intended for use by device drivers: the @ref
   device_iomux_setup function is called from the device driver
   initialization function and the @ref device_iomux_cleanup is called
   when the driver is unloaded.

   Using this API to configure IOs used to drive an external device is
   wrong. The @xref{General purpose IO} class must be used for that
   purpose.

   @end section
*/

#ifndef __DEVICE_IOMUX_H__
#define __DEVICE_IOMUX_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <device/driver.h>
#include <device/resources.h>

struct device_iomux_s;

typedef uint8_t  iomux_demux_t;
typedef uint16_t iomux_io_id_t;
typedef uint32_t iomux_mux_t;
typedef uint32_t iomux_config_t;

/** This is a reserved invalid demux value. */
#define IOMUX_INVALID_DEMUX 255
/** This is a reserved invalid io index value. */
#define IOMUX_INVALID_ID 65535
/** This is a reserved invalid mux value. */
#define IOMUX_INVALID_MUX 255

/** @see dev_iomux_setup_t */
#define DEV_IOMUX_SETUP(n) error_t (n)(const struct device_iomux_s *accessor, \
                                      iomux_io_id_t io_id,              \
                                      enum dev_pin_driving_e dir,   \
                                      iomux_mux_t mux, iomux_config_t config)

/** @This configures the IO specified by the @tt io_id
    parameter. The meaning of the @tt config parameter is driver specific.

    When the value of the @tt mux parameter is @tt IOMUX_INVALID_MUX,
    only the direction of the io is updated.

    @This is called by the device_iomux_fetch and @ref
    device_iomux_cleanup2 function when a device driver is loaded and
    unloaded.

    It may also be called by a device driver directly in order to
    dynamically change the direction of a pin. */
typedef DEV_IOMUX_SETUP(dev_iomux_setup_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_IOMUX, iomux,
                   dev_iomux_setup_t *f_setup;
		   );

/** @see driver_iomux_s */
#define DRIVER_IOMUX_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_iomux_s){   \
    .class_ = DRIVER_CLASS_IOMUX,                                  \
    .f_setup = prefix ## _setup,                                   \
  })

/**
   @This fetches the pin muxing information declared device resources
   and sets the muxing configuration of IOs whose names are listed in
   the @tt io_list string.

   The device must have a @ref DEV_RES_DEV_PARAM resource entry named
   @tt iomux which specifies the target IO mux controller. @This
   calls the @ref device_iomux_fetch function.

   @section {Examples}
   @code
   device_iomux_setup(uart, ">tx <rx >rts? <cts?", NULL, NULL, NULL);
   @end code
   @end section

   @see dev_pin_driving_e
*/
config_depend(CONFIG_DEVICE_IOMUX)
error_t device_iomux_setup(struct device_s *dev, const char *io_list,
                           iomux_demux_t *demux, iomux_io_id_t *io_id,
                           iomux_config_t *config);

/**
   @This fetches the pin muxing information and optionally sets the
   muxing configuration of IOs whose names are listed in the @tt
   io_list string. IO muxing information from device resources are
   used.

   When not @tt NULL, the @tt demux, @tt io_id and @tt config arrays
   are updated with the associated values from the device resources
   for listed IOs.

   This also calls the @ref dev_iomux_setup_t function of the @tt
   iomux device driver for each IO in the list unless the parameter is
   @tt NULL.

   The list string must contain space separated label names. All IO
   labels in the list are required to match available device resources
   unless the name is suffixed by @tt{?}. In this case, the values @ref
   #IOMUX_INVALID_DEMUX and @ref #IOMUX_INVALID_ID are stored in the
   corresponding arrays if no matching label is found in the device tree.

   The directions of the IOs are specified using prefix characters
   attached to labels in the @tt io_list parameter. (see @ref
   dev_pin_driving_e symbols). A direction prefix can also be used in
   the device resource label name; this overrides the direction specified
   in the list.

   Example:
   @code
   device_iomux_fetch(uart, &pv->iomux, ">tx <rx >rts? <cts?", NULL, NULL, NULL);
   @end code

   @see device_iomux_setup
*/
config_depend(CONFIG_DEVICE_IOMUX)
error_t device_iomux_fetch(struct device_s *dev,
                           struct device_iomux_s *iomux, const char *io_list,
                           iomux_demux_t *demux, iomux_io_id_t *io_id,
                           iomux_config_t *config);

/**
   @This resets the direction of all IOs found in the resources.

   The device must have a @ref DEV_RES_DEV_PARAM resource entry named
   @tt iomux which specifies the target IO mux controller. @This
   calls the @ref device_iomux_cleanup2 function.
*/
config_depend(CONFIG_DEVICE_IOMUX)
void device_iomux_cleanup(struct device_s *dev);

/**
   @This resets the direction of all IOs found in the resources.

   @see device_iomux_cleanup
*/
config_depend(CONFIG_DEVICE_IOMUX)
void device_iomux_cleanup2(struct device_s *dev, struct device_iomux_s *iomux);

/** @This adds an IO mux entry to the device resources list.

    This entry can be used along with a @ref DEV_RES_DEV_PARAM
    resource entry named @tt iomux if an external IO muxing controller
    is used.

    This entry specifies an IO label name along with some IO
    configuration values:
    @list
      @item A @em demux value which selects between multiple wires
      able to carry the IO signal out of the muxed device.
      @item An @em io_id value relevant to the external muxing device
        which specifies the index of the selected IO.
      @item A @em mux value relevant to the external muxing device
        which specifies the selected function for a given IO.
      @item A @em config value which is driver specific. When no external
        muxing device is in use, the value is reported to the driver of
        the muxed device. In the other case, the value is reported to the
        driver of the IO muxing device and 0 is reported to the driver of
        the muxed device. The default value should be 0 for all drivers.
    @end list

    @csee DEV_RES_IOMUX */
config_depend_and2_alwaysinline(CONFIG_DEVICE_IOMUX, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_iomux(struct device_s *dev, const char *label,
                             iomux_demux_t demux, iomux_io_id_t io_id,
                             iomux_mux_t mux, iomux_config_t config),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_IOMUX, label, NULL, &r);
  if (err)
    return err;

  r->u.iomux.demux = demux;
  r->u.iomux.io_id = io_id;
  r->u.iomux.mux = mux;
  r->u.iomux.config = config;

  return 0;
})

#ifdef CONFIG_DEVICE_IOMUX
/** @This specifies a IOMUX resource entry in a static device
    resources table declaration. @csee DEV_RES_IOMUX
    @see device_res_add_iomux @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_IOMUX(label_, demux_, io_id_, mux_, config_)      \
  {                                                     \
    .type = DEV_RES_IOMUX,                              \
      .u = { .iomux = {                                 \
        .label = (label_),                              \
        .demux = (demux_),                              \
        .io_id = (io_id_),                              \
        .mux = (mux_),                                  \
        .config = (config_),                            \
      } }                                               \
  }

/** @This provides a @ref DEV_RES_DEV_PARAM resource entry which
    specifies the IOMUX device relevant for some @cref
    DEV_RES_IOMUX entries. */
# define DEV_STATIC_RES_DEV_IOMUX(path_) DEV_STATIC_RES_DEVCLASS_PARAM("iomux", path_, DRIVER_CLASS_IOMUX)

#else
/** @hidden */
# define DEV_STATIC_RES_IOMUX(label_, demux_, io_id_, mux_, config_)    \
  {                                                     \
    .type = DEV_RES_UNUSED,                             \
  }

/** @hidden */
# define DEV_STATIC_RES_DEV_IOMUX(path_)                                   \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

#endif

