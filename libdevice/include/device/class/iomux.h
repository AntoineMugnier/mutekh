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
 * @short IO muxing driver API
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

#define IOMUX_INVALID_DEMUX 255
#define IOMUX_INVALID_ID 65535
#define IOMUX_INVALID_MUX 255

#define DEV_IOMUX_SETUP(n) error_t (n)(const struct device_iomux_s *accessor, \
                                      iomux_io_id_t io_id,              \
                                      enum dev_pin_driving_e dir,   \
                                      iomux_mux_t mux, iomux_config_t config)

/** @This function configures the IO specified by the @tt io_id
    parameter. The meaning of the @tt config parameter is driver specific. */
typedef DEV_IOMUX_SETUP(dev_iomux_setup_t);

DRIVER_CLASS_TYPES(iomux,
                   dev_iomux_setup_t *f_setup;
		   );

#define DRIVER_IOMUX_METHODS(prefix)                               \
  (&(const struct driver_iomux_s){                                 \
    .class_ = DRIVER_CLASS_IOMUX,                                  \
    .f_setup = prefix ## _setup,                                   \
  })

/**
   @This sets the muxing configuration of IOs whose names are listed
   in the @tt io_list string. IO muxing information from device
   resources are used.

   If the device has a @ref DEV_RES_DEV_PARAM resource entry named @tt
   iomux, the target IO mux controller, will be configured by calling
   the @ref dev_iomux_setup_t function for each IO in the list.

   When not @tt NULL, the @tt demux, @tt io_id and @tt config arrays
   are updated with the associated values from the device resources
   for listed IOs.

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
   device_iomux_setup(uart, ">tx <rx >rts? <cts?", NULL, NULL, NULL);
   @end code
*/
config_depend(CONFIG_DEVICE_IOMUX)
error_t device_iomux_setup(struct device_s *dev, const char *io_list,
                           iomux_demux_t *demux, iomux_io_id_t *io_id,
                           iomux_config_t *config);

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

    @see #DEV_STATIC_RES_IOMUX
*/
ALWAYS_INLINE error_t device_res_add_iomux(struct device_s *dev, const char *label,
                                           iomux_demux_t demux, iomux_io_id_t io_id,
                                           iomux_mux_t mux, iomux_config_t config)
{
#ifdef CONFIG_DEVICE_IOMUX
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_IOMUX, NULL, label, &r);
  if (err)
    return err;

  r->u.iomux.demux = demux;
  r->u.iomux.io_id = io_id;
  r->u.iomux.mux = mux;
  r->u.iomux.config = config;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_IOMUX
/** @This can be used to include a IOMUX resource entry in a static
    device resources table declaration. The label name must be a static
    string. @see device_res_add_iomux @see #DEV_DECLARE_STATIC_RESOURCES */
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
#else
# define DEV_STATIC_RES_IOMUX(label_, demux_, io_id_, mux_, config_)    \
  {                                                     \
    .type = DEV_RES_UNUSED,                             \
  }
#endif

#endif

