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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file
   @module {Core::Devices support library}
   @short Ethernet PHY driver class

   @section{Purpose} The @em {PHY} class provides an abstraction layer
   to access Ethernet PHYs from MAC drivers. It is up to the PHY
   driver to handle hardware IRQ from phy, or register polling through
   System Management Bus (SMI).

   @end section
 */

#ifndef __DEVICE_PHY_H__
#define __DEVICE_PHY_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>

struct device_s;
struct driver_s;
struct device_phy_s;
struct driver_phy_s;

enum dev_phy_request_type_e
{
  DEV_PHY_GET_CAPS,
  DEV_PHY_SET_CONFIG,
  DEV_PHY_GET_STATE,
};

struct dev_phy_rq_s
{
  struct dev_request_s base;

  enum dev_phy_request_type_e type;

  union {
    struct {

    } capabilities;

    struct {

    } configuration;

    struct {

    } state;
  };
};

DEV_REQUEST_INHERIT(phy); DEV_REQUEST_QUEUE_OPS(phy);

/** @see dev_phy_request_t */
#define DEV_PHY_REQUEST(n) void (n) (                              \
    const struct device_phy_s *accessor,                           \
    struct dev_phy_rq_s *rq)

/** @This enqueues a request.

   The kroutine of the request may be executed from within this
   function. Please read @xref {Nested device request completion}. 

   @xsee {Purpose} @see dev_phy_request_type_e
*/
typedef DEV_PHY_REQUEST(dev_phy_request_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_PHY, phy,
    dev_phy_request_t *f_request;
);

/** @see driver_phy_s */
#define DRIVER_PHY_METHODS(prefix)                                    \
  ((const struct driver_class_s*)&(const struct driver_phy_s){        \
    .class_ = DRIVER_CLASS_PHY,                                       \
    .f_request = prefix ## _request,                                  \
  })

#ifdef CONFIG_DEVICE_PHY
# define DEV_STATIC_RES_DEV_PHY(path_)                          \
  DEV_STATIC_RES_DEVCLASS_PARAM("phy", path_, DRIVER_CLASS_PHY)
#else
# define DEV_STATIC_RES_DEV_PHY(path_)                                  \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

#endif
