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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

/**
   @file
   @module {Core::Devices support library}
   @short Network device driver class
   @index {Network device} {Device classes}
   @csee DRIVER_CLASS_NET

   @section {Description}

   Network devices implement a network layer in libnetwork API terms.

   On request, device is responsible for creating a network layer, and
   returns it to caller.  Caller is then free to stack other layers on
   top of it.

   As network layers are refcounted, caller may close its network
   session by dropping all references it has on layer returned by the
   device.

   When asking for instanciation of a layer, caller must provide
   various parameters, including network layer type.  This gives an
   opportunity to optimize stacks by using the highest-level layer
   possible.

   For instance, in a classical IP over ethernet environment, most
   ethernet device drivers will implement IEEE802.3 MAC layer.  Some
   high-end network adapters do optimizations at IP level, and then
   may also implement IP layer.  A stack creation routine may try to
   use the device driver to implement each layer, going lower in stack
   depth until it gets an implemented layer for the driver.

   For protocols sharing the same medium for various unrelated tasks,
   like Bluetooth Low Energy where Radio is used for Advertising,
   Connection, Scanning, etc., device driver may implement each as a
   separate layer.

   A device should be able to run multiple concurrent layers where it
   makes sense.

   Each layer API (tasks, commands, etc.) is defined in libnetwork.
   Layers returned by network drivers should be drop-in replacement
   for their generic libnetwork counterparts, if any.

   Network layer definition may contain handler specialization.
   Layers returns by device drivers must implement the whose set of
   APIs of a layer definition.

   @end section
*/

#ifndef DEVICE_NET_H
#define DEVICE_NET_H

#include <hexo/types.h>
#include <device/request.h>
#include <device/class/timer.h>

#include <net/addr.h>

struct net_layer_s;
struct net_layer_delegate_vtable_s;
struct net_scheduler_s;
struct device_net_s;

/**
   @this retrieves a layer from the device driver.  Driver is
   responsible for memory allocation for layer structure.  Layer is
   reference counted, ownership is transferred to caller.  Caller may
   drop all references to layer when not needed any more.

   @tt layer is meaningful only if return value is not an error.

   @param accessor Device instance
   @param scheduler Network scheduler to attach layer in
   @param type Network layer type
   @param params Layer creation parameters structure, as defined by layer type
   @param delegate Delegate for network layer side-channel handling
   @param delegate_vtable Delegate vtable, may be extended by layer type declaration
   @param layer Returned layer
   @returns an error level
 */
#define DEV_NET_LAYER_CREATE(n)                                         \
  error_t (n)(struct device_net_s *accessor,                            \
              struct net_scheduler_s *scheduler,                        \
              uint_fast8_t type,                                        \
              const void *params,                                       \
              void *delegate,                                           \
              const struct net_layer_delegate_vtable_s *delegate_vtable, \
              struct net_layer_s **layer)

/** @This defines the prototype of the layer creation function. */
typedef DEV_NET_LAYER_CREATE(device_net_layer_create_t);

struct dev_net_info_s
{
  /**
     A bitfield from @tt{CONFIG_NET_LAYER_TYPE_ENUM} enum allocator
     with a bit set for each layer type implemented by this device.
   */
  uint64_t implemented_layers;
  struct net_addr_s addr;
  uint16_t prefix_size;
  uint16_t mtu;
};

/**
   @this retrieves device information.
 */
#define DEV_NET_GET_INFO(n)                                 \
  error_t (n)(struct device_net_s *accessor,                \
              struct dev_net_info_s *info)

/** @This defines the prototype of the get_info function. */
typedef DEV_NET_GET_INFO(device_net_get_info_t);

/** Driver types. */
DRIVER_CLASS_TYPES(DRIVER_CLASS_NET, net,
                   device_net_layer_create_t *f_layer_create;
                   device_net_get_info_t *f_get_info;
                  );

#define DRIVER_NET_METHODS(prefix)                                      \
  ((const struct driver_class_s*)&(const struct driver_net_s){          \
    .class_ = DRIVER_CLASS_NET,                                         \
    .f_layer_create = prefix ## _layer_create,                          \
    .f_get_info = prefix ## _get_info,                                  \
  })

#endif
