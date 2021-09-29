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

    Copyright (c) 2020 Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for a relay
*/

#ifndef LIBDEVICE_VALIO_RELAY_H_
#define LIBDEVICE_VALIO_RELAY_H_

#include <device/class/valio.h>

/* This interface is used to drive one binary relay.
 */

#define VALIO_RELAY CONFIG_DEVICE_VALIO_RELAY_ATTRIBUTE_FIRST

/* structure for @tt DEVICE_VALIO_WRITE and @tt DEVICE_VALIO_READ
   request type */

struct valio_relay_control_s
{
  bool_t closed;
};


#endif
