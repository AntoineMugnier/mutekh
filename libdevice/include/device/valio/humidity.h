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

    Copyright (c) Vincent DEFILIPPI <vincentdefilippi@gmail.com> 2015
*/

/**
   @file
   @module{Devices support library}
   @short Value IO interface for humidity sensors
*/

#ifndef LIBDEVICE_VALIO_HUMIDITY_H_
#define LIBDEVICE_VALIO_HUMIDITY_H_

enum valio_humidity_e
{
  /**
     This attribute id used only for @tt DEVICE_VALIO_READ request
     type.  The request callback is called with @tt valio_humidity_s
     as data.
   */
  VALIO_HUMIDITY = CONFIG_DEVICE_VALIO_HUMIDITY_ATTRIBUTE_FIRST,
};

/* Return structure for @tt DEVICE_VALIO_READ request type */
struct valio_humidity_s
{
  uint16_t  mil;        /* 1/1000 units from 0 to 1 */
};

#endif
