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
   @short Value IO interface for light sensors
*/

#ifndef LIBDEVICE_VALIO_LIGHT_H_
#define LIBDEVICE_VALIO_LIGHT_H_

enum valio_light_att_e
{
  /**
     The request callback is called with @tt valio_light_read_s as data.
   */
  VALIO_LIGHT_VALUE = CONFIG_DEVICE_VALIO_LIGHT_ATTRIBUTE_FIRST,

};

/* Return structure for @tt DEVICE_VALIO_READ request type */
struct valio_light_read_s
{
  uint32_t  lux;    /* Light level in lux unit */
  uint16_t  mil;    /* Thousandth of lux */
};

struct valio_light_update_s
{
  uint32_t  min;
  uint32_t  max;
  struct valio_light_read_s conv;
};

#endif
