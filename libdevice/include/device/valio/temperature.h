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
    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file
   @module{Devices support library}
   @short Value IO interface for temperature sensors
*/

#ifndef LIBDEVICE_VALIO_TEMPERATURE_H_
#define LIBDEVICE_VALIO_TEMPERATURE_H_

enum valio_temp_att_e
{
  /**
     This attribute id used only for @tt DEVICE_VALIO_READ and @tt
     DEVICE_VALIO_WAIT_EVENT request types.  The request is called
     with @tt valio_temperature_s as data.
   */
  VALIO_TEMPERATURE_VALUE = CONFIG_DEVICE_VALIO_TEMPERATURE_ATTRIBUTE_FIRST,
};

/** Structure for @tt VALIO_TEMPERATURE_VALUE request type */
struct valio_temperature_s
{
  /** Temperature in mK unit */
  uint32_t temperature;
};

#endif
