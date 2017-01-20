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
   @short Value IO interface for luminosity sensors
*/

#ifndef LIBDEVICE_VALIO_LUMINOSITY_H_
#define LIBDEVICE_VALIO_LUMINOSITY_H_

enum valio_luminosity_att_e
{
  /**
     The request data field is a @tt valio_luminosity_s structure.
   */
  VALIO_LUMINOSITY_VALUE = CONFIG_DEVICE_VALIO_LUMINOSITY_ATTRIBUTE_FIRST,

  /**
     The request data field is a @tt valio_luminosity_limits_s structure.
   */
  VALIO_LUMINOSITY_LIMITS = CONFIG_DEVICE_VALIO_LUMINOSITY_ATTRIBUTE_FIRST,
};

/* Return structure for @tt DEVICE_VALIO_READ and
   DEVICE_VALIO_WAIT_EVENT request types */
struct valio_luminosity_s
{
  uint32_t  mlux;   /* Luminosity level in 10^-3 lux unit */
};

/* Structure for @tt DEVICE_VALIO_WRITE request types of
   VALIO_LUMINOSITY_LIMITS.  Tells when a device should return for a
   wait event operation. */
struct valio_luminosity_limits_s
{
  uint32_t  if_above;   /* Luminosity level in 10^-3 lux unit */
  uint32_t  if_below;   /* Luminosity level in 10^-3 lux unit */
};

#endif
