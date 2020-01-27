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

    Copyright (c) 2019, Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file
   @module{Devices support library}
   @short Value IO interface for pressure sensors
*/

#ifndef LIBDEVICE_VALIO_PRESSURE_H_
#define LIBDEVICE_VALIO_PRESSURE_H_

enum valio_pressure_att_e
{
  /**
     The request data field is a @tt valio_pressure_s structure.
   */
  VALIO_PRESSURE_VALUE = CONFIG_DEVICE_VALIO_PRESSURE_ATTRIBUTE_FIRST,
};

/* Return structure for @tt DEVICE_VALIO_READ and
   DEVICE_VALIO_WAIT_EVENT request types */
struct valio_pressure_s
{
  uint32_t  mpa;   /* Pressure level in 10^-3 pa unit */
};

#endif
