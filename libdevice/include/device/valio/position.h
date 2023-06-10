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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2021
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for a position sensor
*/

#ifndef LIBDEVICE_VALIO_POSITION_H_
#define LIBDEVICE_VALIO_POSITION_H_

#include <hexo/bit.h>
#include <device/class/valio.h>

/** The position valio class is an interface for acquiring an absolute
    or relative position, for instance from a quadrature decoder.
*/

enum valio_position_att_e
{
    /* Use with @tt DEVICE_VALIO_READ or @tt DEVICE_VALIO_WAIT_EVENT
       request type. @tt data is a pointer to a @tt
       uint32_t. */
    VALIO_POSITION_VALUE = CONFIG_DEVICE_VALIO_POSITION_ATTRIBUTE_FIRST,
};

#endif

