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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

/**
   @file
   @module{Devices support library}
   @short Value IO interface for a touchpad
*/

#ifndef LIBDEVICE_VALIO_TOUCHPAD_H_
#define LIBDEVICE_VALIO_TOUCHPAD_H_

enum valio_touchpad_att {
    /** A struct valio_touchpad_size_s defining size of touch area in
        raw coordinates, constant */
    VALIO_TOUCHPAD_SIZE,
    /** A struct valio_touchpad_state_s with current position, read
        or update */
    VALIO_TOUCHPAD_STATE,
};

struct valio_touchpad_size_s
{
    uint32_t width;
    uint32_t height;
};

struct valio_touchpad_state_s
{
    uint32_t x;
    uint32_t y;
    uint8_t touch;
};

#endif
